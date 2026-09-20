#include <array>
#include <chrono>
#include <cstdint>
#include <deque>
#include <map>
#include <memory>
#include <optional>
#include <set>
#include <string>
#include <utility>

#include "rclcpp/rclcpp.hpp"

#include "underground_world/msg/local_scan.hpp"
#include "underground_world/msg/move_command.hpp"
#include "underground_world/msg/robot_result.hpp"
#include "underground_world/msg/student_status.hpp"
#include "underground_world/srv/payload_trigger.hpp"

namespace {

using LocalScan = underground_world::msg::LocalScan;
using MoveCommand = underground_world::msg::MoveCommand;
using RobotResult = underground_world::msg::RobotResult;
using StudentStatus = underground_world::msg::StudentStatus;
using PayloadTrigger = underground_world::srv::PayloadTrigger;

constexpr auto kScanTopic = "/robot/local_scan";
constexpr auto kMoveTopic = "/robot/cmd_move";
constexpr auto kResultTopic = "/robot/result";
constexpr auto kStatusTopic = "/student/status";
constexpr auto kTriggerService = "/payload/trigger";

struct Position {
  int x = 0;
  int y = 0;
};

bool operator==(const Position lhs, const Position rhs)
{
  return lhs.x == rhs.x && lhs.y == rhs.y;
}

bool operator<(const Position lhs, const Position rhs)
{
  return lhs.y < rhs.y || (lhs.y == rhs.y && lhs.x < rhs.x);
}

struct Direction {
  std::uint8_t command;
  Position delta;
};

constexpr std::array<Direction, 4> kDirections{{
  {MoveCommand::RIGHT, {1, 0}},
  {MoveCommand::DOWN, {0, 1}},
  {MoveCommand::LEFT, {-1, 0}},
  {MoveCommand::UP, {0, -1}},
}};

Position moved(const Position position, const Position delta)
{
  return Position{position.x + delta.x, position.y + delta.y};
}

bool is_traversable(const std::string& cell_type)
{
  return cell_type == "." || cell_type == "S" || cell_type == "x";
}

}  // namespace

class MissionExplorerNode final : public rclcpp::Node {
public:
  MissionExplorerNode()
    : Node("mission_explorer")
  {
    const auto qos = rclcpp::QoS{10};
    move_pub_ = create_publisher<MoveCommand>(kMoveTopic, qos);
    status_pub_ = create_publisher<StudentStatus>(kStatusTopic, qos);
    trigger_client_ = create_client<PayloadTrigger>(kTriggerService);

    scan_sub_ = create_subscription<LocalScan>(
      kScanTopic, qos, [this](const LocalScan::SharedPtr msg) { on_scan(*msg); });
    result_sub_ = create_subscription<RobotResult>(
      kResultTopic, qos, [this](const RobotResult::SharedPtr msg) { on_result(*msg); });

    drive_timer_ = create_wall_timer(std::chrono::milliseconds{100}, [this]() { drive(); });
    publish_status(StudentStatus::EXPLORING);
  }

private:
  void on_scan(const LocalScan& scan)
  {
    have_scan_ = true;
    robot_ = Position{scan.robot_x, scan.robot_y};
    visited_.insert(robot_);

    for (const auto& cell : scan.cells) {
      const Position position{cell.x, cell.y};
      cells_[position] = cell.cell_type;
      if (cell.cell_type == "C" && cell.contact_id > 0) {
        contacts_[cell.contact_id] = position;
      }
      if (cell.cell_type == "x" && cell.contact_id > 0) {
        processed_contacts_.insert(cell.contact_id);
      }
    }

    if (waiting_for_move_ && robot_ == expected_position_) {
      waiting_for_move_ = false;
    }

    if (trigger_contact_.has_value() && processed_contacts_.contains(*trigger_contact_)) {
      trigger_contact_.reset();
    }

    drive();
  }

  void on_result(const RobotResult& result)
  {
    if (result.mission_result == "SUCCESS") {
      terminal_ = true;
      publish_status(StudentStatus::DONE);
      RCLCPP_INFO(get_logger(), "scenario=%s result=SUCCESS steps=%u", result.scenario_name.c_str(), result.steps_taken);
    }
    else if (result.mission_result != "RUNNING") {
      terminal_ = true;
      publish_status(StudentStatus::FAILED);
      RCLCPP_ERROR(get_logger(),
                   "scenario=%s result=%s reason=%s",
                   result.scenario_name.c_str(),
                   result.mission_result.c_str(),
                   result.reason.c_str());
    }
  }

  void drive()
  {
    if (!have_scan_ || terminal_ || waiting_for_move_) {
      return;
    }

    if (trigger_contact_.has_value()) {
      publish_status(StudentStatus::ENGAGING);
      return;
    }

    for (const auto& [contact_id, position] : contacts_) {
      if (!processed_contacts_.contains(contact_id) && cells_[position] == "C") {
        trigger_contact(contact_id, position);
        return;
      }
    }

    const auto next = next_step_to_unvisited();
    if (!next.has_value()) {
      publish_status(StudentStatus::DONE);
      return;
    }

    publish_status(StudentStatus::EXPLORING);
    MoveCommand command;
    command.direction = next->first;
    expected_position_ = next->second;
    waiting_for_move_ = true;
    move_pub_->publish(command);
  }

  void trigger_contact(const int contact_id, const Position position)
  {
    publish_status(StudentStatus::ENGAGING);
    if (!trigger_client_->service_is_ready()) {
      return;
    }

    trigger_contact_ = contact_id;
    auto request = std::make_shared<PayloadTrigger::Request>();
    request->contact_id = contact_id;
    request->x = position.x;
    request->y = position.y;

    trigger_client_->async_send_request(
      request, [this, contact_id](rclcpp::Client<PayloadTrigger>::SharedFuture future) {
        const auto response = future.get();
        if (!response->accepted && trigger_contact_ == contact_id) {
          terminal_ = true;
          publish_status(StudentStatus::FAILED);
          RCLCPP_ERROR(get_logger(), "payload rejected contact_id=%d reason=%s", contact_id, response->reason.c_str());
        }
      });
  }

  std::optional<std::pair<std::uint8_t, Position>> next_step_to_unvisited() const
  {
    std::deque<Position> queue;
    std::set<Position> reached;
    std::map<Position, std::uint8_t> first_command;

    queue.push_back(robot_);
    reached.insert(robot_);

    while (!queue.empty()) {
      const auto current = queue.front();
      queue.pop_front();

      if (!(current == robot_) && !visited_.contains(current)) {
        return std::pair<std::uint8_t, Position>{first_command.at(current),
                                                 moved(robot_, delta_for_command(first_command.at(current)))};
      }

      for (const auto& direction : kDirections) {
        const auto neighbor = moved(current, direction.delta);
        const auto cell = cells_.find(neighbor);
        if (cell == cells_.end() || !is_traversable(cell->second) || reached.contains(neighbor)) {
          continue;
        }

        reached.insert(neighbor);
        first_command[neighbor] = current == robot_ ? direction.command : first_command.at(current);
        queue.push_back(neighbor);
      }
    }

    return std::nullopt;
  }

  static Position delta_for_command(const std::uint8_t command)
  {
    for (const auto& direction : kDirections) {
      if (direction.command == command) {
        return direction.delta;
      }
    }
    return Position{};
  }

  void publish_status(const std::uint8_t state)
  {
    StudentStatus status;
    status.state = state;
    status_pub_->publish(status);
  }

  bool have_scan_ = false;
  bool terminal_ = false;
  bool waiting_for_move_ = false;
  Position robot_;
  Position expected_position_;
  std::map<Position, std::string> cells_;
  std::map<int, Position> contacts_;
  std::set<int> processed_contacts_;
  std::set<Position> visited_;
  std::optional<int> trigger_contact_;

  rclcpp::Publisher<MoveCommand>::SharedPtr move_pub_;
  rclcpp::Publisher<StudentStatus>::SharedPtr status_pub_;
  rclcpp::Subscription<LocalScan>::SharedPtr scan_sub_;
  rclcpp::Subscription<RobotResult>::SharedPtr result_sub_;
  rclcpp::Client<PayloadTrigger>::SharedPtr trigger_client_;
  rclcpp::TimerBase::SharedPtr drive_timer_;
};

int main(int argc, char** argv)
{
  rclcpp::init(argc, argv);
  rclcpp::spin(std::make_shared<MissionExplorerNode>());
  rclcpp::shutdown();
  return 0;
}
