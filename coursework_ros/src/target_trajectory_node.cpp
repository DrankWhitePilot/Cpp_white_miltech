#include <chrono>
#include <fstream>
#include <memory>
#include <stdexcept>
#include <string>
#include <vector>

#include "geometry_msgs/msg/point_stamped.hpp"
#include "rclcpp/rclcpp.hpp"

#include "json.hpp"

namespace
{
struct Point2d
{
  double x;
  double y;
};

std::vector<std::vector<Point2d>> loadTrajectories(const std::string & path)
{
  std::ifstream input(path);
  if (!input) {
    throw std::runtime_error("cannot open trajectories file: " + path);
  }

  nlohmann::json document;
  input >> document;
  const int declared_count = document.at("targetCount").get<int>();
  const int declared_steps = document.at("timeSteps").get<int>();
  const auto & targets = document.at("targets");
  if (!targets.is_array() || static_cast<int>(targets.size()) != declared_count) {
    throw std::runtime_error("targetCount does not match targets array");
  }

  std::vector<std::vector<Point2d>> result;
  result.reserve(targets.size());
  for (const auto & target : targets) {
    const auto & positions = target.at("positions");
    if (!positions.is_array() || static_cast<int>(positions.size()) != declared_steps) {
      throw std::runtime_error("timeSteps does not match target positions");
    }
    std::vector<Point2d> trajectory;
    trajectory.reserve(positions.size());
    for (const auto & position : positions) {
      trajectory.push_back({position.at("x").get<double>(), position.at("y").get<double>()});
    }
    result.push_back(std::move(trajectory));
  }
  return result;
}
}  // namespace

class TargetTrajectoryNode final : public rclcpp::Node
{
public:
  TargetTrajectoryNode()
  : Node("target_trajectory_node")
  {
    const std::string targets_file = declare_parameter<std::string>("targets_file");
    target_index_ = declare_parameter("target_index", 0);
    array_time_step_ = declare_parameter("array_time_step", 10.0);
    time_scale_ = declare_parameter("time_scale", 1000.0);

    trajectories_ = loadTrajectories(targets_file);
    if (target_index_ < 0 || target_index_ >= static_cast<int>(trajectories_.size())) {
      throw std::invalid_argument("target_index is outside available trajectories");
    }
    if (array_time_step_ <= 0.0 || time_scale_ <= 0.0) {
      throw std::invalid_argument("array_time_step and time_scale must be positive");
    }

    target_publisher_ = create_publisher<geometry_msgs::msg::PointStamped>("target", 10);
    const std::chrono::duration<double> period(array_time_step_ / time_scale_);
    timer_ = create_wall_timer(
      std::chrono::duration_cast<std::chrono::nanoseconds>(period),
      [this]() {publishNextPoint();});

    const auto & selected = trajectories_.at(target_index_);
    RCLCPP_INFO(
      get_logger(),
      "Loaded teacher trajectory %d: %zu samples, array step %.3f s, time scale %.3f",
      target_index_, selected.size(), array_time_step_, time_scale_);
    publishNextPoint();
  }

private:
  void publishNextPoint()
  {
    const auto & trajectory = trajectories_.at(target_index_);
    const std::size_t wrapped_index = sample_index_ % trajectory.size();
    const Point2d & point = trajectory.at(wrapped_index);

    geometry_msgs::msg::PointStamped message;
    message.header.stamp = now();
    message.header.frame_id = "teacher_sim";
    message.point.x = point.x;
    message.point.y = point.y;
    message.point.z = 0.0;
    target_publisher_->publish(message);

    RCLCPP_INFO_THROTTLE(
      get_logger(), *get_clock(), 1000,
      "Target %d sample %zu/%zu: (%.3f, %.3f)",
      target_index_, wrapped_index, trajectory.size(), point.x, point.y);
    ++sample_index_;
  }

  int target_index_{};
  double array_time_step_{};
  double time_scale_{};
  std::size_t sample_index_{};
  std::vector<std::vector<Point2d>> trajectories_;
  rclcpp::Publisher<geometry_msgs::msg::PointStamped>::SharedPtr target_publisher_;
  rclcpp::TimerBase::SharedPtr timer_;
};

int main(int argc, char ** argv)
{
  rclcpp::init(argc, argv);
  try {
    rclcpp::spin(std::make_shared<TargetTrajectoryNode>());
  } catch (const std::exception & error) {
    RCLCPP_FATAL(rclcpp::get_logger("target_trajectory_node"), "%s", error.what());
    rclcpp::shutdown();
    return 1;
  }
  rclcpp::shutdown();
  return 0;
}
