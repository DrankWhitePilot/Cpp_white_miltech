#include <algorithm>
#include <cmath>
#include <fstream>
#include <memory>
#include <optional>
#include <sstream>
#include <stdexcept>
#include <string>

#include "geometry_msgs/msg/point_stamped.hpp"
#include "geometry_msgs/msg/twist.hpp"
#include "nav_msgs/msg/odometry.hpp"
#include "rclcpp/rclcpp.hpp"

#include "model_math.hpp"
#include "types.hpp"

namespace
{
double yawFromQuaternion(const geometry_msgs::msg::Quaternion & q)
{
  const double siny = 2.0 * (q.w * q.z + q.x * q.y);
  const double cosy = 1.0 - 2.0 * (q.y * q.y + q.z * q.z);
  return std::atan2(siny, cosy);
}

std::optional<Coord> loadTargetFile(const std::string & path)
{
  if (path.empty()) {
    return std::nullopt;
  }

  std::ifstream input(path);
  if (!input) {
    throw std::runtime_error("cannot open target file: " + path);
  }

  std::string line;
  while (std::getline(input, line)) {
    if (line.empty() || line.front() == '#') {
      continue;
    }
    std::replace(line.begin(), line.end(), ';', ',');
    std::istringstream row(line);
    std::string x_text;
    std::string y_text;
    if (std::getline(row, x_text, ',') && std::getline(row, y_text)) {
      try {
        return Coord{std::stod(x_text), std::stod(y_text)};
      } catch (const std::exception &) {
        continue;
      }
    }
  }
  throw std::runtime_error("target file has no valid x,y row: " + path);
}
}

class GuidanceNode final : public rclcpp::Node
{
public:
  GuidanceNode()
  : Node("guidance_node")
  {
    max_linear_speed_ = declare_parameter("max_linear_speed", 5.0);
    max_angular_speed_ = declare_parameter("max_angular_speed", 1.0);
    heading_tolerance_ = declare_parameter("heading_tolerance", 0.05);
    stop_distance_ = declare_parameter("stop_distance", 1.0);
    const std::string target_file = declare_parameter("target_file", "");

    validateParameters();
    if (!target_file.empty()) {
      target_ = loadTargetFile(target_file);
      RCLCPP_INFO(get_logger(), "Loaded target from %s: (%.3f, %.3f)",
                        target_file.c_str(), target_->x, target_->y);
    }

    command_publisher_ = create_publisher<geometry_msgs::msg::Twist>("command", 10);
    target_subscription_ = create_subscription<geometry_msgs::msg::PointStamped>(
            "target", 10,
      [this](const geometry_msgs::msg::PointStamped::SharedPtr message) {
        target_ = Coord{message->point.x, message->point.y};
        RCLCPP_INFO(get_logger(), "Received target: (%.3f, %.3f)",
                            target_->x, target_->y);
            });
    telemetry_subscription_ = create_subscription<nav_msgs::msg::Odometry>(
            "telemetry", 10,
      [this](const nav_msgs::msg::Odometry::SharedPtr message) {
        publishCommand(*message);
            });

    RCLCPP_INFO(get_logger(), "Guidance module ready");
  }

private:
  void validateParameters() const
  {
    if (max_linear_speed_ <= 0.0 || max_angular_speed_ <= 0.0 ||
      heading_tolerance_ < 0.0 || stop_distance_ < 0.0)
    {
      throw std::invalid_argument("guidance parameters must be positive");
    }
  }

  void publishCommand(const nav_msgs::msg::Odometry & telemetry)
  {
    geometry_msgs::msg::Twist command;
    if (!target_) {
      command_publisher_->publish(command);
      RCLCPP_WARN_THROTTLE(get_logger(), *get_clock(), 2000,
                                 "No target: publishing stop command");
      return;
    }

    const Coord position{telemetry.pose.pose.position.x,
      telemetry.pose.pose.position.y};
    const double yaw = yawFromQuaternion(telemetry.pose.pose.orientation);
    const Coord delta = *target_ - position;
    const double distance = model::length(delta);

    if (distance <= stop_distance_) {
      command_publisher_->publish(command);
      RCLCPP_INFO_THROTTLE(get_logger(), *get_clock(), 2000,
                                 "Target reached: publishing stop command");
      return;
    }

    const double desired_heading = model::directionToRadians(position, *target_, yaw);
    const double heading_error = model::calcTurnDeltaRadians(yaw, desired_heading);
    command.angular.z = std::clamp(heading_error, -max_angular_speed_, max_angular_speed_);
    if (std::abs(heading_error) <= heading_tolerance_) {
      command.linear.x = max_linear_speed_;
    } else {
      const double alignment = std::max(0.0, std::cos(heading_error));
      command.linear.x = max_linear_speed_ * alignment;
    }
    command_publisher_->publish(command);
  }

  double max_linear_speed_{};
  double max_angular_speed_{};
  double heading_tolerance_{};
  double stop_distance_{};
  std::optional<Coord> target_;

  rclcpp::Publisher<geometry_msgs::msg::Twist>::SharedPtr command_publisher_;
  rclcpp::Subscription<geometry_msgs::msg::PointStamped>::SharedPtr target_subscription_;
  rclcpp::Subscription<nav_msgs::msg::Odometry>::SharedPtr telemetry_subscription_;
};

int main(int argc, char ** argv)
{
  rclcpp::init(argc, argv);
  try {
    rclcpp::spin(std::make_shared<GuidanceNode>());
  } catch (const std::exception & error) {
    RCLCPP_FATAL(rclcpp::get_logger("guidance_node"), "%s", error.what());
    rclcpp::shutdown();
    return 1;
  }
  rclcpp::shutdown();
  return 0;
}
