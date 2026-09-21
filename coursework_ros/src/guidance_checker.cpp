#include <cmath>
#include <fstream>
#include <memory>
#include <string>

#include "geometry_msgs/msg/point_stamped.hpp"
#include "geometry_msgs/msg/twist.hpp"
#include "nav_msgs/msg/odometry.hpp"
#include "rclcpp/rclcpp.hpp"

class GuidanceChecker final : public rclcpp::Node
{
public:
  GuidanceChecker()
  : Node("guidance_checker")
  {
    target_x_ = declare_parameter("target_x", 10.0);
    target_y_ = declare_parameter("target_y", 5.0);
    publish_target_ = declare_parameter("publish_target", true);
    log_file_ = declare_parameter("log_file", "/tmp/guidance_demo.csv");

    telemetry_publisher_ = create_publisher<nav_msgs::msg::Odometry>("telemetry", 10);
    target_publisher_ = create_publisher<geometry_msgs::msg::PointStamped>("target", 10);
    command_subscription_ = create_subscription<geometry_msgs::msg::Twist>(
            "command", 10,
      [this](const geometry_msgs::msg::Twist::SharedPtr message) {
        checkCommand(*message);
            });
    timer_ = create_wall_timer(
            std::chrono::milliseconds(100), [this]() {publishInputs();});

    log_.open(log_file_, std::ios::trunc);
    if (log_) {
      log_ << "sample,target_x,target_y,linear_x,angular_z,valid\n";
    }
    RCLCPP_INFO(get_logger(), "Checker started; log: %s", log_file_.c_str());
  }

private:
  void publishInputs()
  {
    nav_msgs::msg::Odometry telemetry;
    telemetry.header.stamp = now();
    telemetry.header.frame_id = "map";
    telemetry.child_frame_id = "base_link";
    telemetry.pose.pose.position.x = 0.0;
    telemetry.pose.pose.position.y = 0.0;
    telemetry.pose.pose.orientation.w = 1.0;
    telemetry_publisher_->publish(telemetry);

    if (publish_target_) {
      geometry_msgs::msg::PointStamped target;
      target.header = telemetry.header;
      target.point.x = target_x_;
      target.point.y = target_y_;
      target_publisher_->publish(target);
    }

    ++ticks_;
    if (ticks_ > 50 && valid_commands_ == 0) {
      RCLCPP_ERROR(get_logger(), "CHECK FAILED: no valid command in 5 seconds");
      rclcpp::shutdown();
    }
  }

  void checkCommand(const geometry_msgs::msg::Twist & command)
  {
    const double expected_turn = std::atan2(target_y_, target_x_);
    const bool finite = std::isfinite(command.linear.x) &&
      std::isfinite(command.angular.z);
    const bool moves_forward = command.linear.x > 0.0;
    const bool turns_correctly =
      (expected_turn > 0.0 && command.angular.z > 0.0) ||
      (expected_turn < 0.0 && command.angular.z < 0.0) ||
      (std::abs(expected_turn) < 1e-9 && std::abs(command.angular.z) < 1e-9);
    const bool valid = finite && moves_forward && turns_correctly;

    ++samples_;
    if (valid) {
      ++valid_commands_;
    }
    if (log_) {
      log_       << samples_ << ',' << target_x_ << ',' << target_y_ << ','
                 << command.linear.x << ',' << command.angular.z << ','
                 << (valid ? 1 : 0) << '\n';
      log_.flush();
    }

    if (valid_commands_ >= 5) {
      RCLCPP_INFO(get_logger(),
                        "CHECK PASSED: received %d valid guidance commands",
                        valid_commands_);
      rclcpp::shutdown();
    }
  }

  double target_x_{};
  double target_y_{};
  bool publish_target_{};
  std::string log_file_;
  int ticks_{};
  int samples_{};
  int valid_commands_{};
  std::ofstream log_;

  rclcpp::Publisher<nav_msgs::msg::Odometry>::SharedPtr telemetry_publisher_;
  rclcpp::Publisher<geometry_msgs::msg::PointStamped>::SharedPtr target_publisher_;
  rclcpp::Subscription<geometry_msgs::msg::Twist>::SharedPtr command_subscription_;
  rclcpp::TimerBase::SharedPtr timer_;
};

int main(int argc, char ** argv)
{
  rclcpp::init(argc, argv);
  rclcpp::spin(std::make_shared<GuidanceChecker>());
  if (rclcpp::ok()) {
    rclcpp::shutdown();
  }
  return 0;
}
