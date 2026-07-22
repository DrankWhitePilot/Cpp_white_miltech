#include <memory>
#include <string>

#include <rclcpp/rclcpp.hpp>

#include "antidrone_turret/msg/servo_command.hpp"

namespace antidrone_turret {

class YawServoDriverNode : public rclcpp::Node {
public:
  YawServoDriverNode()
  : Node("yaw_servo_driver_node")
  {
    subscription_ =
      create_subscription<msg::ServoCommand>(
      "/servo/cmd",
      10,
      [this](const msg::ServoCommand::SharedPtr message) {
        RCLCPP_INFO(
          get_logger(),
          "yaw_servo_driver_node received: direction=%s target_x=%.2f error_x=%.2f",
          direction_name(message->direction).c_str(),
          message->target_x,
          message->error_x);
      });
  }

private:
  [[nodiscard]] static std::string direction_name(std::int8_t direction)
  {
    if (direction == msg::ServoCommand::RIGHT) {
      return "RIGHT";
    }

    if (direction == msg::ServoCommand::LEFT) {
      return "LEFT";
    }

    return "CENTER";
  }

  rclcpp::Subscription<msg::ServoCommand>::SharedPtr subscription_;
};

}  // namespace antidrone_turret

int main(int argc, char** argv)
{
  rclcpp::init(argc, argv);
  rclcpp::spin(
    std::make_shared<antidrone_turret::YawServoDriverNode>());
  rclcpp::shutdown();
  return 0;
}
