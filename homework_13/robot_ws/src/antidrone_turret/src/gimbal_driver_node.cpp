#include <memory>
#include <string>

#include <rclcpp/rclcpp.hpp>

#include "antidrone_turret/msg/gimbal_command.hpp"

namespace antidrone_turret {

class GimbalDriverNode : public rclcpp::Node {
public:
  GimbalDriverNode()
  : Node("gimbal_driver_node")
  {
    subscription_ =
      create_subscription<msg::GimbalCommand>(
      "/gimbal/cmd",
      10,
      [this](const msg::GimbalCommand::SharedPtr message) {
        RCLCPP_INFO(
          get_logger(),
          "gimbal_driver_node received: direction=%s target_y=%.2f error_y=%.2f",
          direction_name(message->direction).c_str(),
          message->target_y,
          message->error_y);
      });
  }

private:
  [[nodiscard]] static std::string direction_name(std::int8_t direction)
  {
    if (direction == msg::GimbalCommand::UP) {
      return "UP";
    }

    if (direction == msg::GimbalCommand::DOWN) {
      return "DOWN";
    }

    return "CENTER";
  }

  rclcpp::Subscription<msg::GimbalCommand>::SharedPtr subscription_;
};

}  // namespace antidrone_turret

int main(int argc, char** argv)
{
  rclcpp::init(argc, argv);
  rclcpp::spin(
    std::make_shared<antidrone_turret::GimbalDriverNode>());
  rclcpp::shutdown();
  return 0;
}
