#include <cstdint>
#include <memory>

#include <rclcpp/rclcpp.hpp>

#include "antidrone_turret/msg/actuator_status.hpp"
#include "antidrone_turret/msg/gimbal_command.hpp"
#include "antidrone_turret/msg/servo_command.hpp"
#include "antidrone_turret/msg/target.hpp"
#include "antidrone_turret/msg/turret_status.hpp"
#include "antidrone_turret/srv/trigger_actuator.hpp"
#include "antidrone_turret/turret_logic.hpp"

namespace antidrone_turret {

class TurretControllerNode : public rclcpp::Node {
public:
  TurretControllerNode()
  : Node("turret_controller_node")
  {
    confidence_threshold_ =
      declare_parameter<double>("confidence_threshold", 0.80);
    max_distance_m_ =
      declare_parameter<double>("max_distance_m", 30.0);

    servo_publisher_ =
      create_publisher<msg::ServoCommand>("/servo/cmd", 10);

    gimbal_publisher_ =
      create_publisher<msg::GimbalCommand>("/gimbal/cmd", 10);

    status_publisher_ =
      create_publisher<msg::TurretStatus>("/turret/status", 10);

    trigger_client_ =
      create_client<srv::TriggerActuator>("/actuator/trigger");

    target_subscription_ =
      create_subscription<msg::Target>(
      "/perception/target",
      10,
      [this](const msg::Target::SharedPtr message) {
        process_target(*message);
      });

    actuator_status_subscription_ =
      create_subscription<msg::ActuatorStatus>(
      "/actuator/status",
      10,
      [this](const msg::ActuatorStatus::SharedPtr message) {
        actuator_ready_ =
          message->state == msg::ActuatorStatus::READY;

        trigger_request_pending_ = false;
      });
  }

private:
  void process_target(const msg::Target& target_message)
  {
    const auto target = TargetInput{
      target_message.visible,
      target_message.x,
      target_message.y,
      target_message.distance_m,
      target_message.confidence,
    };

    const bool trigger_available =
      actuator_ready_ && !trigger_request_pending_;

    const auto status = TurretLogic::make_status(
      target,
      static_cast<float>(confidence_threshold_),
      static_cast<float>(max_distance_m_),
      trigger_available);

    if (status.action == TurretAction::kTrack) {
      publish_servo_command(target.x);
      publish_gimbal_command(target.y);
    }

    publish_status(status);

    if (status.trigger_state == TriggerState::kRequested) {
      request_trigger(target);
    }
  }

  void publish_servo_command(float target_x)
  {
    const auto command_data =
      TurretLogic::make_servo_command(target_x);

    auto command = msg::ServoCommand{};
    command.direction =
      static_cast<std::int8_t>(command_data.direction);
    command.target_x = command_data.target_x;
    command.error_x = command_data.error_x;

    servo_publisher_->publish(command);
  }

  void publish_gimbal_command(float target_y)
  {
    const auto command_data =
      TurretLogic::make_gimbal_command(target_y);

    auto command = msg::GimbalCommand{};
    command.direction =
      static_cast<std::int8_t>(command_data.direction);
    command.target_y = command_data.target_y;
    command.error_y = command_data.error_y;

    gimbal_publisher_->publish(command);
  }

  void publish_status(const TurretStatusData& status_data)
  {
    auto status = msg::TurretStatus{};
    status.target_state =
      static_cast<std::uint8_t>(status_data.target_state);
    status.action =
      static_cast<std::uint8_t>(status_data.action);
    status.trigger_state =
      static_cast<std::uint8_t>(status_data.trigger_state);
    status.confidence = status_data.confidence;
    status.distance_m = status_data.distance_m;

    status_publisher_->publish(status);
  }

  void request_trigger(const TargetInput& target)
  {
    if (!trigger_client_->service_is_ready()) {
      RCLCPP_WARN(
        get_logger(),
        "Service /actuator/trigger is not ready");
      return;
    }

    trigger_request_pending_ = true;

    auto request =
      std::make_shared<srv::TriggerActuator::Request>();

    request->confidence = target.confidence;
    request->distance_m = target.distance_m;

    trigger_client_->async_send_request(
      request,
      [this](
        rclcpp::Client<srv::TriggerActuator>::SharedFuture future) {
        const auto response = future.get();

        if (!response->accepted) {
          trigger_request_pending_ = false;
        }
      });
  }

  double confidence_threshold_{0.80};
  double max_distance_m_{30.0};

  bool actuator_ready_{true};
  bool trigger_request_pending_{false};

  rclcpp::Publisher<msg::ServoCommand>::SharedPtr
    servo_publisher_;

  rclcpp::Publisher<msg::GimbalCommand>::SharedPtr
    gimbal_publisher_;

  rclcpp::Publisher<msg::TurretStatus>::SharedPtr
    status_publisher_;

  rclcpp::Subscription<msg::Target>::SharedPtr
    target_subscription_;

  rclcpp::Subscription<msg::ActuatorStatus>::SharedPtr
    actuator_status_subscription_;

  rclcpp::Client<srv::TriggerActuator>::SharedPtr
    trigger_client_;
};

}  // namespace antidrone_turret

int main(int argc, char** argv)
{
  rclcpp::init(argc, argv);
  rclcpp::spin(
    std::make_shared<antidrone_turret::TurretControllerNode>());
  rclcpp::shutdown();
  return 0;
}
