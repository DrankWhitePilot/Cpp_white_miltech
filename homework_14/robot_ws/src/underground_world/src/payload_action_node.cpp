#include <memory>
#include <string>

#include "rclcpp/rclcpp.hpp"

#include "underground_world/msg/enemy_down.hpp"
#include "underground_world/srv/payload_trigger.hpp"

namespace {

constexpr auto kEnemyDownTopic = "/payload/enemy_down";
constexpr auto kTriggerService = "/payload/trigger";

}  // namespace

class PayloadActionNode final : public rclcpp::Node {
public:
  PayloadActionNode()
    : Node("payload_action")
  {
    enemy_down_pub_ = create_publisher<underground_world::msg::EnemyDown>(kEnemyDownTopic, 10);
    trigger_service_ = create_service<underground_world::srv::PayloadTrigger>(
      kTriggerService,
      [this](const std::shared_ptr<underground_world::srv::PayloadTrigger::Request> request,
             std::shared_ptr<underground_world::srv::PayloadTrigger::Response> response) {
        underground_world::msg::EnemyDown event;
        event.contact_id = request->contact_id;
        event.x = request->x;
        event.y = request->y;
        enemy_down_pub_->publish(event);

        response->accepted = true;
        response->reason = "payload action published";
        RCLCPP_INFO(get_logger(),
                    "triggered contact_id=%d position=(%d,%d)",
                    request->contact_id,
                    request->x,
                    request->y);
      });
  }

private:
  rclcpp::Publisher<underground_world::msg::EnemyDown>::SharedPtr enemy_down_pub_;
  rclcpp::Service<underground_world::srv::PayloadTrigger>::SharedPtr trigger_service_;
};

int main(int argc, char** argv)
{
  rclcpp::init(argc, argv);
  rclcpp::spin(std::make_shared<PayloadActionNode>());
  rclcpp::shutdown();
  return 0;
}
