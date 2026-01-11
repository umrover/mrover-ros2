#include "can_device.hpp"
#include "messaging.hpp"

#include <mrover/msg/led.hpp>

#include <memory>
#include <mrover/msg/pdlb.hpp>
#include <rclcpp/rclcpp.hpp>
#include <std_srvs/srv/set_bool.hpp>
#include <unordered_map>

#include "mrover/msg/detail/dynamixel_set_position__struct.hpp"
#include "mrover/srv/detail/dynamixel_get_position__struct.hpp"
#include "servo_library/servo.hpp"

namespace mrover {

    class PDLBBridge final : public rclcpp::Node {
    public:
        PDLBBridge() : rclcpp::Node("pdlb_hw_bridge") {
            Servo::init("/dev/Ubs");
            setPositionSubscriber = create_subscription<mrover::msg::DynamixelSetPosition>("set_position", 10, [this](mrover::msg::DynamixelSetPosition::ConstSharedPtr const& msg) {
                servo->setPosition(msg->position, Servo::ServoMode::Optimal);
            });

            getPositionService = this->create_service<mrover::srv::DynamixelGetPosition>("get_position", [this](
                                                                                                             mrover::srv::DynamixelGetPosition::Request::SharedPtr const& request,
                                                                                                              mrover::srv::DynamixelGetPosition::Response::SharedPtr response) {
                servo->getPosition(response->position);
            });
        }

        void initialize() {
            // Use this->shared_from_this() since rclcpp::Node already supports it
            servo = std::make_unique<mrover::Servo>(3);
        }

    private:
        rclcpp::Subscription<mrover::msg::DynamixelSetPosition>::SharedPtr setPositionSubscriber;
        rclcpp::Service<mrover::srv::DynamixelGetPosition>::SharedPtr getPositionService;

        std::unordered_map<std::string, Servo> servos;

        std::unique_ptr<mrover::Servo> servo;
    };

} // namespace mrover

auto main(int argc, char** argv) -> int {
    rclcpp::init(argc, argv);
    auto led = std::make_shared<mrover::PDLBBridge>();
    led->initialize();
    rclcpp::spin(led);
    rclcpp::shutdown();
    return EXIT_SUCCESS;
}