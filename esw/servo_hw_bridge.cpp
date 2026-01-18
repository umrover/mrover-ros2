#include "can_device.hpp"
#include "messaging.hpp"

#include <cstdint>
#include <mrover/msg/led.hpp>

#include <memory>
#include <mrover/msg/pdlb.hpp>
#include <rclcpp/rclcpp.hpp>
#include <std_srvs/srv/set_bool.hpp>
#include <unordered_map>

#include "mrover/msg/servo_position.hpp"
#include "mrover/srv/servo_position.hpp"
#include "servo_library/servo.hpp"


namespace mrover {

    class PDLBBridge final : public rclcpp::Node {
    public:
        PDLBBridge() : rclcpp::Node("servo_hw_bridge") {
            Servo::init("/dev/ttyUSB0");
            setPositionSubscriber = create_subscription<mrover::msg::ServoPosition>("set_position", 10, [this](mrover::msg::ServoPosition::ConstSharedPtr const& msg) {
                servos.at(msg->name)->setPosition(msg->position, Servo::ServoMode::Optimal);
            });

            getPositionService = this->create_service<mrover::srv::ServoPosition>("get_position", [this](
                                                                                                             mrover::srv::ServoPosition::Request::SharedPtr const& request,
                                                                                                              mrover::srv::ServoPosition::Response::SharedPtr response) {
                servos.at(request->name)->getPosition(request->position);

                const auto timeout = std::chrono::seconds(3);
                const auto start = this->get_clock()->now();                                                                                 
                Servo::ServoStatus status = servos.at(request->name)->setPosition(request->position, Servo::ServoMode::Optimal);
                


                while(status == Servo::ServoStatus::Active){
                    status = servos.at(request->name)->getTargetStatus();
                    if(this->get_clock()->now() - start > timeout){
                        RCLCPP_WARN(this->get_logger(), "Timeout reached while waiting for servo to reach target position");
                        break;
                    }
                }
                response->at_tgt = (status == Servo::ServoStatus::Success);

            });
        }

        void create_servo(uint8_t id, const std::string& name)
        {
          servos.insert({name, std::make_unique<mrover::Servo>(id, name)});
        }

        void initialize() {
            // Use this->shared_from_this() since rclcpp::Node already supports it
            // servo = std::make_unique<mrover::Servo>(3);

            create_servo(4, "name");
        } 

    private:
        rclcpp::Subscription<mrover::msg::ServoPosition>::SharedPtr setPositionSubscriber;
        rclcpp::Service<mrover::srv::ServoPosition>::SharedPtr getPositionService;

        std::unordered_map<std::string, std::unique_ptr<mrover::Servo>> servos;

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