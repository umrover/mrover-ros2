#include <memory>
#include <rclcpp/create_subscription.hpp>
#include <rclcpp/logger.hpp>
#include <rclcpp/rclcpp.hpp>
#include <rclcpp/subscription.hpp>
#include <std_srvs/srv/detail/set_bool__struct.hpp>

#include <mrover/msg/can.hpp>
#include <mrover/msg/led.hpp>
#include <mrover/msg/state_machine_state_update.hpp>

enum class LEDMode {
    Unknown = 0,
    Off = 1,
    Red = 2,
    BlinkingGreen = 3,
    Blue = 4
};

static constexpr std::string_view DONE_STATE = "DoneState";

namespace mrover {
    class LED final : public rclcpp::Node {
    public:
        LED() : Node{"led"} {
            mStateSub = create_subscription<mrover::msg::StateMachineStateUpdate>(
                    "/nav_state", 10, [this](mrover::msg::StateMachineStateUpdate::ConstSharedPtr const& msg) {
                        stateMachineUpdateCallback(msg);
                    });

            mLedPub = create_publisher<mrover::msg::LED>("led", 10);

            mTeleopEnableServer = create_service<std_srvs::srv::SetBool>(
                    "/enable_teleop", [this](std_srvs::srv::SetBool::Request::SharedPtr const& req,
                                             std_srvs::srv::SetBool::Response::SharedPtr const& res) {
                        teleopEnabledCallback(req, res);
                    });
        }

        auto updateLED() -> void {
            if (is_teleop_enabled) {
                led_mode = LEDMode::Blue;
            } else if (is_navigation_done) {
                led_mode = LEDMode::BlinkingGreen;
            } else {
                led_mode = LEDMode::Red;
            }

            mrover::msg::LED led_msg;

            if (led_mode == LEDMode::Red) {
                led_msg.color = mrover::msg::LED::RED;
            } else if (led_mode == LEDMode::Blue) {
                led_msg.color = mrover::msg::LED::BLUE;
            } else if (led_mode == LEDMode::BlinkingGreen) {
                led_msg.color = mrover::msg::LED::BLINKING_GREEN;
            }

            mLedPub->publish(led_msg);
        }

        // When navigation reaches a waypoint it will publish "DoneState" to the "nav_state" topic
        auto stateMachineUpdateCallback(mrover::msg::StateMachineStateUpdate::ConstSharedPtr const& msg) -> void {
            is_navigation_done = msg->state == DONE_STATE;

            updateLED();
        }

        auto teleopEnabledCallback(std_srvs::srv::SetBool::Request::SharedPtr const& request, std_srvs::srv::SetBool::Response::SharedPtr const& response) -> bool {
            is_teleop_enabled = request->data;

            updateLED();

            return response->success = true;
        }

    private:
        bool is_navigation_done = false;
        bool is_teleop_enabled = false;
        LEDMode led_mode = LEDMode::Unknown;

        rclcpp::Publisher<mrover::msg::LED>::SharedPtr mLedPub;
        rclcpp::Subscription<mrover::msg::StateMachineStateUpdate>::SharedPtr mStateSub;
        rclcpp::Service<std_srvs::srv::SetBool>::SharedPtr mTeleopEnableServer;
    };
} // namespace mrover

auto main(int argc, char** argv) -> int {
    rclcpp::init(argc, argv);
    rclcpp::spin(std::make_shared<mrover::LED>());
    rclcpp::shutdown();

    return 0;
}