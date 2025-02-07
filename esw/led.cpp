#include <rclcpp/rclcpp.hpp>
#include "std_srvs/srv/detail/set_bool__struct.hpp"

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

rclcpp::Publisher led_publisher;

// When navigation reaches a waypoint it will publish "DoneState" to the "nav_state" topic

bool is_navigation_done = false;
bool is_teleop_enabled = false;
LEDMode led_mode = LEDMode::Unknown;

auto update_led() -> void {
    // Teleoperation takes precedence over autonomous control
    if (is_teleop_enabled) {
        led_mode = LEDMode::Blue;
    } else if (is_navigation_done) {
        led_mode = LEDMode::BlinkingGreen;
    } else {
        led_mode = LEDMode::Red;
    }

    mrover::msg::LED led_msg;
    led_msg.red = led_mode == LEDMode::Red;
    led_msg.green = led_mode == LEDMode::BlinkingGreen;
    led_msg.blue = led_mode == LEDMode::Blue;
    led_msg.is_blinking = led_mode == LEDMode::BlinkingGreen;
    led_publisher.publish(led_msg);
}

auto state_machine_state_update(mrover::msg::StateMachineStateUpdate::ConstPtr const& msg) -> void {
    is_navigation_done = msg->state == DONE_STATE;

    update_led();
}

auto teleop_enabled_update(std_srvs::srv::SetBoolRequest& request, std_srvs::srv::SetBoolResponse& response) -> bool {
    is_teleop_enabled = request.data;

    update_led();

    return response.success = true;
}

auto main(int argc, char** argv) -> int {
    rclcpp::init(argc, argv);

    led_publisher = nh.advertise<mrover::msg::LED>("led", 1);
    rclcpp::Service teleop_enabled_client = nh.advertiseService("enable_teleop", teleop_enabled_update);
    ros::Subscriber state_machine_update_sub = nh.subscribe<mrover::StateMachineStateUpdate>("nav_state", 1, state_machine_state_update);

    ros::spin();

    return 0;
}
