#include "can_device.hpp"
#include "messaging.hpp"


#include <rclcpp/rclcpp.hpp>
#include <rclcpp/subscription.hpp>
#include <rclcpp/timer.hpp>

#include <mrover/msg/LED.h>

namespace mrover {
    std::unique_ptr<mrover::CanDevice> ledCanDevice;

    class LedHardwareBridge final :  public rclcpp::Node{
        public:
            auto changeLED(mrover::LED::ConstPtr const& msg) -> void {
                mrover::LEDInfo ledInfo{};
                ledInfo.red = msg->red;
                ledInfo.green = msg->green;
                ledInfo.blue = msg->blue;
                ledInfo.blinking = msg->is_blinking;
                ledCanDevice->publish_message(mrover::InBoundPDLBMessage{mrover::LEDCommand{.led_info = ledInfo}});
            }
    };




    auto main(int argc, char** argv) -> int {
        // Initialize the ROS node
        rclcpp::init(argc, argv); 

        ledCanDevice = std::make_unique<mrover::CanDevice>(nh, "jetson", "pdlb");

        ros::Subscriber changeLEDSubscriber = nh.subscribe<mrover::LED>("led", 1, changeLED);
        ledStateSub = create_subscription<msg::>("can/science_a/in", 10, [this](msg::CAN::ConstSharedPtr const& msg) { processCANData(msg); });

        rclcpp::spin(LedHardwareBridge);
        rclcpp::shutdown();
        return EXIT_SUCCESS;
    }

}