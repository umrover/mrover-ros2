#include "can_device.hpp"
#include "messaging.hpp"

#include <mrover/msg/led.hpp>

#include <memory>
#include <rclcpp/rclcpp.hpp>

namespace mrover {

    class PDLBBridge final : public rclcpp::Node {
    public:
        PDLBBridge() : rclcpp::Node("pdlb_hw_bridge") {
            changeLEDSubscriber_ = this->create_subscription<mrover::msg::LED>(
                    "led", 10, std::bind(&PDLBBridge::changeLED, this, std::placeholders::_1));
        }

        void initialize() {
            // Use this->shared_from_this() since rclcpp::Node already supports it
            ledCanDevice = std::make_unique<mrover::CanDevice>(this->shared_from_this(), "jetson", "pdlb");
        }

        void changeLED(mrover::msg::LED::SharedPtr const msg) {
            if (!ledCanDevice) {
                RCLCPP_ERROR(this->get_logger(), "ledCanDevice not initialized!");
                return;
            }
            mrover::LEDInfo ledInfo{};
            ledInfo.red = msg->red;
            ledInfo.green = msg->green;
            ledInfo.blue = msg->blue;
            ledInfo.blinking = msg->is_blinking;
            ledCanDevice->publish_message(mrover::InBoundPDLBMessage{mrover::LEDCommand{.led_info = ledInfo}});
        }

    private:
        rclcpp::Subscription<mrover::msg::LED>::SharedPtr changeLEDSubscriber_;
        std::unique_ptr<mrover::CanDevice> ledCanDevice;
    };

} // namespace mrover

auto main(int argc, char** argv) -> int {
    rclcpp::init(argc, argv);

    auto node = std::make_shared<mrover::PDLBBridge>();
    node->initialize(); // Call initialize() after shared_ptr is created

    rclcpp::spin(node);
    rclcpp::shutdown();
    return EXIT_SUCCESS;
}
