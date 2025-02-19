#include "messaging.hpp"
#include "can_device.hpp"

#include <mrover/msg/led.hpp>

#include <rclcpp/rclcpp.hpp>
#include <memory>
#include <mrover/msg/PDLB.hpp>

namespace mrover {

    class PDLBBridge final : public rclcpp::Node {
    public:
        PDLBBridge() : rclcpp::Node("pdlb_hw_bridge") {
            changeLEDSubscriber_ = this->create_subscription<mrover::msg::LED>(
                "led", 10, std::bind(&PDLBBridge::changeLED, this, std::placeholders::_1));
            PDLCANSubscriber_ = create_subscription<msg::CAN>(
                "can/pdlb/in", 10, [this](const msg::CAN::SharedPtr msg) { processCANMessage(msg); });
        }

        void initialize() {
            // Use this->shared_from_this() since rclcpp::Node already supports it
            ledCanDevice = std::make_unique<mrover::CanDevice>(this->shared_from_this(), "jetson", "pdlb");
        }

        void changeLED(const mrover::msg::LED::SharedPtr msg) {
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
        void processMessage(const mrover::PDBData msg) {
            mrover::msg::PDLB msgToSend;
            msgToSend.temperatures = msg.temperatures;
            msgToSend.currents = msg.currents;
            // Publish the message
            pdlbPublisher_->publish(msgToSend);
        }
        void processCANMessage(msg::CAN::ConstSharedPtr msg){
            OutBoundPDLBMessage const& message = *reinterpret_cast<OutBoundPDLBMessage const*>(msg->data);
            std::visit([this](auto&& arg) { processMessage(arg); }, message);
        }
    private:
        rclcpp::Subscription<mrover::msg::LED>::SharedPtr changeLEDSubscriber_;
        rclcpp::Subscription<msg::CAN>::ConstSharedPtr PDLCANSubscriber_;
        std::unique_ptr<mrover::CanDevice> ledCanDevice;
        rclcpp::Publisher<mrover::msg::PDLB>::SharedPtr pdlbPublisher_;
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
