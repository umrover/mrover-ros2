#include "can_device.hpp"
#include "messaging.hpp"

#include <mrover/msg/led.hpp>

#include <memory>
#include <mrover/msg/pdlb.hpp>
#include <rclcpp/rclcpp.hpp>
#include <std_srvs/srv/set_bool.hpp>

namespace mrover {

    class PDLBBridge final : public rclcpp::Node {
    public:
        PDLBBridge() : rclcpp::Node("pdlb_hw_bridge") {
            changeLEDSubscriber = create_subscription<mrover::msg::LED>("led", 10, [this](mrover::msg::LED::ConstSharedPtr const& msg) {
                PDLBBridge::changeLED(msg);
            });
            PDLCANSubscriber = create_subscription<msg::CAN>( "can/pdlb/in", 10, [this](msg::CAN::ConstSharedPtr const& msg) {
                PDLBBridge::processCANMessage(msg);
            });

            enableArmLaserService = this->create_service<std_srvs::srv::SetBool>("enable_arm_laser", [this] (
                std_srvs::srv::SetBool::Request::SharedPtr const& request,
                std_srvs::srv::SetBool::Response::SharedPtr response) {
                    PDLBBridge::handleEnableArmLaser(request, response);
                });
        }

        void initialize() {
            // Use this->shared_from_this() since rclcpp::Node already supports it
            ledCanDevice = std::make_unique<mrover::CanDevice>(this->shared_from_this(), "jetson", "pdlb");
        }

        void changeLED(mrover::msg::LED::ConstSharedPtr const& msg) {
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

        void handleEnableArmLaser(
                std::shared_ptr<std_srvs::srv::SetBool::Request> const& request,
                std::shared_ptr<std_srvs::srv::SetBool::Response> &response) {
            if (request->data) {
                // Code to enable the arm laser
                response->message = "Arm laser enabled";
                ledCanDevice->publish_message(mrover::InBoundPDLBMessage{mrover::ArmLaserCommand{.enable = true}});
                response->success = true;
            } else {
                // Code to disable the arm laser
                response->message = "Arm laser disabled";
                ledCanDevice->publish_message(mrover::InBoundPDLBMessage{mrover::ArmLaserCommand{.enable = false}});
                response->success = true;
            }
        }

        void processMessage(mrover::PDBData const msg) {
            mrover::msg::PDLB msgToSend;
            msgToSend.temperatures = msg.temperatures;
            msgToSend.currents = msg.currents;
            // Publish the message
            pdlbPublisher->publish(msgToSend);
        }
        void processCANMessage(msg::CAN::ConstSharedPtr const& msg) {
            OutBoundPDLBMessage const& message = *reinterpret_cast<OutBoundPDLBMessage const*>(msg->data.data());
            std::visit([this](auto&& arg) { processMessage(arg); }, message);
        }

    private:
        rclcpp::Subscription<mrover::msg::LED>::SharedPtr changeLEDSubscriber;
        rclcpp::Subscription<msg::CAN>::ConstSharedPtr PDLCANSubscriber;
        std::unique_ptr<mrover::CanDevice> ledCanDevice;
        rclcpp::Publisher<mrover::msg::PDLB>::SharedPtr pdlbPublisher;
        rclcpp::Service<std_srvs::srv::SetBool>::SharedPtr enableArmLaserService;
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