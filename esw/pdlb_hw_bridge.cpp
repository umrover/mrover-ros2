#include "can_device.hpp"
#include "messaging.hpp"

#include <boost/asio.hpp>
#include <mrover/msg/led.hpp>

#include <memory>
#include <mrover/msg/pdlb.hpp>
#include <rclcpp/rclcpp.hpp>
#include <std_srvs/srv/set_bool.hpp>

namespace mrover {

    class PDLBBridge final : public rclcpp::Node {
    public:
        PDLBBridge() : rclcpp::Node("pdlb_hw_bridge"), serial(io, "/dev/led_arduino") {
            changeLEDSubscriber = create_subscription<mrover::msg::LED>("led", 10, [this](mrover::msg::LED::ConstSharedPtr const& msg) {
                PDLBBridge::changeLED(msg);
            });
            // PDLCANSubscriber = create_subscription<msg::CAN>("can/pdlb/in", 10, [this](msg::CAN::ConstSharedPtr const& msg) {
            //     PDLBBridge::processCANMessage(msg);
            // });
        }

        void initialize() {
            serial.set_option(boost::asio::serial_port_base::baud_rate(115200));

            // ledCanDevice = std::make_unique<mrover::CanDevice>(this->shared_from_this(), "jetson", "pdlb");
        }

        void changeLED(mrover::msg::LED::ConstSharedPtr const& msg) {
            // if (!ledCanDevice) {
            //     RCLCPP_ERROR(this->get_logger(), "ledCanDevice not initialized!");
            //     return;
            // }
            mrover::LEDInfo ledInfo{};
            ledInfo.red = msg->color == mrover::msg::LED::RED;
            ledInfo.green = msg->color == mrover::msg::LED::BLINKING_GREEN;
            ledInfo.blue = msg->color == mrover::msg::LED::BLUE;
            ledInfo.blinking = msg->color == mrover::msg::LED::BLINKING_GREEN;

            char c = msg->color == mrover::msg::LED::RED ? 'R' : msg->color == mrover::msg::LED::BLINKING_GREEN ? 'G' : 'B';
            boost::asio::write(serial, boost::asio::buffer(&c, 1));
            // ledCanDevice->publish_message(mrover::InBoundPDLBMessage{mrover::LEDCommand{.led_info = ledInfo}});
        }

        // void handleEnableArmLaser(
        //         std::shared_ptr<std_srvs::srv::SetBool::Request> const& request,
        //         std::shared_ptr<std_srvs::srv::SetBool::Response>& response) {
        //     if (request->data) {
        //         // Code to enable the arm laser
        //         response->message = "Arm laser enabled";
        //         ledCanDevice->publish_message(mrover::InBoundPDLBMessage{mrover::ArmLaserCommand{.enable = true}});
        //         response->success = true;
        //     } else {
        //         // Code to disable the arm laser
        //         response->message = "Arm laser disabled";
        //         ledCanDevice->publish_message(mrover::InBoundPDLBMessage{mrover::ArmLaserCommand{.enable = false}});
        //         response->success = true;
        //     }
        // }

        // void processMessage(mrover::PDBData const msg) {
        //     mrover::msg::PDLB msgToSend;
        //     msgToSend.temperatures = msg.temperatures;
        //     msgToSend.currents = msg.currents;
        //     // Publish the message
        //     pdlbPublisher->publish(msgToSend);
        // }
        // void processCANMessage(msg::CAN::ConstSharedPtr const& msg) {
        //     OutBoundPDLBMessage const& message = *reinterpret_cast<OutBoundPDLBMessage const*>(msg->data.data());
        //     std::visit([this](auto&& arg) { processMessage(arg); }, message);
        // }

    private:
        rclcpp::Subscription<mrover::msg::LED>::SharedPtr changeLEDSubscriber;
        // rclcpp::Subscription<msg::CAN>::ConstSharedPtr PDLCANSubscriber;
        // // std::unique_ptr<mrover::CanDevice> ledCanDevice;
        // rclcpp::Publisher<mrover::msg::PDLB>::SharedPtr pdlbPublisher;

        boost::asio::io_service io;
        boost::asio::serial_port serial;
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