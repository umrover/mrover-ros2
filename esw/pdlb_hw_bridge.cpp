#include "MRoverCAN.hpp"
#include "can_device.hpp"
#include <rclcpp/rclcpp.hpp>

#include <mrover/msg/led.hpp>
#include <mrover/srv/pdlb_reset.hpp>

namespace mrover {

    class PDLBHWBridge final : public rclcpp::Node {
    private:
        CANDevice mDevice;
        rclcpp::Subscription<mrover::msg::LED>::SharedPtr mChangeLEDSubscriber;
        rclcpp::Service<srv::PdlbReset>::SharedPtr mPdlbReset;

    public:
        PDLBHWBridge() : Node{"pdlb_hw_bridge"} {}

        void changeLED(mrover::msg::LED::ConstSharedPtr const& ros_msg) {
            bool red = ros_msg->color == mrover::msg::LED::RED;
            bool green = ros_msg->color == mrover::msg::LED::BLINKING_GREEN;
            bool blue = ros_msg->color == mrover::msg::LED::BLUE;
            bool blinking = ros_msg->color == mrover::msg::LED::BLINKING_GREEN;

            CANMsg_t const can_msg = AutonLEDCommand(red, green, blue, blinking);
            mDevice.publishMessage(can_msg);
        }

        void processRequest(srv::PdlbReset::Request::SharedPtr const req, srv::PdlbReset::Response::SharedPtr res) {
            CANMsg_t const msg = PDLBResetCommand(req->reset, req->clear_faults);
            mDevice.publishMessage(msg);

            res->success = true;
        }

        void init() {
            mDevice = {rclcpp::Node::shared_from_this(), "jetson", "pdlb"};

            mChangeLEDSubscriber = create_subscription<mrover::msg::LED>("led", 10,
                                                                         [this](mrover::msg::LED::ConstSharedPtr const& msg) {
                                                                             changeLED(msg);
                                                                         });

            mPdlbReset = create_service<srv::PdlbReset>("pdlb_reset",
                                                        [this](srv::PdlbReset::Request::SharedPtr const req, srv::PdlbReset::Response::SharedPtr res) {
                                                            processRequest(req, res);
                                                        });
        }
    };

} // namespace mrover

auto main(int argc, char** argv) -> int {
    rclcpp::init(argc, argv);

    auto pdlb = std::make_shared<mrover::PDLBHWBridge>();
    pdlb->init();

    rclcpp::spin(pdlb);
    rclcpp::shutdown();
    return EXIT_SUCCESS;
}