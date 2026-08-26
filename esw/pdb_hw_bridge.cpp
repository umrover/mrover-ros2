#include "MRoverCAN.hpp"
#include "can_device.hpp"
#include <rclcpp/rclcpp.hpp>

#include <mrover/msg/led.hpp>
#include <mrover/srv/pdb_reset.hpp>

namespace mrover {

    class PDBHWBridge final : public rclcpp::Node {
    private:
        CANDevice mDevice;
        rclcpp::Subscription<mrover::msg::LED>::SharedPtr mChangeLEDSubscriber;
        rclcpp::Service<srv::PDBReset>::SharedPtr mPDBReset;

    public:
        PDBHWBridge() : Node{"pdb_hw_bridge"} {}

        void changeLED(mrover::msg::LED::ConstSharedPtr const& ros_msg) {
            bool red = ros_msg->color == mrover::msg::LED::RED;
            bool green = ros_msg->color == mrover::msg::LED::BLINKING_GREEN;
            bool blue = ros_msg->color == mrover::msg::LED::BLUE;
            bool blinking = ros_msg->color == mrover::msg::LED::BLINKING_GREEN;

            CANMsg_t const can_msg = PDBAutonLEDCmd(red, green, blue, blinking);
            mDevice.publishMessage(can_msg);
        }

        void processRequest(srv::PDBReset::Request::SharedPtr const req, srv::PDBReset::Response::SharedPtr res) {
            CANMsg_t const msg = PDBResetCmd(req->reset, req->clear_faults);
            mDevice.publishMessage(msg);

            res->success = true;
        }

        void init() {
            mDevice = {rclcpp::Node::shared_from_this(), "jetson", "pdb"};

            mChangeLEDSubscriber = create_subscription<mrover::msg::LED>("led", 10,
                                                                         [this](mrover::msg::LED::ConstSharedPtr const& msg) {
                                                                             changeLED(msg);
                                                                         });

            mPDBReset = create_service<srv::PDBReset>("pdb_reset",
                                                      [this](srv::PDBReset::Request::SharedPtr const req, srv::PDBReset::Response::SharedPtr res) {
                                                          processRequest(req, res);
                                                      });
        }
    };

} // namespace mrover

auto main(int argc, char** argv) -> int {
    rclcpp::init(argc, argv);

    auto pdb = std::make_shared<mrover::PDBHWBridge>();
    pdb->init();

    rclcpp::spin(pdb);
    rclcpp::shutdown();
    return EXIT_SUCCESS;
}
