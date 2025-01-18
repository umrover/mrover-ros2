#include "lib/messaging.hpp"
#include <memory>
#include <unordered_map>
#include <algorithm>
#include <rclcpp/rclcpp.hpp>
#include "std_msgs/msg/float32.hpp"
#include <mrover/msg/can.hpp>
#include <units/units.hpp>

namespace mrover {

    class ScienceTestBridge final : public rclcpp::Node {

    private:
        rclcpp::Publisher<msg::CAN>::SharedPtr scienceAPub;
        rclcpp::Publisher<msg::CAN>::SharedPtr scienceBPub;

    public:
        ScienceTestBridge() : Node{"science_hw_bridge"} {
            scienceAPub = create_publisher<msg::CAN>("can/science_a/in", 10);
            scienceBPub = create_publisher<msg::CAN>("can/science_b/in", 10);
        }

        void publishCanData() {
            msg::CAN msgA;
            msg::CAN msgB;

            msgA.source = "science_a";
            msgA.destination = "jetson";
            msgA.reply_required = false; 

            msgB.source = "science_b";
            msgB.destination = "jetson";
            msgB.reply_required = false;

            for (uint8_t i = 1; i < 6; i++) {
                msgA.data.push_back(i);
                msgB.data.push_back(i);
            }

            scienceAPub->publish(msgA);
            scienceBPub->publish(msgB);
        }
    };

} // namespace mrover

auto main(int argc, char** argv) -> int {
    rclcpp::init(argc, argv);
    rclcpp::Rate rate(10);
    mrover::ScienceTestBridge test_bridge;
    while (rclcpp::ok()) {
        test_bridge.publishCanData();
        rate.sleep();
    }
    rclcpp::shutdown();
    return EXIT_SUCCESS;
}
