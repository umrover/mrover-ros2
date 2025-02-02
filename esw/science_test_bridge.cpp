#include <rclcpp/rclcpp.hpp>

#include <mrover/msg/can.hpp>

#include <messaging.hpp>
#include <units.hpp>

namespace mrover {

    union TestUnion {
        double data;
        char data_bytes[8];
    };

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

            TestUnion test_union;
            test_union.data = 23.45;

            msgA.data.push_back(1);
            msgB.data.push_back(1);

            for (uint8_t i = 0; i < 8; i++) {
                msgA.data.push_back(test_union.data_bytes[i]);
                msgB.data.push_back(test_union.data_bytes[i]);
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
