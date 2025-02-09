#include <cstdint>
#include <rclcpp/rclcpp.hpp>

#include <mrover/msg/can.hpp>

#include <messaging_science.hpp>
#include <units.hpp>

namespace mrover {

    union TestUnion {
        uint8_t on : 6 {};
        uint8_t data_bytes[1];
    };

    class ScienceTestBridge final : public rclcpp::Node {

    private:
        rclcpp::Publisher<msg::CAN>::SharedPtr scienceAPub;
        rclcpp::Publisher<msg::CAN>::SharedPtr scienceBPub;

    public:
        ScienceTestBridge() : Node{"science_test_bridge"} {
            scienceAPub = create_publisher<msg::CAN>("can/science_a/in", 10);
            scienceBPub = create_publisher<msg::CAN>("can/science_b/in", 10);
        }

        void publishCanData() {
            msg::CAN msgA;
            msg::CAN msgB;

            msgA.data.resize(1);
            msgB.data.resize(1);

            msgA.source = "science_a";
            msgA.destination = "jetson";
            msgA.reply_required = false;

            msgB.source = "science_b";
            msgB.destination = "jetson";
            msgB.reply_required = false;

            TestUnion test_union{};

            SET_BIT_AT_INDEX(test_union.on, 0, 1);
            SET_BIT_AT_INDEX(test_union.on, 1, 0);
            SET_BIT_AT_INDEX(test_union.on, 2, 1);
            SET_BIT_AT_INDEX(test_union.on, 3, 0);
            SET_BIT_AT_INDEX(test_union.on, 4, 1);
            SET_BIT_AT_INDEX(test_union.on, 5, 0);

            for (uint8_t i = 0; i < 1; i++) {
                msgA.data.at(i) = test_union.data_bytes[i];
                msgB.data.at(i) = test_union.data_bytes[i];
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