#include <cstdint>
#include <rclcpp/rclcpp.hpp>

#include <mrover/msg/can.hpp>

#include <messaging_science.hpp>
#include <sys/types.h>
#include <units.hpp>

namespace mrover {

    class ScienceTestBridge final : public rclcpp::Node {

    private:
        rclcpp::Publisher<msg::CAN>::SharedPtr scienceAPub;
        rclcpp::Publisher<msg::CAN>::SharedPtr scienceBPub;

    public:
        ScienceTestBridge() : Node{"science_test_bridge"} {
            scienceAPub = create_publisher<msg::CAN>("can/science_a/in", 10);
            scienceBPub = create_publisher<msg::CAN>("can/science_b/in", 10);
        }

        void publishSensorData(uint8_t id, float data) {
            OutBoundScienceMessage scienceMessage = SensorData{id, data};
            publishCanData(&scienceMessage);
        }

        void publishThermistorData(std::array<float, 6> temps) {
            OutBoundScienceMessage scienceMessage = ThermistorData{temps};
            publishCanData(&scienceMessage);
        }

        void publishHeaterData(std::array<bool, 6> on) {
            HeaterStateData heaterStateData;
            for (uint8_t i = 0; i < 6; i++) {
                SET_BIT_AT_INDEX(heaterStateData.heater_state_info.on, i, on[i]);
            }
            OutBoundScienceMessage scienceMessage = heaterStateData;
            publishCanData(&scienceMessage);
        }

        void publishCanData(OutBoundScienceMessage* testMessage) {
            msg::CAN msgA;
            msg::CAN msgB;

            msgA.source = "science_a";
            msgA.destination = "jetson";
            msgA.reply_required = false;

            msgB.source = "science_b";
            msgB.destination = "jetson";
            msgB.reply_required = false;

            auto bytePtr = reinterpret_cast<uint8_t*>(testMessage);
            std::array<uint8_t, sizeof(OutBoundScienceMessage)> byteArray;
            std::copy(bytePtr, bytePtr + sizeof(OutBoundScienceMessage), byteArray.begin());

            msgA.data.resize(byteArray.size());
            msgB.data.resize(byteArray.size());

            for (uint8_t i = 0; i < byteArray.size(); i++) {
                msgA.data.at(i) = byteArray[i];
                msgB.data.at(i) = byteArray[i];
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
        test_bridge.publishSensorData(1, 23.45);
        test_bridge.publishThermistorData({1.1,2.2,3.3,4.4,5.5,6.6});
        test_bridge.publishHeaterData({0,1,0,1,0,1});
        rate.sleep();
    }
    rclcpp::shutdown();
    return EXIT_SUCCESS;
}