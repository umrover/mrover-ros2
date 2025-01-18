#include "lib/messaging.hpp"
#include <memory>
#include <unordered_map>
#include <algorithm>
#include <rclcpp/rclcpp.hpp>
#include "std_msgs/msg/float32.hpp"
#include <mrover/msg/can.hpp>
#include <units.hpp>

namespace mrover {

    class ScienceBridge final : public rclcpp::Node {
    public:
        ScienceBridge() : Node{"science_hw_bridge"} {
            auto tempPub = create_publisher<std_msgs::msg::Float32>("science_temperature_data", 10);
            auto humidityPub = create_publisher<std_msgs::msg::Float32>("science_humidity_data", 10);
            auto oxygenPub = create_publisher<std_msgs::msg::Float32>("science_oxygen_data", 10);
            auto methanePub = create_publisher<std_msgs::msg::Float32>("science_methane_data", 10);
            auto uvPub = create_publisher<std_msgs::msg::Float32>("science_uv_data", 10);

            auto canSubA = create_subscription<msg::CAN>("can/science_a/in", 10, processCANData(msg::CAN::ConstPtr const& msg));
            auto canSubB = create_subscription<msg::CAN>("can/science_b/in", 10, processCANData(msg::CAN::ConstPtr const& msg));
        }

    private:
        // void processMessage(mrover::HeaterStateData const& message) {
        //     // ROS_ERROR("heater!");
        //     mrover::HeaterData heaterData;
        //     // TODO - this crashes program!
        //     heaterData.state.resize(6);
        //     for (int i = 0; i < 6; ++i) {
        //         heaterData.state.at(i) = GET_BIT_AT_INDEX(message.heater_state_info.on, i);
        //     }

        //     heaterDataPublisher->publish(heaterData);
        // }

        // void processMessage(mrover::ThermistorData const& message) {
        //     // ROS_ERROR("Thermistors!");
        //     mrover::ScienceThermistors scienceThermistors;
        //     scienceThermistors.temps.resize(6);
        //     for (int i = 0; i < 6; ++i) {
        //         scienceThermistors.temps.at(i).temperature = message.temps.at(i);
        //     }
        //     thermistorDataPublisher->publish(scienceThermistors);
        // }

        void processMessage(mrover::SensorData const& message) {
            switch (message.id) {
                case 1:
                    tempPub->publish(message.data);
                    break;
                
                case 2:
                    humidityPub->publish(message.data);
                    break;

                case 3:
                    oxygenPub->publish(message.data);
                    break;

                case 4:
                    methanePub->publish(message.data);
                    break;

                case 5:
                    uvPub->publish(message.data);
                    break;
            }
        }

        void processCANData(msg::CAN::ConstPtr const& msg) {
            // TODO - fix in future
            // ROS_ERROR("Source: %s Destination: %s", msg->source.c_str(), msg->destination.c_str());
            
            mrover::OutBoundScienceMessage const& message = *reinterpret_cast<mrover::OutBoundScienceMessage const*>(msg->data.data());
            std::visit([&](auto const& messageAlternative) { processMessage(messageAlternative); }, message);
        }
    };

} // namespace mrover

auto main(int argc, char** argv) -> int {
    rclcpp::init(argc, argv);
    rclcpp::spin(std::make_shared<mrover::ScienceBridge>());
    rclcpp::shutdown();
    return EXIT_SUCCESS;
}
