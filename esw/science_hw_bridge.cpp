#include "lib/messaging.hpp"
#include "std_msgs/msg/float32.hpp"
#include <algorithm>
#include <memory>
#include <mrover/msg/can.hpp>
#include <rclcpp/rclcpp.hpp>
#include <units/units.hpp>
#include <unordered_map>

namespace mrover {

    class ScienceBridge final : public rclcpp::Node {

    private:
        rclcpp::Publisher<std_msgs::msg::Float32>::SharedPtr tempPub;
        rclcpp::Publisher<std_msgs::msg::Float32>::SharedPtr humidityPub;
        rclcpp::Publisher<std_msgs::msg::Float32>::SharedPtr oxygenPub;
        rclcpp::Publisher<std_msgs::msg::Float32>::SharedPtr methanePub;
        rclcpp::Publisher<std_msgs::msg::Float32>::SharedPtr uvPub;

        rclcpp::Subscription<msg::CAN>::ConstSharedPtr canSubA;
        rclcpp::Subscription<msg::CAN>::ConstSharedPtr canSubB;

        void processMessage([[maybe_unused]] mrover::HeaterStateData const& message) {
            // // ROS_ERROR("heater!");
            // mrover::HeaterData heaterData;
            // // TODO - this crashes program!
            // heaterData.state.resize(6);
            // for (int i = 0; i < 6; ++i) {
            //     heaterData.state.at(i) = GET_BIT_AT_INDEX(message.heater_state_info.on, i);
            // }

            // heaterDataPublisher->publish(heaterData);
        }

        void processMessage([[maybe_unused]] mrover::ThermistorData const& message) {
            // // ROS_ERROR("Thermistors!");
            // mrover::ScienceThermistors scienceThermistors;
            // scienceThermistors.temps.resize(6);
            // for (int i = 0; i < 6; ++i) {
            //     scienceThermistors.temps.at(i).temperature = message.temps.at(i);
            // }
            // thermistorDataPublisher->publish(scienceThermistors);
        }

        void processMessage(mrover::SensorData const& message) {
            std_msgs::msg::Float32 msg;
            switch (static_cast<ScienceDataID>(message.id)) {
                case ScienceDataID::TEMPERATURE:
                    msg.data = message.data;
                    tempPub->publish(msg);
                    break;

                case ScienceDataID::HUMIDITY:
                    msg.data = message.data;
                    humidityPub->publish(msg);
                    break;

                case ScienceDataID::OXYGEN:
                    msg.data = message.data;
                    oxygenPub->publish(msg);
                    break;

                case ScienceDataID::METHANE:
                    msg.data = message.data;
                    methanePub->publish(msg);
                    break;

                case ScienceDataID::UV:
                    msg.data = message.data;
                    uvPub->publish(msg);
                    break;
            }
        }

        void processCANData(msg::CAN::ConstSharedPtr const& msg) {
            // TODO - fix in future
            // ROS_ERROR("Source: %s Destination: %s", msg->source.c_str(), msg->destination.c_str());

            mrover::OutBoundScienceMessage const& message = *reinterpret_cast<mrover::OutBoundScienceMessage const*>(msg->data.data());
            std::visit([&](auto const& messageAlternative) { processMessage(messageAlternative); }, message);
        }

    public:
        ScienceBridge() : Node{"science_hw_bridge"} {
            tempPub = create_publisher<std_msgs::msg::Float32>("science_temperature_data", 10);
            humidityPub = create_publisher<std_msgs::msg::Float32>("science_humidity_data", 10);
            oxygenPub = create_publisher<std_msgs::msg::Float32>("science_oxygen_data", 10);
            methanePub = create_publisher<std_msgs::msg::Float32>("science_methane_data", 10);
            uvPub = create_publisher<std_msgs::msg::Float32>("science_uv_data", 10);

            // [this](msg::CAN::ConstSharedPtr const& msg) { processCANMessage(msg); });
            canSubA = create_subscription<msg::CAN>("can/science_a/in", 10, [this](msg::CAN::ConstSharedPtr const& msg) { processCANData(msg); });
            canSubB = create_subscription<msg::CAN>("can/science_b/in", 10, [this](msg::CAN::ConstSharedPtr const& msg) { processCANData(msg); });
        }
    };

} // namespace mrover

auto main(int argc, char** argv) -> int {
    rclcpp::init(argc, argv);
    rclcpp::spin(std::make_shared<mrover::ScienceBridge>());
    rclcpp::shutdown();
    return EXIT_SUCCESS;
}
