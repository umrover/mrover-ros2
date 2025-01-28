#include <algorithm>
#include <memory>
#include <unordered_map>

#include <rclcpp/rclcpp.hpp>
#include <sensor_msgs/msg/relative_humidity.hpp>
#include <sensor_msgs/msg/temperature.hpp>

#include <messaging.hpp>
#include <units.hpp>

#include <mrover/msg/can.hpp>
#include <mrover/msg/methane.hpp>
#include <mrover/msg/oxygen.hpp>
#include <mrover/msg/uv.hpp>

namespace mrover {

    class ScienceBridge final : public rclcpp::Node {

    private:
        rclcpp::Publisher<sensor_msgs::msg::Temperature>::SharedPtr tempPub;
        rclcpp::Publisher<sensor_msgs::msg::RelativeHumidity>::SharedPtr humidityPub;
        rclcpp::Publisher<msg::Oxygen>::SharedPtr oxygenPub;
        rclcpp::Publisher<msg::Methane>::SharedPtr methanePub;
        rclcpp::Publisher<msg::UV>::SharedPtr uvPub;

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

        void processMessage([[maybe_unused]] mrover::SpectralData const& message) {
            // Just here for the compiler to stay happy :D
        }

        void processMessage(mrover::SensorData const& message) {
            switch (static_cast<ScienceDataID>(message.id)) {
                case ScienceDataID::TEMPERATURE: {
                    sensor_msgs::msg::Temperature msg;
                    msg.temperature = message.data;
                    msg.variance = 0;
                    tempPub->publish(msg);
                    break;
                }

                case ScienceDataID::HUMIDITY: {
                    sensor_msgs::msg::RelativeHumidity msg;
                    msg.relative_humidity = message.data;
                    msg.variance = 0;
                    humidityPub->publish(msg);
                    break;
                }

                case ScienceDataID::OXYGEN: {
                    msg::Oxygen msg;
                    msg.percent = message.data;
                    msg.variance = 0;
                    oxygenPub->publish(msg);
                    break;
                }

                case ScienceDataID::METHANE: {
                    msg::Methane msg;
                    msg.ppm = message.data;
                    msg.variance = 0;
                    methanePub->publish(msg);
                    break;
                }

                case ScienceDataID::UV: {
                    msg::UV msg;
                    msg.uv_index = message.data;
                    msg.variance = 0;
                    uvPub->publish(msg);
                    break;
                }
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
            tempPub = create_publisher<sensor_msgs::msg::Temperature>("science_temperature_data", 10);
            humidityPub = create_publisher<sensor_msgs::msg::RelativeHumidity>("science_humidity_data", 10);
            oxygenPub = create_publisher<msg::Oxygen>("science_oxygen_data", 10);
            methanePub = create_publisher<msg::Methane>("science_methane_data", 10);
            uvPub = create_publisher<msg::UV>("science_uv_data", 10);

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
