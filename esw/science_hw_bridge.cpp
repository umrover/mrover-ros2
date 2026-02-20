#include "mrover/msg/detail/humidity__struct.hpp"
#include <cstddef>
#include <cstring>
#include <memory>
#include <span>

#include <rclcpp/node.hpp>
#include <rclcpp/rclcpp.hpp>
#include <sensor_msgs/msg/detail/temperature__struct.hpp>
#include <sensor_msgs/msg/relative_humidity.hpp>
#include <sensor_msgs/msg/temperature.hpp>

#include <can_device.hpp>

#include <mrover/msg/can.hpp>
#include <mrover/msg/temperature.hpp>
#include <mrover/msg/humidity.hpp>
#include <mrover/msg/pressure.hpp>
#include <mrover/msg/oxygen.hpp>
#include <mrover/msg/ozone.hpp>
#include <mrover/msg/co2.hpp>
#include <mrover/msg/uv.hpp>
// #include <pch.hpp>


namespace mrover {
    class ScienceHWBridge final : public rclcpp::Node {
    private:
        rclcpp::Publisher<msg::Temperature>::SharedPtr mTemperaturePub;
        rclcpp::Publisher<msg::Humidity>::SharedPtr mHumidityPub;
        rclcpp::Publisher<msg::Pressure>::SharedPtr mPressurePub;
        rclcpp::Publisher<msg::Oxygen>::SharedPtr mOxygenPub;
        rclcpp::Publisher<msg::Ozone>::SharedPtr mOzonePub;
        rclcpp::Publisher<msg::CO2>::SharedPtr mCO2Pub;
        rclcpp::Publisher<msg::UV>::SharedPtr mUVPub;

        rclcpp::Subscription<msg::CAN>::ConstSharedPtr mCANSub;

        CANDevice mCANDevice;

        void processSensorData(std::span<const float> message) {
            msg::UV uv_msg;
            uv_msg.uv_index = message[0];
            mUVPub->publish(uv_msg);

            msg::Temperature temp_msg;
            temp_msg.temperature = message[1];
            mTemperaturePub->publish(temp_msg);

            msg::Humidity humidity_msg;
            humidity_msg.relative_humidity = message[2];
            mHumidityPub->publish(humidity_msg);

            msg::Pressure pressure_msg;
            pressure_msg.pressure = message[3];
            mPressurePub->publish(pressure_msg);

            msg::Oxygen oxygen_msg;
            oxygen_msg.percent = message[4];
            mOxygenPub->publish(oxygen_msg);

            msg::Ozone ozone_msg;
            ozone_msg.ppb = message[5];
            mOzonePub->publish(ozone_msg);

            msg::CO2 co2_msg;
            co2_msg.percent = message[6];
            mCO2Pub->publish(co2_msg);
        }

        void processCANData(msg::CAN::ConstSharedPtr const& message) {
            processSensorData(std::span<const float>(reinterpret_cast<const float*>(message->data.data()), message->data.size() / sizeof(float)));
        }

    public:
        ScienceHWBridge() : Node{"science_hw_bridge"} {}

        void init() {
            mTemperaturePub = create_publisher<msg::Temperature>("sp_temperature_data", 10);
            mHumidityPub = create_publisher<msg::Humidity>("sp_humidity_data", 10);
            mPressurePub = create_publisher<msg::Pressure>("sp_pressure_data", 10);
            mOxygenPub = create_publisher<msg::Oxygen>("sp_oxygen_data", 10);
            mOzonePub = create_publisher<msg::Ozone>("sp_ozone_data", 10);
            mCO2Pub = create_publisher<msg::CO2>("sp_co2_data", 10);
            mUVPub = create_publisher<msg::UV>("sp_uv_data", 10);

            mCANSub = create_subscription<msg::CAN>("/can/science/in", 10, [this](msg::CAN::ConstSharedPtr const& msg) { processCANData(msg); });

            // mCANDevice = CanDevice(rclcpp::Node::shared_from_this(), "jetson", "science");
        }
    };

} // namespace mrover

auto main(int argc, char** argv) -> int {
    rclcpp::init(argc, argv);
    auto scienceBridge = std::make_shared<mrover::ScienceHWBridge>();
    scienceBridge->init();
    rclcpp::spin(scienceBridge);
    rclcpp::shutdown();
    return EXIT_SUCCESS;
}
