#pragma once

#include "can_device.hpp"
#include <rclcpp/rclcpp.hpp>
#include <variant>

// Standard ROS 2 message includes
#include <mrover/msg/temperature.hpp>
#include <mrover/msg/humidity.hpp>
#include <mrover/msg/pressure.hpp>
#include <mrover/msg/oxygen.hpp>
#include <mrover/msg/ozone.hpp>
#include <mrover/msg/co2.hpp>
#include <mrover/msg/uv.hpp>

namespace mrover {

    class ScienceBoard {
    public:
        ScienceBoard(rclcpp::Node::SharedPtr node, std::string masterName, std::string deviceName)
            : mNode{node},
              mDevice{node, std::move(masterName), std::move(deviceName),
                      [this](CANMsg_t const& msg) { processMessage(msg); }} {

            // initialize publishers
            mTemperaturePub = mNode->create_publisher<msg::Temperature>("sp_temperature_data", 10);
            mHumidityPub = mNode->create_publisher<msg::Humidity>("sp_humidity_data", 10);
            mPressurePub = mNode->create_publisher<msg::Pressure>("sp_pressure_data", 10);
            mOxygenPub = mNode->create_publisher<msg::Oxygen>("sp_oxygen_data", 10);
            mOzonePub = mNode->create_publisher<msg::Ozone>("sp_ozone_data", 10);
            mCO2Pub = mNode->create_publisher<msg::CO2>("sp_co2_data", 10);
            mUVPub = mNode->create_publisher<msg::UV>("sp_uv_data", 10);
        }

        auto processMessage(CANMsg_t const& msg) -> void {
            std::visit([this](auto const& decoded) {
                using T = std::decay_t<decltype(decoded)>;

                // send science sensor data
                if constexpr (std::is_same_v<T, SCISensorData>) {
                    publishROSData(decoded);
                }
            }, msg);
        }

    private:
        rclcpp::Node::SharedPtr mNode;
        CANDevice mDevice;

        rclcpp::Publisher<msg::Temperature>::SharedPtr mTemperaturePub;
        rclcpp::Publisher<msg::Humidity>::SharedPtr mHumidityPub;
        rclcpp::Publisher<msg::Pressure>::SharedPtr mPressurePub;
        rclcpp::Publisher<msg::Oxygen>::SharedPtr mOxygenPub;
        rclcpp::Publisher<msg::Ozone>::SharedPtr mOzonePub;
        rclcpp::Publisher<msg::CO2>::SharedPtr mCO2Pub;
        rclcpp::Publisher<msg::UV>::SharedPtr mUVPub;

        auto publishROSData(SCISensorData const& data) -> void {
            msg::UV uv_msg;
            uv_msg.uv_index = data.uv_index;
            mUVPub->publish(uv_msg);

            msg::Temperature temp_msg;
            temp_msg.temperature = data.temperature;
            mTemperaturePub->publish(temp_msg);

            msg::Humidity hum_msg;
            hum_msg.relative_humidity = data.humidity;
            mHumidityPub->publish(hum_msg);

            msg::Pressure press_msg;
            press_msg.pressure = data.pressure;
            mPressurePub->publish(press_msg);

            msg::Oxygen o2_msg;
            o2_msg.percent = data.oxygen;
            mOxygenPub->publish(o2_msg);

            msg::Ozone ozone_msg;
            ozone_msg.ppb = data.ozone;
            mOzonePub->publish(ozone_msg);

            msg::CO2 co2_msg;
            co2_msg.percent = data.co2;
            mCO2Pub->publish(co2_msg);
        }
    };

} // namespace mrover
