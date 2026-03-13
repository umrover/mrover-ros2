#pragma once

#include "CANBus1.hpp"
#include "can_device.hpp"
#include <rclcpp/rclcpp.hpp>
#include <typeinfo>
#include <variant>

// Standard ROS 2 message includes
#include <mrover/msg/co2.hpp>
#include <mrover/msg/humidity.hpp>
#include <mrover/msg/oxygen.hpp>
#include <mrover/msg/ozone.hpp>
#include <mrover/msg/pressure.hpp>
#include <mrover/msg/sensor_states.hpp>
#include <mrover/msg/temperature.hpp>
#include <mrover/msg/uv.hpp>
#include <mrover/srv/science_board_reset.hpp>

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
            mSensorStatesPub = mNode->create_publisher<msg::SensorStates>("sensor_states", 10);

            // initialize services
            mSBReset = mNode->create_service<srv::ScienceBoardReset>("sb_reset",
                                                                     [this](srv::ScienceBoardReset::Request::SharedPtr const req, srv::ScienceBoardReset::Response::SharedPtr res) { processRequest(req, res); });
        }

        void processRequest(srv::ScienceBoardReset::Request::SharedPtr const req, srv::ScienceBoardReset::Response::SharedPtr res) {
            CANMsg_t const msg = SCIResetCommand(req->reset, req->clear_faults);
            mDevice.publishMessage(msg);

            res->success = true;
        }

        void processMessage(CANMsg_t const& msg) {
            std::visit([this](auto const& decoded) {
                using T = std::decay_t<decltype(decoded)>;

                // send science sensor data
                if constexpr (std::is_same_v<T, SCISensorData>) {
                    publishROSData(decoded);
                } else if constexpr (std::is_same_v<T, SCISensorState>) {
                    publishROSData(decoded);
                } else {
                    RCLCPP_WARN(mNode->get_logger(), "science received unexpected message type %s", typeid(T).name());
                }
            },
                       msg);
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
        rclcpp::Publisher<msg::SensorStates>::SharedPtr mSensorStatesPub;

        rclcpp::Service<srv::ScienceBoardReset>::SharedPtr mSBReset;

        void publishROSData(SCISensorData const& data) {
            msg::UV yvMsg;
            yvMsg.uv_index = data.uv_index;
            mUVPub->publish(yvMsg);

            msg::Temperature tempMsg;
            tempMsg.temperature = data.temperature;
            mTemperaturePub->publish(tempMsg);

            msg::Humidity humMsg;
            humMsg.relative_humidity = data.humidity;
            mHumidityPub->publish(humMsg);

            msg::Pressure pressMsg;
            pressMsg.pressure = data.pressure;
            mPressurePub->publish(pressMsg);

            msg::Oxygen o2Msg;
            o2Msg.percent = data.oxygen;
            mOxygenPub->publish(o2Msg);

            msg::Ozone ozoneMsg;
            ozoneMsg.ppb = data.ozone;
            mOzonePub->publish(ozoneMsg);

            msg::CO2 co2Msg;
            co2Msg.percent = data.co2;
            mCO2Pub->publish(co2Msg);
        }

        void publishROSData(SCISensorState const& data) {
            msg::SensorStates sensorStates;
            sensorStates.uv_state = data.uv_state;
            sensorStates.thp_state = data.thp_state;
            sensorStates.oxygen_state = data.oxygen_state;
            sensorStates.ozone_state = data.ozone_state;
            sensorStates.co2_state = data.co2_state;
            mSensorStatesPub->publish(sensorStates);
        }
    };

} // namespace mrover
