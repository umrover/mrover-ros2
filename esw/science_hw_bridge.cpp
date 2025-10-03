#include <cstdint>
#include <cstring>
#include <memory>

#include <rclcpp/node.hpp>
#include <rclcpp/rclcpp.hpp>
#include <sensor_msgs/msg/relative_humidity.hpp>
#include <sensor_msgs/msg/temperature.hpp>

#include <can_device.hpp>
#include <messaging_science.hpp>
#include <parameter.hpp>
#include <units.hpp>

#include <mrover/msg/can.hpp>
#include <mrover/msg/heater_data.hpp>
#include <mrover/msg/oxygen.hpp>
#include <mrover/msg/science_thermistors.hpp>
#include <mrover/msg/uv.hpp>
#include <mrover/srv/enable_bool.hpp>


namespace mrover {

    enum ScienceBoard {
        A = 0,
        B = 1,
    };

    class ScienceHWBridge final : public rclcpp::Node {

    private:
        static constexpr std::uint8_t NUM_HEATERS = 4;
        static constexpr std::uint8_t NUM_THERMISTORS = 2;
        bool mUVSensorOnA;
        bool mTempHumiditySensorOnA;
        bool mOxygenSensorOnA;
        std::array<std::uint8_t, NUM_HEATERS> mHeaterStates;
        std::array<float, NUM_THERMISTORS> mThermistorTemps;

        rclcpp::Publisher<sensor_msgs::msg::Temperature>::SharedPtr mTemperaturePub;
        rclcpp::Publisher<sensor_msgs::msg::RelativeHumidity>::SharedPtr mHumidityPub;
        rclcpp::Publisher<msg::Oxygen>::SharedPtr mOxygenPub;
        rclcpp::Publisher<msg::UV>::SharedPtr mUVPub;
        rclcpp::Publisher<msg::ScienceThermistors>::SharedPtr mThermistorsPub;
        rclcpp::Publisher<msg::HeaterData>::SharedPtr mHeaterPub;

        rclcpp::Service<srv::EnableBool>::SharedPtr mHeaterAutoShutoffSrv;

        rclcpp::Service<srv::EnableBool>::SharedPtr mScienceEnableSrvHA0;
        rclcpp::Service<srv::EnableBool>::SharedPtr mScienceEnableSrvHB0;
        rclcpp::Service<srv::EnableBool>::SharedPtr mScienceEnableSrvHA1;
        rclcpp::Service<srv::EnableBool>::SharedPtr mScienceEnableSrvHB1;
        rclcpp::Service<srv::EnableBool>::SharedPtr mScienceEnableSrvWLA;
        rclcpp::Service<srv::EnableBool>::SharedPtr mScienceEnableSrvWLB;

        rclcpp::Subscription<msg::CAN>::ConstSharedPtr mCANSubA;
        rclcpp::Subscription<msg::CAN>::ConstSharedPtr mCANSubB;

        CanDevice mCANDeviceA;
        CanDevice mCANDeviceB;

        ScienceBoard prevScienceMessage;

        void processHeaterAutoShutoff(std::shared_ptr<srv::EnableBool::Request> const& request, std::shared_ptr<srv::EnableBool::Response>& response) {
            mCANDeviceA.publish_message(InBoundScienceMessage{HeaterAutoShutOffCommand{.enable_auto_shutoff = request->enable}});
            mCANDeviceB.publish_message(InBoundScienceMessage{HeaterAutoShutOffCommand{.enable_auto_shutoff = request->enable}});
            response->success = true;
            response->message = request->enable ? "Enabled" : "Disabled";
        }

        void processScienceEnable(ScienceBoard scienceBoard, ScienceDevice const dev, std::shared_ptr<srv::EnableBool::Request> const& request, std::shared_ptr<srv::EnableBool::Response>& response) {
            if (scienceBoard == ScienceBoard::A)
                mCANDeviceA.publish_message(InBoundScienceMessage{EnableScienceDeviceCommand{.science_device = dev, .enable = request->enable}});
            else
                mCANDeviceB.publish_message(InBoundScienceMessage{EnableScienceDeviceCommand{.science_device = dev, .enable = request->enable}});

            response->success = true;
            response->message = request->enable ? "Enabled" : "Disabled";
        }

        void processMessage(mrover::HeaterStateData const& message) {
            msg::HeaterData heaterData;
            heaterData.state.resize(NUM_HEATERS);

            for (int i = 0; i < NUM_HEATERS; i++) {
                heaterData.state.at(i) = mHeaterStates[i];
            }

            for (int i = 0; i < NUM_THERMISTORS; i++) {
                int h = prevScienceMessage == ScienceBoard::A ? i : i + NUM_THERMISTORS;
                heaterData.state.at(h) = GET_BIT_AT_INDEX(message.heater_state_info.on, i);
                mHeaterStates[h] = GET_BIT_AT_INDEX(message.heater_state_info.on, i);
            }

            mHeaterPub->publish(heaterData);
        }

        void processMessage(mrover::ThermistorData const& message) {
            msg::ScienceThermistors scienceThermistors;
            scienceThermistors.temps.resize(NUM_HEATERS);

            for (int i = 0; i < NUM_HEATERS; i++) {
                scienceThermistors.temps.at(i).temperature = mThermistorTemps[i / 2];
            }

            for (int i = 0; i < NUM_THERMISTORS; i++) {
                int t = prevScienceMessage == ScienceBoard::A ? i : i + NUM_THERMISTORS;
                scienceThermistors.temps.at(t).temperature = message.temps[i];
                mThermistorTemps[i] = message.temps[i];
            }

            mThermistorsPub->publish(scienceThermistors);
        }

        void processMessage(mrover::SensorData const& message) {
            switch (static_cast<ScienceDataID>(message.id)) {
                case ScienceDataID::TEMPERATURE: {
                    if ((prevScienceMessage == ScienceBoard::A && mTempHumiditySensorOnA) || (prevScienceMessage == ScienceBoard::B && !mTempHumiditySensorOnA)) {
                        sensor_msgs::msg::Temperature msg;
                        msg.temperature = message.data;
                        msg.variance = 0;
                        mTemperaturePub->publish(msg);
                        break;
                    }
                }

                case ScienceDataID::HUMIDITY: {
                    if ((prevScienceMessage == ScienceBoard::A && mTempHumiditySensorOnA) || (prevScienceMessage == ScienceBoard::B && !mTempHumiditySensorOnA)) {
                        sensor_msgs::msg::RelativeHumidity msg;
                        msg.relative_humidity = message.data;
                        msg.variance = 0;
                        mHumidityPub->publish(msg);
                        break;
                    }
                }

                case ScienceDataID::OXYGEN: {
                    if ((prevScienceMessage == ScienceBoard::A && mOxygenSensorOnA) || (prevScienceMessage == ScienceBoard::B && !mOxygenSensorOnA)) {
                        msg::Oxygen msg;
                        msg.percent = message.data;
                        msg.variance = 0;
                        mOxygenPub->publish(msg);
                        break;
                    }
                }

                case ScienceDataID::UV: {
                    if ((prevScienceMessage == ScienceBoard::A && mUVSensorOnA) || (prevScienceMessage == ScienceBoard::B && !mUVSensorOnA)) {
                        msg::UV msg;
                        msg.uv_index = message.data;
                        msg.variance = 0;
                        mUVPub->publish(msg);
                        break;
                    }
                }
            }
        }

        void processCANData(msg::CAN::ConstSharedPtr const& msg) {
            if (msg->source == "science_a")
                prevScienceMessage = ScienceBoard::A;
            else if (msg->source == "science_b")
                prevScienceMessage = ScienceBoard::B;

            OutBoundScienceMessage const& message = *reinterpret_cast<OutBoundScienceMessage const*>(msg->data.data());
            std::visit([&](auto const& messageAlternative) { processMessage(messageAlternative); }, message);
        }

    public:
        ScienceHWBridge() : Node{"science_hw_bridge"} {}

        void init() {
            std::vector<ParameterWrapper> parameters = {{"uv_sensor_on_a", mUVSensorOnA, true},
                                                        {"temperature_humidity_sensor_on_a", mTempHumiditySensorOnA, true},
                                                        {"oxygen_sensor_on_a", mOxygenSensorOnA, true}};

            ParameterWrapper::declareParameters(rclcpp::Node::shared_from_this().get(), parameters);

            for (int i = 0; i < NUM_HEATERS; i++) {
                mHeaterStates[i] = 0;
                mThermistorTemps[i] = 0;
            }

            mTemperaturePub = create_publisher<sensor_msgs::msg::Temperature>("science_temperature_data", 10);
            mHumidityPub = create_publisher<sensor_msgs::msg::RelativeHumidity>("science_humidity_data", 10);
            mOxygenPub = create_publisher<msg::Oxygen>("science_oxygen_data", 10);
            mUVPub = create_publisher<msg::UV>("science_uv_data", 10);
            mThermistorsPub = create_publisher<msg::ScienceThermistors>("science_thermistors", 10);
            mHeaterPub = create_publisher<msg::HeaterData>("science_heater_state", 10);

            mHeaterAutoShutoffSrv = create_service<srv::EnableBool>("science_change_heater_auto_shutoff_state",
                                                                    [this](srv::EnableBool::Request::SharedPtr const& req, srv::EnableBool::Response::SharedPtr& res) { processHeaterAutoShutoff(req, res); });

            mScienceEnableSrvHA0 = create_service<srv::EnableBool>("science_enable_heater_a0",
                                                                   [this](srv::EnableBool::Request::SharedPtr const& req, srv::EnableBool::Response::SharedPtr& res) { processScienceEnable(ScienceBoard::A, ScienceDevice::HEATER_0, req, res); });

            mScienceEnableSrvHB0 = create_service<srv::EnableBool>("science_enable_heater_b0",
                                                                   [this](srv::EnableBool::Request::SharedPtr const& req, srv::EnableBool::Response::SharedPtr& res) { processScienceEnable(ScienceBoard::B, ScienceDevice::HEATER_0, req, res); });

            mScienceEnableSrvHA1 = create_service<srv::EnableBool>("science_enable_heater_a1",
                                                                   [this](srv::EnableBool::Request::SharedPtr const& req, srv::EnableBool::Response::SharedPtr& res) { processScienceEnable(ScienceBoard::A, ScienceDevice::HEATER_1, req, res); });

            mScienceEnableSrvHB1 = create_service<srv::EnableBool>("science_enable_heater_b1",
                                                                   [this](srv::EnableBool::Request::SharedPtr const& req, srv::EnableBool::Response::SharedPtr& res) { processScienceEnable(ScienceBoard::B, ScienceDevice::HEATER_1, req, res); });

            mScienceEnableSrvWLA = create_service<srv::EnableBool>("science_enable_white_led_a",
                                                                   [this](srv::EnableBool::Request::SharedPtr const& req, srv::EnableBool::Response::SharedPtr& res) { processScienceEnable(ScienceBoard::A, ScienceDevice::WHITE_LED, req, res); });

            mScienceEnableSrvWLB = create_service<srv::EnableBool>("science_enable_white_led_b",
                                                                   [this](srv::EnableBool::Request::SharedPtr const& req, srv::EnableBool::Response::SharedPtr& res) { processScienceEnable(ScienceBoard::B, ScienceDevice::WHITE_LED, req, res); });

            mCANSubA = create_subscription<msg::CAN>("/can/science_a/in", 10, [this](msg::CAN::ConstSharedPtr const& msg) { processCANData(msg); });
            mCANSubB = create_subscription<msg::CAN>("/can/science_b/in", 10, [this](msg::CAN::ConstSharedPtr const& msg) { processCANData(msg); });

            mCANDeviceA = CanDevice(rclcpp::Node::shared_from_this(), "jetson", "science_a");
            mCANDeviceB = CanDevice(rclcpp::Node::shared_from_this(), "jetson", "science_b");
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
