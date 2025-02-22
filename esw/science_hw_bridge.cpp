#include "mrover/srv/detail/enable_bool__struct.hpp"
#include <cstring>
#include <memory>

#include <rclcpp/node.hpp>
#include <rclcpp/rclcpp.hpp>
#include <sensor_msgs/msg/relative_humidity.hpp>
#include <sensor_msgs/msg/temperature.hpp>

#include <can_device.hpp>
#include <messaging_science.hpp>
#include <units.hpp>

#include <mrover/msg/can.hpp>
#include <mrover/msg/oxygen.hpp>
#include <mrover/msg/uv.hpp>
#include <mrover/msg/science_thermistors.hpp>
#include <mrover/msg/heater_data.hpp>

#include <mrover/srv/enable_bool.hpp>


namespace mrover {

    enum ScienceBoard {
        A = 0,
        B = 1,
    };

    class ScienceBridge final : public rclcpp::Node {

    private:
        rclcpp::Publisher<sensor_msgs::msg::Temperature>::SharedPtr tempPub;
        rclcpp::Publisher<sensor_msgs::msg::RelativeHumidity>::SharedPtr humidityPub;
        rclcpp::Publisher<msg::Oxygen>::SharedPtr oxygenPub;
        rclcpp::Publisher<msg::UV>::SharedPtr uvPub;
        rclcpp::Publisher<msg::ScienceThermistors>::SharedPtr thermistorsPub;
        rclcpp::Publisher<msg::HeaterData>::SharedPtr heaterPub;

        rclcpp::Service<srv::EnableBool>::SharedPtr heaterAutoShutoffSrv;

        rclcpp::Service<srv::EnableBool>::SharedPtr scienceEnableSrvHA0;
        rclcpp::Service<srv::EnableBool>::SharedPtr scienceEnableSrvHB0;
        rclcpp::Service<srv::EnableBool>::SharedPtr scienceEnableSrvHA1;
        rclcpp::Service<srv::EnableBool>::SharedPtr scienceEnableSrvHB1;
        rclcpp::Service<srv::EnableBool>::SharedPtr scienceEnableSrvWLA;
        rclcpp::Service<srv::EnableBool>::SharedPtr scienceEnableSrvWLB;

        rclcpp::Subscription<msg::CAN>::ConstSharedPtr canSubA;
        rclcpp::Subscription<msg::CAN>::ConstSharedPtr canSubB;

        mrover::CanDevice canDevA;
        mrover::CanDevice canDevB;

        void processHeaterAutoShutoff(const std::shared_ptr<srv::EnableBool::Request> request, std::shared_ptr<srv::EnableBool::Response> response) {
            canDevA.publish_message(InBoundScienceMessage{HeaterAutoShutOffCommand{.enable_auto_shutoff = request->enable}});
            canDevB.publish_message(InBoundScienceMessage{HeaterAutoShutOffCommand{.enable_auto_shutoff = request->enable}}); 
            response->success = true;
            response->message = request->enable ? "Enabled" : "Disabled";
        }

        void processScienceEnable(ScienceBoard scienceBoard, const ScienceDevice dev, const std::shared_ptr<srv::EnableBool::Request> request, std::shared_ptr<srv::EnableBool::Response> response) {
            if (scienceBoard == ScienceBoard::A)
                canDevA.publish_message(InBoundScienceMessage{EnableScienceDeviceCommand{.science_device = dev, .enable = request->enable}});
            else
                canDevB.publish_message(InBoundScienceMessage{EnableScienceDeviceCommand{.science_device = dev, .enable = request->enable}});
            
            response->success = true;
            response->message = request->enable ? "Enabled" : "Disabled";
        }

        void processMessage(mrover::HeaterStateData const& message) {
            // RCLCPP_ERROR(get_logger(), "Heaters!");
            msg::HeaterData heaterData;
            heaterData.state.resize(6);
            for (int i = 0; i < 6; ++i) {
                heaterData.state.at(i) = GET_BIT_AT_INDEX(message.heater_state_info.on, i);
            }
            heaterPub->publish(heaterData);
        }

        void processMessage(mrover::ThermistorData const& message) {
            //RCLCPP_ERROR(get_logger(), "Thermistor data!");
            msg::ScienceThermistors scienceThermistors;
            scienceThermistors.temps.resize(6);
            for (int i = 0; i < 6; ++i) {
                scienceThermistors.temps.at(i).temperature = message.temps[i];
            }
            thermistorsPub->publish(scienceThermistors);
        }

        void processMessage(mrover::SensorData const& message) {
            // RCLCPP_ERROR(get_logger(), "Sensor data!");
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
            //RCLCPP_ERROR(get_logger(), "Source: %s Destination: %s", msg->source.c_str(), msg->destination.c_str());
            OutBoundScienceMessage const& message = *reinterpret_cast<OutBoundScienceMessage const*>(msg->data.data());
            std::visit([&](auto const& messageAlternative) { processMessage(messageAlternative); }, message);
        }

    public:
        ScienceBridge() : Node{"science_hw_bridge"} {}

        void init() {
            tempPub = create_publisher<sensor_msgs::msg::Temperature>("science_temperature_data", 10);
            humidityPub = create_publisher<sensor_msgs::msg::RelativeHumidity>("science_humidity_data", 10);
            oxygenPub = create_publisher<msg::Oxygen>("science_oxygen_data", 10);
            uvPub = create_publisher<msg::UV>("science_uv_data", 10);
            thermistorsPub = create_publisher<msg::ScienceThermistors>("science_thermistors", 10);
            heaterPub = create_publisher<msg::HeaterData>("science_heater_state", 10);

            heaterAutoShutoffSrv = create_service<srv::EnableBool>("science_change_heater_auto_shutoff_state", 
                            [this](srv::EnableBool::Request::SharedPtr const req,srv::EnableBool::Response::SharedPtr res) {processHeaterAutoShutoff(req, res);});
            
            scienceEnableSrvHA0 = create_service<srv::EnableBool>("science_enable_heater_a0", 
                            [this](srv::EnableBool::Request::SharedPtr const req,srv::EnableBool::Response::SharedPtr res) {processScienceEnable(ScienceBoard::A, ScienceDevice::HEATER_0, req, res);});
                
            scienceEnableSrvHB0 = create_service<srv::EnableBool>("science_enable_heater_b0", 
                            [this](srv::EnableBool::Request::SharedPtr const req,srv::EnableBool::Response::SharedPtr res) {processScienceEnable(ScienceBoard::B, ScienceDevice::HEATER_0, req, res);});

            scienceEnableSrvHA1 = create_service<srv::EnableBool>("science_enable_heater_a1", 
                            [this](srv::EnableBool::Request::SharedPtr const req,srv::EnableBool::Response::SharedPtr res) {processScienceEnable(ScienceBoard::A, ScienceDevice::HEATER_1, req, res);});

            scienceEnableSrvHB1 = create_service<srv::EnableBool>("science_enable_heater_b1", 
                            [this](srv::EnableBool::Request::SharedPtr const req,srv::EnableBool::Response::SharedPtr res) {processScienceEnable(ScienceBoard::B, ScienceDevice::HEATER_1, req, res);});

            scienceEnableSrvWLA = create_service<srv::EnableBool>("science_enable_white_led_a", 
                            [this](srv::EnableBool::Request::SharedPtr const req,srv::EnableBool::Response::SharedPtr res) {processScienceEnable(ScienceBoard::A, ScienceDevice::WHITE_LED, req, res);});

            scienceEnableSrvWLB = create_service<srv::EnableBool>("science_enable_white_led_b", 
                            [this](srv::EnableBool::Request::SharedPtr const req,srv::EnableBool::Response::SharedPtr res) {processScienceEnable(ScienceBoard::B, ScienceDevice::WHITE_LED, req, res);});
                        
            canSubA = create_subscription<msg::CAN>("/can/science_a/in", 10, [this](msg::CAN::ConstSharedPtr const& msg) { processCANData(msg); });
            canSubB = create_subscription<msg::CAN>("/can/science_b/in", 10, [this](msg::CAN::ConstSharedPtr const& msg) { processCANData(msg); });

            canDevA = CanDevice(rclcpp::Node::shared_from_this(), "jetson", "science_a");
            canDevB = CanDevice(rclcpp::Node::shared_from_this(), "jetson", "science_b");
        }
    };

} // namespace mrover

auto main(int argc, char** argv) -> int {
    rclcpp::init(argc, argv);
    auto scienceBridge = std::make_shared<mrover::ScienceBridge>();
    scienceBridge->init();
    rclcpp::spin(scienceBridge);
    rclcpp::shutdown();
    return EXIT_SUCCESS;
}
