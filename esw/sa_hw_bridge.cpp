#include <boost/asio/io_service.hpp>
#include <cstring>
#include <fcntl.h>
#include <chrono>
#include <memory>
#include <string>
#include <unordered_map>
#include <vector>

#include <rclcpp/rclcpp.hpp>

#include <units.hpp>
#include <units_eigen.hpp>

#include <boost/asio.hpp>
#include <boost/algorithm/string.hpp>

#include <mrover/msg/controller_state.hpp>
#include <mrover/msg/position.hpp>
#include <mrover/msg/throttle.hpp>
#include <mrover/msg/velocity.hpp>
#include <mrover/srv/adjust_motor.hpp>
#include <mrover/msg/position.hpp>
#include <mrover/srv/servo_set_pos.hpp>
#include <sensor_msgs/msg/joint_state.hpp>
#include <sensor_msgs/msg/relative_humidity.hpp>
#include <sensor_msgs/msg/temperature.hpp>

#include "motor_library/brushed.hpp"

namespace mrover {

#pragma pack(push, 1)
        static constexpr std::uint8_t HEADER_BYTE = 0xA6;
        static constexpr std::uint8_t SERVO_SET_POSITION = 0x00;

        struct ServoSetPosition {
            std::uint8_t header = HEADER_BYTE;
            std::uint8_t messageID = SERVO_SET_POSITION;
            std::uint8_t id{}; // internal id of servo
            std::uint8_t isCounterClockwise{};
            float radians{};
        };
#pragma pack(pop)

    class SAHWBridge : public rclcpp::Node {
        static constexpr int SERIAL_POLL_FREQ = 10;
        static constexpr int SERIAL_INPUT_MSG_SIZE = 14;

        const std::uint8_t mServoID = 0;
        std::vector<std::string> const mMotorNames = {"linear_actuator", "auger", "pump_a", "pump_b", "sensor_actuator"};
        std::unordered_map<std::string, std::shared_ptr<BrushedController>> mMotors;

        rclcpp::TimerBase::SharedPtr mPublishDataTimer;

        rclcpp::Subscription<msg::Throttle>::SharedPtr mSAThrottleSub;

        rclcpp::Publisher<sensor_msgs::msg::JointState>::SharedPtr mJointDataPub;
        rclcpp::Publisher<msg::ControllerState>::SharedPtr mControllerStatePub;
        rclcpp::Publisher<sensor_msgs::msg::Temperature>::SharedPtr mTemperatureDataPub;
        rclcpp::Publisher<sensor_msgs::msg::RelativeHumidity>::SharedPtr mHumidityDataPub;
        rclcpp::Publisher<msg::Position>::SharedPtr mServoPositionPub;

        rclcpp::Service<srv::ServoSetPos>::SharedPtr mSetServoPositionSrv;
        
        sensor_msgs::msg::JointState mJointData;
        msg::ControllerState mControllerState;

        boost::asio::serial_port mSerial;
        boost::asio::io_service& io;
        boost::asio::streambuf mInputBuffer;
        std::vector<unsigned char> mBuffer;
        std::deque<std::vector<unsigned char>> mWriteQueue;
        unsigned long mSerialBaudRate{};
        std::string mSerialPort;

        auto processThrottleCmd(msg::Throttle::ConstSharedPtr const& msg) -> void {
            if (msg->names.size() != msg->throttles.size()) {
                RCLCPP_ERROR(get_logger(), "Name count and value count mismatched!");
                return;
            }

            for (std::size_t i = 0; i < msg->names.size(); ++i) {
                std::string const& name = msg->names[i];
                Dimensionless const& throttle = msg->throttles[i];
                mMotors[name]->setDesiredThrottle(throttle);
            }
        }

        auto publishDataCallback() -> void {
            mJointData.header.stamp = get_clock()->now();

            for (size_t i = 0; i < mMotorNames.size(); ++i) {
                auto const& name = mMotorNames[i];
                auto const& motor = mMotors[name];

                mJointData.position[i] = {motor->getPosition().get()};
                mJointData.velocity[i] = {motor->getVelocity().get()};
                mJointData.effort[i] = {motor->getEffort()};

                mControllerState.state[i] = {motor->getState()};
                mControllerState.error[i] = {motor->getErrorState()};
                mControllerState.limit_hit[i] = {motor->getLimitsHitBits()};
            }

            mJointDataPub->publish(mJointData);
            mControllerStatePub->publish(mControllerState);
        }

        auto setServoPositionServiceCallback(srv::ServoSetPos::Request::ConstSharedPtr const req, const srv::ServoSetPos::Response::SharedPtr res) -> void {
            ServoSetPosition set_pos {
                .id = mServoID,
                .isCounterClockwise = req->is_counterclockwise,
                .radians = req->position
            };

            // parse set pos into a vector of bytes
            std::vector<unsigned char> bytes(sizeof(set_pos));
            std::memcpy(bytes.data(), &set_pos, sizeof(set_pos));

            bool is_writing = !mWriteQueue.empty();
            mWriteQueue.push_back(bytes);

            if (!is_writing) {
                startAsyncWrite();
            }

            res->success = true; // TODO: what is fail condition here? service no longer performs serial write
        }

        auto startAsyncRead() -> void {
            boost::asio::async_read(mSerial, boost::asio::buffer(mBuffer), boost::asio::transfer_exactly(SERIAL_INPUT_MSG_SIZE),
            [this](const boost::system::error_code& ec, std::size_t len) {
                if (ec) {
                    RCLCPP_ERROR(this->get_logger(), "Serial read failed: %s", ec.message().c_str());
                } else if (len != SERIAL_INPUT_MSG_SIZE) {
                    RCLCPP_ERROR(this->get_logger(), "Failed to read whole serial buffer");
                } else {
                    // RCLCPP_INFO(this->get_logger(), "data:");
                    // for (auto val : mBuffer) {
                    //     RCLCPP_INFO(this->get_logger(), "\t%x", val);
                    // }
                    parseAndPublishBuffer(mBuffer);
                }
                // continue reads
                startAsyncRead();
            });
        }

        void startAsyncWrite() {
            boost::asio::async_write(mSerial, boost::asio::buffer(mWriteQueue.front()), [this](const boost::system::error_code& ec, std::size_t) {
                if (ec) {
                    RCLCPP_ERROR(this->get_logger(), "Serial write failed: %s", ec.message().c_str());
                }
                // RCLCPP_INFO(this->get_logger(), "size of set_pos: %lu", mWriteQueue.front().size());
                // for (auto val : mWriteQueue.front()) {
                //     RCLCPP_INFO(this->get_logger(), "\t%x", val);
                // }
                mWriteQueue.pop_front();
                if (!mWriteQueue.empty()) {
                    startAsyncWrite();
                }
            });
        }

        auto parseAndPublishBuffer(std::vector<unsigned char>& buffer) const -> void {
            // read from serial port and parse data
            float raw_pos_rad{};
            float raw_temp{};
            float raw_hum{};

            auto data = buffer.data();

            if (*data == HEADER_BYTE) {
                ++data; // points to DXL_ID
                const auto dxl_id = static_cast<uint8_t>(*data);
                RCLCPP_DEBUG(this->get_logger(), "Using serial device %u", dxl_id);
                ++data;
                std::memcpy(&raw_pos_rad, data, sizeof(float));
                data += 4;
                std::memcpy(&raw_temp, data, sizeof(float));
                data += 4;
                std::memcpy(&raw_hum, data, sizeof(float));
            } else {
                RCLCPP_WARN(this->get_logger(), "The first byte in serial message was not the header byte");
            }

            // send data to pubs
            sensor_msgs::msg::Temperature temp_msg;
            sensor_msgs::msg::RelativeHumidity humidity_msg;
            msg::Position position_msg;

            temp_msg.temperature = raw_temp;
            humidity_msg.relative_humidity = raw_hum;
            position_msg.names = { "gear_diff" };
            position_msg.positions = { raw_pos_rad };

            mServoPositionPub->publish(position_msg);
            mTemperatureDataPub->publish(temp_msg);
            mHumidityDataPub->publish(humidity_msg);
        }

    public:
        explicit SAHWBridge(boost::asio::io_service& io) : rclcpp::Node{"sa_hw_bridge"}, mSerial(io), io(io), mBuffer(SERIAL_INPUT_MSG_SIZE) {
            // all initialization is done in the init() function to allow for the usage of shared_from_this()
        }

        auto init() -> void {

            // parse port and baud parameters
            mSerialPort = this->declare_parameter<std::string>("port_unicore", "/dev/ttyACM0");
            mSerialBaudRate = this->declare_parameter<int>("baud_unicore", 115200);

            // configure inputs
            mSAThrottleSub = create_subscription<msg::Throttle>("sa_throttle_cmd", 1, [this](msg::Throttle::ConstSharedPtr const& msg) { processThrottleCmd(msg); });

            // configure outputs
            mJointDataPub = create_publisher<sensor_msgs::msg::JointState>("sa_joint_data", 1);
            mControllerStatePub = create_publisher<msg::ControllerState>("sa_controller_state", 1);
            mTemperatureDataPub = create_publisher<sensor_msgs::msg::Temperature>("sa_temp_data", SERIAL_POLL_FREQ);
            mHumidityDataPub = create_publisher<sensor_msgs::msg::RelativeHumidity>("sa_humidity_data", SERIAL_POLL_FREQ);
            mServoPositionPub = create_publisher<msg::Position>("sa_gear_diff_position", SERIAL_POLL_FREQ);
            
            // configure services
            mSetServoPositionSrv = create_service<srv::ServoSetPos>(
                "sa_gear_diff_set_position",
                [this](srv::ServoSetPos::Request::ConstSharedPtr const req, srv::ServoSetPos::Response::SharedPtr res) {
                    setServoPositionServiceCallback(req, res);
                }
            );

            // define periodic publishing
            mPublishDataTimer = create_wall_timer(
                    std::chrono::milliseconds(100),
                    [this]() { publishDataCallback(); });

            // configure serial io
            boost::system::error_code ec;
            mSerial.open(mSerialPort, ec);
            if (ec) {
                RCLCPP_FATAL(this->get_logger(), "Couldn't open serial port: %s", ec.message().c_str());
                rclcpp::shutdown();
                return;
            }
            mSerial.set_option(boost::asio::serial_port_base::baud_rate(mSerialBaudRate));

            // configure motor controllers
            for (auto const& name: mMotorNames) {
                mMotors[name] = std::make_shared<BrushedController>(shared_from_this(), "jetson", name);
            }

            // configure 3bm data message
            mJointData.name = mMotorNames;
            mJointData.position.resize(mMotorNames.size());
            mJointData.velocity.resize(mMotorNames.size());
            mJointData.effort.resize(mMotorNames.size());

            mControllerState.name = mMotorNames;
            mControllerState.state.resize(mMotorNames.size());
            mControllerState.error.resize(mMotorNames.size());
            mControllerState.limit_hit.resize(mMotorNames.size());

            // begin serial reads
            startAsyncRead();
            io.run();
        }

        auto stop() -> void {
            boost::system::error_code ec;
            mSerial.close(ec);
            if (ec) {
                RCLCPP_WARN(this->get_logger(), "Failed to close serial port: %s", ec.message().c_str());
            }
        }
        
    };
} // namespace mrover


auto main(const int argc, char** argv) -> int {
    rclcpp::init(argc, argv);

    boost::asio::io_service io;
    auto sa_hw_bridge = std::make_shared<mrover::SAHWBridge>(io);

    sa_hw_bridge->init();
    rclcpp::spin(sa_hw_bridge);

    sa_hw_bridge->stop();
    rclcpp::shutdown();
    return EXIT_SUCCESS;
}
