#include <chrono>
#include <cstring>
#include <memory>
#include <string>
#include <unordered_map>
#include <vector>

#include <fcntl.h>

#include <boost/algorithm/string.hpp>
#include <boost/asio.hpp>
#include <boost/asio/io_service.hpp>

#include <rclcpp/rclcpp.hpp>

#include <mrover/msg/controller_state.hpp>
#include <mrover/msg/position.hpp>
#include <mrover/msg/throttle.hpp>
#include <mrover/msg/velocity.hpp>
#include <mrover/srv/adjust_motor.hpp>
#include <mrover/srv/servo_set_pos.hpp>
#include <sensor_msgs/msg/joint_state.hpp>
#include <sensor_msgs/msg/relative_humidity.hpp>
#include <sensor_msgs/msg/temperature.hpp>

#include "motor_library/brushed.hpp"
#include <units.hpp>

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
        static constexpr int SERIAL_INPUT_MSG_SIZE = 14;

        std::vector<std::string> const mMotorNames = {"linear_actuator", "auger", "pump_0", "pump_1", "sensor_actuator"};
        std::unordered_map<std::string, std::shared_ptr<BrushedController>> mMotors;

        rclcpp::TimerBase::SharedPtr mPublishDataTimer;

        rclcpp::Subscription<msg::Throttle>::SharedPtr mThrottleSub;

        rclcpp::Publisher<sensor_msgs::msg::JointState>::SharedPtr mJointDataPub;
        rclcpp::Publisher<msg::ControllerState>::SharedPtr mControllerStatePub;
        rclcpp::Publisher<sensor_msgs::msg::Temperature>::SharedPtr mTemperatureDataPub;
        rclcpp::Publisher<sensor_msgs::msg::RelativeHumidity>::SharedPtr mHumidityDataPub;
        rclcpp::Publisher<msg::Position>::SharedPtr mServoPositionPub;

        sensor_msgs::msg::JointState mJointData;
        msg::ControllerState mControllerState;


        rclcpp::Service<srv::ServoSetPos>::SharedPtr mSetServoPositionSrv;

        boost::asio::serial_port mSerial;
        boost::asio::io_service& io;
        boost::asio::streambuf mInputBuffer;
        std::vector<unsigned char> mBuffer;
        std::deque<std::vector<unsigned char>> mWriteQueue;
        unsigned long mSerialBaudRate{};
        std::uint8_t mServoID{};
        std::string mSerialPort;


        auto processThrottleCmd(msg::Throttle::ConstSharedPtr const& msg) -> void {
            if (msg->name.size() != msg->throttle.size()) {
                RCLCPP_ERROR(get_logger(), "Name count and value count mismatched!");
                return;
            }

            for (std::size_t i = 0; i < msg->name.size(); ++i) {
                std::string const& name = msg->name[i];
                Dimensionless const& throttle = msg->throttle[i];
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

        auto setServoPositionServiceCallback(srv::ServoSetPos::Request::ConstSharedPtr const& req, srv::ServoSetPos::Response::SharedPtr const& res) -> void {
            ServoSetPosition set_pos{
                    .id = mServoID,
                    .isCounterClockwise = req->is_counterclockwise,
                    .radians = req->position};

            // parse set pos into a vector of bytes
            std::vector<unsigned char> bytes(sizeof(set_pos));
            std::memcpy(bytes.data(), &set_pos, sizeof(set_pos));

            bool is_writing = !mWriteQueue.empty();
            mWriteQueue.push_back(bytes);

            if (!is_writing) {
                asyncWriteSerial();
            }

            res->success = true; // TODO: what is fail condition here? service no longer performs serial write
        }

        auto startAsyncReadThread() -> void {
            std::thread([this]() {
                asyncReadSerial();
                io.run();
            }).detach();
        }

        auto asyncReadSerial() -> void {
            boost::asio::async_read(mSerial, boost::asio::buffer(mBuffer), boost::asio::transfer_exactly(SERIAL_INPUT_MSG_SIZE),
                                    [this](boost::system::error_code const& ec, std::size_t len) {
                                        if (ec) {
                                            RCLCPP_ERROR(this->get_logger(), "Serial read failed: %s", ec.message().c_str());
                                        } else if (len != SERIAL_INPUT_MSG_SIZE) {
                                            RCLCPP_ERROR(this->get_logger(), "Failed to read whole serial buffer");
                                        } else {
                                            parseAndPublishBuffer(mBuffer);
                                        }
                                        // continue reads
                                        asyncReadSerial();
                                    });
        }

        void asyncWriteSerial() {
            boost::asio::async_write(mSerial, boost::asio::buffer(mWriteQueue.front()), [this](boost::system::error_code const& ec, std::size_t) {
                if (ec) {
                    RCLCPP_ERROR(this->get_logger(), "Serial write failed: %s", ec.message().c_str());
                }
                mWriteQueue.pop_front();
                if (!mWriteQueue.empty()) {
                    asyncWriteSerial();
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
                auto const dxl_id = static_cast<uint8_t>(*data);
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
            position_msg.name = {"gear_diff"};
            position_msg.position = {raw_pos_rad};

            mServoPositionPub->publish(position_msg);
            mTemperatureDataPub->publish(temp_msg);
            mHumidityDataPub->publish(humidity_msg);
        }

    public:
        explicit SAHWBridge(boost::asio::io_service& io) : rclcpp::Node{"sa_hw_bridge"}, mSerial(io), io(io), mBuffer(SERIAL_INPUT_MSG_SIZE) {
            // all initialization is done in the init() function to allow for the usage of shared_from_this()
        }

        auto init() -> void {
            mSerialPort = this->declare_parameter<std::string>("port_unicore", "/dev/arduino");
            mSerialBaudRate = this->declare_parameter<int>("baud_unicore", 115200);
            mServoID = this->declare_parameter<std::uint8_t>("servo_id", 0);

            for (auto const& name: mMotorNames) {
                mMotors[name] = std::make_shared<BrushedController>(shared_from_this(), "jetson", name);
            }

            mThrottleSub = create_subscription<msg::Throttle>("sa_throttle_cmd", 1, [this](msg::Throttle::ConstSharedPtr const& msg) { processThrottleCmd(msg); });

            mJointDataPub = create_publisher<sensor_msgs::msg::JointState>("sa_joint_data", 1);
            mControllerStatePub = create_publisher<msg::ControllerState>("sa_controller_state", 1);
            mTemperatureDataPub = create_publisher<sensor_msgs::msg::Temperature>("sa_temp_data", 1);
            mHumidityDataPub = create_publisher<sensor_msgs::msg::RelativeHumidity>("sa_humidity_data", 1);
            mServoPositionPub = create_publisher<msg::Position>("sa_gear_diff_position", 1);

            mPublishDataTimer = create_wall_timer(
                    std::chrono::milliseconds(100),
                    [this]() { publishDataCallback(); });

            mJointData.name = mMotorNames;
            mJointData.position.resize(mMotorNames.size());
            mJointData.velocity.resize(mMotorNames.size());
            mJointData.effort.resize(mMotorNames.size());

            mControllerState.name = mMotorNames;
            mControllerState.state.resize(mMotorNames.size());
            mControllerState.error.resize(mMotorNames.size());
            mControllerState.limit_hit.resize(mMotorNames.size());


            mSetServoPositionSrv = create_service<srv::ServoSetPos>(
                    "sa_gear_diff_set_position",
                    [this](srv::ServoSetPos::Request::ConstSharedPtr const req, srv::ServoSetPos::Response::SharedPtr res) {
                        setServoPositionServiceCallback(req, res);
                    });

            // configure serial io
            boost::system::error_code ec;
            mSerial.open(mSerialPort, ec);
            if (ec) {
                RCLCPP_FATAL(this->get_logger(), "Couldn't open serial port: %s", ec.message().c_str());
            } else {
                mSerial.set_option(boost::asio::serial_port_base::baud_rate(mSerialBaudRate));
                // begin serial reads on a separate thread
                startAsyncReadThread();
            }
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


auto main(int const argc, char** argv) -> int {
    rclcpp::init(argc, argv);

    boost::asio::io_service io;
    auto sa_hw_bridge = std::make_shared<mrover::SAHWBridge>(io);

    sa_hw_bridge->init();
    rclcpp::spin(sa_hw_bridge);

    sa_hw_bridge->stop();
    rclcpp::shutdown();
    return EXIT_SUCCESS;
}
