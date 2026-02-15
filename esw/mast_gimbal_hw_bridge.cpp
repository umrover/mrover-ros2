#include <chrono>
#include <memory>
#include <string>
#include <unordered_map>
#include <vector>

#include <rclcpp/rclcpp.hpp>

#include <units.hpp>

#include <mrover/msg/controller_state.hpp>
#include <mrover/msg/position.hpp>
#include <mrover/msg/throttle.hpp>
#include <mrover/msg/velocity.hpp>
#include <mrover/srv/adjust_motor.hpp>
#include <sensor_msgs/msg/joint_state.hpp>

#include "mrover/srv/servo_position.hpp"

#include "servo_library/servo.hpp"

namespace mrover {

    class MastGimbalHWBridge : public rclcpp::Node {

    public:
        MastGimbalHWBridge() : rclcpp::Node{"mast_gimbal_hw_bridge"} {
            // all initialization is done in the init() function to allow for the usage of shared_from_this()
        }

        auto create_servo(uint8_t id, std::string const& name) -> void {
            servos.insert({name, std::make_shared<mrover::Servo>(shared_from_this(), id, name)});
        }

        static auto servos_active(std::vector<Servo::ServoStatus>& statuses) -> bool {
            bool active = false;
            for (auto& status: statuses) {
                if (status == Servo::ServoStatus::Active) {
                    active = true;
                }
            }
            return active;
        }

        auto init() -> void {

            Servo::init("/dev/ttyUSB0");

            for (auto const& servo: mServoNames) {
                create_servo(servo.second, servo.first);
            }

            getPositionService = this->create_service<mrover::srv::ServoPosition>("get_position", [this](
                                                                                                          mrover::srv::ServoPosition::Request::SharedPtr const& request,
                                                                                                          mrover::srv::ServoPosition::Response::SharedPtr const& response) {
                size_t const n = request->names.size();
                std::vector<Servo::ServoStatus> statuses = std::vector<Servo::ServoStatus>(n);

                auto const timeout = std::chrono::seconds(3);
                auto const start = this->get_clock()->now();

                for (size_t i = 0; i < n; i++) {
                    statuses[i] = servos.at(request->names[i])->setPosition(request->positions[i], Servo::ServoMode::Limited);
                }

                while (servos_active(statuses)) {
                    for (size_t i = 0; i < n; i++) {
                        statuses[i] = servos.at(request->names[i])->getTargetStatus();
                        response->at_tgts[i] = (statuses[i] == Servo::ServoStatus::Success);
                    }
                    if (this->get_clock()->now() - start > timeout) {
                        RCLCPP_WARN(this->get_logger(), "Timeout reached while waiting for servo to reach target position");
                        break;
                    }
                }
            });

            mGimbalStatePub = this->create_publisher<mrover::msg::ControllerState>("controller_state", 10);

            mPublishDataTimer = this->create_wall_timer(std::chrono::milliseconds(100),
                                                        [&]() { return MastGimbalHWBridge::publishDataCallback(); });
        }

    private:
        rclcpp::Service<mrover::srv::ServoPosition>::SharedPtr getPositionService;

        std::unordered_map<std::string, std::shared_ptr<mrover::Servo>> servos;

        std::vector<std::pair<std::string, int>> const mServoNames = {{"gimbal_pitch", 3}, {"gimbal_yaw", 4}};

        rclcpp::Publisher<mrover::msg::ControllerState>::SharedPtr mGimbalStatePub;
        rclcpp::TimerBase::SharedPtr mPublishDataTimer;
        msg::ControllerState mControllerState;

        auto publishDataCallback() -> void {
            size_t const n = mServoNames.size();

            mControllerState.names.resize(n);
            mControllerState.positions.resize(n);
            mControllerState.velocities.resize(n);
            mControllerState.currents.resize(n);
            mControllerState.errors.resize(n);
            mControllerState.states.resize(n);
            mControllerState.limits_hit.resize(n);

            for (size_t i = 0; i < n; ++i) {
                auto const& name = mServoNames[i].first;
                auto& servo = *servos.at(name);

                mControllerState.names[i] = name;
                mControllerState.limits_hit[i] = servo.getLimitStatus();

                float pos = 0.0f;
                float vel = 0.0f;
                float cur = 0.0f;

                Servo::ServoStatus err = servo.getPosition(pos);
                servo.getVelocity(vel);
                servo.getCurrent(cur);

                mControllerState.positions[i] = pos;
                mControllerState.velocities[i] = vel;
                mControllerState.currents[i] = cur;

                Servo::ServoStatus ts = servo.getTargetStatus();

                /*mControllerState.state[i] = {servo->getState()};*/
                if (ts == Servo::ServoStatus::Active) {
                    mControllerState.states[i] = "active";
                } else if (ts == Servo::ServoStatus::Success) {
                    mControllerState.states[i] = "success";
                } else {
                    mControllerState.states[i] = "error";
                }

                /*mControllerState.error[i] = {servo->getErrorState()};*/
                switch (err) {
                    case Servo::ServoStatus::CommNotAvailable:
                        mControllerState.errors[i] = "CommNotAvailable";
                        break;
                    case Servo::ServoStatus::CommTxError:
                        mControllerState.errors[i] = "CommTxError";
                        break;
                    case Servo::ServoStatus::HardwareFailure:
                        mControllerState.errors[i] = "HardwareFailure";
                        break;
                    case Servo::ServoStatus::CommRxCorrupt:
                        mControllerState.errors[i] = "CommRxCorrupt";
                        break;
                    case Servo::ServoStatus::CommRxTimeout:
                        mControllerState.errors[i] = "CommRxTimeout";
                        break;
                    case Servo::ServoStatus::CommRxFail:
                        mControllerState.errors[i] = "CommRxFail";
                        break;
                    case Servo::ServoStatus::CommTxFail:
                        mControllerState.errors[i] = "CommTxFail";
                        break;
                    case Servo::ServoStatus::CommPortBusy:
                        mControllerState.errors[i] = "CommPortBusy";
                        break;
                    case Servo::ServoStatus::CommRxWaiting:
                        mControllerState.errors[i] = "CommRxWaiting";
                        break;
                    case Servo::ServoStatus::FailedToOpenPort:
                        mControllerState.errors[i] = "FailedToOpenPort";
                        break;
                    case Servo::ServoStatus::FailedToSetBaud:
                        mControllerState.errors[i] = "FailedToSetBaud";
                        break;
                    default:
                        mControllerState.errors[i] = "NoError";
                        break;
                }

                /*mControllerState.limit_hit[i] = {servo->getLimitsHitBits()};*/
            }

            mGimbalStatePub->publish(mControllerState);
        }
    }; // namespace mrover
} // namespace mrover


auto main(int argc, char** argv) -> int {
    rclcpp::init(argc, argv);
    auto mast_gimbal_hw_bridge = std::make_shared<mrover::MastGimbalHWBridge>();
    mast_gimbal_hw_bridge->init();
    rclcpp::spin(mast_gimbal_hw_bridge);
    rclcpp::shutdown();
    return EXIT_SUCCESS;
}
