#include <chrono>
#include <memory>
#include <string>
#include <unordered_map>
#include <vector>

#include <rclcpp/rclcpp.hpp>

#include <units.hpp>

#include <mrover/msg/gimbal_control_state.hpp>
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

        void create_servo(uint8_t id, const std::string& name)
        {
            servos.insert({name, std::make_unique<mrover::Servo>(id, name)});
        }

        auto init() -> void {

            Servo::init("/dev/ttyUSB0");

            for (auto const& servo : mServoNames) {
                create_servo(servo.second, servo.first);
            }

            getPositionService = this->create_service<mrover::srv::ServoPosition>("get_position", [this](
                                                                                                             mrover::srv::ServoPosition::Request::SharedPtr const& request,
                                                                                                              mrover::srv::ServoPosition::Response::SharedPtr const& response) {

                const auto timeout = std::chrono::seconds(3);
                const auto start = this->get_clock()->now();                                                                                 
                Servo::ServoStatus status = servos.at(request->name)->setPosition(request->position, Servo::ServoMode::Optimal);

                while(status == Servo::ServoStatus::Active){
                    status = servos.at(request->name)->getTargetStatus();
                    if(this->get_clock()->now() - start > timeout){
                        RCLCPP_WARN(this->get_logger(), "Timeout reached while waiting for servo to reach target position");
                        break;
                    }
                }
                response->at_tgt = (status == Servo::ServoStatus::Success);

            });

            mGimbalStatePub = this->create_publisher<mrover::msg::GimbalControlState>("gimbal_control_state", 10);

            mPublishDataTimer = this->create_wall_timer(std::chrono::milliseconds(100),
                          std::bind(&MastGimbalHWBridge::publishDataCallback, this));

        }

    private:

        rclcpp::Service<mrover::srv::ServoPosition>::SharedPtr getPositionService;

        std::unordered_map<std::string, std::unique_ptr<mrover::Servo>> servos;

        std::unique_ptr<mrover::Servo> servo;

        std::vector<std::pair<std::string, int>> const mServoNames = {{"mast_gimbal_pitch", 1}, {"mast_gimbal_yaw", 2}};

        rclcpp::Publisher<mrover::msg::GimbalControlState>::SharedPtr mGimbalStatePub;
        rclcpp::TimerBase::SharedPtr mPublishDataTimer;
        msg::GimbalControlState mControllerState;
        
        auto publishDataCallback() -> void {
            const size_t n = mServoNames.size();

            mControllerState.name.resize(n);
            mControllerState.position.resize(n);
            mControllerState.velocity.resize(n);
            mControllerState.current.resize(n);
            mControllerState.error.resize(n);
            mControllerState.state.resize(n);
            mControllerState.limit_hit.resize(n);
            
            for (size_t i = 0; i < n; ++i) {
                const auto& name = mServoNames[i].first;
                auto& servo = *servos.at(name);

                mControllerState.name[i] = name;

                float pos = 0.0f;
                float vel = 0.0f;
                float cur = 0.0f;

                Servo::ServoStatus ps = servo.getPosition(pos);
                Servo::ServoStatus vs = servo.getVelocity(vel);
                Servo::ServoStatus cs = servo.getCurrent(cur);

                mControllerState.position[i] = pos;
                mControllerState.velocity[i] = vel;
                mControllerState.current[i]  = cur;

                Servo::ServoStatus ts = servo.getTargetStatus();

                /*mControllerState.state[i] = {servo->getState()};*/
                if (ts == Servo::ServoStatus::Active) {
                    mControllerState.state[i] = "active";
                } else if (ts == Servo::ServoStatus::Success) {
                    mControllerState.state[i] = "success";
                } else {
                    mControllerState.state[i] = "error";
                }      

                /*mControllerState.error[i] = {servo->getErrorState()};*/
                mControllerState.error[i] = "";
                if (ps != Servo::ServoStatus::Success) {
                    mControllerState.error[i] = "pos_read_fail";
                } else if (vs != Servo::ServoStatus::Success) {
                    mControllerState.error[i] = "vel_read_fail";
                } else if (cs != Servo::ServoStatus::Success) {
                    mControllerState.error[i] = "cur_read_fail";
                } else if (ts == Servo::ServoStatus::HardwareFailure) {
                    mControllerState.error[i] = "hardware_failure";
                }

                /*mControllerState.limit_hit[i] = {servo->getLimitsHitBits()};*/
            }

            mGimbalStatePub->publish(mControllerState);
        }
    };// namespace mrover
}

auto main(int argc, char** argv) -> int {
    rclcpp::init(argc, argv);
    auto mast_gimbal_hw_bridge = std::make_shared<mrover::MastGimbalHWBridge>();
    mast_gimbal_hw_bridge->init();
    rclcpp::spin(mast_gimbal_hw_bridge);
    rclcpp::shutdown();
    return EXIT_SUCCESS;
}
