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
        }

    private:

        rclcpp::Service<mrover::srv::ServoPosition>::SharedPtr getPositionService;

        std::unordered_map<std::string, std::unique_ptr<mrover::Servo>> servos;

        std::unique_ptr<mrover::Servo> servo;


        std::vector<std::pair<std::string, int>> const mServoNames = {{"mast_gimbal_pitch", 1}, {"mast_gimbal_yaw", 2}};




        rclcpp::TimerBase::SharedPtr mPublishDataTimer;
        rclcpp::Publisher<sensor_msgs::msg::JointState>::SharedPtr mJointDataPub;
        rclcpp::Publisher<msg::ControllerState>::SharedPtr mControllerStatePub;

        msg::ControllerState mControllerState;


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
    };
} // namespace mrover

auto main(int argc, char** argv) -> int {
    rclcpp::init(argc, argv);
    auto mast_gimbal_hw_bridge = std::make_shared<mrover::MastGimbalHWBridge>();
    mast_gimbal_hw_bridge->init();
    rclcpp::spin(mast_gimbal_hw_bridge);
    rclcpp::shutdown();
    return EXIT_SUCCESS;
}
