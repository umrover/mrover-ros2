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

#include "motor_library/brushed.hpp"

namespace mrover {

    class MastGimbalHWBridge : public rclcpp::Node {

    public:
        MastGimbalHWBridge() : rclcpp::Node{"mast_gimbal_hw_bridge"} {
            // all initialization is done in the init() function to allow for the usage of shared_from_this()
        }

        auto init() -> void {

            for (auto const& name: mMotorNames) {
                mMotors[name] = std::make_shared<BrushedController>(shared_from_this(), "jetson", name);
            }

            mThrottleSub = create_subscription<msg::Throttle>("mast_gimbal_throttle_cmd", 1, [this](msg::Throttle::ConstSharedPtr const& msg) { processThrottleCmd(msg); });

            mPublishDataTimer = create_wall_timer(
                    std::chrono::milliseconds(100),
                    [this]() { publishDataCallback(); });
            mJointDataPub = create_publisher<sensor_msgs::msg::JointState>("mast_gimbal_joint_data", 1);
            mControllerStatePub = create_publisher<msg::ControllerState>("mast_gimbal_controller_state", 1);

            mJointData.name = mMotorNames;
            mJointData.position.resize(mMotorNames.size());
            mJointData.velocity.resize(mMotorNames.size());
            mJointData.effort.resize(mMotorNames.size());

            mControllerState.names = mMotorNames;
            mControllerState.states.resize(mMotorNames.size());
            mControllerState.errors.resize(mMotorNames.size());
            mControllerState.limits_hit.resize(mMotorNames.size());
        }

    private:
        std::vector<std::string> const mMotorNames = {"mast_gimbal_pitch", "mast_gimbal_yaw"};
        std::unordered_map<std::string, std::shared_ptr<BrushedController>> mMotors;

        rclcpp::Subscription<msg::Throttle>::SharedPtr mThrottleSub;

        rclcpp::TimerBase::SharedPtr mPublishDataTimer;
        rclcpp::Publisher<sensor_msgs::msg::JointState>::SharedPtr mJointDataPub;
        rclcpp::Publisher<msg::ControllerState>::SharedPtr mControllerStatePub;
        sensor_msgs::msg::JointState mJointData;
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

                mControllerState.states[i] = {motor->getState()};
                mControllerState.errors[i] = {motor->getErrorState()};
                mControllerState.limits_hit[i] = {motor->getLimitsHitBits()};
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
