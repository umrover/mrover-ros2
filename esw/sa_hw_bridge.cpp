#include <chrono>
#include <memory>
#include <string>
#include <unordered_map>
#include <vector>

#include <rclcpp/rclcpp.hpp>

#include <units.hpp>
#include <units_eigen.hpp>

#include <mrover/msg/controller_state.hpp>
#include <mrover/msg/position.hpp>
#include <mrover/msg/throttle.hpp>
#include <mrover/msg/velocity.hpp>
#include <mrover/srv/adjust_motor.hpp>
#include <sensor_msgs/msg/joint_state.hpp>

#include "motor_library/brushed.hpp"
#include "motor_library/brushless.hpp"

namespace mrover {

    class SAHWBridge : public rclcpp::Node {

        // using Controller = std::variant<BrushedController, BrushlessController<Meters>, BrushlessController<Revolutions>>;

    public:
        SAHWBridge() : rclcpp::Node{"sa_hw_bridge", rclcpp::NodeOptions{}
                                                              .allow_undeclared_parameters(true)
                                                              .automatically_declare_parameters_from_overrides(true)} {
            // all initialization is done in the init() function to allow for the usage of shared_from_this()
        }

        auto init() -> void {

            for (const auto& name : mMotorNames) {
                mMotors[name] = std::make_shared<BrushedController>(shared_from_this(), "jetson", name);
            }

            mSAThrottleSub = create_subscription<msg::Throttle>("sa_throttle_cmd", 1, [this](msg::Throttle::ConstSharedPtr const& msg) { processThrottleCmd(msg); });

            mPublishDataTimer = create_wall_timer(
                    std::chrono::milliseconds(100),
                    [this]() { publishDataCallback(); });
            mJointDataPub = create_publisher<sensor_msgs::msg::JointState>("sa_joint_data", 1);
            mControllerStatePub = create_publisher<msg::ControllerState>("sa_controller_state", 1);

            mJointData.name = mMotorNames;
            mJointData.position.resize(mMotorNames.size());
            mJointData.velocity.resize(mMotorNames.size());
            mJointData.effort.resize(mMotorNames.size());

            mControllerState.name = mMotorNames;
            mControllerState.state.resize(mMotorNames.size());
            mControllerState.error.resize(mMotorNames.size());
            mControllerState.limit_hit.resize(mMotorNames.size());
        }

    private:
        const std::vector<std::string> mMotorNames = {"linear_actuator", "auger", "pump_a", "pump_b", "sensor_actuator"};
        std::unordered_map<std::string, std::shared_ptr<BrushedController>> mMotors;

        rclcpp::Subscription<msg::Throttle>::SharedPtr mSAThrottleSub;

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

            for (size_t i = 0 ; i < mMotorNames.size() ; ++i) {
                const auto& name = mMotorNames[i];
                const auto& motor = mMotors[name];

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
    auto sa_hw_bridge = std::make_shared<mrover::SAHWBridge>();
    sa_hw_bridge->init();
    rclcpp::spin(sa_hw_bridge);
    rclcpp::shutdown();
    return EXIT_SUCCESS;
}
