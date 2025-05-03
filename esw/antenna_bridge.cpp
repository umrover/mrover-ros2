#include <memory>
#include <string>

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

    class AntennaBridge : public rclcpp::Node {

    public:
        AntennaBridge() : rclcpp::Node{"antenna_bridge"} {
            // all initialization is done in the init() function to allow for the usage of shared_from_this()
        }

        auto init() -> void {
            mController = std::make_shared<BrushedController>(shared_from_this(), "rpi", antennaName);

            mThrottleSub = create_subscription<msg::Throttle>("antenna_throttle_cmd", 1, [this](msg::Throttle::ConstSharedPtr const& msg) { processThrottleCmd(msg); });

            mPublishDataTimer = create_wall_timer(
                    std::chrono::milliseconds(100),
                    [this]() { publishDataCallback(); });
            mJointDataPub = create_publisher<sensor_msgs::msg::JointState>("antenna_joint_data", 1);

            mJointData.name = {antennaName};
            mJointData.position = {std::numeric_limits<double>::quiet_NaN()};
            mJointData.velocity = {std::numeric_limits<double>::quiet_NaN()};
            mJointData.effort = {std::numeric_limits<double>::quiet_NaN()};
        }

    private:
        std::string const antennaName = "antenna";
        std::shared_ptr<BrushedController> mController;

        rclcpp::Subscription<msg::Throttle>::SharedPtr mThrottleSub;

        rclcpp::TimerBase::SharedPtr mPublishDataTimer;
        rclcpp::Publisher<sensor_msgs::msg::JointState>::SharedPtr mJointDataPub;
        sensor_msgs::msg::JointState mJointData;

        auto processThrottleCmd(msg::Throttle::ConstSharedPtr const& msg) -> void {
            if (msg->names.size() != 1 || msg->throttles.size() != 1) {
                RCLCPP_ERROR(get_logger(), "Both name and throttle should be of size 1 (just antenna)!");
                return;
            }

            std::string const& name = msg->names[0];
            if (name == antennaName) {
                Dimensionless const& throttle = msg->throttles[0];
                mController->setDesiredThrottle(throttle);
            }
        }

        auto publishDataCallback() -> void {
            mJointData.header.stamp = get_clock()->now();

            mJointData.position[0] = {mController->getPosition().get()};
            mJointData.velocity[0] = {mController->getVelocity().get()};
            mJointData.effort[0] = {mController->getEffort()};

            mJointDataPub->publish(mJointData);
        }
    };
} // namespace mrover

auto main(int argc, char** argv) -> int {
    rclcpp::init(argc, argv);
    auto antenna_bridge = std::make_shared<mrover::AntennaBridge>();
    antenna_bridge->init();
    rclcpp::spin(antenna_bridge);
    rclcpp::shutdown();
    return EXIT_SUCCESS;
}
