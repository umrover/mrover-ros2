#include <chrono>
#include <memory>
#include <rclcpp/node.hpp>
#include <rclcpp/rate.hpp>
#include <string>
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

    constexpr static BrushedController::Config JOINT_B_CONFIG = {
            .limitSwitchPresent = {true, false},
            .limitSwitchEnabled = {true, false},
            .limitSwitchLimitsFwd = {false, false},
            .limitSwitchActiveHigh = {false, false},
            .limitSwitchUsedForReadjustment = {false, false},
            // .limitSwitchReadjustPosition = {Radians{0.0}},
            .limitMaxForwardPosition = true,
            .limitMaxBackwardPosition = false,

            .gearRatio = 1.0,
            .isInverted = false,

            .driverVoltage = 10.5,
            .motorMaxVoltage = 12.0,

            .quadPresent = false,
            // .quadRatio = 1.0,

            .absPresent = true,
            .absRatio = -1.0,
            .absOffset = 2.68_rad,

            .minVelocity = -1.0_rad_per_s,
            .maxVelocity = 1.0_rad_per_s,
            .minPosition = -0.7853981633974483_rad,
            .maxPosition = 0_rad,

            .positionGains = {.p = 30.0},
            // .velocityGains{},

            .calibrationThrottle = 0.5,
    };

    constexpr static BrushedController::Config GRIPPER_CONFIG{};
    constexpr static BrushedController::Config FINGER_CONFIG{};

} // namespace mrover

auto main(int argc, char** argv) -> int {
    rclcpp::init(argc, argv);

    auto node = rclcpp::Node::make_shared("bm_test_bridge");
    auto jointB = std::make_shared<mrover::BrushedController>(node, "jetson", "joint_b", mrover::JOINT_B_CONFIG);
    auto gripper = std::make_shared<mrover::BrushedController>(node, "jetson", "gripper", mrover::GRIPPER_CONFIG);
    auto finger = std::make_shared<mrover::BrushedController>(node, "jetson", "finger", mrover::FINGER_CONFIG);

    auto throttle = mrover::Percent{0.5};
    auto rate = mrover::Percent{0.1};

    rclcpp::Rate loop_rate(10);

    while (rclcpp::ok()) {
        if (throttle >= mrover::Percent{1.0} || throttle <= mrover::Percent{-1.0}) {
            rate *= -1;
        }
        throttle += rate;
        jointB->setDesiredThrottle(throttle);
        gripper->setDesiredThrottle(throttle);
        finger->setDesiredThrottle(throttle);
        RCLCPP_INFO(node->get_logger(), "joint_b    --- pos: %f | vel: %f", jointB->getPosition().get(), jointB->getVelocity().get());

        rclcpp::spin_some(node);
        loop_rate.sleep();
    }

    rclcpp::shutdown();
    return EXIT_SUCCESS;
}
