#include <rclcpp/rclcpp.hpp>

#include <units/units.hpp>

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
            .absRatio = 1.0,
            .absOffset = 0.0_rad,

            .minVelocity = -1.0_rad_per_s,
            .maxVelocity = 1.0_rad_per_s,
            .minPosition = -0.7853981633974483_rad,
            .maxPosition = 0_rad,

            // .positionGains{},
            // .velocityGains{},

            .calibrationThrottle = 0.5,
    };
    constexpr static BrushedController::Config GRIPPER_CONFIG = {
            .gearRatio = 47.0,
            .isInverted = false,
            .driverVoltage = 10.5,
            .motorMaxVoltage = 12.0,
            .quadPresent = false,
            .absPresent = false,
            .calibrationThrottle = 0.5,
    };
} // namespace mrover

auto main(int argc, char** argv) -> int {
    rclcpp::init(argc, argv);
    auto joint_b = std::make_shared<mrover::BrushedController>("jetson", "joint_b", mrover::JOINT_B_CONFIG);
    auto gripper = std::make_shared<mrover::BrushedController>("jetson", "gripper", mrover::GRIPPER_CONFIG);
    rclcpp::spin(joint_b);
    rclcpp::shutdown();
    return EXIT_SUCCESS;
}
