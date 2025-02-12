#include <chrono>
#include <fstream>
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

} // namespace mrover

auto main(int argc, char** argv) -> int {
    rclcpp::init(argc, argv);

    auto node = rclcpp::Node::make_shared("bm_test_bridge");
    auto jointB = std::make_shared<mrover::BrushedController>(node, "jetson", "joint_b", mrover::JOINT_B_CONFIG);

    rclcpp::Rate loop_rate(10);
    auto throttle = mrover::Percent{0.5};
    auto rate = mrover::Percent{0.1};

    std::ofstream f_handle("data/motor_encoder_data.csv");
    f_handle << "time,throttle,pos,vel" << std::endl;

    auto timer = node->create_wall_timer(std::chrono::seconds(2), [&]() {
        if (throttle >= mrover::Percent{1.0} || throttle <= mrover::Percent{-1.0}) {
            rate *= -1;
        }
        throttle += rate;
        RCLCPP_INFO(node->get_logger(), "RATE UPDATED TO %f", throttle.get());
    });

    auto start = node->now();

    while (rclcpp::ok()) {
        jointB->setDesiredThrottle(0.5);

        auto pos = jointB->getPosition().get();
        auto vel = jointB->getVelocity().get();
        RCLCPP_INFO(node->get_logger(), "joint_b    --- pos: %f | vel: %f", pos, vel);

        auto now = node->now() - start;
        // RCLCPP_INFO(node->get_logger(), "Uptime: %.2f seconds", now.seconds());
        // f_handle << now.seconds() << "," << throttle.get() << "," << pos << "," << vel << std::endl;

        rclcpp::spin_some(node);
        loop_rate.sleep();
    }

    rclcpp::shutdown();
    return EXIT_SUCCESS;
}
