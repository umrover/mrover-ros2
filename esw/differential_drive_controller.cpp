#include <algorithm>
#include <chrono>
#include <memory>
#include <string>

#include <geometry_msgs/msg/twist.hpp>

#include <rclcpp/rclcpp.hpp>

#include <units.hpp>

#include <mrover/msg/velocity.hpp>

/*
 *  Converts from a Twist (linear and angular velocity) to the individual wheel velocities.
 *  This is for a 6-wheel differential drive rover.
 */

namespace mrover {

    class DifferentialDriveController final : public rclcpp::Node {
        static constexpr RadiansPerSecond MAX_SPEED{70.0};

        std::vector<std::string> const NAMES{"front_left", "middle_left", "back_left", "front_right", "middle_right", "back_right"};

        rclcpp::Publisher<msg::Velocity>::SharedPtr mVelocityPub;
        rclcpp::Subscription<geometry_msgs::msg::Twist>::SharedPtr mCmdVelSub;
        rclcpp::TimerBase::SharedPtr mTimer;

        auto moveDrive(geometry_msgs::msg::Twist::ConstSharedPtr const& msg) const -> void {
            // See 13.3.1.4 in "Modern Robotics" for the math
            // Link: https://hades.mech.northwestern.edu/images/7/7f/MR.pdf

            Meters const roverWidth{get_parameter("rover.width").as_double()};
            Meters const roverLength{get_parameter("rover.length").as_double()};
            Meters const wheelRadius{get_parameter("rover.wheel_radius").as_double()};

            compound_unit<Radians, inverse<Meters>> const wheelLinearToAngular = Radians{1} / wheelRadius;

            MetersPerSecond const forward{msg->linear.x};
            RadiansPerSecond const angular{msg->angular.z};

            Meters const wheelDistanceInner = roverWidth / 2;
            Meters const wheelDistanceOuter = sqrt(square(roverWidth / 2) + square(roverLength / 2));

            // The outer wheel needs to cover more ground than the inner wheel, so spin at a higher angular velocity
            MetersPerSecond const deltaInner = angular / Radians{1} * wheelDistanceInner;
            MetersPerSecond const deltaOuter = angular / Radians{1} * wheelDistanceOuter;

            RadiansPerSecond leftInner = (forward - deltaInner) * wheelLinearToAngular;
            RadiansPerSecond rightInner = (forward + deltaInner) * wheelLinearToAngular;
            RadiansPerSecond leftOuter = (forward - deltaOuter) * wheelLinearToAngular;
            RadiansPerSecond rightOuter = (forward + deltaOuter) * wheelLinearToAngular;

            // It is possible for another node to request an invalid combination of linear and angular velocities that the rover can not realize
            // Instead of clipping, scale down based on the maximal speed to preserve the ratio
            if (RadiansPerSecond const maximal = std::max(abs(leftOuter), abs(rightOuter)); maximal > MAX_SPEED) {
                Dimensionless const changeRatio = MAX_SPEED / maximal;
                leftInner = leftInner * changeRatio;
                rightInner = rightInner * changeRatio;
                leftOuter = leftOuter * changeRatio;
                rightOuter = rightOuter * changeRatio;
            }

            msg::Velocity velocity;
            velocity.names = NAMES;
            velocity.velocities = {leftOuter.get(), leftInner.get(), leftOuter.get(), rightOuter.get(), rightInner.get(), rightOuter.get()};
            mVelocityPub->publish(velocity);
        }

    public:
        DifferentialDriveController() : Node{"differential_drive_controller"} {
            declare_parameter("rover.width", rclcpp::ParameterType::PARAMETER_DOUBLE);
            declare_parameter("rover.length", rclcpp::ParameterType::PARAMETER_DOUBLE);
            declare_parameter("rover.wheel_radius", rclcpp::ParameterType::PARAMETER_DOUBLE);

            mVelocityPub = create_publisher<msg::Velocity>("drive_velocity_cmd", 1);

            mCmdVelSub = create_subscription<geometry_msgs::msg::Twist>("/cmd_vel", 1, [this](geometry_msgs::msg::Twist::ConstSharedPtr const& msg) { moveDrive(msg); });
        }
    };

} // namespace mrover


auto main(int const argc, char** argv) -> int {
    rclcpp::init(argc, argv);
    rclcpp::spin(std::make_shared<mrover::DifferentialDriveController>());
    rclcpp::shutdown();
    return EXIT_SUCCESS;
}
