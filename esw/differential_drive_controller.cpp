#include <algorithm>
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

    std::vector<std::string> LEFT_NAMES{"front_left", "middle_left", "back_left"};
    std::vector<std::string> RIGHT_NAMES{"front_right", "middle_right", "back_right"};

    class DifferentialDriveController final : public rclcpp::Node {
    public:
        DifferentialDriveController() : Node{"differential_drive_controller"} {
            declare_parameter("rover.width", rclcpp::ParameterType::PARAMETER_DOUBLE);
            declare_parameter("rover.length", rclcpp::ParameterType::PARAMETER_DOUBLE);
            declare_parameter("rover.wheel_radius", rclcpp::ParameterType::PARAMETER_DOUBLE);

            leftVelocityPub = create_publisher<msg::Velocity>("drive_left_velocity_cmd", 10);
            rightVelocityPub = create_publisher<msg::Velocity>("drive_right_velocity_cmd", 10);

            twistSub = create_subscription<geometry_msgs::msg::Twist>("cmd_vel", 10, [this](geometry_msgs::msg::Twist::ConstSharedPtr const& msg) {
                moveDrive(msg);
            });
        }

    private:
        auto moveDrive(geometry_msgs::msg::Twist::ConstSharedPtr const& msg) -> void {
            // See 13.3.1.4 in "Modern Robotics" for the math
            // Link: https://hades.mech.northwestern.edu/images/7/7f/MR.pdf

            Meters roverWidth{get_parameter("rover.width").as_double()};
            Meters roverLength{get_parameter("rover.length").as_double()};
            Meters wheelRadius{get_parameter("rover.wheel_radius").as_double()};

            compound_unit<Radians, inverse<Meters>> wheelLinearToAngular = Radians{1} / wheelRadius;

            MetersPerSecond forward{msg->linear.x};
            RadiansPerSecond angular{msg->angular.z};

            Meters wheelDistanceInner = roverWidth / 2;
            Meters wheelDistanceOuter = sqrt(square(roverWidth / 2) + square(roverLength / 2));

            // TODO(quintin):   Don't ask me to explain perfectly why we need to cancel out a meters unit in the numerator
            //                  I think it comes from the fact that there is a unit vector in the denominator of the equation

            // The outer wheel needs to cover more ground than the inner wheel, so spin at a higher angular velocity

            MetersPerSecond deltaInner = angular / Radians{1} * wheelDistanceInner;
            MetersPerSecond deltaOuter = angular / Radians{1} * wheelDistanceOuter;

            RadiansPerSecond leftInner = -1 * (forward - deltaInner) * wheelLinearToAngular;
            RadiansPerSecond rightInner = -1 * (forward + deltaInner) * wheelLinearToAngular;
            RadiansPerSecond leftOuter = -1 * (forward - deltaOuter) * wheelLinearToAngular;
            RadiansPerSecond rightOuter = -1 * (forward + deltaOuter) * wheelLinearToAngular;

            // It is possible for another node to request an invalid combination of linear and angular velocities that the rover can not realize
            // Instead of clipping, scale down based on the maximal speed to preserve the ratio

            RadiansPerSecond MAX_SPEED{70.0};

            if (RadiansPerSecond maximal = std::max(abs(leftOuter), abs(rightOuter)); maximal > MAX_SPEED) {
                Dimensionless changeRatio = MAX_SPEED / maximal;
                leftInner = leftInner * changeRatio;
                rightInner = rightInner * changeRatio;
                leftOuter = leftOuter * changeRatio;
                rightOuter = rightOuter * changeRatio;
            }

            {
                msg::Velocity leftVelocity;
                leftVelocity.names = LEFT_NAMES;
                leftVelocity.velocities = {leftOuter.get(), leftInner.get(), leftOuter.get()};
                leftVelocityPub->publish(leftVelocity);
            }
            {
                msg::Velocity rightVelocity;
                rightVelocity.names = RIGHT_NAMES;
                rightVelocity.velocities = {rightOuter.get(), rightInner.get(), rightOuter.get()};
                rightVelocityPub->publish(rightVelocity);
            }
        }

        rclcpp::Publisher<msg::Velocity>::SharedPtr leftVelocityPub, rightVelocityPub;
        rclcpp::Subscription<geometry_msgs::msg::Twist>::SharedPtr twistSub;
    };

} // namespace mrover

auto main(int argc, char** argv) -> int {
    rclcpp::init(argc, argv);
    rclcpp::spin(std::make_shared<mrover::DifferentialDriveController>());
    rclcpp::shutdown();
    return EXIT_SUCCESS;
}
