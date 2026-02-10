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
        struct cmd_t {
            geometry_msgs::msg::Twist msg;
            rclcpp::Time stamp;
            bool valid{false};
        };

        static constexpr RadiansPerSecond MAX_SPEED{70.0};
        static constexpr auto TIMEOUT = std::chrono::milliseconds(200);

        std::vector<std::string> const NAMES{"front_left", "middle_left", "back_left", "front_right", "middle_right", "back_right"};

        cmd_t mJoystick, mController, mNavigation;
        rclcpp::Publisher<msg::Velocity>::SharedPtr mVelocityPub;
        rclcpp::Subscription<geometry_msgs::msg::Twist>::SharedPtr mJoystickSub, mControllerSub, mNavigationSub;
        rclcpp::TimerBase::SharedPtr mTimer;

        auto joystickCallback(geometry_msgs::msg::Twist::ConstSharedPtr const& msg) -> void {
            mJoystick = {*msg, now(), true};
        }

        auto controllerCallback(geometry_msgs::msg::Twist::ConstSharedPtr const& msg) -> void {
            mController = {*msg, now(), true};
        }

        auto navigationCallback(geometry_msgs::msg::Twist::ConstSharedPtr const& msg) -> void {
            mNavigation = {*msg, now(), true};
        }

        void update() {
            auto is_fresh = [&](cmd_t const& c) {
                return c.valid && (now() - c.stamp) < TIMEOUT;
            };

            if (is_fresh(mJoystick)) {
                moveDrive(std::make_shared<geometry_msgs::msg::Twist>(mJoystick.msg));
            } else if (is_fresh(mController)) {
                moveDrive(std::make_shared<geometry_msgs::msg::Twist>(mController.msg));
            } else if (is_fresh(mNavigation)) {
                moveDrive(std::make_shared<geometry_msgs::msg::Twist>(mNavigation.msg));
            }
        }

        auto moveDrive(geometry_msgs::msg::Twist::ConstSharedPtr const& msg) const -> void {
            // See 13.3.1.4 in "Modern Robotics" for the math
            // Link: https://hades.mech.northwestern.edu/images/7/7f/MR.pdf

            Meters const rover_width{get_parameter("rover.width").as_double()};
            Meters const rover_length{get_parameter("rover.length").as_double()};
            Meters const wheel_radius{get_parameter("rover.wheel_radius").as_double()};

            compound_unit<Radians, inverse<Meters>> const wheel_linear_to_angular = Radians{1} / wheel_radius;

            MetersPerSecond const forward{msg->linear.x};
            RadiansPerSecond const angular{msg->angular.z};

            Meters const wheel_distance_inner = rover_width / 2;
            Meters const wheel_distance_outer = sqrt(square(rover_width / 2) + square(rover_length / 2));

            // The outer wheel needs to cover more ground than the inner wheel, so spin at a higher angular velocity
            MetersPerSecond const delta_inner = angular / Radians{1} * wheel_distance_inner;
            MetersPerSecond const delta_outer = angular / Radians{1} * wheel_distance_outer;

            RadiansPerSecond left_inner = (forward - delta_inner) * wheel_linear_to_angular;
            RadiansPerSecond right_inner = (forward + delta_inner) * wheel_linear_to_angular;
            RadiansPerSecond left_outer = (forward - delta_outer) * wheel_linear_to_angular;
            RadiansPerSecond right_outer = (forward + delta_outer) * wheel_linear_to_angular;

            // It is possible for another node to request an invalid combination of linear and angular velocities that the rover can not realize
            // Instead of clipping, scale down based on the maximal speed to preserve the ratio
            if (RadiansPerSecond const maximal = std::max(abs(left_outer), abs(right_outer)); maximal > MAX_SPEED) {
                Dimensionless const change_ratio = MAX_SPEED / maximal;
                left_inner = left_inner * change_ratio;
                right_inner = right_inner * change_ratio;
                left_outer = left_outer * change_ratio;
                right_outer = right_outer * change_ratio;
            }

            msg::Velocity velocity;
            velocity.names = NAMES;
            velocity.velocities = {left_outer.get(), left_inner.get(), left_outer.get(), right_outer.get(), right_inner.get(), right_outer.get()};
            mVelocityPub->publish(velocity);
        }

    public:
        DifferentialDriveController() : Node{"differential_drive_controller"} {
            using std::placeholders::_1;

            declare_parameter("rover.width", rclcpp::ParameterType::PARAMETER_DOUBLE);
            declare_parameter("rover.length", rclcpp::ParameterType::PARAMETER_DOUBLE);
            declare_parameter("rover.wheel_radius", rclcpp::ParameterType::PARAMETER_DOUBLE);

            mVelocityPub = create_publisher<msg::Velocity>("drive_velocity_cmd", 10);

            mJoystickSub = create_subscription<geometry_msgs::msg::Twist>("/joystick_vel_cmd", 10, std::bind(&DifferentialDriveController::joystickCallback, this, _1));
            mControllerSub = create_subscription<geometry_msgs::msg::Twist>("/controller_vel_cmd", 10, std::bind(&DifferentialDriveController::controllerCallback, this, _1));
            mNavigationSub = create_subscription<geometry_msgs::msg::Twist>("/nav_vel_cmd", 10, std::bind(&DifferentialDriveController::navigationCallback, this, _1));

            mTimer = create_wall_timer(std::chrono::milliseconds(20), std::bind(&DifferentialDriveController::update, this));
        }
    };

} // namespace mrover


auto main(int const argc, char** argv) -> int {
    rclcpp::init(argc, argv);
    rclcpp::spin(std::make_shared<mrover::DifferentialDriveController>());
    rclcpp::shutdown();
    return EXIT_SUCCESS;
}
