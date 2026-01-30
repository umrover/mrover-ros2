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

        std::vector<std::string> const LEFT_NAMES{"front_left", "middle_left", "back_left"};
        std::vector<std::string> const RIGHT_NAMES{"front_right", "middle_right", "back_right"};

        cmd_t m_joy, m_ctrl, m_nav;
        rclcpp::Publisher<msg::Velocity>::SharedPtr m_left_velocity_pub, m_right_velocity_pub;
        rclcpp::Subscription<geometry_msgs::msg::Twist>::SharedPtr m_joy_sub, m_ctrl_sub, m_nav_sub;
        rclcpp::TimerBase::SharedPtr m_timer;

        auto joy_cb(geometry_msgs::msg::Twist::ConstSharedPtr const& msg) -> void {
            m_joy = {*msg, now(), true};
        }

        auto ctrl_cb(geometry_msgs::msg::Twist::ConstSharedPtr const& msg) -> void {
            m_ctrl = {*msg, now(), true};
        }

        auto nav_cb(geometry_msgs::msg::Twist::ConstSharedPtr const& msg) -> void {
            m_nav = {*msg, now(), true};
        }

        void update() {
            auto is_fresh = [&](cmd_t const& c) {
                return c.valid && (now() - c.stamp) < TIMEOUT;
            };

            if (is_fresh(m_joy)) {
                move_drive(std::make_shared<geometry_msgs::msg::Twist>(m_joy.msg));
            } else if (is_fresh(m_ctrl)) {
                move_drive(std::make_shared<geometry_msgs::msg::Twist>(m_ctrl.msg));
            } else if (is_fresh(m_nav)) {
                move_drive(std::make_shared<geometry_msgs::msg::Twist>(m_nav.msg));
            }
        }

        auto move_drive(geometry_msgs::msg::Twist::ConstSharedPtr const& msg) const -> void {
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

            {
                msg::Velocity left_velocity;
                left_velocity.name = LEFT_NAMES;
                left_velocity.velocity = {left_outer.get(), left_inner.get(), left_outer.get()};
                m_left_velocity_pub->publish(left_velocity);
            }
            {
                msg::Velocity right_velocity;
                right_velocity.name = RIGHT_NAMES;
                right_velocity.velocity = {right_outer.get(), right_inner.get(), right_outer.get()};
                m_right_velocity_pub->publish(right_velocity);
            }
        }

    public:
        DifferentialDriveController() : Node{"differential_drive_controller"} {
            using std::placeholders::_1;

            declare_parameter("rover.width", rclcpp::ParameterType::PARAMETER_DOUBLE);
            declare_parameter("rover.length", rclcpp::ParameterType::PARAMETER_DOUBLE);
            declare_parameter("rover.wheel_radius", rclcpp::ParameterType::PARAMETER_DOUBLE);

            m_left_velocity_pub = create_publisher<msg::Velocity>("drive_left_velocity_cmd", 10);
            m_right_velocity_pub = create_publisher<msg::Velocity>("drive_right_velocity_cmd", 10);

            m_joy_sub = create_subscription<geometry_msgs::msg::Twist>("/joystick_vel_cmd", 10, std::bind(&DifferentialDriveController::joy_cb, this, _1));
            m_ctrl_sub = create_subscription<geometry_msgs::msg::Twist>("/controller_vel_cmd", 10, std::bind(&DifferentialDriveController::ctrl_cb, this, _1));
            m_nav_sub = create_subscription<geometry_msgs::msg::Twist>("/nav_vel_cmd", 10, std::bind(&DifferentialDriveController::nav_cb, this, _1));

            m_timer = create_wall_timer(std::chrono::milliseconds(20), std::bind(&DifferentialDriveController::update, this));
        }
    };

} // namespace mrover


auto main(int const argc, char** argv) -> int {
    rclcpp::init(argc, argv);
    rclcpp::spin(std::make_shared<mrover::DifferentialDriveController>());
    rclcpp::shutdown();
    return EXIT_SUCCESS;
}
