#include <chrono>
#include <memory>

#include <rclcpp/rclcpp.hpp>
#include <rclcpp/subscription.hpp>
#include <rclcpp/timer.hpp>

#include <units.hpp>

#include <mrover/msg/controller_state.hpp>
#include <mrover/msg/position.hpp>
#include <mrover/msg/throttle.hpp>
#include <mrover/msg/velocity.hpp>

#include "motor_library/brushless.hpp"

/*
 *  Initializes the necessary motor controllers for drive
 */

namespace mrover {

    using namespace std::chrono_literals;

    class MotorTestBridge final : public rclcpp::Node {
    public:
        MotorTestBridge() : Node{"drive_hw_bridge"} {
            // all initialization is done in the init() function to allow for the usage of shared_from_this()
        }

        auto init() -> void {
            // Create publishers and subscribers for left and right motor groups
            for (std::string const& group: mMotorGroups) {
                // TODO (ali): does this actually emplace lol
                m_throttle_subs.emplace_back(create_subscription<msg::Throttle>(std::format("drive_{}_throttle_cmd", group), 1, [this](msg::Throttle::ConstSharedPtr const& msg) {
                    process_throttle_cmd(msg);
                }));
                m_velocity_subs.emplace_back(create_subscription<msg::Velocity>(std::format("drive_{}_velocity_cmd", group), 1, [this](msg::Velocity::ConstSharedPtr const& msg) {
                    process_velocity_cmd(msg);
                }));
                m_position_subs.emplace_back(create_subscription<msg::Position>(std::format("drive_{}_position_cmd", group), 1, [this](msg::Position::ConstSharedPtr const& msg) {
                    process_position_cmd(msg);
                }));
                for (std::string const& motor: mMotors) {
                    std::string name = std::format("{}_{}", motor, group);
                    m_controllers.try_emplace(name, shared_from_this(), "jetson", name);
                    m_joint_state.name.push_back(name);
                    m_controller_state.name.push_back(name);
                }

                m_joint_state_pubs.emplace_back(create_publisher<sensor_msgs::msg::JointState>(std::format("drive_{}_joint_data", group), 1));
                m_controller_state_pubs.emplace_back(create_publisher<msg::ControllerState>(std::format("drive_{}_controller_state", group), 1));
            }
            m_joint_state.position.resize(m_controllers.size());
            m_joint_state.velocity.resize(m_controllers.size());
            m_joint_state.effort.resize(m_controllers.size());

            m_controller_state.state.resize(m_controllers.size());
            m_controller_state.error.resize(m_controllers.size());
            m_controller_state.limit_hit.resize(m_controllers.size());

            // Periodic timer for published motor states
            m_publish_state_timer = create_wall_timer(std::chrono::milliseconds(100), [this]() { publish_state_callback(); });
        }

        auto getController(std::string const& name) -> BrushlessController<Revolutions>& {
            return m_controllers.at(name);
        }

        auto process_throttle_cmd(msg::Throttle::ConstPtr const& msg) -> void {
            for (size_t i = 0; i < msg->names.size(); ++i) {
                std::string const& name = msg->names[i];
                float const& throttle = msg->throttles[i];
                if (!m_controllers.contains(name)) {
                    RCLCPP_ERROR(get_logger(), "Group throttle request for %s ignored (%f)!", name.c_str(), msg->throttles[i]);
                    continue;
                }
                BrushlessController<Revolutions>& controller = getController(name);
                if (msg->names.at(i) != controller.getControllerName()) {
                    RCLCPP_ERROR(get_logger(), "Throttle request at topic for %s ignored!", msg->names.at(0).c_str());
                    continue;
                }
                controller.setDesiredThrottle(throttle);
            }
        }

        auto process_velocity_cmd(msg::Velocity::ConstPtr const& msg) -> void {
            for (size_t i = 0; i < msg->names.size(); ++i) {
                std::string const& name = msg->names[i];
                float const& velocity = msg->velocities[i];
                if (!m_controllers.contains(name)) {
                    RCLCPP_ERROR(get_logger(), "Velocity request for %s ignored!", name.c_str());
                    continue;
                }
                BrushlessController<Revolutions>& controller = getController(name);
                if (msg->names.at(i) != controller.getControllerName()) {
                    RCLCPP_ERROR(get_logger(), "Velocity request at topic for %s ignored!", msg->names.at(0).c_str());
                    continue;
                }
                controller.setDesiredVelocity(RadiansPerSecond{velocity});
            }
        }

        auto process_position_cmd(msg::Position::ConstPtr const& msg) -> void {
            // TODO - if any of the motor positions are invalid, then u should cancel the message.
            for (std::size_t i = 0; i < msg->names.size(); ++i) {
                std::string const& name = msg->names[i];
                float const& position = msg->positions[i];
                if (!m_controllers.contains(name)) {
                    RCLCPP_ERROR(get_logger(), "Position request for %s ignored!", name.c_str());
                    continue;
                }
                BrushlessController<Revolutions>& controller = getController(name);
                if (msg->names.at(i) != controller.getControllerName()) {
                    RCLCPP_ERROR(get_logger(), "Position request at topic for %s ignored!", msg->names.at(0).c_str());
                    continue;
                }
                controller.setDesiredPosition(Radians{position});
            }
        }

        auto publish_state_callback() -> void {
            m_joint_state.header.stamp = get_clock()->now();
            for (size_t i = 0; i < m_joint_state.name.size(); ++i) {
                BrushlessController<Revolutions>& controller = getController(m_joint_state.name[i]);
                m_joint_state.position[i] = controller.getPosition().get();
                m_joint_state.velocity[i] = controller.getVelocity().get();
                m_joint_state.effort[i] = controller.getEffort();

                m_controller_state.state[i] = controller.getState();
                m_controller_state.error[i] = controller.getErrorState();
                m_controller_state.limit_hit[i] = controller.getLimitsHitBits();
            }
            for (auto const& jointPub: m_joint_state_pubs) {
                jointPub->publish(m_joint_state);
            }
            for (auto const& statePub: m_controller_state_pubs) {
                statePub->publish(m_controller_state);
            }
        }

    private:
        std::vector<std::string> mMotorGroups = {"left", "right"};
        std::vector<std::string> mMotors = {"front", "middle", "back"};
        std::unordered_map<std::string, BrushlessController<Revolutions>> m_controllers;

        std::vector<rclcpp::Subscription<msg::Throttle>::SharedPtr> m_throttle_subs;
        std::vector<rclcpp::Subscription<msg::Velocity>::SharedPtr> m_velocity_subs;
        std::vector<rclcpp::Subscription<msg::Position>::SharedPtr> m_position_subs;

        rclcpp::TimerBase::SharedPtr m_publish_state_timer;

        std::vector<rclcpp::Publisher<sensor_msgs::msg::JointState>::SharedPtr> m_joint_state_pubs;
        std::vector<rclcpp::Publisher<msg::ControllerState>::SharedPtr> m_controller_state_pubs;

        sensor_msgs::msg::JointState m_joint_state;
        msg::ControllerState m_controller_state;
    };

} // namespace mrover

auto main(int argc, char** argv) -> int {
    rclcpp::init(argc, argv);
    auto drive_bridge = std::make_shared<mrover::MotorTestBridge>();
    drive_bridge->init();
    rclcpp::spin(drive_bridge);
    rclcpp::shutdown();
    return EXIT_SUCCESS;
}
