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

#include "brushless.hpp"

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
            // publishers and subscribers for left and right motor groups
            for (std::string const& group: m_motor_groups) {
                m_velocity_sub = create_subscription<msg::Velocity>(std::format("drive_velocity_cmd", group), 1, [this](msg::Velocity::ConstSharedPtr const& msg) {
                    process_velocity_cmd(msg);
                });

                // emplace controller names
                for (std::string const& motor: m_motors) {
                    std::string name = std::format("{}_{}", motor, group);
                    m_controllers.try_emplace(name, shared_from_this(), "jetson", name);
                    m_controller_state.names.push_back(name);
                }

                // initialize outbound message
                m_controller_state.header.stamp = now();
                m_controller_state.header.frame_id = "";
                m_controller_state.names.resize(m_controllers.size());
                m_controller_state.states.resize(m_controllers.size());
                m_controller_state.errors.resize(m_controllers.size());
                m_controller_state.positions.resize(m_controllers.size());
                m_controller_state.velocities.resize(m_controllers.size());
                m_controller_state.currents.resize(m_controllers.size());
                m_controller_state.limits_hit.resize(m_controllers.size());

                m_controller_state_pubs.emplace_back(create_publisher<msg::ControllerState>(std::format("{}_controller_state", group), 1));
            }

            // periodic timer for published motor states
            m_publish_state_timer = create_wall_timer(std::chrono::milliseconds(100), [this]() { publish_state_callback(); });
        }

        auto get_controller(std::string const& name) -> BrushlessController<Revolutions>& {
            return m_controllers.at(name);
        }

        auto process_throttle_cmd(msg::Throttle::ConstSharedPtr const& msg) -> void {
            for (size_t i = 0; i < msg->names.size(); ++i) {
                std::string const& name = msg->names[i];
                float const& throttle = msg->throttles[i];
                if (!m_controllers.contains(name)) {
                    RCLCPP_ERROR(get_logger(), "Group throttle request for %s ignored (%f)!", name.c_str(), msg->throttles[i]);
                    continue;
                }
                BrushlessController<Revolutions>& controller = get_controller(name);
                if (msg->names.at(i) != controller.get_name()) {
                    RCLCPP_ERROR(get_logger(), "Throttle request at topic for %s ignored!", msg->names.at(0).c_str());
                    continue;
                }
                controller.set_desired_throttle(throttle);
            }
        }

        auto process_velocity_cmd(msg::Velocity::ConstSharedPtr const& msg) -> void {
            for (size_t i = 0; i < msg->names.size(); ++i) {
                std::string const& name = msg->names[i];
                float const& velocity = msg->velocities[i];
                if (!m_controllers.contains(name)) {
                    RCLCPP_ERROR(get_logger(), "Velocity request for %s ignored!", name.c_str());
                    continue;
                }
                BrushlessController<Revolutions>& controller = get_controller(name);
                if (msg->names.at(i) != controller.get_name()) {
                    RCLCPP_ERROR(get_logger(), "Velocity request at topic for %s ignored!", msg->names.at(0).c_str());
                    continue;
                }
                controller.set_desired_velocity(RadiansPerSecond{velocity});
            }
        }

        auto process_position_cmd(msg::Position::ConstSharedPtr const& msg) -> void {
            // TODO - if any of the motor positions are invalid, then u should cancel the message.
            for (std::size_t i = 0; i < msg->names.size(); ++i) {
                std::string const& name = msg->names[i];
                float const& position = msg->positions[i];
                if (!m_controllers.contains(name)) {
                    RCLCPP_ERROR(get_logger(), "Position request for %s ignored!", name.c_str());
                    continue;
                }
                BrushlessController<Revolutions>& controller = get_controller(name);
                if (msg->names.at(i) != controller.get_name()) {
                    RCLCPP_ERROR(get_logger(), "Position request at topic for %s ignored!", msg->names.at(0).c_str());
                    continue;
                }
                controller.set_desired_position(Radians{position});
            }
        }

        auto publish_state_callback() -> void {
            m_controller_state.header.stamp = now();
            for (size_t i = 0; i < m_controller_state.names.size(); ++i) {
                auto& controller = get_controller(m_controller_state.names[i]);
                m_controller_state.states[i] = controller.get_state();
                m_controller_state.errors[i] = controller.get_error();
                m_controller_state.positions[i] = controller.get_position();
                m_controller_state.velocities[i] = controller.get_velocity();
                m_controller_state.currents[i] = controller.get_current();
                m_controller_state.limits_hit[i] = controller.getLimitsHitBits();
            }
            for (auto const& statePub: m_controller_state_pubs) {
                statePub->publish(m_controller_state);
            }
        }

    private:
        std::vector<std::string> m_motor_groups = {"left", "right"};
        std::vector<std::string> m_motors = {"front", "middle", "back"};
        std::unordered_map<std::string, BrushlessController<Revolutions>> m_controllers;

        rclcpp::Subscription<msg::Velocity>::SharedPtr m_velocity_sub;

        rclcpp::TimerBase::SharedPtr m_publish_state_timer;

        std::vector<rclcpp::Publisher<msg::ControllerState>::SharedPtr> m_controller_state_pubs;

        msg::ControllerState m_controller_state;
    };

} // namespace mrover


auto main(int const argc, char** argv) -> int {
    rclcpp::init(argc, argv);
    auto const drive_bridge = std::make_shared<mrover::MotorTestBridge>();
    drive_bridge->init();
    rclcpp::spin(drive_bridge);
    rclcpp::shutdown();
    return EXIT_SUCCESS;
}
