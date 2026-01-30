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

#include "brushed.hpp"
#include "brushless.hpp"

/*
 *  Initializes the necessary motor controllers for drive
 */

namespace mrover {

    using namespace std::chrono_literals;

    class MotorTestBridge final : public rclcpp::Node {
    public:
        MotorTestBridge() : Node{"motor_test_bridge"} {
            // all initialization is done in the init() function to allow for the usage of shared_from_this()
        }

        auto init() -> void {
            m_throttle_sub = create_subscription<msg::Throttle>("motor_thr_cmd", 1, [this](msg::Throttle::ConstSharedPtr const& msg) {
                process_throttle_cmd(msg);
            });
            m_position_sub = create_subscription<msg::Position>("motor_pos_cmd", 1, [this](msg::Position::ConstSharedPtr const& msg) {
                process_position_cmd(msg);
            });
            m_velocity_sub = create_subscription<msg::Velocity>("motor_vel_cmd", 1, [this](msg::Velocity::ConstSharedPtr const& msg) {
                process_velocity_cmd(msg);
            });

            m_controller_state_pub = create_publisher<msg::ControllerState>("motor_controller_state", 1);

            for (std::string const& name: m_motor_names) {
                if (name.front() == 'b') {
                    m_bmc = std::make_shared<BrushedController>(shared_from_this(), "jetson", name);
                }
                if (name.front() == 'm') {
                    m_moteus = std::make_shared<BrushlessController<Radians>>(shared_from_this(), "jetson", name);
                }
            }

            m_controller_state.header.stamp = now();
            m_controller_state.header.frame_id = "";
            m_controller_state.names.resize(m_motor_names.size());
            m_controller_state.states.resize(m_motor_names.size());
            m_controller_state.errors.resize(m_motor_names.size());
            m_controller_state.positions.resize(m_motor_names.size());
            m_controller_state.velocities.resize(m_motor_names.size());
            m_controller_state.currents.resize(m_motor_names.size());
            m_controller_state.limits_hit.resize(m_motor_names.size());

            m_publish_state_timer = create_wall_timer(std::chrono::milliseconds(100), [this]() { publish_state_callback(); });
        }

        auto process_throttle_cmd(msg::Throttle::ConstSharedPtr const& msg) -> void {
            for (size_t i = 0; i < msg->names.size(); ++i) {
                std::string const& name = msg->names[i];
                float const& throttle = msg->throttles[i];
                if (!std::ranges::any_of(m_motor_names.begin(), m_motor_names.end(), [&](std::string const& x) { return x == name; })) {
                    RCLCPP_ERROR(get_logger(), "throttle request for %s ignored (%f)!", name.c_str(), throttle);
                    continue;
                }

                if (name.front() == 'b') {
                    m_bmc->set_desired_throttle(throttle);
                }
                if (name.front() == 'm') {
                    RCLCPP_ERROR(get_logger(), "throttle request for %s ignored (%f)!", name.c_str(), throttle);
                }
            }
        }

        auto process_position_cmd(msg::Position::ConstSharedPtr const& msg) -> void {
            // TODO(eric) if any of the motor positions are invalid, then should cancel the message
            for (size_t i = 0; i < msg->names.size(); ++i) {
                std::string const& name = msg->names[i];
                float const& position = msg->positions[i];
                if (!std::ranges::any_of(m_motor_names.begin(), m_motor_names.end(), [&](std::string const& x) { return x == name; })) {
                    RCLCPP_ERROR(get_logger(), "position request for %s ignored (%f)!", name.c_str(), position);
                    continue;
                }

                if (name.front() == 'b') {
                    m_bmc->set_desired_position(Radians{position});
                }
                if (name.front() == 'm') {
                    m_moteus->set_desired_position(Radians{position});
                }
            }
        }

        auto process_velocity_cmd(msg::Velocity::ConstSharedPtr const& msg) -> void {
            for (size_t i = 0; i < msg->names.size(); ++i) {
                std::string const& name = msg->names[i];
                float const& velocity = msg->velocities[i];
                if (!std::ranges::any_of(m_motor_names.begin(), m_motor_names.end(), [&](std::string const& x) { return x == name; })) {
                    RCLCPP_ERROR(get_logger(), "position request for %s ignored (%f)!", name.c_str(), velocity);
                    continue;
                }

                if (name.front() == 'b') {
                    m_bmc->set_desired_velocity(RadiansPerSecond{velocity});
                }
                if (name.front() == 'm') {
                    m_moteus->set_desired_velocity(RadiansPerSecond{velocity});
                }
            }
        }

        auto publish_state_callback() -> void {
            m_controller_state.header.stamp = get_clock()->now();
            for (size_t i = 0; i < m_motor_names.size(); ++i) {
                if (std::string const& name = m_motor_names[i]; name.front() == 'b' && m_bmc) {
                    fill_state_msg(i, name, m_bmc);
                } else if (name.front() == 'm' && m_moteus) {
                    fill_state_msg(i, name, m_moteus);
                }
            }
            m_controller_state_pub->publish(m_controller_state);
        }

    private:
        std::vector<std::string> m_motor_names = {"bmc", "moteus"};

        std::shared_ptr<BrushedController> m_bmc;
        std::shared_ptr<BrushlessController<Radians>> m_moteus;

        rclcpp::Subscription<msg::Throttle>::SharedPtr m_throttle_sub;
        rclcpp::Subscription<msg::Velocity>::SharedPtr m_velocity_sub;
        rclcpp::Subscription<msg::Position>::SharedPtr m_position_sub;

        rclcpp::TimerBase::SharedPtr m_publish_state_timer;

        rclcpp::Publisher<msg::ControllerState>::SharedPtr m_controller_state_pub;

        msg::ControllerState m_controller_state;

        template<typename T>
        auto fill_state_msg(size_t const i, std::string const& name, std::shared_ptr<T>& controller) -> void {
            m_controller_state.names[i] = name;
            m_controller_state.states[i] = controller->get_state();
            m_controller_state.errors[i] = controller->get_error();
            m_controller_state.positions[i] = controller->get_position();
            m_controller_state.velocities[i] = controller->get_velocity();
            m_controller_state.currents[i] = controller->get_current();
            m_controller_state.limits_hit[i] = controller->getLimitsHitBits();
        }
    };

} // namespace mrover


auto main(int const argc, char** argv) -> int {
    rclcpp::init(argc, argv);
    auto const motor_test_bridge = std::make_shared<mrover::MotorTestBridge>();
    motor_test_bridge->init();
    rclcpp::spin(motor_test_bridge);
    rclcpp::shutdown();
    return EXIT_SUCCESS;
}
