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

    class DriveHWBridge final : public rclcpp::Node {
    public:
        DriveHWBridge() : Node{"drive_hw_bridge"} {
            // all initialization is done in the init() function to allow for the usage of shared_from_this()
        }

        auto init() -> void {
            // publishers and subscribers for left and right motor groups
            mVelocitySub = create_subscription<msg::Velocity>("drive_velocity_cmd", 1, [this](msg::Velocity::ConstSharedPtr const& msg) {
                processVelocityCmd(msg);
            });

            // emplace controller names
            for (std::string const& motor: mMotors) {
                mControllers.try_emplace(motor, shared_from_this(), "jetson", motor);
                mControllerState.names.push_back(motor);
            }

            // initialize outbound message
            mControllerState.header.stamp = now();
            mControllerState.header.frame_id = "";
            mControllerState.names.resize(mControllers.size());
            mControllerState.states.resize(mControllers.size());
            mControllerState.errors.resize(mControllers.size());
            mControllerState.positions.resize(mControllers.size());
            mControllerState.velocities.resize(mControllers.size());
            mControllerState.currents.resize(mControllers.size());
            mControllerState.limits_hit.resize(mControllers.size());

            mControllerStatePubs.emplace_back(create_publisher<msg::ControllerState>("drive_controller_state", 1));

            // periodic timer for published motor states
            mPublishStateTimer = create_wall_timer(std::chrono::milliseconds(100), [this]() { publishStateCallback(); });
        }

        auto getController(std::string const& name) -> BrushlessController<Revolutions>& {
            return mControllers.at(name);
        }

        auto processThrottleCmd(msg::Throttle::ConstSharedPtr const& msg) -> void {
            for (size_t i = 0; i < msg->names.size(); ++i) {
                std::string const& name = msg->names[i];
                float const& throttle = msg->throttles[i];
                if (!mControllers.contains(name)) {
                    RCLCPP_ERROR(get_logger(), "Group throttle request for %s ignored (%f)!", name.c_str(), msg->throttles[i]);
                    continue;
                }
                BrushlessController<Revolutions>& controller = getController(name);
                if (msg->names.at(i) != controller.getName()) {
                    RCLCPP_ERROR(get_logger(), "Throttle request at topic for %s ignored!", msg->names.at(0).c_str());
                    continue;
                }
                controller.setDesiredThrottle(throttle);
            }
        }

        auto processVelocityCmd(msg::Velocity::ConstSharedPtr const& msg) -> void {
            for (size_t i = 0; i < msg->names.size(); ++i) {
                std::string const& name = msg->names[i];
                float const& velocity = msg->velocities[i];
                if (!mControllers.contains(name)) {
                    RCLCPP_ERROR(get_logger(), "Velocity request for %s ignored!", name.c_str());
                    continue;
                }
                BrushlessController<Revolutions>& controller = getController(name);
                if (msg->names.at(i) != controller.getName()) {
                    RCLCPP_ERROR(get_logger(), "Velocity request at topic for %s ignored!", msg->names.at(0).c_str());
                    continue;
                }
                controller.setDesiredVelocity(RadiansPerSecond{velocity});
            }
        }

        auto processPositionCmd(msg::Position::ConstSharedPtr const& msg) -> void {
            // TODO - if any of the motor positions are invalid, then u should cancel the message.
            for (std::size_t i = 0; i < msg->names.size(); ++i) {
                std::string const& name = msg->names[i];
                float const& position = msg->positions[i];
                if (!mControllers.contains(name)) {
                    RCLCPP_ERROR(get_logger(), "Position request for %s ignored!", name.c_str());
                    continue;
                }
                BrushlessController<Revolutions>& controller = getController(name);
                if (msg->names.at(i) != controller.getName()) {
                    RCLCPP_ERROR(get_logger(), "Position request at topic for %s ignored!", msg->names.at(0).c_str());
                    continue;
                }
                controller.setDesiredPosition(Radians{position});
            }
        }

        auto publishStateCallback() -> void {
            mControllerState.header.stamp = now();
            for (size_t i = 0; i < mControllerState.names.size(); ++i) {
                auto& controller = getController(mControllerState.names[i]);
                mControllerState.states[i] = controller.getState();
                mControllerState.errors[i] = controller.getError();
                mControllerState.positions[i] = controller.getPosition();
                mControllerState.velocities[i] = controller.getVelocity();
                mControllerState.currents[i] = controller.getCurrent();
                mControllerState.limits_hit[i] = controller.getLimitsHitBits();
            }
            for (auto const& statePub: mControllerStatePubs) {
                statePub->publish(mControllerState);
            }
        }

    private:
        std::vector<std::string> mMotors = {"front_right", "middle_right", "back_right", "front_left", "middle_left", "back_left"};
        std::unordered_map<std::string, BrushlessController<Revolutions>> mControllers;

        rclcpp::Subscription<msg::Velocity>::SharedPtr mVelocitySub;

        rclcpp::TimerBase::SharedPtr mPublishStateTimer;

        std::vector<rclcpp::Publisher<msg::ControllerState>::SharedPtr> mControllerStatePubs;

        msg::ControllerState mControllerState;
    };

} // namespace mrover


auto main(int const argc, char** argv) -> int {
    rclcpp::init(argc, argv);
    auto const drive_bridge = std::make_shared<mrover::DriveHWBridge>();
    drive_bridge->init();
    rclcpp::spin(drive_bridge);
    rclcpp::shutdown();
    return EXIT_SUCCESS;
}
