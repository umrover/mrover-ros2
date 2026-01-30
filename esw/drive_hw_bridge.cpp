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

    class DriveHardwareBridge final : public rclcpp::Node {
    public:
        DriveHardwareBridge() : Node{"drive_hw_bridge"} {
            // all initialization is done in the init() function to allow for the usage of shared_from_this()
        }

        auto init() -> void {
            // Create publishers and subscribers for left and right motor groups
            for (std::string const& group: mMotorGroups) {
                // TODO (ali): does this actually emplace lol
                mThrottleSubs.emplace_back(create_subscription<msg::Throttle>(std::format("drive_{}_throttle_cmd", group), 1, [this](msg::Throttle::ConstSharedPtr const& msg) {
                    moveMotorsThrottle(msg);
                }));
                mVelocitySubs.emplace_back(create_subscription<msg::Velocity>(std::format("drive_{}_velocity_cmd", group), 1, [this](msg::Velocity::ConstSharedPtr const& msg) {
                    moveMotorsVelocity(msg);
                }));
                mPositionSubs.emplace_back(create_subscription<msg::Position>(std::format("drive_{}_position_cmd", group), 1, [this](msg::Position::ConstSharedPtr const& msg) {
                    moveMotorsPosition(msg);
                }));
                for (std::string const& motor: mMotors) {
                    std::string name = std::format("{}_{}", motor, group);
                    mControllers.try_emplace(name, shared_from_this(), "jetson", name);
                    mJointState.name.push_back(name);
                    mControllerState.names.push_back(name);
                }

                mJointStatePubs.emplace_back(create_publisher<sensor_msgs::msg::JointState>(std::format("drive_{}_joint_data", group), 1));
                mControllerStatePubs.emplace_back(create_publisher<msg::ControllerState>(std::format("{}_controller_state", group), 1));
            }
            mJointState.position.resize(mControllers.size());
            mJointState.velocity.resize(mControllers.size());
            mJointState.effort.resize(mControllers.size());

            mControllerState.states.resize(mControllers.size());
            mControllerState.errors.resize(mControllers.size());
            mControllerState.limits_hit.resize(mControllers.size());

            // Periodic timer for published motor states
            mPublishDataTimer = create_wall_timer(std::chrono::milliseconds(100), [this]() { publishStatesDataCallback(); });
        }

        auto getController(std::string const& name) -> BrushlessController<Revolutions>& {
            return mControllers.at(name);
        }

        auto moveMotorsThrottle(msg::Throttle::ConstPtr const& msg) -> void {
            for (size_t i = 0; i < msg->name.size(); ++i) {
                std::string const& name = msg->name[i];
                float const& throttle = msg->throttle[i];
                if (!mControllers.contains(name)) {
                    RCLCPP_ERROR(get_logger(), "Group throttle request for %s ignored (%f)!", name.c_str(), msg->throttle[i]);
                    continue;
                }
                BrushlessController<Revolutions>& controller = getController(name);
                if (msg->name.at(i) != controller.getControllerName()) {
                    RCLCPP_ERROR(get_logger(), "Throttle request at topic for %s ignored!", msg->name.at(0).c_str());
                    continue;
                }
                controller.setDesiredThrottle(throttle);
            }
        }

        auto moveMotorsVelocity(msg::Velocity::ConstPtr const& msg) -> void {
            for (size_t i = 0; i < msg->name.size(); ++i) {
                std::string const& name = msg->name[i];
                float const& velocity = msg->velocity[i];
                if (!mControllers.contains(name)) {
                    RCLCPP_ERROR(get_logger(), "Velocity request for %s ignored!", name.c_str());
                    continue;
                }
                BrushlessController<Revolutions>& controller = getController(name);
                if (msg->name.at(i) != controller.getControllerName()) {
                    RCLCPP_ERROR(get_logger(), "Velocity request at topic for %s ignored!", msg->name.at(0).c_str());
                    continue;
                }
                controller.setDesiredVelocity(RadiansPerSecond{velocity});
            }
        }

        auto moveMotorsPosition(msg::Position::ConstPtr const& msg) -> void {
            // TODO - if any of the motor positions are invalid, then u should cancel the message.
            for (std::size_t i = 0; i < msg->name.size(); ++i) {
                std::string const& name = msg->name[i];
                float const& position = msg->position[i];
                if (!mControllers.contains(name)) {
                    RCLCPP_ERROR(get_logger(), "Position request for %s ignored!", name.c_str());
                    continue;
                }
                BrushlessController<Revolutions>& controller = getController(name);
                if (msg->name.at(i) != controller.getControllerName()) {
                    RCLCPP_ERROR(get_logger(), "Position request at topic for %s ignored!", msg->name.at(0).c_str());
                    continue;
                }
                controller.setDesiredPosition(Radians{position});
            }
        }

        auto publishStatesDataCallback() -> void {
            mJointState.header.stamp = get_clock()->now();
            for (size_t i = 0; i < mJointState.name.size(); ++i) {
                BrushlessController<Revolutions>& controller = getController(mJointState.name[i]);
                mJointState.position[i] = controller.getPosition().get();
                mJointState.velocity[i] = controller.getVelocity().get();
                mJointState.effort[i] = controller.getEffort();

                mControllerState.states[i] = controller.getState();
                mControllerState.errors[i] = controller.getErrorState();
                mControllerState.limits_hit[i] = controller.getLimitsHitBits();
            }
            for (auto const& jointPub: mJointStatePubs) {
                jointPub->publish(mJointState);
            }
            for (auto const& statePub: mControllerStatePubs) {
                statePub->publish(mControllerState);
            }
        }

    private:
        std::vector<std::string> mMotorGroups = {"left", "right"};
        std::vector<std::string> mMotors = {"front", "middle", "back"};
        std::unordered_map<std::string, BrushlessController<Revolutions>> mControllers;

        std::vector<rclcpp::Subscription<msg::Throttle>::SharedPtr> mThrottleSubs;
        std::vector<rclcpp::Subscription<msg::Velocity>::SharedPtr> mVelocitySubs;
        std::vector<rclcpp::Subscription<msg::Position>::SharedPtr> mPositionSubs;

        rclcpp::TimerBase::SharedPtr mPublishDataTimer;

        std::vector<rclcpp::Publisher<sensor_msgs::msg::JointState>::SharedPtr> mJointStatePubs;
        std::vector<rclcpp::Publisher<msg::ControllerState>::SharedPtr> mControllerStatePubs;

        sensor_msgs::msg::JointState mJointState;
        msg::ControllerState mControllerState;
    };

} // namespace mrover

auto main(int argc, char** argv) -> int {
    rclcpp::init(argc, argv);
    auto drive_bridge = std::make_shared<mrover::DriveHardwareBridge>();
    drive_bridge->init();
    rclcpp::spin(drive_bridge);
    rclcpp::shutdown();
    return EXIT_SUCCESS;
}
