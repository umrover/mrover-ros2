#include <chrono>
#include <memory>
#include <rclcpp/rclcpp.hpp>

#include <rclcpp/subscription.hpp>
#include <rclcpp/timer.hpp>
#include <units/units.hpp>

#include "can_device/can_device.hpp"
#include "motor_library/brushless.hpp"

#include <mrover/msg/controller_state.hpp>
#include <mrover/msg/position.hpp>
#include <mrover/msg/throttle.hpp>
#include <mrover/msg/velocity.hpp>

/*
 *  Initializes the necessary motor controllers for drive
 */

namespace mrover {

    using namespace std::chrono_literals;

    class DriveHardwareBridge final : public rclcpp::Node {
    public:
        DriveHardwareBridge() : Node{"drive_hw_bridge"} {

            // Create publishers and subscribers for left and right motor groups
            for (std::string const& group : mMotorGroups) {
                // TODO (ali): does this actually emplace lol
                mThrottleSubsByName.emplace_back(create_subscription<msg::Throttle>(std::format("{}_drive_throttle_cmd", group), 1, [this](msg::Throttle::ConstSharedPtr const& msg) {
                    moveMotorsThrottle(msg);
                }));
                mVelocitySubsByName.emplace_back(create_subscription<msg::Velocity>(std::format("{}_drive_velocity_cmd", group), 1, [this](msg::Velocity::ConstSharedPtr const& msg) {
                    moveMotorsVelocity(msg);
                }));
                mPositionSubsByName.emplace_back(create_subscription<msg::Position>(std::format("{}_drive_throttle_cmd", group), 1, [this](msg::Position::ConstSharedPtr const& msg) {
                    moveMotorsPosition(msg);
                }));
                for (std::string const& motor : mMotors) {
                    std::string name = std::format("{}_{}", motor, group);
                    mControllers.try_emplace(name, shared_from_this(), "jetson", name, BrushlessController<Revolutions>::Config{});
                    mJointState.name.push_back(name);
                }
                mJointState.position.resize(mMotors.size());
                mJointState.velocity.resize(mMotors.size());
                mJointState.effort.resize(mMotors.size());

                mJointStatePub = create_publisher<sensor_msgs::msg::JointState>(std::format("{}_drive_joint_data", group), 1);
                mControllerStatePub = create_publisher<msg::ControllerState>(std::format("{}_drive_controller_state", group), 1);
            }

            // Periodic timer for published motor states
            mPublishDataTimer = create_wall_timer(std::chrono::milliseconds(100), [this]() {publishStatesDataCallback();});
        }

        auto getController(std::string const& name) -> BrushlessController<Revolutions>& {
            return mControllers.at(name);
        }

        auto moveMotorsThrottle(msg::Throttle::ConstPtr const& msg) -> void {
            for (size_t i = 0; i < msg->names.size(); ++i) {
                std::string const& name = msg->names[i];
                float const& throttle = msg->throttles[i];
                if (!mControllers.contains(name)) {
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

        auto moveMotorsVelocity(msg::Velocity::ConstPtr const& msg) -> void {
            for (size_t i = 0; i < msg->names.size(); ++i) {
                std::string const& name = msg->names[i];
                float const& velocity = msg->velocities[i];
                if (!mControllers.contains(name)) {
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

        auto moveMotorsPosition(msg::Position::ConstPtr const& msg) -> void {
        // TODO - if any of the motor positions are invalid, then u should cancel the message.

            for (std::size_t i = 0; i < msg->names.size(); ++i) {
                std::string const& name = msg->names[i];
                float const& position = msg->positions[i];
                if (!mControllers.contains(name)) {
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

        auto publishStatesDataCallback() -> void {
            mJointState.header.stamp = get_clock()->now();
            for (size_t i = 0; i < mJointState.name.size(); ++i) {
                BrushlessController<Revolutions>& controller = getController(mJointState.name[i]);
                mJointState.position[i] = controller.getPosition().get();
                mJointState.velocity[i] = controller.getVelocity().get();
                mJointState.effort[i] = controller.getEffort();

                mControllerState.state[i] = controller.getState();
                mControllerState.error[i] = controller.getErrorState();
                mControllerState.limit_hit[i] = controller.getLimitsHitBits();
            }
            mJointStatePub->publish(mJointState);
            mControllerStatePub->publish(mControllerState);
        }

    private:

        std::unique_ptr<mrover::CanDevice> driveCanDevice;

        std::vector<std::string> mMotorGroups = {"left", "right"};
        std::vector<std::string> mMotors = {"front", "middle", "back"};
        std::unordered_map<std::string, BrushlessController<Revolutions>> mControllers;

        std::vector<rclcpp::Subscription<msg::Throttle>::SharedPtr> mThrottleSubsByName;
        std::vector<rclcpp::Subscription<msg::Velocity>::SharedPtr> mVelocitySubsByName;
        std::vector<rclcpp::Subscription<msg::Position>::SharedPtr> mPositionSubsByName;

        rclcpp::TimerBase::SharedPtr mPublishDataTimer;

        rclcpp::Publisher<sensor_msgs::msg::JointState>::SharedPtr mJointStatePub;
        rclcpp::Publisher<msg::ControllerState>::SharedPtr mControllerStatePub;

        sensor_msgs::msg::JointState mJointState;
        msg::ControllerState mControllerState;
    };

} // namespace mrover

auto main(int argc, char** argv) -> int {
    rclcpp::init(argc, argv);
    rclcpp::spin(std::make_shared<mrover::DriveHardwareBridge>());
    rclcpp::shutdown();
    return EXIT_SUCCESS;
}