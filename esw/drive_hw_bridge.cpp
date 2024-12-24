#include <memory>
#include <rclcpp/rclcpp.hpp>

#include <rclcpp/subscription.hpp>
#include <units/units.hpp>

#include "can_device/can_device.hpp"
#include "motor_library/brushless.hpp"
#include "motor_library/brushed.hpp"
#include "mrover/msg/detail/position__struct.hpp"
#include "mrover/msg/detail/throttle__struct.hpp"


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
                mThrottleSubsByName[group] = create_subscription<msg::Throttle>(std::format("{}_drive_throttle_cmd", group), 1, [this](msg::Throttle::ConstSharedPtr const& msg) {
                    moveMotorsThrottle(msg);
                });
                mVelocitySubsByName[group] = create_subscription<msg::Velocity>(std::format("{}_drive_velocity_cmd", group), 1, [this](msg::Velocity::ConstSharedPtr const& msg) {
                    // moveMotorsVelocity(msg, group);
                });
                mPositionSubsByName[group] = create_subscription<msg::Position>(std::format("{}_drive_throttle_cmd", group), 1, [this](msg::Position::ConstSharedPtr const& msg) {
                    // moveMotorsPosition(msg, group);
                });
                for (std::string const& motor : mMotors) {
                    std::string name = std::format("{}_{}", motor, group);
                    mControllers.emplace(name, std::in_place_type<BrushlessController<Revolutions>>, "jetson", name);
                }
            }
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
                controller.setDesiredVelocity(velocity);
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
            controller.setDesiredPosition(position);
        }
    }

    private:

        std::unique_ptr<mrover::CanDevice> driveCanDevice;

        std::vector<std::string> mMotorGroups = {"left", "right"};
        std::vector<std::string> mMotors = {"front", "middle", "back"};
        std::unordered_map<std::string, BrushlessController<Revolutions>> mControllers;

        std::unordered_map<std::string, rclcpp::Subscription<msg::Throttle>::SharedPtr> mThrottleSubsByName;
        std::unordered_map<std::string, rclcpp::Subscription<msg::Velocity>::SharedPtr> mVelocitySubsByName;
        std::unordered_map<std::string, rclcpp::Subscription<msg::Position>::SharedPtr> mPositionSubsByName;
    };

} // namespace mrover

auto main(int argc, char** argv) -> int {
    rclcpp::init(argc, argv);
    rclcpp::spin(std::make_shared<mrover::DriveHardwareBridge>());
    rclcpp::shutdown();
    return EXIT_SUCCESS;
}