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
            mThrottleSub = create_subscription<msg::Throttle>("motor_thr_cmd", 1, [this](msg::Throttle::ConstSharedPtr const& msg) {
                processThrottleCmd(msg);
            });
            mPositionSub = create_subscription<msg::Position>("motor_pos_cmd", 1, [this](msg::Position::ConstSharedPtr const& msg) {
                processPositionCmd(msg);
            });
            mVelocitySub = create_subscription<msg::Velocity>("motor_vel_cmd", 1, [this](msg::Velocity::ConstSharedPtr const& msg) {
                processVelocityCmd(msg);
            });

            mControllerStatePub = create_publisher<msg::ControllerState>("motor_controller_state", 1);

            for (std::string const& name: mMotorNames) {
                if (name.front() == 'b') {
                    mBMC = std::make_shared<BrushedController<Radians>>(shared_from_this(), "jetson", name);
                }
                if (name.front() == 'm') {
                    mMoteus = std::make_shared<BrushlessController<Radians>>(shared_from_this(), "jetson", name);
                }
            }

            mControllerState.header.stamp = now();
            mControllerState.header.frame_id = "";
            mControllerState.names.resize(mMotorNames.size());
            mControllerState.states.resize(mMotorNames.size());
            mControllerState.errors.resize(mMotorNames.size());
            mControllerState.positions.resize(mMotorNames.size());
            mControllerState.velocities.resize(mMotorNames.size());
            mControllerState.currents.resize(mMotorNames.size());
            mControllerState.limits_hit.resize(mMotorNames.size());

            mPublishStateTimer = create_wall_timer(std::chrono::milliseconds(100), [this]() { publishStateCallback(); });
        }

        auto processThrottleCmd(msg::Throttle::ConstSharedPtr const& msg) -> void {
            for (size_t i = 0; i < msg->names.size(); ++i) {
                std::string const& name = msg->names[i];
                float const& throttle = msg->throttles[i];
                if (!std::ranges::any_of(mMotorNames.begin(), mMotorNames.end(), [&](std::string const& x) { return x == name; })) {
                    RCLCPP_ERROR(get_logger(), "throttle request for %s ignored (%f)!", name.c_str(), throttle);
                    continue;
                }

                if (name.front() == 'b') {
                    mBMC->setDesiredThrottle(throttle);
                }
                if (name.front() == 'm') {
                    mMoteus->setDesiredThrottle(throttle);
                }
            }
        }

        auto processPositionCmd(msg::Position::ConstSharedPtr const& msg) -> void {
            // TODO(eric) if any of the motor positions are invalid, then should cancel the message
            for (size_t i = 0; i < msg->names.size(); ++i) {
                std::string const& name = msg->names[i];
                float const& position = msg->positions[i];
                if (!std::ranges::any_of(mMotorNames.begin(), mMotorNames.end(), [&](std::string const& x) { return x == name; })) {
                    RCLCPP_ERROR(get_logger(), "position request for %s ignored (%f)!", name.c_str(), position);
                    continue;
                }

                if (name.front() == 'b') {
                    mBMC->setDesiredPosition(Radians{position});
                }
                if (name.front() == 'm') {
                    mMoteus->setDesiredPosition(Radians{position});
                }
            }
        }

        auto processVelocityCmd(msg::Velocity::ConstSharedPtr const& msg) -> void {
            for (size_t i = 0; i < msg->names.size(); ++i) {
                std::string const& name = msg->names[i];
                float const& velocity = msg->velocities[i];
                if (!std::ranges::any_of(mMotorNames.begin(), mMotorNames.end(), [&](std::string const& x) { return x == name; })) {
                    RCLCPP_ERROR(get_logger(), "position request for %s ignored (%f)!", name.c_str(), velocity);
                    continue;
                }

                if (name.front() == 'b') {
                    mBMC->setDesiredVelocity(RadiansPerSecond{velocity});
                }
                if (name.front() == 'm') {
                    mMoteus->setDesiredVelocity(RadiansPerSecond{velocity});
                }
            }
        }

        auto publishStateCallback() -> void {
            mControllerState.header.stamp = get_clock()->now();
            for (size_t i = 0; i < mMotorNames.size(); ++i) {
                if (std::string const& name = mMotorNames[i]; name.front() == 'b' && mBMC) {
                    fillStateMsg(i, name, mBMC);
                } else if (name.front() == 'm' && mMoteus) {
                    fillStateMsg(i, name, mMoteus);
                }
            }
            mControllerStatePub->publish(mControllerState);
        }

    private:
        std::vector<std::string> mMotorNames = {"bmc", "moteus"};

        std::shared_ptr<BrushedController<Radians>> mBMC;
        std::shared_ptr<BrushlessController<Radians>> mMoteus;

        rclcpp::Subscription<msg::Throttle>::SharedPtr mThrottleSub;
        rclcpp::Subscription<msg::Velocity>::SharedPtr mVelocitySub;
        rclcpp::Subscription<msg::Position>::SharedPtr mPositionSub;

        rclcpp::TimerBase::SharedPtr mPublishStateTimer;

        rclcpp::Publisher<msg::ControllerState>::SharedPtr mControllerStatePub;

        msg::ControllerState mControllerState;

        template<typename T>
        auto fillStateMsg(size_t const i, std::string const& name, std::shared_ptr<T>& controller) -> void {
            mControllerState.names[i] = name;
            mControllerState.states[i] = controller->getState();
            mControllerState.errors[i] = controller->getError();
            mControllerState.positions[i] = controller->getPosition();
            mControllerState.velocities[i] = controller->getVelocity();
            mControllerState.currents[i] = controller->getCurrent();
            mControllerState.limits_hit[i] = controller->getLimitsHitBits();
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
