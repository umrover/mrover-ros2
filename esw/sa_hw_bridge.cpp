#include <chrono>
#include <memory>
#include <string>
#include <unordered_map>
#include <vector>

#include <rclcpp/rclcpp.hpp>

#include <units.hpp>
#include <units_eigen.hpp>

#include <mrover/msg/controller_state.hpp>
#include <mrover/msg/position.hpp>
#include <mrover/msg/throttle.hpp>
#include <mrover/msg/velocity.hpp>
#include <mrover/srv/adjust_motor.hpp>
#include <sensor_msgs/msg/joint_state.hpp>

#include "motor_library/brushed.hpp"
#include "motor_library/brushless.hpp"

namespace mrover {
    template<typename E>
    using Vector2 = Eigen::Matrix<E, 2, 1>;

    template<typename E>
    using Matrix2 = Eigen::Matrix<E, 2, 2>;

    // Maps pitch and roll values to the DE0 and DE1 motors outputs
    // For example when only pitching the motor, both controllers should be moving in the same direction
    // When rolling, the controllers should move in opposite directions
    auto static const PITCH_ROLL_TO_0_1 = (Matrix2<Dimensionless>{} << -1, -1, -1, 1).finished();
    Dimensionless static constexpr PITCH_ROLL_TO_01_SCALE{40};

    // How often we send an adjust command to the DE motors
    // This corrects the HALL-effect motor source on the Moteus based on the absolute encoder readings
    std::chrono::seconds static constexpr DE_OFFSET_TIMER_PERIOD = std::chrono::seconds{1};

    class SAHWBridge : public rclcpp::Node {

        using Controller = std::variant<BrushedController, BrushlessController<Meters>, BrushlessController<Revolutions>>;

    public:
        SAHWBridge() : rclcpp::Node{"sa_hw_bridge", rclcpp::NodeOptions{}
                                                              .allow_undeclared_parameters(true)
                                                              .automatically_declare_parameters_from_overrides(true)} {
            // all initialization is done in the init() function to allow for the usage of shared_from_this()
        }

        auto init() -> void {

            
            mJointA = std::make_shared<BrushlessController<Meters>>(shared_from_this(), "jetson", "linear_actuator");
            mJointB = std::make_shared<BrushedController>(shared_from_this(), "jetson", "augar");
            mJointC = std::make_shared<BrushlessController<Revolutions>>(shared_from_this(), "jetson", "joint_c");
            mJointDe0 = std::make_shared<BrushlessController<Revolutions>>(shared_from_this(), "jetson", "joint_de_0");
            mJointDe1 = std::make_shared<BrushlessController<Revolutions>>(shared_from_this(), "jetson", "joint_de_1");
            mGripper = std::make_shared<BrushedController>(shared_from_this(), "jetson", "gripper");
            mCam = std::make_shared<BrushedController>(shared_from_this(), "jetson", "cam");

            mSAThrottleSub = create_subscription<msg::Throttle>("sa_throttle_cmd", 1, [this](msg::Throttle::ConstSharedPtr const& msg) { processThrottleCmd(msg); });
            mSAVelocitySub = create_subscription<msg::Velocity>("sa_velocity_cmd", 1, [this](msg::Velocity::ConstSharedPtr const& msg) { processVelocityCmd(msg); });
            mSAPositionSub = create_subscription<msg::Position>("sa_position_cmd", 1, [this](msg::Position::ConstSharedPtr const& msg) { processPositionCmd(msg); });

            mDeOffsetTimer = create_wall_timer(DE_OFFSET_TIMER_PERIOD, [this]() { updateDeOffsets(); });
            mJointDe0AdjustClient = create_client<srv::AdjustMotor>("joint_de_0_adjust");
            mJointDe1AdjustClient = create_client<srv::AdjustMotor>("joint_de_1_adjust");

            mPublishDataTimer = create_wall_timer(
                    std::chrono::milliseconds(100),
                    [this]() { publishDataCallback(); });
            mJointDataPub = create_publisher<sensor_msgs::msg::JointState>("sa_joint_data", 1);
            mControllerStatePub = create_publisher<msg::ControllerState>("sa_controller_state", 1);

            mJointData.name = mJointNames;
            mJointData.position.resize(mJointNames.size());
            mJointData.velocity.resize(mJointNames.size());
            mJointData.effort.resize(mJointNames.size());

            mControllerState.name = mJointNames;
            mControllerState.state.resize(mJointNames.size());
            mControllerState.error.resize(mJointNames.size());
            mControllerState.limit_hit.resize(mJointNames.size());
        }

    private:
        std::unordered_map<std::string, std::shared_ptr<BrushedController>> mMotors;

        rclcpp::Subscription<msg::Throttle>::SharedPtr mSAThrottleSub;

        rclcpp::TimerBase::SharedPtr mPublishDataTimer;
        rclcpp::Publisher<sensor_msgs::msg::JointState>::SharedPtr mJointDataPub;
        rclcpp::Publisher<msg::ControllerState>::SharedPtr mControllerStatePub;
        sensor_msgs::msg::JointState mJointData;
        msg::ControllerState mControllerState;


        auto processThrottleCmd(msg::Throttle::ConstSharedPtr const& msg) -> void {
            if (msg->names.size() != msg->throttles.size()) {
                RCLCPP_ERROR(get_logger(), "Name count and value count mismatched!");
                return;
            }

            std::optional<Dimensionless> jointDePitchThrottle, jointDeRollThrottle;

            for (std::size_t i = 0; i < msg->names.size(); ++i) {
                std::string const& name = msg->names[i];
                Dimensionless const& throttle = msg->throttles[i];

                // Silly little thing to save some speed. Could easily just do the straight up string comparision
                switch (name.front() + name.back()) {
                    case 'j' + 'h':
                        jointDePitchThrottle = throttle;
                        break;
                    case 'j' + 'l':
                        jointDeRollThrottle = throttle;
                        break;
                    case 'j' + 'a':
                        mJointA->setDesiredThrottle(throttle);
                        break;
                    case 'j' + 'b':
                        mJointB->setDesiredThrottle(throttle);
                        break;
                    case 'j' + 'c':
                        mJointC->setDesiredThrottle(throttle);
                        break;
                    case 'g' + 'r':
                        mGripper->setDesiredThrottle(throttle);
                        break;
                    case 'c' + 'm':
                        mCam->setDesiredThrottle(throttle);
                        break;
                }
            }

            if (jointDePitchThrottle.has_value() && jointDeRollThrottle.has_value()) {
                Vector2<Dimensionless> const pitchRollThrottles{jointDePitchThrottle.value(), jointDeRollThrottle.value()};
                Vector2<Dimensionless> const motorThrottles = PITCH_ROLL_TO_0_1 * pitchRollThrottles;

                mJointDe0->setDesiredThrottle(motorThrottles[0].get());
                mJointDe1->setDesiredThrottle(motorThrottles[1].get());
            }
        }

        auto processVelocityCmd(msg::Velocity::ConstSharedPtr const& msg) -> void {
            if (msg->names.size() != msg->velocities.size()) {
                RCLCPP_ERROR(get_logger(), "Name count and value count mismatched!");
                return;
            }

            std::optional<RadiansPerSecond> jointDePitchVelocity, jointDeRollVelocity;

            for (std::size_t i = 0; i < msg->names.size(); ++i) {
                std::string const& name = msg->names[i];
                float const& velocity = msg->velocities[i];

                // Silly little thing to save some speed. Could easily just do the straight up string comparision
                switch (name.front() + name.back()) {
                    case 'j' + 'h':
                        jointDePitchVelocity = RadiansPerSecond{velocity};
                        break;
                    case 'j' + 'l':
                        jointDeRollVelocity = RadiansPerSecond{velocity};
                        break;
                    case 'j' + 'a':
                        mJointA->setDesiredVelocity(MetersPerSecond{velocity});
                        break;
                    case 'j' + 'b':
                        mJointB->setDesiredVelocity(RadiansPerSecond{velocity});
                        break;
                    case 'j' + 'c':
                        mJointC->setDesiredVelocity(RadiansPerSecond{velocity});
                        break;
                    case 'g' + 'r':
                        mGripper->setDesiredVelocity(RadiansPerSecond{velocity});
                        break;
                    case 'c' + 'm':
                        mCam->setDesiredVelocity(RadiansPerSecond{velocity});
                        break;
                }
            }

            if (jointDePitchVelocity.has_value() && jointDeRollVelocity.has_value()) {
                Vector2<RadiansPerSecond> const pitchRollVelocities{jointDePitchVelocity.value(), jointDeRollVelocity.value()};
                Vector2<RadiansPerSecond> const motorVelocities = PITCH_ROLL_TO_0_1 * pitchRollVelocities;

                mJointDe0->setDesiredVelocity(motorVelocities[0]);
                mJointDe1->setDesiredVelocity(motorVelocities[1]);
            }
        }

        auto processPositionCmd(msg::Position::ConstSharedPtr const& msg) -> void {
            if (msg->names.size() != msg->positions.size()) {
                RCLCPP_ERROR(get_logger(), "Name count and value count mismatched!");
                return;
            }

            std::optional<Radians> jointDePitchPosition, jointDeRollPosition;

            for (std::size_t i = 0; i < msg->names.size(); ++i) {
                std::string const& name = msg->names[i];
                float const& position = msg->positions[i];

                // Silly little thing to save some speed. Could easily just do the straight up string comparision
                switch (name.front() + name.back()) {
                    case 'j' + 'h':
                        jointDePitchPosition = Radians{position};
                        break;
                    case 'j' + 'l':
                        jointDeRollPosition = Radians{position};
                        break;
                    case 'j' + 'a':
                        mJointA->setDesiredPosition(Meters{position});
                        break;
                    case 'j' + 'b':
                        mJointB->setDesiredPosition(Radians{position});
                        break;
                    case 'j' + 'c':
                        mJointC->setDesiredPosition(Radians{position});
                        break;
                    case 'g' + 'r':
                        mGripper->setDesiredPosition(Radians{position});
                        break;
                    case 'c' + 'm':
                        mCam->setDesiredPosition(Radians{position});
                        break;
                }

                if (jointDePitchPosition.has_value() && jointDeRollPosition.has_value()) {
                    Vector2<Radians> const pitchRollPositions{jointDePitchPosition.value(), jointDeRollPosition.value()};
                    Vector2<Radians> motorPositions = PITCH_ROLL_TO_01_SCALE * PITCH_ROLL_TO_0_1 * pitchRollPositions;

                    mJointDe0->setDesiredPosition(motorPositions[0]);
                    mJointDe1->setDesiredPosition(motorPositions[1]);
                }
            }
        }
        static auto wrapAngle(float angle) -> float {
            constexpr float pi = std::numbers::pi_v<float>;
            constexpr float tau = 2 * pi;
            return std::fmod(angle + pi, tau) - pi;
        }

        auto publishDataCallback() -> void {
            mJointData.header.stamp = get_clock()->now();

            mJointData.position[0] = {mJointA->getPosition().get()};
            mJointData.velocity[0] = {mJointA->getVelocity().get()};
            mJointData.effort[0] = {mJointA->getEffort()};

            mJointData.position[1] = {mJointB->getPosition().get()};
            mJointData.velocity[1] = {mJointB->getVelocity().get()};
            mJointData.effort[1] = {mJointB->getEffort()};

            mJointData.position[2] = {Radians{mJointC->getPosition()}.get()};
            mJointData.velocity[2] = {RadiansPerSecond{mJointC->getVelocity()}.get()};
            mJointData.effort[2] = {mJointC->getEffort()};

            auto const pitchWrapped = wrapAngle(Radians{mJointDe0->getPosition()}.get());
            auto const rollWrapped = wrapAngle(Radians{mJointDe1->getPosition()}.get());
            mJointDePitchRoll = {pitchWrapped, rollWrapped};

            mJointData.position[3] = pitchWrapped;
            mJointData.velocity[3] = {RadiansPerSecond{mJointDe0->getVelocity()}.get()};
            mJointData.effort[3] = {mJointDe0->getEffort()};

            mJointData.position[4] = rollWrapped;
            mJointData.velocity[4] = {RadiansPerSecond{mJointDe1->getVelocity()}.get()};
            mJointData.effort[4] = {mJointDe1->getEffort()};

            mJointData.position[5] = {mGripper->getPosition().get()};
            mJointData.velocity[5] = {mGripper->getVelocity().get()};
            mJointData.effort[5] = {mGripper->getEffort()};

            mJointData.position[6] = {mCam->getPosition().get()};
            mJointData.velocity[6] = {mCam->getVelocity().get()};
            mJointData.effort[6] = {mCam->getEffort()};

            mJointDataPub->publish(mJointData);

            mControllerState.state[0] = {mJointA->getState()};
            mControllerState.error[0] = {mJointA->getErrorState()};
            mControllerState.limit_hit[0] = {mJointA->getLimitsHitBits()};

            mControllerState.state[1] = {mJointB->getState()};
            mControllerState.error[1] = {mJointB->getErrorState()};
            mControllerState.limit_hit[1] = {mJointB->getLimitsHitBits()};

            mControllerState.state[2] = {mJointC->getState()};
            mControllerState.error[2] = {mJointC->getErrorState()};
            mControllerState.limit_hit[2] = {mJointC->getLimitsHitBits()};

            mControllerState.state[3] = {mJointDe0->getState()};
            mControllerState.error[3] = {mJointDe0->getErrorState()};
            mControllerState.limit_hit[3] = {mJointDe0->getLimitsHitBits()};

            mControllerState.state[4] = {mJointDe1->getState()};
            mControllerState.error[4] = {mJointDe1->getErrorState()};
            mControllerState.limit_hit[4] = {mJointDe1->getLimitsHitBits()};

            mControllerState.state[5] = {mGripper->getState()};
            mControllerState.error[5] = {mGripper->getErrorState()};
            mControllerState.limit_hit[5] = {mGripper->getLimitsHitBits()};

            mControllerState.state[6] = {mCam->getState()};
            mControllerState.error[6] = {mCam->getErrorState()};
            mControllerState.limit_hit[6] = {mCam->getLimitsHitBits()};

            mControllerStatePub->publish(mControllerState);
        }

        auto updateDeOffsets() -> void {
            if (!mJointDePitchRoll) return;

            Vector2<Radians> motorPositions = PITCH_ROLL_TO_01_SCALE * PITCH_ROLL_TO_0_1 * mJointDePitchRoll.value();
            {
                auto adjust = std::make_shared<srv::AdjustMotor::Request>();
                adjust->name = "joint_de_0";
                adjust->value = {motorPositions[0].get()};
                mJointDe0AdjustClient->async_send_request(adjust);
            }
            {
                auto adjust = std::make_shared<srv::AdjustMotor::Request>();
                adjust->name = "joint_de_1";
                adjust->value = {motorPositions[1].get()};
                mJointDe1AdjustClient->async_send_request(adjust);
            }
        }
    };
} // namespace mrover

auto main(int argc, char** argv) -> int {
    rclcpp::init(argc, argv);
    auto sa_hw_bridge = std::make_shared<mrover::SAHWBridge>();
    sa_hw_bridge->init();
    rclcpp::spin(sa_hw_bridge);
    rclcpp::shutdown();
    return EXIT_SUCCESS;
}
