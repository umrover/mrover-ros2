#include <chrono>
#include <memory>
#include <string>
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

    // TODO: ros params?
    constexpr static BrushlessController<Meters>::Config JOINT_A_CONFIG = {
            .minVelocity = MetersPerSecond{-0.05},
            .maxVelocity = MetersPerSecond{0.05},
            .minPosition = Meters{0.0},
            .maxPosition = Meters{0.35}, // limit switch to limit switch is roughly 13.25in. joint a roller is roughly 4.5in
            .maxTorque = 20.0,

            .limitSwitch0Present = true,
            .limitSwitch0Enabled = true,
            .limitSwitch0LimitsFwd = true,
            .limitSwitch0ActiveHigh = false,
            .limitSwitch0UsedForReadjustment = true,
            .limitSwitch0ReadjustPosition = Meters{0.4},

            .limitSwitch1Present = true,
            .limitSwitch1Enabled = true,
            .limitSwitch1LimitsFwd = false,
            .limitSwitch1ActiveHigh = false,
            .limitSwitch1UsedForReadjustment = true,
            .limitSwitch1ReadjustPosition = Meters{0.0},
    };
    constexpr static BrushedController::Config JOINT_B_CONFIG = {
            .limitSwitchPresent = {true, false},
            .limitSwitchEnabled = {true, false},
            .limitSwitchLimitsFwd = {false, false},
            .limitSwitchActiveHigh = {false, false},
            .limitSwitchUsedForReadjustment = {false, false},
            // .limitSwitchReadjustPosition = {Radians{0.0}},
            .limitMaxForwardPosition = true,
            .limitMaxBackwardPosition = false,

            .gearRatio = 1.0,
            .isInverted = false,

            .driverVoltage = 10.5,
            .motorMaxVoltage = 12.0,

            .quadPresent = false,
            // .quadRatio = 1.0,

            .absPresent = true,
            .absRatio = -1.0,
            .absOffset = 0.995_rad,

            .minVelocity = -1.0_rad_per_s,
            .maxVelocity = 1.0_rad_per_s,
            .minPosition = -0.9_rad,
            .maxPosition = 0_rad,

            .positionGains = {.p = 30.0},
            .velocityGains = {.p = 30.0},

            .calibrationThrottle = 0.5,
    };
    constexpr static BrushlessController<Revolutions>::Config JOINT_C_CONFIG = {
            .minVelocity = RevolutionsPerSecond{-0.05},
            .maxVelocity = RevolutionsPerSecond{0.05},
            .minPosition = Revolutions{-0.125},
            .maxPosition = Revolutions{0.30},
            .maxTorque = 200.0,
    };
    constexpr static BrushlessController<Revolutions>::Config JOINT_DE_0_CONFIG = {
            .minVelocity = RevolutionsPerSecond{-5.0},
            .maxVelocity = RevolutionsPerSecond{5.0},
            .minPosition = Revolutions{-10000.0},
            .maxPosition = Revolutions{10000.0},
            .maxTorque = 20.0,
    };
    constexpr static BrushlessController<Revolutions>::Config JOINT_DE_1_CONFIG = {
            .minVelocity = RevolutionsPerSecond{-5.0},
            .maxVelocity = RevolutionsPerSecond{5.0},
            .minPosition = Revolutions{-10000.0},
            .maxPosition = Revolutions{10000.0},
            .maxTorque = 20.0,
    };
    constexpr static BrushedController::Config GRIPPER_CONFIG = {
            .gearRatio = 47.0,
            .isInverted = false,
            .driverVoltage = 10.5,
            .motorMaxVoltage = 12.0,
            .quadPresent = false,
            .absPresent = false,
            .calibrationThrottle = 0.5,
    };

    constexpr static BrushedController::Config CAM_CONFIG = {
            .limitSwitchPresent = {true, true},
            .limitSwitchEnabled = {true, true},
            .limitSwitchLimitsFwd = {false, false},
            .limitSwitchActiveHigh = {false, false},
            .limitSwitchUsedForReadjustment = {false, false},
            // .limitSwitchReadjustPosition = {Radians{0.0}},
            .limitMaxForwardPosition = true,
            .limitMaxBackwardPosition = true,

            // TODO: (owen) ask for gear ratio
            // .gearRatio = 1.0,
            .isInverted = false,
            .driverVoltage = 10.5,
            .motorMaxVoltage = 12.0,
            .quadPresent = false,
            .absPresent = false,
            .calibrationThrottle = 0.5,
    };

    class ArmHWBridge : public rclcpp::Node {

        using Controller = std::variant<BrushedController, BrushlessController<Meters>, BrushlessController<Revolutions>>;

    public:
        ArmHWBridge() : rclcpp::Node{"arm_hw_bridge"} {
            // all initialization is done in the init() function to allow for the usage of shared_from_this()
        }

        auto init() -> void {
            mJointA = std::make_shared<BrushlessController<Meters>>(shared_from_this(), "jetson", "joint_a", JOINT_A_CONFIG);
            mJointB = std::make_shared<BrushedController>(shared_from_this(), "jetson", "joint_b", JOINT_B_CONFIG);
            mJointC = std::make_shared<BrushlessController<Revolutions>>(shared_from_this(), "jetson", "joint_c", JOINT_C_CONFIG);
            mJointDe0 = std::make_shared<BrushlessController<Revolutions>>(shared_from_this(), "jetson", "joint_de_0", JOINT_DE_0_CONFIG);
            mJointDe1 = std::make_shared<BrushlessController<Revolutions>>(shared_from_this(), "jetson", "joint_de_1", JOINT_DE_1_CONFIG);
            mGripper = std::make_shared<BrushedController>(shared_from_this(), "jetson", "gripper", GRIPPER_CONFIG);
            mCam = std::make_shared<BrushedController>(shared_from_this(), "jetson", "cam", CAM_CONFIG);

            mArmThrottleSub = create_subscription<msg::Throttle>("arm_throttle_cmd", 1, [this](msg::Throttle::ConstSharedPtr const& msg) { processThrottleCmd(msg); });
            mArmVelocitySub = create_subscription<msg::Velocity>("arm_velocity_cmd", 1, [this](msg::Velocity::ConstSharedPtr const& msg) { processVelocityCmd(msg); });
            mArmPositionSub = create_subscription<msg::Position>("arm_position_cmd", 1, [this](msg::Position::ConstSharedPtr const& msg) { processPositionCmd(msg); });

            mDeOffsetTimer = create_wall_timer(DE_OFFSET_TIMER_PERIOD, [this]() { updateDeOffsets(); });
            mJointDe0AdjustClient = create_client<srv::AdjustMotor>("joint_de_0_adjust");
            mJointDe1AdjustClient = create_client<srv::AdjustMotor>("joint_de_1_adjust");

            mPublishDataTimer = create_wall_timer(
                    std::chrono::milliseconds(100),
                    [this]() { publishDataCallback(); });
            mJointDataPub = create_publisher<sensor_msgs::msg::JointState>("arm_joint_data", 1);
            mControllerStatePub = create_publisher<msg::ControllerState>("arm_controller_state", 1);

            mJointData.name = mArmJointNames;
            mJointData.position.resize(mArmJointNames.size());
            mJointData.velocity.resize(mArmJointNames.size());
            mJointData.effort.resize(mArmJointNames.size());

            mControllerState.name = mArmJointNames;
            mControllerState.state.resize(mArmJointNames.size());
            mControllerState.error.resize(mArmJointNames.size());
            mControllerState.limit_hit.resize(mArmJointNames.size());
        }

    private:
        std::vector<std::string> const mArmJointNames{"joint_a", "joint_b", "joint_c", "joint_de_pitch", "joint_de_roll", "allen_key", "gripper", "cam"};

        std::shared_ptr<BrushlessController<Meters>> mJointA;
        std::shared_ptr<BrushedController> mJointB;
        std::shared_ptr<BrushlessController<Revolutions>> mJointC;
        std::shared_ptr<BrushlessController<Revolutions>> mJointDe0;
        std::shared_ptr<BrushlessController<Revolutions>> mJointDe1;
        std::shared_ptr<BrushedController> mGripper;
        std::shared_ptr<BrushedController> mCam; // rip the solenoid. tentative name until design finalized


        rclcpp::Subscription<msg::Throttle>::SharedPtr mArmThrottleSub;
        rclcpp::Subscription<msg::Velocity>::SharedPtr mArmVelocitySub;
        rclcpp::Subscription<msg::Position>::SharedPtr mArmPositionSub;

        rclcpp::TimerBase::SharedPtr mDeOffsetTimer;
        rclcpp::Client<srv::AdjustMotor>::SharedPtr mJointDe0AdjustClient;
        rclcpp::Client<srv::AdjustMotor>::SharedPtr mJointDe1AdjustClient;
        std::optional<Vector2<Radians>> mJointDePitchRoll;

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
    auto arm_hw_bridge = std::make_shared<mrover::ArmHWBridge>();
    arm_hw_bridge->init();
    rclcpp::spin(arm_hw_bridge);
    rclcpp::shutdown();
    return EXIT_SUCCESS;
}
