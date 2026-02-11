#include <chrono>
#include <memory>
#include <numbers>
#include <string>
#include <vector>

#include <rclcpp/rclcpp.hpp>
#include <rclcpp/subscription.hpp>
#include <rclcpp/timer.hpp>

#include <units.hpp>
#include <units_eigen.hpp>

#include <parameter.hpp>

#include <mrover/msg/controller_state.hpp>
#include <mrover/msg/position.hpp>
#include <mrover/msg/throttle.hpp>
#include <mrover/msg/velocity.hpp>

#include "brushed.hpp"
#include "brushless.hpp"


namespace mrover {
    template<typename E>
    using Vector2 = Eigen::Matrix<E, 2, 1>;

    template<typename E>
    using Matrix2 = Eigen::Matrix<E, 2, 2>;

    template<typename T>
    struct is_brushless : std::false_type {};

    template<typename U>
    struct is_brushless<std::shared_ptr<BrushlessController<U>>> : std::true_type {};

    template<typename T>
    inline constexpr bool is_brushless_v = is_brushless<T>::value;

    // Maps pitch and roll values to the DE0 and DE1 motors outputs
    // For example when only pitching the motor, both controllers should be moving in the same direction
    // When rolling, the controllers should move in opposite directions
    auto static const PITCH_ROLL_TO_0_1 = (Matrix2<Dimensionless>{} << -1, -1, -1, 1).finished();
    Dimensionless static constexpr PITCH_ROLL_TO_01_SCALE{40};

    class ArmHWBridge : public rclcpp::Node {

        using Controller = std::variant<
                std::shared_ptr<BrushedController<Meters>>,
                std::shared_ptr<BrushedController<Radians>>,
                std::shared_ptr<BrushlessController<Meters>>,
                std::shared_ptr<BrushlessController<Radians>>,
                std::shared_ptr<BrushlessController<Revolutions>>>;

    public:
        ArmHWBridge() : rclcpp::Node{"arm_hw_bridge"} {
            // all initialization is done in the init() function to allow for the usage of shared_from_this()
        }

        auto init() -> void {
            mJointA = std::make_shared<BrushlessController<Meters>>(shared_from_this(), "jetson", "joint_a");
            mJointB = std::make_shared<BrushedController<Meters>>(shared_from_this(), "jetson", "joint_b");
            mJointC = std::make_shared<BrushlessController<Radians>>(shared_from_this(), "jetson", "joint_c");
            mJointDE0 = std::make_shared<BrushlessController<Revolutions>>(shared_from_this(), "jetson", "joint_de_0");
            mJointDE1 = std::make_shared<BrushlessController<Revolutions>>(shared_from_this(), "jetson", "joint_de_1");
            mGripper = std::make_shared<BrushedController<Meters>>(shared_from_this(), "jetson", "gripper");
            mPusher = std::make_shared<BrushedController<Meters>>(shared_from_this(), "jetson", "pusher");

            mArmThrottleSub = create_subscription<msg::Throttle>("arm_thr_cmd", 1, [this](msg::Throttle::ConstSharedPtr const& msg) { processThrottleCmd(msg); });
            mArmVelocitySub = create_subscription<msg::Velocity>("arm_vel_cmd", 1, [this](msg::Velocity::ConstSharedPtr const& msg) { processVelocityCmd(msg); });
            mArmPositionSub = create_subscription<msg::Position>("arm_pos_cmd", 1, [this](msg::Position::ConstSharedPtr const& msg) { processPositionCmd(msg); });

            std::vector<ParameterWrapper> parameters = {
                    {"joint_b_segment_length", mJointBSegmentLength.rep, 0.0},
                    {"joint_b_actuator_length", mJointBActuatorLength.rep, 0.0},
                    {"joint_b_mount_length", mJointBMountLength.rep, 0.0},
                    {"joint_b_mount_theta", mJointBMountTheta.rep, 0.0},
                    {"joint_b_segment_offset_theta", mJointBSegmentOffsetTheta.rep, 0.0},
                    {"joint_de_pitch_offset", mJointDePitchOffset.rep, 0.0},
                    {"joint_de_roll_offset", mJointDeRollOffset.rep, 0.0},
                    {"joint_de_pitch_max_position", mJointDePitchMaxPosition.rep, std::numeric_limits<float>::infinity()},
                    {"joint_de_pitch_min_position", mJointDePitchMinPosition.rep, -std::numeric_limits<float>::infinity()},
                    {"joint_de_roll_max_position", mJointDeRollMaxPosition.rep, std::numeric_limits<float>::infinity()},
                    {"joint_de_roll_min_position", mJointDeRollMinPosition.rep, -std::numeric_limits<float>::infinity()},
            };
            ParameterWrapper::declareParameters(this, parameters);

            // sanity check joint b geometry
            if ((mJointBSegmentLength + mJointBActuatorLength <= mJointBMountLength) ||
                (mJointBSegmentLength + mJointBMountLength <= mJointBActuatorLength) ||
                (mJointBActuatorLength + mJointBMountLength <= mJointBSegmentLength)) {
                RCLCPP_ERROR(get_logger(), "arm joint b segment lengths are invalid");
            }

            mProbeTimer = create_wall_timer(
            mProbeInterval,
            [this]() {
                probeIfBrushless(mJointA, "joint_a");
                probeIfBrushless(mJointC, "joint_c");
                probeIfBrushless(mJointDE0, "joint_de_0");
                probeIfBrushless(mJointDE1, "joint_de_1");
            });

            mPublishDataTimer = create_wall_timer(
                    std::chrono::milliseconds(100),
                    [this]() { publishDataCallback(); });
            mControllerStatePub = create_publisher<msg::ControllerState>("arm_controller_state", 1);

            mControllerState.names = mJointNames;
            mControllerState.states.resize(mJointNames.size());
            mControllerState.errors.resize(mJointNames.size());
            mControllerState.limits_hit.resize(mJointNames.size());

            mControllerState.header.stamp = now();
            mControllerState.header.frame_id = "";
            mControllerState.names = mJointNames;
            mControllerState.states.resize(mJointNames.size());
            mControllerState.errors.resize(mJointNames.size());
            mControllerState.positions.resize(mJointNames.size());
            mControllerState.velocities.resize(mJointNames.size());
            mControllerState.currents.resize(mJointNames.size());
            mControllerState.limits_hit.resize(mJointNames.size());
        }

    private:
        static constexpr auto PI = Radians{std::numbers::pi};

        std::vector<std::string> const mJointNames{"joint_a", "joint_b", "joint_c", "joint_de_pitch", "joint_de_roll", "gripper", "pusher"};

        std::shared_ptr<BrushlessController<Meters>> mJointA;
        std::shared_ptr<BrushedController<Meters>> mJointB;
        std::shared_ptr<BrushlessController<Radians>> mJointC;
        std::shared_ptr<BrushlessController<Revolutions>> mJointDE0;
        std::shared_ptr<BrushlessController<Revolutions>> mJointDE1;
        std::shared_ptr<BrushedController<Meters>> mGripper;
        std::shared_ptr<BrushedController<Meters>> mPusher;

        rclcpp::Subscription<msg::Throttle>::SharedPtr mArmThrottleSub;
        rclcpp::Subscription<msg::Velocity>::SharedPtr mArmVelocitySub;
        rclcpp::Subscription<msg::Position>::SharedPtr mArmPositionSub;

        Meters mJointBSegmentLength, mJointBActuatorLength, mJointBMountLength;
        Radians mJointBMountTheta, mJointBSegmentOffsetTheta;

        Radians mJointDePitchOffset, mJointDeRollOffset;
        std::optional<Vector2<Radians>> mJointDePitchRoll; // position after offset is applied (raw - offset)

        Radians mJointDePitchMaxPosition, mJointDePitchMinPosition;
        Radians mJointDeRollMaxPosition, mJointDeRollMinPosition;

        std::unordered_map<std::string_view, rclcpp::Time> mLastCommandTime;
        std::chrono::milliseconds const mProbeInterval{100};
        std::chrono::milliseconds const mCommandTimeout{50};

        rclcpp::TimerBase::SharedPtr mProbeTimer;
        rclcpp::TimerBase::SharedPtr mPublishDataTimer;
        rclcpp::Publisher<msg::ControllerState>::SharedPtr mControllerStatePub;
        msg::ControllerState mControllerState;

        auto markCommandTime(std::string_view const& name) -> void {
            mLastCommandTime[name] = now();
        }

        auto probeIfBrushless(Controller const& joint_variant, std::string_view const name) -> void {
            if (auto const it = mLastCommandTime.find(name); it != mLastCommandTime.end())
                if ((now() - it->second) < mCommandTimeout)
                    return;

            auto const brushless_meters = std::get_if<std::shared_ptr<BrushlessController<Meters>>>(&joint_variant);
            auto const brushless_revs = std::get_if<std::shared_ptr<BrushlessController<Revolutions>>>(&joint_variant);
            auto const brushless_rads = std::get_if<std::shared_ptr<BrushlessController<Radians>>>(&joint_variant);

            if (brushless_meters && *brushless_meters) {
                (*brushless_meters)->sendQuery();
                (*brushless_meters)->getPressedLimitSwitchInfo();
            } else if (brushless_revs && *brushless_revs) {
                (*brushless_revs)->sendQuery();
                (*brushless_revs)->getPressedLimitSwitchInfo();
            } else if (brushless_rads && *brushless_rads) {
                (*brushless_rads)->sendQuery();
                (*brushless_rads)->getPressedLimitSwitchInfo();
            }
        }

        auto bRadiansToMeters(Radians const rad) -> Meters {
            auto const standard_rad = -rad;

            auto const a = mJointBMountLength;
            auto const c = mJointBSegmentLength;

            // calculate internal angle
            auto const beta = PI - standard_rad - mJointBMountTheta - mJointBSegmentOffsetTheta;

            // law of cosines for total actuator length
            // b^2 = a^2 + c^2 - 2ac * cos(beta)
            auto const cos_beta = cos(beta.get());
            auto const b_squared = (a * a) + (c * c) - (2.0 * a * c * cos_beta);
            auto const b = Meters{fsqrt(std::fmax(0.0, b_squared.get()))};

            // determine extended length
            return b - mJointBActuatorLength;
        }

        auto bMetersToRadians(Meters const len) -> Radians {
            auto const a = mJointBMountLength;
            auto const b = mJointBActuatorLength + len;
            auto const c = mJointBSegmentLength;

            // law of cosines to find internal angle beta
            // b^2 = a^2 + c^2 - 2ac * cos(beta)
            auto cos_beta = (((a * a) + (c * c) - (b * b)) / (2 * a * c)).get();

            // check acos bounds
            if (cos_beta > 1.0) cos_beta = 1.0;
            if (cos_beta < -1.0) cos_beta = -1.0;

            // transform to radians in rover plane relative to horizontal
            auto const interior_beta = Radians{acos(cos_beta)};
            auto const segment_angle_to_horizontal = PI - interior_beta - mJointBMountTheta;
            auto const standard_rad = segment_angle_to_horizontal - mJointBSegmentOffsetTheta;

            return -standard_rad;
        }

        auto bVelocityToAngular(MetersPerSecond const vel, Meters const current_len, Radians const current_rad) -> RadiansPerSecond {
            auto const a = mJointBMountLength;
            auto const c = mJointBSegmentLength;
            auto const b = mJointBActuatorLength + current_len;

            auto const standard_rad = -current_rad;
            auto const beta = PI - standard_rad - mJointBMountTheta - mJointBSegmentOffsetTheta;
            auto const sin_beta = sin(beta.get());

            if (std::abs(sin_beta) < 1e-6) return RadiansPerSecond{0.0};

            // omega_std = -(b * v) / (a * c * sin_beta)
            // omega = -omega_std
            auto const omega_standard = -(b.get() * vel.get()) / (a.get() * c.get() * sin_beta);
            return RadiansPerSecond{-omega_standard};
        }

        auto bAngularToVelocity(RadiansPerSecond const omega, Meters const current_len, Radians const current_rad) -> MetersPerSecond {
            auto const a = mJointBMountLength;
            auto const c = mJointBSegmentLength;
            auto const b = mJointBActuatorLength + current_len;

            auto const standard_rad = -current_rad;
            auto const standard_omega = -omega;

            auto const beta = PI - standard_rad - mJointBMountTheta - mJointBSegmentOffsetTheta;
            auto const sin_beta = sin(beta.get());

            // v = (ac * sin_beta / b) * (-omega_std)
            auto const vel = (a.get() * c.get() * sin_beta / b.get()) * (-standard_omega.get());
            return MetersPerSecond{vel};
        }

        auto processThrottleCmd(msg::Throttle::ConstSharedPtr const& msg) -> void {
            if (msg->names.size() != msg->throttles.size()) {
                RCLCPP_ERROR(get_logger(), "Name count and value count mismatched!");
                return;
            }

            std::optional<Dimensionless> jointDePitchThrottle, jointDeRollThrottle;

            for (std::size_t i = 0; i < msg->names.size(); ++i) {
                std::string const& name = msg->names[i];
                Dimensionless const& throttle = msg->throttles[i];
                markCommandTime(name);

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
                    case 'p' + 'r':
                        mPusher->setDesiredThrottle(throttle);
                        break;
                }
            }

            if (jointDePitchThrottle.has_value() || jointDeRollThrottle.has_value()) {
                if (!jointDePitchThrottle.has_value()) {
                    jointDePitchThrottle = Dimensionless{0};
                } else if (!jointDeRollThrottle.has_value()) {
                    jointDeRollThrottle = Dimensionless{0};
                }

                if (mJointDePitchRoll.has_value()) {
                    if ((*jointDePitchThrottle > Dimensionless{0} && (*mJointDePitchRoll)[0] >= mJointDePitchMaxPosition) ||
                        (*jointDePitchThrottle < Dimensionless{0} && (*mJointDePitchRoll)[0] <= mJointDePitchMinPosition)) {
                        RCLCPP_INFO(get_logger(), "Joint DE Pitch limit hit!");
                        jointDePitchThrottle = Dimensionless{0};
                    }
                    if ((*jointDeRollThrottle > Dimensionless{0} && (*mJointDePitchRoll)[1] >= mJointDeRollMaxPosition) ||
                        (*jointDeRollThrottle < Dimensionless{0} && (*mJointDePitchRoll)[1] <= mJointDeRollMinPosition)) {
                        RCLCPP_INFO(get_logger(), "Joint DE Roll limit hit!");
                        jointDeRollThrottle = Dimensionless{0};
                    }
                } else {
                    RCLCPP_WARN(get_logger(), "Commanding Joint DE throttle with no position readings! No limits will be enforced");
                }

                Vector2<Dimensionless> const pitchRollThrottles{jointDePitchThrottle.value(), jointDeRollThrottle.value()};
                Vector2<Dimensionless> const motorThrottles = PITCH_ROLL_TO_0_1 * pitchRollThrottles;

                mJointDE0->setDesiredThrottle(motorThrottles[0].get());
                mJointDE1->setDesiredThrottle(motorThrottles[1].get());
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
                markCommandTime(name);

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
                    case 'j' + 'b': {
                        auto const current_len = Meters{mJointB->getPosition()};
                        auto const current_rad = bMetersToRadians(current_len);
                        auto const target_vel = bAngularToVelocity(RadiansPerSecond{velocity}, current_len, current_rad);
                        mJointB->setDesiredVelocity(target_vel);
                        break;
                    }
                    case 'j' + 'c':
                        mJointC->setDesiredVelocity(RadiansPerSecond{velocity});
                        break;
                    case 'g' + 'r':
                        mGripper->setDesiredVelocity(MetersPerSecond{velocity});
                        break;
                    case 'p' + 'r':
                        mPusher->setDesiredVelocity(MetersPerSecond{velocity});
                        break;
                }
            }

            if (jointDePitchVelocity.has_value() || jointDeRollVelocity.has_value()) {
                if (!jointDePitchVelocity.has_value()) {
                    jointDePitchVelocity = RadiansPerSecond{0};
                } else if (!jointDeRollVelocity.has_value()) {
                    jointDeRollVelocity = RadiansPerSecond{0};
                }

                if (mJointDePitchRoll.has_value()) {
                    if ((*jointDePitchVelocity > RadiansPerSecond{0} && (*mJointDePitchRoll)[0] >= mJointDePitchMaxPosition) ||
                        (*jointDePitchVelocity < RadiansPerSecond{0} && (*mJointDePitchRoll)[0] <= mJointDePitchMinPosition)) {
                        RCLCPP_INFO(get_logger(), "Joint DE Pitch limit hit!");
                        jointDePitchVelocity = RadiansPerSecond{0};
                    }
                    if ((*jointDeRollVelocity > RadiansPerSecond{0} && (*mJointDePitchRoll)[1] >= mJointDeRollMaxPosition) ||
                        (*jointDeRollVelocity < RadiansPerSecond{0} && (*mJointDePitchRoll)[1] <= mJointDeRollMinPosition)) {
                        RCLCPP_INFO(get_logger(), "Joint DE Roll limit hit!");
                        jointDeRollVelocity = RadiansPerSecond{0};
                    }
                } else {
                    RCLCPP_WARN(get_logger(), "Commanding Joint DE velocity with no position readings! No limits will be enforced");
                }

                Vector2<RadiansPerSecond> const pitchRollVelocities{jointDePitchVelocity.value(), jointDeRollVelocity.value()};
                Vector2<RadiansPerSecond> const motorVelocities = PITCH_ROLL_TO_01_SCALE * PITCH_ROLL_TO_0_1 * pitchRollVelocities;

                mJointDE0->setDesiredVelocity(motorVelocities[0]);
                mJointDE1->setDesiredVelocity(motorVelocities[1]);
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
                markCommandTime(name);

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
                    case 'j' + 'b': {
                        auto const target_meters = bRadiansToMeters(Radians{position});
                        mJointB->setDesiredPosition(target_meters);
                        break;
                    }
                    case 'j' + 'c':
                        mJointC->setDesiredPosition(Radians{position});
                        break;
                    case 'g' + 'r':
                        mGripper->setDesiredPosition(Meters{position});
                        break;
                    case 'p' + 'r':
                        mPusher->setDesiredPosition(Meters{position});
                        break;
                }

                if (jointDePitchPosition.has_value() || jointDeRollPosition.has_value()) {
                    if (!mJointDePitchRoll.has_value()) {
                        RCLCPP_WARN(get_logger(), "Commanding Joint DE position with no position readings! Not commanding position");
                        return;
                    }

                    if (!jointDePitchPosition.has_value()) {
                        jointDePitchPosition = (*mJointDePitchRoll)[0];
                    } else if (!jointDeRollPosition.has_value()) {
                        jointDeRollPosition = (*mJointDePitchRoll)[1];
                    }

                    if ((*jointDePitchPosition >= mJointDePitchMaxPosition) || (*jointDePitchPosition <= mJointDePitchMinPosition)) {
                        RCLCPP_INFO(get_logger(), "Joint DE Pitch limit hit!");
                        jointDePitchPosition = (*mJointDePitchRoll)[0];
                    }
                    if ((*jointDeRollPosition >= mJointDeRollMaxPosition) || (*jointDeRollPosition <= mJointDeRollMinPosition)) {
                        RCLCPP_INFO(get_logger(), "Joint DE Roll limit hit!");
                        jointDeRollPosition = (*mJointDePitchRoll)[1];
                    }

                    Vector2<Radians> const pitchRollPositions{jointDePitchPosition.value(), jointDeRollPosition.value()};
                    Vector2<Radians> motorPositions = PITCH_ROLL_TO_01_SCALE * PITCH_ROLL_TO_0_1 * pitchRollPositions;

                    mJointDE0->setDesiredPosition(motorPositions[0]);
                    mJointDE1->setDesiredPosition(motorPositions[1]);
                }
            }
        }

        static auto wrapAngle(float const angle) -> float {
            constexpr float pi = std::numbers::pi_v<float>;
            constexpr float tau = 2 * pi;
            return std::fmod(angle + pi, tau) - pi;
        }

        auto publishDataCallback() -> void {
            mControllerState.header.stamp = now();

            auto const pitchWrapped = wrapAngle((Radians{mJointDE0->getPosition()} - mJointDePitchOffset).get());
            auto const rollWrapped = wrapAngle((Radians{mJointDE1->getPosition()} - mJointDeRollOffset).get());
            mJointDePitchRoll = {pitchWrapped, rollWrapped};

            for (std::size_t i = 0; i < mJointNames.size(); ++i) {
                std::string_view const name = mJointNames[i];

                switch (name.front() + name.back()) {
                    case 'j' + 'h':
                        mControllerState.names[i] = name;
                        mControllerState.states[i] = mJointDE0->getState();
                        mControllerState.errors[i] = mJointDE0->getError();
                        mControllerState.positions[i] = pitchWrapped;
                        mControllerState.velocities[i] = {RadiansPerSecond{mJointDE0->getVelocity()}.get()};
                        mControllerState.currents[i] = mJointDE0->getCurrent();
                        mControllerState.limits_hit[i] = mJointDE0->getLimitsHitBits();
                        break;
                    case 'j' + 'l':
                        mControllerState.names[i] = name;
                        mControllerState.states[i] = mJointDE1->getState();
                        mControllerState.errors[i] = mJointDE1->getError();
                        mControllerState.positions[i] = rollWrapped;
                        mControllerState.velocities[i] = {RadiansPerSecond{mJointDE1->getVelocity()}.get()};
                        mControllerState.currents[i] = mJointDE1->getCurrent();
                        mControllerState.limits_hit[i] = mJointDE1->getLimitsHitBits();
                        break;
                    case 'j' + 'a':
                        mControllerState.names[i] = name;
                        mControllerState.states[i] = mJointA->getState();
                        mControllerState.errors[i] = mJointA->getError();
                        mControllerState.positions[i] = mJointA->getPosition();
                        mControllerState.velocities[i] = mJointA->getVelocity();
                        mControllerState.currents[i] = mJointA->getCurrent();
                        mControllerState.limits_hit[i] = mJointA->getLimitsHitBits();
                        break;
                    case 'j' + 'b': {
                        auto const pos_m = Meters{mJointB->getPosition()};
                        auto const vel_mps = MetersPerSecond{mJointB->getVelocity()};
                        auto const pos_rad = bMetersToRadians(pos_m);
                        auto const vel_radps = bVelocityToAngular(vel_mps, pos_m, pos_rad);
                        mControllerState.names[i] = name;
                        mControllerState.states[i] = mJointB->getState();
                        mControllerState.errors[i] = mJointB->getError();
                        mControllerState.positions[i] = pos_rad.get();
                        mControllerState.velocities[i] = vel_radps.get();
                        mControllerState.currents[i] = mJointB->getCurrent();
                        mControllerState.limits_hit[i] = mJointB->getLimitsHitBits();
                        break;
                    }
                    case 'j' + 'c':
                        mControllerState.names[i] = name;
                        mControllerState.states[i] = mJointC->getState();
                        mControllerState.errors[i] = mJointC->getError();
                        mControllerState.positions[i] = mJointC->getPosition();
                        mControllerState.velocities[i] = mJointC->getVelocity();
                        mControllerState.currents[i] = mJointC->getCurrent();
                        mControllerState.limits_hit[i] = mJointC->getLimitsHitBits();
                        break;
                    case 'g' + 'r':
                        mControllerState.names[i] = name;
                        mControllerState.states[i] = mGripper->getState();
                        mControllerState.errors[i] = mGripper->getError();
                        mControllerState.positions[i] = mGripper->getPosition();
                        mControllerState.velocities[i] = mGripper->getVelocity();
                        mControllerState.currents[i] = mGripper->getCurrent();
                        mControllerState.limits_hit[i] = mGripper->getLimitsHitBits();
                        break;
                    case 'p' + 'r':
                        mControllerState.names[i] = name;
                        mControllerState.states[i] = mPusher->getState();
                        mControllerState.errors[i] = mPusher->getError();
                        mControllerState.positions[i] = mPusher->getPosition();
                        mControllerState.velocities[i] = mPusher->getVelocity();
                        mControllerState.currents[i] = mPusher->getCurrent();
                        mControllerState.limits_hit[i] = mPusher->getLimitsHitBits();
                        break;
                }
            }

            mControllerStatePub->publish(mControllerState);
        }

        auto queryMoteusCallback() -> void {

        }
    };
} // namespace mrover


auto main(int const argc, char** argv) -> int {
    rclcpp::init(argc, argv);
    auto const arm_hw_bridge = std::make_shared<mrover::ArmHWBridge>();
    arm_hw_bridge->init();

    rclcpp::executors::MultiThreadedExecutor executor;
    executor.add_node(arm_hw_bridge);
    executor.spin();

    rclcpp::shutdown();
    return EXIT_SUCCESS;
}
