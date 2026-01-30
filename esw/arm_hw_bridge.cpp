#include <chrono>
#include <memory>
#include <string>
#include <vector>

#include <rclcpp/rclcpp.hpp>
#include <sensor_msgs/msg/joint_state.hpp>

#include <units.hpp>
#include <units_eigen.hpp>

#include <parameter.hpp>

#include <mrover/msg/controller_state.hpp>
#include <mrover/msg/position.hpp>
#include <mrover/msg/throttle.hpp>
#include <mrover/msg/velocity.hpp>
#include <mrover/srv/adjust_motor.hpp>
#include <mrover/srv/control_cam.hpp>

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
    std::chrono::seconds static constexpr DE_OFFSET_TIMER_PERIOD = std::chrono::seconds{5};


    // using MetersPerRadian = compound_unit<Meters, inverse<Radians>>;
    // using RadiansPerMeter = compound_unit<Radians, inverse<Meters>>;
    float static constexpr GRIPPER_METERS_PER_RADIAN{0.00001727579};
    float static constexpr GRIPPER_RADIANS_PER_METER{1 / 0.00001727579};

    class ArmHWBridge : public rclcpp::Node {

        using Controller = std::variant<BrushedController, BrushlessController<Meters>, BrushlessController<Revolutions>>;

    public:
        ArmHWBridge() : rclcpp::Node{"arm_hw_bridge"} {
            // all initialization is done in the init() function to allow for the usage of shared_from_this()
        }

        auto init() -> void {
            mJointA = std::make_shared<BrushlessController<Meters>>(shared_from_this(), "jetson", "joint_a");
            mJointB = std::make_shared<BrushedController>(shared_from_this(), "jetson", "joint_b");
            mJointC = std::make_shared<BrushlessController<Revolutions>>(shared_from_this(), "jetson", "joint_c");
            mJointDe0 = std::make_shared<BrushlessController<Revolutions>>(shared_from_this(), "jetson", "joint_de_0");
            mJointDe1 = std::make_shared<BrushlessController<Revolutions>>(shared_from_this(), "jetson", "joint_de_1");
            mGripper = std::make_shared<BrushedController>(shared_from_this(), "jetson", "gripper");
            mCam = std::make_shared<BrushedController>(shared_from_this(), "jetson", "cam");

            mArmThrottleSub = create_subscription<msg::Throttle>("arm_throttle_cmd", 1, [this](msg::Throttle::ConstSharedPtr const& msg) { processThrottleCmd(msg); });
            mArmVelocitySub = create_subscription<msg::Velocity>("arm_velocity_cmd", 1, [this](msg::Velocity::ConstSharedPtr const& msg) { processVelocityCmd(msg); });
            mArmPositionSub = create_subscription<msg::Position>("arm_position_cmd", 1, [this](msg::Position::ConstSharedPtr const& msg) { processPositionCmd(msg); });

            mDeOffsetTimer = create_wall_timer(DE_OFFSET_TIMER_PERIOD, [this]() { updateDeOffsets(); });

            int camControlDuration, camControlPeriod;
            std::vector<ParameterWrapper> parameters = {
                    {"joint_de_pitch_offset", mJointDePitchOffset.rep, 0.0},
                    {"joint_de_roll_offset", mJointDeRollOffset.rep, 0.0},
                    {"joint_de_pitch_max_position", mJointDePitchMaxPosition.rep, std::numeric_limits<float>::infinity()},
                    {"joint_de_pitch_min_position", mJointDePitchMinPosition.rep, -std::numeric_limits<float>::infinity()},
                    {"joint_de_roll_max_position", mJointDeRollMaxPosition.rep, std::numeric_limits<float>::infinity()},
                    {"joint_de_roll_min_position", mJointDeRollMinPosition.rep, -std::numeric_limits<float>::infinity()},
                    {"cam_control_duration", camControlDuration, 0},
                    {"cam_control_period", camControlPeriod, 50},
            };
            ParameterWrapper::declareParameters(this, parameters);

            mCamControlDuration = std::chrono::milliseconds{camControlDuration};
            mCamControlPeriod = std::chrono::milliseconds{camControlPeriod};

            mCamControlCallbackGroup = create_callback_group(rclcpp::CallbackGroupType::MutuallyExclusive);
            mCamControlServer = create_service<srv::ControlCam>(
                    "cam_control",
                    [this](srv::ControlCam::Request::ConstSharedPtr const& req,
                           srv::ControlCam::Response::SharedPtr const& res) {
                        camControlServiceCallback(req, res);
                    },
                    rmw_qos_profile_services_default,
                    mCamControlCallbackGroup);

            mPublishDataTimer = create_wall_timer(
                    std::chrono::milliseconds(100),
                    [this]() { publishDataCallback(); });
            mJointDataPub = create_publisher<sensor_msgs::msg::JointState>("arm_joint_data", 1);
            mControllerStatePub = create_publisher<msg::ControllerState>("arm_controller_state", 1);

            mJointData.name = mJointNames;
            mJointData.position.resize(mJointNames.size());
            mJointData.velocity.resize(mJointNames.size());
            mJointData.effort.resize(mJointNames.size());

            mControllerState.names = mJointNames;
            mControllerState.states.resize(mJointNames.size());
            mControllerState.errors.resize(mJointNames.size());
            mControllerState.limits_hit.resize(mJointNames.size());
        }

    private:
        std::vector<std::string> const mJointNames{"joint_a", "joint_b", "joint_c", "joint_de_pitch", "joint_de_roll", "gripper", "cam"};

        std::shared_ptr<BrushlessController<Meters>> mJointA;
        std::shared_ptr<BrushedController> mJointB;
        std::shared_ptr<BrushlessController<Revolutions>> mJointC;
        std::shared_ptr<BrushlessController<Revolutions>> mJointDe0;
        std::shared_ptr<BrushlessController<Revolutions>> mJointDe1;
        std::shared_ptr<BrushedController> mGripper;
        std::shared_ptr<BrushedController> mCam;

        rclcpp::Subscription<msg::Throttle>::SharedPtr mArmThrottleSub;
        rclcpp::Subscription<msg::Velocity>::SharedPtr mArmVelocitySub;
        rclcpp::Subscription<msg::Position>::SharedPtr mArmPositionSub;

        rclcpp::TimerBase::SharedPtr mDeOffsetTimer;
        Radians mJointDePitchOffset, mJointDeRollOffset;
        std::optional<Vector2<Radians>> mJointDePitchRoll; // position after offset is applied (raw - offset)

        Radians mJointDePitchMaxPosition, mJointDePitchMinPosition;
        Radians mJointDeRollMaxPosition, mJointDeRollMinPosition;


        // The duration in which we command the cam to move in one direction
        std::chrono::milliseconds mCamControlDuration;
        // Defines the rate in which we send CAN messages to the cam
        std::chrono::milliseconds mCamControlPeriod;

        rclcpp::CallbackGroup::SharedPtr mCamControlCallbackGroup;
        rclcpp::Service<srv::ControlCam>::SharedPtr mCamControlServer;

        rclcpp::TimerBase::SharedPtr mPublishDataTimer;
        rclcpp::Publisher<sensor_msgs::msg::JointState>::SharedPtr mJointDataPub;
        rclcpp::Publisher<msg::ControllerState>::SharedPtr mControllerStatePub;
        sensor_msgs::msg::JointState mJointData;
        msg::ControllerState mControllerState;


        auto processThrottleCmd(msg::Throttle::ConstSharedPtr const& msg) -> void {
            if (msg->name.size() != msg->throttle.size()) {
                RCLCPP_ERROR(get_logger(), "Name count and value count mismatched!");
                return;
            }

            std::optional<Dimensionless> jointDePitchThrottle, jointDeRollThrottle;

            for (std::size_t i = 0; i < msg->name.size(); ++i) {
                std::string const& name = msg->name[i];
                Dimensionless const& throttle = msg->throttle[i];

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

                mJointDe0->setDesiredThrottle(motorThrottles[0].get());
                mJointDe1->setDesiredThrottle(motorThrottles[1].get());
            }
        }

        auto processVelocityCmd(msg::Velocity::ConstSharedPtr const& msg) -> void {
            if (msg->name.size() != msg->velocity.size()) {
                RCLCPP_ERROR(get_logger(), "Name count and value count mismatched!");
                return;
            }

            std::optional<RadiansPerSecond> jointDePitchVelocity, jointDeRollVelocity;

            for (std::size_t i = 0; i < msg->name.size(); ++i) {
                std::string const& name = msg->name[i];
                float const& velocity = msg->velocity[i];

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
                        mGripper->setDesiredVelocity(RadiansPerSecond{velocity * GRIPPER_RADIANS_PER_METER});
                        break;
                    case 'c' + 'm':
                        mCam->setDesiredVelocity(RadiansPerSecond{velocity});
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

                mJointDe0->setDesiredVelocity(motorVelocities[0]);
                mJointDe1->setDesiredVelocity(motorVelocities[1]);
            }
        }

        auto processPositionCmd(msg::Position::ConstSharedPtr const& msg) -> void {
            if (msg->name.size() != msg->position.size()) {
                RCLCPP_ERROR(get_logger(), "Name count and value count mismatched!");
                return;
            }

            std::optional<Radians> jointDePitchPosition, jointDeRollPosition;

            for (std::size_t i = 0; i < msg->name.size(); ++i) {
                std::string const& name = msg->name[i];
                float const& position = msg->position[i];

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
                        mGripper->setDesiredPosition(Radians{position * GRIPPER_RADIANS_PER_METER});
                        break;
                    case 'c' + 'm':
                        mCam->setDesiredPosition(Radians{position});
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

            auto const pitchWrapped = wrapAngle((Radians{mJointDe0->getPosition()} - mJointDePitchOffset).get());
            auto const rollWrapped = wrapAngle((Radians{mJointDe1->getPosition()} - mJointDeRollOffset).get());
            mJointDePitchRoll = {pitchWrapped, rollWrapped};

            mJointData.position[3] = pitchWrapped;
            mJointData.velocity[3] = {RadiansPerSecond{mJointDe0->getVelocity()}.get()};
            mJointData.effort[3] = {mJointDe0->getEffort()};

            mJointData.position[4] = rollWrapped;
            mJointData.velocity[4] = {RadiansPerSecond{mJointDe1->getVelocity()}.get()};
            mJointData.effort[4] = {mJointDe1->getEffort()};

            mJointData.position[5] = {mGripper->getPosition().get() * GRIPPER_METERS_PER_RADIAN};
            mJointData.velocity[5] = {mGripper->getVelocity().get() * GRIPPER_METERS_PER_RADIAN};
            mJointData.effort[5] = {mGripper->getEffort()};

            mJointData.position[6] = {mCam->getPosition().get()};
            mJointData.velocity[6] = {mCam->getVelocity().get()};
            mJointData.effort[6] = {mCam->getEffort()};

            mJointDataPub->publish(mJointData);

            mControllerState.states[0] = {mJointA->getState()};
            mControllerState.errors[0] = {mJointA->getErrorState()};
            mControllerState.limits_hit[0] = {mJointA->getLimitsHitBits()};

            mControllerState.states[1] = {mJointB->getState()};
            mControllerState.errors[1] = {mJointB->getErrorState()};
            mControllerState.limits_hit[1] = {mJointB->getLimitsHitBits()};

            mControllerState.states[2] = {mJointC->getState()};
            mControllerState.errors[2] = {mJointC->getErrorState()};
            mControllerState.limits_hit[2] = {mJointC->getLimitsHitBits()};

            mControllerState.states[3] = {mJointDe0->getState()};
            mControllerState.errors[3] = {mJointDe0->getErrorState()};
            mControllerState.limits_hit[3] = {mJointDe0->getLimitsHitBits()};

            mControllerState.states[4] = {mJointDe1->getState()};
            mControllerState.errors[4] = {mJointDe1->getErrorState()};
            mControllerState.limits_hit[4] = {mJointDe1->getLimitsHitBits()};

            mControllerState.states[5] = {mGripper->getState()};
            mControllerState.errors[5] = {mGripper->getErrorState()};
            mControllerState.limits_hit[5] = {mGripper->getLimitsHitBits()};

            mControllerState.states[6] = {mCam->getState()};
            mControllerState.errors[6] = {mCam->getErrorState()};
            mControllerState.limits_hit[6] = {mCam->getLimitsHitBits()};

            mControllerStatePub->publish(mControllerState);
        }

        auto updateDeOffsets() -> void {
            if (!mJointDePitchRoll) return;

            Vector2<Radians> motorPositions = PITCH_ROLL_TO_01_SCALE * PITCH_ROLL_TO_0_1 * mJointDePitchRoll.value();
            mJointDe0->adjust(motorPositions[0]);
            mJointDe1->adjust(motorPositions[1]);
        }

        auto camIn() -> void {
            auto endTime = get_clock()->now() + mCamControlDuration;
            while (get_clock()->now() < endTime) {
                mCam->setDesiredThrottle(-1.0);
                std::this_thread::sleep_for(mCamControlPeriod);
            }
        }

        auto camOut() -> void {
            auto endTime = get_clock()->now() + mCamControlDuration;
            while (get_clock()->now() < endTime) {
                mCam->setDesiredThrottle(1.0);
                std::this_thread::sleep_for(mCamControlPeriod);
            }
        }

        auto camPulse() -> void {
            camOut();
            std::this_thread::sleep_for(mCamControlDuration);
            camIn();
        }

        auto camControlServiceCallback(srv::ControlCam::Request::ConstSharedPtr const& req, srv::ControlCam::Response::SharedPtr const& res) -> void {
            switch (req->action) {
                case srv::ControlCam::Request::CAM_IN: {
                    camIn();
                    res->success = true;
                    break;
                }
                case srv::ControlCam::Request::CAM_OUT: {
                    camOut();
                    res->success = true;
                    break;
                }
                case srv::ControlCam::Request::CAM_PULSE: {
                    camPulse();
                    res->success = true;
                    break;
                }
                default: {
                    res->success = false;
                    break;
                }
            }
        }
    };
} // namespace mrover

auto main(int argc, char** argv) -> int {
    rclcpp::init(argc, argv);
    auto arm_hw_bridge = std::make_shared<mrover::ArmHWBridge>();
    arm_hw_bridge->init();

    rclcpp::executors::MultiThreadedExecutor executor;
    executor.add_node(arm_hw_bridge);
    executor.spin();

    rclcpp::shutdown();
    return EXIT_SUCCESS;
}
