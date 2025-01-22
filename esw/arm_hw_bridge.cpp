#include <sensor_msgs/msg/detail/joint_state__struct.hpp>
#include <string>
#include <unordered_map>
#include <vector>

#include <rclcpp/rclcpp.hpp>

#include <units/units.hpp>
#include <units/units_eigen.hpp>

#include <mrover/msg/controller_state.hpp>
#include <mrover/msg/position.hpp>
#include <mrover/msg/throttle.hpp>
#include <mrover/msg/velocity.hpp>
#include <mrover/srv/adjust_motor.hpp>
#include <sensor_msgs/msg/joint_state.hpp>

#include "motor_library/brushed.hpp"
#include "motor_library/brushless.hpp"
#include "mrover/srv/detail/adjust_motor__struct.hpp"

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
            .absRatio = 1.0,
            .absOffset = 0.0_rad,

            .minVelocity = -1.0_rad_per_s,
            .maxVelocity = 1.0_rad_per_s,
            .minPosition = -0.7853981633974483_rad,
            .maxPosition = 0_rad,

            // .positionGains{},
            // .velocityGains{},

            .calibrationThrottle = 0.5,
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


    class ArmHWBridge : public rclcpp::Node {

        using Controller = std::variant<BrushedController, BrushlessController<Meters>, BrushlessController<Radians>>;

    public:
        ArmHWBridge() : rclcpp::Node{"arm_hw_bridge"} {
            // all initialization is done in the init() function to allow for the usage of shared_from_this()
        }

        auto init() -> void {
            mControllers.try_emplace("joint_b", std::in_place_type<BrushedController>, shared_from_this(), "jetson", "joint_b", JOINT_B_CONFIG);
            mControllers.try_emplace("gripper", std::in_place_type<BrushedController>, shared_from_this(), "jetson", "gripper", GRIPPER_CONFIG);
            mControllers.try_emplace("finger", std::in_place_type<BrushedController>, shared_from_this(), "jetson", "finger", BrushedController::Config{});
            mControllers.try_emplace("joint_a", std::in_place_type<BrushlessController<Meters>>, shared_from_this(), "jetson", "joint_a", BrushlessController<Meters>::Config{});
            mControllers.try_emplace("joint_c", std::in_place_type<BrushlessController<Radians>>, shared_from_this(), "jetson", "joint_c", BrushlessController<Radians>::Config{});
            mControllers.try_emplace("joint_de_0", std::in_place_type<BrushlessController<Radians>>, shared_from_this(), "jetson", "joint_de_0", BrushlessController<Radians>::Config{});
            mControllers.try_emplace("joint_de_1", std::in_place_type<BrushlessController<Radians>>, shared_from_this(), "jetson", "joint_de_1", BrushlessController<Radians>::Config{});

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
        std::vector<std::string> const mArmJointNames{"joint_a", "joint_b", "joint_c", "joint_de_pitch", "joint_de_roll", "allen_key", "gripper"};

        std::unordered_map<std::string, Controller> mControllers;
        // BrushedController mJointB;
        // BrushedController mGripper;
        // BrushedController mFinger; // rip the solenoid. tentative name until design finalized

        // BrushlessController<Meters> mJointA;
        // BrushlessController<Radians> mJointC;
        // BrushlessController<Radians> mJointDe0;
        // BrushlessController<Radians> mJointDe1;

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


        auto findJointByName(std::string const& name) -> Controller& {
            return mControllers.at(name);
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

                // Silly little thing to save some speed. Could easily just do the straight up string comparision
                switch (name.front() + name.back()) {
                    case 'j' + '0':
                        jointDePitchThrottle = throttle;
                        break;
                    case 'j' + '1':
                        jointDeRollThrottle = throttle;
                        break;
                    case 'j' + 'a':
                        std::get<BrushlessController<Meters>>(findJointByName("joint_a")).setDesiredThrottle(throttle);
                        break;
                    case 'j' + 'b':
                        std::get<BrushedController>(findJointByName("joint_b")).setDesiredThrottle(throttle);
                        break;
                    case 'j' + 'c':
                        std::get<BrushlessController<Radians>>(findJointByName("joint_c")).setDesiredThrottle(throttle);
                        break;
                    case 'g' + 'r':
                        std::get<BrushedController>(findJointByName("gripper")).setDesiredThrottle(throttle);
                        break;
                    case 'f' + 'r':
                        std::get<BrushedController>(findJointByName("finger")).setDesiredThrottle(throttle);
                        break;
                }
            }

            if (jointDePitchThrottle.has_value() && jointDeRollThrottle.has_value()) {
                Vector2<Dimensionless> const pitchRollThrottles{jointDePitchThrottle.value(), jointDeRollThrottle.value()};
                Vector2<Dimensionless> const motorThrottles = PITCH_ROLL_TO_0_1 * pitchRollThrottles;

                std::get<BrushlessController<Radians>>(findJointByName("joint_de_0")).setDesiredThrottle(motorThrottles[0].get());
                std::get<BrushlessController<Radians>>(findJointByName("joint_de_1")).setDesiredThrottle(motorThrottles[1].get());
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
                    case 'j' + '0':
                        jointDePitchVelocity = RadiansPerSecond{velocity};
                        break;
                    case 'j' + '1':
                        jointDeRollVelocity = RadiansPerSecond{velocity};
                        break;
                    case 'j' + 'a':
                        std::get<BrushlessController<Meters>>(findJointByName("joint_a")).setDesiredVelocity(MetersPerSecond{velocity});
                        break;
                    case 'j' + 'b':
                        std::get<BrushedController>(findJointByName("joint_b")).setDesiredVelocity(RadiansPerSecond{velocity});
                        break;
                    case 'j' + 'c':
                        std::get<BrushlessController<Radians>>(findJointByName("joint_c")).setDesiredVelocity(RadiansPerSecond{velocity});
                        break;
                    case 'g' + 'r':
                        std::get<BrushedController>(findJointByName("gripper")).setDesiredVelocity(RadiansPerSecond{velocity});
                        break;
                    case 'f' + 'r':
                        std::get<BrushedController>(findJointByName("finger")).setDesiredVelocity(RadiansPerSecond{velocity});
                        break;
                }
            }

            if (jointDePitchVelocity.has_value() && jointDeRollVelocity.has_value()) {
                Vector2<RadiansPerSecond> const pitchRollVelocities{jointDePitchVelocity.value(), jointDeRollVelocity.value()};
                Vector2<RadiansPerSecond> const motorVelocities = PITCH_ROLL_TO_0_1 * pitchRollVelocities;

                std::get<BrushlessController<Radians>>(findJointByName("joint_de_0")).setDesiredVelocity(RadiansPerSecond{motorVelocities[0].get()});
                std::get<BrushlessController<Radians>>(findJointByName("joint_de_1")).setDesiredVelocity(RadiansPerSecond{motorVelocities[1].get()});
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
                    case 'j' + '0':
                        jointDePitchPosition = Radians{position};
                        break;
                    case 'j' + '1':
                        jointDeRollPosition = Radians{position};
                        break;
                    case 'j' + 'a':
                        std::get<BrushlessController<Meters>>(findJointByName("joint_a")).setDesiredPosition(Meters{position});
                        break;
                    case 'j' + 'b':
                        std::get<BrushedController>(findJointByName("joint_b")).setDesiredPosition(Radians{position});
                        break;
                    case 'j' + 'c':
                        std::get<BrushlessController<Radians>>(findJointByName("joint_c")).setDesiredPosition(Radians{position});
                        break;
                    case 'g' + 'r':
                        std::get<BrushedController>(findJointByName("gripper")).setDesiredPosition(Radians{position});
                        break;
                    case 'f' + 'r':
                        std::get<BrushedController>(findJointByName("finger")).setDesiredPosition(Radians{position});
                        break;
                }

                if (jointDePitchPosition.has_value() && jointDeRollPosition.has_value()) {
                    Vector2<Radians> const pitchRollPositions{jointDePitchPosition.value(), jointDeRollPosition.value()};
                    Vector2<Radians> motorPositions = PITCH_ROLL_TO_01_SCALE * PITCH_ROLL_TO_0_1 * pitchRollPositions;

                    std::get<BrushlessController<Radians>>(findJointByName("joint_de_0")).setDesiredPosition(Radians{motorPositions[0].get()});
                    std::get<BrushlessController<Radians>>(findJointByName("joint_de_1")).setDesiredPosition(Radians{motorPositions[1].get()});
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

            auto& jointA = std::get<BrushlessController<Meters>>(findJointByName("joint_a"));
            mJointData.position[0] = jointA.getPosition().get();
            mJointData.velocity[0] = jointA.getVelocity().get();
            mJointData.effort[0] = jointA.getEffort();

            auto& jointB = std::get<BrushedController>(findJointByName("joint_b"));
            mJointData.position[1] = jointB.getPosition().get();
            mJointData.velocity[1] = jointB.getVelocity().get();
            mJointData.effort[1] = jointB.getEffort();

            auto& jointC = std::get<BrushlessController<Radians>>(findJointByName("joint_c"));
            mJointData.position[2] = jointC.getPosition().get();
            mJointData.velocity[2] = jointC.getVelocity().get();
            mJointData.effort[2] = jointC.getEffort();

            auto& jointDe0 = std::get<BrushlessController<Radians>>(findJointByName("joint_de_0"));
            auto& jointDe1 = std::get<BrushlessController<Radians>>(findJointByName("joint_de_1"));
            auto pitchWrapped = wrapAngle(jointDe0.getPosition().get());
            auto rollWrapped = wrapAngle(jointDe1.getPosition().get());
            mJointDePitchRoll = {pitchWrapped, rollWrapped};

            mJointData.position[3] = pitchWrapped;
            mJointData.velocity[3] = jointDe0.getVelocity().get();
            mJointData.effort[3] = jointDe0.getEffort();

            mJointData.position[4] = rollWrapped;
            mJointData.velocity[4] = jointDe1.getVelocity().get();
            mJointData.effort[4] = jointDe1.getEffort();

            auto& gripper = std::get<BrushedController>(findJointByName("gripper"));
            mJointData.position[5] = gripper.getPosition().get();
            mJointData.velocity[5] = gripper.getVelocity().get();
            mJointData.effort[5] = gripper.getEffort();

            auto& finger = std::get<BrushedController>(findJointByName("finger"));
            mJointData.position[6] = finger.getPosition().get();
            mJointData.velocity[6] = finger.getVelocity().get();
            mJointData.effort[6] = finger.getEffort();

            mJointDataPub->publish(mJointData);

            mControllerState.state[0] = jointA.getState();
            mControllerState.error[0] = jointA.getErrorState();
            mControllerState.limit_hit[0] = jointA.getLimitsHitBits();

            mControllerState.state[1] = jointB.getState();
            mControllerState.error[1] = jointB.getErrorState();
            mControllerState.limit_hit[1] = jointB.getLimitsHitBits();

            mControllerState.state[2] = jointC.getState();
            mControllerState.error[2] = jointC.getErrorState();
            mControllerState.limit_hit[2] = jointC.getLimitsHitBits();

            mControllerState.state[3] = jointDe0.getState();
            mControllerState.error[3] = jointDe0.getErrorState();
            mControllerState.limit_hit[3] = jointDe0.getLimitsHitBits();

            mControllerState.state[4] = jointDe1.getState();
            mControllerState.error[4] = jointDe1.getErrorState();
            mControllerState.limit_hit[4] = jointDe1.getLimitsHitBits();

            mControllerState.state[5] = gripper.getState();
            mControllerState.error[5] = gripper.getErrorState();
            mControllerState.limit_hit[5] = gripper.getLimitsHitBits();

            mControllerState.state[6] = finger.getState();
            mControllerState.error[6] = finger.getErrorState();
            mControllerState.limit_hit[6] = finger.getLimitsHitBits();

            mControllerStatePub->publish(mControllerState);
        }

        auto updateDeOffsets() -> void {
            if (!mJointDePitchRoll) return;

            Vector2<Radians> motorPositions = PITCH_ROLL_TO_01_SCALE * PITCH_ROLL_TO_0_1 * mJointDePitchRoll.value();
            {
                auto adjust = std::make_shared<srv::AdjustMotor::Request>();
                adjust->name = "joint_de_0";
                adjust->value = motorPositions[0].get();
                mJointDe0AdjustClient->async_send_request(adjust);
            }
            {
                auto adjust = std::make_shared<srv::AdjustMotor::Request>();
                adjust->name = "joint_de_1";
                adjust->value = motorPositions[1].get();
                mJointDe0AdjustClient->async_send_request(adjust);
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
