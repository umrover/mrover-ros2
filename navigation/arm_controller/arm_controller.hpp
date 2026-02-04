#pragma once
#include "pch.hpp"

namespace mrover {

    class ArmController final : public rclcpp::Node {
        struct ArmPos {
            double x{0}, y{0}, z{0}, pitch{0}, roll{0}, gripper{0};
            [[nodiscard]] auto toSE3() const -> SE3d {
                return SE3d{{x, y, z,}, SO3d{Eigen::Quaterniond{Eigen::AngleAxisd{pitch, R3d::UnitY()} * Eigen::AngleAxisd{roll, R3d::UnitX()}}}};
            }

            auto operator+(R3d offset) const -> ArmPos {
                return {x + offset.x(), y + offset.y(), z + offset.z(), pitch, roll};
            }

            auto operator=(msg::IK ik_target) -> ArmPos& {
                x = ik_target.pos.x;
                y = ik_target.pos.y;
                z = ik_target.pos.z;
                pitch = ik_target.pitch;
                roll = ik_target.roll;
                return *this;
            }
        };


        struct JointWrapper {
            struct JointLimits {
                double minPos, maxPos, minVel, maxVel;
                [[nodiscard]] auto posInBounds(double pos) const -> bool {return minPos <= pos && pos <= maxPos;}
                [[nodiscard]] auto velInBounds(double vel) const -> bool {return minPos <= vel && vel <= maxPos;}
            };
            
            JointLimits limits;
            double pos;
        };
        
        // TODO: update velocity limits to make them real
        std::unordered_map<std::string, JointWrapper> joints = {
            {"joint_a", {
                .limits = {.minPos = 0, .maxPos = 0.35, .minVel = -0.05, .maxVel = 0.05},
                .pos = 0
            }},
            {"joint_b", {
                .limits = {.minPos = -0.9, .maxPos = 0, .minVel = -0.05, .maxVel = 0.05},
                .pos = 0
            }},
            {"joint_c", {
                .limits = {.minPos = -0.959931, .maxPos = 2.87979, .minVel = -0.05 * 2 * std::numbers::pi, .maxVel = 0.05 * 2 * std::numbers::pi},
                .pos = 0
            }},
            {"joint_de_pitch", {
                .limits = {.minPos = -1.3, .maxPos = 1.2, .minVel = -0.2, .maxVel = 0.2}, // pretty conservative limits atm
                .pos = 0
            }},
            {"joint_de_roll", {
                .limits = {.minPos = -2.36, .maxPos = 1.44, .minVel = -1, .maxVel = 1},
                .pos = 0
            }},
            {"gripper", {
                .limits = {.minPos = 0, .maxPos = 0.1, .minVel = -1, .maxVel = 1},
                .pos = 0
            }},
        };

        [[maybe_unused]] rclcpp::Subscription<msg::IK>::SharedPtr mIkSub;
        [[maybe_unused]] rclcpp::Subscription<geometry_msgs::msg::Twist>::SharedPtr mVelSub;
        [[maybe_unused]] rclcpp::Subscription<sensor_msgs::msg::JointState>::SharedPtr mJointSub;

        rclcpp::Publisher<msg::Position>::SharedPtr mPosPub;
        rclcpp::Publisher<msg::Velocity>::SharedPtr mVelPub;
        tf2_ros::TransformBroadcaster mTfBroadcaster{this};
        tf2_ros::Buffer mTfBuffer{get_clock()};
        tf2_ros::TransformListener mTfListener{mTfBuffer};
        rclcpp::TimerBase::SharedPtr mTimer;
        rclcpp::Service<srv::IkMode>::SharedPtr mModeServ;

        auto ikPosCalc(ArmPos target) -> std::optional<msg::Position>;
        auto ikVelCalc(geometry_msgs::msg::Twist) -> std::optional<msg::Velocity>;
        auto timerCallback() -> void;

        ArmPos mArmPos, mTypingOrigin, mPosTarget;
        geometry_msgs::msg::Twist mVelTarget;
        rclcpp::Time mLastUpdate;

        enum class ArmMode : char {
            VELOCITY_CONTROL,
            POSITION_CONTROL,
            TYPING
        };
        ArmMode mArmMode = ArmMode::POSITION_CONTROL;
        static const rclcpp::Duration TIMEOUT;

    public:
        // TODO(quintin): Neven, please load these from config YAML files instead of hard coding. Ideally they would even be computed at runtime. This way you can change the xacro without worry.

        // From: rover.urdf.xacro
        // A is the prismatic joint, B is the first revolute joint, C is the second revolute joint
        static constexpr double LINK_BC = 0.5344417294;
        static constexpr double LINK_CD = 0.5531735368;
        static constexpr double END_EFFECTOR_LENGTH = 0.20482814; // from CAD
        static constexpr double JOINT_C_OFFSET = 0.1608485915;
        static constexpr double JOINT_VEL_THRESH = 0.05;
        static constexpr double MAX_SPEED = 0.1; // in m/s, this is just an estimate

        ArmController();

        void posCallback(msg::IK::ConstSharedPtr const& ik_target);
        void velCallback(geometry_msgs::msg::Twist::ConstSharedPtr const& ik_vel);
        void fkCallback(sensor_msgs::msg::JointState::ConstSharedPtr const& joint_state);
        auto modeCallback(srv::IkMode::Request::ConstSharedPtr const& req, srv::IkMode::Response::SharedPtr const& resp) -> void;
    };

} // namespace mrover
