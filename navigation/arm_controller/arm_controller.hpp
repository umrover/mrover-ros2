#pragma once

#include "pch.hpp"

namespace mrover {

    class ArmController final : public rclcpp::Node {
        struct ArmPos {
            double x{0}, y{0}, z{0}, pitch{0}, roll{0};
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

        [[maybe_unused]] rclcpp::Subscription<msg::IK>::SharedPtr mIkSub;
        [[maybe_unused]] rclcpp::Subscription<geometry_msgs::msg::Twist>::SharedPtr mVelSub;
        [[maybe_unused]] rclcpp::Subscription<sensor_msgs::msg::JointState>::SharedPtr mJointSub;

        rclcpp::Publisher<msg::Position>::SharedPtr mPosPub;
        tf2_ros::TransformBroadcaster mTfBroadcaster{this};
        tf2_ros::Buffer mTfBuffer{get_clock()};
        tf2_ros::TransformListener mTfListener{mTfBuffer};
        rclcpp::TimerBase::SharedPtr mTimer;
        rclcpp::Service<srv::IkMode>::SharedPtr mModeServ;

        auto ikCalc(ArmPos target) -> std::optional<msg::Position>;
        auto timerCallback() -> void;

        ArmPos mArmPos;
        ArmPos mPosTarget = {0, -1, 0}; // THIS IS SCUFFED - set y target to -1 to make sure this is position is not reachable
        R3d mVelTarget = {0, 0, 0};
        double mPitchVel = 0.f;
        double mRollVel = 0.f;
        rclcpp::Time mLastUpdate; // TODO: consider making this negative or something? to resolve above issue

        enum class ArmMode : bool {
            VELOCITY_CONTROL,
            POSITION_CONTROL
        };
        ArmMode mArmMode = ArmMode::POSITION_CONTROL;
        static const rclcpp::Duration TIMEOUT;

    public:
        // TODO(quintin): Neven, please load these from config YAML files instead of hard coding. Ideally they would even be computed at runtime. This way you can change the xacro without worry.

        // From: rover.urdf.xacro
        // A is the prismatic joint, B is the first revolute joint, C is the second revolute joint
        static constexpr double LINK_BC = 0.5344417294;
        static constexpr double LINK_CD = 0.5531735368;
        static constexpr double LINK_DE = 0.044886000454425812;
        static constexpr double JOINT_A_MIN = 0;
        static constexpr double JOINT_A_MAX = 0.45;
        static constexpr double JOINT_B_MIN = -0.25 * std::numbers::pi;
        static constexpr double JOINT_B_MAX = 0;
        static constexpr double JOINT_C_MIN = -0.959931;
        static constexpr double JOINT_C_MAX = 2.87979;
        static constexpr double JOINT_DE_PITCH_MIN = -0.75 * std::numbers::pi;
        static constexpr double JOINT_DE_PITCH_MAX = 0.75 * std::numbers::pi;
        static constexpr double JOINT_DE_ROLL_MIN = -2.36;
        static constexpr double JOINT_DE_ROLL_MAX = 1.44;
        static constexpr double END_EFFECTOR_LENGTH = 0.13; // Measured from blender
        static constexpr double JOINT_C_OFFSET = 0.1608485915;

        ArmController();

        void ikCallback(msg::IK::ConstSharedPtr const& ik_target);
        void velCallback(geometry_msgs::msg::Twist::ConstSharedPtr const& ik_vel);
        void fkCallback(sensor_msgs::msg::JointState::ConstSharedPtr const& joint_state);
        auto modeCallback(srv::IkMode::Request::ConstSharedPtr const& req, srv::IkMode::Response::SharedPtr const& resp) -> void;
    };

} // namespace mrover