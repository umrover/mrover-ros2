#pragma once

#include "pch.hpp"

namespace mrover {

    class ArmController final : public rclcpp::Node {

        [[maybe_unused]] rclcpp::Subscription<msg::IK>::SharedPtr mIkSub;
        [[maybe_unused]] rclcpp::Subscription<geometry_msgs::msg::Vector3>::SharedPtr mVelSub;
        [[maybe_unused]] rclcpp::Subscription<sensor_msgs::msg::JointState>::SharedPtr mJointSub;

        rclcpp::Publisher<msg::Position>::SharedPtr mPosPub;
        tf2_ros::TransformBroadcaster mTfBroadcaster{this};
        tf2_ros::Buffer mTfBuffer{get_clock()};
        tf2_ros::TransformListener mTfListener{mTfBuffer};
        rclcpp::TimerBase::SharedPtr mTimer;
        rclcpp::Service<srv::IkMode>::SharedPtr mModeServ;

        auto ikCalc(SE3d target) -> std::optional<msg::Position>;
        auto timerCallback() -> void;
        auto modeCallback(srv::IkMode::Request::ConstSharedPtr const& req, srv::IkMode::Response::SharedPtr const& resp) -> void;

        SE3d mArmPos;
        SE3d mPosTarget;
        R3d mVelTarget = {0, 0, 0};
        rclcpp::Time mLastUpdate;

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
        static constexpr double END_EFFECTOR_LENGTH = 0.13; // Measured from blender
        static constexpr double JOINT_C_OFFSET = 0.1608485915;

        ArmController();

        void ikCallback(msg::IK::ConstSharedPtr const& ik_target);
        void velCallback(geometry_msgs::msg::Vector3::ConstSharedPtr const& ik_vel);
        void fkCallback(sensor_msgs::msg::JointState::ConstSharedPtr const& joint_state);
    };

} // namespace mrover