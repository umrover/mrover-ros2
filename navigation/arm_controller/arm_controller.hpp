#pragma once

#include "pch.hpp"

namespace mrover {

    class ArmController final : public rclcpp::Node {

        [[maybe_unused]] rclcpp::Subscription<msg::IK>::SharedPtr mIkSub;
        rclcpp::Publisher<msg::Position>::SharedPtr mPosPub;
        tf2_ros::TransformBroadcaster mTfBroadcaster{this};
        tf2_ros::Buffer mTfBuffer{get_clock()};
        tf2_ros::TransformListener mTfListener{mTfBuffer};

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

        ArmController();

        void ikCallback(msg::IK::ConstSharedPtr const& ik_target);
    };

} // namespace mrover