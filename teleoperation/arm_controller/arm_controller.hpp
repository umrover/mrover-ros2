#pragma once

#include "mrover/srv/detail/enable_auton__struct.hpp"
#include "mrover/srv/detail/ik_test__struct.hpp"
#include "pch.hpp"
#include <rclcpp/service.hpp>

namespace mrover {

    class ArmController final : public rclcpp::Node {

        [[maybe_unused]] rclcpp::Subscription<msg::IK>::SharedPtr mIkSub;
        rclcpp::Publisher<msg::Position>::SharedPtr mPosPub;
        tf2_ros::TransformBroadcaster mTfBroadcaster{this};
        tf2_ros::Buffer mTfBuffer{get_clock()};
        tf2_ros::TransformListener mTfListener{mTfBuffer};
        //rclcpp::Publisher<msg::ArmStatus>::SharedPtr mArmStatusPub;
        rclcpp::Service<srv::IkTest>::SharedPtr mIkServer;
        

    public:
        // TODO(quintin): Neven, please load these from config YAML files instead of hard coding. Ideally they would even be computed at runtime. This way you can change the xacro without worry.

        // From: rover.urdf.xacro
        // A is the prismatic joint, B is the first revolute joint, C is the second revolute joint
        static constexpr double LINK_BC = 0.5344417294;
        static constexpr double LINK_CD = 0.5531735368;
        static constexpr double LINK_DE = 0.044886000454425812;
        static constexpr double JOINT_A_MIN = 0;
        static constexpr double JOINT_A_MAX = 0.45;
        static constexpr double JOINT_B_MIN = -1. * std::numbers::pi/3.;
        static constexpr double JOINT_B_MAX = 0;
        static constexpr double JOINT_C_MIN = -0.959931;
        static constexpr double JOINT_C_MAX = 2.87979;
        static constexpr double JOINT_DE_PITCH_MIN = -0.71;
        static constexpr double JOINT_DE_PITCH_MAX = 0.97;
        static constexpr double END_EFFECTOR_LENGTH = 0.13; // Measured from blender

        ArmController();

        void ikCallback(msg::IK::ConstSharedPtr const& ik_target);
        void ikTestCallback(srv::IkTest_Request::ConstSharedPtr& req, srv::IkTest_Response::SharedPtr& res);
    };

} // namespace mrover