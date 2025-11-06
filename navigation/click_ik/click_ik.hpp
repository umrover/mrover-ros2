#pragma once

#include "mrover/action/detail/click_ik__struct.hpp"
#include "pch.hpp"
#include <mrover/msg/arm_status.hpp>
#include <mrover/msg/ik.hpp>
#include <rclcpp/publisher.hpp>
#include <rclcpp_action/server.hpp>
#include <rclcpp_action/types.hpp>
#include <tf2_ros/transform_listener.h>

namespace mrover {

    class ClickIkNode final : public rclcpp::Node {
        rclcpp::Subscription<sensor_msgs::msg::PointCloud2>::ConstSharedPtr mPcSub;
        rclcpp::Publisher<msg::IK>::SharedPtr mIkPub;
        rclcpp::Subscription<msg::ArmStatus>::ConstSharedPtr mStatusSub;

        rclcpp_action::Server<action::ClickIk>::SharedPtr server;

        msg::IK message;
        rclcpp::TimerBase::SharedPtr timer;

        std::shared_ptr<rclcpp_action::ServerGoalHandle<action::ClickIk>> mCurrentGoalHandle;

        Point const* mPoints{};
        std::size_t mNumPoints{};
        std::size_t mPointCloudWidth{};
        std::size_t mPointCloudHeight{};

        std::unique_ptr<tf2_ros::Buffer> mTfBuffer = std::make_unique<tf2_ros::Buffer>(get_clock());
        std::shared_ptr<tf2_ros::TransformListener> mTfListener = std::make_shared<tf2_ros::TransformListener>(*mTfBuffer);
        std::shared_ptr<tf2_ros::TransformBroadcaster> mTfBroadcaster = std::make_shared<tf2_ros::TransformBroadcaster>(this);
    public:

        void executeClickIk(std::shared_ptr<rclcpp_action::ServerGoalHandle<action::ClickIk>> goal_handle);
        
        explicit ClickIkNode(rclcpp::NodeOptions const& options = rclcpp::NodeOptions());

        ~ClickIkNode() override = default;

        auto pointCloudCallback(sensor_msgs::msg::PointCloud2::ConstSharedPtr const& msg) -> void;
        auto statusCallback(msg::ArmStatus const& msg) -> void;

        auto cancelClickIk() -> void;

        //Taken line for line from percep object detection code
        auto spiralSearchInImg(size_t xCenter, size_t yCenter) -> std::optional<Point>;
    };

} // namespace mrover
