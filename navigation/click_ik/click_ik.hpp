#pragma once

#include "mrover/msg/detail/arm_status__struct.hpp"
#include "mrover/msg/detail/ik__struct.hpp"
#include "pch.hpp"
#include <mrover/msg/arm_status.hpp>
#include <mrover/msg/ik.hpp>
#include <rclcpp/publisher.hpp>
#include <sensor_msgs/msg/detail/point_cloud2__struct.hpp>
#include <std_msgs/msg/detail/bool__struct.hpp>


namespace mrover {

    class ClickIkNode final : public rclcpp::Node {
        rclcpp::Subscription<sensor_msgs::msg::PointCloud2::ConstSharedPtr>::SharedPtr mPcSub;
        rclcpp::Publisher<msg::IK>::SharedPtr mIKPub;
        rclcpp::Subscription<msg::ArmStatus>::SharedPtr mStatusSub;

        // ros::Subscriber mPcSub;
        // ros::Publisher mIkPub;
        rclcpp_action::Server<mrover::ClickIkAction> server = rclcpp::SimpleActionServer<mrover::ClickIkAction>(mNh, "do_click_ik", false);

        IK message;
        ros::Timer timer;

        Point const* mPoints{};
        std::size_t mNumPoints{};
        std::size_t mPointCloudWidth{};
        std::size_t mPointCloudHeight{};

        tf2_ros::Buffer mTfBuffer{};
        tf2_ros::TransformListener mTfListener{mTfBuffer};
        tf2_ros::TransformBroadcaster mTfBroadcaster;

    public:
        explicit ClickIkNode(rclcpp::NodeOptions const& options = rclcpp::NodeOptions());

        ~ClickIkNode() override = default;

        auto pointCloudCallback(sensor_msgs::msg::PointCloud2::ConstSharedPtr const& msg) -> void;
        auto statusCallback(mrover::msg::ArmStatus const& msg) -> void;

        auto startClickIk() -> void;
        auto cancelClickIk() -> void;

        //Taken line for line from percep object detection code
        auto spiralSearchInImg(size_t xCenter, size_t yCenter) -> std::optional<Point>;
    };

} // namespace mrover
