#pragma once

#include "pch.hpp"

namespace mrover {
    class LanderAnalysis : public rclcpp::Node {
    private:
        // node name
        static constexpr char const* NODE_NAME = "lander_analysis";

        // TF
        std::unique_ptr<tf2_ros::Buffer> mTfBuffer = std::make_unique<tf2_ros::Buffer>(get_clock());
        std::shared_ptr<tf2_ros::TransformBroadcaster> mTfBroadcaster = std::make_shared<tf2_ros::TransformBroadcaster>(this);
        std::shared_ptr<tf2_ros::TransformListener> mTfListener = std::make_shared<tf2_ros::TransformListener>(*mTfBuffer);

        // subscribers
        rclcpp::Subscription<sensor_msgs::msg::PointCloud2>::SharedPtr mPcSub;

        // publishers
        rclcpp::Publisher<sensor_msgs::msg::PointCloud2>::SharedPtr mPCDebugPub;

        auto pointCloudCallback(sensor_msgs::msg::PointCloud2::ConstSharedPtr const& msg) -> void;

        void uploadPC(Eigen::MatrixXd const& points);

    public:
        explicit LanderAnalysis(rclcpp::NodeOptions const& options = rclcpp::NodeOptions());

        ~LanderAnalysis() override = default;
    };
}
