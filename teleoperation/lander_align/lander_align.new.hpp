#pragma once
#include "mrover/srv/detail/align_lander__struct.hpp"
#include "pch.hpp"
#include <geometry_msgs/msg/detail/pose__struct.hpp>
#include <rclcpp/node.hpp>
#include <rclcpp/service.hpp>
#include <std_msgs/msg/detail/float32__struct.hpp>
#include <std_msgs/msg/float32.hpp>
#include <rclcpp/logging.hpp>
#include <mrover/action/lander_align.hpp>
#include <mrover/srv/align_lander.hpp>
namespace mrover {
    class LanderAlignNode final : public rclcpp::Node{
        public:
        using LanderAlign = action::LanderAlign;
        using GoalHandleLanderAlign = rclcpp_action::ServerGoalHandle<LanderAlign>;

        // Statics
        static constexpr float Z_NORM_MAX = 0.15;
        static constexpr float FAR_CLIP = 10;
        static constexpr float NEAR_CLIP = 0.5;

        explicit LanderAlignNode(rclcpp::NodeOptions const& options = rclcpp::NodeOptions());

        ~LanderAlignNode() override = default;

        void filterNormals();

        private:
        tf2_ros::Buffer mTfBuffer{get_clock()};
        std::shared_ptr<tf2_ros::TransformBroadcaster> mTfBroadcaster = std::make_shared<tf2_ros::TransformBroadcaster>(this);

        // Service stuff
        rclcpp::Service<mrover::srv::AlignLander>::SharedPtr mLanderService;
        auto startAlignCallback(mrover::srv::AlignLander::Request::ConstSharedPtr& req, mrover::srv::AlignLander::Response::SharedPtr& res) -> void;

        geometry_msgs::msg::Pose mFinalAngle;

        // subscribers/publishers
        rclcpp::Publisher<std_msgs::msg::Float32>::SharedPtr mCostMapPub;
        rclcpp::Subscription<sensor_msgs::msg::PointCloud2>::SharedPtr mPcSub;
        rclcpp::Publisher<sensor_msgs::msg::PointCloud2>::SharedPtr mPCDebugPub;

        void pointCloudCallback(sensor_msgs::msg::PointCloud2::ConstSharedPtr & msg);

        // Callback vars
        bool mRunCallback = false;

        /*
        Using PCA to calculate plane:
        1. Initial filtering of points by removing any normal with z-component above some threshold (plane normals will be
                relatively flat)

        2. Filter out additional points that are some threshold above the median of all points (75th %ile?)
        3. Once filtered, calculate mean and subtract it off from all points
        4. Do PCA and find normal
        5. Keep doing to find better planes?
        */
    };
}