#pragma once
#include "mrover/srv/detail/align_lander__struct.hpp"
#include "pch.hpp"
#include <geometry_msgs/msg/detail/pose__struct.hpp>
#include <rclcpp/node.hpp>
#include <rclcpp/publisher.hpp>
#include <rclcpp/service.hpp>
#include <std_msgs/msg/detail/float32__struct.hpp>
#include <std_msgs/msg/float32.hpp>
#include <rclcpp/logging.hpp>
#include <mrover/srv/align_lander.hpp>
namespace mrover {
    class LanderAlignNode final : public rclcpp::Node{
        public:
        // Statics
        static constexpr float Z_NORM_MAX = 0.15;
        static constexpr float FAR_CLIP = 10;
        static constexpr float NEAR_CLIP = 0.5;
        static constexpr bool DEBUG_PC = true;

        explicit LanderAlignNode(rclcpp::NodeOptions const& options = rclcpp::NodeOptions());

        ~LanderAlignNode() override = default;

        void filterNormals();

        void uploadDebugPC();

        private:
        tf2_ros::Buffer mTfBuffer{get_clock()};
        tf2_ros::TransformListener mTfListener{mTfBuffer};
        std::shared_ptr<tf2_ros::TransformBroadcaster> mTfBroadcaster = std::make_shared<tf2_ros::TransformBroadcaster>(this);
        std::vector<Point> mDebugPC;
        rclcpp::Publisher<sensor_msgs::msg::PointCloud2>::SharedPtr mDebugPub;


        // Service stuff
        rclcpp::Service<mrover::srv::AlignLander>::SharedPtr mLanderService;
        auto startAlignCallback(mrover::srv::AlignLander::Request::ConstSharedPtr& req, mrover::srv::AlignLander::Response::SharedPtr& res) -> void;

        geometry_msgs::msg::Pose mFinalAngle;

        // subscribers/publishers
        rclcpp::Publisher<std_msgs::msg::Float32>::SharedPtr mCostMapPub;
        rclcpp::Subscription<sensor_msgs::msg::PointCloud2>::SharedPtr mPcSub;

        void pointCloudCallback(sensor_msgs::msg::PointCloud2::ConstSharedPtr const& msg);

        // Callback vars
        bool mRunCallback = true;

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