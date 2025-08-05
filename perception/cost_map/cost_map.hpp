#pragma once

#include "pch.hpp"

namespace mrover {

    class CostMapNode final : public rclcpp::Node {

        constexpr static std::int8_t UNKNOWN_COST = -1, FREE_COST = 0, OCCUPIED_COST = 100;

        constexpr static double TAU = 2 * std::numbers::pi;

        static double inflation_radius;

        rclcpp::Publisher<nav_msgs::msg::OccupancyGrid>::SharedPtr mCostMapPub;
        rclcpp::Subscription<sensor_msgs::msg::PointCloud2>::SharedPtr mPcSub;

        // TODO(quintin): This is a hack
        // ros::Subscriber mImuSub;
        // std::optional<ros::Time> mLastImuTime;

        double mZPercent{}, mZThreshold{};
        double mAlpha{};
        double mNearClip{}, mFarClip{};
        double mResolution{}; // Meters per cell
        double mSize{};       // Size of the square costmap in meters
        int mDownSamplingFactor = 4;
        std::string mMapFrame;

        tf2_ros::Buffer mTfBuffer{get_clock()};
        tf2_ros::TransformListener mTfListener{mTfBuffer};

        std::optional<SE3d> mPreviousPose;
        nav_msgs::msg::OccupancyGrid mGlobalGridMsg;

        rclcpp::Service<mrover::srv::MoveCostMap>::SharedPtr mServer;

        rclcpp::Service<mrover::srv::DilateCostMap>::SharedPtr dCostServer;

    public:
        CostMapNode();

        ~CostMapNode() override = default;

        auto pointCloudCallback(sensor_msgs::msg::PointCloud2::ConstSharedPtr const& msg) -> void;

        auto moveCostMapCallback(mrover::srv::MoveCostMap::Request::ConstSharedPtr& req, mrover::srv::MoveCostMap::Response::SharedPtr& res) -> void;

        auto dilateCostMapCallback(mrover::srv::DilateCostMap_Request::ConstSharedPtr& req, mrover::srv::DilateCostMap_Response::SharedPtr& res) -> void;
    };

} // namespace mrover