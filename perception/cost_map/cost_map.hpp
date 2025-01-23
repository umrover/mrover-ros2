#pragma once

#include "pch.hpp"
#include <vector>

namespace mrover {

    class CostMapNode final : public rclcpp::Node {

        constexpr static std::int8_t UNKNOWN_COST = -1, FREE_COST = 0, OCCUPIED_COST = 100;

        constexpr static double TAU = 2 * std::numbers::pi;

		// Perceptron Alg
		float x0 = 0;
		float x1 = 0;
		float x2 = 0;
		float x3 = 0;
		float x4 = 0;
		float x5 = 0;

		// Noise/Debug Vars
		constexpr static bool useNoisyPointCloud = false;
		constexpr static bool uploadDebugPointCloud = true;
		rclcpp::Publisher<sensor_msgs::msg::PointCloud2>::SharedPtr mPCDebugPub;
		std::vector<Point> mInliers;

		// TF Variables
        tf2_ros::Buffer mTfBuffer{get_clock()};
        tf2_ros::TransformBroadcaster mTfBroadcaster{this};
        tf2_ros::TransformListener mTfListener{mTfBuffer};

        rclcpp::Publisher<nav_msgs::msg::OccupancyGrid>::SharedPtr mCostMapPub;
        rclcpp::Subscription<sensor_msgs::msg::PointCloud2>::SharedPtr mPcSub;
        rclcpp::Publisher<sensor_msgs::msg::PointCloud2>::SharedPtr mPcPub;

        double mZPercent{}, mZThreshold{};
        double mAlpha{};
        double mNearClip{}, mFarClip{}, mLeftClip{}, mRightClip{};
        double mResolution{}; // Meters per cell
        double mSize{};       // Size of the square costmap in meters
        int mDownSamplingFactor = 4;
        std::string mMapFrame;
		
        std::optional<SE3d> mPreviousPose;
        nav_msgs::msg::OccupancyGrid mGlobalGridMsg;

        rclcpp::Service<mrover::srv::MoveCostMap>::SharedPtr mServer;

        struct BinEntry {
                R3f pointInCamera;
                R3f pointInMap;
            };

            using Bin = std::vector<BinEntry>;

    public:
        CostMapNode();

        ~CostMapNode() override = default;

        auto pointCloudCallback(sensor_msgs::msg::PointCloud2::UniquePtr const& inputMsg) -> void;

        auto moveCostMapCallback(mrover::srv::MoveCostMap::Request::ConstSharedPtr& req, mrover::srv::MoveCostMap::Response::SharedPtr& res) -> void;

		void uploadPC();

        auto processHeight(mrover::srv::MoveCostMap::Request::ConstSharedPtr& req, mrover::srv::MoveCostMap::Response::SharedPtr& res, std::vector<Bin> bins) -> void;
    };

} // namespace mrover
