#pragma once

#include "pch.hpp"
#include <sensor_msgs/msg/detail/point_cloud2__struct.hpp>

namespace mrover {

    class CostMapNode final : public rclcpp::Node {

        constexpr static std::int8_t UNKNOWN_COST = -1, FREE_COST = 0, OCCUPIED_COST = 100;

        constexpr static double TAU = 2 * std::numbers::pi;

        // Dilation for map, set as number of bins away from object to dilate by (default 1)
        constexpr static int dilation = 1;

		// Noise/Debug Vars
		constexpr static bool useNoisyPointCloud = false;
		constexpr static bool uploadDebugPointCloud = true;
		rclcpp::Publisher<sensor_msgs::msg::PointCloud2>::SharedPtr mPCDebugPub;
		std::vector<Point> mInliers;

        rclcpp::Publisher<nav_msgs::msg::OccupancyGrid>::SharedPtr mCostMapPub;
        rclcpp::Subscription<sensor_msgs::msg::PointCloud2>::SharedPtr mPcSub;

        // TODO(quintin): This is a hack
        // ros::Subscriber mImuSub;
        // std::optional<ros::Time> mLastImuTime;

        double mZPercent{}, mZThreshold{};
        double mAlpha{};
        double mNearClip{}, mFarClip{}, mLeftClip{}, mRightClip{}, mTopClip{};
        double mResolution{}; // Meters per cell
        double mSize{};       // Size of the square costmap in meters
        int mWidth{};         // Number of cells on the grid horizontally
        int mHeight{};        // Number of cells on the grid vertically 
        int mDownSamplingFactor = 4;
        std::string mMapFrame;
        int mDilateAmt = 2;

        // Loop timing stuff
        // LoopProfiler mLoopProfilerGrab;
        // LoopProfiler mLoopProfilerUpdate;
		
        tf2_ros::Buffer mTfBuffer{get_clock()};
        tf2_ros::TransformListener mTfListener{mTfBuffer};

        std::optional<SE3d> mPreviousPose;
        nav_msgs::msg::OccupancyGrid mGlobalGridMsg;

        rclcpp::Service<mrover::srv::MoveCostMap>::SharedPtr mServer;

        rclcpp::Service<mrover::srv::DilateCostMap>::SharedPtr mCostServer;

        struct BinEntry {
                R3f pointInCamera;
                R3f pointInMap;
            };

        using Bin = std::vector<BinEntry>;

    public:
        explicit CostMapNode(rclcpp::NodeOptions const& options = rclcpp::NodeOptions());

        ~CostMapNode() override = default;

        auto pointCloudCallback(sensor_msgs::msg::PointCloud2::ConstSharedPtr const& msg) -> void;

        auto moveCostMapCallback(mrover::srv::MoveCostMap::Request::ConstSharedPtr& req, mrover::srv::MoveCostMap::Response::SharedPtr& res) -> void;

        // To Test (CLI): ros2 service call /dilate_cost_map mrover/srv/DilateCostMap "{d_amt: [enter here]}"
        auto dilateCostMapCallback(mrover::srv::DilateCostMap::Request::ConstSharedPtr& req, mrover::srv::DilateCostMap::Response::SharedPtr& res) -> void;

		void uploadPC();

        // CUDA functions
        // auto fillBinsGPU(sensor_msgs::msg::PointCloud2::ConstSharedPtr const& msg) -> void;
        // auto checkCudaError(cudaError_t err) -> void;

        // Bin vector coordinate
        struct Coordinate{
            int row;
            int col;

            auto operator-(Coordinate& other) const->Coordinate{
                return {row - other.row, col - other.col};
            }

            auto operator+(Coordinate& other) const->Coordinate{
                return {row + other.row, col + other.col};
            }
        };

        constexpr auto diArray()->std::array<CostMapNode::Coordinate,(2*dilation+1)*(2*dilation+1)>;

        auto indexToCoordinate(int index) const -> Coordinate;
        auto coordinateToIndex(Coordinate c) const -> int;
    };

} // namespace mrover
