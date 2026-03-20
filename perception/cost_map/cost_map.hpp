#pragma once

#include "pch.hpp"

namespace mrover {

    class CostMapNode final : public rclcpp::Node {

        constexpr static std::int8_t UNKNOWN_COST = -1, FREE_COST = 0, OCCUPIED_COST = 100, THRESHOLD_COST = 20, DILATED_COST = 10;

        constexpr static double TAU = 2 * std::numbers::pi;

        // Dilation for map, set as number of bins away from object to dilate by (default 1)
        //     Every dilation pass dilates an additional [dilation] number of cells
        constexpr static int dilation = 1;

        // Noise/Debug Vars
        constexpr static bool useNoisyPointCloud = false;
        constexpr static bool uploadDebugPointCloud = true;
        rclcpp::Publisher<sensor_msgs::msg::PointCloud2>::SharedPtr mPCDebugPub;
        std::vector<Point> mInliers;

        rclcpp::Publisher<nav_msgs::msg::OccupancyGrid>::SharedPtr mCostMapPub;
        rclcpp::Subscription<sensor_msgs::msg::PointCloud2>::SharedPtr mPcSub;

        double mZPercent{}, mZThreshold{};
        double mAlpha{};
        double mNearClip{}, mFarClip{}, mNearWidth{}, mFarWidth{}, mTopClip{};
        double mResolution{}; // Meters per cell
        double mSize{};       // Size of the square costmap in meters
        int mWidth{};         // Number of cells on the grid horizontally
        int mHeight{};        // Number of cells on the grid vertically
        int mNumDivisions{};
        int mDownSamplingFactor = 2;
        std::string mMapFrame;
        int mDilateAmt = 1;

        tf2_ros::Buffer mTfBuffer{get_clock()};
        tf2_ros::TransformListener mTfListener{mTfBuffer};
        tf2_ros::TransformBroadcaster mTfBroadcaster{this};

        std::optional<SE3d> mPreviousPose;
        nav_msgs::msg::OccupancyGrid mGlobalGridMsg;

        rclcpp::Service<mrover::srv::MoveCostMap>::SharedPtr mServer;

        rclcpp::Service<mrover::srv::DilateCostMap>::SharedPtr mCostServer;

        rclcpp::Service<std_srvs::srv::SetBool>::SharedPtr mToggleCostMap;

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

        // To Test (CLI): ros2 service call /dilate_cost_map mrover/srv/DilateCostMap "{dilation_amount: [enter here]}"
        auto dilateCostMapCallback(mrover::srv::DilateCostMap::Request::ConstSharedPtr& req, mrover::srv::DilateCostMap::Response::SharedPtr& res) -> void;

        auto toggleCostMapCallback(std_srvs::srv::SetBool::Request::ConstSharedPtr& req, std_srvs::srv::SetBool::Response::SharedPtr& res) -> void;

        void uploadPC();

        // Bin vector coordinate
        struct Coordinate {
            int row;
            int col;

            auto operator-(Coordinate& other) const -> Coordinate {
                return {row - other.row, col - other.col};
            }

            auto operator+(Coordinate& other) const -> Coordinate {
                return {row + other.row, col + other.col};
            }
        };

        constexpr auto diArray() -> std::array<CostMapNode::Coordinate, (2 * dilation + 1) * (2 * dilation + 1)>;

        auto indexToCoordinate(int index) const -> Coordinate;
        auto coordinateToIndex(Coordinate c) const -> int;
        auto coordinateToIndex(Coordinate c, int width) const -> int;

        // Function for calculating bin-boundary intersections for ray tracing
        auto isRayIntersection(R3d const& startSeg, R3d const& endSeg, double binCenterX, double binCenterY) -> std::int8_t;
    };

} // namespace mrover
