#include "cost_map.hpp"

namespace mrover {
    CostMapNode::CostMapNode(rclcpp::NodeOptions const& options) : Node("cost_map", options){
        std::vector<ParameterWrapper> params{
            {"resolution", mResolution, 0.5},
            {"size", mSize, 60.0},
            {"width", mWidth, static_cast<int>(mSize / mResolution)},   // THIS DECLARATION DOES NOT WORK
            {"height", mHeight, static_cast<int>(mSize / mResolution)},
            {"map_frame", mMapFrame, "map"},
            {"near_clip", mNearClip, 1.0},
            {"far_clip", mFarClip, 7.0},
            {"left_clip", mRightClip, -2.0},
            {"right_clip", mLeftClip, 2.0},
            {"top_clip", mTopClip, 3.0},
            {"z_percent", mZPercent, 0.34}, // Tested on static visualizer
            {"alpha", mAlpha, 0.05},
            {"z_threshold", mZThreshold, 0.51}, // Tested on static visualizer
        };

        ParameterWrapper::declareParameters(this, params);

        mCostMapPub = create_publisher<nav_msgs::msg::OccupancyGrid>("costmap", 1); // We publish our results to "costmap"

        mServer = create_service<mrover::srv::MoveCostMap>("move_cost_map", [this](mrover::srv::MoveCostMap::Request::ConstSharedPtr request,
                                                                                   mrover::srv::MoveCostMap::Response::SharedPtr response) {
            moveCostMapCallback(request, response);
        });
        mPcSub = create_subscription<sensor_msgs::msg::PointCloud2>("/zed/left/points", 1, [this](sensor_msgs::msg::PointCloud2::ConstSharedPtr const& msg) {
            pointCloudCallback(msg);
        });

        mCostServer = create_service<srv::DilateCostMap>("dilate_cost_map", [this](mrover::srv::DilateCostMap::Request::ConstSharedPtr req, mrover::srv::DilateCostMap::Response::SharedPtr res) {
            dilateCostMapCallback(req, res);
        });

        mPCDebugPub = create_publisher<sensor_msgs::msg::PointCloud2>("cost_map/debug_pc", 1);
        // mImuSub = mNh.subscribe<sensor_msgs::Imu>("imu/data", 1, [this](sensor_msgs::ImuConstPtr const&) {
        //     mLastImuTime = ros::Time::now();
        // });
        // RCLCPP_INFO_STREAM(get_logger(), std::format("frame: {}", mMapFrame));
        mGlobalGridMsg.info.resolution = mResolution;
        // Number of cells horizontally
        mGlobalGridMsg.info.width = static_cast<int>(mSize / mResolution);
        mWidth = static_cast<int>(mGlobalGridMsg.info.width);
        // Number of cells vertically
        mGlobalGridMsg.info.height = static_cast<int>(mSize / mResolution);
        mHeight = static_cast<int>(mGlobalGridMsg.info.height);

        // Center the map at (0, 0)
        mGlobalGridMsg.header.frame_id = mMapFrame;
        mGlobalGridMsg.info.origin.position.x = -mSize / 2;
        mGlobalGridMsg.info.origin.position.y = -mSize / 2;

        mGlobalGridMsg.data.resize(mGlobalGridMsg.info.width * mGlobalGridMsg.info.height, UNKNOWN_COST);
    }

} // namespace mrover

#include "rclcpp_components/register_node_macro.hpp"
RCLCPP_COMPONENTS_REGISTER_NODE(mrover::CostMapNode)
