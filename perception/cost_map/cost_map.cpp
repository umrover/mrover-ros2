#include "cost_map.hpp"
#include <sensor_msgs/msg/detail/point_cloud2__struct.hpp>

namespace mrover {

    CostMapNode::CostMapNode() : Node("cost_map") {


        std::vector<ParameterWrapper> params{
            {"resolution", mResolution, 0.5},
            {"size", mSize, 20},
            {"width", mWidth, static_cast<int>(mSize / mResolution)},   // THIS DECLARATION DOES NOT WORK
            {"height", mHeight, static_cast<int>(mSize / mResolution)},
            {"map_frame", mMapFrame, "map"},
            {"near_clip", mNearClip, 1},
            {"far_clip", mFarClip, 7.0},
            {"left_clip", mRightClip, -2.0},
            {"right_clip", mLeftClip, 2.0},
            {"top_clip", mTopClip, 3.0},
            {"z_percent", mZPercent, 0.2},
            {"alpha", mAlpha, 0.05},
            {"z_threshold", mZThreshold, .4}
        };

        ParameterWrapper::declareParameters(this, params);

        mCostMapPub = create_publisher<nav_msgs::msg::OccupancyGrid>("costmap", 1); // We publish our results to "costmap"

        mServer = create_service<mrover::srv::MoveCostMap>("move_cost_map", [this](mrover::srv::MoveCostMap::Request::ConstSharedPtr request, 
                                                                                                          mrover::srv::MoveCostMap::Response::SharedPtr response) {
            moveCostMapCallback(request, response);
        });
        mPcSub = create_subscription<sensor_msgs::msg::PointCloud2>("/zed/left/points", 1, [this](sensor_msgs::msg::PointCloud2::UniquePtr const& msg) {
            pointCloudCallback(msg);
        });

        mPCDebugPub = create_publisher<sensor_msgs::msg::PointCloud2>("cost_map/debug_pc", 1);
        // mImuSub = mNh.subscribe<sensor_msgs::Imu>("imu/data", 1, [this](sensor_msgs::ImuConstPtr const&) {
        //     mLastImuTime = ros::Time::now();
        // });
        RCLCPP_INFO_STREAM(get_logger(), std::format("frame: {}", mMapFrame));
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

auto main(int argc, char** argv) -> int {
    rclcpp::init(argc, argv);
    rclcpp::spin(std::make_shared<mrover::CostMapNode>());
    rclcpp::shutdown();
    return EXIT_SUCCESS;
}