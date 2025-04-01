#include "lander_analysis.hpp"

namespace mrover {
    LanderAnalysis::LanderAnalysis(rclcpp::NodeOptions const& options) : rclcpp::Node{NODE_NAME, options}{
        RCLCPP_INFO_STREAM(get_logger(), "Created Node");

        mPcSub = create_subscription<sensor_msgs::msg::PointCloud2>("/zed/left/points", 1, [this](sensor_msgs::msg::PointCloud2::ConstSharedPtr const& msg) {
            LanderAnalysis::pointCloudCallback(msg);
        });
    }

    auto LanderAnalysis::pointCloudCallback(sensor_msgs::msg::PointCloud2::ConstSharedPtr const& msg) -> void{
        RCLCPP_INFO_STREAM(get_logger(), "Running PC Callback");

        // find the means for the point cloud
        Point const* points = reinterpret_cast<Point const*>(msg->data.data());

        std::size_t numPoints = msg->width * msg->height;
        std::size_t numInliers = 0;
        R3f means;
        means.x() = 0.0f;
        means.y() = 0.0f;
        means.z() = 0.0f;
        for(std::size_t i = 0; i < numPoints; ++i){
            Point const& point = points[i];

            if(point.normal_z < 0.5) { // TODO: make this not hardcoded
                means.x() = point.x;
                means.y() = point.y;
                means.z() = point.z;
                ++numInliers;
            }
        }

        RCLCPP_INFO_STREAM(get_logger(), "Means " << means << " inliers " << numInliers);

    }
};

#include "rclcpp_components/register_node_macro.hpp"
RCLCPP_COMPONENTS_REGISTER_NODE(mrover::LanderAnalysis);
