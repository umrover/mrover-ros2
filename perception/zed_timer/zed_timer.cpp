#include "zed_timer.hpp"
#include <rclcpp/duration.hpp>
#include <rclcpp/logging.hpp>
#include <rclcpp/time.hpp>
#include <sensor_msgs/msg/detail/point_cloud2__struct.hpp>
#include <sensor_msgs/msg/point_cloud2.hpp>
#include <sensor_msgs/point_cloud2_iterator.hpp>

namespace mrover {
    ZedTimerNode::ZedTimerNode(rclcpp::NodeOptions const& options) : Node("zed_timer", options) {
        mPcSub = this->create_subscription<sensor_msgs::msg::PointCloud2>("/zed/left/points", 1, [this](sensor_msgs::msg::PointCloud2::ConstSharedPtr const& msg) {
            pointCloudCallback(msg);
        });

        mPrevTime = this->get_clock()->now();
    }

    auto ZedTimerNode::pointCloudCallback(sensor_msgs::msg::PointCloud2::ConstSharedPtr const& msg) -> void {
        rclcpp::Time now = this->get_clock()->now();
        rclcpp::Duration diff = now - mPrevTime;
        mPrevTime = now;
        
        double sec_diff = diff.seconds();
        double rate = 1.0 / sec_diff;

        if (times.size() != mWindowSize) {
            times.push_back(rate);
            avgRate += rate;
        } else {
            avgRate -= times[mRemove];
            avgRate += rate;
            times[mRemove] = rate;
            mRemove = (mRemove + 1) % mWindowSize;

            RCLCPP_INFO(this->get_logger(), "Average Publishing Rate: %f", (avgRate/mWindowSize));
        }
    }
}

#include "rclcpp_components/register_node_macro.hpp"
RCLCPP_COMPONENTS_REGISTER_NODE(mrover::ZedTimerNode)