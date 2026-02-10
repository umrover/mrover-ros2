#pragma once

#include <rclcpp/duration.hpp>
#include <rclcpp/node.hpp>
#include <rclcpp/subscription.hpp>
#include <rclcpp/time.hpp>
#include <sensor_msgs/msg/detail/point_cloud2__struct.hpp>
#include <vector>


namespace mrover {
    class ZedTimerNode final : public rclcpp::Node {
        private:
        rclcpp::Subscription<sensor_msgs::msg::PointCloud2>::SharedPtr mPcSub;

        rclcpp::Time mPrevTime;
        int mWindowSize = 10;
        int mRemove = 0;
        std::vector<double> times;
        double avgRate = 0.0;

        auto pointCloudCallback(sensor_msgs::msg::PointCloud2::ConstSharedPtr const& msg) -> void;

        public:
        explicit ZedTimerNode(rclcpp::NodeOptions const& options = rclcpp::NodeOptions());
        
        ~ZedTimerNode() override = default;
    };
}