#include "pch.hpp"
#include <geometry_msgs/msg/detail/pose__struct.hpp>
#include <rclcpp/node.hpp>
#include <mrover/srv/align_lander.hpp>

namespace mrover {
    class LanderAlignNode final : public rclcpp::Node{
        private:
        rclcpp::Publisher<geometry_msgs::msg::Pose>::SharedPtr mCostMapPub;
        rclcpp::Subscription<sensor_msgs::msg::PointCloud2>::SharedPtr mPcSub;
        rclcpp::Service<mrover::srv::AlignLander>::SharedPtr mAlignServer;
        geometry_msgs::msg::Pose mFinalAngle;

        /*
        Using PCA to calculate plane:
        1. Initial filtering of points by removing any normal with z-component above some threshold (plane normals will be
                relatively flat)

        2. Filter out additional points that are some threshold above the median of all points (75th %ile?)
        3. Once filtered, calculate mean and subtract it off from all points
        4. Do PCA and find normal
        5. Keep doing to find better planes?
        */

        explicit LanderAlignNode(rclcpp::NodeOptions const& options = rclcpp::NodeOptions());

        ~LanderAlignNode() override = default;

        public:
        


        auto alignLanderCallBack(mrover::srv::AlignLander::Request::ConstSharedPtr& req, mrover::srv::AlignLander::Response::SharedPtr& res) -> void;
    };
}