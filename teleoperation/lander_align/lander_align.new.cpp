#include "lander_align.new.hpp"
#include <rclcpp/logging.hpp>

//LanderAlignActionServer member functions:
namespace mrover{

    using LanderAlign = action::LanderAlign;
    using GoalHandleLanderAlign = rclcpp_action::ServerGoalHandle<LanderAlign>;
    
    LanderAlignNode::LanderAlignNode(const rclcpp::NodeOptions& options) 
        : Node("lander_align", options) {

        rclcpp::Service<mrover::srv::AlignLander>::SharedPtr start =
            create_service<mrover::srv::AlignLander>("align_lander", &startCallBack);

        mSensorSub = create_subscription<sensor_msgs::msg::PointCloud2>("/zed/left/points", 1, [this](sensor_msgs::msg::PointCloud2::ConstSharedPtr & msg) {
            LanderAlignNode::pointCloudCallback(msg);
        });

    }

    void LanderAlignNode::pointCloudCallback(sensor_msgs::msg::PointCloud2::ConstSharedPtr & msg) {
        if (enabled) RCLCPP_INFO_STREAM(get_logger(), "PCA ALG HERE");
    }

    void LanderAlignNode::startCallBack(const std::shared_ptr<mrover::srv::AlignLander::Request> request, 
                                        std::shared_ptr<mrover::srv::AlignLander::Response>      response) {
        if (request->is_start == true) {
            enabled = true;
        }
        else {
            enabled = false;
        }
    }

}

int main(void) {
    return 0;
}