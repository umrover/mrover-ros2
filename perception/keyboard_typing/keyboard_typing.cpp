#include "keyboard_typing.hpp"
#include <rclcpp/logging.hpp>

KeyboardTypingNode::KeyboardTypingNode(rclcpp::NodeOptions const& options) : rclcpp::Node("keyboard_typing_node", options)
{
    // subscribe to image stream
    mImageSub = this->create_subscription<sensor_msgs::msg::Image>("/topic/name", 1, [this](sensor_msgs::msg::Image::ConstSharedPtr const& msg) {
        yawCallback(msg);
    });
}


auto KeyboardTypingNode::yawCallback(sensor_msgs::msg::Image::ConstSharedPtr const& msg) -> void {
    return;
}

auto KeyboardTypingNode::poseEstimation(sensor_msgs::msg::Image::ConstSharedPtr const& msg) -> geometry_msgs::msg::Quaternion {
    std::string cameraConstants = "with_refinement.json";
    // cv::Ptr<cv::aruco::Dictionary> dictionary = cv::aruco::getPredefinedDictionary(cv::aruco::DICT_4X4_50);
    
    cv::Ptr<cv::aruco::CharucoBoard> board = cv::aruco::CharucoBoard::create(SQUARES_VERTICALLY, SQUARES_HORIZONTALLY, SQUARE_LENGTH, MARKER_LENGTH, ARUCO_DICT);
    cv::Ptr<cv::aruco::DetectorParameters> params = cv::aruco::DetectorParameters::create();

    cv::Mat bgraImage{static_cast<int>(msg->height), static_cast<int>(msg->width), CV_8UC4, const_cast<uint8_t*>(msg->data.data())};

     cv::cvtColor(bgraImage, bgraImage, cv::COLOR_BGR2BGRA);
    

}

auto KeyboardTypingNode::kalmanFilter(geometry_msgs::msg::Quaternion const& msg) -> geometry_msgs::msg::Quaternion {
    
}


/*
Steps
1. Convert pose estimation code over to c++
    a. Kalman filter the pose at the end of pose estimation
2. Publish yaw to keyboard_yaw in yawCallback
*/