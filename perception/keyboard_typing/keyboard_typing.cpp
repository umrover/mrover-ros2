#include "keyboard_typing.hpp"
#include "lie.hpp"
#include <algorithm>
#include <numeric>

namespace mrover{
    KeyboardTypingNode::KeyboardTypingNode(rclcpp::NodeOptions const& options) : rclcpp::Node("keyboard_typing_node", options) {
        // subscribe to image stream
        mImageSub = create_subscription<sensor_msgs::msg::Image>("/finger_camera/image", 1, [this](sensor_msgs::msg::Image::ConstSharedPtr const& msg) {
            yawCallback(msg);
        });

        mQuaternionPub = create_publisher<geometry_msgs::msg::Quaternion>("/keyboard/yaw", 1);

        mTagDictionary = cv::makePtr<cv::aruco::Dictionary>(cv::aruco::getPredefinedDictionary(cv::aruco::DICT_4X4_100));
    }


    auto KeyboardTypingNode::yawCallback(sensor_msgs::msg::Image::ConstSharedPtr const& msg) -> void {
        return;
    }

    auto KeyboardTypingNode::poseEstimation(sensor_msgs::msg::Image::ConstSharedPtr const& msg) -> geometry_msgs::msg::Quaternion {
        return geometry_msgs::msg::Quaternion();
    }

    auto KeyboardTypingNode::kalmanFilter(geometry_msgs::msg::Quaternion const& msg) -> geometry_msgs::msg::Quaternion {
        
    }

    // General tag finding algorithm:
    // 1. Detect tags in image
    // 2. Transform tags to world frame and select left-most and highest one as reference tag
    // 3. Every five hits on tag of interest, publish yaw relative to that tag
    auto KeyboardTypingNode::findTagsInImage(cv::Mat const& image) -> void {
        // Outputs tagIDs and tagCorners in respective vectors, they will be the same size
        cv::aruco::detectMarkers(image, mTagDictionary, mTagCorners, mTagIds);

        // Take detected tags and update tags map
        for(int i = 0; i < mTagIds.size(); i++){
            int tagId = mTagIds[i];
            Tag& tag = mTagMap[tagId];
            tag.id = mTagIds[i];
            tag.hitCount = std::clamp(tag.hitCount + 1, 0, 5);
            tag.center = std::reduce(mTagCorners[i].begin(), mTagCorners[i].end()) / static_cast<float>(mTagCorners[i].size());

            if(tag.id != mSelectedTag.id && (tag.center.x < mSelectedTag.center.x || tag.center.y < mSelectedTag.center.y)){
                mSelectedTag = tag;
            }
        }

        for(auto& tag : mTagMap){
            
        }
    }


/*
Steps
1. Convert pose estimation code over to c++
    a. Kalman filter the pose at the end of pose estimation
2. Publish yaw to keyboard_yaw in yawCallback
*/
}