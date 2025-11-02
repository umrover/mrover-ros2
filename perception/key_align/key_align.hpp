#pragma once

namespace mrover{
    class KeyAlign : public rclcpp::Node {
        private:
            // Camera stream subscriber
            rclcpp::Subscription<sensor_msgs::msg::Image>::ConstSharedPtr mImageSubscriber;

            // data publisher
            // Msg should contain the pose of the keyboard relative to the camera
            rclcpp::Publisher<mrover::msg::KeyPose>::SharedPtr mTagPublisher;

            cv::Ptr<cv::aruco::Dictionary> mTagDictionary;

            // Aruco detector stuff
            cv::aruco::DetectorParameters detectorParams = cv::aruco::DetectorParameters();

            std::vector<std::vector<cv::Point2f>> markerCorners;

            std::vector<int> mTagIds;

            std::vector<mrover::msg::KeyPose> mTags;

            void imageCallback(sensor_msgs::msg::Image::ConstSharedPtr const& imageMessage);
        public:
            KeyAlign();
        
    }
}