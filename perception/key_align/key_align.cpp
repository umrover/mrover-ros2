#include "key_align.hpp"

namespace mrover{
    KeyAlign::KeyAlign() : Node("keyalign") {
        RCLCPP_INFO_STREAM(get_logger(), "Creating Key Aligner ...");

        // subscribe to img stream here
        mImageSubscriber = create_subscription<sensor_msgs::msg::Image>("zed/left/image", 1, [this](sensor_msgs::msg::Image::ConstSharedPtr const& msg) {
            imageCallback(msg);
        });

        // publish to stream here
        mTagPublisher = create_publisher<mrover::msg::KeyPose>("pose", 1);

        // representation of aruco markers
        mTagDictionary = cv::makePtr<cv::aruco::Dictionary>(cv::aruco::getPredefinedDictionary(cv::aruco::DICT_4X4_50));
    }

    // the "main" function / driver
    auto KeyAlign::imageCallback(sensor_msgs::msg::Image::ConstSharedPtr const& imageMessage) -> void {
        // Create a cv::Mat from the ROS image message
        // Note this does not copy the image data, it is basically a pointer
        // Be careful if you extend its lifetime beyond this function
        cv::Mat imageBGRA{static_cast<int>(imageMessage->height), static_cast<int>(imageMessage->width),
                      CV_8UC4, const_cast<uint8_t*>(imageMessage->data.data())};
        cv::Mat image;

        cv::cvtColor(imageBGRA, image, cv::COLOR_BGRA2BGR);

        // TODO: ADD SUPPORT FOR DEBUG IMAGE WITH ARUCO MARKER

        // loop: detect corner markers
        // Send the pose of the corners to some stream

        findtags(image, mTags);

        // potentially use homography matrix to find orientation of keyboard
    }

    auto KeyAlign::findtags(cv::Mat const& image, std::vector<mrover::msg::KeyPose>& tags) -> void{
        tags.clear();
        cv::aruco::detectMarkers(image, mTagDictionary, markerCorners, mTagIds, detectorParams);
        // corner is a vector of point2fs
        for (size_t i = 0; i < mTagIds.size(); ++i) {
            topleftcorner = markerCorners[i][0];
            toprightcorner = markerCorners[i][1];
            bottomrightcorner = markerCorners[i][2];
            bottomleftcorner = markerCorners[i][3];
            tags.emplace_back()
        }
        // run pose estimation here
        // then return the normal vector
        // need the camera parameters
        // put in tf tree


        // camera pitched with end effector, can ignore roll

        return;
    }

    auto KeyAlign::estimatePose() -> void {
        // run pose estimation here
        // return the x, y, z for each tag

        // get camera params somehow, will have to ask a lead
        //  camera matrix and distortion coefficients

        // set coord system
        cv::Mat objPoints(4, 1, CV_32FC3);

        // markerlength = physical length of marker
        float markerlength = 2; // CHANGE LATER
        objPoints.ptr<Vec3f>(0)[0] = Vec3f(-markerLength/2.f, markerLength/2.f, 0);
        objPoints.ptr<Vec3f>(0)[1] = Vec3f(markerLength/2.f, markerLength/2.f, 0);
        objPoints.ptr<Vec3f>(0)[2] = Vec3f(markerLength/2.f, -markerLength/2.f, 0);
        objPoints.ptr<Vec3f>(0)[3] = Vec3f(-markerLength/2.f, -markerLength/2.f, 0);
    }
} // namespace mrover