#include "keyboard_typing.hpp"
#include <functional>
#include <geometry_msgs/msg/detail/pose__struct.hpp>
#include <opencv2/calib3d.hpp>
#include <opencv2/core/mat.hpp>
#include <opencv2/core/matx.hpp>
#include <opencv2/imgproc.hpp>
#include <opencv2/video/tracking.hpp>

KeyboardTypingNode::KeyboardTypingNode(rclcpp::NodeOptions const& options) : rclcpp::Node("keyboard_typing_node", options)
{
    // subscribe to image stream
    mImageSub = this->create_subscription<sensor_msgs::msg::Image>("/finger_camera/image", rclcpp::QoS(1), [this](sensor_msgs::msg::Image::ConstSharedPtr const& msg) {
        yawCallback(msg);
    });

    // create publisher
    mCostMapPub = this->create_publisher<geometry_msgs::msg::Quaternion>("/keypose/yaw", rclcpp::QoS(1));
}

auto KeyboardTypingNode::yawCallback(sensor_msgs::msg::Image::ConstSharedPtr const& msg) -> void {
    geometry_msgs::msg::Pose pose = estimatePose(msg);

    // extract yaw into a quarterian and then publish to mCostMapPub
    // mCostMapPub->publish(msg);
}

auto KeyboardTypingNode::estimatePose(sensor_msgs::msg::Image::ConstSharedPtr const& msg) -> geometry_msgs::msg::Pose {
    // Read in camera constants
    std::string cameraConstants = "temp.json";
    cv::Mat camMatrix = (cv::Mat_<double>(3,3) << 
        0.0, 0.0, 0.0,
        0.0, 0.0, 0.0,
        0.0, 0.0, 0.0
    );
    cv::Mat distCoeffs = (cv::Mat_<double>(5, 1) << 0.0, 0.0, 0.0, 0.0, 0.0);

    // Read in images
    cv::Mat bgraImage{static_cast<int>(msg->height), static_cast<int>(msg->width), CV_8UC4, const_cast<uint8_t*>(msg->data.data())};

    // Convert to grayscale
    cv::cvtColor(bgraImage, bgraImage, cv::COLOR_BGRA2GRAY);

    // Define variables
    std::vector<int> markerIds;

    std::vector<std::vector<cv::Point2f>> markerCorners, rejectedCandidates;
    std::vector<int> ids;
    cv::Ptr<cv::aruco::DetectorParameters> detectorParams = cv::aruco::DetectorParameters::create();
    cv::Ptr<cv::aruco::Dictionary> dictionary = cv::aruco::getPredefinedDictionary(cv::aruco::DICT_4X4_50);

    // Define coordinate system
    float markerLength = 0.02;  // meters
    cv::Mat objPoints(4, 1, CV_32FC3);
    objPoints.ptr<cv::Vec3f>(0)[0] = cv::Vec3f(-markerLength/2.f, markerLength/2.f, 0);
    objPoints.ptr<cv::Vec3f>(0)[1] = cv::Vec3f(markerLength/2.f, markerLength/2.f, 0);
    objPoints.ptr<cv::Vec3f>(0)[2] = cv::Vec3f(markerLength/2.f, -markerLength/2.f, 0);
    objPoints.ptr<cv::Vec3f>(0)[3] = cv::Vec3f(-markerLength/2.f, -markerLength/2.f, 0);

    // Detect Markers
    cv::aruco::detectMarkers(bgraImage, dictionary, markerCorners, ids, detectorParams, rejectedCandidates);

    // Corner refinement
    cv::Size winSize = cv::Size( 5, 5 );
    cv::Size zeroZone = cv::Size( -1, -1 ); 
    cv::TermCriteria criteria = cv::TermCriteria( cv::TermCriteria::EPS + cv::TermCriteria::COUNT, 40, 0.001 );

    for (std::vector<cv::Point2f> &corners : markerCorners) {
        cv::cornerSubPix(bgraImage, corners, winSize, zeroZone, criteria);
    }

    size_t nMarkers = markerCorners.size();
    std::vector<cv::Vec3d> rvecs(nMarkers), tvecs(nMarkers);

    // Estimate pose
    if (!ids.empty()) {
        for (size_t i = 0; i < nMarkers; ++i) {
            cv::solvePnP(objPoints, markerCorners.at(i), camMatrix, distCoeffs, rvecs.at(i), tvecs.at(i));
        }
    }
    
    // Apply Kalman Fillter
    for (size_t i = 0; i < tvecs.size(); ++i) {
        kalmanFilter(tvecs[i], rvecs[i]);
    }

    // Print rotation and translation sanity check
    for (const cv::Vec3d &translation : tvecs) {
        std::cout << "x vector : " << translation[0] << std::endl;
        std::cout << "y vector : " << translation[1] << std::endl;
        std::cout << "z vector : " << translation[2] << std::endl;
    }

    for (const cv::Vec3d &rotation : rvecs) {
        std::cout << "roll vector : " << rotation[0] << std::endl;
        std::cout << "pitch vector : " << rotation[1] << std::endl;
        std::cout << "yaw vector : " << rotation[2] << std::endl;
    }

    // Convert rvecs to quarterion
    // NOTE: Assume only 1 tag detected for now
    cv::Mat rotation_matrix;
    geometry_msgs::msg::Pose pose;
    if (markerCorners.size() > 0) {
        cv::Rodrigues(rvecs, rotation_matrix);

        Eigen::Matrix3d eigen_rotation;     
        eigen_rotation << 
            rotation_matrix.at<double>(0, 0), rotation_matrix.at<double>(0, 1), rotation_matrix.at<double>(0, 2),
            rotation_matrix.at<double>(1, 0), rotation_matrix.at<double>(1, 1), rotation_matrix.at<double>(1, 2),
            rotation_matrix.at<double>(2, 0), rotation_matrix.at<double>(2, 1), rotation_matrix.at<double>(2, 2);

        Eigen::Quaterniond quat(eigen_rotation);

        // Create pose message and then return it
        pose.position.x = tvecs[0][0];
        pose.position.y = tvecs[0][1];
        pose.position.z = tvecs[0][2];
        pose.orientation.x = quat.x();
        pose.orientation.y = quat.y();
        pose.orientation.z = quat.z();
        pose.orientation.w = quat.w();
    }

    return pose;
}




auto KeyboardTypingNode::kalmanFilter(cv::Vec3d &tvec, cv::Vec3d &rvecs) -> void {
    
    cv::KalmanFilter kf(12,6);
    float dt = 1.0;
    kf.transitionMatrix = (cv::Mat_<float>(12, 12) <<
    1, 0, 0, 0, 0, 0, dt, 0, 0, 0, 0, 0,
    0, 1, 0, 0, 0, 0, 0, dt, 0, 0, 0, 0,
    0, 0, 1, 0, 0, 0, 0, 0, dt, 0, 0, 0,
    0, 0, 0, 1, 0, 0, 0, 0, 0, dt, 0, 0,
    0, 0, 0, 0, 1, 0, 0, 0, 0, 0, dt, 0,
    0, 0, 0, 0, 0, 1, 0, 0, 0, 0, 0, dt,
    0, 0, 0, 0, 0, 0, 1, 0, 0, 0, 0, 0,
    0, 0, 0, 0, 0, 0, 0, 1, 0, 0, 0, 0,
    0, 0, 0, 0, 0, 0, 0, 0, 1, 0, 0, 0,
    0, 0, 0, 0, 0, 0, 0, 0, 0, 1, 0, 0,
    0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 1, 0,
    0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 1
    );

    kf.measurementMatrix = (cv::Mat_)

    

}


/*
Steps
1. Convert pose estimation code over to c++
    a. Kalman filter the pose at the end of pose estimation
2. Publish yaw to keyboard_yaw in yawCallback
*/