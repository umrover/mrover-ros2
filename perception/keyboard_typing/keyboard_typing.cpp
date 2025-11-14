#include "keyboard_typing.hpp"
#include <cmath>
#include <functional>
#include <opencv2/core/matx.hpp>
#include <rclcpp/logging.hpp>
#include <unordered_map>

namespace mrover{ 
    KeyboardTypingNode::KeyboardTypingNode(rclcpp::NodeOptions const& options) : rclcpp::Node("keyboard_typing_node", options),  mLoopProfiler{get_logger()}
    {
        RCLCPP_INFO_STREAM(get_logger(), "KeyBoardTypingNode starting up");

        // subscribe to image stream
        mImageSub = create_subscription<sensor_msgs::msg::Image>("/finger_camera/image", rclcpp::QoS(1), [this](sensor_msgs::msg::Image::ConstSharedPtr const& msg) {
            yawCallback(msg);
        });

        // Top left (TL) is origin
        static int TL_ID = 50;
        static int BL_ID = 51;
        static int TR_ID = 52;
        static int BR_ID = 53;

        // Translation vectors relative to TL
        // Placeholders for now
        static cv::Vec3d TL_OFFSET(0.0, 0.0, 0.0);
        static cv::Vec3d BL_OFFSET(0.0, -1.0, 0.0);
        static cv::Vec3d TR_OFFSET(1.0, 0.0, 0.0);
        static cv::Vec3d BR_OFFSET(1.0, -1.0, 0.0);

        offset_map.insert({TL_ID, TL_OFFSET});
        offset_map.insert({BL_ID, BL_OFFSET});
        offset_map.insert({TR_ID, TR_OFFSET});
        offset_map.insert({BR_ID, BR_OFFSET});

        float dt = 1.0f / 30.0f;  // 30 FPS instead of once every second
        // Initialize Kalman filter
        // State vector: [x, y, z, vx, vy, vz, roll, pitch, yaw, vroll, vpitch, vyaw]
        // Measurement vector: [x, y, z, roll, pitch, yaw]
        kf = cv::KalmanFilter(12, 6, 0, CV_32F);

        // Transition matrix (state update model)
        kf.transitionMatrix = (cv::Mat_<float>(12, 12) <<
            1, 0, 0, dt, 0, 0, 0, 0, 0, 0, 0, 0,
            0, 1, 0, 0, dt, 0, 0, 0, 0, 0, 0, 0,
            0, 0, 1, 0, 0, dt, 0, 0, 0, 0, 0, 0,
            0, 0, 0, 1, 0, 0, 0, 0, 0, 0, 0, 0,
            0, 0, 0, 0, 1, 0, 0, 0, 0, 0, 0, 0,
            0, 0, 0, 0, 0, 1, 0, 0, 0, 0, 0, 0,
            0, 0, 0, 0, 0, 0, 1, 0, 0, dt, 0, 0,
            0, 0, 0, 0, 0, 0, 0, 1, 0, 0, dt, 0,
            0, 0, 0, 0, 0, 0, 0, 0, 1, 0, 0, dt,
            0, 0, 0, 0, 0, 0, 0, 0, 0, 1, 0, 0,
            0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 1, 0,
            0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 1
        );

        // Measurement matrix (maps state to measurements)
        kf.measurementMatrix = (cv::Mat_<float>(6, 12) <<
            1, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0,
            0, 1, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0,
            0, 0, 1, 0, 0, 0, 0, 0, 0, 0, 0, 0,
            0, 0, 0, 0, 0, 0, 1, 0, 0, 0, 0, 0,
            0, 0, 0, 0, 0, 0, 0, 1, 0, 0, 0, 0,
            0, 0, 0, 0, 0, 0, 0, 0, 1, 0, 0, 0
        );
    
        // Process noise covariance (model uncertainty)
        cv::setIdentity(kf.processNoiseCov, cv::Scalar(1e-10));
        
        // Measurement noise covariance (sensor uncertainty)
        cv::setIdentity(kf.measurementNoiseCov, cv::Scalar(1e-2));
        
        // Initial state covariance
        cv::setIdentity(kf.errorCovPost, cv::Scalar(1));

        // Initialize state to zeros
        kf.statePost = cv::Mat::zeros(12, 1, CV_32F);

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
        int hitcount = 0;
        std::string cameraConstants = "temp.json";
        cv::Mat camMatrix = (cv::Mat_<double>(3,3) <<
            415.69, 0.0, 320.0,   // fx, 0, cx
            0.0, 415.69, 240.0,   // 0, fy, cy
            0.0, 0.0, 1.0
        );
        //cv::Mat::eye(3, 3, CV_64F);

        cv::Mat distCoeffs = cv::Mat::zeros(5,1,CV_64F);

        // Read in images
        cv::Mat bgraImage{static_cast<int>(msg->height), static_cast<int>(msg->width), CV_8UC4, const_cast<uint8_t*>(msg->data.data())};

        cv::Mat grayImage;
        cv::cvtColor(bgraImage, grayImage, cv::COLOR_BGRA2GRAY);

        // Define variables

        std::vector<std::vector<cv::Point2f>> markerCorners, rejectedCandidates;
        std::vector<int> ids;
        cv::Ptr<cv::aruco::DetectorParameters> detectorParams = cv::aruco::DetectorParameters::create();
        cv::Ptr<cv::aruco::Dictionary> dictionary = cv::aruco::getPredefinedDictionary(cv::aruco::DICT_4X4_50);

        // Define coordinate system
        float markerLength = 0.02;  // meters
        cv::Mat objPoints(4, 3, CV_32F);
        objPoints.at<float>(0, 0) = -markerLength/2.f; objPoints.at<float>(0, 1) = markerLength/2.f;  objPoints.at<float>(0, 2) = 0;
        objPoints.at<float>(1, 0) = markerLength/2.f;  objPoints.at<float>(1, 1) = markerLength/2.f;  objPoints.at<float>(1, 2) = 0;
        objPoints.at<float>(2, 0) = markerLength/2.f;  objPoints.at<float>(2, 1) = -markerLength/2.f; objPoints.at<float>(2, 2) = 0;
        objPoints.at<float>(3, 0) = -markerLength/2.f; objPoints.at<float>(3, 1) = -markerLength/2.f; objPoints.at<float>(3, 2) = 0;

        // Detect Markers
        cv::aruco::detectMarkers(grayImage, dictionary, markerCorners, ids, detectorParams, rejectedCandidates);

        // Corner refinement
        cv::Size winSize = cv::Size( 13, 13 );
        cv::Size zeroZone = cv::Size( -1, -1 ); 
        cv::TermCriteria criteria = cv::TermCriteria( cv::TermCriteria::EPS + cv::TermCriteria::COUNT, 40, 0.001 );

        for (std::vector<cv::Point2f> &corners : markerCorners) {
            cv::cornerSubPix(grayImage, corners, winSize, zeroZone, criteria);
        }

        size_t nMarkers = markerCorners.size();

        // Rotation and translation vectors
        std::vector<cv::Vec3d> rvecs(nMarkers), tvecs(nMarkers);

        // Estimate pose
        if (!ids.empty()) {
            for (size_t i = 0; i < nMarkers; ++i) {
                // RCLCPP_INFO_STREAM(get_logger(), "x: " << markerCorners[i][0].x);
                // RCLCPP_INFO_STREAM(get_logger(), "y: " << markerCorners[i][0].y);
                cv::solvePnP(objPoints, markerCorners.at(i), camMatrix, distCoeffs, rvecs.at(i), tvecs.at(i));
            }
        }

        // Offset the poses to origin
        for (size_t i = 0; i < tvecs.size(); ++i) {
            tvecs[i] = tvecs[i] - offset_map.at(ids[i]);
        }
        
        // Apply Kalman Filter to smooth the pose estimation
        for (size_t i = 0; i < tvecs.size(); ++i) {
            applyKalmanFilter(tvecs[i], rvecs[i]);
        }

        // Print rotation and translation sanity check
        if (tvecs.size() > 0) {
            auto x = tvecs[0][0];
            auto y = tvecs[0][1];
            auto z = tvecs[0][2];
            RCLCPP_INFO_STREAM(get_logger(), "tag id" << ids[0] << "\n");
            RCLCPP_INFO_STREAM(get_logger(), "x vector : " << x << "\n");
            RCLCPP_INFO_STREAM(get_logger(), "y vector : " << y << "\n");
            RCLCPP_INFO_STREAM(get_logger(), "z vector : " << z << "\n");
            RCLCPP_INFO_STREAM(get_logger(), "distance : " << std::sqrt(x*x + y*y + z*z) << "\n");
        }

        if (rvecs.size() > 0) {
            RCLCPP_INFO_STREAM(get_logger(), "roll vector : " << rvecs[0][0] << "\n");
            RCLCPP_INFO_STREAM(get_logger(), "pitch vector : " << rvecs[0][1] << "\n");
            RCLCPP_INFO_STREAM(get_logger(), "yaw vector : " << rvecs[0][2] << "\n");
        }

        // Convert rvecs to quarterion
        // NOTE: Assume only 1 tag detected for now
        cv::Mat rotation_matrix;
        geometry_msgs::msg::Pose pose;
        if (markerCorners.size() > 0) {
            cv::Rodrigues(rvecs[0], rotation_matrix);

            double m00 = rotation_matrix.at<double>(0, 0);
            double m10 = rotation_matrix.at<double>(1, 0);
            double m11 = rotation_matrix.at<double>(1, 1);
            double m12 = rotation_matrix.at<double>(1, 2);
            double m20 = rotation_matrix.at<double>(2, 0);
            double m21 = rotation_matrix.at<double>(2, 1);
            double m22 = rotation_matrix.at<double>(2, 2);

            double sy = std::sqrt(m00 * m00 + m10 * m10);
            bool singular = sy < 1e-6;

            double roll_rad, pitch_rad, yaw_rad;

            if (!singular) {
                roll_rad = std::atan2(m21, m22);
                pitch_rad = std::atan2(-m20, sy);
                yaw_rad = std::atan2(m10, m00);
            } else {
                roll_rad = std::atan2(-m12, m11);
                pitch_rad = std::atan2(-m20, sy);
                yaw_rad = 0;
            }

            //convert radians to degrees
            double roll_deg = roll_rad * 180.0 / M_PI;
            double pitch_deg = pitch_rad * 180.0 / M_PI;
            double yaw_deg = yaw_rad * 180.0 / M_PI;

            Eigen::Matrix3d eigen_rotation;     
            eigen_rotation << 
                m00, rotation_matrix.at<double>(0, 1), rotation_matrix.at<double>(0, 2),
                m10, m11, m12,
                m20, m21, m22;

            Eigen::Quaterniond quat(eigen_rotation);
            quat = Eigen::AngleAxisd(yaw_deg, Eigen::Vector3d::UnitZ())
                 * Eigen::AngleAxisd(pitch_deg, Eigen::Vector3d::UnitY())
                 * Eigen::AngleAxisd(roll_deg, Eigen::Vector3d::UnitX());

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
    
    auto KeyboardTypingNode::applyKalmanFilter(cv::Vec3d &tvec, cv::Vec3d &rvec) -> void {        
        // --- Convert rvec to roll, pitch, yaw (Euler angles) ---
        cv::Mat rotation_matrix;
        cv::Rodrigues(rvec, rotation_matrix);
        
        double sy = std::sqrt(rotation_matrix.at<double>(0, 0) * rotation_matrix.at<double>(0, 0) +
                              rotation_matrix.at<double>(1, 0) * rotation_matrix.at<double>(1, 0));
        
        bool singular = sy < 1e-6;
        
        double roll, pitch, yaw;
        if (!singular) {
            roll = std::atan2(rotation_matrix.at<double>(2, 1), rotation_matrix.at<double>(2, 2));
            pitch = std::atan2(-rotation_matrix.at<double>(2, 0), sy);
            yaw = std::atan2(rotation_matrix.at<double>(1, 0), rotation_matrix.at<double>(0, 0));
        } else {
            roll = std::atan2(-rotation_matrix.at<double>(1, 2), rotation_matrix.at<double>(1, 1));
            pitch = std::atan2(-rotation_matrix.at<double>(2, 0), sy);
            yaw = 0;
        }
        
        // --- Predict step ---
        cv::Mat prediction = kf.predict();
        
        // --- Prepare measurement vector [x, y, z, roll, pitch, yaw] ---
        cv::Mat measurement(6, 1, CV_32F);
        measurement.at<float>(0) = static_cast<float>(tvec[0]);
        measurement.at<float>(1) = static_cast<float>(tvec[1]);
        measurement.at<float>(2) = static_cast<float>(tvec[2]);
        measurement.at<float>(3) = static_cast<float>(roll);
        measurement.at<float>(4) = static_cast<float>(pitch);
        measurement.at<float>(5) = static_cast<float>(yaw);
        
        // --- Correction step ---
        cv::Mat estimated = kf.correct(measurement);
        
        // --- Update the tvec and rvec with filtered values ---
        // Update translation
        tvec[0] = estimated.at<float>(0);
        tvec[1] = estimated.at<float>(1);
        tvec[2] = estimated.at<float>(2);
        
        // Update rotation (convert filtered Euler angles back to rvec)
        double filtered_roll = estimated.at<float>(6);
        double filtered_pitch = estimated.at<float>(7);
        double filtered_yaw = estimated.at<float>(8);
        
        // Convert Euler angles back to rotation matrix
        cv::Mat R_x = (cv::Mat_<double>(3, 3) <<
            1, 0, 0,
            0, std::cos(filtered_roll), -std::sin(filtered_roll),
            0, std::sin(filtered_roll), std::cos(filtered_roll)
        );
        
        cv::Mat R_y = (cv::Mat_<double>(3, 3) <<
            std::cos(filtered_pitch), 0, std::sin(filtered_pitch),
            0, 1, 0,
            -std::sin(filtered_pitch), 0, std::cos(filtered_pitch)
        );
        
        cv::Mat R_z = (cv::Mat_<double>(3, 3) <<
            std::cos(filtered_yaw), -std::sin(filtered_yaw), 0,
            std::sin(filtered_yaw), std::cos(filtered_yaw), 0,
            0, 0, 1
        );
        
        cv::Mat filtered_rotation_matrix = R_z * R_y * R_x;
        cv::Rodrigues(filtered_rotation_matrix, rvec);
        
        // Log filtered values for debugging
        RCLCPP_DEBUG(get_logger(), 
            "Kalman Filter - Raw: (%.3f, %.3f, %.3f), Filtered: (%.3f, %.3f, %.3f)",
            measurement.at<float>(0), measurement.at<float>(1), measurement.at<float>(2),
            tvec[0], tvec[1], tvec[2]);
    }
}

/*
Steps
1. Convert pose estimation code over to c++
    a. Kalman filter the pose at the end of pose estimation
2. Publish yaw to keyboard_yaw in yawCallback
*/