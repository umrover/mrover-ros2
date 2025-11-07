#include "keyboard_typing.hpp"
#include <functional>

namespace mrover{ 
    KeyboardTypingNode::KeyboardTypingNode(rclcpp::NodeOptions const& options) : rclcpp::Node("keyboard_typing_node", options),  mLoopProfiler{get_logger()}
    {
        RCLCPP_INFO_STREAM(get_logger(), "KeyBoardTypingNode starting up");

        // subscribe to image stream
        mImageSub = create_subscription<sensor_msgs::msg::Image>("/finger_camera/image", rclcpp::QoS(1), [this](sensor_msgs::msg::Image::ConstSharedPtr const& msg) {
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
            554.0, 0.0, 320.0,   // fx, 0, cx
            0.0, 554.0, 240.0,   // 0, fy, cy
            0.0, 0.0, 1.0
        );
        
        cv::Mat distCoeffs = cv::Mat::zeros(5,1,CV_64F);

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
                // RCLCPP_INFO_STREAM(get_logger(), "x: " << markerCorners[i][0].x);
                // RCLCPP_INFO_STREAM(get_logger(), "y: " << markerCorners[i][0].y);
                cv::solvePnP(objPoints, markerCorners.at(i), camMatrix, distCoeffs, rvecs.at(i), tvecs.at(i));
            }
        }
        
        // Apply Kalman Filter to smooth the pose estimation
        for (size_t i = 0; i < tvecs.size(); ++i) {
            kalmanFilter(tvecs[i], rvecs[i]);
        }

        // Print rotation and translation sanity check
        for (const cv::Vec3d &translation : tvecs) {
            RCLCPP_INFO_STREAM(get_logger(), "x vector : " << translation[0] << "\n");
            RCLCPP_INFO_STREAM(get_logger(), "y vector : " << translation[1] << "\n");
            RCLCPP_INFO_STREAM(get_logger(), "z vector : " << translation[2] << "\n");
        }

        for (const cv::Vec3d &rotation : rvecs) {
            RCLCPP_INFO_STREAM(get_logger(), "roll vector : " << rotation[0] << "\n");
            RCLCPP_INFO_STREAM(get_logger(), "pitch vector : " << rotation[1] << "\n");
            RCLCPP_INFO_STREAM(get_logger(), "yaw vector : " << rotation[2] << "\n");
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
    
    auto KeyboardTypingNode::kalmanFilter(cv::Vec3d &tvec, cv::Vec3d &rvec) -> void {
        static cv::KalmanFilter kf;
        static bool isInitialized = false;

        float dt = 1.0f / 30.0f;  // 30 FPS instead of once every second

        // Initialize Kalman filter
        if (!isInitialized) {
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
            cv::setIdentity(kf.processNoiseCov, cv::Scalar(1e-4));
            
            // Measurement noise covariance (sensor uncertainty)
            cv::setIdentity(kf.measurementNoiseCov, cv::Scalar(1e-2));
            
            // Initial state covariance
            cv::setIdentity(kf.errorCovPost, cv::Scalar(1));

            // Initialize state to zeros
            kf.statePost = cv::Mat::zeros(12, 1, CV_32F);
            
            isInitialized = true;
        }
        
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
        RCLCPP_DEBUG(this->get_logger(), 
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