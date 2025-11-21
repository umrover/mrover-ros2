#include "keyboard_typing.hpp"
#include "lie.hpp"
#include <cmath>
#include <opencv2/core/eigen.hpp>
#include <opencv2/core/matx.hpp>
#include <opencv2/core/mat.hpp>
#include <opencv2/highgui.hpp>
#include <opencv2/imgproc.hpp>
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
        // Bottom left (BL)
        // Top right (TR)
        // Bottom right (BR)
        static int TL_ID = 2;
        static int BL_ID = 3;
        static int TR_ID = 4;
        static int BR_ID = 5;

        // Translation vectors relative to TL (x,y,z offset)
        // Placeholders for now
        static cv::Vec3d TL_OFFSET(0.0, 0.0, 0.0);
        static cv::Vec3d BL_OFFSET(0.0, -1.0, 0.0);
        static cv::Vec3d TR_OFFSET(1.0, 0.0, 0.0);
        static cv::Vec3d BR_OFFSET(1.0, -1.0, 0.0);

        offset_map.insert({TL_ID, TL_OFFSET});
        offset_map.insert({BL_ID, BL_OFFSET});
        offset_map.insert({TR_ID, TR_OFFSET});
        offset_map.insert({BR_ID, BR_OFFSET});

        // Initialize Kalman filter
        // State vector: [x, y, z, vx, vy, vz, roll, pitch, yaw, vroll, vpitch, vyaw]
        // Measurement vector: [x, y, z, roll, pitch, yaw]
        kf = cv::KalmanFilter(12, 6, 0, CV_32F);

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
        RCLCPP_INFO_STREAM(get_logger(), "callback");
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

        cv::Mat distCoeffs = cv::Mat::zeros(1,5,CV_64F);

        // Read in images
        cv::Mat bgraImage{static_cast<int>(msg->height), static_cast<int>(msg->width), CV_8UC4, const_cast<uint8_t*>(msg->data.data())};

        // convert to gray for estimation
        cv::Mat grayImage;
        cv::cvtColor(bgraImage, grayImage, cv::COLOR_BGRA2GRAY);
        // Optional: enhance contrast
        cv::equalizeHist(grayImage, grayImage);

        // Optional: apply Gaussian blur to reduce noise
        cv::GaussianBlur(grayImage, grayImage, cv::Size(5,5), 0);

        // Define variables

        std::vector<std::vector<cv::Point2f>> markerCorners, rejectedCandidates;
        std::vector<int> ids;
        cv::Ptr<cv::aruco::DetectorParameters> detectorParams = cv::aruco::DetectorParameters::create();
        cv::Ptr<cv::aruco::Dictionary> dictionary = cv::aruco::getPredefinedDictionary(cv::aruco::DICT_4X4_50);

        // Define coordinate system
        float markerLength = 0.02;  // meters
        std::vector<cv::Point3f> objPoints = {
            {-markerLength/2.f,  markerLength/2.f, 0},
            { markerLength/2.f,  markerLength/2.f, 0},
            { markerLength/2.f, -markerLength/2.f, 0},
            {-markerLength/2.f, -markerLength/2.f, 0}
        };

        // Detect Markers
        cv::aruco::detectMarkers(grayImage, dictionary, markerCorners, ids, detectorParams, rejectedCandidates);

        // Corner refinement
        cv::Size winSize = cv::Size( 5, 5 );
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
            // Check if the detected tag ID exists in our offset map
            if (offset_map.find(ids[i]) != offset_map.end()) {
                auto tag_offset_world = offset_map.at(ids[i]);
                tvecs[i] = getGlobalCameraPosition(rvecs[i], tvecs[i], tag_offset_world);
            } else {
                RCLCPP_WARN(get_logger(), "Tag ID %d not found in offset_map, skipping offset", ids[i]);
            }
        }

        // draw results for debugging
        // debugging reg image
        if (!ids.empty()) {
            cv::aruco::drawDetectedMarkers(grayImage, markerCorners, ids);
            for (size_t i = 0; i < ids.size(); ++i) {
                cv::drawFrameAxes(grayImage, camMatrix, distCoeffs, rvecs[i], tvecs[i], markerLength * 1.5f, 2);
            }
        }
        cv::imshow("out", grayImage);
        cv::waitKey(1);
        
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

        // Apply Kalman Filter to smooth the pose estimation
        if (!tvecs.empty()) {
            // Pass all vectors and the current ROS time
            // Returns the filtered pose
            return updateKalmanFilter(tvecs, rvecs);
        } else {
            // Return nothing??
        }

        // // Convert rvecs to quarterion
        // // NOTE: Assume only 1 tag detected for now
        // cv::Mat rotation_matrix;
        // geometry_msgs::msg::Pose pose;
        // if (markerCorners.size() > 0) {
        //     cv::Rodrigues(rvecs[0], rotation_matrix);

        //     double m00 = rotation_matrix.at<double>(0, 0);
        //     double m10 = rotation_matrix.at<double>(1, 0);
        //     double m11 = rotation_matrix.at<double>(1, 1);
        //     double m12 = rotation_matrix.at<double>(1, 2);
        //     double m20 = rotation_matrix.at<double>(2, 0);
        //     double m21 = rotation_matrix.at<double>(2, 1);
        //     double m22 = rotation_matrix.at<double>(2, 2);

        //     double sy = std::sqrt(m00 * m00 + m10 * m10);
        //     bool singular = sy < 1e-6;

        //     double roll_rad, pitch_rad, yaw_rad;

        //     if (!singular) {
        //         roll_rad = std::atan2(m21, m22);
        //         pitch_rad = std::atan2(-m20, sy);
        //         yaw_rad = std::atan2(m10, m00);
        //     } else {
        //         roll_rad = std::atan2(-m12, m11);
        //         pitch_rad = std::atan2(-m20, sy);
        //         yaw_rad = 0;
        //     }

        //     //convert radians to degrees
        //     double roll_deg = roll_rad * 180.0 / M_PI;
        //     double pitch_deg = pitch_rad * 180.0 / M_PI;
        //     double yaw_deg = yaw_rad * 180.0 / M_PI;

        //     Eigen::Matrix3d eigen_rotation;     
        //     eigen_rotation << 
        //         m00, rotation_matrix.at<double>(0, 1), rotation_matrix.at<double>(0, 2),
        //         m10, m11, m12,
        //         m20, m21, m22;

        //     Eigen::Quaterniond quat(eigen_rotation);
        //     quat = Eigen::AngleAxisd(yaw_deg, Eigen::Vector3d::UnitZ())
        //          * Eigen::AngleAxisd(pitch_deg, Eigen::Vector3d::UnitY())
        //          * Eigen::AngleAxisd(roll_deg, Eigen::Vector3d::UnitX());

        //     // Create pose message and then return it
        //     pose.position.x = tvecs[0][0];
        //     pose.position.y = tvecs[0][1];
        //     pose.position.z = tvecs[0][2];
        //     pose.orientation.x = quat.x();
        //     pose.orientation.y = quat.y();
        //     pose.orientation.z = quat.z();
        //     pose.orientation.w = quat.w();
        // }
    }

    // Helper function to convert OpenCV PnP result to Global Camera Position
    // Returns: Vector3d representing Camera Position in World Frame
    auto getGlobalCameraPosition(cv::Vec3d const& rvec,
                                 cv::Vec3d const& tvec,
                                 cv::Vec3d const& tag_offset_world) -> cv::Vec3d {
        // 1. Convert OpenCV Inputs to Eigen types
        // --------------------------------------
        // Translation: cv::Vec3d -> mrover::R3d (Eigen::Vector3d)
        R3d t_eigen(tvec[0], tvec[1], tvec[2]);

        // Rotation: cv::Vec3d (Rodrigues) -> cv::Mat -> Eigen::Matrix3d -> Eigen::Quaternion
        cv::Mat R_cv;
        cv::Rodrigues(rvec, R_cv);

        Eigen::Matrix3d R_eigen_mat;
        cv::cv2eigen(R_cv, R_eigen_mat);

        S3d q_eigen(R_eigen_mat); // Convert Matrix to Quaternion 

        // T_c_t: Transform of the Tag in the Camera frame (from PnP)
        SE3d T_cam_tag(t_eigen, q_eigen);

        // T_w_t: Offsets from world tag offsets
        R3d t_offset(tag_offset_world[0], tag_offset_world[1], tag_offset_world[2]);

        // Assuming tags are flat on wall (Identity rotation).
        // If tags are rotated, pass in quaternion here instead of identity.
        SE3d T_world_tag(t_offset, S3d::Identity());

        // T_world_cam = T_world_tag * (T_cam_tag)^-1
        SE3d T_world_cam = T_world_tag * T_cam_tag.inverse();
        R3d result_pos = T_world_cam.translation();

        return cv::Vec3d(result_pos.x(), result_pos.y(), result_pos.z());
    }

    auto KeyboardTypingNode::updateKalmanFilter(std::vector<cv::Vec3d> const& tvecs,
                                                std::vector<cv::Vec3d> const& rvecs) -> geometry_msgs::msg::Pose {
        // 1. Get current time from the Node's clock
        rclcpp::Time currentTime = this->get_clock()->now();

        // 2. Handle Initialization
        if (!filter_initialized_) {
            last_prediction_time_ = currentTime;
            filter_initialized_ = true;
            return geometry_msgs::msg::Pose();
        }

        // 3. Calculate Delta Time (dt)
        // Subtracting two rclcpp::Time objects returns an rclcpp::Duration
        double dt = (currentTime - last_prediction_time_).seconds();
        
        // Update tracker
        last_prediction_time_ = currentTime;

        // 2. UPDATE TRANSITION MATRIX & PREDICT (ONCE PER FRAME)
        // ------------------------------------------------------
        // Update position rows (x = x + v*dt)
        kf.transitionMatrix.at<float>(0, 3) = dt;
        kf.transitionMatrix.at<float>(1, 4) = dt;
        kf.transitionMatrix.at<float>(2, 5) = dt;

        // Update rotation rows (yaw = yaw + v_yaw*dt)
        kf.transitionMatrix.at<float>(6, 9) = dt;
        kf.transitionMatrix.at<float>(7, 10) = dt;
        kf.transitionMatrix.at<float>(8, 11) = dt;

        cv::Mat prediction = kf.predict();

        // 3. CORRECTION LOOP (SEQUENTIAL UPDATES)
        // ---------------------------------------
        // We loop through every visible tag and "nudge" the filter
        for (size_t i = 0; i < tvecs.size(); ++i) {

            // --- A. Helper: Extract Yaw from Rvec ---
            cv::Mat R;
            cv::Rodrigues(rvecs[i], R);
            double sy = std::sqrt(R.at<double>(0, 0) * R.at<double>(0, 0) + R.at<double>(1, 0) * R.at<double>(1, 0));
            bool singular = sy < 1e-6;
            double measured_roll, measured_pitch, measured_yaw;

            if (!singular) {
                measured_roll = std::atan2(R.at<double>(2, 1), R.at<double>(2, 2));
                measured_pitch = std::atan2(-R.at<double>(2, 0), sy);
                measured_yaw = std::atan2(R.at<double>(1, 0), R.at<double>(0, 0));
            } else {
                measured_roll = std::atan2(-R.at<double>(1, 2), R.at<double>(1, 1));
                measured_pitch = std::atan2(-R.at<double>(2, 0), sy);
                measured_yaw = 0;
            }

            // --- B. Angle Wrapping (The Fix) ---
            // Compare measured yaw to the filter's current belief (state index 5 for yaw, 8 for yaw_vel? Check your init)
            // Based on your init: State is 12 vars.
            // Indexes: 0=x, 1=y, 2=z... 6=roll, 7=pitch, 8=yaw.

            float current_yaw_state = kf.statePost.at<float>(8); // Index 8 is Yaw position in your state vector

            // Fix for angle clamping
            while (measured_yaw - current_yaw_state > M_PI) {
                measured_yaw -= 2.0 * M_PI;
            }

            while (measured_yaw - current_yaw_state < -M_PI) {
                measured_yaw += 2.0 * M_PI;
            }

            // --- C. Dynamic Noise (Trust closer tags more) ---
            double dist = cv::norm(tvecs[i]);

            // Base noise (0.01) + Distance Penalty.
            // Far away tags (e.g. 3 meters) get much higher variance.
            float pos_noise = 0.001f + (0.05f * dist * dist);
            float ang_noise = 0.01f + (0.1f * dist);

            cv::setIdentity(kf.measurementNoiseCov, cv::Scalar(1));
            kf.measurementNoiseCov.at<float>(0, 0) = pos_noise;
            kf.measurementNoiseCov.at<float>(1, 1) = pos_noise;
            kf.measurementNoiseCov.at<float>(2, 2) = pos_noise;
            kf.measurementNoiseCov.at<float>(3, 3) = ang_noise;
            kf.measurementNoiseCov.at<float>(4, 4) = ang_noise;
            kf.measurementNoiseCov.at<float>(5, 5) = ang_noise;

            // --- D. Build Measurement & Correct ---
            cv::Mat measurement(6, 1, CV_32F);
            measurement.at<float>(0) = static_cast<float>(tvecs[i][0]);
            measurement.at<float>(1) = static_cast<float>(tvecs[i][1]);
            measurement.at<float>(2) = static_cast<float>(tvecs[i][2]);
            measurement.at<float>(3) = static_cast<float>(measured_roll);
            measurement.at<float>(4) = static_cast<float>(measured_pitch);
            measurement.at<float>(5) = static_cast<float>(measured_yaw);

            // This updates the state immediately, so the next tag in the loop
            // benefits from this correction.
            kf.correct(measurement);
        }

        // 4. EXPORT FINAL STATE TO POSE
        // -----------------------------
        geometry_msgs::msg::Pose filtered_pose;

        // Position
        filtered_pose.position.x = kf.statePost.at<float>(0);
        filtered_pose.position.y = kf.statePost.at<float>(1);
        filtered_pose.position.z = kf.statePost.at<float>(2);

        // Orientation (Convert RPY back to Quaternion)
        double final_roll = kf.statePost.at<float>(6);
        double final_pitch = kf.statePost.at<float>(7);
        double final_yaw = kf.statePost.at<float>(8);

        // Using Eigen for clean RPY -> Quat conversion
        Eigen::Quaterniond q;
        q = Eigen::AngleAxisd(final_yaw, Eigen::Vector3d::UnitZ()) *
            Eigen::AngleAxisd(final_pitch, Eigen::Vector3d::UnitY()) *
            Eigen::AngleAxisd(final_roll, Eigen::Vector3d::UnitX());

        filtered_pose.orientation.x = q.x();
        filtered_pose.orientation.y = q.y();
        filtered_pose.orientation.z = q.z();
        filtered_pose.orientation.w = q.w();

        return filtered_pose;
    }
}

/*
Steps
1. Convert pose estimation code over to c++
    a. Kalman filter the pose at the end of pose estimation
2. Publish yaw to keyboard_yaw in yawCallback
*/