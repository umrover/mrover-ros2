#include "keyboard_typing.hpp"
#include <cmath>
#include <geometry_msgs/msg/detail/pose__struct.hpp>
#include <opencv2/core/eigen.hpp>
#include "mrover/msg/detail/keyboard_yaw__struct.hpp"
#include <cmath>
#include <functional>
#include <geometry_msgs/msg/detail/quaternion__struct.hpp>
#include <opencv2/aruco.hpp>
#include <opencv2/calib3d.hpp>
#include <opencv2/core/matx.hpp>
#include <opencv2/core/mat.hpp>
#include <opencv2/highgui.hpp>
#include <opencv2/imgproc.hpp>
#include <optional>
#include <rclcpp/logging.hpp>
#include <unordered_map>
#include <fstream>

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
        static int BL_ID = 4;
        static int TR_ID = 3;
        static int BR_ID = 5;

        static float b_length = 0.354076;
        static float b_width = 0.123444;

        static float seperation_dist = (2/(std::sqrt(2)) + 1) * 0.01; // Centimeters to meters

        // Translation vectors relative to TL (x,y,z offset)
        // Placeholders for now
        static cv::Vec3d TL_OFFSET(0.0, 0.0, 0.0);
        static cv::Vec3d BL_OFFSET(0.0, -(b_width + 2 * seperation_dist), 0.0);
        static cv::Vec3d TR_OFFSET(b_length + 2 * seperation_dist, 0.0, 0.0);
        static cv::Vec3d BR_OFFSET(b_length + 2 * seperation_dist, -(b_width + 2 * seperation_dist), 0.0);

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
        mCostMapPub = this->create_publisher<msg::KeyboardYaw>("/keypose/yaw", rclcpp::QoS(1));

        // Define offsets
        layout[4] = cv::Vec3d(0.0, 0.0, 0.0);       // BL
        layout[5] = cv::Vec3d(0.387, 0.0, 0.0);     // BR
        layout[3] = cv::Vec3d(0.387, 0.183, 0.0);   // TR
        layout[2] = cv::Vec3d(0.0, 0.183, 0.0);     // TL

        createRoverBoard();
    }

    auto KeyboardTypingNode::yawCallback(sensor_msgs::msg::Image::ConstSharedPtr const& msg) -> void {
        std::optional<pose_output> pose = estimatePose(msg);

        // Publish yaw
        if (pose.has_value()) {
            mrover::msg::KeyboardYaw msg;
            msg.yaw = pose->yaw;
            mCostMapPub->publish(msg);
        }
    }

    auto KeyboardTypingNode::createRoverBoard() -> void {
        // 1. Define the physical corners of the tags in the "Board Frame"
        //    (The Board Frame origin will be your Bottom-Left tag)
        std::vector<std::vector<cv::Point3f>> all_obj_points; 
        std::vector<int> all_ids;

        // Loop through your defined IDs to build the board structure
        for(auto const& [id, offset] : layout) {
            std::vector<cv::Point3f> corners;
            double x = offset[0];
            double y = offset[1];
            double z = offset[2];

            // Define the 4 corners for THIS specific tag relative to the Anchor
            // Order: TL, TR, BR, BL
            // Note: We center the tag on the offset coordinate
            corners.push_back(cv::Point3f(x, y, z)); // Top Left
            corners.push_back(cv::Point3f(x, y, z)); // Top Right
            corners.push_back(cv::Point3f(x, y, z)); // Bottom Right
            corners.push_back(cv::Point3f(x, y, z)); // Bottom Left

            all_obj_points.push_back(corners);
            all_ids.push_back(id);
        }

        // Create the Board object
        // "dictionary" must be the same one used for detection
        rover_board = cv::aruco::Board::create(all_obj_points, dictionary, all_ids);
    }

    auto KeyboardTypingNode::estimatePose(sensor_msgs::msg::Image::ConstSharedPtr const& msg) -> std::optional<pose_output>  {
        // Read in camera constants
        std::string cameraConstants = "temp.json";
        cv::Mat camMatrix = (cv::Mat_<double>(3,3) <<
            432.82290127206676, 0.0, 320.66663616737236,   // fx, 0, cx
            0.0, 428.8529386724118, 256.09398022438245,   // 0, fy, cy
            0.0, 0.0, 1.0
        );

        cv::Mat distCoeffs = cv::Mat::zeros(1,5,CV_64F);
        // cv::Mat distCoeffs = (cv::Mat_<double>(1,5) << 0.058961289426335314,
        //     -0.000852015669231157,
        //     -0.00011057338930940814,
        //     0.004049336591019368,
        //     -0.11113072350633851);

        // Read in images
        cv::Mat bgraImage{static_cast<int>(msg->height), static_cast<int>(msg->width), CV_8UC4, const_cast<uint8_t*>(msg->data.data())};

        // convert to gray for estimation
        cv::Mat grayImage;
        cv::cvtColor(bgraImage, grayImage, cv::COLOR_BGRA2GRAY);
        // Optional: enhance contrast
        // cv::equalizeHist(grayImage, grayImage);

        // Optional: apply Gaussian blur to reduce noise
        // cv::GaussianBlur(grayImage, grayImage, cv::Size(5,5), 0);

        // Define variables

        std::vector<std::vector<cv::Point2f>> markerCorners, rejectedCandidates;
        std::vector<int> ids;
        cv::Ptr<cv::aruco::DetectorParameters> detectorParams = cv::aruco::DetectorParameters::create();

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

        // Estimate pose
        cv::Vec3d combined_tvec, combined_rvec;
        if (!ids.empty()) {
            std::vector<int> valid_indices;
            for (size_t i = 0; i < ids.size(); ++i) {
                if (layout.count(ids[i]) > 0) {
                    valid_indices.push_back(i);
                }
            }

            int validcnt = valid_indices.size();
            
            if (validcnt > 2) {
                cv::aruco::estimatePoseBoard(markerCorners, ids, rover_board, camMatrix, distCoeffs, combined_rvec, combined_tvec);
            } else if (validcnt > 0) {
                RCLCPP_INFO_STREAM(get_logger(), "Using solvepnp");
                cv::Vec3d sum_tvec(0,0,0);
                cv::Vec3d sum_rvec(0,0,0);

                for (int idx : valid_indices) {
                    int id = ids[idx];
                    
                    // 1. Solve PnP for this specific tag (IPPE_SQUARE is extremely stable)
                    cv::Vec3d rvec_single, tvec_single;
                    cv::solvePnP(objPoints, markerCorners.at(idx), camMatrix, distCoeffs, rvec_single, tvec_single, false, cv::SOLVEPNP_IPPE_SQUARE);

                    // 2. Shift to Anchor (Origin)
                    // Retrieve offset
                    cv::Vec3d offset_wall = layout[id]; 
                    
                    // Rotate offset
                    cv::Mat R_cv;
                    cv::Rodrigues(rvec_single, R_cv);
                    Eigen::Matrix3d R_eigen;
                    cv::cv2eigen(R_cv, R_eigen);

                    Eigen::Vector3d p_offset_wall(offset_wall[0], offset_wall[1], offset_wall[2]);
                    Eigen::Vector3d p_offset_cam = R_eigen * p_offset_wall;

                    // Calculate Anchor Position according to THIS tag
                    Eigen::Vector3d t_tag_eigen(tvec_single[0], tvec_single[1], tvec_single[2]);
                    Eigen::Vector3d t_anchor = t_tag_eigen - p_offset_cam;

                    // Accumulate
                    sum_tvec += cv::Vec3d(t_anchor.x(), t_anchor.y(), t_anchor.z());
                    sum_rvec += rvec_single; // Linear sum is okay for small jitter averaging
                }

            // Average the results
            combined_tvec = sum_tvec / (double)validcnt;
            combined_rvec = sum_rvec / (double)validcnt;
            }

            cv::drawFrameAxes(grayImage, camMatrix, distCoeffs, combined_rvec, combined_tvec, markerLength * 1.5f, 2);
        }


        // Rotation and translation vectors
        std::vector<cv::Vec3d> rvecs(nMarkers), tvecs(nMarkers);
        // populate rvecs and tvecs
        if (!ids.empty()) {
            for (size_t i = 0; i < nMarkers; ++i) {
                cv::solvePnP(objPoints, markerCorners.at(i), camMatrix, distCoeffs, rvecs.at(i), tvecs.at(i), false, cv::SOLVEPNP_IPPE_SQUARE);
            }
        }

        // draw results for debugging
        if (!ids.empty()) {
            cv::aruco::drawDetectedMarkers(grayImage, markerCorners, ids);
            // for (size_t i = 0; i < ids.size(); ++i) {
            //     cv::drawFrameAxes(grayImage, camMatrix, distCoeffs, rvecs[i], tvecs[i], markerLength * 1.5f, 2);
            // }
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


        // Print rotation and translation sanity check
        // if (tvecs.size() > 0) {
        //     auto x = tvecs[0][0];
        //     auto y = tvecs[0][1];
        //     auto z = tvecs[0][2];
        //     RCLCPP_INFO_STREAM(get_logger(), "tag id" << ids[0] << "\n");
        //     RCLCPP_INFO_STREAM(get_logger(), "x vector : " << x << "\n");
        //     RCLCPP_INFO_STREAM(get_logger(), "y vector : " << y << "\n");
        //     RCLCPP_INFO_STREAM(get_logger(), "z vector : " << z << "\n");
        //     RCLCPP_INFO_STREAM(get_logger(), "distance : " << std::sqrt(x*x + y*y + z*z) << "\n");
        // }

        // if (rvecs.size() > 0) {
        //     RCLCPP_INFO_STREAM(get_logger(), "roll vector : " << (rvecs[0][0]*180)/M_PI << "\n");
        //     RCLCPP_INFO_STREAM(get_logger(), "pitch vector : " << (rvecs[0][1]*180)/M_PI << "\n");
        //     RCLCPP_INFO_STREAM(get_logger(), "yaw vector : " << (rvecs[0][2]*180)/M_PI << "\n");
        // }

        // Apply Kalman Filter to smooth the pose estimation
        if (!tvecs.empty()) {
            // Pass all vectors and the current ROS time
            // Returns the filtered pose
            geometry_msgs::msg::Pose finalestimation = updateKalmanFilter(combined_tvec, combined_rvec);

            // Draw axes for debugging
            // cv::drawFrameAxes(grayImage, camMatrix, distCoeffs, final_rvec, final_tvec, markerLength * 1.5f, 2);
            cv::imshow("out", grayImage);
            cv::waitKey(1);
            outputToCSV(combined_tvec, combined_rvec);
            return pose_output{finalestimation, combined_rvec[2]*180 / M_PI};

            // outputToCSV(tvecs[0], rvecs[0]);
            // return updateKalmanFilter(tvecs, rvecs);
        } else {
            cv::imshow("out", grayImage);
            cv::waitKey(1);
            return std::nullopt;
        }
    }

    // Helper function to convert OpenCV PnP result to Global Camera Position
    // Returns: Vector3d representing Camera Position in World Frame
    auto KeyboardTypingNode::getGlobalCameraPosition(cv::Vec3d const& rvec,
                                 cv::Vec3d const& tvec,
                                 cv::Vec3d const& tag_offset_world) -> cv::Vec3d {
        // 1. Convert OpenCV Inputs to Eigen types
        // --------------------------------------
        // Translation: cv::Vec3d -> Eigen::Vector3d
        Eigen::Vector3d t_c_t(tvec[0], tvec[1], tvec[2]);

        // Rotation: cv::Vec3d (Rodrigues) -> cv::Mat -> Eigen::Matrix3d
        cv::Mat R_cv;
        cv::Rodrigues(rvec, R_cv);

        Eigen::Matrix3d R_c_t;
        cv::cv2eigen(R_cv, R_c_t);

        // Calculate Camera Position in Tag Frame
        // The transform from Tag to Camera is T_c_t = [R_c_t | t_c_t]
        // We want the inverse transform T_t_c = [R_c_t^T | -R_c_t^T * t_c_t]
        // The camera position in tag frame is the translation part of T_t_c
        Eigen::Vector3d p_cam_in_tag = -R_c_t.transpose() * t_c_t;

        // Calculate Camera Position in World Frame
        // We assume the tag is axis-aligned with the world (Identity rotation)
        // So we just add the tag's world offset
        Eigen::Vector3d t_offset(tag_offset_world[0], tag_offset_world[1], tag_offset_world[2]);
        Eigen::Vector3d p_cam_in_world = p_cam_in_tag + t_offset;

        return cv::Vec3d(p_cam_in_world.x(), p_cam_in_world.y(), p_cam_in_world.z());
    }

    auto KeyboardTypingNode::updateKalmanFilter(cv::Vec3d& tvec, cv::Vec3d& rvec) -> geometry_msgs::msg::Pose {
        // 1. Get current time from the Node's clock
        rclcpp::Time currentTime = this->get_clock()->now();

        // 2. Handle Initialization
        if (!filter_initialized_) {
            last_prediction_time_ = currentTime;
            filter_initialized_ = true;

            // Optional: You might want to initialize the state to the first detection
            // to avoid a long "convergence" time from 0,0,0.
            // kf.statePost.at<float>(0) = tvec[0]; etc...

            return geometry_msgs::msg::Pose();
        }

        // 3. Calculate Delta Time (dt)
        double dt = (currentTime - last_prediction_time_).seconds();
        
        // Update tracker
        last_prediction_time_ = currentTime;

        // 4. PREDICT STEP
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

        // 5. CORRECTION STEP (Single Tag)
        // ---------------------------------------
        
        // --- A. Extract Rotation (Rodrigues -> Euler) ---
        cv::Mat R;
        cv::Rodrigues(rvec, R); // Using the single input rvec
        
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

        // --- B. Angle Wrapping ---
        // Get current Yaw state (Index 8)
        float current_yaw_state = kf.statePost.at<float>(8); 

        // Prevent 'jumps' when crossing PI/-PI boundaries
        while (measured_yaw - current_yaw_state > M_PI) {
            measured_yaw -= 2.0 * M_PI;
        }

        while (measured_yaw - current_yaw_state < -M_PI) {
            measured_yaw += 2.0 * M_PI;
        }

        // --- C. Dynamic Noise (Trust closer tags more) ---
        double dist = cv::norm(tvec); // Using the single input tvec

        // Base noise + Distance Penalty
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
        measurement.at<float>(0) = static_cast<float>(tvec[0]);
        measurement.at<float>(1) = static_cast<float>(tvec[1]);
        measurement.at<float>(2) = static_cast<float>(tvec[2]);
        measurement.at<float>(3) = static_cast<float>(measured_roll);
        measurement.at<float>(4) = static_cast<float>(measured_pitch);
        measurement.at<float>(5) = static_cast<float>(measured_yaw);

        // Perform the correction with the single measurement
        kf.correct(measurement);


        // 6. EXPORT FINAL STATE TO POSE
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

    auto KeyboardTypingNode::outputToCSV(cv::Vec3d &tvec, cv::Vec3d &rvec) -> void {
        static bool is_first_run = true; // Runs only once per program execution
        std::string path = "perception/keyboard_typing/csv_camera_output/test.csv";

        if(is_first_run){
            std::string header;
            std::fstream fin(path, std::ios::in);
            if (fin.is_open()){
                std::getline(fin, header);
                fin.close();
            }

            std::fstream freset(path, std::ios::out | std::ios::trunc);
            if(freset.is_open()){
                if (!header.empty()) freset << header << std::endl;
                freset.close();
            }
            is_first_run = false;
        }

        std::fstream fout(path, std::ios::out | std::ios::app);
        if(fout.is_open()){
            double x = tvec[0];
            double y = tvec[1];
            double z = tvec[2];
            double roll = rvec[0] * 180.0 / M_PI;
            double pitch = rvec[1] * 180.0 / M_PI;
            double yaw = rvec[2] * 180.0 / M_PI;
            
            fout << x << "," << y << "," << z << ","
                << roll << "," << pitch << "," << yaw << "," << std::endl;
            fout.close();
        }
    }    

}

/*
Steps
1. Convert pose estimation code over to c++
    a. Kalman filter the pose at the end of pose estimation
2. Publish yaw to keyboard_yaw in yawCallback
*/