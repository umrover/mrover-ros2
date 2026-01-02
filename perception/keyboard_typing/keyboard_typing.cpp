#include "keyboard_typing.hpp"
#include <cmath>
#include <algorithm>
#include <cctype>
#include <chrono>
#include <cstddef>
#include <geometry_msgs/msg/detail/pose__struct.hpp>
#include <geometry_msgs/msg/detail/vector3__struct.hpp>
#include <memory>
#include <mutex>
#include <opencv2/core/eigen.hpp>
#include "keyboard_typing/constants.h"
#include "lie.hpp"
#include "mrover/msg/detail/ik__struct.hpp"
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
#include <rclcpp_action/client_goal_handle.hpp>
#include <rclcpp_action/create_client.hpp>
#include <rclcpp_action/server.hpp>
#include <thread>
#include <unordered_map>
#include <fstream>
#include <geometry_msgs/msg/vector3.hpp>
#include <mrover/msg/ik.hpp>

namespace mrover{
    KeyboardTypingNode::KeyboardTypingNode(rclcpp::NodeOptions const& options) : rclcpp::Node("keyboard_typing_node", options),  mLoopProfiler{get_logger()}
    {
        RCLCPP_INFO_STREAM(get_logger(), "KeyBoardTypingNode starting up");

        std::vector<ParameterWrapper> params{
                {"min_code_length", mMinCodeLength, 3},
                {"max_code_length", mMaxCodeLength, 6},
        };

        ParameterWrapper::declareParameters(this, params);

        // Set up action server
        mTypingClient = rclcpp_action::create_client<TypingDeltas>(this, "typing_deltas");

        mTypingServer = rclcpp_action::create_server<TypingCode>(
            this,
            "typing_code",
            std::bind(&KeyboardTypingNode::handle_goal, this, std::placeholders::_1, std::placeholders::_2),
            std::bind(&KeyboardTypingNode::handle_cancel, this, std::placeholders::_1),
            std::bind(&KeyboardTypingNode::handle_accepted, this, std::placeholders::_1));

        // subscribe to image stream
        mImageSub = create_subscription<sensor_msgs::msg::Image>("/finger_camera/image", rclcpp::QoS(1), [this](sensor_msgs::msg::Image::ConstSharedPtr const& msg) {
            if (updatePoseEstimate)
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
        cv::setIdentity(kf.processNoiseCov, cv::Scalar(1e-4));
        
        // Measurement noise covariance (sensor uncertainty)
        cv::setIdentity(kf.measurementNoiseCov, cv::Scalar(1e-2));
        
        // Initial state covariance
        cv::setIdentity(kf.errorCovPost, cv::Scalar(1));

        // Initialize state to zeros
        kf.statePost = cv::Mat::zeros(12, 1, CV_32F);

        // create publisher
        mCostMapPub = this->create_publisher<msg::KeyboardYaw>("/keypose/yaw", rclcpp::QoS(1));
        mIKPub = this->create_publisher<msg::IK>("ee_pos_cmd",rclcpp::QoS(1));
        // Define offsets
        layout[4] = cv::Vec3d(0.0, 0.0, 0.0);       // BL
        layout[5] = cv::Vec3d(0.387, 0.0, 0.0);     // BR
        layout[3] = cv::Vec3d(0.387, 0.183, 0.0);   // TR
        layout[2] = cv::Vec3d(0.0, 0.183, 0.0);     // TL

        createRoverBoard();
    }

    auto KeyboardTypingNode::yawCallback(sensor_msgs::msg::Image::ConstSharedPtr const& msg) -> void {
        RCLCPP_INFO_STREAM(get_logger(), "callback");
        sendIKCommand(1.215, .253, .367, 0, 0);

        
    

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
        // for (size_t i = 0; i < tvecs.size(); ++i) {
        //     // Check if the detected tag ID exists in our offset map
        //     if (offset_map.find(ids[i]) != offset_map.end()) {
        //         auto tag_offset_world = offset_map.at(ids[i]);
        //         tvecs[i] = getGlobalCameraPosition(rvecs[i], tvecs[i], tag_offset_world);
        //     } else {
        //         RCLCPP_WARN(get_logger(), "Tag ID %d not found in offset_map, skipping offset", ids[i]);
        //     }
        // }

        // Apply Kalman Filter to smooth the pose estimation
        if (!tvecs.empty()) {
            // Pass all vectors and the current ROS time
            // Returns the filtered pose
            geometry_msgs::msg::Pose finalestimation = updateKalmanFilter(combined_tvec, combined_rvec);

            // Draw after kalman filter
            cv::drawFrameAxes(grayImage, camMatrix, distCoeffs, combined_rvec, combined_tvec, markerLength * 3.0f, 4);

            // Draw axes for debugging
            // cv::drawFrameAxes(grayImage, camMatrix, distCoeffs, final_rvec, final_tvec, markerLength * 1.5f, 2);
            cv::imshow("out", grayImage);
            int key = cv::waitKey(1) & 0xFF;
            if(key == 'r'){
                logPose = !logPose;
                RCLCPP_INFO_STREAM(get_logger(), "Toggled logPose: " << (logPose ? "ON" : "OFF"));
        }
            if(logPose){
                outputToCSV(combined_tvec, combined_rvec);
            }

            mCameraToKey = SE3Conversions::fromPose(finalestimation);

            return pose_output{finalestimation, combined_rvec[2]*180 / M_PI};

            // outputToCSV(tvecs[0], rvecs[0]);
            // return updateKalmanFilter(tvecs, rvecs);
        } else {
            cv::imshow("out", grayImage);
            cv::waitKey(1);
            return std::nullopt;
        }
    }

    // auto KeyboardTypingNode::getKeyToCameraTransform(cv::Vec3d const& rvec,
    //                                              cv::Vec3d const& tvec,
    //                                              cv::Vec3d const& tag_offset_key) -> cv::Mat {
        
    //     // 1. Define "Tag in Camera" (T_cam_tag)
    //     // Isometry3d is a 4x4 matrix specifically for rigid transforms (Rotation + Translation)
    //     Eigen::Isometry3d T_cam_tag = Eigen::Isometry3d::Identity();
        
    //     // Transform the rvec into eigen rotation matrix
    //     cv::Mat R_cv;
    //     cv::Rodrigues(rvec, R_cv);
    //     Eigen::Matrix3d R_eigen;
    //     cv::cv2eigen(R_cv, R_eigen);
        
    //     // Set Rotation
    //     T_cam_tag.linear() = R_eigen;  
    //     // Set Translation   
    //     T_cam_tag.translation() = Eigen::Vector3d(tvec[0], tvec[1], tvec[2]);

    //     // Transform from tag to key (only translation since on same plane as keyboard)
    //     Eigen::Isometry3d T_key_tag = Eigen::Isometry3d::Identity();
    //     T_key_tag.translation() = Eigen::Vector3d(tag_offset_key[0], tag_offset_key[1], tag_offset_key[2]);

    //     // Logic: Camera <-- Tag <-- Key
    //     // Math:  T_cam_key = T_cam_tag * T_key_tag.inverse()
    //     Eigen::Isometry3d T_cam_key = T_cam_tag * T_key_tag.inverse();

    //     // OpenCV Matrix
    //     cv::Mat T_out;
    //     cv::eigen2cv(T_cam_key.matrix(), T_out);
    //     return T_out;
    // }

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

        // Update combined tvecs and rvecs
        tvec[0] = filtered_pose.position.x;
        tvec[1] = filtered_pose.position.y;
        tvec[2] = filtered_pose.position.z;

        rvec[0] = final_roll;
        rvec[1] = final_pitch;
        rvec[2] = final_yaw;
        
        return filtered_pose;
    }

    auto KeyboardTypingNode::sendIKCommand(float x, float y, float z, float pitch, float roll) -> void {
        if(!mIKPub){
            RCLCPP_ERROR(get_logger(), "IK publisher not initialized");
        }
        msg::IK message;
        message.pos.x = x;
        message.pos.y = y;
        message.pos.z = z;
        message.pitch = pitch;
        message.roll = roll;
        mIKPub->publish(message);

        RCLCPP_INFO(get_logger(), "Published IK Command {x=%.3f, y=%.3f, z=%.3f}", x, y, z);
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

    // ----------------- ACTION CLIENT/SERVER ------------------
    
    // Typing IK action client functions (communication with Nav)
    auto KeyboardTypingNode::send_goal(float x_delta, float y_delta) -> bool {
        if (!mTypingClient->wait_for_action_server()) {
            RCLCPP_INFO_STREAM(this->get_logger(), "Typing Deltas Action Server not available");
            return false;
        }

        auto goal_msg = TypingDeltas::Goal();
        goal_msg.x_delta = x_delta;
        goal_msg.y_delta = y_delta;

        auto send_goal_options = rclcpp_action::Client<TypingDeltas>::SendGoalOptions();
        send_goal_options.goal_response_callback = std::bind(&KeyboardTypingNode::goal_response_callback, this, std::placeholders::_1);
        send_goal_options.feedback_callback = std::bind(&KeyboardTypingNode::feedback_callback, this, std::placeholders::_1, std::placeholders::_2);
        send_goal_options.result_callback = std::bind(&KeyboardTypingNode::result_callback, this, std::placeholders::_1);

        mTypingClient->async_send_goal(goal_msg, send_goal_options);
        return true;
    }

    void KeyboardTypingNode::goal_response_callback(const GoalHandleTypingDeltas::SharedPtr & goal_handle) {}

    void KeyboardTypingNode::feedback_callback(GoalHandleTypingDeltas::SharedPtr, const std::shared_ptr<const TypingDeltas::Feedback> feedback) {
        RCLCPP_INFO_STREAM(get_logger(), std::format("Feedback: {}", feedback->dist_remaining));
    }

    void KeyboardTypingNode::result_callback(const GoalHandleTypingDeltas::WrappedResult & result) {
        std::unique_lock<std::mutex> guard(mActionMutex);
        mGoalReached = (result.code == rclcpp_action::ResultCode::SUCCEEDED);
        mActionCV.notify_all();
    }

    // Typing IK action server functions (communication with teleop)
    auto KeyboardTypingNode::handle_goal(const rclcpp_action::GoalUUID & uuid,std::shared_ptr<const TypingCode::Goal> goal) -> rclcpp_action::GoalResponse {
        std::unique_lock<std::mutex> guard(mActionMutex);
        // Reject goal if code length is incorrect, code already active, or not letters
        if (goal->launch_code.length() < mMinCodeLength || goal->launch_code.length() > mMaxCodeLength) {
            RCLCPP_WARN(this->get_logger(), "Launch code length must be in between %d and %d!", mMinCodeLength, mMaxCodeLength);
            return rclcpp_action::GoalResponse::REJECT;
        }

        if (mLaunchCode.has_value()) {
            RCLCPP_WARN(this->get_logger(), "Launch code already active!");
            return rclcpp_action::GoalResponse::REJECT;
        }

        std::string temp = goal->launch_code;
        std::transform(temp.begin(), temp.end(), temp.begin(), ::toupper);
        if (!std::all_of(temp.begin(), temp.end(), [](unsigned char c){
            return std::isalpha(c);
        })) {
            RCLCPP_WARN(this->get_logger(), "All keys must be letters!");
            return rclcpp_action::GoalResponse::REJECT;
        }

        mTypingUUID = uuid;
        mLaunchCode = temp;
        return rclcpp_action::GoalResponse::ACCEPT_AND_EXECUTE;
    }

    auto KeyboardTypingNode::handle_cancel(const std::shared_ptr<GoalHandleTypingCode> goal_handle) -> rclcpp_action::CancelResponse {
        std::unique_lock<std::mutex> guard(mActionMutex);
        if (!mTypingUUID || goal_handle->get_goal_id() != mTypingUUID.value()) {
            RCLCPP_WARN(this->get_logger(), "Invalid Cancel ID");
            return rclcpp_action::CancelResponse::REJECT;
        }

        RCLCPP_INFO_STREAM(get_logger(), std::format("Cacelling goal {}", mLaunchCode.value()));
        mLaunchCode = std::nullopt;
        return rclcpp_action::CancelResponse::ACCEPT;
    }

    void KeyboardTypingNode::handle_accepted(const std::shared_ptr<GoalHandleTypingCode> goal_handle) {
        std::thread([this, goal_handle]() {
            std::unique_lock<std::mutex> guard(mActionMutex);
            auto result = std::make_shared<TypingCode::Result>();
            auto feedback = std::make_shared<TypingCode::Feedback>();

            result->success = false;

            for (int i = 0; i < mLaunchCode.value().length(); i++) {
                feedback->current_index = static_cast<uint32_t>(i);
                goal_handle->publish_feedback(feedback);

                RCLCPP_WARN(this->get_logger(), "Sending Goal");

                // TODO logic for figuring out deltas
                // send_goal(x, y);

                while (!mGoalReached.has_value()) { // TODO add logic for if goal is cancelled by teleop
                    mActionCV.wait(guard);
                } // Block until goal reached

                // TODO add handling for if goal fails
                mGoalReached = std::nullopt;
            }

            result->success = true;
            mLaunchCode = std::nullopt;
            mTypingUUID = std::nullopt;
            goal_handle->succeed(result);
        }).detach();
    }

}

/*
Steps
1. Convert pose estimation code over to c++
    a. Kalman filter the pose at the end of pose estimation
2. Publish yaw to keyboard_yaw in yawCallback
*/