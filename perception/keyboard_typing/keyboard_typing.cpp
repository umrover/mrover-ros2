#include "keyboard_typing.hpp"
#include "keyboard_typing/pch.hpp"
#include "lie.hpp"
#include "mrover/action/detail/typing_position__struct.hpp"
#include "mrover/srv/detail/ik_mode__struct.hpp"
#include "mrover/srv/detail/pusher__struct.hpp"
#include <Eigen/src/Core/Matrix.h>
#include <Eigen/src/Geometry/Quaternion.h>
#include <cmath>
#include <cstddef>
#include <limits>
#include <opencv2/calib3d.hpp>
#include <Eigen/Eigenvalues>
#include <opencv2/core/mat.hpp>
#include <opencv2/core/matx.hpp>
#include <rclcpp/logging.hpp>


namespace mrover{
    KeyboardTypingNode::KeyboardTypingNode(rclcpp::NodeOptions const& options) : rclcpp::Node("keyboard_typing_node", options),  mLoopProfiler{get_logger()}
    {
        RCLCPP_INFO_STREAM(get_logger(), "KeyBoardTypingNode starting up");

        std::vector<ParameterWrapper> params{
                {"min_code_length", mMinCodeLength, 3},
                {"max_code_length", mMaxCodeLength, 6},
                {"tag_size", mTagSize, 0.02},
        };

        ParameterWrapper::declareParameters(this, params);

        // Declare vector params manually since param wrapper doesnt support vector
        std::map<std::string, rclcpp::ParameterValue> vector_params;
        vector_params["camera_matrix"] = rclcpp::ParameterValue(std::vector<double>(9, 0.0));
        vector_params["distortion_coefficients"] = rclcpp::ParameterValue(std::vector<double>(5, 0.0));
        vector_params["tag_offsets"] = rclcpp::ParameterValue(std::vector<double>{});
        vector_params["z_key_transform"] = rclcpp::ParameterValue(std::vector<double>{});

        this->declare_parameters("", vector_params);
        std::vector<double> cam_raw = this->get_parameter("camera_matrix").as_double_array();
        std::vector<double> dist_raw = this->get_parameter("distortion_coefficients").as_double_array();
        std::vector<double> zTransformRaw = this->get_parameter("z_key_transform").as_double_array();

        // Read in Camera intrinsics
        mCameraMatrix = cv::Mat(3, 3, CV_64F, cam_raw.data()).clone();
        mDistCoeffs = cv::Mat(1, (int)dist_raw.size(), CV_64F, dist_raw.data()).clone();
        mZKeyTransform = Eigen::Vector3d::Map(zTransformRaw.data());

        // Grab tag offsets
        std::vector<double> offset_raw = this->get_parameter("tag_offsets").as_double_array();

        for (size_t i = 0; i + 3 < offset_raw.size(); i += 4) {
            int id = static_cast<int>(offset_raw[i]);
            float x = static_cast<float>(offset_raw[i+1]);
            float y = static_cast<float>(offset_raw[i+2]);
            float z = static_cast<float>(offset_raw[i+3]);
            
            layout[id] = cv::Vec3d(x, y, z);
        }
        
        // Set up action server
        mTypingClient = rclcpp_action::create_client<action::TypingPosition>(this, "typing_ik");

        mTypingServer = rclcpp_action::create_server<TypingCode>(
            this,
            "es_typing_code",
            std::bind(&KeyboardTypingNode::handle_goal, this, std::placeholders::_1, std::placeholders::_2),
            std::bind(&KeyboardTypingNode::handle_cancel, this, std::placeholders::_1),
            std::bind(&KeyboardTypingNode::handle_accepted, this, std::placeholders::_1));

        // subscribe to image stream
        mImageSub = create_subscription<sensor_msgs::msg::Image>("/video4/image", rclcpp::QoS(1), [this](sensor_msgs::msg::Image::ConstSharedPtr const& msg) {
            yawCallback(msg);
        });

        // grab transform from finger_cam to gripper
        // wait until the transformation is acquired
        while (true) {
            try {
                cam_to_gripper = SE3Conversions::fromTfTree(tf_buffer, "finger_camera_frame", "arm_fk");
                break;
            } catch (tf2::TransformException const& e) {
                RCLCPP_WARN_STREAM_THROTTLE(get_logger(), *get_clock(), 1000, std::format("TF tree error processing keyboard typing: {}", e.what()));
            }
        }

        // Create Ik mode client
        mIkModeClient = create_client<srv::IkMode>("ik_mode");

        current_key = "";

        // create publisher
        mYawPub = this->create_publisher<msg::KeyboardYaw>("/Keyboard/Yaw", rclcpp::QoS(1));

        mIKPub = this->create_publisher<msg::IK>("ik_pos_cmd",rclcpp::QoS(1));
    }

    auto KeyboardTypingNode::yawCallback(sensor_msgs::msg::Image::ConstSharedPtr const& msg) -> void {
        // If we are still updating pose estimate (i.e. no launch code sent, send IK and update pose)
        std::optional<pose_output> output = std::nullopt;
        if (mUpdatePoseEstimate) {
            // sendIKCommand(1.100, .253, .367, 0, 0);
            output = estimatePose(msg);
        }

        // Publish yaw & se3d
        if (output.has_value()) {
            mrover::msg::KeyboardYaw msg;
            msg.yaw = output->yaw;
            mYawPub->publish(msg);

            // Output debug
            // RCLCPP_INFO_STREAM(get_logger(), "X: " << output->pose.position.x);
            // RCLCPP_INFO_STREAM(get_logger(), "Y: " << output->pose.position.y);
            // RCLCPP_INFO_STREAM(get_logger(), "Z: " << output->pose.position.z);

            // Convert pose to se3d
            SE3d cam_to_tag = SE3Conversions::fromPose(output->pose);

            // Transform from camera frame to arm_fk frame:
            // Camera X (horizontal) -> arm_fk Y
            // Camera Y (vertical)   -> arm_fk Z
            // Camera Z (forward)    -> arm_fk X

            // open cv coordinates
            // +X is right
            // -Y is up
            // +Z is forward

            // arm_fk coordinates
            // -Y is right
            // +Z is up
            // +X is forwards

            Eigen::Vector3d cam_pos = cam_to_tag.translation();
            Eigen::Vector3d arm_fk_pos(cam_pos.z(),-cam_pos.x(),-cam_pos.y());

            // Rotation to convert camera frame orientation to arm_fk frame
            // This rotation matrix maps: X->Y, Y->Z, Z->X
            Eigen::Matrix3d cam_to_arm_rotation;
            cam_to_arm_rotation << 0, 0, 1,   // arm_fk X comes from camera Z
                                   -1, 0, 0,   // arm_fk Y comes from camera X
                                   0, -1, 0;   // arm_fk Z comes from camera Y

            Eigen::Quaterniond cam_rot(cam_to_tag.rotation());
            Eigen::Quaterniond frame_rotation(cam_to_arm_rotation);
            Eigen::Quaterniond transformed_rotation = (frame_rotation * cam_rot).normalized();

            SE3d arm_fk_to_tag{arm_fk_pos, transformed_rotation};

            // Publish to tf tree
            SE3Conversions::pushToTfTree(tf_broadcaster, "keyboard_tag", "arm_fk", cam_to_gripper*arm_fk_to_tag, get_clock()->now());

            // camera_to_tag
            // camera_to_gripper * camera_to_tag

            // // Initialize transforms for every key, temporarily here for now
            SE3d z_to_tag{mZKeyTransform, Eigen::Quaterniond::Identity()};
            SE3Conversions::pushToTfTree(tf_broadcaster, "keyboard_z", "keyboard_tag", z_to_tag, get_clock()->now());
        }
    }

    auto KeyboardTypingNode::estimatePose(sensor_msgs::msg::Image::ConstSharedPtr const& msg) -> std::optional<pose_output>  {
        // Read in images
        cv::Mat bgraImage{static_cast<int>(msg->height), static_cast<int>(msg->width), CV_8UC4, const_cast<uint8_t*>(msg->data.data())};

        // For debugging image
        cv::Mat bgrImage;
        cv::cvtColor(bgraImage, bgrImage, cv::COLOR_BGRA2BGR);

        // convert to gray for estimation
        cv::Mat grayImage;
        cv::cvtColor(bgraImage, grayImage, cv::COLOR_BGRA2GRAY);
        // Optional: enhance contrast
        // cv::equalizeHist(grayImage, grayImage);

        cv::Ptr<cv::CLAHE> clahe = cv::createCLAHE();
        clahe->setClipLimit(2.0); 
        clahe->setTilesGridSize(cv::Size(8, 8));

        clahe->apply(grayImage, grayImage);

        // Optional: apply Gaussian blur to reduce noise
        // cv::GaussianBlur(grayImage, grayImage, cv::Size(5,5), 0);

        // Define variables

        std::vector<std::vector<cv::Point2f>> markerCorners, rejectedCandidates;
        std::vector<int> ids;
        cv::Ptr<cv::aruco::DetectorParameters> detectorParams = cv::aruco::DetectorParameters::create();

        // Could potentially run a mild gaussian blur plus sharpening kernel if instability is encountered

        // detectorParams->adaptiveThreshWinSizeMin = 3;
        // detectorParams->adaptiveThreshWinSizeMax = 23;
        // detectorParams->adaptiveThreshWinSizeStep = 2;

        // detectorParams->adaptiveThreshConstant = 7;

        // detectorParams->perspectiveRemovePixelPerCell = 3;

        // detectorParams->minDistanceToBorder = 1;   // default is 3
        // detectorParams->minMarkerPerimeterRate = 0.05;   // default ~0.03

        detectorParams->cornerRefinementMethod = cv::aruco::CORNER_REFINE_SUBPIX;

        // detectorParams->polygonalApproxAccuracyRate = 0.05;

        // detectorParams->cornerRefinementWinSize = 3;

        // detectorParams->cornerRefinementMaxIterations = 20;

        // detectorParams->errorCorrectionRate = 0.8;

        // detectorParams->maxErroneousBitsInBorderRate = 0.5;

        // detectorParams->minCornerDistanceRate = 0.05;

        // Define coordinate system
        float markerLength = 0.019;  // meters
        std::vector<cv::Point3f> objPoints = {
            {-markerLength/2.f,  markerLength/2.f, 0},
            { markerLength/2.f,  markerLength/2.f, 0},
            { markerLength/2.f, -markerLength/2.f, 0},
            {-markerLength/2.f, -markerLength/2.f, 0}
        };

        // Detect Markers
        cv::aruco::detectMarkers(grayImage, dictionary, markerCorners, ids, detectorParams, rejectedCandidates);

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

            float tag_size = 0.02f; 
            float half_size = tag_size / 2.0f;

            std::vector<cv::Point3f> all_objPoints;
            std::vector<cv::Point2f> all_imgPoints; 

            if (validcnt > 0) {
                for (int idx : valid_indices) {
                    int id = ids[idx];
                    
                    cv::Point3f tag_center(layout[id][0], layout[id][1], layout[id][2]);
                    
                    all_objPoints.push_back(tag_center + cv::Point3f(-half_size,  half_size, 0)); // Top-Left
                    all_objPoints.push_back(tag_center + cv::Point3f( half_size,  half_size, 0)); // Top-Right
                    all_objPoints.push_back(tag_center + cv::Point3f( half_size, -half_size, 0)); // Bottom-Right
                    all_objPoints.push_back(tag_center + cv::Point3f(-half_size, -half_size, 0)); // Bottom-Left

                    for (int i = 0; i < 4; i++) {
                        all_imgPoints.push_back(markerCorners.at(idx)[i]);
                    }
                }
                cv::solvePnP(all_objPoints, all_imgPoints, mCameraMatrix, mDistCoeffs, combined_rvec, combined_tvec, false, cv::SOLVEPNP_ITERATIVE);
            }
        }


        // Rotation and translation vectors
        std::vector<cv::Vec3d> rvecs(nMarkers), tvecs(nMarkers);
        // populate rvecs and tvecs
        if (!ids.empty()) {
            for (size_t i = 0; i < nMarkers; ++i) {
                cv::solvePnP(objPoints, markerCorners.at(i), mCameraMatrix, mDistCoeffs, rvecs.at(i), tvecs.at(i), false, cv::SOLVEPNP_IPPE_SQUARE);
            }
        }

        // draw results for debugging
        if (!ids.empty()) {
            cv::aruco::drawDetectedMarkers(bgrImage, markerCorners, ids);
            for (size_t i = 0; i < ids.size(); ++i) {
                cv::drawFrameAxes(bgrImage, mCameraMatrix, mDistCoeffs, rvecs[i], tvecs[i], markerLength * 1.5f, 2);
            }
        }

        if (!tvecs.empty()) {
            // Pass all vectors and the current ROS time
            // Returns the filtered pose
            geometry_msgs::msg::Pose finalestimation;

            std::tie(combined_tvec, combined_rvec) = vectorMedianFilter(combined_tvec, combined_rvec);

            RCLCPP_INFO_STREAM(get_logger(), "X: " << combined_tvec[0]);
            RCLCPP_INFO_STREAM(get_logger(), "Y: " << combined_tvec[1]);
            RCLCPP_INFO_STREAM(get_logger(), "Z: " << combined_tvec[2]);

            Eigen::Vector3d rvec(
                combined_rvec[0],
                combined_rvec[1],
                combined_rvec[2]
            );

            double angle = rvec.norm();
            Eigen::Quaterniond q;

            if (angle < 1e-12) {
                q.setIdentity();
            } else {
                Eigen::Vector3d axis = rvec / angle;
                q = Eigen::AngleAxisd(angle, axis);
            }

            finalestimation.orientation.x = q.x();
            finalestimation.orientation.y = q.y();
            finalestimation.orientation.z = q.z();
            finalestimation.orientation.w = q.w();

            finalestimation.position.x = combined_tvec[0];
            finalestimation.position.y = combined_tvec[1];
            finalestimation.position.z = combined_tvec[2];

            double siny = 2.0 * (q.w() * q.y() - q.z() * q.x());
            double yaw_rad;
            if (std::abs(siny) >= 1)
                yaw_rad = std::copysign(M_PI / 2, siny);
            else
                yaw_rad = std::asin(siny);

            double yaw_deg = yaw_rad * 180.0 / M_PI;

            // Draw after kalman filter
            // cv::drawFrameAxes(grayImage, camMatrix, distCoeffs, combined_rvec, combined_tvec, markerLength * 3.0f, 4);
            cv::drawFrameAxes(bgrImage, mCameraMatrix, mDistCoeffs, combined_rvec, combined_tvec, markerLength * 1.5f, 2);

            // Draw circle for debugging
            int cx = grayImage.cols / 2;
            int cy = grayImage.rows / 2;

            cv::circle(
                bgrImage,
                cv::Point(cx, cy),
                2,                  // radius
                cv::Scalar(0, 0, 255), // BGR: red
                cv::FILLED
            );

            // Draw axes for debugging
            // cv::drawFrameAxes(grayImage, camMatrix, distCoeffs, final_rvec, final_tvec, markerLength * 1.5f, 2);
            cv::imshow("out1", bgrImage);
            // cv::imshow("out", grayImage);
            int key = cv::waitKey(1) & 0xFF;
            if(key == 'r'){
                // logPose = !logPose;
                // RCLCPP_INFO_STREAM(get_logger(), "Toggled logPose: " << (logPose ? "ON" : "OFF"));
                align_arm();
                // send_z_key_command();
            }
            if(key == 't'){
                align_to_z();
            }
            if(key == 'h'){
                SE3d transform = SE3Conversions::fromTfTree(tf_buffer, "arm_fk", "arm_gripper_link");
                RCLCPP_INFO_STREAM(get_logger(), "y: " << transform.translation().y());
                RCLCPP_INFO_STREAM(get_logger(), "z: " << transform.translation().z());
            }
            // if(logPose){
            //     outputToCSV(combined_tvec, combined_rvec);
            // }

            return pose_output{finalestimation, yaw_deg};
        } else {
            cv::imshow("out1", bgraImage);
            cv::waitKey(1);
            return std::nullopt;
        }
    }

    auto KeyboardTypingNode::vectorMedianFilter(cv::Vec3d tvec, cv::Vec3d rvec) -> std::pair<cv::Vec3d, cv::Vec3d> {
        if (tvec_window.size() > 4) {
            tvec_window.pop_front();
            rvec_window.pop_front();
        }
        tvec_window.push_back(tvec);
        rvec_window.push_back(rvec);

        cv::Vec3d medianTvec = tvec_window.back();
        double min_sum = std::numeric_limits<double>::max();

        // Take the median
        int filtered_idx = 0;

        // Find median rvec
        for (size_t i = 0; i < rvec_window.size(); ++i) {
            double angle_diff_radians = 0;
            double currentTvecSum = 0;
            for (size_t j = 0; j < rvec_window.size(); ++j) {
                if (i == j) {
                    continue;
                }
                cv::Mat R_i;
                cv::Rodrigues(rvec_window[i], R_i);
                cv::Mat R_j;
                cv::Rodrigues(rvec_window[j], R_j);

                // Compute diff and then convert back to rodrigues to get magnitude
                cv::Mat R_diff = R_i * R_j.t(); // transpose is inverse b/c orthogonal, so computes difference
                cv::Mat axis_angle_vec;
                cv::Rodrigues(R_diff, axis_angle_vec);

                angle_diff_radians += cv::norm(axis_angle_vec);
                currentTvecSum += cv::norm(tvec_window[i] - tvec_window[j]);
            }
            double min_sum_current = angle_diff_radians + currentTvecSum;
            if(min_sum_current < min_sum) {
                min_sum = min_sum_current;
                filtered_idx = i;
            }
        }


        return {tvec_window[filtered_idx], rvec_window[filtered_idx]};
    }

    auto KeyboardTypingNode::align_arm() -> void {
        // Grab gripper_to_tag and then calculate deltas
        SE3d gripper_to_tag;
        SE3d armbase_to_armfk;

        try {
            gripper_to_tag = SE3Conversions::fromTfTree(tf_buffer, "keyboard_tag", "arm_base_link");
            armbase_to_armfk = SE3Conversions::fromTfTree(tf_buffer, "arm_fk", "arm_base_link");

            // Grab pitch from tag transform
            double r00 = gripper_to_tag.transform()(0,0);
            double r10 = gripper_to_tag.transform()(1,0);
            double r20 = gripper_to_tag.transform()(2,0);

            double pitch_rad = -std::atan2(-r20, std::hypot(r00, r10));

            RCLCPP_INFO_STREAM(this->get_logger(), "pitch = " << pitch_rad);

            sendIKCommand(armbase_to_armfk.translation().x(), gripper_to_tag.translation().y(), gripper_to_tag.translation().z(), 0, 0);

        } catch (tf2::TransformException const& e) {
            RCLCPP_WARN_STREAM_THROTTLE(get_logger(), *get_clock(), 1000, std::format("TF tree error processing keyboard typing: {}", e.what()));
        }
    }

    auto KeyboardTypingNode:: align_to_z() -> void {
        // Grab gripper_to_tag and then calculate deltas
        SE3d armbase_to_z;
        SE3d armbase_to_armfk;

        try {
            armbase_to_z = SE3Conversions::fromTfTree(tf_buffer, "keyboard_z", "arm_base_link");
            armbase_to_armfk = SE3Conversions::fromTfTree(tf_buffer, "arm_fk", "arm_base_link");

            // Grab pitch from tag transform
            double r00 = armbase_to_z.transform()(0,0);
            double r10 = armbase_to_z.transform()(1,0);
            double r20 = armbase_to_z.transform()(2,0);

            double pitch_rad = -std::atan2(-r20, std::hypot(r00, r10));

            sendIKCommand(armbase_to_armfk.translation().x(),
            armbase_to_z.translation().y(), armbase_to_z.translation().z(), pitch_rad, 0);

            RCLCPP_INFO_STREAM(get_logger(), "y_delta = " << (armbase_to_armfk.translation().y() - armbase_to_z.translation().y()));
                            
        } catch (tf2::TransformException const& e) {
            RCLCPP_WARN_STREAM_THROTTLE(get_logger(), *get_clock(), 1000, std::format("TF tree error processing keyboard typing: {}", e.what()));
        }

        current_key = 'z';
    }

    auto KeyboardTypingNode::sendIKCommand(float x, float y, float z, float pitch, float roll) -> void {
        if(!mIKPub){
            RCLCPP_ERROR(get_logger(), "IK publisher not initialized");
        }
        // shift over to arm_gripper_link
        float y_offset_local = -0.0062535;

        float dx = y_offset_local * (std::sin(pitch) * std::sin(roll));
        float dy = y_offset_local * std::cos(roll);
        float dz = y_offset_local * (std::cos(pitch) * std::sin(roll));

        msg::IK message;

        message.pos.x = x + dx;
        message.pos.y = y + dy;
        message.pos.z = z + dz;

        message.pitch = pitch;
        message.roll = roll;

        using clock = std::chrono::steady_clock;
        auto start = clock::now();
        auto duration = std::chrono::duration<double>(10);

        SE3d curarmpos = SE3Conversions::fromTfTree(tf_buffer, "arm_fk", "arm_base_link");

        double dist =  pow(curarmpos.translation().x()-x, 2) + pow(curarmpos.translation().y()-y,2);
        while (clock::now() - start < duration || dist > 0.007) {
            mIKPub->publish(message);
            curarmpos = SE3Conversions::fromTfTree(tf_buffer, "arm_fk", "arm_base_link");
            dist =  pow(curarmpos.translation().x()-x, 2) + pow(curarmpos.translation().y()-y,2);
            RCLCPP_INFO_STREAM(get_logger(), "remaining distance = " << dist);
        }

        RCLCPP_INFO(get_logger(), "Published IK Command {x=%.3f, y=%.3f, z=%.3f, p=%.3f, r=%.3f}", x, y, z, pitch, roll);
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

    // ----------------------- ACTION SERVER/CLIENT ---------------------------
    // Typing IK action client functions (communication with Nav)
    auto KeyboardTypingNode::send_goal(float x_delta, float y_delta) -> bool {
        if (!mTypingClient->wait_for_action_server()) {
            RCLCPP_INFO_STREAM(this->get_logger(), "Typing Deltas Action Server not available");
            return false;
        }

        RCLCPP_INFO_STREAM(this->get_logger(), "Sending Goal");

        auto goal_msg = action::TypingPosition::Goal();
        goal_msg.x = x_delta;
        goal_msg.y = y_delta;

        auto send_goal_options = rclcpp_action::Client<TypingPosition>::SendGoalOptions();
        // send_goal_options.goal_response_callback = std::bind(&KeyboardTypingNode::goal_response_callback, this, std::placeholders::_1);
        send_goal_options.feedback_callback = std::bind(&KeyboardTypingNode::feedback_callback, this, std::placeholders::_1, std::placeholders::_2);
        // send_goal_options.result_callback = std::bind(&KeyboardTypingNode::result_callback, this, std::placeholders::_1);

        std::shared_future<GoalHandleTypingPosition::SharedPtr> future_goal_handle = mTypingClient->async_send_goal(goal_msg, send_goal_options);
        RCLCPP_INFO_STREAM(this->get_logger(), "Sent first goal, waiting for response");
        future_goal_handle.wait();

        if (future_goal_handle.get() == nullptr)
            return false;

        std::shared_future<GoalHandleTypingPosition::WrappedResult> future_result = mTypingClient->async_get_result(future_goal_handle.get());
        future_result.wait();
        return (future_result.get().code == rclcpp_action::ResultCode::SUCCEEDED);
    }

    void KeyboardTypingNode::feedback_callback(GoalHandleTypingPosition::SharedPtr, const std::shared_ptr<const TypingPosition::Feedback> feedback) {
        // RCLCPP_INFO_STREAM(get_logger(), std::format("Feedback: {}", feedback->dist_remaining));
    }

    // void KeyboardTypingNode::result_callback(const GoalHandleTypingPosition::WrappedResult & result) {
    //     mGoalReached = (result.code == rclcpp_action::ResultCode::SUCCEEDED);
    // }

    // Typing IK action server functions (communication with teleop)
    auto KeyboardTypingNode::handle_goal(const rclcpp_action::GoalUUID & uuid,std::shared_ptr<const TypingCode::Goal> goal) -> rclcpp_action::GoalResponse {
        // Reject goal if code length is incorrect, code already active, or not letters
        if (goal->launch_code.length() < mMinCodeLength || goal->launch_code.length() > mMaxCodeLength) {
            RCLCPP_WARN(this->get_logger(), "Launch code length must be in between %d and %d!", mMinCodeLength, mMaxCodeLength);
            return rclcpp_action::GoalResponse::REJECT;
        }

        if (mAcceptedGoalHandle && mAcceptedGoalHandle->is_active()) {
            RCLCPP_WARN(this->get_logger(), "Launch code already active!");
            return rclcpp_action::GoalResponse::REJECT;
        }

        std::string temp = goal->launch_code;
        if (!std::all_of(temp.begin(), temp.end(), [](unsigned char c){
            return std::isalpha(c);
        })) {
            RCLCPP_WARN(this->get_logger(), "All keys must be letters!");
            return rclcpp_action::GoalResponse::REJECT;
        }

        mUpdatePoseEstimate = false; // Once we accept a goal, we no longer want to be updating the pose estimate

        RCLCPP_INFO_STREAM(get_logger(), std::format("Goal Accepted: {}", temp));
        return rclcpp_action::GoalResponse::ACCEPT_AND_EXECUTE;
    }

    auto KeyboardTypingNode::handle_cancel(const std::shared_ptr<GoalHandleTypingCode> goal_handle) -> rclcpp_action::CancelResponse {
        if (goal_handle != mAcceptedGoalHandle) {
            RCLCPP_WARN(this->get_logger(), "Invalid Cancel ID");
            return rclcpp_action::CancelResponse::REJECT;
        }

        RCLCPP_INFO_STREAM(get_logger(), std::format("Cacelling goal {}", goal_handle->get_goal()->launch_code));
        mUpdatePoseEstimate = true;
        mAcceptedGoalHandle = nullptr;
        mTypingClient->async_cancel_all_goals(); // Cancel goal send to nav
        return rclcpp_action::CancelResponse::ACCEPT;
    }

    void KeyboardTypingNode::handle_accepted(const std::shared_ptr<GoalHandleTypingCode> goal_handle) {
        mAcceptedGoalHandle = goal_handle; // Set goal handle in the executor thread, not a new one

        std::thread([this, goal_handle]() {
            auto result = std::make_shared<TypingCode::Result>();
            auto feedback = std::make_shared<TypingCode::Feedback>();

            result->success = false;

            std::string launchCode = goal_handle->get_goal()->launch_code;
            std::transform(launchCode.begin(), launchCode.end(), launchCode.begin(), ::toupper);

            // Align arm over z key
            // RCLCPP_INFO_STREAM(this->get_logger(), "Aligning to z key");
            align_to_z();

            // rotate gripper 90 degrees
            // RCLCPP_INFO_STREAM(this->get_logger(), "Rotating gripper 90 degrees");
            // rotateGripper(-1.5708);

            // Activate typing mode
            RCLCPP_INFO_STREAM(this->get_logger(), "Sending ik mode request");
            auto ik_req = std::make_shared<srv::IkMode::Request>();
            ik_req->mode = srv::IkMode::Request::TYPING;
            mIkModeClient->async_send_request(ik_req);

            RCLCPP_INFO_STREAM(this->get_logger(), "Typing Sequence starting");
            // Start typing sequence
            for (size_t i = 0; i < launchCode.length(); i++) {
                feedback->current_index = static_cast<uint32_t>(i);
                goal_handle->publish_feedback(feedback);

                RCLCPP_WARN(this->get_logger(), "Sending Goal");

                float x_delta = 0;
                float y_delta = 0;

                x_delta = keyboard_offset[launchCode[i]][0];
                y_delta = keyboard_offset[launchCode[i]][1];

                RCLCPP_WARN(this->get_logger(), "X_pos", mMinCodeLength, mMaxCodeLength);

                RCLCPP_INFO_STREAM(this->get_logger(), "current letter = " << launchCode[i]);
                RCLCPP_INFO_STREAM(this->get_logger(), "x_delta = " << x_delta);
                RCLCPP_INFO_STREAM(this->get_logger(), "y_delta = " << y_delta);

                send_goal(-x_delta, y_delta);

                if (goal_handle->is_canceling()) {
                    result->success = false;
                    goal_handle->canceled(result);
                    return;
                } // Check if goal canceled again because we returned from a long function

                // TODO add handling for if goal fails

                current_key = launchCode[i];
            }

            result->success = true;
            mUpdatePoseEstimate = true;
            goal_handle->succeed(result);
        }).detach();
    }
}