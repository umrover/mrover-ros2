#include "zed_wrapper.hpp"

namespace mrover {
    template<typename TEnum>
    [[nodiscard]] auto stringToZedEnum(std::string_view string) -> TEnum {
        using int_t = std::underlying_type_t<TEnum>;
        for (int_t i = 0; i < static_cast<int_t>(TEnum::LAST); ++i) {
            if (sl::String{string.data()} == sl::toString(static_cast<TEnum>(i))) {
                return static_cast<TEnum>(i);
            }
        }
        throw std::invalid_argument("Invalid enum string");
    }


    ZedWrapper::ZedWrapper() : Node(NODE_NAME, rclcpp::NodeOptions().use_intra_process_comms(true)), mLoopProfilerGrab{get_logger()}, mLoopProfilerUpdate{get_logger()} {
        try {
            RCLCPP_INFO(this->get_logger(), "Created Zed Wrapper Node, %s", NODE_NAME);

            // Publishers
            mRightImgPub = create_publisher<sensor_msgs::msg::Image>("zed/right/image", 1);
            mLeftImgPub = create_publisher<sensor_msgs::msg::Image>("zed/left/image", 1);
            mImuPub = create_publisher<sensor_msgs::msg::Imu>("imu/data", 1);
            mMagPub = create_publisher<sensor_msgs::msg::MagneticField>("zed_imu/mag", 1);
            mPcPub = create_publisher<sensor_msgs::msg::PointCloud2>("zed/left/points", 1);
            mRightCamInfoPub = create_publisher<sensor_msgs::msg::CameraInfo>("zed/right/camera_info", 1);
            mLeftCamInfoPub = create_publisher<sensor_msgs::msg::CameraInfo>("zed/left/camera_info", 1);

            // Declare and set Params
            int imageWidth{};
            int imageHeight{};

            std::string svoFile{};

            std::string grabResolutionString, depthModeString;

            std::vector<ParameterWrapper> params{
                    {"depth_confidence", mDepthConfidence, 70},
                    {"serial_number", mSerialNumber, -1},
                    {"grab_target_fps", mGrabTargetFps, 60},
                    {"texture_confidence", mTextureConfidence, 100},
                    {"image_width", imageWidth, 1280},
                    {"image_height", imageHeight, 720},
                    {"svo_file", svoFile, ""},
                    {"use_depth_stabilization", mUseDepthStabilization, false},
                    {"grab_resolution", grabResolutionString, std::string{sl::toString(sl::RESOLUTION::HD720)}},
                    {"depth_mode", depthModeString, std::string{sl::toString(sl::DEPTH_MODE::ULTRA)}},
                    {"depth_maximum_distance", mDepthMaximumDistance, 12.0},
                    {"use_builtin_visual_odom", mUseBuiltinPosTracking, false},
                    {"use_pose_smoothing", mUsePoseSmoothing, true},
                    {"use_area_memory", mUseAreaMemory, true}};

            ParameterWrapper::declareParameters(this, params);

            mSvoPath = svoFile.c_str();

            if (imageWidth < 0 || imageHeight < 0) {
                throw std::invalid_argument("Invalid image dimensions");
            }
            if (mGrabTargetFps < 0) {
                throw std::invalid_argument("Invalid grab target framerate");
            }

            mImageResolution = sl::Resolution(imageWidth, imageHeight);
            mPointResolution = sl::Resolution(imageWidth, imageHeight);
            mNormalsResolution = sl::Resolution(imageWidth, imageHeight);

            RCLCPP_INFO_STREAM(get_logger(), std::format("Resolution: {} image: {}x{} points: {}x{}",
                                                         grabResolutionString, mImageResolution.width, mImageResolution.height, mPointResolution.width, mPointResolution.height)
                                                     .c_str());
            RCLCPP_INFO_STREAM(get_logger(), std::format("Use builtin visual odometry: {}", mUseBuiltinPosTracking ? "true" : "false"));

            sl::InitParameters initParameters;

            if (mSvoPath) {
                initParameters.input.setFromSVOFile(mSvoPath);
            } else {
                if (mSerialNumber == -1) {
                    initParameters.input.setFromCameraID(-1, sl::BUS_TYPE::USB);
                } else {
                    initParameters.input.setFromSerialNumber(mSerialNumber, sl::BUS_TYPE::USB);
                }
            }

            RCLCPP_INFO_STREAM(get_logger(), grabResolutionString);

            initParameters.depth_stabilization = mUseDepthStabilization;
            initParameters.camera_resolution = stringToZedEnum<sl::RESOLUTION>(grabResolutionString);
            initParameters.depth_mode = stringToZedEnum<sl::DEPTH_MODE>(depthModeString);
            initParameters.coordinate_units = sl::UNIT::METER;
            initParameters.sdk_verbose = true; // Log useful information
            initParameters.camera_fps = mGrabTargetFps;
            initParameters.coordinate_system = sl::COORDINATE_SYSTEM::RIGHT_HANDED_Z_UP_X_FWD; // Match ROS
            initParameters.depth_maximum_distance = static_cast<float>(mDepthMaximumDistance);
            RCLCPP_INFO_STREAM(get_logger(), depthModeString);

            mDepthEnabled = initParameters.depth_mode != sl::DEPTH_MODE::NONE;

            if (mZed.open(initParameters) != sl::ERROR_CODE::SUCCESS) {
                throw std::runtime_error("ZED failed to open");
            }

            mZedInfo = mZed.getCameraInformation();

            if (mUseBuiltinPosTracking) {
                sl::PositionalTrackingParameters positionalTrackingParameters;
                positionalTrackingParameters.enable_pose_smoothing = mUsePoseSmoothing;
                positionalTrackingParameters.enable_area_memory = mUseAreaMemory;
                mZed.enablePositionalTracking(positionalTrackingParameters);
            }

            cudaDeviceProp prop{};
            cudaGetDeviceProperties(&prop, 0);
            RCLCPP_INFO_STREAM(get_logger(), std::format("MP count: {}, Max threads/MP: {}, Max blocks/MP: {}, max threads/block: {}",
                                                         prop.multiProcessorCount, prop.maxThreadsPerMultiProcessor, prop.maxBlocksPerMultiProcessor, prop.maxThreadsPerBlock));

            mGrabThread = std::thread(&ZedWrapper::grabThread, this);
            mPointCloudThread = std::thread(&ZedWrapper::pointCloudUpdateThread, this);
        } catch (std::exception const& e) {
            RCLCPP_FATAL_STREAM(get_logger(), std::format("Exception while starting: {}", e.what()));
            rclcpp::shutdown();
        }
    }


    auto ZedWrapper::grabThread() -> void {
        try {
            RCLCPP_INFO(this->get_logger(), "Starting grab thread");
            while (rclcpp::ok()) {
                //mLoopProfilerGrab.beginLoop();

                sl::RuntimeParameters runtimeParameters;
                runtimeParameters.confidence_threshold = mDepthConfidence;
                runtimeParameters.texture_confidence_threshold = mTextureConfidence;


                if (sl::ERROR_CODE error = mZed.grab(runtimeParameters); error != sl::ERROR_CODE::SUCCESS)
                    throw std::runtime_error(std::format("ZED failed to grab {}", sl::toString(error).c_str()));

                mLoopProfilerGrab.measureEvent("zed_grab");

                // Retrieval has to happen on the same thread as grab so that the image and point cloud are synced
                if (mZed.retrieveImage(mGrabMeasures.rightImage, sl::VIEW::RIGHT, sl::MEM::GPU, mImageResolution) != sl::ERROR_CODE::SUCCESS)
                    throw std::runtime_error("ZED failed to retrieve right image");

                mLoopProfilerGrab.measureEvent("zed_retrieve_right_image");

                // Only left set is used for processing
                if (mDepthEnabled) {
                    if (mZed.retrieveImage(mGrabMeasures.leftImage, sl::VIEW::LEFT, sl::MEM::GPU, mImageResolution) != sl::ERROR_CODE::SUCCESS)
                        throw std::runtime_error("ZED failed to retrieve left image");
                    if (mZed.retrieveMeasure(mGrabMeasures.leftPoints, sl::MEASURE::XYZ, sl::MEM::GPU, mPointResolution) != sl::ERROR_CODE::SUCCESS)
                        throw std::runtime_error("ZED failed to retrieve point cloud");
                }

                mLoopProfilerGrab.measureEvent("zed_retrieve_left_image");

                if (mZed.retrieveMeasure(mGrabMeasures.leftNormals, sl::MEASURE::NORMALS, sl::MEM::GPU, mNormalsResolution) != sl::ERROR_CODE::SUCCESS)
                    throw std::runtime_error("ZED failed to retrieve point cloud normals");

                assert(mGrabMeasures.leftImage.timestamp == mGrabMeasures.leftPoints.timestamp);

                mGrabMeasures.time = mSvoPath.c_str() ? now() : slTime2Ros(mZed.getTimestamp(sl::TIME_REFERENCE::IMAGE));

                // If the processing thread is busy skip
                // We want this thread to run as fast as possible for grab and positional tracking
                if (mSwapMutex.try_lock()) {
                    std::swap(mGrabMeasures, mPcMeasures);
                    mIsSwapReady = true;
                    mSwapMutex.unlock();
                    mSwapCv.notify_one();
                }

                // Positional tracking module publishing
                if (mUseBuiltinPosTracking) {
                    sl::Pose pose;
                    sl::POSITIONAL_TRACKING_STATE status = mZed.getPosition(pose);
                    if (status == sl::POSITIONAL_TRACKING_STATE::OK) {
                        sl::Translation const& translation = pose.getTranslation();
                        sl::Orientation const& orientation = pose.getOrientation();
                        try {
                            SE3d leftCameraInOdom{{translation.x, translation.y, translation.z},
                                                  Eigen::Quaterniond{orientation.w, orientation.x, orientation.y, orientation.z}.normalized()};
                            SE3d baseLinkToLeftCamera = SE3Conversions::fromTfTree(*mTfBuffer, "base_link", "zed_left_camera_frame");
                            SE3d baseLinkInOdom = leftCameraInOdom * baseLinkToLeftCamera;
                            SE3Conversions::pushToTfTree(*mTfBroadcaster, "base_link", "odom", baseLinkInOdom, now());
                        } catch (tf2::TransformException& e) {
                            RCLCPP_INFO_STREAM(get_logger(), "Failed to get transform: " << e.what());
                        }
                    } else {
                        RCLCPP_INFO_STREAM(get_logger(), "Positional tracking failed: " << status);
                    }
                }

                mLoopProfilerGrab.measureEvent("pose_tracking");

                if (mZedInfo.camera_model != sl::MODEL::ZED) {
                    sl::SensorsData sensorData;
                    mZed.getSensorsData(sensorData, sl::TIME_REFERENCE::CURRENT);

                    sensor_msgs::msg::Imu imuMsg;
                    fillImuMessage(this, sensorData.imu, imuMsg);
                    imuMsg.header.frame_id = "zed_mag_frame";
                    imuMsg.header.stamp = now();
                    mImuPub->publish(imuMsg);

                    if (mZedInfo.camera_model != sl::MODEL::ZED_M) {
                        sensor_msgs::msg::MagneticField magMsg;
                        fillMagMessage(sensorData.magnetometer, magMsg);
                        magMsg.header.frame_id = "zed_mag_frame";
                        magMsg.header.stamp = now();
                        mMagPub->publish(magMsg);
                    }
                }

                mLoopProfilerGrab.measureEvent("publish_imu_and_mag");
            }

            mZed.close();
            RCLCPP_INFO(get_logger(), "Grab thread finished");

        } catch (std::exception const& e) {
            RCLCPP_FATAL_STREAM(get_logger(), std::format("Exception while running grab thread: {}", e.what()));
            mZed.close();
            rclcpp::shutdown();
            std::exit(EXIT_FAILURE);
        }
    }

    auto ZedWrapper::pointCloudUpdateThread() -> void {
        try {
            RCLCPP_INFO(get_logger(), "Starting point cloud thread");

            while (rclcpp::ok()) {
                //mLoopProfilerUpdate.beginLoop();

                // TODO(quintin): May be bad to allocate every update, best case optimized by tcache
                // Needed because publish directly shares the pointer to other nodelets running in this process
                auto pointCloudMsg = std::make_unique<sensor_msgs::msg::PointCloud2>();

                // Swap critical section
                {
                    std::unique_lock lock{mSwapMutex};
                    // Waiting on the condition variable will drop the lock and reacquire it when the condition is met
                    mSwapCv.wait(lock, [this] { return mIsSwapReady; });
                    mIsSwapReady = false;
                    mLoopProfilerUpdate.measureEvent("wait_and_alloc");

                    if (mDepthEnabled) {
                        fillPointCloudMessageFromGpu(mPcMeasures.leftPoints, mPcMeasures.leftImage, mPcMeasures.leftNormals, mPointCloudGpu, pointCloudMsg);
                        pointCloudMsg->header.stamp = mPcMeasures.time;
                        pointCloudMsg->header.frame_id = "zed_left_camera_frame";
                        mLoopProfilerUpdate.measureEvent("fill_pc");
                    }


                    auto leftImgMsg = std::make_unique<sensor_msgs::msg::Image>();
                    fillImageMessage(mPcMeasures.leftImage, leftImgMsg);
                    leftImgMsg->header.frame_id = "zed_left_camera_optical_frame";
                    leftImgMsg->header.stamp = mPcMeasures.time;
                    mLeftImgPub->publish(std::move(leftImgMsg));
                    mLoopProfilerUpdate.measureEvent("pub_left");

                    auto rightImgMsg = std::make_unique<sensor_msgs::msg::Image>();
                    fillImageMessage(mPcMeasures.rightImage, rightImgMsg);
                    rightImgMsg->header.frame_id = "zed_right_camera_optical_frame";
                    rightImgMsg->header.stamp = mPcMeasures.time;
                    mRightImgPub->publish(std::move(rightImgMsg));
                    mLoopProfilerUpdate.measureEvent("pub_right");
                }

                if (mDepthEnabled) {
                    mPcPub->publish(std::move(pointCloudMsg));
                }
                mLoopProfilerUpdate.measureEvent("pub_pc");

                sl::CalibrationParameters calibration = mZedInfo.camera_configuration.calibration_parameters;
                auto leftCamInfoMsg = sensor_msgs::msg::CameraInfo();
                auto rightCamInfoMsg = sensor_msgs::msg::CameraInfo();
                fillCameraInfoMessages(calibration, mImageResolution, leftCamInfoMsg, rightCamInfoMsg);
                leftCamInfoMsg.header.frame_id = "zed_left_camera_optical_frame";
                leftCamInfoMsg.header.stamp = mPcMeasures.time;
                rightCamInfoMsg.header.frame_id = "zed_right_camera_optical_frame";
                rightCamInfoMsg.header.stamp = mPcMeasures.time;
                mLeftCamInfoPub->publish(leftCamInfoMsg);
                mRightCamInfoPub->publish(rightCamInfoMsg);
                mLoopProfilerUpdate.measureEvent("pub_camera_info");
            }

            RCLCPP_INFO(get_logger(), "Tag thread finished");
        } catch (std::exception const& e) {
            RCLCPP_FATAL_STREAM(get_logger(), std::format("Exception while running point cloud thread: {}", e.what()));
            rclcpp::shutdown();
            std::exit(EXIT_FAILURE);
        }
    }

    ZedWrapper::~ZedWrapper() {
        RCLCPP_INFO(get_logger(), "ZED node shutting down");
        mPointCloudThread.join();
        mGrabThread.join();
    }

    ZedWrapper::Measures::Measures(Measures&& other) noexcept {
        *this = std::move(other);
    }

    auto ZedWrapper::Measures::operator=(Measures&& other) noexcept -> Measures& {
        sl::Mat::swap(other.leftImage, leftImage);
        sl::Mat::swap(other.rightImage, rightImage);
        sl::Mat::swap(other.leftPoints, leftPoints);
        sl::Mat::swap(other.leftNormals, leftNormals);
        std::swap(time, other.time);
        return *this;
    }
}; // namespace mrover

auto main(int argc, char* argv[]) -> int {
    rclcpp::init(argc, argv);
    rclcpp::spin(std::make_shared<mrover::ZedWrapper>());
    rclcpp::shutdown();
    return 0;
}
