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


	ZedWrapper::ZedWrapper() : Node(NODE_NAME) {
		try{
			RCLCPP_INFO(this->get_logger(), "Created Zed Wrapper Node, %s", NODE_NAME);

			mRightImgPub = create_publisher<sensor_msgs::msg::Image>("right/image", 1);

			// Declare Params
			auto paramSub = std::make_shared<rclcpp::ParameterEventHandler>(this);

			int imageWidth{};
			int imageHeight{};

			std::string grabResolutionString{}, depthModeString{};

			std::vector<ParameterWrapper> params{
				{"depth_confidence", mDepthConfidence},
				{"serial_number", mSerialNumber},
				{"grab_target_fps", mGrabTargetFps},
				{"texture_confidence", mTextureConfidence},
				{"image_width", imageWidth},
				{"image_height", imageHeight},
				{"svo_file", mSvoPath},
				{"use_depth_stabilization", mUseDepthStabilization},
				{"grab_resolution", grabResolutionString},
				{"depth_mode", depthModeString},
				{"depth_maximum_distance", mDepthMaximumDistance},
				{"use_builtin_visual_odom", mUseBuiltinPosTracking},
				{"use_pose_smoothing", mUsePoseSmoothing},
				{"use_area_memory", mUseAreaMemory}
			};

			RCLCPP_INFO(get_logger(), "Camera Resolution: %s", sl::toString(sl::DEPTH_MODE::PERFORMANCE).c_str());

			ParameterWrapper::declareParameters(this, paramSub, params);

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
											grabResolutionString, mImageResolution.width, mImageResolution.height, mPointResolution.width, mPointResolution.height).c_str());
			RCLCPP_INFO_STREAM(get_logger(), std::format("Use builtin visual odometry: {}", mUseBuiltinPosTracking ? "true" : "false"));

			sl::InitParameters initParameters;

			if (mSvoPath.c_str()) {
				initParameters.input.setFromSVOFile(mSvoPath.c_str());
			} else {
				if (mSerialNumber == -1) {
					initParameters.input.setFromCameraID(-1, sl::BUS_TYPE::USB);
				} else {
					initParameters.input.setFromSerialNumber(mSerialNumber, sl::BUS_TYPE::USB);
				}
			}

			initParameters.depth_stabilization = mUseDepthStabilization;
			initParameters.camera_resolution = stringToZedEnum<sl::RESOLUTION>(grabResolutionString);
			initParameters.depth_mode = stringToZedEnum<sl::DEPTH_MODE>(depthModeString);
			initParameters.coordinate_units = sl::UNIT::METER;
			initParameters.sdk_verbose = true; // Log useful information
			initParameters.camera_fps = mGrabTargetFps;
			initParameters.coordinate_system = sl::COORDINATE_SYSTEM::RIGHT_HANDED_Z_UP_X_FWD; // Match ROS
			initParameters.depth_maximum_distance = static_cast<float>(mDepthMaximumDistance);

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
		}catch (std::exception const& e) {
            RCLCPP_FATAL_STREAM(get_logger(), std::format("Exception while starting: {}", e.what()));
            rclcpp::shutdown();
        }
	}


	auto ZedWrapper::grabThread() -> void{
		RCLCPP_INFO(this->get_logger(), "Starting grab thread");
		while(rclcpp::ok()){
			sl::RuntimeParameters runtimeParameters;
			runtimeParameters.confidence_threshold = mDepthConfidence;
			runtimeParameters.texture_confidence_threshold = mTextureConfidence;

			if (sl::ERROR_CODE error = mZed.grab(runtimeParameters); error != sl::ERROR_CODE::SUCCESS)
                    throw std::runtime_error(std::format("ZED failed to grab {}", sl::toString(error).c_str()));

			// Retrieval has to happen on the same thread as grab so that the image and point cloud are synced
			if (mZed.retrieveImage(mGrabMeasures.rightImage, sl::VIEW::RIGHT, sl::MEM::GPU, mImageResolution) != sl::ERROR_CODE::SUCCESS)
				throw std::runtime_error("ZED failed to retrieve right image");

			// Only left set is used for processing
			if (mDepthEnabled) {
				if (mZed.retrieveImage(mGrabMeasures.leftImage, sl::VIEW::LEFT, sl::MEM::GPU, mImageResolution) != sl::ERROR_CODE::SUCCESS)
					throw std::runtime_error("ZED failed to retrieve left image");
				if (mZed.retrieveMeasure(mGrabMeasures.leftPoints, sl::MEASURE::XYZ, sl::MEM::GPU, mPointResolution) != sl::ERROR_CODE::SUCCESS)
					throw std::runtime_error("ZED failed to retrieve point cloud");
			}

			// if (mZed.retrieveMeasure(mGrabMeasures.leftNormals, sl::MEASURE::NORMALS, sl::MEM::GPU, mNormalsResolution) != sl::ERROR_CODE::SUCCESS)
			//     throw std::runtime_error("ZED failed to retrieve point cloud normals");
			
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
		}
	}

	auto pointCloudUpdateThread() -> void{

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
};

auto main(int argc, char * argv[]) -> int
{
  rclcpp::init(argc, argv);
  rclcpp::spin(std::make_shared<mrover::ZedWrapper>());
  rclcpp::shutdown();
  return 0;
}
