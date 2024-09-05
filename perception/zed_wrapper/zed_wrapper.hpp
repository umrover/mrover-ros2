#pragma once

#include "pch.hpp"

namespace mrover {
	class ZedWrapper : public rclcpp::Node{
	private:
		static constexpr char const* NODE_NAME = "zed_wrapper";

		std::unique_ptr<tf2_ros::Buffer> mTfBuffer = std::make_unique<tf2_ros::Buffer>(get_clock());
		std::shared_ptr<tf2_ros::TransformBroadcaster> mTfBroadcaster = std::make_shared<tf2_ros::TransformBroadcaster>(this);
		std::shared_ptr<tf2_ros::TransformListener> mTfListener = std::make_shared<tf2_ros::TransformListener>(*mTfBuffer);

		struct Measures {
			rclcpp::Time time;
			sl::Mat leftImage;
			sl::Mat rightImage;
			sl::Mat leftPoints;
			sl::Mat leftNormals;

			Measures() = default;

			Measures(Measures&) = delete;
			auto operator=(Measures&) -> Measures& = delete;

			Measures(Measures&&) noexcept;
			auto operator=(Measures&&) noexcept -> Measures&;
		};

		// Params
		int mSerialNumber{};
		int mGrabTargetFps{};
		int mDepthConfidence{};
		int mTextureConfidence{};

		bool mUseDepthStabilization{};
		bool mDepthEnabled{};
		bool mUseBuiltinPosTracking{};
		bool mUsePoseSmoothing{};
		bool mUseAreaMemory{};

		double mDepthMaximumDistance{};


		// ZED
		sl::Camera mZed;
		sl::CameraInformation mZedInfo;

		Measures mGrabMeasures, mPcMeasures;

		sl::Resolution mImageResolution, mPointResolution, mNormalsResolution;

		std::string mSvoPath;

		// Publishers
		rclcpp::Publisher<sensor_msgs::msg::Image>::SharedPtr mRightImgPub;

		// Thread
		
		std::thread mPointCloudThread, mGrabThread;
		std::mutex mSwapMutex;
		std::condition_variable mSwapCv;
		bool mIsSwapReady = false;

		auto grabThread() -> void;

		auto pointCloudUpdateThread() -> void;

	public:
		ZedWrapper();

		~ZedWrapper() override;
	};

	auto slTime2Ros(sl::Timestamp t) -> rclcpp::Time;
};
