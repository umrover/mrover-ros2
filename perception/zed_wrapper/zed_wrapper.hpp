#pragma once

#include "pch.hpp"

namespace mrover {
	class ZedWrapper : public rclcpp::Node{
		private:
			static constexpr char const* NODE_NAME = "zed_wrapper";

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

			// Publishers
			rclcpp::Publisher<sensor_msgs::msg::Image>::SharedPtr mRightImgPub;

			auto grabThread() -> void;

			auto pointCloudUpdateThread() -> void;

		public:
			ZedWrapper();
	};
};
