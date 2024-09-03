#pragma once

#include "pch.hpp"

namespace mrover {
	class ZedWrapper : public rclcpp::Node{
		private:
			static constexpr char const* NODE_NAME = "zed_wrapper";

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

			sl::Resolution mImageResolution, mPointResolution, mNormalsResolution;

			auto grabThread() -> void;

			auto pointCloudUpdateThread() -> void;

		public:
			ZedWrapper();
	};
};
