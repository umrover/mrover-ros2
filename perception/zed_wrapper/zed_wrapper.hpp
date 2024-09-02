#pragma once

#include "pch.hpp"

namespace mrover {
	class ZedWrapper : public rclcpp::Node{
		private:
			static constexpr char const* NODE_NAME = "zed_wrapper";

			auto grabThread() -> void;

			auto pointCloudUpdateThread() -> void;

		public:
			ZedWrapper();
	};
};
