#pragma once

#include "pch.hpp"

namespace mrover {
	class ZedWrapper : public rclcpp::Node{
		private:
			static constexpr char const* NODE_NAME = "zed_wrapper";
		public:
			ZedWrapper();
	};
};
