#pragma once

#include "../pch.hpp"
#include <rclcpp/rate.hpp>

namespace mrover{
	class Wait : public State {
	private:
        // milliseconds ?
		rclcpp::Rate sleepRate;
	public:
		explicit Wait();

		auto onLoop() -> State* override;
	};
}
