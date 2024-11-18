#include "State1.hpp"

namespace mrover {
	State1::State1() : numLoops{0}{}

	auto State1::onLoop() -> State*{
		if(numLoops >= 5){
			return new State2();
		}
		auto logger = rclcpp::get_logger("State1");
		RCLCPP_INFO_STREAM(logger, "In State1 " << numLoops << "\n");
		++numLoops;
		return this;
	}

	auto State1::getName() -> std::string{
		return "State1";
	}
}
