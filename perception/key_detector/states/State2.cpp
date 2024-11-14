#include "State2.hpp"

namespace mrover {
	State2::State2() : numLoops{0}{}

	 auto State2::onLoop() -> State*{
		if(numLoops >= 5){
			return new State1();
		}
		auto logger = rclcpp::get_logger("State2");
		RCLCPP_INFO_STREAM(logger, "In State2 " << numLoops << "\n");
		++numLoops;
		return this;
	}
}
