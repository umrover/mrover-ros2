#include "State2.hpp"
namespace mrover {
	State2::State2() : numLoops{0}{}

	 auto State2::onLoop() -> State*{
		if(numLoops >= 5){
			return StateMachine::make_state<State1>(2);
		}
		auto logger = rclcpp::get_logger("State2");
		RCLCPP_INFO_STREAM(logger, "In State2 " << numLoops << "\n");
		++numLoops;
		return this;
	}
}
