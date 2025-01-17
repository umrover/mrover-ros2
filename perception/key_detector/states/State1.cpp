#include "State1.hpp"
namespace mrover {
	State1::State1(int _numLoops) : numLoops{_numLoops}{}

	auto State1::onLoop() -> State*{
		if(numLoops >= 5){
			return StateMachine::make_state<State2>();
		}
		auto logger = rclcpp::get_logger("State1");
		RCLCPP_INFO_STREAM(logger, "In State1 " << numLoops << "\n");
		++numLoops;
		return this;
	}
}
