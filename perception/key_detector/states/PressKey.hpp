#pragma once

#include "../pch.hpp"
namespace mrover{
	class PressKey : public State {
	public:

		using KeyAction = mrover::action::KeyAction;
		using GoalHandleKeyAction = rclcpp_action::ServerGoalHandle<KeyAction>;

		explicit PressKey(const std::shared_ptr<FSMData> fsm_data);

		auto onLoop() -> State* override;
	private:
        
		const std::shared_ptr<FSMData> fsm_data;
	};
}
