#pragma once

#include "../pch.hpp"
namespace mrover{
	class TargetKey : public State {
	public:

		using KeyAction = mrover::action::KeyAction;
		using GoalHandleKeyAction = rclcpp_action::ServerGoalHandle<KeyAction>;

		explicit TargetKey(const std::shared_ptr<GoalHandleKeyAction> _goal);

		auto onLoop() -> State* override;
	private:
        
		const std::shared_ptr<GoalHandleKeyAction> goal;
	};
}
