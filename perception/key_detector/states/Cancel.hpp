#pragma once

#include "../pch.hpp"
namespace mrover{
	class Cancel : public State {
	public:

		using KeyAction = mrover::action::KeyAction;
		using GoalHandleKeyAction = rclcpp_action::ServerGoalHandle<KeyAction>;

		explicit Cancel(const std::shared_ptr<FSMCtx> fsm_data);

		auto onLoop() -> State* override;
	private:
        
		const std::shared_ptr<FSMCtx> fsm_data;
	};
}
