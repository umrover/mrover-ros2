#pragma once

#include "../pch.hpp"
#include <rclcpp/rate.hpp>

namespace mrover{
	class Wait : public State {
	public:

		using KeyAction = mrover::action::KeyAction;
		using GoalHandleKeyAction = rclcpp_action::ServerGoalHandle<KeyAction>;

		explicit Wait(const std::shared_ptr<FSMCtx> fsm_ctx);

		auto onLoop() -> State* override;
	private:
        // milliseconds ?
		rclcpp::Rate sleepRate;
		const std::shared_ptr<FSMCtx> fsm_ctx;
	};
}
