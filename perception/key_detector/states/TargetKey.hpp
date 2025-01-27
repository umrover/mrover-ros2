#pragma once

#include "../pch.hpp"
namespace mrover{
	class TargetKey : public State {
	public:

		using KeyAction = mrover::action::KeyAction;
		using GoalHandleKeyAction = rclcpp_action::ServerGoalHandle<KeyAction>;

		explicit TargetKey(const std::shared_ptr<FSMCtx> fsm_ctx);

		auto onLoop() -> State* override;

	private:
		rclcpp::Publisher<msg::IK>::SharedPtr mIkTargetPub;
		const std::shared_ptr<FSMCtx> fsm_ctx;
		rclcpp::Rate sleepRate;
	};
}
