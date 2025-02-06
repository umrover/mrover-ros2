#pragma once

#include "../pch.hpp"
#include "mrover/msg/detail/position__struct.hpp"
namespace mrover{
	class TargetKey : public State {
	public:

		using KeyAction = mrover::action::KeyAction;
		using GoalHandleKeyAction = rclcpp_action::ServerGoalHandle<KeyAction>;

		explicit TargetKey(const std::shared_ptr<FSMCtx> fsm_ctx);

		auto onLoop() -> State* override;

	private:
		rclcpp::Publisher<msg::IK>::SharedPtr mIkTargetPub;
		rclcpp::Subscription<msg::Position>::SharedPtr mPosSub;
		const std::shared_ptr<FSMCtx> fsm_ctx;
		rclcpp::Rate sleepRate;
	};
}
