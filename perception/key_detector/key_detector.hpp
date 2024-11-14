#pragma once

#include "pch.hpp"

namespace mrover {

	class KeyDetector : public rclcpp::Node{
	private:
        static constexpr char const* NODE_NAME = "key_detector";

		rclcpp::TimerBase::SharedPtr mFSMTimer;

		StateMachine mStateMachine;

		void updateFSM();

	public:
		KeyDetector();
		~KeyDetector() override;
	};

} // namespace mrover
