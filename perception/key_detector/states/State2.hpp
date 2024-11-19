#pragma once

#include "../pch.hpp"

namespace mrover {
	class State2 : public State {
	private:
		std::int64_t numLoops;
	public:
		explicit State2();

		auto onLoop() -> State* override;
	};
}
