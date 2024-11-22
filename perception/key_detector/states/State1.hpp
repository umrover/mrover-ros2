#pragma once

#include "../pch.hpp"

namespace mrover{
	class State1 : public State {
	private:
		std::int64_t numLoops;
	public:
		explicit State1(int _numLoops);

		auto onLoop() -> State* override;
	};
}
