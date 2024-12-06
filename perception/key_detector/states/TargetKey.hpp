#pragma once

#include "../pch.hpp"

namespace mrover{
	class TargetKey : public State {
	private:
        
		std::int64_t numLoops;
	public:
		explicit TargetKey();

		auto onLoop() -> State* override;
	};
}
