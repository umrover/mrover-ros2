#pragma once

#include "../pch.hpp"
namespace mrover{
	class Off : public State {
	public:
		explicit Off(const std::shared_ptr<FSMCtx> fsm_ctx);

		auto onLoop() -> State* override;
	private:
		const std::shared_ptr<FSMCtx> fsm_ctx;
	};
}
