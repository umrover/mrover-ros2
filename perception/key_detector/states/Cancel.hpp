#pragma once

#include "../pch.hpp"
namespace mrover{
	class Cancel : public State {
	public:
		explicit Cancel(const std::shared_ptr<FSMCtx> fsm_ctx);

		auto onLoop() -> State* override;
	private:
		const std::shared_ptr<FSMCtx> fsm_ctx;
	};
}
