#pragma once
#include "../pch.hpp"
#include "../FSMContext.hpp"

namespace mrover {
    class Done : public State {
    public:
        explicit Done(std::shared_ptr<FSMContext> const& fsmContext);

        auto onLoop() -> State* override;

    private:
        std::shared_ptr<FSMContext> const mFSMContext;
    };
} // namespace mrover
