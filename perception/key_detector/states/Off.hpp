#pragma once
#include "../pch.hpp"
#include "../FSMContext.hpp"

namespace mrover {
    class Off : public State {
    public:
        explicit Off(std::shared_ptr<FSMContext> const& fsmContext);

        auto onLoop() -> State* override;

    private:
        std::shared_ptr<FSMContext> const mFSMContext;
    };
} // namespace mrover
