#include "Done.hpp"

namespace mrover {
    Done::Done(std::shared_ptr<FSMContext> const& fsmContext) : mFSMContext(fsmContext) {}

    auto Done::onLoop() -> State* {
        return this;
    }
} // namespace mrover
