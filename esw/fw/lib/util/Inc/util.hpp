#pragma once

#include <concepts>


namespace mrover {

    static auto check(bool const cond, std::invocable auto handler) -> void {
        if (cond) return;

        handler();
    }

} // namespace mrover
