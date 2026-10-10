#include <gtest/gtest.h>

/*
 * This file provides a different entrypoint for C++ unit tests so that nodes
 * can still use main() without problems.
 */

extern "C" auto __wrap_main(int argc, char** argv) -> int {
    testing::InitGoogleTest(&argc, argv);
    return RUN_ALL_TESTS();
}
