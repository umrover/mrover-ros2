#include "harness.hpp"
#include <mrover_test.hpp>

auto test_main(int argc, char** argv) -> int {
    testing::InitGoogleTest(&argc, argv);
    return RUN_ALL_TESTS();
}
