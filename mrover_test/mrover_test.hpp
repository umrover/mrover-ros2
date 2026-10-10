#pragma once

/*
 * This file provides a different entrypoint for C++ unit tests so that nodes
 * can still use main() without problems.
 */

auto test_main(int, char**) -> int; 

extern "C" auto __wrap_main(int argc, char** argv) -> int {
    return test_main(argc, argv);
}
