cmake_minimum_required(VERSION 3.22)

project(dbc_runtime_tests LANGUAGES CXX)

set(CMAKE_CXX_STANDARD 23)
set(CMAKE_CXX_STANDARD_REQUIRED ON)

add_subdirectory(${CMAKE_CURRENT_LIST_DIR}/.. ${CMAKE_BINARY_DIR}/dbc_runtime)

add_executable(dbc_runtime_test test.cpp)

target_link_libraries(dbc_runtime_test PRIVATE dbc_runtime::dbc_runtime)

