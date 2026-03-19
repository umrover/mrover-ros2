cmake_minimum_required(VERSION 3.22)

project(dbc_runtime LANGUAGES CXX)

add_library(dbc_runtime STATIC
    src/file_parser.cpp
    src/frame_processor.cpp
    src/message.cpp
    src/signal.cpp
)
add_library(dbc_runtime::dbc_runtime ALIAS dbc_runtime)

target_compile_features(dbc_runtime PUBLIC cxx_std_23)

target_include_directories(dbc_runtime
  PUBLIC
    $<BUILD_INTERFACE:${CMAKE_CURRENT_LIST_DIR}/include>
)

