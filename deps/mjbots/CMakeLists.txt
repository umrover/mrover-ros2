cmake_minimum_required(VERSION 3.16)
project(moteus VERSION 2024.5.20 LANGUAGES CXX)

add_library(moteus INTERFACE)
target_include_directories(moteus INTERFACE ${CMAKE_CURRENT_LIST_DIR})
