cmake_minimum_required(VERSION 3.22)

file(GLOB UNITS_SOURCES
    "${CMAKE_CURRENT_SOURCE_DIR}/Src/*.c"
    "${CMAKE_CURRENT_SOURCE_DIR}/Src/*.cpp"
)

if(STM32_SOURCES)
    add_library(units STATIC ${UNITS_SOURCES})
else()
    add_library(units INTERFACE)
endif()

target_include_directories(units INTERFACE
    "${CMAKE_CURRENT_SOURCE_DIR}/Inc"
)
