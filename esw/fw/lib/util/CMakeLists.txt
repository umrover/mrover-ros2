cmake_minimum_required(VERSION 3.22)

file(GLOB UTIL_SOURCES
    "${CMAKE_CURRENT_SOURCE_DIR}/Src/*.c"
    "${CMAKE_CURRENT_SOURCE_DIR}/Src/*.cpp"
)

if(UTIL_SOURCES)
    add_library(util STATIC ${UTIL_SOURCES})
    set(util_scope PUBLIC)
else()
    add_library(util INTERFACE)
    set(util_scope INTERFACE)
endif()

target_include_directories(util ${util_scope}
    "${CMAKE_CURRENT_SOURCE_DIR}/Inc"
)

target_link_libraries(util ${util_scope}
    stm32
)
