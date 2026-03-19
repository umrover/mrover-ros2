cmake_minimum_required(VERSION 3.22)

add_subdirectory(../../tools tools)

get_filename_component(PROJECT_ROOT "${CMAKE_CURRENT_SOURCE_DIR}/../.." ABSOLUTE)
set(DBC_SOURCE_DIR "${PROJECT_ROOT}/dbc")
set(DBC_GEN_DIR "${CMAKE_CURRENT_SOURCE_DIR}/build")

file(GLOB DBC_FILES CONFIGURE_DEPENDS "${DBC_SOURCE_DIR}/*.dbc")

set(DBC_SCRIPT "${TOOLS_DIR}/scripts/can_header_gen.py")
if(NOT EXISTS ${DBC_SCRIPT})
    message(FATAL_ERROR "could not find script ${DBC_SCRIPT}")
endif()

file(MAKE_DIRECTORY ${DBC_GEN_DIR})

set(GENERATED_HEADERS "")

foreach(DBC_FILE ${DBC_FILES})
    get_filename_component(FILE_NAME ${DBC_FILE} NAME_WE)
    set(HEADER_FILE "${DBC_GEN_DIR}/${FILE_NAME}.hpp")
    # NOTE(eric): when you want to change the headers for testing, comment out the command below
    # to ensure the header is not overwritten on build
    add_custom_command(
        OUTPUT ${HEADER_FILE}
        COMMAND ${VENV_PYTHON} ${DBC_SCRIPT} --dest ${DBC_GEN_DIR} --ctx ${CMAKE_CURRENT_SOURCE_DIR} ${DBC_FILE}
        DEPENDS ${DBC_FILE} ${DBC_SCRIPT} python_env_ready
        WORKING_DIRECTORY ${TOOLS_DIR}
        COMMENT "generating header for ${FILE_NAME}.dbc"
        VERBATIM
    )
    list(APPEND GENERATED_HEADERS ${HEADER_FILE})
endforeach()

add_library(dbc INTERFACE ${GENERATED_HEADERS})
target_include_directories(dbc INTERFACE ${DBC_GEN_DIR})
if(DEFINED STM32)
    target_link_libraries(dbc INTERFACE stm32)
endif()
