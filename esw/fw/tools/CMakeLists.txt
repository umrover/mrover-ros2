cmake_minimum_required(VERSION 3.22)

project(tools)

find_package(Python3 REQUIRED COMPONENTS Interpreter)

set(TOOLS_DIR ${CMAKE_CURRENT_SOURCE_DIR} CACHE INTERNAL "tools directory")
set(VENV_DIR ${TOOLS_DIR}/venv)

if (WIN32)
    set(VENV_PYTHON ${VENV_DIR}/Scripts/python.exe CACHE INTERNAL "python installation")
    set(VENV_PIP ${VENV_DIR}/Scripts/pip.exe)
else()
    set(VENV_PYTHON ${VENV_DIR}/bin/python CACHE INTERNAL "python installation")
    set(VENV_PIP ${VENV_DIR}/bin/pip)
endif()

add_custom_command(
    OUTPUT ${VENV_DIR}/pyvenv.cfg
    COMMAND ${Python3_EXECUTABLE} -m venv ${VENV_DIR}
    COMMENT "creating venv"
    VERBATIM
)

add_custom_target(python_venv DEPENDS ${VENV_DIR}/pyvenv.cfg)

set(PYPROJECT_TOML ${TOOLS_DIR}/pyproject.toml)

add_custom_command(
    OUTPUT ${VENV_DIR}/package.timestamp
    DEPENDS python_venv ${PYPROJECT_TOML}
    WORKING_DIRECTORY ${TOOLS_DIR}
    COMMAND ${VENV_PIP} install --upgrade pip setuptools wheel
    COMMAND ${VENV_PIP} install --editable ".[dev]"
    COMMAND ${CMAKE_COMMAND} -E touch ${VENV_DIR}/package.timestamp
    COMMENT "installing esw python tools"
    VERBATIM
)

add_custom_target(python_package DEPENDS ${VENV_DIR}/package.timestamp)

add_custom_target(python_env_ready DEPENDS python_package)

function(run_python_tool TARGET_NAME SCRIPT)
    add_custom_target(
        ${TARGET_NAME}
        COMMAND ${VENV_PYTHON} ${SCRIPT} ${ARGN}
        DEPENDS python_env_ready
        WORKING_DIRECTORY ${TOOLS_DIR}
        COMMENT "running tool ${SCRIPT}"
        VERBATIM
    )
endfunction()

add_custom_target(clean_venv
    COMMAND ${CMAKE_COMMAND} -E rm -rf ${VENV_DIR}
    COMMENT "removing venv"
)
