#!/usr/bin/env bash

set -euo pipefail

RED="\e[1;31m"
NC="\e[0m"

ESW_ROOT="$(dirname "$(dirname "$(realpath "$0")")")"
SCRIPT_NAME=$(basename "$0")

TOOLS_DIR="$ESW_ROOT/tools"
VENV_PATH="$TOOLS_DIR/venv"
BUILD_DIR="$TOOLS_DIR/build"
GENERATE_SCRIPT="$TOOLS_DIR/scripts/generate_project.py"
CMAKE_SCRIPT="$TOOLS_DIR/scripts/update_cmake_cfg.py"

usage() {
    cat <<EOF
Usage: $SCRIPT_NAME [--mcu <mcu> | --board <board>] --src <source> [--lib <library>]+

options:
  -m, --mcu <mcu>       target mcu for new project
  -b, --board <board>   target board for new project
  -s, --src <source>    path to project root
  -l, --lib <library>   cmake library dependency
  -h, --help            show this help message
EOF
    exit 1
}

MCU=""
BOARD=""
SRC=""
LIBS=()

# parse opts
while [[ $# -gt 0 ]]; do
    case "$1" in
        -m|--mcu)       MCU="$2"; shift 2 ;;
        -b|--board)     BOARD="$2"; shift 2 ;;
        -s|--src)       SRC="$(realpath "$2")"; shift 2 ;;
        -l|--lib)       LIBS+=("$2"); shift 2 ;;
        -h|--help)      usage ;;
        *)              printf "%b\n" "${RED}✗ unknown option: $1${NC}"; usage ;;
    esac
done

# select project source
if [[ -z "$SRC" ]]; then
    printf "%b\n" "${RED}Project source path must be specified${NC}"
    exit 1
fi

# assert target directory does not exist
if [[ -d "$SRC" ]]; then
    printf "%b\n" "${RED}Project source path must not exist already${NC}"
    exit 1
fi

# select target
TARGET=""
if { [ -n "$MCU" ] && [ -n "$BOARD" ]; } || { [ -z "$MCU" ] && [ -z "$BOARD" ]; }; then
    printf "%b\n" "${RED}Exactly one of MCU or BOARD must be specified${NC}"
    exit 1
fi
if [ -n "$MCU" ]; then
    TARGET="$MCU"
else
    TARGET="$BOARD"
fi

# create venv if it does not exist
if [[ ! -f "${VENV_PATH}/pyvenv.cfg" ]]; then
    mkdir -p "$BUILD_DIR"
    if [[ ! -f "${BUILD_DIR}/CMakeCache.txt" ]]; then
        cmake -S "${TOOLS_DIR}" -B "${BUILD_DIR}"
    fi
    cmake --build "${BUILD_DIR}" --target python_env_ready
    rm -rf "$BUILD_DIR"
fi

# activate venv
# shellcheck source=/dev/null
source "$VENV_PATH/bin/activate"

# run project generation script
"$VENV_PATH/bin/python" "$GENERATE_SCRIPT" --mcu "$TARGET" --src "$SRC"

# create args
PY_LIB_ARGS=()
for lib in "${LIBS[@]}"; do
    PY_LIB_ARGS+=(--lib "$lib")
done

# run cmake configuration script
"$VENV_PATH/bin/python" "$CMAKE_SCRIPT" \
    --src "$SRC" \
    --root "$ESW_ROOT" \
    --ctx "$ESW_ROOT/lib/stm32g4" \
    "${PY_LIB_ARGS[@]}"
