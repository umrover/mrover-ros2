#!/usr/bin/env bash

# See: https://vaneyckt.io/posts/safer_bash_scripts_with_set_euxo_pipefail/
set -Eeuo pipefail

shopt -s nullglob globstar
GLOBIGNORE="./venv/**"
GLOBIGNORE="$GLOBIGNORE:./esw/fw/**"

readonly RED='\033[0;31m'
readonly NC='\033[0m'
readonly YELLOW_BOLD='\033[1;33m'

BLACK_ARGS=(
  "--line-length=120"
  "--color"
)
CLANG_FORMAT_ARGS=(
  "-style=file"
)

# Just do a dry run if the "fix" argument is not passed
if [ $# -eq 0 ] || [ "$1" != "--fix" ]; then
  BLACK_ARGS+=("--diff") # Show difference
  BLACK_ARGS+=("--check") # Exit with non-zero code if changes are required (for CI)
  CLANG_FORMAT_ARGS+=("--dry-run" "--Werror") # Don't modify, exit with non-zero code if changes are required
fi

function print_update_error() {
  echo -e "${RED}[Error] Please update with ./ansible.sh build.yml${NC}"
  exit 1
}

function find_executable() {
  local -r executable="$1"
  local -r version="$2"
  local -r path=$(which "${executable}")
  if [ ! -x "${path}" ]; then
    echo -e "${RED}[Error] Could not find ${executable}${NC}"
    print_update_error
  fi
  if ! "${path}" --version | grep -q "${version}"; then
    echo -e "${RED}[Error] Wrong ${executable} version${NC}"
    print_update_error
  fi
  echo "${path}"
}

## Check that all tools are installed

readonly CLANG_FORMAT_PATH=$(find_executable clang-format-18 18.1)
readonly BLACK_PATH=$(find_executable black 24.8.0)
readonly MYPY_PATH=$(find_executable mypy 1.11.2)

## Run checks

# Add new directories with C++ code here:
readonly CPP_FILES=(
  ./{perception,lie,esw,simulator,parameter_utils}/**/*.{cpp,hpp,h,cu,cuh}
)
echo "Style checking C++ ..."
"${CLANG_FORMAT_PATH}" "${CLANG_FORMAT_ARGS[@]}" -i "${CPP_FILES[@]}"
echo "Done"

# Add new directories with Python code here:
readonly PYTHON_LINT_DIRS=(
  ./navigation
  ./localization
  ./scripts
  ./state_machine
  ./lie
  ./superstructure
)
readonly PYTHON_STYLE_DIRS=(
  "${PYTHON_LINT_DIRS[@]}"
  ./launch
)

echo
echo "Style checking Python with black ..."
"${BLACK_PATH}" "${BLACK_ARGS[@]}" "${PYTHON_STYLE_DIRS[@]}"

echo
echo "Linting Python with mypy ..."
"${MYPY_PATH}" --config-file=mypy.ini --check "${PYTHON_LINT_DIRS[@]}"

if [ -d "./teleoperation/basestation_gui/frontend" ]; then
  echo
  echo "Type checking TypeScript with vue-tsc ..."
  (cd ./teleoperation/basestation_gui/frontend && bun run type-check)
  echo "Done"

  echo
  if [ $# -eq 0 ] || [ "$1" != "--fix" ]; then
    echo "Linting TypeScript/Vue with eslint (dry run) ..."
    (cd ./teleoperation/basestation_gui/frontend && bun run check)
  else
    echo "Linting and fixing TypeScript/Vue with eslint ..."
    (cd ./teleoperation/basestation_gui/frontend && bun run lint)
  fi
  echo "Done"
fi

if command -v shellcheck &> /dev/null; then
  echo
  echo "Linting bash scripts with shellcheck ..."
  readonly SHELL_FILES=(
    ./ansible/**/*.sh
    ./scripts/**/*.sh
    ./starter_project/**/*.sh
    ./teleoperation/**/*.sh
    ./*.sh
  )
  # SC2155 is separate declaration and command.
  shellcheck --exclude=SC2155 "${SHELL_FILES[@]}"
  echo "Done"
fi

if [ $# -eq 0 ] || [ "$1" != "--fix" ]; then
  echo
  echo -e "====================================================================="
  echo -e "${YELLOW_BOLD}Please run ./style.sh --fix if you want to actually update the files!${NC}"
  echo -e "====================================================================="
fi
