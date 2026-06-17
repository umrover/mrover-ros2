#!/usr/bin/env bash

# See: https://vaneyckt.io/posts/safer_bash_scripts_with_set_euxo_pipefail/
set -Eeuo pipefail

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
  echo -e "${RED}[Error] Please update with ./ansible.sh build.yml${NC}" >&2
  exit 1
}

function find_executable() {
  local -r executable="$1"
  local path
  if ! path=$(command -v "${executable}" 2> /dev/null) || [ ! -x "${path}" ]; then
    echo -e "${RED}[Error] Could not find ${executable}${NC}" >&2
    print_update_error
  fi
  echo "${path}"
}

function find_first_executable() {
  local executable
  for executable in "$@"; do
    local path
    if path=$(command -v "${executable}" 2> /dev/null) && [ -x "${path}" ]; then
      echo "${path}"
      return
    fi
  done
  echo -e "${RED}[Error] Could not find any of: $*${NC}" >&2
  print_update_error
}

## Check that all tools are installed

CLANG_FORMAT_PATH=$(find_first_executable clang-format clang-format-18)
readonly CLANG_FORMAT_PATH
BLACK_PATH=$(find_executable black)
readonly BLACK_PATH
MYPY_PATH=$(find_executable mypy)
readonly MYPY_PATH

## Run checks

# Add new directories with C++ code here:
CPP_FILES=()
readonly CPP_DIRS=(
  ./perception
  ./lie
  ./esw
  ./simulator
  ./parameter_utils
  ./teleoperation
)
echo "Style checking C++ ..."
for dir in "${CPP_DIRS[@]}"; do
  if [ -d "${dir}" ]; then
    while IFS= read -r -d '' file; do
      case "${file}" in
        ./esw/fw/*) continue ;;
      esac
      CPP_FILES+=("${file}")
    done < <(find "${dir}" -type f \( -name "*.cpp" -o -name "*.hpp" -o -name "*.h" -o -name "*.cu" -o -name "*.cuh" \) -print0)
  fi
done
if [ ${#CPP_FILES[@]} -gt 0 ]; then
  "${CLANG_FORMAT_PATH}" "${CLANG_FORMAT_ARGS[@]}" -i "${CPP_FILES[@]}"
fi
echo "Done"

# Add new directories with Python code here:
readonly PYTHON_LINT_DIRS=(
  ./navigation
  ./localization
  ./scripts
  ./state_machine
  ./lie
  ./superstructure
  ./teleoperation/
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
  echo "Installing frontend dependencies ..."
  (cd ./teleoperation/basestation_gui/frontend && bun install)
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
  SHELL_FILES=()
  readonly SHELL_DIRS=(
    ./ansible
    ./scripts
    ./starter_project
    ./teleoperation
  )
  for dir in "${SHELL_DIRS[@]}"; do
    if [ -d "${dir}" ]; then
      while IFS= read -r -d '' file; do
        SHELL_FILES+=("${file}")
      done < <(find "${dir}" -type f -name "*.sh" -print0)
    fi
  done
  while IFS= read -r -d '' file; do
    SHELL_FILES+=("${file}")
  done < <(find . -maxdepth 1 -type f -name "*.sh" -print0)
  # SC2155 is separate declaration and command.
  if [ ${#SHELL_FILES[@]} -gt 0 ]; then
    shellcheck --shell=bash --exclude=SC2155 "${SHELL_FILES[@]}"
  fi
  echo "Done"
fi

if [ $# -eq 0 ] || [ "$1" != "--fix" ]; then
  echo
  echo -e "====================================================================="
  echo -e "${YELLOW_BOLD}Please run ./style.sh --fix if you want to actually update the files!${NC}"
  echo -e "====================================================================="
fi
