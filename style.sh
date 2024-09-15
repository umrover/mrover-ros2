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
  CLANG_FORMAT_ARGS+=("--dry-run")
fi

function print_update_error() {
  echo -e "${RED}[Error] Please update with ./ansible.sh build.yml${NC}"
  exit 1
}

function find_executable() {
  local readonly executable="$1"
  local readonly version="$2"
  local readonly path=$(which "${executable}")
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

readonly CLANG_FORMAT_PATH=$(find_executable clang-format-18 18.1.8)
readonly BLACK_PATH=$(find_executable black 24.8.0)
readonly MYPY_PATH=$(find_executable mypy 1.11.2)

## Run checks

echo "Style checking C++ ..."
find ./perception ./lie ./esw ./simulator -regex '.*\.\(cpp\|hpp\|h\)' -exec "${CLANG_FORMAT_PATH}" "${CLANG_FORMAT_ARGS[@]}" -i {} \;
echo "Done"

echo
echo "Style checking Python with black ..."
"$BLACK_PATH" "${BLACK_ARGS[@]}" . --exclude=venv

echo
echo "Style checking Python with mypy ..."
# TODO(quintin): Add other subteam folders and scripts folder
"$MYPY_PATH" --config-file=mypy.ini --check ./navigation ./scripts

if [ $# -eq 0 ] || [ "$1" != "--fix" ]; then
  echo
  echo -e "====================================================================="
  echo -e "${YELLOW_BOLD}Please run ./style.sh --fix if you want to actually update the files!${NC}"
  echo -e "====================================================================="
fi
