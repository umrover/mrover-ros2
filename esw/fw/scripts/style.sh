#!/usr/bin/env bash

set -euo pipefail

BOLD="\033[1m"
RED="\033[1;31m"
GREEN="\033[1;32m"
BLUE="\033[1;34m"
YELLOW="\033[1;33m"
NC="\033[0m"

shopt -s nullglob

collect_cpp_files() {
    # skips symlinks
    find src lib \
        -type f \
        \( -name '*.cpp' -o -name '*.hpp' -o -name '*.h' -o -name '*.cu' -o -name '*.cuh' \) \
        ! -path '*/venv/*'
}
CPP_FILES=()
while IFS= read -r file; do
    CPP_FILES+=("$file")
done < <(collect_cpp_files)
readonly CPP_FILES

readonly PYTHON_DIRS=(
    tools
)
readonly SHELL_FILES=(
    scripts/*.sh
)

if (( ${#CPP_FILES[@]} == 0 )); then
    printf "%b\n\n" "${YELLOW}warning:${NC} no C/C++ files found!"
fi

if (( ${#PYTHON_DIRS[@]} == 0 )); then
    printf "%b\n\n" "${YELLOW}warning:${NC} no python directories found!"
fi

if (( ${#SHELL_FILES[@]} == 0 )); then
    printf "%b\n\n" "${YELLOW}warning:${NC} no shell script files found!"
fi


# =========================
# Argument parsing
# =========================
FORMAT_MODE=0
LINT_MODE=0
FIX_MODE=0
VERBOSE_MODE=0
EXIT_CODE=0

for arg in "$@"; do
    case "$arg" in
    --format)
        FORMAT_MODE=1
        ;;
    --lint)
        LINT_MODE=1
        ;;
    --fix)
        FIX_MODE=1
        ;;
    --verbose)
        VERBOSE_MODE=1
        ;;
    *)
        printf "%b\n" "${RED}[Error] Unknown argument: ${arg}${NC}" >&2
        printf "%s\n" "Usage: ${0##*/} [--format] [--lint] [--fix] [--verbose]" >&2
        exit 1
        ;;
    esac
done

if ((!FIX_MODE)); then
    printf "%b\n\n" "${YELLOW}note:${NC} running in check-only mode (use --fix to apply fixes)" >&2
fi

# default behavior: if neither format nor lint is specified, run format
if ((FORMAT_MODE == 0 && LINT_MODE == 0)); then
    FORMAT_MODE=1
fi

find_executable() {
    local -r executable="$1"

    local path
    if ! path=$(command -v "$executable" 2>/dev/null); then
        printf "%b\n" "${RED}error: could not find ${executable}${NC}" >&2
        exit 1
    fi

    local version_output
    version_output=$("$path" --version 2>&1)

    if [[ -n "$version_output" ]]; then
        local version
        version=$(echo "$version_output" | grep -oE '[0-9]+(\.[0-9]+)+' | head -n 1)
        if [[ -n "$version" ]]; then
            printf "%b\n" "${GREEN}${executable} version: ${version}${NC}" >&2
        else
            printf "%b\n" "${YELLOW}${executable} version: found, but version string hidden${NC}" >&2
        fi
    else
        printf "%b\n" "${YELLOW}${executable} version: unavailable${NC}" >&2
    fi

    printf '%s\n' "$path"
}


readonly CLANG_FORMAT_PATH=$(find_executable clang-format)
readonly RUFF_PATH=$(find_executable ruff)
readonly TY_PATH=$(find_executable ty)
readonly SHELLCHECK_PATH=$(find_executable shellcheck)

RUFF_ARGS=("--respect-gitignore")
if ((VERBOSE_MODE)); then
    RUFF_ARGS+=("--verbose")
fi

# =========================
# FORMAT MODE
# =========================
if ((FORMAT_MODE)); then
    # ===== clang-format (C/C++) =====
    if ((${#CPP_FILES[@]})); then
        CLANG_FORMAT_ARGS=(
            "--style=file"
        )
        if ((!FIX_MODE)); then
            CLANG_FORMAT_ARGS+=("--dry-run" "--Werror")
        else
            CLANG_FORMAT_ARGS+=("-i")
        fi

        if ((VERBOSE_MODE)); then
            CLANG_FORMAT_ARGS+=("--verbose")
        fi

        printf "%b\n" "${BOLD}formatting with ${BLUE}clang-format${NC}..."
        if "${CLANG_FORMAT_PATH}" "${CLANG_FORMAT_ARGS[@]}" "${CPP_FILES[@]}"; then
            printf "%b\n" "${GREEN}done!${NC}"
        else
            EXIT_CODE=1
            printf "%b\n" "${RED}clang-format found issues${NC}"
        fi
    fi

    # ===== ruff (Python) =====
    if ((${#PYTHON_DIRS[@]})); then
        RUFF_FORMAT_ARGS=()
        if ((!FIX_MODE)); then
            RUFF_FORMAT_ARGS+=("--check")
        fi

        printf "\n"
        printf "%b\n" "${BOLD}formatting with ${BLUE}ruff${NC}..."
        if "${RUFF_PATH}" format "${RUFF_ARGS[@]}" "${RUFF_FORMAT_ARGS[@]}" "${PYTHON_DIRS[@]}"; then
            printf "%b\n" "${GREEN}done!${NC}"
        else
            EXIT_CODE=1
            printf "%b\n" "${RED}ruff format found issues${NC}"
        fi
    fi
fi

# =========================
# LINT MODE
# =========================
if ((LINT_MODE)); then
    # ===== ruff (Python) =====
    if ((${#PYTHON_DIRS[@]})); then
        RUFF_CHECK_ARGS=()
        if ((!FIX_MODE)); then
            RUFF_CHECK_ARGS+=("--no-fix")
        else
            RUFF_CHECK_ARGS+=("--fix")
        fi

        printf "\n"
        printf "%b\n" "${BOLD}linting with ${BLUE}ruff${NC}..."
        if "${RUFF_PATH}" check "${RUFF_ARGS[@]}" "${RUFF_CHECK_ARGS[@]}" "${PYTHON_DIRS[@]}"; then
            printf "%b\n" "${GREEN}done!${NC}"
        else
            EXIT_CODE=1
            printf "%b\n" "${RED}ruff check found issues${NC}"
        fi
    fi

    # ===== ty (Python) =====
    if ((${#PYTHON_DIRS[@]})); then
        TY_ARGS=("--respect-ignore-files")
        if ((VERBOSE_MODE)); then
            TY_ARGS+=("--verbose")
        fi

        printf "\n"
        printf "%b\n" "${BOLD}type checking with ${BLUE}ty${NC}..."
        if "${TY_PATH}" check "${TY_ARGS[@]}" "${PYTHON_DIRS[@]}"; then
            printf "%b\n" "${GREEN}done!${NC}"
        else
            EXIT_CODE=1
            printf "%b\n" "${RED}ty found issues${NC}"
        fi
    fi

    # ===== shellcheck (Bash) =====
    if ((${#SHELL_FILES[@]})); then
        printf "\n"
        printf "%b\n" "${BOLD}linting with ${BLUE}shellcheck${NC}..."
        if ((VERBOSE_MODE)); then
            printf "%s\n" "${SHELL_FILES[@]}"
        fi
        # SC2155 is separate declaration and command.
        if "${SHELLCHECK_PATH}" --exclude=SC2155 "${SHELL_FILES[@]}"; then
            printf "%b\n" "${GREEN}done!${NC}"
        else
            EXIT_CODE=1
            printf "%b\n" "${RED}shellcheck found issues${NC}"
        fi
    fi
fi

exit "$EXIT_CODE"
