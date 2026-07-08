#!/usr/bin/env bash
# Linux: downloads mujoco-<ver>-<platform>.tar.gz from the GitHub release
# macOS: homebrew cask install, assumes already done through ansible

set -Eeuo pipefail

readonly MUJOCO_VERSION="3.10.0"
readonly BASE_URL="https://github.com/google-deepmind/mujoco/releases/download/${MUJOCO_VERSION}"

readonly REPO_ROOT="$(cd "$(dirname "${BASH_SOURCE[0]}")/.." && pwd)"
readonly DEST="${REPO_ROOT}/deps/mujoco-prebuilt"
readonly VERSION_FILE="${DEST}/.version"

if [ -f "${VERSION_FILE}" ] && [ "$(cat "${VERSION_FILE}")" = "${MUJOCO_VERSION}" ] && [ -f "${DEST}/include/mujoco/mujoco.h" ]; then
    exit 0
fi

readonly PLATFORM="$(uname -s)/$(uname -m)"

tmpdir=$(mktemp -d)
trap 'rm -rf "${tmpdir}"' EXIT

setup_linux() {
    case "${PLATFORM}" in
        Linux/x86_64)   local mujoco_platform="linux-x86_64" ;;
        Linux/aarch64)  local mujoco_platform="linux-aarch64" ;;
        *)
            echo >&2 "No prebuilt MuJoCo wired for ${PLATFORM} (Linux x86_64/aarch64, macOS supported)."
            exit 1
            ;;
    esac
    local binary_tarball="mujoco-${MUJOCO_VERSION}-${mujoco_platform}.tar.gz"

    echo "Downloading MuJoCo ${MUJOCO_VERSION} for ${PLATFORM} (${mujoco_platform})..."
    # use system libcurl over pixi's
    env -u LD_LIBRARY_PATH curl -fL --silent --show-error "${BASE_URL}/${binary_tarball}" -o "${tmpdir}/mujoco.tar.gz"

    mkdir -p "${tmpdir}/mujoco"
    tar -xzf "${tmpdir}/mujoco.tar.gz" -C "${tmpdir}/mujoco" --strip-components 1

    rm -rf "${DEST}"
    mv "${tmpdir}/mujoco" "${DEST}"
}

setup_macos() {
    if ! command -v brew >/dev/null 2>&1; then
        echo >&2 "Homebrew not found on PATH. Run the dev-portable ansible playbook first (installs the mujoco cask)."
        exit 1
    fi

    # check where brew installed
    local app_path
    app_path="$(brew list --cask mujoco 2>/dev/null | grep -m1 '\.app/$\|\.app$' || true)"
    if [ -z "${app_path}" ]; then
        echo >&2 "MuJoCo cask not installed. Run the dev-portable ansible playbook first (installs the mujoco cask)."
        exit 1
    fi

    local framework="${app_path%/}/Contents/Frameworks/mujoco.framework"
    if [ ! -d "${framework}" ]; then
        echo >&2 "Expected ${framework} inside the installed cask but it's missing."
        exit 1
    fi

    local dylib
    dylib="$(find -L "${framework}/Versions/Current" -maxdepth 1 -name 'libmujoco.*.dylib' | head -1)"
    if [ -z "${dylib}" ]; then
        echo >&2 "No libmujoco.*.dylib found under ${framework}/Versions/Current."
        exit 1
    fi

    mkdir -p "${tmpdir}/mujoco/include/mujoco" "${tmpdir}/mujoco/lib"
    cp "${framework}/Versions/Current/Headers/"*.h "${tmpdir}/mujoco/include/mujoco/"
    cp "${dylib}" "${tmpdir}/mujoco/lib/libmujoco.dylib"

    # Rewrite the install name to match our flat lib/ dir and re-sign
    install_name_tool -id "@rpath/libmujoco.dylib" "${tmpdir}/mujoco/lib/libmujoco.dylib"
    codesign --force --sign - "${tmpdir}/mujoco/lib/libmujoco.dylib"

    rm -rf "${DEST}"
    mv "${tmpdir}/mujoco" "${DEST}"
}

case "$(uname -s)" in
    Linux)  setup_linux ;;
    Darwin) setup_macos ;;
    *)
        echo >&2 "No prebuilt MuJoCo wired for ${PLATFORM}, only Linux x86_64/aarch64, macOS supported"
        exit 1
        ;;
esac

echo "${MUJOCO_VERSION}" > "${VERSION_FILE}"
echo "MuJoCo ${MUJOCO_VERSION} installed."
