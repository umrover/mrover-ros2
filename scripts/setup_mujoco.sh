#!/usr/bin/env bash
# Downloads the official prebuilt MuJoCo C library to deps/mujoco-prebuilt/.
# Release: github.com/google-deepmind/mujoco, asset mujoco-<ver>-<platform>.tar.gz.

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

case "${PLATFORM}" in
    Linux/x86_64)   readonly MUJOCO_PLATFORM="linux-x86_64" ;;
    Linux/aarch64)  readonly MUJOCO_PLATFORM="linux-aarch64" ;;
    *)
        # macOS ships a .dmg (mujoco.framework) with a different layout; not handled yet.
        echo >&2 "No prebuilt MuJoCo wired for ${PLATFORM} (Linux x86_64/aarch64 supported)."
        exit 1
        ;;
esac

readonly BINARY_TARBALL="mujoco-${MUJOCO_VERSION}-${MUJOCO_PLATFORM}.tar.gz"

tmpdir=$(mktemp -d)
trap 'rm -rf "${tmpdir}"' EXIT

echo "Downloading MuJoCo ${MUJOCO_VERSION} for ${PLATFORM} (${MUJOCO_PLATFORM})..."
curl -fL --silent --show-error "${BASE_URL}/${BINARY_TARBALL}" -o "${tmpdir}/mujoco.tar.gz"

mkdir -p "${tmpdir}/mujoco"
tar -xzf "${tmpdir}/mujoco.tar.gz" -C "${tmpdir}/mujoco" --strip-components 1

rm -rf "${DEST}"
mv "${tmpdir}/mujoco" "${DEST}"

echo "${MUJOCO_VERSION}" > "${VERSION_FILE}"
echo "MuJoCo ${MUJOCO_VERSION} installed."
