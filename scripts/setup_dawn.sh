#!/usr/bin/env bash
# Downloads prebuilt Dawn to deps/dawn-prebuilt/
# Pinned at v20260423.175430.

set -Eeuo pipefail

readonly DAWN_SHA="31e25af254ab572c77054edec4946d2244e184dd"
readonly DAWN_VERSION="v20260423.175430"
readonly BASE_URL="https://github.com/google/dawn/releases/download/${DAWN_VERSION}"

readonly REPO_ROOT="$(cd "$(dirname "${BASH_SOURCE[0]}")/.." && pwd)"
readonly DEST="${REPO_ROOT}/deps/dawn-prebuilt"
readonly VERSION_FILE="${DEST}/.version"

# Checks if downloaded
if [ -f "${VERSION_FILE}" ] && [ "$(cat "${VERSION_FILE}")" = "${DAWN_SHA}" ] && [ -f "${DEST}/lib/cmake/Dawn/DawnConfig.cmake" ]; then
    exit 0
fi

readonly PLATFORM="$(uname -s)/$(uname -m)"

case "${PLATFORM}" in
    Linux/x86_64)  readonly BINARY_TARBALL="Dawn-${DAWN_SHA}-ubuntu-latest-Release.tar.gz" ;;
    Darwin/arm64)  readonly BINARY_TARBALL="Dawn-${DAWN_SHA}-macos-latest-Release.tar.gz" ;;
    *)
        echo >&2 "Unsupported platform: ${PLATFORM}. Supported: Linux/x86_64, Darwin/arm64."
        exit 1
        ;;
esac

tmpdir=$(mktemp -d)
trap 'rm -rf "${tmpdir}"' EXIT

echo "Downloading Dawn ${DAWN_VERSION} for ${PLATFORM}..."
curl -fL --silent --show-error "${BASE_URL}/${BINARY_TARBALL}" -o "${tmpdir}/dawn.tar.gz"

mkdir -p "${tmpdir}/dawn"
tar -xzf "${tmpdir}/dawn.tar.gz" -C "${tmpdir}/dawn" --strip-components 1

# Native build bug:
# Linux tarballs install to lib64/, but CMake's prefix path search checks lib/
# Symlink lib -> lib64 if exists so both work.
if [ -d "${tmpdir}/dawn/lib64" ] && [ ! -e "${tmpdir}/dawn/lib" ]; then
    ln -sf lib64 "${tmpdir}/dawn/lib"
fi

rm -rf "${DEST}"
mv "${tmpdir}/dawn" "${DEST}"

echo "${DAWN_SHA}" > "${VERSION_FILE}"
echo "Dawn ${DAWN_VERSION} installed."
