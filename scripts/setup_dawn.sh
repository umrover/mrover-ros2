#!/usr/bin/env bash
# Downloads prebuilt Dawn to deps/dawn-prebuilt/ from the umrover/dawn fork.
# Release: tag v<upstream_sha>, asset Dawn-<platform>.tar.gz where <platform> is
# linux-x86_64, linux-arm64, macos-arm64 or macos-x86_64.

set -Eeuo pipefail

# Upstream Dawn commit; the release tag is v<sha>.
readonly DAWN_SHA="31e25af254ab572c77054edec4946d2244e184dd"
readonly BASE_URL="https://github.com/umrover/dawn/releases/download/v${DAWN_SHA}"

readonly REPO_ROOT="$(cd "$(dirname "${BASH_SOURCE[0]}")/.." && pwd)"
readonly DEST="${REPO_ROOT}/deps/dawn-prebuilt"
readonly VERSION_FILE="${DEST}/.version"

if [ -f "${VERSION_FILE}" ] && [ "$(cat "${VERSION_FILE}")" = "${DAWN_SHA}" ] && [ -f "${DEST}/lib/cmake/Dawn/DawnConfig.cmake" ]; then
    exit 0
fi

readonly PLATFORM="$(uname -s)/$(uname -m)"

case "${PLATFORM}" in
    Linux/x86_64)   readonly DAWN_PLATFORM="linux-x86_64" ;;
    Linux/aarch64)  readonly DAWN_PLATFORM="linux-arm64" ;;
    Darwin/arm64)   readonly DAWN_PLATFORM="macos-arm64" ;;
    Darwin/x86_64)  readonly DAWN_PLATFORM="macos-x86_64" ;;
    *)
        echo >&2 "No prebuilt Dawn published for ${PLATFORM} (Linux x86_64/aarch64, macOS arm64/x86_64 available)."
        exit 1
        ;;
esac

readonly BINARY_TARBALL="Dawn-${DAWN_PLATFORM}.tar.gz"

tmpdir=$(mktemp -d)
trap 'rm -rf "${tmpdir}"' EXIT

echo "Downloading Dawn ${DAWN_SHA} for ${PLATFORM} (${DAWN_PLATFORM})..."
curl -fL --silent --show-error "${BASE_URL}/${BINARY_TARBALL}" -o "${tmpdir}/dawn.tar.gz"

mkdir -p "${tmpdir}/dawn"
tar -xzf "${tmpdir}/dawn.tar.gz" -C "${tmpdir}/dawn" --strip-components 1

# build bug on native: linux tarballs install to lib64/, but CMake's prefix path search checks lib/, symlink to both.
if [ -d "${tmpdir}/dawn/lib64" ] && [ ! -e "${tmpdir}/dawn/lib" ]; then
    ln -sf lib64 "${tmpdir}/dawn/lib"
fi

rm -rf "${DEST}"
mkdir -p "$(dirname "${DEST}")"
mv "${tmpdir}/dawn" "${DEST}"

echo "${DAWN_SHA}" > "${VERSION_FILE}"
echo "Dawn ${DAWN_SHA} installed."
