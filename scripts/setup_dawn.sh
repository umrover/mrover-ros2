#!/usr/bin/env bash
# Downloads prebuilt Dawn to deps/dawn-prebuilt/ from Google's official release

set -Eeuo pipefail

readonly DAWN_VERSION="v20260423.175430"
readonly DAWN_SHA="31e25af254ab572c77054edec4946d2244e184dd"
readonly BASE_URL="https://github.com/google/dawn/releases/download/${DAWN_VERSION}"

readonly REPO_ROOT="$(cd "$(dirname "${BASH_SOURCE[0]}")/.." && pwd)"
readonly DEST="${REPO_ROOT}/deps/dawn-prebuilt"
readonly VERSION_FILE="${DEST}/.version"

if [ -f "${VERSION_FILE}" ] && [ "$(cat "${VERSION_FILE}")" = "${DAWN_SHA}" ] && [ -f "${DEST}/lib/cmake/Dawn/DawnConfig.cmake" ]; then
    exit 0
fi

readonly PLATFORM="$(uname -s)/$(uname -m)"

# ansible's build role invokes this unconditionally, including on Jetson, so skip rather than fail provisioning
if [ "${PLATFORM}" != "Linux/x86_64" ]; then
    echo "No prebuilt Dawn published for ${PLATFORM} (only Linux x86_64), skipping. The simulator will not be built."
    exit 0
fi

readonly BINARY_TARBALL="Dawn-${DAWN_SHA}-ubuntu-latest-Release.tar.gz"

tmpdir=$(mktemp -d)
trap 'rm -rf "${tmpdir}"' EXIT

echo "Downloading Dawn ${DAWN_VERSION} for ${PLATFORM}..."
curl -fL --silent --show-error "${BASE_URL}/${BINARY_TARBALL}" -o "${tmpdir}/dawn.tar.gz"

mkdir -p "${tmpdir}/dawn"
tar -xzf "${tmpdir}/dawn.tar.gz" -C "${tmpdir}/dawn" --strip-components 1

# build bug on native: linux tarballs install to lib64/, but CMake's prefix path search checks lib/, symlink to both.
if [ -d "${tmpdir}/dawn/lib64" ] && [ ! -e "${tmpdir}/dawn/lib" ]; then
    ln -sf lib64 "${tmpdir}/dawn/lib"
fi

rm -rf "${DEST}"
mv "${tmpdir}/dawn" "${DEST}"

echo "${DAWN_SHA}" > "${VERSION_FILE}"
echo "Dawn ${DAWN_SHA} installed."
