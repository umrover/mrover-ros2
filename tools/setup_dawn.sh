#!/usr/bin/env bash
# Downloads prebuilt Dawn to deps/dawn-prebuilt/. Pinned at v20260423.175430.

set -Eeuo pipefail

readonly DAWN_SHA="31e25af254ab572c77054edec4946d2244e184dd"
readonly DAWN_VERSION="v20260423.175430"
readonly BASE_URL="https://github.com/google/dawn/releases/download/${DAWN_VERSION}"

cd "$(cd "$(dirname "${BASH_SOURCE[0]}")/.." && pwd)"

readonly DEST="deps/dawn-prebuilt"
readonly VERSION_FILE="${DEST}/.version"

if [ -f "${VERSION_FILE}" ] && [ "$(cat "${VERSION_FILE}")" = "${DAWN_SHA}" ]; then
    exit 0
fi

OS=$(uname -s)
ARCH=$(uname -m)

if [ "${OS}" = "Darwin" ] && [ "${ARCH}" = "arm64" ]; then
    BINARY_TARBALL="Dawn-${DAWN_SHA}-macos-latest-Release.tar.gz"
elif [ "${OS}" = "Linux" ] && [ "${ARCH}" = "x86_64" ]; then
    BINARY_TARBALL="Dawn-${DAWN_SHA}-ubuntu-latest-Release.tar.gz"
else
    echo "Unsupported platform: ${OS}/${ARCH}" >&2
    exit 1
fi

readonly HEADERS_TARBALL="dawn-headers-${DAWN_SHA}.tar.gz"

tmpdir=$(mktemp -d)
trap 'rm -rf "${tmpdir}"' EXIT

echo "Downloading Dawn ${DAWN_VERSION} for ${OS}/${ARCH}..."
curl -fSL "${BASE_URL}/${BINARY_TARBALL}" -o "${tmpdir}/dawn-bin.tar.gz"
curl -fSL "${BASE_URL}/${HEADERS_TARBALL}" -o "${tmpdir}/dawn-headers.tar.gz"

rm -rf "${DEST}"
mkdir -p "${DEST}/lib" "${DEST}/include"

mkdir -p "${tmpdir}/bin"
tar -xzf "${tmpdir}/dawn-bin.tar.gz" -C "${tmpdir}/bin"
if [ "${OS}" = "Darwin" ]; then
    find "${tmpdir}/bin" -name "libwebgpu_dawn.dylib" -exec cp {} "${DEST}/lib/" \;
else
    find "${tmpdir}/bin" -name "libwebgpu_dawn.so" -exec cp {} "${DEST}/lib/" \;
fi

mkdir -p "${tmpdir}/hdr"
tar -xzf "${tmpdir}/dawn-headers.tar.gz" -C "${tmpdir}/hdr"
if [ -d "${tmpdir}/hdr/include" ]; then
    cp -r "${tmpdir}/hdr/include/." "${DEST}/include/"
else
    cp -r "${tmpdir}/hdr/." "${DEST}/include/"
fi

echo "${DAWN_SHA}" > "${VERSION_FILE}"
echo "Dawn ${DAWN_VERSION} installed."
