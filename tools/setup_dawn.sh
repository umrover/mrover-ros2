#!/usr/bin/env bash
# Downloads prebuilt Dawn to deps/dawn-prebuilt/. Pinned at v20260423.175430.
#
# Supported platforms (prebuilt only, no source builds):
#   - Linux  x86_64  (amd64)         -> ubuntu-latest Release
#   - macOS  arm64   (Apple Silicon) -> macos-latest  Release
# Anything else is unsupported and exits non-zero.

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

case "${OS}/${ARCH}" in
    Linux/x86_64)
        BINARY_TARBALL="Dawn-${DAWN_SHA}-ubuntu-latest-Release.tar.gz"
        ;;
    Darwin/arm64)
        BINARY_TARBALL="Dawn-${DAWN_SHA}-macos-latest-Release.tar.gz"
        ;;
    *)
        cat >&2 <<EOF
Unsupported platform: ${OS}/${ARCH}

This project only supports prebuilt Dawn on:
  - Linux x86_64 (amd64)
  - macOS arm64  (Apple Silicon)

There is no supported build path for ${OS}/${ARCH}.
EOF
        exit 1
        ;;
esac

tmpdir=$(mktemp -d)
trap 'rm -rf "${tmpdir}"' EXIT

echo "Downloading Dawn ${DAWN_VERSION} for ${OS}/${ARCH}..."
curl -fSL "${BASE_URL}/${BINARY_TARBALL}" -o "${tmpdir}/dawn.tar.gz"

rm -rf "${DEST}"
mkdir -p "${DEST}"
tar -xzf "${tmpdir}/dawn.tar.gz" -C "${DEST}" --strip-components 1

echo "${DAWN_SHA}" > "${VERSION_FILE}"
echo "Dawn ${DAWN_VERSION} installed."
