#!/usr/bin/env bash
# Downloads prebuilt Dawn to deps/dawn-prebuilt/ from the umrover/dawn fork.
# Release (Linux only): tag v<upstream_sha>, asset Dawn-<platform>.tar.gz
# where <platform> is linux-x86_64 or linux-arm64.

set -Eeuo pipefail

# Upstream Dawn commit == main's deps/dawn submodule SHA; the release tag is v<sha>.
readonly DAWN_SHA="79bc2cda3ac2c9ebacb598a13f97349e35bf783a"
readonly BASE_URL="https://github.com/umrover/dawn/releases/download/v${DAWN_SHA}"

readonly REPO_ROOT="$(cd "$(dirname "${BASH_SOURCE[0]}")/.." && pwd)"
readonly DEST="${REPO_ROOT}/deps/dawn-prebuilt"
readonly VERSION_FILE="${DEST}/.version"

if [ -f "${VERSION_FILE}" ] && [ "$(cat "${VERSION_FILE}")" = "${DAWN_SHA}" ] && [ -f "${DEST}/include/webgpu/webgpu.h" ]; then
    exit 0
fi

readonly PLATFORM="$(uname -s)/$(uname -m)"

case "${PLATFORM}" in
    Linux/x86_64)   readonly DAWN_PLATFORM="linux-x86_64" ;;
    Linux/aarch64)  readonly DAWN_PLATFORM="linux-arm64" ;;
    *)
        echo >&2 "No prebuilt Dawn published for ${PLATFORM} yet (only Linux x86_64/aarch64 available)."
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
mv "${tmpdir}/dawn" "${DEST}"

echo "${DAWN_SHA}" > "${VERSION_FILE}"
echo "Dawn ${DAWN_SHA} installed."
