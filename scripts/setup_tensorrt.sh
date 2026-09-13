#!/usr/bin/env bash
# Downloads TensorRT (matching the version native/percep_build installs via apt) to

set -Eeuo pipefail

readonly TRT_VERSION="10.13.3.9-1+cuda13.0"
readonly REPO_URL="https://developer.download.nvidia.com/compute/cuda/repos/ubuntu2404/x86_64"

readonly TRT_PACKAGES=(
    "libnvinfer-headers-dev:ddc5a6f2bc72e572141cf9f117604fe07ec0d01a59edfb411decdaa780dc4b08"
    "libnvinfer-dev:b794514a6e3d39b2affcedf8e667900fa40e46d39f48bf7a602e3447d57e56ab"
    "libnvinfer10:bc99693fa55a62afc193b32b1eecb18006393ff2959d95a737683b6c238ab546"
    "libnvonnxparsers-dev:1efba486fcf795f4ebbb802ed3cf5e999ca6e75caed25c90b4920265e5ccc43e"
    "libnvonnxparsers10:1d4d48eca9ba5eafb2c48e1f62b0631194ac43967577666ed24b36b95d64184e"
)

readonly REPO_ROOT="$(cd "$(dirname "${BASH_SOURCE[0]}")/.." && pwd)"
readonly DEST="${REPO_ROOT}/deps/tensorrt-prebuilt"
readonly VERSION_FILE="${DEST}/.version"

if [ -f "${VERSION_FILE}" ] && [ "$(cat "${VERSION_FILE}")" = "${TRT_VERSION}" ] && [ -f "${DEST}/include/NvInfer.h" ]; then
    exit 0
fi

readonly PLATFORM="$(uname -s)/$(uname -m)"

if [ "${PLATFORM}" != "Linux/x86_64" ]; then
    # avoid exiting nonzero so other scripts running setup_tensorrt don't fail
    echo "TensorRT .deb packages are only published for Linux x86_64, skipping. Object detection will not be built."
    exit 0
fi

tmpdir=$(mktemp -d)
trap 'rm -rf "${tmpdir}"' EXIT

echo "Downloading TensorRT ${TRT_VERSION}..."
mkdir -p "${tmpdir}/root"
for entry in "${TRT_PACKAGES[@]}"; do
    name="${entry%%:*}"
    sha256="${entry##*:}"
    deb="${tmpdir}/${name}.deb"

    curl -fL --silent --show-error "${REPO_URL}/${name}_${TRT_VERSION}_amd64.deb" -o "${deb}"
    echo "${sha256}  ${deb}" | sha256sum --check --status

    ar x "${deb}" data.tar.xz --output "${tmpdir}"
    tar -xJf "${tmpdir}/data.tar.xz" -C "${tmpdir}/root"
    rm -f "${tmpdir}/data.tar.xz"
done

find "${tmpdir}/root/usr/lib/x86_64-linux-gnu" -name "*.a" -delete

rm -rf "${DEST}"
mkdir -p "${DEST}"
mv "${tmpdir}/root/usr/include/x86_64-linux-gnu" "${DEST}/include"
mv "${tmpdir}/root/usr/lib/x86_64-linux-gnu" "${DEST}/lib"

echo "${TRT_VERSION}" >"${VERSION_FILE}"
echo "TensorRT ${TRT_VERSION} installed."
