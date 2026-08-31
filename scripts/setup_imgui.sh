#!/usr/bin/env bash
# Clones the imgui source to deps/imgui-src/ so build.sh never needs network access.

set -Eeuo pipefail

readonly IMGUI_TAG="v1.92.8"

readonly REPO_ROOT="$(cd "$(dirname "${BASH_SOURCE[0]}")/.." && pwd)"
readonly DEST="${REPO_ROOT}/deps/imgui-src"
readonly VERSION_FILE="${DEST}/.version"

if [ -f "${VERSION_FILE}" ] && [ "$(cat "${VERSION_FILE}")" = "${IMGUI_TAG}" ]; then
    exit 0
fi

tmpdir=$(mktemp -d)
trap 'rm -rf "${tmpdir}"' EXIT

echo "Cloning imgui ${IMGUI_TAG}..."
git clone --quiet --depth 1 --branch "${IMGUI_TAG}" https://github.com/ocornut/imgui.git "${tmpdir}/imgui"

rm -rf "${DEST}"
mkdir -p "$(dirname "${DEST}")"
mv "${tmpdir}/imgui" "${DEST}"

echo "${IMGUI_TAG}" > "${VERSION_FILE}"
echo "imgui ${IMGUI_TAG} installed."
