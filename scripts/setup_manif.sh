#!/usr/bin/env bash
# Clones the manif source to deps/manif-src/ so build.sh never needs network access.

set -Eeuo pipefail

readonly MANIF_TAG="0.0.5"

readonly REPO_ROOT="$(cd "$(dirname "${BASH_SOURCE[0]}")/.." && pwd)"
readonly DEST="${REPO_ROOT}/deps/manif-src"
readonly VERSION_FILE="${DEST}/.version"

if [ -f "${VERSION_FILE}" ] && [ "$(cat "${VERSION_FILE}")" = "${MANIF_TAG}" ]; then
    exit 0
fi

tmpdir=$(mktemp -d)
trap 'rm -rf "${tmpdir}"' EXIT

echo "Cloning manif ${MANIF_TAG}..."
git clone --quiet --depth 1 --branch "${MANIF_TAG}" https://github.com/artivis/manif.git "${tmpdir}/manif"

rm -rf "${DEST}"
mkdir -p "$(dirname "${DEST}")"
mv "${tmpdir}/manif" "${DEST}"

echo "${MANIF_TAG}" > "${VERSION_FILE}"
echo "manif ${MANIF_TAG} installed."
