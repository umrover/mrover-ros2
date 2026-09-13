#!/usr/bin/env bash
# migrate portable to ros2_ws/src/mrover structure
# run from a new terminal without sourcing mrover

set -euo pipefail

old_repo="$(cd "$(dirname "$0")/.." && pwd -P)"
new_repo="${HOME}/ros2_ws/src/mrover"

if [ -n "${PIXI_PROJECT_ROOT:-}" ]; then
    echo "mrover is sourced, open a new terminal" >&2
    exit 1
fi

if [ -e "${new_repo}" ]; then
    echo "${new_repo} already exists" >&2
    exit 1
fi

rm -rf "${old_repo}/.pixi" "${old_repo}/build" "${old_repo}/install" "${old_repo}/log" "${old_repo}/compile_commands.json"

mkdir -p "${HOME}/ros2_ws/src"
mv "${old_repo}" "${new_repo}"
sed -i.bak "s|${old_repo}/scripts/|${new_repo}/scripts/|g" ~/.zshrc

cd "${new_repo}"
pixi install

echo "Done. Open a new terminal, then run: mrover && ./build.sh"
