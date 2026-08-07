#!/usr/bin/env bash

# Pick which wire to cut from a comma separated list of wire colors, given left
# to right. The colors are r (red), w (white), bl (blue), y (yellow) and
# bk (black).
#
#   ./scripts/wire_cut.sh r,bl,y,bk

set -euo pipefail

SCRIPT_DIR=$(cd -- "$(dirname -- "${BASH_SOURCE[0]}")" && pwd)
readonly SCRIPT_DIR

exec python3 "${SCRIPT_DIR}/wire_cut.py" "$@"
