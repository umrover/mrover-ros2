#!/usr/bin/env bash

# Open a window that previews every connected screen. Tick the ones you want
# and press capture, once per screenshot. The window stays open.
#
#   ./scripts/screenshot.sh ~/rover_screenshots

set -euo pipefail

SCRIPT_DIR=$(cd -- "$(dirname -- "${BASH_SOURCE[0]}")" && pwd)
readonly SCRIPT_DIR

exec python3 "${SCRIPT_DIR}/screenshot.py" "$@"
