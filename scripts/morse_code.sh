#!/usr/bin/env bash

# Translate the "input" string in a JSON file to or from Morse code, writing the
# result back into the "output" field of that same file.
#
#   ./scripts/morse_code.sh scripts/morse_code.json

set -euo pipefail

SCRIPT_DIR=$(cd -- "$(dirname -- "${BASH_SOURCE[0]}")" && pwd)
readonly SCRIPT_DIR

exec python3 "${SCRIPT_DIR}/morse_code.py" "$@"
