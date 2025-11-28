#!/bin/bash
# Generates API documentation from the basestation backend server.
# Starts backend if not already running, generates docs, and opens in browser.

SCRIPT_DIR="$(cd "$(dirname "${BASH_SOURCE[0]}")" && pwd)"
PROJECT_ROOT="$(dirname "$(dirname "$SCRIPT_DIR")")"
OUTPUT_DIR="$PROJECT_ROOT/teleoperation/api-docs"

BACKEND_PID=""

if ! curl -s http://localhost:8000/openapi.json > /dev/null 2>&1; then
    "$PROJECT_ROOT/teleoperation/basestation_gui/gui_backend.sh" > /dev/null 2>&1 &
    BACKEND_PID=$!

    for i in {1..30}; do
        curl -s http://localhost:8000/openapi.json > /dev/null 2>&1 && break
        [ "$i" -eq 30 ] && echo "Server failed to start" && exit 1
        sleep 1
    done
fi

cleanup() {
    [ -n "$BACKEND_PID" ] && kill $BACKEND_PID 2>/dev/null
}
trap cleanup EXIT

npx @openapitools/openapi-generator-cli generate \
    -i http://localhost:8000/openapi.json \
    -g html2 \
    -o "$OUTPUT_DIR" && xdg-open "$OUTPUT_DIR/index.html"
