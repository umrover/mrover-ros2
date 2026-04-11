#!/bin/bash

readonly ADDRESS="http://localhost:8000"

echo "Waiting for server..."
while ! curl -s http://localhost:8000 >/dev/null 2>&1; do
    sleep 0.5
done

echo "Server is up, launching browser..."
if command -v chromium &>/dev/null; then
    exec chromium --app=${ADDRESS}
elif command -v google-chrome &>/dev/null; then
    exec google-chrome --app=${ADDRESS}
else
    echo "Error: neither chromium nor google-chrome found" >&2
    exit 1
fi
