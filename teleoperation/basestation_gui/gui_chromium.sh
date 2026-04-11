#!/bin/bash

readonly ADDRESS="http://localhost:8000"

echo "Waiting for server..."
while ! curl -s http://localhost:8000 >/dev/null 2>&1; do
    sleep 0.5
done

echo "Server is up, launching Chromium..."
if command -v chromium &>/dev/null; then
	exec chromium --app=${ADDRESS}
else
	exec google-chrome --app=${ADDRESS}
fi
