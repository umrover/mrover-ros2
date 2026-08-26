#!/bin/bash

readonly ADDRESS="http://localhost:8000"

echo "Waiting for server..."
while ! curl -s http://localhost:8000 >/dev/null 2>&1; do
    sleep 0.5
done

echo "Server is up, launching Chromium..."
exec chromium --app=${ADDRESS}
