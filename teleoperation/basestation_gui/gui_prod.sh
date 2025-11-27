#!/bin/bash

# Navigate to basestation directory
cd teleoperation/basestation_gui || exit 1

# 1. Build Frontend
echo "Building Frontend..."
cd frontend || exit 1
bun install
bun run build
cd .. || exit 1

# 2. Start Backend (which serves the built frontend)
echo "Starting Production Server (serving static files)..."
export SERVE_STATIC_FRONTEND="true"
# We use python3 directly. Ensure dependencies are installed.
exec python3 server.py