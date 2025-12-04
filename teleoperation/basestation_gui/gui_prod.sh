#!/bin/bash

cd teleoperation/basestation_gui || exit 1

cd frontend || exit 1
bun install || exit 1
bun run build || exit 1
cd .. || exit 1

export SERVE_STATIC_FRONTEND="true"
exec python3 server.py