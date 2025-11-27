#!/bin/bash

cd teleoperation/basestation_gui || exit 1
# Launch the ConnectRPC server
# We use python3 directly. Ensure dependencies are installed.
python3 server.py
