#!/bin/bash

export RCUTILS_CONSOLE_OUTPUT_FORMAT="[{severity}] {message}"

cd teleoperation/basestation_gui || exit 1
exec python3 server.py
