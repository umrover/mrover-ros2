#!/bin/bash

cd teleoperation/basestation_gui || exit 1
exec python3 server.py
