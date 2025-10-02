#!/bin/bash

chmod +x "$0"

# sudo apt-get install lld
source /opt/ros/humble/setup.bash

cd /ros2_ws/src/mrover
./build.sh

source /ros2_ws/install/setup.bash
exec "$@"  # runs whatever command is passed, keeps it as PID 1
