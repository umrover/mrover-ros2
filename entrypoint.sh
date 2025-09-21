#!/bin/bash


echo "Delay for NoVNC"
sleep 5
source /opt/ros/humble/setup.bash
source /home/mrover/ros2_ws/install/setup.bash
exec "$@"  # runs whatever command is passed, keeps it as PID 1
