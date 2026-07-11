#!/usr/bin/env bash

# Helper script to run Ansible playbooks

if [ "$#" -le 0 ]; then
    echo "Usage: $0 <playbook> <extra argument>"
    exit 1
fi

sudo -v # Ensure Ansible has sudo permission

readonly MROVER_PATH=$(realpath "$(dirname "$0")")
readonly ROS2_WS_PATH=$(realpath "${MROVER_PATH}"/../..)
ansible-playbook -i "localhost," -c local "${MROVER_PATH}"/ansible/"$1" --extra-vars "ros2_workspace=${ROS2_WS_PATH} mrover_repo=${MROVER_PATH} $2"
