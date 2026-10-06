#!/usr/bin/env bash

# Helper script to run Ansible playbooks

if [ "$#" -le 0 ]; then
    echo "Usage: $0 <playbook> <extra argument>"
    exit 1
fi

readonly MROVER_PATH=$(dirname "$(realpath "$0")")
readonly ROS2_WS_PATH=$(realpath "${MROVER_PATH}/../..")
readonly PLAYBOOK=$1
shift

export ANSIBLE_CONFIG="${MROVER_PATH}/ansible/ansible.cfg"

ansible-playbook -i "localhost," -c local --ask-become-pass "${MROVER_PATH}/ansible/${PLAYBOOK}" \
    --extra-vars "ros2_workspace=${ROS2_WS_PATH}" "$@"
