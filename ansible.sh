#!/usr/bin/env bash

# Helper script to run Ansible playbooks

if [ "$#" -le 0 ]; then
    echo "Usage: $0 <playbook> <extra argument>"
    exit 1
fi

readonly MROVER_PATH=$(dirname "$0")
readonly ROS2_WS_PATH=$(realpath "${MROVER_PATH}"/../..)

run_playbook() {
    ansible-playbook -i "localhost," -c local -K "${MROVER_PATH}"/ansible/"$1" --extra-vars "ros2_workspace=${ROS2_WS_PATH} $2"
}

# workaround for a first-run-only ansible "No serialization profile" bug
# the cause of this is unconfirmed, there are multiple sources online such as
# https://github.com/ansible/ansible/issues/84782 that does not provide a clean solution
# retrying the playbook always works to resolve the issue, so temporary fix is below
if ! run_playbook "$@"; then
    echo "Ansible run failed; retrying once to counter the known first-run cache flake ..." >&2
    run_playbook "$@"
fi
