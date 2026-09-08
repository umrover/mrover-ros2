#!/usr/bin/env bash

# Helper script to run Ansible playbooks

if [ "$#" -le 0 ]; then
    echo "Usage: $0 <playbook> <extra argument>"
    exit 1
fi

readonly MROVER_PATH=$(dirname "$(realpath "$0")")
# pass flags correctly, pull out playbook name then shift vars
readonly PLAYBOOK=$1
shift

ansible-playbook -i "localhost," -c local -K "${MROVER_PATH}/ansible/${PLAYBOOK}" \
    --extra-vars "mrover_repo=${MROVER_PATH}" "$@"
