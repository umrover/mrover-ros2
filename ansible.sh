#!/usr/bin/env bash

# Helper script to run Ansible playbooks

if [ "$#" -le 0 ]; then
    echo "Usage: $0 <playbook> <extra argument>"
    exit 1
fi

readonly MROVER_PATH=$(dirname "$(realpath "$0")")

ansible-playbook -i "localhost," -c local -K "${MROVER_PATH}"/ansible/"$1" --extra-vars "mrover_repo=${MROVER_PATH}" ${2:+"$2"}
