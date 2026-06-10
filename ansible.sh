#!/usr/bin/env bash

# Helper script to run Ansible playbooks

if [ "$#" -le 0 ]; then
    echo "Usage: $0 <playbook> <extra argument>"
    exit 1
fi

readonly MROVER_PATH=$(cd "$(dirname "$0")" && pwd)
ansible-playbook -i "localhost," -c local "${MROVER_PATH}"/ansible/"$1" --ask-become-pass --extra-vars "mrover_repo=${MROVER_PATH} ${2:-}"
