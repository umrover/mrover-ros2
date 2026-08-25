#!/usr/bin/env bash

# Helper script to run Ansible playbooks

if [ "$#" -le 0 ]; then
    echo "Usage: $0 <playbook> <extra argument>"
    exit 1
fi

sudo -v # Ensure Ansible has sudo permission

readonly MROVER_PATH=$(realpath "$(dirname "$0")")

run_playbook() {
    ansible-playbook -i "localhost," -c local --ask-become-pass "${MROVER_PATH}"/ansible/"$1" --extra-vars "mrover_repo=${MROVER_PATH}" ${2:+"$2"}
}

# works around a first-run-only ansible-core "No serialization profile" flake
# the cause of this is unconfirmed, there are multiple sources online such as 
# https://github.com/ansible/ansible/issues/84782 that does not provide a clear explanation
# re-trying the playbook always works to resolve the issue, so temporary fix is below
if ! run_playbook "$@"; then
    echo "Ansible run failed; retrying once to counter the known first-run cache flake ..." >&2
    run_playbook "$@"
fi
