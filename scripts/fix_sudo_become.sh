#!/usr/bin/env bash
# Run this manually if ./ansible.sh prompts for the become password more
# than once during a single run: https://github.com/ansible/ansible/issues/85536

set -Eeuo pipefail

readonly SUDOERS_FILE="/etc/sudoers.d/mrover-timestamp-type"

echo "Defaults timestamp_type=global" | sudo tee "${SUDOERS_FILE}" >/dev/null
sudo chmod 0440 "${SUDOERS_FILE}"
sudo visudo -c
