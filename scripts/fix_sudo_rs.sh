#!/usr/bin/env bash
# On Ubuntu 26, switch the default sudo-rs to sudo to avoid breaking ansible become
set -Eeuo pipefail

if [ -x /usr/bin/sudo.ws ] && sudo --version 2>/dev/null | grep -qi "sudo-rs"; then
  sudo update-alternatives --set sudo /usr/bin/sudo.ws
fi
