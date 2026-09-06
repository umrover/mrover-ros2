#!/usr/bin/env bash
# First-time native Ubuntu/Jetpack setup: clones the repo then runs setup.sh.
# If you already have the repo, just run ./setup.sh directly.
# For other platforms, use bootstrap-portable.sh.

# See: https://vaneyckt.io/posts/safer_bash_scripts_with_set_euxo_pipefail/
set -Eeuo pipefail

readonly GREY='\033[1;30m'
readonly RED='\033[1;31m'
readonly NC='\033[0m'

if ! grep -qi "ubuntu" /etc/os-release 2>/dev/null; then
  echo -e "${RED}This script requires Ubuntu. For other platforms, use bootstrap-portable.sh.${NC}"
  exit 1
fi

echo -e "${GREY}Checking SSH keys ...${NC}"
if [ ! -f ~/.ssh/id_ed25519 ] && [ ! -f ~/.ssh/id_rsa ]; then
  echo -e "${RED}No SSH key found. See: https://docs.github.com/en/authentication/connecting-to-github-with-ssh/generating-a-new-ssh-key-and-adding-it-to-the-ssh-agent${NC}"
  exit 1
fi

# distro git is fine for cloning; no git-core PPA, the version gap isn't meaningful and CVEs are backported
sudo apt install -y git git-lfs

readonly MROVER_PATH=~/mrover-ros2

if [ ! -d "${MROVER_PATH}/.git" ]; then
  echo -e "${GREY}Cloning mrover-ros2 ...${NC}"
  git clone git@github.com:umrover/mrover-ros2 "${MROVER_PATH}"
fi

exec "${MROVER_PATH}/setup.sh"
