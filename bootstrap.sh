#!/usr/bin/env bash
# Native Ubuntu/Jetpack workstation setup. For other platforms, use bootstrap-portable.sh.

# See: https://vaneyckt.io/posts/safer_bash_scripts_with_set_euxo_pipefail/
set -Eeuo pipefail

readonly GREY='\033[1;30m'
readonly GREEN='\033[1;32m'
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

PPAS=(ansible/ansible git-core/ppa)
NEED_APT_UPDATE=false
for PPA in "${PPAS[@]}"; do
  if ! grep -rq "^deb .*${PPA}" /etc/apt/sources.list /etc/apt/sources.list.d/ 2>/dev/null; then
    echo -e "${GREY}Adding PPA: ${PPA}${NC}"
    sudo apt-add-repository ppa:"${PPA}" -y
    NEED_APT_UPDATE=true
  fi
done

if [ "${NEED_APT_UPDATE}" = true ]; then
  sudo apt update
fi
sudo apt install -y ansible git git-lfs

readonly MROVER_PATH=~/mrover-ros2

if [ ! -d "${MROVER_PATH}" ]; then
  echo -e "${GREY}Cloning mrover-ros2 ...${NC}"
  git clone --recurse-submodules git@github.com:umrover/mrover-ros2 "${MROVER_PATH}"
fi

if [ -f /etc/nv_tegra_release ]; then
  "${MROVER_PATH}"/ansible.sh jetson_build.yml
else
  "${MROVER_PATH}"/ansible.sh dev.yml
fi

echo -e "${GREEN}Done. Log out and back in to apply shell changes.${NC}"
