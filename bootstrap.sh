#!/usr/bin/env bash

# Run on a fresh Ubuntu 24 install.
# Installs Ansible and Git, then clones the mrover repo
# Ansible will be used to finish configuring the system

# See: https://vaneyckt.io/posts/safer_bash_scripts_with_set_euxo_pipefail/
set -Eeuo pipefail

readonly RED_BOLD='\033[1;31m'
readonly BLUE_BOLD='\033[1;34m'
readonly GREY_BOLD='\033[1;30m'
readonly YELLOW_BOLD='\033[1;33m'
readonly NC='\033[0m'

echo -e "${GREY_BOLD}Ensuring SSH keys are set up ...${NC}"
if [ ! -f ~/.ssh/id_ed25519 ] && [ ! -f ~/.ssh/id_rsa ]; then
  echo -e "${RED_BOLD}Please see: https://docs.github.com/en/authentication/connecting-to-github-with-ssh/generating-a-new-ssh-key-and-adding-it-to-the-ssh-agent${NC}"
  exit 1
fi

PPAS=(
  ansible/ansible
  git-core/ppa
)
NEED_APT_UPDATE=false
for PPA in "${PPAS[@]}"; do
  if ! grep -q "^deb .*${PPA}" /etc/apt/sources.list /etc/apt/sources.list.d/*;
  then
    echo -e "${GREY_BOLD}Adding PPA: ${PPA}${NC}"
    sudo apt-add-repository ppa:"${PPA}" -y
    NEED_APT_UPDATE=true
  fi
done

if [ "${NEED_APT_UPDATE}" = true ]; then
    sudo apt update
fi
sudo apt install -y ansible git git-lfs

readonly MROVER_PATH=~/mrover-ros2

FIRST_TIME_SETUP=false

if [ ! -d "${MROVER_PATH}" ]; then
  echo -e "${GREY_BOLD}Cloning mrover-ros2 ...${NC}"
  git clone --recurse-submodules git@github.com:umrover/mrover-ros2 "${MROVER_PATH}"
  FIRST_TIME_SETUP=true
fi

echo -e "${GREY_BOLD}Using Ansible to finish up ...${NC}"
"${MROVER_PATH}"/ansible.sh dev.yml

if [ "${FIRST_TIME_SETUP}" ]; then
  echo -e "${GREY_BOLD}All done! Welcome to MRover!${NC}"
  echo -e "${YELLOW_BOLD}Please log out and back in!${NC}"
fi
