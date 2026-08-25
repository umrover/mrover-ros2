#!/usr/bin/env bash
# Native Ubuntu setup. Run this after cloning if you did not use bootstrap.sh.
# For macOS, Arch, Fedora or any non-Ubuntu host, use setup-portable.sh.

set -Eeuo pipefail

readonly CYAN='\033[1;36m'
readonly GREEN='\033[1;32m'
readonly RED='\033[1;31m'
readonly NC='\033[0m'

# The build role installs ROS and the toolchain from apt, so this path is
# Ubuntu-only. Everything else goes through pixi.
if ! grep -qi "ubuntu" /etc/os-release 2>/dev/null; then
  echo -e "${RED}This script requires Ubuntu. For other platforms, use ./setup-portable.sh${NC}" >&2
  exit 1
fi

if ! command -v ansible-playbook >/dev/null 2>&1; then
  echo -e "${CYAN}Installing Ansible ...${NC}"
  if ! grep -rq "^deb .*ansible/ansible" /etc/apt/sources.list /etc/apt/sources.list.d/ 2>/dev/null; then
    sudo apt-add-repository ppa:ansible/ansible -y
    sudo apt update
  fi
  sudo apt install -y ansible
fi

readonly MROVER_PATH=$(cd "$(dirname "$0")" && pwd)
cd "${MROVER_PATH}"

echo -e "${CYAN}Installing Ansible collections ...${NC}"
ansible-galaxy collection install -r ansible/requirements.yml

echo -e "${CYAN}Running Ansible ...${NC}"
if [ -f /etc/nv_tegra_release ]; then
  "${MROVER_PATH}/ansible.sh" jetson_build.yml
else
  "${MROVER_PATH}/ansible.sh" dev.yml
fi

echo ""
echo -e "${GREEN}================================================================${NC}"
echo -e "${GREEN}  Done! Log out and back in to apply shell changes.${NC}"
echo -e "${GREEN}  Then open a new terminal and run: mrover${NC}"
echo -e "${GREEN}================================================================${NC}"
echo ""
