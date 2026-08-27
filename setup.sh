#!/usr/bin/env bash
# Native Ubuntu 22 setup. Run after cloning.

set -Eeuo pipefail

readonly CYAN='\033[1;36m'
readonly GREEN='\033[1;32m'
readonly RED='\033[1;31m'
readonly NC='\033[0m'

if ! grep -q '^VERSION_CODENAME=jammy' /etc/os-release 2>/dev/null; then
  echo -e "${RED}This script requires Ubuntu 22.${NC}" >&2
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

readonly MROVER_PATH=$(dirname "$(realpath "$0")")
cd "${MROVER_PATH}"

echo -e "${CYAN}Installing Ansible collections ...${NC}"
ansible-galaxy collection install -r ansible/requirements.yml

echo -e "${CYAN}Running Ansible ...${NC}"
"${MROVER_PATH}/ansible.sh" dev.yml

echo ""
echo -e "${GREEN}================================================================${NC}"
echo -e "${GREEN}  Done! Log out and back in to apply shell changes.${NC}"
echo -e "${GREEN}================================================================${NC}"
echo ""
