#!/usr/bin/env bash
# Run this after cloning to set up the development environment.

set -Eeuo pipefail

readonly CYAN='\033[1;36m'
readonly GREEN='\033[1;32m'
readonly RED='\033[1;31m'
readonly NC='\033[0m'

OS="$(uname -s)"

if [[ "$OS" == "Darwin" ]] && ! command -v brew >/dev/null 2>&1; then
  echo -e "${CYAN}Installing Homebrew ...${NC}"
  /bin/bash -c "$(curl -fsSL https://raw.githubusercontent.com/Homebrew/install/HEAD/install.sh)"
  if [ -f /opt/homebrew/bin/brew ]; then
    eval "$(/opt/homebrew/bin/brew shellenv)"
  else
    eval "$(/usr/local/bin/brew shellenv)"
  fi
fi

if ! command -v ansible-playbook >/dev/null 2>&1; then
  echo -e "${CYAN}Installing Ansible ...${NC}"
  case "$OS" in
    Darwin) brew install ansible ;;
    Linux)
      if command -v dnf >/dev/null 2>&1; then sudo dnf install -y ansible git git-lfs
      elif command -v pacman >/dev/null 2>&1; then sudo pacman -S --noconfirm ansible git git-lfs
      elif command -v apt-get >/dev/null 2>&1; then sudo apt-get install -y ansible git git-lfs
      else
        echo -e "${RED}Unsupported package manager. Install Ansible manually and re-run.${NC}"
        exit 1
      fi
      ;;
    *)
      echo -e "${RED}Unsupported OS: ${OS}${NC}"
      exit 1
      ;;
  esac
fi

if ! command -v pixi >/dev/null 2>&1; then
  echo -e "${CYAN}Installing pixi ...${NC}"
  curl -fsSL https://pixi.sh/install.sh | sh
fi
export PATH="${HOME}/.pixi/bin:${PATH}"

readonly MROVER_PATH=$(cd "$(dirname "$0")" && pwd)
cd "${MROVER_PATH}"

echo -e "${CYAN}Updating submodules ...${NC}"
git submodule update --init

echo -e "${CYAN}Fetching LFS objects ...${NC}"
git lfs pull

echo -e "${CYAN}Installing pixi packages ...${NC}"
pixi install

echo -e "${CYAN}Installing Ansible collections ...${NC}"
ansible-galaxy collection install -r ansible/requirements.yml

echo -e "${CYAN}Running Ansible ...${NC}"
"${MROVER_PATH}/ansible.sh" dev-portable.yml

echo -e "${GREEN}Done. Open a new terminal and run 'mrover'.${NC}"
