#!/usr/bin/env bash
# Portable (pixi) setup. Run this after cloning if you did not use
# bootstrap-portable.sh. For a native Ubuntu install, use setup.sh.

set -Eeuo pipefail

readonly CYAN='\033[1;36m'
readonly GREEN='\033[1;32m'
readonly RED='\033[1;31m'
readonly NC='\033[0m'

OS="$(uname -s)"
ARCH="$(uname -m)"

# pixi.toml declares platforms = ["osx-arm64", "linux-64"]; bail before doing
# any work rather than failing deep inside `pixi install`
case "${OS}/${ARCH}" in
  Linux/x86_64 | Darwin/arm64) ;;
  *)
    echo -e "${RED}The portable environment supports Linux x86_64 and macOS arm64, not ${OS}/${ARCH}.${NC}" >&2
    echo -e "${RED}Add the platform to pixi.toml, or use the native install: ./ansible.sh dev.yml${NC}" >&2
    exit 1
    ;;
esac

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

readonly MROVER_PATH=$(cd "$(dirname "$0")" && pwd)
cd "${MROVER_PATH}"

echo -e "${CYAN}Installing Ansible collections ...${NC}"
ansible-galaxy collection install -r ansible/requirements.yml

"${MROVER_PATH}/scripts/fix_sudo_rs.sh"

echo -e "${CYAN}Running Ansible ...${NC}"
"${MROVER_PATH}/ansible.sh" dev-portable.yml

echo ""
echo -e "${GREEN}================================================================${NC}"
echo -e "${GREEN}  Done! Log out and back in to apply shell changes.${NC}"
echo -e "${GREEN}  Then open a new terminal and run: mrover${NC}"
echo -e "${GREEN}================================================================${NC}"
echo ""
