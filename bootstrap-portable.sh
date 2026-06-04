#!/usr/bin/env bash
# Portable setup for macOS, Fedora, Arch, and other Linux distros.
# Uses pixi for the build environment and Ansible for dev tooling.

set -Eeuo pipefail

readonly GREY='\033[1;30m'
readonly GREEN='\033[1;32m'
readonly RED='\033[1;31m'
readonly NC='\033[0m'

OS="$(uname -s)"

echo -e "${GREY}Checking SSH keys ...${NC}"
if [ ! -f ~/.ssh/id_ed25519 ] && [ ! -f ~/.ssh/id_rsa ]; then
  echo -e "${RED}No SSH key found. See: https://docs.github.com/en/authentication/connecting-to-github-with-ssh/generating-a-new-ssh-key-and-adding-it-to-the-ssh-agent${NC}"
  exit 1
fi

if [[ "$OS" == "Darwin" ]] && ! command -v brew >/dev/null 2>&1; then
  echo -e "${GREY}Installing Homebrew ...${NC}"
  /bin/bash -c "$(curl -fsSL https://raw.githubusercontent.com/Homebrew/install/HEAD/install.sh)"
  if [ -f /opt/homebrew/bin/brew ]; then
    eval "$(/opt/homebrew/bin/brew shellenv)"
  else
    eval "$(/usr/local/bin/brew shellenv)"
  fi
fi

if ! command -v ansible-playbook >/dev/null 2>&1; then
  echo -e "${GREY}Installing Ansible ...${NC}"
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

if ! command -v git >/dev/null 2>&1; then
  echo -e "${GREY}Installing git ...${NC}"
  case "$OS" in
    Darwin) brew install git git-lfs ;;
    Linux)
      if command -v dnf >/dev/null 2>&1; then sudo dnf install -y git git-lfs
      elif command -v pacman >/dev/null 2>&1; then sudo pacman -S --noconfirm git git-lfs
      fi
      ;;
  esac
fi

if ! command -v pixi >/dev/null 2>&1; then
  echo -e "${GREY}Installing pixi ...${NC}"
  curl -fsSL https://pixi.sh/install.sh | sh
fi
export PATH="${HOME}/.pixi/bin:${PATH}"

readonly DEFAULT_MROVER_PATH=~/mrover
read -r -p "$(echo -e "${GREY}Clone path [${DEFAULT_MROVER_PATH}]: ${NC}")" MROVER_PATH
MROVER_PATH="${MROVER_PATH:-$DEFAULT_MROVER_PATH}"

if [ ! -d "${MROVER_PATH}/.git" ]; then
  git clone git@github.com:umrover/mrover-ros2 "${MROVER_PATH}"
fi

cd "${MROVER_PATH}"
git submodule update --init

echo -e "${GREY}Installing pixi packages ...${NC}"
pixi install

echo -e "${GREY}Installing Ansible collections ...${NC}"
ansible-galaxy collection install -r ansible/requirements.yml

echo -e "${GREY}Running Ansible ...${NC}"
ansible-playbook -i "localhost," -c local ansible/dev-portable.yml \
  --extra-vars "mrover_repo=${MROVER_PATH}"

echo -e "${GREEN}Done. Open a new terminal and run 'mrover'.${NC}"
