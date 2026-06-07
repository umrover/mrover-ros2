#!/usr/bin/env bash
# First-time setup: clones the repo then runs setup.sh.
# If you already have the repo, just run ./setup.sh directly.

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

if ! command -v git >/dev/null 2>&1; then
  echo -e "${GREY}Installing git ...${NC}"
  case "$OS" in
    Darwin) brew install git git-lfs ;;
    Linux)
      if command -v dnf >/dev/null 2>&1; then sudo dnf install -y git git-lfs
      elif command -v pacman >/dev/null 2>&1; then sudo pacman -S --noconfirm git git-lfs
      elif command -v apt-get >/dev/null 2>&1; then sudo apt-get install -y git git-lfs
      fi
      ;;
  esac
fi

readonly DEFAULT_MROVER_PATH=~/mrover
read -r -p "$(echo -e "${GREY}Clone path [${DEFAULT_MROVER_PATH}]: ${NC}")" MROVER_PATH
MROVER_PATH="${MROVER_PATH:-$DEFAULT_MROVER_PATH}"

if [ ! -d "${MROVER_PATH}/.git" ]; then
  git clone git@github.com:umrover/mrover-ros2 "${MROVER_PATH}"
fi

exec "${MROVER_PATH}/setup.sh"
