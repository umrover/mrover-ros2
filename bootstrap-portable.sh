#!/usr/bin/env bash
# First-time portable setup: clones the repo then runs setup-portable.sh.
# If you already have the repo, just run ./setup-portable.sh directly.

set -Eeuo pipefail

readonly RED_BOLD='\033[1;31m'
readonly GREY_BOLD='\033[1;30m'
readonly NC='\033[0m'

OS="$(uname -s)"

echo -e "${GREY_BOLD}Ensuring SSH keys are set up ...${NC}"
if [ ! -f ~/.ssh/id_ed25519 ] && [ ! -f ~/.ssh/id_rsa ]; then
    echo -e "${RED_BOLD}Please see: https://docs.github.com/en/authentication/connecting-to-github-with-ssh/generating-a-new-ssh-key-and-adding-it-to-the-ssh-agent${NC}"
    exit 1
fi

if [[ "$OS" == "Darwin" ]] && ! command -v brew >/dev/null 2>&1; then
    echo -e "${GREY_BOLD}Installing Homebrew ...${NC}"
    /bin/bash -c "$(curl -fsSL https://raw.githubusercontent.com/Homebrew/install/HEAD/install.sh)"
    if [ -f /opt/homebrew/bin/brew ]; then
        eval "$(/opt/homebrew/bin/brew shellenv)"
    else
        eval "$(/usr/local/bin/brew shellenv)"
    fi
fi

if ! command -v git >/dev/null 2>&1; then
    echo -e "${GREY_BOLD}Installing git ...${NC}"
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

readonly CATKIN_PATH=~/ros2_ws
readonly MROVER_PATH=${CATKIN_PATH}/src/mrover

if [ ! -d "${MROVER_PATH}" ]; then
    mkdir -p "${CATKIN_PATH}"/src
    git clone git@github.com:umrover/mrover-ros2 "${MROVER_PATH}"
fi

exec "${MROVER_PATH}/setup-portable.sh"
