#!/usr/bin/env bash
# First-time native Ubuntu 22 setup.
# Ensures git is installed, clones the repo, then runs setup.sh.
# If you already have the repo, just run setup.sh directly.

# See: https://vaneyckt.io/posts/safer_bash_scripts_with_set_euxo_pipefail/
set -Eeuo pipefail

readonly RED_BOLD='\033[1;31m'
readonly GREY_BOLD='\033[1;30m'
readonly NC='\033[0m'

if ! grep -q '^VERSION_CODENAME=jammy' /etc/os-release 2>/dev/null; then
    echo -e "${RED_BOLD}This script requires Ubuntu 22.04.${NC}"
    exit 1
fi

echo -e "${GREY_BOLD}Ensuring SSH keys are set up ...${NC}"
if [ ! -f ~/.ssh/id_ed25519 ] && [ ! -f ~/.ssh/id_rsa ]; then
    echo -e "${RED_BOLD}Please see: https://docs.github.com/en/authentication/connecting-to-github-with-ssh/generating-a-new-ssh-key-and-adding-it-to-the-ssh-agent${NC}"
    exit 1
fi

# intentionally avoiding PPA, 2.34 to 2.51 jump isn't meaningful. CVEs covered via backport.
sudo apt install -y git git-lfs

readonly MROVER_PATH=~/mrover-ros2

if [ ! -d "${MROVER_PATH}/.git" ]; then
    echo -e "${GREY_BOLD}Cloning mrover-ros2 ...${NC}"
    git clone git@github.com:umrover/mrover-ros2 "${MROVER_PATH}"
fi

exec "${MROVER_PATH}/setup.sh"
