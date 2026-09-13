#!/usr/bin/env bash
# First-time native Ubuntu 24 setup.
# Ensures git is installed, clones the repo, then runs setup.sh.
# If you already have the repo, just run setup.sh directly.
# For other platforms, use bootstrap-portable.sh.

# See: https://vaneyckt.io/posts/safer_bash_scripts_with_set_euxo_pipefail/
set -Eeuo pipefail

readonly RED_BOLD='\033[1;31m'
readonly GREY_BOLD='\033[1;30m'
readonly NC='\033[0m'

if ! grep -q '^VERSION_CODENAME=noble' /etc/os-release 2>/dev/null; then
    echo -e "${RED_BOLD}This script requires Ubuntu 24.04. For other platforms, use bootstrap-portable.sh.${NC}"
    exit 1
fi

echo -e "${GREY_BOLD}Ensuring SSH keys are set up ...${NC}"
if [ ! -f ~/.ssh/id_ed25519 ] && [ ! -f ~/.ssh/id_rsa ]; then
    echo -e "${RED_BOLD}Please see: https://docs.github.com/en/authentication/connecting-to-github-with-ssh/generating-a-new-ssh-key-and-adding-it-to-the-ssh-agent${NC}"
    exit 1
fi

# distro git is fine for cloning; no git-core PPA, the version gap isn't meaningful and CVEs are backported
sudo apt install -y git git-lfs

readonly CATKIN_PATH=~/ros2_ws
readonly MROVER_PATH=${CATKIN_PATH}/src/mrover

if [ ! -d "${MROVER_PATH}/.git" ]; then
    echo -e "${GREY_BOLD}Creating ROS workspace ...${NC}"
    mkdir -p "${CATKIN_PATH}"/src
    git clone git@github.com:umrover/mrover-ros2 "${CATKIN_PATH}"/src/mrover
    cd "${CATKIN_PATH}"/src/mrover
fi

exec "${MROVER_PATH}/setup.sh"
