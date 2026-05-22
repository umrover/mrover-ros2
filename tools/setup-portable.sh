#!/usr/bin/env bash
# Tier 2 (portable) one-time setup: macOS, Linux, WSL.

set -Eeuo pipefail

readonly GREY='\033[1;30m'
readonly GREEN='\033[1;32m'
readonly RED='\033[1;31m'
readonly NC='\033[0m'

cd "$(cd "$(dirname "${BASH_SOURCE[0]}")/.." && pwd)"

if ! command -v pixi >/dev/null 2>&1; then
  echo -e "${GREY}Installing pixi ...${NC}"
  curl -fsSL https://pixi.sh/install.sh | sh
  export PATH="${HOME}/.pixi/bin:${PATH}"
fi

echo -e "${GREY}Initializing submodules ...${NC}"
git submodule update --init deps/imgui deps/webgpuhpp deps/glfw3webgpu

echo -e "${GREY}Installing packages ...${NC}"
pixi install

readonly REPO_DIR="$PWD"
readonly ZSHENV="$HOME/.zshenv"
readonly ZSHRC="$HOME/.zshrc"
readonly MARKER="# MRover portable dev environment"

zshenv_has=no
zshrc_has=no
if [ -f "$ZSHENV" ] && grep -qF "$MARKER" "$ZSHENV"; then zshenv_has=yes; fi
if [ -f "$ZSHRC" ] && grep -qF "$MARKER" "$ZSHRC"; then zshrc_has=yes; fi

if [ "$zshenv_has" = yes ] && [ "$zshrc_has" = yes ]; then
  echo -e "${GREY}'mrover' command already configured.${NC}"
else
  read -r -p "Install the 'mrover' shell command? [y/N] " reply || reply=""
  if [[ "$reply" =~ ^[Yy] ]]; then
    if [ "$zshenv_has" = no ]; then
      { echo ""; echo "$MARKER"; echo "[ -f \"${REPO_DIR}/tools/mrover.zshenv\" ] && source \"${REPO_DIR}/tools/mrover.zshenv\""; } >> "$ZSHENV"
    fi
    if [ "$zshrc_has" = no ]; then
      { echo ""; echo "$MARKER"; echo "alias mrover='activate_mrover'"; } >> "$ZSHRC"
    fi
    echo -e "${GREEN}Done. Open a new terminal and run 'mrover'.${NC}"
  fi
fi

echo -e "${GREEN}Setup complete.${NC}"
echo "Run 'mrover' to enter the environment, then './build.sh' to build."
