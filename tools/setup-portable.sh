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
git submodule update --init

echo -e "${GREY}Installing packages ...${NC}"
pixi install

readonly REPO_DIR="$PWD"
readonly ZSHRC="$HOME/.zshrc"
readonly MARKER="# MRover portable dev environment"

if [ -f "$ZSHRC" ] && grep -qF "$MARKER" "$ZSHRC"; then
  echo -e "${GREY}'mrover' command already configured.${NC}"
else
  read -r -p "Install the 'mrover' shell command? [y/N] " reply || reply=""
  if [[ "$reply" =~ ^[Yy] ]]; then
    printf '\n%s\nsource "%s/tools/mrover.zshenv"\n' "$MARKER" "$REPO_DIR" >> "$ZSHRC"
    echo -e "${GREEN}Done. Open a new terminal and run 'mrover'.${NC}"
  fi
fi

echo -e "${GREEN}Setup complete.${NC}"
echo "Run 'mrover' to enter the environment, then './build.sh' to build."
