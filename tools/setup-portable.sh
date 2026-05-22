#!/usr/bin/env bash
# MRover Tier 2 (portable) dev setup: macOS, Linux, WSL.
# Tier 1 (robot / Ubuntu-native) uses setup-native.sh.

set -Eeuo pipefail

readonly GREY='\033[1;30m'
readonly GREEN='\033[1;32m'
readonly RED='\033[1;31m'
readonly NC='\033[0m'

# cd to the repo root (this script lives in tools/).
cd "$(cd "$(dirname "${BASH_SOURCE[0]}")/.." && pwd)"

readonly PIXI_BIN="${PIXI_HOME:-$HOME/.pixi}/bin"

if ! command -v pixi >/dev/null 2>&1; then
  if ! command -v curl >/dev/null 2>&1; then
    echo -e "${RED}curl is required to install pixi. Install curl and re-run.${NC}"
    exit 1
  fi
  echo -e "${GREY}Installing pixi ...${NC}"
  curl -fsSL https://pixi.sh/install.sh | bash
  export PATH="${PIXI_BIN}:${PATH}"  # for this run; installer updates future shells
  hash -r
fi

if ! command -v pixi >/dev/null 2>&1; then
  echo -e "${RED}pixi was installed but is not on PATH. Open a new terminal and re-run.${NC}"
  exit 1
fi

echo -e "${GREY}Resolving the environment with pixi ...${NC}"
pixi install

# Offer the 'mrover' command: source the repo's Tier 2 shell file from
# ~/.zshenv, add the alias to ~/.zshrc. Append-only, never replace.
readonly REPO_DIR="$PWD"
readonly ZSHENV="$HOME/.zshenv"
readonly ZSHRC="$HOME/.zshrc"
readonly MARKER="# MRover portable dev environment"

zshenv_has=no
zshrc_has=no
if [ -f "$ZSHENV" ] && grep -qF "$MARKER" "$ZSHENV"; then zshenv_has=yes; fi
if [ -f "$ZSHRC" ] && grep -qF "$MARKER" "$ZSHRC"; then zshrc_has=yes; fi

if [ "$zshenv_has" = yes ] && [ "$zshrc_has" = yes ]; then
  echo -e "${GREY}'mrover' command already installed; leaving it.${NC}"
else
  read -r -p "Install the 'mrover' command (~/.zshenv source line + ~/.zshrc alias)? [y/N] " reply || reply=""
  if [[ "$reply" =~ ^[Yy] ]]; then
    if [ "$zshenv_has" = no ]; then
      {
        echo ""
        echo "$MARKER"
        echo "[ -f \"${REPO_DIR}/tools/mrover.zshenv\" ] && source \"${REPO_DIR}/tools/mrover.zshenv\""
      } >> "$ZSHENV"
    fi
    if [ "$zshrc_has" = no ]; then
      {
        echo ""
        echo "$MARKER"
        echo "alias mrover='activate_mrover'"
      } >> "$ZSHRC"
    fi
    echo -e "${GREEN}Installed 'mrover'. Open a new terminal and type 'mrover'.${NC}"
  fi
fi

echo -e "${GREEN}Done.${NC}"
echo
echo "Open a new terminal and type 'mrover' to enter the environment."
echo "Then use ros2, colcon, bun, and python natively; the mrover package"
echo "overlay is sourced once the project has been built."
