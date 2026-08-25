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

# On Apple Silicon, `uname -m` can wrongly report x86_64:
# 1. The shell/terminal itself is running translated under Rosetta 2
#      (sysctl.proc_translated=1) -- brew/ansible/pixi would run translated
#      too, so bail with a clear message instead of failing later inside
#      `pixi install`.
#   2. The shell itself is native, but this particular `uname` invocation
#      still picked the x86_64 slice of its universal binary (seen even with
#      the system /usr/bin/uname, not just a shadowed one -- an exec
#      architecture-preference quirk). hw.optional.arm64=1 confirms the
#      hardware/shell are actually arm64, and since the rest of this script
#      spawns from this same untranslated shell, it's safe to correct ARCH
#      and continue.
if [[ "$OS" == "Darwin" && "$ARCH" == "x86_64" ]]; then
  if [[ "$(sysctl -n sysctl.proc_translated 2>/dev/null || echo 0)" == "1" ]]; then
    echo -e "${RED}Detected Apple Silicon running under Rosetta translation (uname reports x86_64, but this is arm64 hardware).${NC}" >&2
    echo -e "${RED}Quit Terminal/iTerm, make sure \"Open using Rosetta\" is unchecked (Get Info on the app), and re-run this script natively.${NC}" >&2
    exit 1
  elif [[ "$(sysctl -n hw.optional.arm64 2>/dev/null || echo 0)" == "1" ]]; then
    echo -e "${CYAN}Note: uname reported x86_64 but this shell is running natively on arm64 hardware. Continuing as arm64.${NC}" >&2
    ARCH="arm64"
  fi
fi

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

echo -e "${CYAN}Running Ansible ...${NC}"
"${MROVER_PATH}/ansible.sh" dev-portable.yml

echo ""
echo -e "${GREEN}================================================================${NC}"
echo -e "${GREEN}  Done! Log out and back in to apply shell changes.${NC}"
echo -e "${GREEN}  Then open a new terminal and run: mrover${NC}"
echo -e "${GREEN}================================================================${NC}"
echo ""
