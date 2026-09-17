#!/usr/bin/env bash
# https://github.com/conda-forge/cctools-and-ld64-feedstock/issues/112
set -euo pipefail

label=$(softwareupdate --list 2>/dev/null | grep -o 'Command Line Tools for Xcode-26[0-9.]*' | sort -V | tail -1)
sudo softwareupdate --install "$label"

sdk=$(find /Library/Developer/CommandLineTools/SDKs -maxdepth 1 -name 'MacOSX26*.sdk' | sort -V | tail -1)

echo "export SDKROOT=${sdk}" >>~/.zshrc
