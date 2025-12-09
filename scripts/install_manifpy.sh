#!/bin/bash

set -e

if [ "$EUID" -ne 0 ]; then
    echo "Error: This script must be run as root"
    exit 1
fi

if ! python3 -c "import manifpy" 2>/dev/null; then
    echo "installing manifpy"

    sudo apt-get install -y libeigen3-dev

    if [ ! -d "$HOME/manif" ]; then
        cd "$HOME"
        git clone https://github.com/artivis/manif.git
    fi

    cd "$HOME/manif"
    python3 -m pip install .

    echo "manifpy installed"
else
    echo "manifpy already installed"
fi

echo "initializing submodule"
cd "$(dirname "$0")/.."
git submodule update --init deps/manif

echo "done"
