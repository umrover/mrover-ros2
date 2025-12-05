#!/bin/bash

set -e

if [ "$EUID" -ne 0 ]; then
    echo "Error: This script must be run as root"
    exit 1
fi

apt-get install -y libeigen3-dev

TEMP_DIR=$(mktemp -d)
cd "$TEMP_DIR"

git clone https://github.com/artivis/manif.git
cd manif

python3 -m pip install .

cd /
rm -rf "$TEMP_DIR"

echo "manifpy installed successfully"
