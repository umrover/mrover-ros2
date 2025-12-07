#!/bin/bash

set -e

if [ "$EUID" -ne 0 ]; then
    echo "Error: This script must be run as root"
    exit 1
fi

SOURCES_DIR="/etc/apt/sources.list.d"
TRUSTED_GPG_D="/etc/apt/trusted.gpg.d"

OLD_PPA_FILES=(
    "$SOURCES_DIR/ppa_launchpad_net_deadsnakes_ppa_ubuntu.list"
    "$SOURCES_DIR/deadsnakes-ubuntu-ppa-*.list"
    "$SOURCES_DIR/ppa_launchpad_net_ubuntu_toolchain_r_test_ubuntu.list"
    "$SOURCES_DIR/ubuntu-toolchain-r-ubuntu-test-*.list"
    "$SOURCES_DIR/vscode-old.list"
    "$SOURCES_DIR/github-cli-old.list"
)

OLD_GPG_KEYS=(
    "BA6932366A755776"
    "1E9377A2BA9EF27F"
)

for pattern in "${OLD_PPA_FILES[@]}"; do
    for file in $pattern; do
        [ -f "$file" ] && rm -f "$file" && echo "Removed: $file"
    done
done

for key_id in "${OLD_GPG_KEYS[@]}"; do
    if apt-key list 2>/dev/null | grep -q "$key_id"; then
        apt-key del "$key_id" 2>/dev/null || true
        echo "Removed key: $key_id"
    fi
done

if [ -d "$TRUSTED_GPG_D" ]; then
    OLD_KEYRING_FILES=(
        "$TRUSTED_GPG_D/deadsnakes*.gpg"
        "$TRUSTED_GPG_D/ubuntu-toolchain*.gpg"
        "$TRUSTED_GPG_D/microsoft.gpg"
        "$TRUSTED_GPG_D/githubcli*.gpg"
    )

    for pattern in "${OLD_KEYRING_FILES[@]}"; do
        for file in $pattern; do
            [ -f "$file" ] && rm -f "$file" && echo "Removed: $file"
        done
    done
fi

apt-get update
