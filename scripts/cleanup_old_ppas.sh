#!/bin/bash

set -e

if [ "$EUID" -ne 0 ]; then
    echo "Error: This script must be run as root"
    exit 1
fi

SOURCES_DIR="/etc/apt/sources.list.d"
TRUSTED_GPG_D="/etc/apt/trusted.gpg.d"
KEYRINGS_DIR="/usr/share/keyrings"

OLD_PPA_FILES=(
    "$SOURCES_DIR/ppa_launchpad_net_deadsnakes_ppa_ubuntu.list"
    "$SOURCES_DIR/deadsnakes-ubuntu-ppa-*.list"
    "$SOURCES_DIR/ppa_launchpad_net_ubuntu_toolchain_r_test_ubuntu.list"
    "$SOURCES_DIR/ubuntu-toolchain-r-ubuntu-test-*.list"
    "$SOURCES_DIR/vscode-old.list"
    "$SOURCES_DIR/github-cli-old.list"
    "$SOURCES_DIR/llvm.list"
    "$SOURCES_DIR/kitware.list"
    "$SOURCES_DIR/ros.list"
    "$SOURCES_DIR/ros2.list"
)

OLD_GPG_KEYS=(
    "BA6932366A755776"
    "1E9377A2BA9EF27F"
    "15CF4D18AF4F7421"
    "6084F3CF814B57C1CF12EFD515CF4D18AF4F7421"
    "DE19EB17684BA42D40D3D4102CFFAA61D743BF70"
    "EA587CE6512D89C580AAE55BA65337CCA8A748B8"
    "C1CF6E31E6BADE8868B172B4F42ED6FBAB17C654"
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
        "$TRUSTED_GPG_D/llvm*.gpg"
        "$TRUSTED_GPG_D/kitware*.gpg"
        "$TRUSTED_GPG_D/ros*.gpg"
    )

    for pattern in "${OLD_KEYRING_FILES[@]}"; do
        for file in $pattern; do
            [ -f "$file" ] && rm -f "$file" && echo "Removed: $file"
        done
    done
fi

if [ -d "$KEYRINGS_DIR" ]; then
    OLD_SIGNED_BY_KEYRINGS=(
        "$KEYRINGS_DIR/llvm-archive-keyring.asc"
    )

    for pattern in "${OLD_SIGNED_BY_KEYRINGS[@]}"; do
        for file in $pattern; do
            [ -f "$file" ] && rm -f "$file" && echo "Removed: $file"
        done
    done
fi

apt-get update
