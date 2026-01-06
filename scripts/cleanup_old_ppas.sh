#!/bin/bash

# Removes legacy GPG keys and sources that conflict with modern signed-by method

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

# Removed to avoid "Key is stored in legacy trusted.gpg keyring" warning
OLD_GPG_KEYS=(
    "BA6932366A755776"                            # Deadsnakes PPA
    "1E9377A2BA9EF27F"                            # Ubuntu Toolchain PPA
    "15CF4D18AF4F7421"                            # LLVM (short)
    "6084F3CF814B57C1CF12EFD515CF4D18AF4F7421"    # LLVM (full)
    "DE19EB17684BA42D40D3D4102CFFAA61D743BF70"    # Kitware 2022
    "EA587CE6512D89C580AAE55BA65337CCA8A748B8"    # Kitware 2025
    "4DBEBE3EEC96E7B8C6EC5BE99E92FDC6C5B9BA75"    # Kitware 2026
    "C1CF6E31E6BADE8868B172B4F42ED6FBAB17C654"    # ROS
)

for pattern in "${OLD_PPA_FILES[@]}"; do
    for file in $pattern; do
        [ -f "$file" ] && rm -f "$file" && echo "Removed: $file"
    done
done

for key_id in "${OLD_GPG_KEYS[@]}"; do
    if apt-key list 2>/dev/null | grep -q "$key_id"; then
        apt-key del "$key_id" 2>/dev/null || true
        echo "Removed legacy apt-key: $key_id"
    fi
done

# Remove old keyrings from legacy trusted.gpg.d directory
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

# Remove old/conflicting keyrings from /usr/share/keyrings
# These may conflict with new installations
if [ -d "$KEYRINGS_DIR" ]; then
    OLD_SIGNED_BY_KEYRINGS=(
        "$KEYRINGS_DIR/llvm-archive-keyring.asc"    # Old ASCII armored format
    )

    for pattern in "${OLD_SIGNED_BY_KEYRINGS[@]}"; do
        for file in $pattern; do
            [ -f "$file" ] && rm -f "$file" && echo "Removed: $file"
        done
    done
fi

# Remove old kitware keyring only if not managed by package
if [ -f "$KEYRINGS_DIR/kitware-archive-keyring.gpg" ]; then
    if ! dpkg -s kitware-archive-keyring >/dev/null 2>&1; then
        rm -f "$KEYRINGS_DIR/kitware-archive-keyring.gpg"
        echo "Removed: $KEYRINGS_DIR/kitware-archive-keyring.gpg (not managed by package)"
    fi
fi

apt-get update
