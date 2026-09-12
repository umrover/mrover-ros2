#!/usr/bin/env bash

set -euxo pipefail

JETSON_SOURCE_DIR="$1"
UBUNTU_CODENAME="$2"
PCAN_SOURCE_DIR="${JETSON_SOURCE_DIR}/peak-linux-driver-${3}.${4}.${5}"

# Make Variables
export CROSS_COMPILE="${JETSON_SOURCE_DIR}/x-tools/bin/aarch64-none-linux-gnu-"
export KERNEL_LOCATION="${JETSON_SOURCE_DIR}/Linux_for_Tegra/source/kernel/kernel-${UBUNTU_CODENAME}/"
export ARCH="arm64"
export PCAN_BASIC=""
PATH="$PATH:/tmp/x-tools/lib"

# Change to the kernel sources directory
pushd "${PCAN_SOURCE_DIR}/" || exit

sudo make DESTDIR="${JETSON_SOURCE_DIR}/Linux_for_Tegra/rootfs" clean
sudo make DESTDIR="${JETSON_SOURCE_DIR}/Linux_for_Tegra/rootfs" netdev
sudo make DESTDIR="${JETSON_SOURCE_DIR}/Linux_for_Tegra/rootfs" install
