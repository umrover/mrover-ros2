#!/usr/bin/env bash

set -euxo pipefail

JETSON_SOURCE_DIR="$1"
UBUNTU_CODENAME="$2"
PCAN_SOURCE_DIR="${JETSON_SOURCE_DIR}/peak-linux-driver-${3}.${4}.${5}"
X_TOOLS_NAME="$6"

# Make Variables
export CROSS_COMPILE="${JETSON_SOURCE_DIR}/x-tools/${X_TOOLS_NAME}/bin/${X_TOOLS_NAME}-"
export KERNEL_LOCATION="${JETSON_SOURCE_DIR}/Linux_for_Tegra/source/kernel/kernel-${UBUNTU_CODENAME}/"
export ARCH="arm64"
export PCAN_BASIC=""
PATH="$PATH:{JETSON_SOURCE_DIR}/x-tools/"

# Change to the kernel sources directory
pushd "${PCAN_SOURCE_DIR}/" || exit

sudo -E make DESTDIR="${JETSON_SOURCE_DIR}/Linux_for_Tegra/rootfs" clean
sudo -E make DESTDIR="${JETSON_SOURCE_DIR}/Linux_for_Tegra/rootfs" netdev
sudo -E make DESTDIR="${JETSON_SOURCE_DIR}/Linux_for_Tegra/rootfs" install
