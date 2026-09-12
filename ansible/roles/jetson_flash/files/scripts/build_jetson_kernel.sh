#!/usr/bin/env bash

JETSON_SOURCE_DIR="$1"
UBUNTU_CODENAME="$2"
X_TOOLS_NAME="$4"

# Make Variables
export CROSS_COMPILE="${JETSON_SOURCE_DIR}/x-tools/${X_TOOLS_NAME}/bin/${X_TOOLS_NAME}-"
export INSTALL_MOD_PATH="${JETSON_SOURCE_DIR}/Linux_for_Tegra/rootfs/"
export KERNEL_HEADERS="${JETSON_SOURCE_DIR}/Linux_for_Tegra/source/kernel/kernel-${UBUNTU_CODENAME}"
export INSTALL_MOD_PATH="${JETSON_SOURCE_DIR}/Linux_for_Tegra/rootfs/"

# Change to the kernel sources directory
pushd "${JETSON_SOURCE_DIR}/Linux_for_Tegra/source" || exit

# Invoke Make commands
# Build the kernel
make clean
make KERNEL_DEF_CONFIG=oldconfig -C kernel
sudo -E make install -C kernel
cp "kernel/kernel-${UBUNTU_CODENAME}/arch/arm64/boot/Image" "${JETSON_SOURCE_DIR}/Linux_for_Tegra/kernel/Image"

# Make NVIDIA Out Of Tree Modules
make modules
sudo -E make modules_install

pushd "${JETSON_SOURCE_DIR}/Linux_for_Tegra" || exit
sudo ./tools/l4t_update_initrd.sh

# Build the DTBs
pushd "${JETSON_SOURCE_DIR}/Linux_for_Tegra/source" || exit
make dtbs
cp -r "./build/nvidia-public/devicetree/generic-dtbs/." "${JETSON_SOURCE_DIR}/Linux_for_Tegra/kernel/dtb/"
