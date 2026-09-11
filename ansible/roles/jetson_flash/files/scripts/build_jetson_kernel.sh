#!/usr/bin/env bash

JETSON_SOURCE_DIR="$1"
UBUNTU_CODENAME="$2"

# Make Variables
export CROSS_COMPILE="${JETSON_SOURCE_DIR}/x-tools/*/bin/aarch64-none-linux-gnu-"
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
cp "kernel/kernel-${UBUNTU_CODENAME}-src/arch/arm64/boot/Image" "${JETSON_SOURCE_DIR}/Linux_for_Tegra/kernel/Image"

# Make NVIDIA Out Of Tree Modules
make modules
sudo -E make modules_install

pushd "${JETSON_SOURCE_DIR}/Linux_for_Tegra" || exit
sudo ./tools/l4t_update_initrd.sh

# Build the DTBs
pushd "${JETSON_SOURCE_DIR}/Linux_for_Tegra/source" || exit
make dtbs
cp "kernel-devicetree/generic-dts/dtbs/*" "${JETSON_SOURCE_DIR}/Linux_for_Tegra/kernel/dtb/"
