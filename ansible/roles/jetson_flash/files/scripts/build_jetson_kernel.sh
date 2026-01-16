#!/usr/bin/env bash

JETSON_SOURCE_DIR="/tmp/jetson-flash"

# Make Variables
export CROSS_COMPILE="/tmp/aarch64--glibc--stable-2022.08-1/bin/aarch64-buildroot-linux-gnu-"
export INSTALL_MOD_PATH="${JETSON_SOURCE_DIR}/Linux_for_Tegra/rootfs/"
export KERNEL_HEADERS="${JETSON_SOURCE_DIR}/Linux_for_Tegra/source/kernel/kernel-jammy-src"
export INSTALL_MOD_PATH="${JETSON_SOURCE_DIR}/Linux_for_Tegra/rootfs/"

# Change to the kernel sources directory
pushd "${JETSON_SOURCE_DIR}/Linux_for_Tegra/source"

# Invoke Make commands
# Build the kernel
make clean
make KERNEL_DEF_CONFIG=oldconfig -C kernel
sudo -E make install -C kernel
cp "kernel/kernel-jammy-src/arch/arm64/boot/Image" "${JETSON_SOURCE_DIR}/Linux_for_Tegra/kernel/Image"

# Make NVIDIA Out Of Tree Modules
make modules
sudo -E make modules_install

pushd "${JETSON_SOURCE_DIR}/Linux_for_Tegra"
sudo ./tools/l4t_update_initrd.sh

# Build the DTBs
pushd "${JETSON_SOURCE_DIR}/Linux_for_Tegra/source"
make dtbs
cp "kernel-devicetree/generic-dts/dtbs/*" "${JETSON_SOURCE_DIR}/Linux_for_Tegra/kernel/dtb/"
