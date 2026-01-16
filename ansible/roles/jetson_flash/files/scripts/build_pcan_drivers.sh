#!/usr/bin/env bash

set -euxo pipefail

PCAN_SOURCE_DIR="/tmp/peak-linux-driver-$1.$2.$3"

# Make Variables
CROSS_COMPILE="/tmp/aarch64--glibc--stable-2022.08-1/bin/aarch64-buildroot-linux-gnu-"
KERNEL_LOCATION="/tmp/jetson-flash/Linux_for_Tegra/source/kernel/kernel-jammy-src/"
PATH="$PATH:/tmp/aarch64--glibc--stable-2022.08-1/lib"

# Change to the kernel sources directory
pushd "${PCAN_SOURCE_DIR}/"

sudo make CROSS_COMPILE=${CROSS_COMPILE} KERNEL_LOCATION=${KERNEL_LOCATION} ARCH=arm64 PCAN_BASIC="" DESTDIR="../../jetson-flash/Linux_for_Tegra/rootfs" clean

sudo make CROSS_COMPILE=${CROSS_COMPILE} KERNEL_LOCATION=${KERNEL_LOCATION} ARCH=arm64 PCAN_BASIC="" DESTDIR="../../jetson-flash/Linux_for_Tegra/rootfs" netdev

sudo make CROSS_COMPILE=${CROSS_COMPILE} KERNEL_LOCATION=${KERNEL_LOCATION} ARCH=arm64 PCAN_BASIC="" DESTDIR="../../jetson-flash/Linux_for_Tegra/rootfs" install
