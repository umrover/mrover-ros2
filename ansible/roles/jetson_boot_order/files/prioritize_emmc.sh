#!/usr/bin/env bash

BOOT_ID=$(sudo efibootmgr | grep -i 'emmc' | awk '{print $1}' | sed 's/Boot//;s/\*//')
BOOT_ORDER=$(sudo efibootmgr | grep -i "BootOrder" | sed "s/$BOOT_ID,//" | sed "s/BootOrder: /$BOOT_ID,/")
if [ -n "$BOOT_ID" ]; then
    sudo efibootmgr -o "$BOOT_ORDER"
else 
    echo "eMMC Jetson boot priority elevation failed"
fi