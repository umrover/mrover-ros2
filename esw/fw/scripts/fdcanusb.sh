#!/usr/bin/env bash

set -euo pipefail

RED="\e[1;31m"
BLUE="\e[1;34m"
NC="\e[0m"

ESW_ROOT="$(dirname "$(dirname "$(realpath "$0")")")"
SCRIPT_NAME=$(basename "$0")
CACHE_DIR="$ESW_ROOT/tools/fdcanusb"
SOURCE_URL="https://raw.githubusercontent.com/mjbots/fdcanusb/4361dc321262693fa2548d8d4245fe1161b65f39/sw/fdcanusb_daemon.cc"
UDEV_SOURCE_URL="https://raw.githubusercontent.com/mjbots/fdcanusb/refs/heads/master/70-fdcanusb.rules"
SOURCE_FILE="${CACHE_DIR}/fdcanusb_daemon.cc"
BINARY="${CACHE_DIR}/fdcanusb_daemon"
IFACE=""

usage() {
    cat <<EOF
Usage: sudo $SCRIPT_NAME --net <vcan-network>

options:
  -n, --net <vcan-network>      set vcan interface
  -h, --help                    show this help message
EOF
    exit 1
}

# enforce sudo
if [ "$EUID" -ne 0 ]; then
    printf "%b\n" "${RED}✗ error: run this script as root${NC}"
    usage
fi

# parse parameters
while [[ $# -gt 0 ]]; do
    case "$1" in
        -n|--net)       IFACE="$2"; shift 2 ;;
        -h|--help)      usage ;;
        *)              printf "%b\n" "${RED}✗ unknown option: $1${NC}"; usage ;;
    esac
done

# verify network interface provided
if [ -z "$IFACE" ]; then
    usage
fi

# verify curl is installed
if ! command -v curl >/dev/null 2>&1; then
    printf "%b\n" "${RED}✗ error: curl not installed${NC}"
    exit 1
fi

# create cache directory
if [ ! -d "$CACHE_DIR" ]; then
    mkdir -p "$CACHE_DIR"
    printf "%b\n" "${BLUE}created cache directory at ${CACHE_DIR}${NC}"
fi

# download fdcanusb daemon
if [ ! -f "$SOURCE_FILE" ]; then
    curl -L "$SOURCE_URL" -o "$SOURCE_FILE" || {
        printf "%b\n" "${RED}$✗ error: could not download source from $SOURCE_URL${NC}"
        exit 1
    }
    printf "%b\n" "${BLUE}downloaded fdcanusb daemon source to $SOURCE_FILE${NC}"
fi

# build fdcanusb daemon
if [ ! -f "$BINARY" ]; then
    g++ -Wall -g -O2 "$SOURCE_FILE" -o "$BINARY"
    printf "%b\n" "${BLUE}built fdcanusb daemon${NC}"
fi

# install and load udev rules
if [ ! -f "/etc/udev/rules.d/70-fdcanusb.rules" ] && [ ! -f "/lib/udev/rules.d/70-fdcanusb.rules" ]; then
    curl -L "$UDEV_SOURCE_URL" -o "/etc/udev/rules.d/70-fdcanusb.rules" || {
        printf "%b\n" "${RED}✗ error: could not download source from $UDEV_SOURCE_URL${NC}"
        exit 1
    }
    printf "%b\n" "${BLUE}installing udev rules${NC}"
    udevadm control --reload-rules
    udevadm trigger --subsystem-match=tty
    sleep 1
    printf "%b\n" "${BLUE}installed udev rules for fdcanusb${NC}"
fi

# verify fdcanusb exists
if [ ! -e "/dev/fdcanusb" ]; then
    printf "%b\n" "${RED}✗ error: fdcanusb device not found${NC}"
    usage
fi

# setup virtual CAN network interface
if ! ip link show "$IFACE" >/dev/null 2>&1; then
    printf "%b\n" "${BLUE}creating vcan interface, you may be prompted for your password $IFACE${NC}"
    ip link add name "$IFACE" type vcan || true
    printf "%b\n" "${BLUE}created $IFACE${NC}"
fi

# start vcan network
ip link set "$IFACE" up

# run fdcanusb daemon in foreground
"${BINARY}" -F -v "/dev/fdcanusb" "$IFACE"

