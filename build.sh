#!/usr/bin/env bash

set -euxo pipefail

# determine the build profile
build_profile=Debug

if [[ "$#" -ne "0" ]]; then
	if [[ "$#" -eq "1" && ( "$1" == "Release" || "$1" == "RelWithDebInfo" || "$1" == "Debug" ) ]]; then
		build_profile=$1
	else
		echo "Usage ./build.sh [Release|RelWithDebInfo|Debug]"
		exit 1
	fi
fi

echo "Using build profile: $build_profile"

# Build in the colcon workspace, not the package
pushd ../..

# Set C/C++ compilers
export CC=clang
export CXX=clang++

# Set CUDA compilers
export CUDAHOSTCXX=g++-9
export CUDACXX=/usr/local/cuda-12/bin/nvcc

# invoke colcon
COLCON_EXTENSION_BLOCKLIST=colcon_core.event_handler.desktop_notification colcon build \
	--cmake-args -G Ninja -W no-dev -DCMAKE_BUILD_TYPE="$build_profile" \
	--symlink-install \
	--event-handlers console_direct+ \
	--build-base "build/$build_profile" \
	--install-base "install/$build_profile"

rm -rf "$(pwd)/build/$build_profile/mrover/.cmake/api"

ln -sf "$(pwd)/build/$build_profile/compile_commands.json" "$(pwd)/src/mrover/compile_commands.json"

# same as .zshrc
readonly MROVER_ROS2_WS_PATH="$HOME/ros2_ws"

source_mrover_overlay(){
    build_profiles=("RelWithDebInfo" "Release" "Debug")

    target_file="/not/a/real/file"

    for profile in "${build_profiles[@]}"; do
        file="${MROVER_ROS2_WS_PATH}/install/${profile}/setup.zsh"

        if [ -f "${file}" ]; then
            if [[ "${file}" -nt "${target_file}" ]]; then
                target_file="${file}"
            fi
        fi
    done

    if [ -f "${target_file}" ]; then
        source "${target_file}"
    fi
}

source_mrover_overlay
