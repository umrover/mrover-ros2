#!/usr/bin/env bash

set -euxo pipefail

build_profile=RelWithDebInfo

if [[ "$#" -ne "0" ]]; then
    if [[ "$#" -eq "1" && ("$1" == "Release" || "$1" == "RelWithDebInfo" || "$1" == "Debug") ]]; then
        build_profile=$1
    else
        echo "Usage ./build.sh [Release|RelWithDebInfo|Debug]"
        exit 1
    fi
fi

echo "Using build profile: $build_profile"

if [ -n "${PIXI_PROJECT_ROOT:-}" ]; then
    # portable environment
    os_cmake_args=()
    if [[ "$(uname)" == "Darwin" ]]; then
        macos_sysroot=$(xcrun --sdk macosx --show-sdk-path)
        os_cmake_args=("-DCMAKE_OSX_SYSROOT=${macos_sysroot}" "-DMROVER_BUILD_ESW=OFF")
    else
        # conda's pkg-config wrapper runs the GCC-only
        os_cmake_args=("-DPKG_CONFIG_EXECUTABLE=${CONDA_PREFIX}/bin/pkg-config.bin")
        os_cmake_args+=("-DCMAKE_DISABLE_FIND_PACKAGE_ZED=ON" "-DCMAKE_DISABLE_FIND_PACKAGE_CUDA=ON" "-DMROVER_BUILD_ESW=OFF")
        os_cmake_args+=("-DCMAKE_C_COMPILER=${CONDA_PREFIX}/bin/clang" "-DCMAKE_CXX_COMPILER=${CONDA_PREFIX}/bin/clang++")
    fi

    COLCON_EXTENSION_BLOCKLIST=colcon_core.event_handler.desktop_notification colcon build \
        --cmake-args -G Ninja -W no-dev \
        -DCMAKE_BUILD_TYPE="${build_profile}" \
        -DCMAKE_PREFIX_PATH="${CONDA_PREFIX}" \
        "${os_cmake_args[@]}" \
        --symlink-install \
        --event-handlers console_direct+ \
        --packages-select mrover

    ln -sf "$(pwd)/build/mrover/compile_commands.json" "$(pwd)/compile_commands.json"
else
    # native environment

    # Build in the colcon workspace, not the package
    pushd ../..

    # Set C/C++ compilers
    export CC=clang
    export CXX=clang++

    # Set CUDA compilers
    export CUDAHOSTCXX=g++-9
    export CUDACXX=/usr/local/cuda/bin/nvcc

    # invoke colcon
    COLCON_EXTENSION_BLOCKLIST=colcon_core.event_handler.desktop_notification colcon build \
        --cmake-args -G Ninja -W no-dev -DCMAKE_BUILD_TYPE="$build_profile" \
        --symlink-install \
        --event-handlers console_direct+ \
        --build-base "build/$build_profile" \
        --install-base "install/$build_profile" \
        --packages-select mroverrr

    rm -rf "$(pwd)/build/$build_profile/mrover/.cmake/api"
    ln -sf "$(pwd)/build/$build_profile/mrover/compile_commands.json" "$(pwd)/src/mrover/compile_commands.json"
fi
