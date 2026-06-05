#!/usr/bin/env bash

set -euxo pipefail

build_profile=RelWithDebInfo

if [[ "$#" -ne "0" ]]; then
    if [[ "$#" -eq "1" && ( "$1" == "Release" || "$1" == "RelWithDebInfo" || "$1" == "Debug" ) ]]; then
        build_profile=$1
    else
        echo "Usage ./build.sh [Release|RelWithDebInfo|Debug]"
        exit 1
    fi
fi

echo "Using build profile: $build_profile"

# Cap parallel jobs to avoid OOM. The simulator's TUs (Bullet + Eigen + Boost +
# ImGui + webgpu PCH) can each peak well above 2 GB, so building N cores at once
# easily exhausts RAM on weaker machines.
#
# Default to a SINGLE job (safe everywhere, including low-RAM laptops). Override
# on a strong machine with: MROVER_BUILD_JOBS=N ./build.sh
parallel_jobs="${MROVER_BUILD_JOBS:-1}"
if ! [[ "${parallel_jobs}" =~ ^[1-9][0-9]*$ ]]; then
    echo "MROVER_BUILD_JOBS must be a positive integer, got '${parallel_jobs}'" >&2
    exit 1
fi
echo "Parallel jobs: ${parallel_jobs} (override with MROVER_BUILD_JOBS=N)"

if [ -n "${PIXI_PROJECT_ROOT:-}" ]; then
    bash tools/setup_dawn.sh

    CMAKE_BUILD_PARALLEL_LEVEL="${parallel_jobs}" \
    COLCON_EXTENSION_BLOCKLIST=colcon_core.event_handler.desktop_notification \
        colcon build \
        --parallel-workers "${parallel_jobs}" \
        --cmake-args -G Ninja -W no-dev \
            -DCMAKE_BUILD_TYPE="${build_profile}" \
            -DMROVER_PORTABLE=ON \
            -DCMAKE_PREFIX_PATH="${CONDA_PREFIX}" \
        --symlink-install \
        --event-handlers console_direct+

    ln -sf "$(pwd)/build/mrover/compile_commands.json" "$(pwd)/compile_commands.json"
else
    pushd ../..

    if [ -x /usr/local/cuda-12/bin/nvcc ]; then
        export CUDAHOSTCXX=g++-9
        export CUDACXX=/usr/local/cuda-12/bin/nvcc
    fi

    CMAKE_BUILD_PARALLEL_LEVEL="${parallel_jobs}" \
    COLCON_EXTENSION_BLOCKLIST=colcon_core.event_handler.desktop_notification \
        colcon build \
        --parallel-workers "${parallel_jobs}" \
        --cmake-args -G Ninja -W no-dev \
            -DCMAKE_BUILD_TYPE="${build_profile}" \
        --symlink-install \
        --event-handlers console_direct+ \
        --build-base "build/${build_profile}" \
        --install-base "install/${build_profile}"

    rm -rf "$(pwd)/build/${build_profile}/mrover/.cmake/api"
    ln -sf "$(pwd)/build/${build_profile}/compile_commands.json" \
           "$(pwd)/src/mrover/compile_commands.json"
fi
