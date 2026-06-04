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

# Cap parallel jobs to avoid OOM. Dawn and Boost headers push each TU to ~2 GB peak.
# Use floor(MemAvailableKB / 2097152) i.e. one job per ~2 GB of available RAM,
# capped to the number of cores.
available_kb=$(awk '/MemAvailable/ {print $2}' /proc/meminfo 2>/dev/null || echo 8388608)
jobs_by_mem=$(( available_kb / 2097152 ))
jobs_by_cpu=$(nproc)
parallel_jobs=$(( jobs_by_mem < jobs_by_cpu ? jobs_by_mem : jobs_by_cpu ))
parallel_jobs=$(( parallel_jobs > 1 ? parallel_jobs : 1 ))
echo "Parallel jobs: ${parallel_jobs}"

if [ -n "${PIXI_PROJECT_ROOT:-}" ]; then
    bash tools/setup_dawn.sh

    CMAKE_BUILD_PARALLEL_LEVEL="${parallel_jobs}" \
    COLCON_EXTENSION_BLOCKLIST=colcon_core.event_handler.desktop_notification \
        colcon build \
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

    COLCON_EXTENSION_BLOCKLIST=colcon_core.event_handler.desktop_notification \
        colcon build \
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
