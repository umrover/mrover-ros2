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
# ImGui + webgpu PCH) can each peak well above 2 GB, so we derive a safe default
# from total RAM (assuming ~3 GB peak per job) rather than hardcoding 1.
if [[ -z "${MROVER_BUILD_JOBS:-}" ]]; then
    if page_size=$(sysctl -n hw.pagesize 2>/dev/null); then
        # macOS: sum free + inactive pages (inactive can be reclaimed immediately)
        free_pages=$(vm_stat | awk '/^Pages free:/{gsub(/\./,"",$3); print $3}')
        inactive_pages=$(vm_stat | awk '/^Pages inactive:/{gsub(/\./,"",$3); print $3}')
        avail_bytes=$(( (free_pages + inactive_pages) * page_size ))
        auto_jobs=$(( avail_bytes / (3 * 1024 * 1024 * 1024) ))
    elif [[ -r /proc/meminfo ]]; then
        # Linux: MemAvailable already accounts for reclaimable caches
        avail_kb=$(awk '/^MemAvailable:/{print $2}' /proc/meminfo)
        auto_jobs=$(( avail_kb / (3 * 1024 * 1024) ))
    else
        auto_jobs=1
    fi
    parallel_jobs=$(( auto_jobs < 1 ? 1 : auto_jobs ))
else
    parallel_jobs="${MROVER_BUILD_JOBS}"
fi
if ! [[ "${parallel_jobs}" =~ ^[1-9][0-9]*$ ]]; then
    echo "MROVER_BUILD_JOBS must be a positive integer, got '${parallel_jobs}'" >&2
    exit 1
fi
echo "Parallel jobs: ${parallel_jobs} (override with MROVER_BUILD_JOBS=N)"

if [ -x /usr/local/cuda-12/bin/nvcc ]; then
    export CUDAHOSTCXX=g++-9
    export CUDACXX=/usr/local/cuda-12/bin/nvcc
fi

# MROVER_CI=ON additionally enables clang-tidy static analysis and -Werror
mrover_ci="${MROVER_CI:-OFF}"

CMAKE_BUILD_PARALLEL_LEVEL="${parallel_jobs}" \
COLCON_EXTENSION_BLOCKLIST=colcon_core.event_handler.desktop_notification \
    colcon build \
    --parallel-workers "${parallel_jobs}" \
    --cmake-args -G Ninja -W no-dev \
        -DCMAKE_BUILD_TYPE="${build_profile}" \
        -DMROVER_CI="${mrover_ci}" \
    --symlink-install \
    --event-handlers console_direct+ \
    --build-base "build/${build_profile}" \
    --install-base "install/${build_profile}"

rm -rf "$(pwd)/build/${build_profile}/mrover/.cmake/api"
ln -sf "$(pwd)/build/${build_profile}/mrover/compile_commands.json" \
       "$(pwd)/compile_commands.json"
