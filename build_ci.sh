#!/usr/bin/env bash

set -euxo pipefail

REPO_ROOT=$(pwd)

# Build in the colcon workspace, not the package
pushd ../..

# set C/CXX compilers
export CC=clang
export CXX=clang++

# TODO (ali): add build configs for debug vs release
colcon build \
	--cmake-args -G Ninja -W no-dev \
		-D CMAKE_BUILD_TYPE=Release \
		-D MROVER_CI=ON \
		-D Dawn_DIR="${REPO_ROOT}/deps/dawn-prebuilt/lib/cmake/Dawn" \
	--symlink-install \
	--event-handlers console_direct+ \
	"$@"
