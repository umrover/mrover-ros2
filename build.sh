#!/usr/bin/env bash

set -euxo pipefail

# Build in the colcon workspace, not the package
pushd ../..

# TODO (ali): add build configs for debug vs release
colcon build \
	--cmake-args -G Ninja -W no-dev -DCMAKE_BUILD_TYPE=RelWithDebInfo -DCMAKE_TOOLCHAIN_FILE=cmake/toolchain.cmake \
	--symlink-install \
	--event-handlers console_direct+ \
	"$@"

rm -rf ../../build/mrover/.cmake/api
