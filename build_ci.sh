#!/usr/bin/env bash

set -euxo pipefail

export CC=clang
export CXX=clang++

colcon build \
	--cmake-args -G Ninja -W no-dev -D CMAKE_BUILD_TYPE=Release -D MROVER_CI=ON \
	--symlink-install \
	--event-handlers console_direct+ \
	"$@"
