#!/usr/bin/env bash

set -euxo pipefail

# Build in the colcon workspace, not the package
pushd ../..

# set C/CXX compilers
export CC=clang
export CXX=clang++ 

# TODO (ali): add build configs for debug vs release
colcon build \
	--cmake-args -G Ninja -W no-dev -DCMAKE_BUILD_TYPE=RelWithDebInfo \
	--symlink-install \
	--event-handlers console_direct+ \
	$@
