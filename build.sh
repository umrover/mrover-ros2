#!/usr/bin/env bash

set -euxo pipefail

# Build in the colcon workspace, not the package
pushd ../..

export CC=clang-18
export CXX=clang++-18

colcon build \
	--cmake-args -G Ninja \
	--cmake-args -W no-dev \
	--symlink-install \
	$@
