#!/usr/bin/env bash

set -euxo pipefail

# Build in the colcon workspace, not the package
pushd ../..

# Set C/C++ compilers
export CC=clang
export CXX=clang++

# Set CUDA compilers
export CUDAHOSTCXX=g++-9
export CUDACXX=/usr/local/cuda-12.2/bin/nvcc

# TODO (ali): add build configs for debug vs release
colcon build \
	--cmake-args -G Ninja -W no-dev -DCMAKE_BUILD_TYPE=RelWithDebInfo \
	--symlink-install \
	--event-handlers console_direct+ \
	"$@"
