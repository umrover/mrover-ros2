#!/usr/bin/env bash

set -euxo pipefail

# determine the build profile
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

# Build in the colcon workspace, not the package
pushd ../..

# Set C/C++ compilers
export CC=clang
export CXX=clang++

# Set CUDA compilers
export CUDAHOSTCXX=g++-9
export CUDACXX=/usr/local/cuda-12/bin/nvcc

# invoke colcon
colcon build \
	--cmake-args -G Ninja -W no-dev -DCMAKE_BUILD_TYPE="$build_profile" \
	--symlink-install \
	--event-handlers console_direct+ \
	--build-base "build/$build_profile" \
	--install-base "install/$build_profile"

rm -rf ../../build/mrover/.cmake/api
