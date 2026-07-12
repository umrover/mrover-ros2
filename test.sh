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

export LLVM_PROFILE_FILE="coverage-%m.profraw"

# invoke colcon
COLCON_EXTENSION_BLOCKLIST=colcon_core.event_handler.desktop_notification colcon test \
	--event-handlers console_direct+ \
	--build-base "build/$build_profile" \
	--ctest-args -R test_

# generate coverage
llvm-profdata merge -sparse build/$build_profile/mrover/*.profraw -o build/$build_profile/mrover/merged.profdata

test_binaries=($(find build/$build_profile/mrover -type f -executable -name "test*"))
PRIMARY="${test_binaries[0]}"
OBJECTS=""
for obj in "${test_binaries[@]:1}"; do
	OBJECTS="$OBJECTS -object $obj"
done

llvm-cov show $PRIMARY $OBJECTS -instr-profile=build/$build_profile/mrover/merged.profdata -format=html -output-dir=build/$build_profile/mrover/coverage_html src/mrover

echo "Coverage report: file://$PWD/build/$build_profile/mrover/coverage_html/index.html"
