#!/usr/bin/env bash

set -euxo pipefail

# Determine the build profile and whether to report coverage
build_profile=RelWithDebInfo
run_coverage=false

while [[ "$#" -gt "0" ]]; do
	case "$1" in
		"Release" | "RelWithDebInfo" | "Debug")
			build_profile=$1 ;;
		"-c")
			# nothing should come after -c
			if [[ "$#" -gt 1 ]]; then
				echo "Usage ./test.sh [Release|RelWithDebInfo|Debug] [-c]"
				exit 1
			fi

			run_coverage=true ;;
		*)
			echo "Usage ./test.sh [Release|RelWithDebInfo|Debug] [-c]"
			exit 1 ;;
	esac
	shift
done

echo "Using build profile: $build_profile"

# Test in the colcon workspace, not the package
pushd ../..

if [[ ! -d "build/$build_profile" ]]; then
	echo "Build profile $build_profile not found. Please build first."
	exit 1
fi

export LLVM_PROFILE_FILE="coverage-%m.profraw"
export GTEST_COLOR=1

if [[ "$run_coverage" = true ]] ; then
	export PYTEST_ADDOPTS="--cov=$PWD/src/mrover --cov-report=html:$PWD/build/$build_profile/mrover/pytest_cov/"
fi

# invoke colcon
COLCON_EXTENSION_BLOCKLIST=colcon_core.event_handler.desktop_notification colcon test \
	--event-handlers console_direct+ \
	--build-base "build/$build_profile" \
	--install-base "install/$build_profile" \
	--ctest-args -R test_

if [[ "$run_coverage" = false ]] ; then
	exit 0
fi

# Generate C++ coverage
llvm-profdata merge -sparse build/$build_profile/mrover/*.profraw -o build/$build_profile/mrover/merged.profdata

test_binaries=($(find build/$build_profile/mrover -type f -executable -name "test*"))
PRIMARY="${test_binaries[0]}"
OBJECTS=""
for obj in "${test_binaries[@]:1}"; do
	OBJECTS="$OBJECTS -object $obj"
done

llvm-cov show $PRIMARY $OBJECTS -instr-profile=build/$build_profile/mrover/merged.profdata -format=html -output-dir=build/$build_profile/mrover/coverage_html src/mrover

echo "C++ coverage report: file://$PWD/build/$build_profile/mrover/coverage_html/index.html"

echo "Python coverage report: file://$PWD/build/$build_profile/mrover/pytest_cov/index.html"
