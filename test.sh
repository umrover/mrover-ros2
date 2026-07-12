#!/usr/bin/env bash

set -euxo pipefail

# determine the build profile
build_profile=RelWithDebInfo

if [[ "$#" -ne "0" ]]; then
	if [[ ("$#" -eq "1" || "$#" -eq "2") && ( "$1" == "Release" || "$1" == "RelWithDebInfo" || "$1" == "Debug" ) ]]; then
		build_profile=$1
	else
		echo "Usage ./build.sh [Release|RelWithDebInfo|Debug] [-c]"
		exit 1
	fi
fi

run_coverage=false
OPTIND=2 # skip first arg (build profile)
while getopts ":c" opt; do
	case "${opt}" in
		c)
			run_coverage=true;;
		\?)
			echo "Error: Invalid option"
			echo "Usage ./build.sh [Release|RelWithDebInfo|Debug] [-c]"
			exit 1;;
	esac
done

echo "Using build profile: $build_profile"

# Test in the colcon workspace, not the package
pushd ../..

export LLVM_PROFILE_FILE="coverage-%m.profraw"

if [ "$run_coverage" = true ] ; then
	export PYTEST_COV=1
fi

# invoke colcon
COLCON_EXTENSION_BLOCKLIST=colcon_core.event_handler.desktop_notification colcon test \
	--event-handlers console_direct+ \
	--build-base "build/$build_profile" \
	--ctest-args -R test_

if [ "$run_coverage" = false ] ; then
	exit 0
fi

# generate C++ coverage
llvm-profdata merge -sparse build/$build_profile/mrover/*.profraw -o build/$build_profile/mrover/merged.profdata

test_binaries=($(find build/$build_profile/mrover -type f -executable -name "test*"))
PRIMARY="${test_binaries[0]}"
OBJECTS=""
for obj in "${test_binaries[@]:1}"; do
	OBJECTS="$OBJECTS -object $obj"
done

llvm-cov show $PRIMARY $OBJECTS -instr-profile=build/$build_profile/mrover/merged.profdata -format=html -output-dir=build/$build_profile/mrover/coverage_html src/mrover

echo "C++ coverage report: file://$PWD/build/$build_profile/mrover/coverage_html/index.html"

echo "Python coverage report: file://$PWD/build/$build_profile/mrover/pytest_cov/navigation_test_suite/coverage.html/index.html"
