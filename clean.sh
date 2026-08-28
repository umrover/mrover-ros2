#!/usr/bin/env bash

set -euxo pipefail

# remove the cached esw libraries
if [[ "${1:-}" == "--esw" ]]; then
    rm -rf deps/.fetchcontent/esw
fi

pushd ../..

rm -rf build install log
