#!/usr/bin/env bash

set -euxo pipefail

# remove the cached mrover_can library
if [[ "${1:-}" == "--esw" ]]; then
    rm -rf .cache/esw
fi

pushd ../..

rm -rf build install log
