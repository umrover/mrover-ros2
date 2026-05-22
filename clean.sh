#!/usr/bin/env bash

set -euxo pipefail

if [ -n "${PIXI_PROJECT_ROOT:-}" ]; then
    rm -rf build install log
else
    pushd ../..
    rm -rf build install log
fi
