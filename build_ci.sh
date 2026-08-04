#!/usr/bin/env bash

# Reproduces exactly what CI runs. See .github/workflows/ci.yml.
set -euxo pipefail

export MROVER_CI=ON
exec ./build.sh Release
