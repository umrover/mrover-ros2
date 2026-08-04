#!/usr/bin/env bash

# Reproduces exactly what CI runs. See .github/workflows/ci.yml.
set -euxo pipefail

exec ./build.sh Release
