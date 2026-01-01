#!/bin/bash

set -euxo pipefail

git submodule update --init deps/manif

pushd deps/manif

python3 -m pip install .
