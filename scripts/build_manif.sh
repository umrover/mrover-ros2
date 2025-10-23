#!/usr/bin/bash

git submodule update --init deps/manif

pushd deps/manif || exit

# build and instal cmake
cmake . -B build
cmake --build build
sudo cmake --install build

popd || exit
