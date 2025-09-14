#!/usr/bin/zsh

git submodule update --init deps/manif

pushd deps/manif

# build and instal cmake
cmake . -B build
cmake --build build
sudo cmake --install build

popd
