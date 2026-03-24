#!/bin/bash

# update/clone dynamixel_sdk
git submodule update --init deps/dynamixel_sdk

# make build directory
mkdir -p deps/dynamixel_sdk/dynamixel_sdk/build/
pushd deps/dynamixel_sdk/dynamixel_sdk/build/ || exit

# build & install sdk
cmake .. -DCMAKE_BUILD_TYPE=Release
make
sudo make install

# refresh linker cache (colcon has trouble finding sdk otherwise)
sudo ldconfig

# leave build dir
popd || exit

# remove build dir
rm -rf deps/dynamixel_sdk/dynamixel_sdk/build/
