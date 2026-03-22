#!/bin/bash

git submodule update --init deps/dynamixel_sdk

pushd deps/dynamixel_sdk/ || exit

colcon build --packages-select dynamixel_sdk --paths dynamixel_sdk/