#!/bin/bash
# CREDIT: https://github.com/AastaNV/JEP/blob/master/script/install_opencv4.10.0_Jetpack6.1.sh

version="4.14.0"
folder="opencv-workspace"

set -e

cd /tmp

echo "------------------------------------"
echo "** Install requirement (1/4)"
echo "------------------------------------"
sudo apt-get update
sudo apt-get install -y build-essential cmake git libgtk2.0-dev pkg-config libavcodec-dev libavformat-dev libswscale-dev
sudo apt-get install -y libgstreamer1.0-dev libgstreamer-plugins-base1.0-dev python3.12-dev python3-numpy
sudo apt-get install -y libtbb12 libtbb-dev libjpeg-dev libpng-dev libtiff-dev libv4l-dev v4l-utils qv4l2
sudo apt-get install -y curl

echo "------------------------------------"
echo "** Download opencv ${version} (2/4)"
echo "------------------------------------"
mkdir -p $folder
cd ${folder}
curl -L https://github.com/opencv/opencv/archive/${version}.zip -o opencv-${version}.zip
curl -L https://github.com/opencv/opencv_contrib/archive/${version}.zip -o opencv_contrib-${version}.zip
unzip opencv-${version}.zip
unzip opencv_contrib-${version}.zip
rm opencv-${version}.zip opencv_contrib-${version}.zip
cd opencv-${version}/

echo "------------------------------------"
echo "** Build opencv ${version} (3/4)"
echo "------------------------------------"
mkdir -p release
cd release/
cmake -D WITH_CUDA=ON -D WITH_CUDNN=ON -D CUDA_ARCH_BIN="8.7" -D CUDA_ARCH_PTX="" -D OPENCV_GENERATE_PKGCONFIG=ON -D OPENCV_EXTRA_MODULES_PATH=../../opencv_contrib-${version}/modules -D WITH_GSTREAMER=ON -D WITH_LIBV4L=ON -D BUILD_opencv_python3=ON -D BUILD_TESTS=OFF -D BUILD_PERF_TESTS=OFF -D BUILD_EXAMPLES=OFF -D CMAKE_BUILD_TYPE=RELEASE -D CMAKE_INSTALL_PREFIX=/usr/local -D CMAKE_CXX_STANDARD=17 -D CMAKE_CXX_STANDARD_REQUIRED=ON -D CUDA_NVCC_FLAGS="-std=c++17" ..
make "-j$(nproc)"

echo "------------------------------------"
echo "** Install opencv ${version} (4/4)"
echo "------------------------------------"
sudo make install
cat >> ~/.bashrc <<'EOF'
export LIBRARY_PATH=/usr/local/lib${LIBRARY_PATH:+:$LIBRARY_PATH}
export CPLUS_INCLUDE_PATH=/usr/local/include${CPLUS_INCLUDE_PATH:+:$CPLUS_INCLUDE_PATH}
export LD_LIBRARY_PATH=/usr/local/lib${LD_LIBRARY_PATH:+:$LD_LIBRARY_PATH}
export PYTHONPATH=/usr/local/lib/python3.12/site-packages${PYTHONPATH:+:$PYTHONPATH}
EOF
# shellcheck disable=SC1091
source "${HOME}/.bashrc"

echo "** Install opencv ${version} successfully"
echo "** Bye :)"
