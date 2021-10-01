#!/bin/bash

set -e -x

mkdir -p deps &> /dev/null

#Installing Pangolin
if [ ! -d deps/Pangolin ]; then
    git -C deps clone https://github.com/stevenlovegrove/Pangolin.git
fi
cmake ../ -DAVFORMAT_INCLUDE_DIR="" -DCPP11_NO_BOOST=ON -S deps/Pangolin -B deps/Pangolin/build
make -C deps/Pangolin/build -j$(nproc)

#Up to date OpenNI2
if [ ! -d deps/OpenNI2 ]; then
    git -C deps clone https://github.com/occipital/OpenNI2.git
fi
make -C deps/OpenNI2 -j$(nproc)

#Actually build ElasticFusion
cmake -S Core/src -B Core/build
make -C Core/build -j$(nproc)

cmake -S GPUTest/src -B GPUTest/build
make -C GPUTest/build -j$(nproc)

cmake -S GUI/src -B GUI/build
make -C GUI/build -j$(nproc)
