#!/bin/bash

rm -rf build

mkdir build

cd build

cmake ..

make -j2

./stereo_VO $1