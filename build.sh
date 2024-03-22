#!/bin/bash

cd build

cmake ..

make -j2

./stereo_VO $1