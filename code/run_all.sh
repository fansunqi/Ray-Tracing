#!/usr/bin/env bash

if [[ ! -d build ]]; then
    echo "good"
else
    rm -rf build
fi
mkdir -p build
cd build
cmake ..
make -j
cd ..

mkdir -p output
# bin/PA1 pt testcases/ball.txt output/ball.bmp nodebug
bin/PA4 pt testcases/ball_and_moving_knife.txt output/ball_and_moving_knife.bmp nodebug
# bin/PA4 ppm testcases/glass.txt output/glass.bmp nodebug
# bin/PA4 ppm testcases/water.txt output/water.bmp nodebug
# bin/PA4 ppm testcases/rabbit.txt output/rabbit.bmp nodebug 
