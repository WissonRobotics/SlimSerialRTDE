#!/bin/sh
rm -drf build
mkdir build
cd build
cmake ..
make -j12
make install
