#!/bin/sh
rm -drf build
mkdir build
cd build
cmake ..
make
make install
