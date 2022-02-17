#!/bin/sh

cd ./build

# cmake -DCMAKE_BUILD_TYPE=Debug \
#       -DCMAKE_C_COMPILER=/usr/bin/arm-linux-gnueabihf-gcc \
#       -DCMAKE_CXX_COMPILER=/usr/bin/arm-linux-gnueabihf-g++ \
#       ..

# cmake -DCMAKE_BUILD_TYPE=Release \
#       -DCMAKE_C_COMPILER=aarch64-linux-gnu-gcc \
#       -DCMAKE_CXX_COMPILER=aarch64-linux-gnu-g++ \
#       ..

cmake -DCMAKE_BUILD_TYPE=Release \
      -DCMAKE_C_COMPILER=gcc \
      -DCMAKE_CXX_COMPILER=g++ \
      ..

make -j4
