#!/bin/bash

cmake -B ./build -DARTINX_HUB_CAMERA=USB2 -DCMAKE_MAKE_PROGRAM=make -DCMAKE_C_COMPILER=gcc -DCMAKE_CXX_COMPILER=g++ -DCMAKE_BUILD_TYPE=Release -DCMAKE_TOOLCHAIN_FILE=/opt/vcpkg/scripts/buildsystems/vcpkg.cmake
cmake --build ./build --config Release -j 6
sudo bash ./scripts/setup-service.bash <robot_type>
