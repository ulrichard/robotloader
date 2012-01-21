#! /bin/sh

rm -rf build
mkdir build
cd build
cmake -G "CodeBlocks - Unix Makefiles" -DCMAKE_BUILD_TYPE:STRING=Release -DCMAKE_TOOLCHAIN_FILE:PATH=/usr/share/cmake-2.8/Modules/arexx_robot_arm_crosscompile.cmake ..

make

