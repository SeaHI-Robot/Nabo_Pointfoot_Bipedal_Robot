#!/usr/bin/bash

cd ./nabo_core/build
cmake ..
make -j12
cd ..
sh copy_tools.sh
cd ../nabo_mujoco/build
cmake ..
make -j12
./main
