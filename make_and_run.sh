#!/usr/bin/bash


# Check if "nabo_core/build" and "nabo_mujoco/build" exist
# ======================================================== #
if [ ! -d "nabo_core/build" ]; then
  mkdir -p nabo_core/build
fi

if [ ! -d "nabo_mujoco/build" ]; then
  mkdir -p nabo_mujoco/build
fi
# ======================================================== #


cd ./nabo_core/build
cmake ..
make -j12
cd ..
sh copy_tools.sh
cd ../nabo_mujoco/build
cmake ..
make -j12
./main
