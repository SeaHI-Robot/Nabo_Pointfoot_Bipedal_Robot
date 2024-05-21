#!/bin/bash

# 定义目标目录
target_dir="../nabo_mujoco/nabo/"

# 检查目标目录是否存在
if [ ! -d "$target_dir" ]; then
    # 如果不存在，则创建目录
    mkdir -p "$target_dir"
fi

# 使用cp命令复制）
cp ./nabo_output/libnabo.so ../nabo_mujoco/nabo/
cp ./nabo_output/nabo.h ../nabo_mujoco/nabo/
cp ./nabo_output/nabo_config.h ../nabo_mujoco/nabo/

echo "\n*********************************\nDone! files in nabo_output copied.\n*********************************\n"
