#!/bin/bash

BLENDER="/home/finnox/blender-5.2.0-linux-x64/blender"
BLEND_FILE="/home/finnox/Pikachu/PikachuRobot/pikachu_mocap/assets/pikachu_role.blend"

# 检查 Blender 程序是否存在
if [ ! -f "$BLENDER" ]; then
    echo "错误：Blender 可执行文件未找到: $BLENDER"
    exit 1
fi

# 检查 blend 文件是否存在
if [ ! -f "$BLEND_FILE" ]; then
    echo "错误：blend 文件未找到: $BLEND_FILE"
    exit 1
fi  

# 启动 Blender 并打开文件（如果需要在后台运行，可以加上 &）
"$BLENDER" "$BLEND_FILE"