#!/usr/bin/env bash
# mesh2pcd.sh — 用 PCL 的 pcl_mesh2pcd 从皮卡丘网格生成点云, 再喂给 pikachu_cloud
set -e
ROOT="$(cd "$(dirname "$0")/.." && pwd)"
HERE="$(cd "$(dirname "$0")" && pwd)"

echo "[1/3] OBJ -> PLY(合并多材质 mesh, 供 pcl_mesh2pcd 输入)"
conda run -n mocap python "$HERE/pikachu_cloud.py" --ply-only

echo "[2/3] 生成点云计算(pcl_mesh2pcd, 缺失则用 trimesh 表面均匀采样 fallback)"
if command -v pcl_mesh2pcd 2>/dev/null || conda run -n pcltool pcl_mesh2pcd --help >/dev/null 2>&1; then
  # 真 PCL 光追采样: -level 球细分; -resolution 弧分层; -leaf_size 体素滤波控点数
  # conda-forge 的 pcl 跑可视化库需 LD_PRELOAD 系统 libX11 避免 XKeysymToString 未定义
  LD_PRELOAD=/usr/lib/x86_64-linux-gnu/libX11.so.6 conda run -n pcltool pcl_mesh2pcd \
    "$HERE/models/pikachu_skin_merged.ply" \
    "$HERE/models/pikachu_skin.pcd" \
    -leaf_size 0.006 2>&1 | tail -3
else
  echo "  (未装 PCL, 用 trimesh 表面均匀采样 : $HERE/models/pikachu_skin.pcd)"
  conda run -n mocap python "$HERE/pikachu_cloud.py" --sample-pcd 20000
fi

echo "[3/3] 生成本地点云+骨骼 MJCF (--pcd 走 PCL 点云)"
conda run -n mocap python "$HERE/pikachu_cloud.py" --build --pcd "$HERE/models/pikachu_skin.pcd"

echo "完成. 后续: conda run -n mocap python flex/pikachu_cloud.py --viewer --pcd flex/models/pikachu_skin.pcd"