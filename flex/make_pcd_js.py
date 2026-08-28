#!/usr/bin/env python3
# -*- coding: utf-8 -*-
"""
make_pcd_js.py — 为 three.js「带骨骼控制的点云模型」生成内嵌数据 (pcd_data.js)

把真 PCL 光追点云(pikachu_skin_real.pcd)均匀降采样到 ~1200 点,
用最近骨分配(bone assignment), 输出每骨 rest 坐标 + 层级(parent) + 每点所属骨,
打包成浏览器可直接引用的 window.PCD_DATA, 供 pcd_rig_viewer.html 做 LBS 骨骼驱动。

用法: conda run -n mocap python flex/make_pcd_js.py
"""
import os
import sys
import numpy as np
import json

HERE = os.path.dirname(os.path.abspath(__file__))
sys.path.insert(0, HERE)
import trimesh
from pikachu_cloud import GLB, load_pcd, load_skeleton, PER_BONE, SEED

PER_JS_BONE = 80        # 每骨在浏览器里保留的点数(总~1200)
OUT = os.path.join(HERE, "pcd_data.js")

# 层级渲染顺序(与 elastic_collision/build_mjcf 一致)
ORDER = ["骨架", "base_link", "hip_L", "hip_R", "hip_pitch_L", "hip_pitch_R",
         "hip_knee_L", "hip_knee_R", "hip_ankle_L", "hip_ankle_R", "head",
         "arm_L", "arm_R", "arm_pitch_L", "arm_pitch_R"]
ROOT = "骨架"


def parent_of(b):
    if b == ROOT:
        return None
    if b == "base_link":
        return ROOT
    if b in ("hip_L", "hip_R", "head", "arm_L", "arm_R"):
        return "base_link"
    if b.startswith("hip_pitch"):
        return "hip_" + "LR"["LR".index(b[-1])]
    if b.startswith("hip_knee"):
        return "hip_pitch_" + b[-1]
    if b.startswith("hip_ankle"):
        return "hip_knee_" + b[-1]
    if b.startswith("arm_pitch"):
        return "arm_" + b[-1]
    return ROOT


def _fit_points_to_bones(P, bones, origin):
    """把点云包围盒每轴仿射到骨架骨点包围盒, 做坐标对齐(LBS 才自然)。"""
    bp = np.array([bones[b][:3, 3] for b in bones])
    pmin, pmax = P.min(0), P.max(0)
    bmin, bmax = bp.min(0), bp.max(0)
    s = (bmax - bmin) / np.maximum(pmax - pmin, 1e-6)
    t = bmin - pmin * s
    Pf = P * s + t
    print("  对齐缩放 s=%s 平移 t=%s" % (s.round(3), t.round(3)))
    return Pf


def main():
    import sys
    from pikachu_cloud import assign_bones
    P = load_pcd(os.path.join(HERE, "models", "pikachu_skin_real.pcd"))
    glb = trimesh.load(GLB, force=None)
    bones = load_skeleton(glb)
    origin = bones["骨架"][:3, 3]
    P = _fit_points_to_bones(P, bones, origin)
    groups, _ = assign_bones(P, bones)

    rng = np.random.default_rng(SEED)
    # 只保留存在点的骨, 每骨抽 PER_JS_BONE
    used = [b for b in ORDER if len(groups.get(b, [])) > 0]
    pts_all, assign, bone_seq = [], [], []
    for b in used:
        g = groups[b]
        if len(g) > PER_JS_BONE:
            g = g[rng.choice(len(g), PER_JS_BONE, replace=False)]
        pts_all.append(g)
        assign.extend([len(bone_seq)] * len(g))
        bone_seq.append(b)
    Px = np.vstack(pts_all)
    origin = bones[ROOT][:3, 3].copy()
    Px = Px - origin
    print("点云 %d 点 -> %d 骨" % (len(Px), len(bone_seq)))

    # bones: 索引,name,parent索引,restPos(全局-origin)
    bone_map = {b: i for i, b in enumerate(bone_seq)}
    used_set = set(bone_seq)

    def resolve_parent(b):
        """沿祖先链上溯, 直到某个仍在 used 的骨(跳过零点的中间骨)。"""
        p = parent_of(b)
        seen = 0
        while p is not None and p not in used_set and seen < 8:
            p = parent_of(p)
            seen += 1
        return bone_map.get(p, -1) if p is not None else -1

    pdat = []
    for b in bone_seq:
        pdat.append({
            "id": bone_map[b],
            "name": b,
            "parent": resolve_parent(b),
            "pos": [round(float(v), 4) for v in (bones[b][:3, 3] - origin)],
        })

    data = {
        "pts": [[round(float(x), 4), round(float(y), 4), round(float(z), 4)]
                for x, y, z in Px],
        "assign": assign,
        "bones": pdat,
        "note": "每点绑定最近骨; 滑条旋转骨 -> 点云=LBS(offset 经骨世界矩阵)。"
    }
    with open(OUT, "w", encoding="utf-8") as f:
        f.write("window.PCD_DATA = %s;\n" % json.dumps(data, ensure_ascii=False))
    print("已写:", OUT, " %d 点 / %d 骨" % (len(Px), len(bone_seq)))


if __name__ == "__main__":
    main()