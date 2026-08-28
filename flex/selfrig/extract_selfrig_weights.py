#!/usr/bin/env python3
# -*- coding: utf-8 -*-
"""
slap_selfrig_data.json 生成器 —— 在 Blender (bpy) 内跑: 从蒙皮网格重采 10000 点(带多骨权重) + 骨架 rest。

在打开 assets/FBX/pikachu_skin_self_rig.fbx 的 Blender 会话里执行:
    bpy  中运行本文件, 或 Blender MCP 载入。

原理:
  body 网格 Cube.003 的顶点组 = 每顶点对多骨骼的蒙皮权重(bone, w)。
  网格三角面上按面积均匀采 N_firstpoints, 每个采样点落在某三角面内,
  用重心坐标 (wA,wB,wC) 把三个顶点的蒙皮权重插值到采样点 → 得到可跟随骨骼的表面点云。
  比之前的「最近骨分配」精确: 关节处平滑过渡, 真 LBS。

输出 /tmp/selfrig_data.json (供 three.js 驱动查看器):
  { "bones": [{name,parent,pos(x,y,z)}],      # 活动 14 骨, Y-up(高)
    "pts":   [[x,y,z],...],                        # 10000 个 rest 点, Y-up
    "weights": [{bid:[boneIdx], w:[w]}...] 或 存整数量化                # 每点可变长 top-4 骨
  }
"""
import json
import os
import numpy as np

ARM = "骨架"                 # armature 对象名
MESH = "Cube.003"            # 蒙皮网格对象名
N_POINTS = 10000             # 目标点云点数
TOP_K = 4                    # 每点保留骨骼数(真蒙皮 4 骨已覆盖 ~99% 权重)
OUT = os.path.join(os.path.dirname(os.path.abspath(__file__)), "selfrig_data.json")
SEED = 7


def load_mesh_weights(mesh):
    """返回 (pos: Vx3 顶点位置, weights: Vx[ (boneIdx, w) ] 稀疏列表, vg_name: gid->name)。"""
    vgroups = {g.index: g.name for g in mesh.vertex_groups}
    # 顶点 -> (gid, w)
    vw = []
    for v in mesh.data.vertices:
        rows = []
        for el in v.groups:
            if el.weight > 1e-4:
                rows.append((el.group, float(el.weight)))
        vw.append(rows)
    pos = np.array([list(v.co) for v in mesh.data.vertices], dtype=np.float64)
    return pos, vw, vgroups


def tessellate(mesh):
    """把 mesh.polygons 拆成三角形, 返回 [(v0,v1,v2),(...)] 与面积。"""
    tris, areas = [], []
    for p in mesh.data.polygons:
        idx = p.vertices
        n = len(idx)
        # 扇形拆三角
        for i in range(1, n - 1):
            a, b, c = idx[0], idx[i], idx[i + 1]
            tris.append((int(a), int(b), int(c)))
    return tris, areas


def sample_with_weights(pos, vw, mesh, n_points, top_k=4, seed=SEED):
    rng = np.random.default_rng(seed)
    lo = pos.min(0)
    # 三角索引按面积
    verts = np.array([list(v.co) for v in mesh.data.vertices], np.float64)
    tris = []
    for p in mesh.data.polygons:
        idx = p.vertices
        for i in range(1, len(idx) - 1):
            tris.append((int(idx[0]), int(idx[i]), int(idx[i + 1])))
    tris = np.array(tris)
    a, b, c = verts[tris[:, 0]], verts[tris[:, 1]], verts[tris[:, 2]]
    areas = 0.5 * np.linalg.norm(np.cross(b - a, c - a), axis=1)
    areas = np.maximum(areas, 1e-12)
    area_w = areas / areas.sum()
    chosen = rng.choice(len(tris), size=n_points, p=area_w)

    pts = np.empty((n_points, 3))
    outw = np.empty((n_points, top_k), np.int32)   # bone id
    outv = np.empty((n_points, top_k))            # weight
    outk = np.empty((n_points,), np.int32)         # 实际骨数

    for i, ti in enumerate(chosen):
        A, B, C = a[ti], b[ti], c[ti]
        u, v = rng.random(), rng.random()
        if u + v > 1:
            u, v = 1 - u, 1 - v
        wA, wB, wC = 1 - u - v, u, v
        pts[i] = wA * A + wB * B + wC * C
        # 三顶点骨骼权重并集
        acc = {}
        for (wd, vtx) in ((wA, tris[ti, 0]), (wB, tris[ti, 1]), (wC, tris[ti, 2])):
            for gid, w in vw[vtx]:
                if gid not in acc:
                    acc[gid] = 0.0
                acc[gid] += wd * w
        # 归一 + 保留 top_k
        items = sorted(acc.items(), key=lambda kv: -kv[1])[: top_k]
        s = sum(v for _, v in items)
        if s < 1e-9:
            continue
        for k, (gid, w) in enumerate(items):
            outw[i, k], outv[i, k], outk[i] = gid, w / s, len(items)
    return pts, outw, outv, outk


def main():
    import bpy
    print("main: 找对象")
    arm = bpy.data.objects[ARM] if ARM in bpy.data.objects else next(
        o for o in bpy.data.objects if o.type == 'ARMATURE')
    mesh = bpy.data.objects[MESH] if MESH in bpy.data.objects else None
    if mesh is None:
        mesh = next(o for o in bpy.data.objects if o.type == 'MESH' and o.vertex_groups and o.name.startswith('Cube'))
    _extract(arm, mesh)


def full_run(fbx_path):
    """自包含: 导入 fbx + 定位骨架/蒙皮 + 抽取 + 写文件(单次调用, 不依赖 scene 状态)。"""
    import bpy
    # 清掉上次同名对象(避免 bpy 自动改名 Cube.00x)
    for nm in ('骨架', 'Cube.003', 'Cube'):
        if nm in bpy.data.objects:
            bpy.data.objects.remove(bpy.data.objects[nm], do_unlink=True)
    for o in bpy.data.objects:
        if o.type == 'ARMATURE':
            bpy.data.objects.remove(o, do_unlink=True)
        elif o.name.startswith('Sphere') and o.type == 'MESH':
            bpy.data.objects.remove(o, do_unlink=True)
    bpy.ops.import_scene.fbx(filepath=fbx_path, use_manual_orientation=False)
    arm = next(o for o in bpy.data.objects if o.type == 'ARMATURE')
    # 蒙皮 mesh: 选「顶点组名与骨架骨名重合最多」的网格。
    # 场景里可能混入 Rigify 的 body(DEF-* 顶点组, 与 14dof 骨架不匹配),
    # 不能用「只要有一个同名」判据, 否则误选; 按重合度取最匹配者。
    bone_names = {b.name for b in arm.data.bones}
    candidates = [o for o in bpy.data.objects if o.type == 'MESH' and o.vertex_groups]
    mesh = max(candidates,
               key=lambda o: sum(1 for g in o.vertex_groups if g.name in bone_names))
    print("选中蒙皮网格:", mesh.name, "重合度", sum(1 for g in mesh.vertex_groups if g.name in bone_names))
    _extract(arm, mesh)


def _extract(arm, mesh):
    import os
    print("提取: 读顶点权重")
    pos, vw, vgroups = load_mesh_weights(mesh)
    print("采样 %d 点(面积均匀+重心插值权重)..." % N_POINTS)
    pts, outw, outv, outk = sample_with_weights(pos, vw, mesh, N_POINTS, TOP_K, SEED)
    print("写 JSON")
    # 骨: 剔除 *_end, 保留活动骨; 层级沿用 fbx
    bones = [b for b in arm.data.bones if not b.name.endswith('_end')]
    bone_idx = {b.name: i for i, b in enumerate(bones)}
    # rest 位置(head_local), 转 Y-up: (x,y,z)->(x,z,y)
    bdat = []
    for i, b in enumerate(bones):
        h = np.array(b.head_local)
        parent = bone_idx.get(b.parent.name, -1) if b.parent and b.parent.name in bone_idx else -1
        bdat.append({"name": b.name, "parent": parent,
                     "pos": [round(float(h[0]), 4), round(float(h[2]), 4), round(float(h[1]), 4)]})
    # 点 Y-up
    ptsY = pts[:, [0, 2, 1]]
    # 权重: vgroups gid -> bone name => bone_idx
    gid2bone = {gid: bone_idx[name] for gid, name in vgroups.items() if name in bone_idx}
    weights = []
    for i in range(N_POINTS):
        k = int(outk[i])
        pair = []
        for p in range(k):
            gid = int(outw[i, p])
            bi = gid2bone.get(gid)
            if bi is None:
                continue
            pair.append([bi, round(float(outv[i, p]), 4)])
        weights.append(pair)

    data = {"bones": bdat,
            "pts": [[round(float(x), 4) for x in row] for row in ptsY],
            "weights": weights,
            "n_points": N_POINTS,
            "note": "from Blender mesh vertices' vertex-group skin weights, barycentric-sampled on triangles; Y-up; per-point top-%d bones" % TOP_K}
    with open(OUT, "w", encoding="utf-8") as f:
        json.dump(data, f, ensure_ascii=False)
    print("已写", OUT, " %.2f MB" % (os.path.getsize(OUT) / 1048576),
          " 点", N_POINTS, " 骨", len(bones))


if __name__ == "__main__":
    main()