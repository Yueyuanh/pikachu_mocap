#!/usr/bin/env python3
# -*- coding: utf-8 -*-
"""
pikachu_cloud.py — 把皮卡丘 OBJ 皮肤转为点云, 绑定到 GLB 骨架, 导入 MuJoCo
做「骨骼驱动的点云模型」最小验证。

路线 (对应 flex/readme.md):
    1. 读 assets/Obj/pikachu_skin_self_rig.obj 的顶点 → 候选点云
    2. 读 assets/Glb/pikachu_skin_self_rig.glb 的骨架 rest 位姿(trimesh scene graph)
    3. 每个顶点按「最近骨架骨」分组 → 转成该骨局部系坐标
    4. 生成 MJCF: 一个 body 一根骨 + hinge 关节 + 若干小半径 sphere geom(点云)
       → 骨骼动, 其附着的点云跟着动; 骨架线用 mjRND_SKELETON 渲染可见

用法 (mocap conda 环境):
    python flex/pikachu_cloud.py --build            # 生成 models/pikachu_cloud.xml
    python flex/pikachu_cloud.py --render           # 离屏渲染: 摆腿动画 + 骨架线, 存 png 序列/合成图
    python flex/pikachu_cloud.py --viewer           # 打开 MuJoCo 实时窗口(右下 data 界面可拖 joint)
"""
import argparse
import os
import sys

import numpy as np

ROOT = os.path.dirname(os.path.dirname(os.path.abspath(__file__)))  # 仓库根
GLB = os.path.join(ROOT, "assets", "Glb", "pikachu_skin_self_rig.glb")
OBJ = os.path.join(ROOT, "assets", "Obj", "pikachu_skin_self_rig.obj")
HERE = os.path.dirname(os.path.abspath(__file__))
XML_PATH = os.path.join(HERE, "models", "pikachu_cloud.xml")
PNG_OUT = os.path.join(HERE, "pikachu_cloud_skel.png")

# 作为可驱动关节的骨(其余焊死)。格式: bone名 -> 关节标号(ftype hinge)、轴、范围(弧度)
DRIVEN = []
# 关节记录: (bone, pt_joint_axis_field) —— 由骨架自动拼接
CLOUD_R = 0.018          # 点云小球半径 m
PER_BONE = 220           # 每骨最多取的云点数(降采样,保证稳定/易看)
SEED = 7


# ── 1. 读骨架(trimesh scene graph) ───────────────────────────────────────
def load_skeleton(scene):
    import trimesh
    edges = scene.graph.to_edgelist()
    # 每个 edge: (parent, child, {matrix}) —— 相对父系
    children = {}
    rel = {}
    for p, c, m in edges:
        mm = m.get("matrix")  # 4x4 或 scale/params
        rel[str(c)] = np.asarray(mm, float) if mm is not None else np.eye(4)
        children.setdefault(str(p), []).append(str(c))
    bones = {}          # bone -> 全局 4x4
    names = ["骨架", "base_link", "head",
             "hip_R", "hip_pitch_R", "hip_knee_R", "hip_ankle_R",
             "hip_L", "hip_pitch_L", "hip_knee_L", "hip_ankle_L",
             "arm_R", "arm_pitch_R", "arm_L", "arm_pitch_L"]
    # 根: world -> 骨架
    root = "世界根"
    sum4 = {}
    def visit_frame(fr, acc):
        # 记录该 frame 全局
        if fr in names:
            bones[fr] = acc.copy()
        for k in children.get(fr, []):
            A = rel.get(k, np.eye(4))
            visit_frame(k, acc @ A) if A.shape == (4, 4) else None
    # world 节点通常是根
    roots = [str(n) for n in scene.graph.nodes if not any(str(c) == str(n) for _, c, _ in edges)]
    for rt in roots:
        visit_frame(rt, np.eye(4))
    for k, v in children.items():
        if k == "世界根" or any(str(x) == k for x, _, _ in edges):
            pass
    return bones


# ── 2. 读 OBJ 点云 ────────────────────────────────────────────────────────
def load_obj_cloud():
    import trimesh
    scene = trimesh.load(OBJ, force=None)
    ys = []
    geoms = scene.geometry if isinstance(scene, trimesh.Scene) else {"m": scene}
    for g in geoms.values():
        if hasattr(g, "vertices"):
            ys.append(g.vertices)
    if not ys:
        raise RuntimeError("OBJ 无顶点")
    P = np.vstack(ys)
    return P, scene


def load_pcd(path):
    """读取 PCL 的 pcl_mesh2pcd 输出的 ASCII PCD(只取 x y z)。"""
    pts = []
    with open(path, "r", encoding="utf-8", errors="replace") as f:
        data_started = False
        for line in f:
            s = line.strip()
            if not s:
                continue
            if s.upper().startswith("DATA"):
                data_started = True
                continue
            if not data_started:
                if s.startswith(("#", "VERSION", "FIELDS", "SIZE", "TYPE",
                                 "COUNT", "WIDTH", "HEIGHT", "VIEWPOINT", "POINTS")):
                    continue
                continue  # 其它 meta
            p = s.split()
            if len(p) >= 3:
                try:
                    pts.append([float(p[0]), float(p[1]), float(p[2])])
                except ValueError:
                    continue
    if not pts:
        raise RuntimeError("PCD %s 无数据(或非 ascii)" % path)
    return np.asarray(pts, dtype=float)


def write_pcd_ascii(path, P):
    """写 PCL 标准 ASCII PCD(x y z)。"""
    with open(path, "w", encoding="utf-8") as f:
        f.write("# .PCD v0.7 - Point Cloud Data file format\n")
        f.write("VERSION 0.7\nFIELDS x y z\nSIZE 4 4 4\nTYPE F F F\n"
                "COUNT 1 1 1\nVIEWPOINT 0 0 0 1 0 0 0\n")
        f.write("WIDTH %d\nHEIGHT 1\nPOINTS %d\nDATA ascii\n" % (len(P), len(P)))
        for p in P:
            f.write("%.6f %.6f %.6f\n" % (p[0], p[1], p[2]))
    print("[pcl] 已写 PCD:", path, " n=%d" % len(P))


def sample_cloud_trimesh(n_per_geom=20000, out=None):
    """trimesh 表面均匀采样(pcl_mesh2pcd 的等效 fallback), 返回 Nx3 点云。
    按三角面面积概率采样, 分布比纯顶点均匀得多。"""
    import trimesh
    from trimesh.sample import sample_surface
    scene = trimesh.load(OBJ, force=None)
    geoms = [g for g in (scene.dump() if isinstance(scene, trimesh.Scene) else [scene])
             if hasattr(g, "vertices")]
    samples = []
    for g in geoms:
        if len(getattr(g, "faces", [])) == 0:
            samples.append(np.asarray(g.vertices, float))
            continue
        pts, _ = sample_surface(g, n_per_geom, seed=SEED)
        samples.append(pts)
    P = np.vstack(samples)
    out = out or os.path.join(HERE, "models", "pikachu_skin.pcd")
    write_pcd_ascii(out, P)
    return P


def obj_to_ply(force=True):
    """用 trimesh 把 OBJ(可能多材质) 合并为单网格 PLY, 供 pcl_mesh2pcd 输入。"""
    import trimesh
    import trimesh.util as tu
    scene = trimesh.load(OBJ, force=None)
    if isinstance(scene, trimesh.Scene):
        geoms = [g for g in scene.dump() if hasattr(g, "vertices")]
    else:
        geoms = [scene]
    mesh = tu.concatenate(geoms) if len(geoms) > 1 else geoms[0]
    out = os.path.join(HERE, "models", "pikachu_skin_merged.ply")
    mesh.export(out)
    print("[pcl] 已合并 mesh ->", out, " (verts=%d faces=%d)" % (len(mesh.vertices), len(mesh.faces)))
    return out


# ── 3. 按最近骨分组 ───────────────────────────────────────────────────────
def assign_bones(P, bones):
    bone_names = list(bones.keys())
    bp = np.array([bones[b][:3, 3] for b in bone_names])     # 全局骨位置 (K,3)
    # 最近骨
    d2 = ((P[:, None, :] - bp[None, :, :]) ** 2).sum(-1)     # (N,K)
    closest = d2.argmin(1)
    groups = {}
    for i, b in enumerate(bone_names):
        idx = np.where(closest == i)[0]
        if len(idx) > PER_BONE:
            rng = np.random.default_rng(SEED)
            idx = rng.choice(idx, PER_BONE, replace=False)
        groups[b] = P[idx]
    counts = {b: len(groups[b]) for b in bone_names}
    return groups, counts


# ── 4. 生成 MJCF ─────────────────────────────────────────────────────────
def build_mjcf(bones, groups):
    bone_names = list(bones.keys())
    # 关节: 可驱动骨(腿三节L/R、臂下节、头) hinge; 其余焊死
    drive_bones = ["hip_pitch_L", "hip_pitch_R", "hip_knee_L", "hip_knee_R",
                   "hip_ankle_L", "hip_ankle_R",
                   "arm_pitch_L", "arm_pitch_R", "head"]
    # 骨架父子关系(以骨名+特殊 root)
    def parent_of(b):
        # 从 models 用户视角: 骨架->{base_link,hip_*} ; base_link->{head,arm_*}
        if b.startswith("hip_"):
            return "骨架"
        if b in ("base_link",):
            return "骨架"
        if b in ("head", "arm_L", "arm_R"):
            return "base_link"
        if b.startswith("arm_pitch_") or b.startswith("arm_"):
            return "arm_" + b[-1]
        return "世界根"
    # 全局->相对坐标系
    def global_pos(b):
        return bones[b][:3, 3]
    def rel_pos(child, parent):
        return bones[child][:3, 3] - bones[parent][:3, 3]

    lines = ['<?xml version="1.0" encoding="UTF-8"?>',
             '<mujoco model="pikachu_cloud">',
             '  <option timestep="0.004" gravity="0 0 -9.81"/>',
             '  <visual><global elevation="-35" azimuth="110" offwidth="800" offheight="800"/></visual>',
             '  <worldbody>']
    body2id = {}
    # 节点顺序: 先骨架, 再 base_link/hip_x, 再其子…… 简化: 按依赖拓扑拼
    order = ["骨架", "base_link", "hip_L", "hip_R",
             "hip_pitch_L", "hip_pitch_R", "hip_knee_L", "hip_knee_R",
             "hip_ankle_L", "hip_ankle_R",
             "head", "arm_L", "arm_R", "arm_pitch_L", "arm_pitch_R"]
    jname = {}  # bone -> dof 关节名
    for b in order:
        if b not in bones:
            continue
        p = parent_of(b)
        parent_body = body2id[p] if p in body2id else "world"
        lines.append(f'    <body name="{b}" pos="0 0 0">')
        # 相对父的位置: 用父系/全局差值
        # 为简化,把该骨全局位置设为 body pos,再由上层链安放
        lines.append(f'      <geom name="skel_{b}" type="sphere" size="0.008" '
                     f'rgba="1 1 1 0.2"/>')
        if b in drive_bones:
            jn = f"j_{b}"
            jname[b] = jn
            # 轴: 绕 x(可视骨沿 y 方向屈伸); 头顶简单范围
            axis = "1 0 0"
            lim = "0 1.2"
            lines.append(f'      <joint name="{jn}" type="hinge" axis="{axis}" '
                         f'range="{lim}" damping="0.1"/>')
        lines.append('    </body>')
        body2id[b] = b

    # 实际 MJCF 里让 body pos 用全局坐标,并给每个 body 放点云(局部=全局未旋转时用相对)
    # —— 上面占位,下面重新生成严谨版(替换 body 段)
    # (为避免复杂,改为:每个骨一个 body,pos=global pos(相对其自身系原点=全局), 点云用局部坐标写在 body 框体内)
    return "\n".join(lines)


def parent_of(b):
    if b.startswith("hip_"):
        return "骨架"
    if b == "base_link":
        return "骨架"
    if b in ("head", "arm_L", "arm_R"):
        return "base_link"
    if b.startswith("arm_pitch_"):
        return "arm_" + b[-1]
    return "世界根"


def finalize_mjcf(bones, groups):
    """生成 MJCF: body 树 + hinge 关节 + 点云 + 自绘骨链线(本地无 skel flag)。"""
    root_bone = "骨架"
    drive_bones = ["hip_pitch_L", "hip_pitch_R", "hip_knee_L", "hip_knee_R",
                   "hip_ankle_L", "hip_ankle_R", "arm_pitch_L", "arm_pitch_R", "head"]
    order = ["骨架", "base_link", "hip_L", "hip_R",
             "hip_pitch_L", "hip_pitch_R", "hip_knee_L", "hip_knee_R",
             "hip_ankle_L", "hip_ankle_R", "head", "arm_L", "arm_R",
             "arm_pitch_L", "arm_pitch_R"]
    bones_present = [b for b in order if b in bones]
    body_pos = {b: bones[b][:3, 3].copy() for b in bones_present}
    root_origin = body_pos[root_bone].copy()
    for b in list(body_pos):
        body_pos[b] = body_pos[b] - root_origin

    # 骨段(fromto): 父骨原点 -> 子骨相对父骨位移(无旋转,即子骨全局-父骨全局)
    bone_seg = {}
    for c in bones_present:
        p = parent_of(c)
        if p not in body_pos:
            continue                       # 跳 world/不存在的父
        delta = body_pos[c] - body_pos[p]
        if float(np.linalg.norm(delta)) < 0.004:
            continue
        bone_seg[c] = ('      <geom name="bone_%s" type="capsule" '
                       'fromto="0 0 0 %s" size="0.008" '
                       'rgba="0.80 0.80 0.85 0.7"/>'
                       % (c, " ".join("%.4f" % d for d in delta)))

    L = ['<?xml version="1.0"?>',
         '<mujoco model="pikachu_cloud">',
         '  <compiler angle="radian"/>',
         '  <option timestep="0.004" gravity="0 0 -9.81" integrator="RK4"/>',
         '  <visual><global elevation="-35" azimuth="110" '
         'offwidth="800" offheight="800"/></visual>',
         '  <worldbody>']
    L.append('    <geom name="floor" type="plane" size="3 3 0.1" '
             'pos="0 0 -0.05" rgba="0.42 0.46 0.5 1"/>')
    L.append('    <light name="key" pos="1.5 2.0 2.5" dir="-0.4 -0.5 -1" '
             'diffuse="0.95 0.95 0.95" ambient="0.75 0.75 0.75" castshadow="false"/>')

    def emit_body(b):
        pos = body_pos[b]
        L.append(f'    <body name="{b}" pos="{pos[0]:.4f} {pos[1]:.4f} {pos[2]:.4f}">')
        if b in drive_bones:
            jn = "j_" + b
            axis = "0 1 0" if b.startswith("head") else "1 0 0"
            lim = "-1.2 0.6" if not b.startswith("head") else "-0.7 0.7"
            L.append(f'      <joint name="{jn}" type="hinge" axis="{axis}" '
                     f'range="{lim}" damping="0.05"/>')
        # 关节点(白)
        L.append('      <site type="sphere" size="0.012" rgba="1 1 1 0.95"/>')
        # 若本骨有指向其子的骨段, 画在这(骨段几何放父 body)
        for c in bones_present:
            if parent_of(c) == b and c in bone_seg:
                L.append(bone_seg[c])
        # 点云小球(局部坐标)
        for p in groups.get(b, []):
            q = p - bones[b][:3, 3]
            L.append(f'      <geom type="sphere" size="{CLOUD_R}" '
                     f'pos="{q[0]:.4f} {q[1]:.4f} {q[2]:.4f}" '
                     f'rgba="0.95 0.55 0.05 1" contype="0" conaffinity="0"/>')
        L.append('    </body>')

    for b in bones_present:
        emit_body(b)
    L.append('    <camera name="cam" pos="0.5 -1.0 0.9" xyaxes="1 0 0 0 0.6 1"/>')
    L.append('  </worldbody>')
    L.append('</mujoco>')
    return "\n".join(L) + "\n"


# ── 5. 渲染 + 驱动演示 ───────────────────────────────────────────────────
def render_skel(model, data, camera="cam", w=800, h=800):
    """离屏渲染一帧(骨骼线用 MJCF 内自绘胶囊骨段), 返回 RGBA。"""
    from mujoco import Renderer
    rend = Renderer(model, w, h)
    rend.update_scene(data, camera)
    img = rend.render()
    return img


def make_demo_pose(model, data, t):
    """摆一段动画: 腿/臂/头正弦摆动, 让点云随骨移动。"""
    joints = {name: mujoco.mj_name2id(model, mujoco.mjtObj.mjOBJ_JOINT, "j_" + name)
              for name in ["hip_pitch_L", "hip_pitch_R", "hip_knee_L", "hip_knee_R",
                           "hip_ankle_L", "hip_ankle_R", "arm_pitch_L", "arm_pitch_R", "head"]}
    amp = {b: 0.8 for b in joints}
    freq = {b: 0.5 for b in joints}
    phase = {b: i * 0.5 for i, b in enumerate(joints)}
    def set_joint(jid):
        return None
    # 直接写 data.qpos
    return None


def main():
    ap = argparse.ArgumentParser(description="皮卡丘 OBJ 点云 + 骨骼 -> MuJoCo")
    ap.add_argument("--build", action="store_true", help="生成 MJCF xml")
    ap.add_argument("--render", action="store_true", help="离屏渲染摆腿动画+骨骼线")
    ap.add_argument("--viewer", action="store_true", help="实时窗口")
    ap.add_argument("--pcd", type=str, default=None,
                    help="用 PCL pcl_mesh2pcd 生成的 PCD 作为点云源(否则从 OBJ 顶点采样)")
    ap.add_argument("--ply-only", action="store_true",
                    help="仅把 OBJ 合并导出为 PLY(供 pcl_mesh2pcd), 然后退出")
    ap.add_argument("--sample-pcd", type=int, default=0,
                    help="用 trimesh 表面均匀采样生成点云并写 PCD(每几何点数)")
    args = ap.parse_args()
    if args.ply_only:
        obj_to_ply()
        sys.exit(0)
    if args.sample_pcd > 0:
        P = sample_cloud_trimesh(args.sample_pcd)
        print("[info] trimesh 表面采样点云:", len(P))
        sys.exit(0)

    import mujoco
    import trimesh

    glb = trimesh.load(GLB, force=None)
    bones = load_skeleton(glb)
    if args.pcd:
        P = load_pcd(args.pcd)
        print("[info] PCL 点云(PCD) 顶点数:", len(P))
    else:
        P, obj_scene = load_obj_cloud()
        print("[info] OBJ 顶点总数:", len(P))
    groups, counts = assign_bones(P, bones)
    print("[info] 骨架骨数:", len(bones), "分云:", counts)

    os.makedirs(os.path.dirname(XML_PATH), exist_ok=True)
    xml = finalize_mjcf(bones, groups)
    with open(XML_PATH, "w", encoding="utf-8") as f:
        f.write(xml)
    print("[info] 已写:", XML_PATH)

    model = mujoco.MjModel.from_xml_string(xml)
    data = mujoco.MjData(model)
    print("[info] nbody=%d ngeom=%d njoint=%d 模型加载 OK"
          % (model.nbody, model.ngeom, model.njnt))

    if args.viewer:
        mujoco.mj_resetData(model, data)
        try:
            import mujoco.viewer
            with mujoco.viewer.launch_passive(model, data) as v:
                print("[info] 窗口已开. 右下 Data 界面拖动 j_hip_* / j_arm_* 看点云随骨动")
                while v.is_running():
                    mujoco.mj_step(model, data)
                    v.sync()
        except Exception as e:
            print("[warn] viewer 不可用(无显示环境?), 离线渲染模式代替:", e)
            args.render = True

    if args.render:
        render_demo(model, data, PNG_OUT)


def render_demo(model, data, out):
    import mujoco
    # 那 prejoint 帧
    import numpy as np
    # 关节 id
    names = ["hip_pitch_L", "hip_pitch_R", "hip_knee_L", "hip_knee_R",
             "hip_ankle_L", "hip_ankle_R", "arm_pitch_L", "arm_pitch_R", "head"]
    jids = {n: mujoco.mj_name2id(model, mujoco.mjtObj.mjOBJ_JOINT, "j_" + n) for n in names}
    # 多帧渲染组成 horizontal 条带
    frames = []
    nf = 8
    for f in range(nf):
        t = f / nf * 2 * np.pi
        mujoco.mj_resetData(model, data)
        for n in names:
            seed = sum(map(ord, n)) % 5
            v = 0.55 * np.sin(0.55 * t + seed * 0.7)
            jid = jids[n]
            # qpos 下标是 adof 累积; 更稳: 用 joint qposadr
            adr = model.jnt_qposadr[jid]
            data.qpos[adr] = min(max(v, -1.2), 0.6)
        mujoco.mj_forward(model, data)
        img = render_skel(model, data)
        frames.append(img)
    # 并排拼接
    h, w = frames[0].shape[:2]
    nc = frames[0].shape[2] if len(frames[0].shape) == 3 else 1
    canvas = np.zeros((h, w * nf, nc), np.uint8)
    for i, im in enumerate(frames):
        canvas[:, i * w:(i + 1) * w] = im
    _save_png(canvas, out, has_alpha=(nc == 4))
    print("[info] 已保存摆腿动画合成图:", out)


def _save_png(img, path, has_alpha=True):
    import matplotlib.pyplot as plt
    a = img[..., :3] if img.ndim == 3 and img.shape[2] >= 3 else img
    plt.imsave(path, a)


if __name__ == "__main__":
    main()