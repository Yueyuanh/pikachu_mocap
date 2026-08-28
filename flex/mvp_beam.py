#!/usr/bin/env python3
# -*- coding: utf-8 -*-
"""
mvp_beam.py — 「物理属性点云」柔性外皮 MVP（最小可行性验证）

对应 flex/readme.md 的核心理念：摒弃 FEA，用「物理属性点云」作为柔性外皮的代理模型。

本脚本搭建一棵「橡胶棒」刚体核心骨骼，其外围包裹一圈可碰撞的稀疏点云小球，
这些点云随刚体骨骼运动，并与地面产生物理碰撞——即验证
「点云附着在 MuJoCo 刚体骨骼上，既能产生物理交互，又随骨骼运动驱动形变」的第一步。

用法(在 mocap conda 环境):
    python flex/mvp_beam.py                 # 离屏仿真:生成 xml、跑一段、存 2D 渲染 png、打印物理指标
    python flex/mvp_beam.py --viewer        # 打开 MuJoCo 实时窗口，可拖拽交互
    python flex/mvp_beam.py --steps 60      # 覆盖仿真步数(默认 40, 每步 1/240s≈0.167s 末段)
"""
import argparse
import os

import numpy as np

HERE = os.path.dirname(os.path.abspath(__file__))
MODEL_DIR = os.path.join(HERE, "models")
XML_PATH = os.path.join(MODEL_DIR, "beam_soft.xml")

# ── 橡胶棒几何/点云参数 ──────────────────────────────────────────────────
CORE_R = 0.030       # 刚体核心(芯)半径 m
BEAM_LEN = 0.50      # 棒长 m
SKIN_R = 0.042       # 外皮半径(点云球心所在圆) m
CLOUD_R = 0.012      # 点云小球半径 m
N_RINGS = 7          # 沿轴向环数
N_PER_RING = 6       # 每环点数
X_BASE = 0.60        # 初始离地高度(杆中心) m
X_BACK = 0.0         # 杆 z 向摆放(沿 z 轴)


def build_mjcf() -> str:
    """生成橡胶棒核心 + 柔性点云层的 MJCF 文本。"""
    lines = []
    lines.append('<?xml version="1.0" encoding="UTF-8"?>')
    lines.append('<mujoco model="beam_soft_mvp">')
    lines.append('  <option timestep="0.004" gravity="0 0 -9.81"/>')
    lines.append('  <visual><global elevation="-15" azimuth="130" '
                 'offwidth="720" offheight="720"/></visual>')
    lines.append('  <worldbody>')
    # 地面(验证点云先着地、充当缓冲)
    lines.append('    <geom name="floor" type="plane" size="2 2 0.1" '
                 'pos="0 0 0" rgba="0.45 0.50 0.55 1" friction="0.8"/>')
    lines.append('    <body name="beam" pos="0 0 %g">' % X_BASE)
    lines.append('      <freejoint/>')
    # 刚性核心(橡胶棒骨): 沿 x 轴水平横躺,便于底部点云先触地作缓冲
    lines.append('      <geom name="core" type="capsule" '
                 'fromto="0 0 0 %g 0 0" size="%g" '
                 'rgba="0.10 0.55 0.90 0.55" />'
                 % (BEAM_LEN, CORE_R))
    # 外皮点云小球:沿轴向均匀布 N_RINGS 环,每环绕 core 圆周 N_PER_RING 点
    # (x: 轴向, y/z: 径向) —— 杆横躺下落时,Z 方向底部的点云先于 core 触地
    idx = 0
    for ring in range(N_RINGS):
        x = (ring + 0.5) / N_RINGS * BEAM_LEN
        for k in range(N_PER_RING):
            ang = 2 * np.pi * k / N_PER_RING
            y = SKIN_R * np.cos(ang)
            z = SKIN_R * np.sin(ang)
            lines.append('      <geom name="cloud_%02d" type="sphere" '
                         'pos="%g %g %g" size="%g" '
                         'rgba="1.00 0.42 0.10 0.9" friction="0.3" condim="1"/>'
                         % (idx, x, y, z, CLOUD_R))
            idx += 1
    lines.append('    </body>')
    # 相机
    lines.append('    <camera name="main" pos="0.6 -0.7 0.8" xyaxes="0.9 0.5 0 -0.3 0.15 1"/>')
    lines.append('  </worldbody>')
    lines.append('</mujoco>')
    return "\n".join(lines) + "\n"


def main():
    ap = argparse.ArgumentParser(description="柔性橡皮筋点云 MVP 验证")
    ap.add_argument("--viewer", action="store_true", help="打开 MuJoCo 实时窗口")
    ap.add_argument("--steps", type=int, default=120,
                    help="仿真步数(离屏模式); 默认 120")
    ap.add_argument("--out", type=str, default=None,
                    help="离屏渲染 png 输出路径")
    args = ap.parse_args()

    import mujoco

    os.makedirs(MODEL_DIR, exist_ok=True)
    xml = build_mjcf()
    with open(XML_PATH, "w", encoding="utf-8") as f:
        f.write(xml)
    print(f"[info] 已写出模型: {XML_PATH}")

    model = mujoco.MjModel.from_xml_string(xml)
    data = mujoco.MjData(model)

    cloud_ids = [mujoco.mj_name2id(model, mujoco.mjtObj.mjOBJ_GEOM, f"cloud_{i:02d}")
                 for i in range(N_RINGS * N_PER_RING)]
    cloud_ids = [g for g in cloud_ids if g >= 0]
    core_id = mujoco.mj_name2id(model, mujoco.mjtObj.mjOBJ_GEOM, "core")

    if args.viewer:
        # 实时交互窗口
        try:
            import mujoco.viewer
        except Exception as e:
            print("[err] 无法加载 mujoco.viewer:", e)
            return
        mujoco.mj_resetData(model, data)
        with mujoco.viewer.launch_passive(model, data) as viewer:
            try:
                while viewer.is_running():
                    mujoco.mj_step(model, data)
                    viewer.sync()
            except KeyboardInterrupt:
                pass
        return

    # ── 离屏: 跑一段仿真 + 统计物理指标 ────────────────────────────────
    mujoco.mj_resetData(model, data)
    n_cloud = len(cloud_ids)
    print(f"[info] 核心刚体: core(芯半径{CORE_R})  外皮点云小球: {n_cloud} 个")
    print("[info] 物理指标采样(每 10 步): 高度 / 动能 / 接触中的点云小球数")
    max_contact = 0
    for step in range(args.steps):
        mujoco.mj_step(model, data)
        if mj_needs_stop(data):
            break
        if step % 10 == 0:
            # 统计与地面接触中的点云小球(验证「外皮先着地、随骨运动参与碰撞」)
            n_cloud_contact = count_cloud_floor_contacts(data, cloud_ids)
            max_contact = max(max_contact, n_cloud_contact)
            ke = mj_kinetic_energy(model, data)
            print(f"  t={data.time:6.3f}s  z={float(data.qpos[2]):6.3f}m  "
                  f"KE={float(ke):7.4f}J  点云接触={n_cloud_contact:>2d}/{n_cloud}")

    # 打印汇总结论
    slow = data.qpos[2]
    contact_ratio = max_contact / n_cloud
    print("─" * 56)
    print("[结果] 末段杆中心高度: %.3f m  |  峰值点云触地点数占比: %.0f%%"
          % (slow, contact_ratio * 100))
    if contact_ratio > 0.3:
        print("[结论] ✓ 点云随刚体骨骼运动并参与地面碰撞,柔性外皮缓冲通路打通")
    else:
        print("[结论] 点云接触占比偏低,可增大下落高度/加长仿真步数再观察")

    # 离屏渲染保存首末帧
    out = args.out or os.path.join(HERE, "mvp_beam_shots.png")
    try:
        save_render(model, data, out)
        print(f"[info] 已保存渲染图: {out}")
    except Exception as e:
        print(f"[warn] 离屏渲染不可用(可忽略,仅物理指标有效): {type(e).__name__}: {e}")
        print("       提示: 若需渲染,可 pip install EGL/osmesa 或改用 --viewer")


# ── helpers ──────────────────────────────────────────────────────────────
def mj_needs_stop(_d):
    """扩展点: 可在此加入仿真中止条件(如落地后稳定)。"""
    return False


def count_cloud_floor_contacts(data, cloud_ids):
    """统计触地(与 floor 碰撞)中的点云小球数。"""
    floor_id = 0
    ids = set(cloud_ids)
    n = 0
    for c in data.contact[:data.ncon]:
        if not (c.geom1 in ids or c.geom2 in ids):
            continue
        other = c.geom2 if c.geom1 in ids else c.geom1
        if other == floor_id:
            n += 1
    return n


def mj_kinetic_energy(model, data):
    """动能(点云+核心), 用物体线/角速度粗算。"""
    ke = 0.0
    for i in range(model.nbody):
        m = model.body_mass[i]
        v = data.cvel[i, 3:6]
        w = data.cvel[i, 0:3]
        ke += 0.5 * m * np.dot(v, v)
        ke += 0.5 * np.sum(model.body_inertia[i] * w * w)
    return float(ke)


def save_render(model, data, out):
    """尝试离屏渲染一帧(首或末)并存 png。依赖 mujoco.Renderer(需 EGL/osmesa)。"""
    from mujoco import Renderer
    rend = Renderer(model, 720, 480)  # 创建最晚用,确保 EGL 可用
    rend.update_scene(data, "main")
    img = rend.render()
    # 使用小 m 画幅原图存 PNG
    import struct, zlib
    h, w = img.shape[0], img.shape[1]
    raw = img.tobytes()
    def chunk(mtype, b):
        c = struct.pack(">I", len(b)) + mtype + b
        return c + struct.pack(">I", zlib.crc32(mtype + b) & 0xffffffff)
    png = (b"\x89PNG\r\n\x1a\n"
           + chunk(b"IHDR", struct.pack(">IIBBBBB", w, h, 8, 6, 0, 0, 0))
           + chunk(b"IDAT", zlib.compress(b"\x00" * w + raw, 6))
           + chunk(b"IEND", b""))
    with open(out, "wb") as f:
        f.write(png)


if __name__ == "__main__":
    main()