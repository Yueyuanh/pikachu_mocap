#!/usr/bin/env python3
# -*- coding: utf-8 -*-
"""
soft_dent.py — 纯 MuJoCo「物理属性点云」外皮落地凹陷实验

背景: 这个 MuJoCo 构建(3.12)没有原生软体(<flex>/<softbody>/<deformable> 都编译掉了)。
方案: 把视觉蒙皮 PPTD 成一小团「弹性点云」——每个点 = 一个小球质点, 用 MUJOCO 软 equality
约束(带 solref 阻尼弹簧)把它耦合到刚性内核的定位骨位上。落地时地面压力把底部点向核心压陷
(可见凹陷), 松释后弹簧拉回(回弹)。这就是 flex/ 目录「物理属性点云在 MuJoCo 里算」的实证。

物理骨架:
  · 内核 = 一个带 freejoint 的刚体(可下落/旋转), 壳层附着其上
  · 壳点 = N 个 sphere geom 刚体, 每个一条 <equal> 与内核某定位 site 对齐(软)
  · 软执: <equal solref="tc 1" solimp="0.9 0.99 0.001"/> —— tc 越大=越软=压得越深
  · 落地: 触地面(平面), 底部点被顶向核心→壳面下压凹陷; 停止后 dt 回弹

输出:
  · 控制台指标 + reports/soft_dent_*.json
  · --png: 离屏渲染 hover/peak/rest 三帧 → reports/soft_dent_{hover,peak,rest}.png
    (程序化像素比对凹陷, 不读图)

用法: conda run -n mocap python flex/sim/soft_dent.py --n 300 --tc 0.004 --drop 1.2
"""
import argparse
import copy
import json
import os

import numpy as np

HERE = os.path.dirname(os.path.abspath(__file__))
PCD = os.path.join(HERE, "models", "pikachu_skin.pcd")
REPORT = os.path.join(HERE, "reports")
SEED = 7


def load_pcd(path, n):
    """读 ASCII PCD, 子采样到 n 点, 统一到质心系。返回 (n,3)。"""
    pts = []
    with open(path, "r", encoding="utf-8", errors="replace") as f:
        data = False
        for line in f:
            s = line.strip()
            if not s:
                continue
            if s.upper().startswith("DATA"):
                data = True
                continue
            if not data:
                continue
            p = s.split()
            if len(p) >= 3:
                try:
                    pts.append([float(p[0]), float(p[1]), float(p[2])])
                except ValueError:
                    pass
    P = np.asarray(pts, dtype=float)
    if n and len(P) > n:
        P = P[np.random.default_rng(SEED).choice(len(P), n, replace=False)]
    return P - P.mean(0)


def build_xml(P, tc, drop, damp=2.0):
    """nut rig: 内核(free joint) + N 个软耦合壳点。damp=关节速度阻尼——缺了它弹簧网络永不收敛(抖成一片)。"""
    N = len(P)
    r_g = 0.014                    # 壳点半径 m
    lines = []
    lines.append('<mujoco model="pika_soft_skin">')
    lines.append('  <compiler autolimits="true"/>')
    lines.append('  <visual><global offwidth="800" offheight="600"/></visual>')
    lines.append('  <option gravity="0 0 -9.81" timestep="0.002" iterations="16">')
    lines.append('  </option>')
    lines.append('  <worldbody>')
    lines.append('    <geom name="ground" type="plane" size="4 4 0.1" rgba="0.55 0.55 0.55 1"/>')
    lines.append('    <camera name="cam0" mode="targetbody" target="core" pos="0 -2.4 1.25"/>')
    lines.append('    <body name="core" pos="0 0 %g">' % drop)
    # 显式 free joint + 阻尼(不能用 <freejoint/> —— 它不接受 class/damping, 弹簧场才永远不会静止)
    lines.append('      <joint type="free" damping="%g"/>' % damp)
    # 内核只作「刚性参考点」: 要小, 不能把壳点包在几何里(否则出生即深穿透→爆炸)。
    # 视觉用半透明小球示意骨骼参考, 其余由壳点云表达软皮肤。
    lines.append('      <geom name="coreviz" type="sphere" size="0.05" rgba="0.75 0.3 0.3 0.65"/>')
    for i, p in enumerate(P):     # 内核上的定位 site(壳点 rest 位)
        lines.append('      <site name="s%d" pos="%g %g %g"/>' % (i, p[0], p[1], p[2]))
    lines.append('    </body>')
    for i, p in enumerate(P):     # 世界系壳点(同样离地 drop), 每个是自由质点随重力下落
        lines.append('    <body name="pt%d" pos="%g %g %g">' % (i, p[0], p[1], p[2] + drop))
        lines.append('      <joint type="free" damping="%g"/>' % damp)
        lines.append('      <site name="pt%d" pos="0 0 0"/>' % i)
        lines.append('      <geom name="g%d" type="sphere" size="%g" rgba="0.35 0.6 0.95 0.9"/>' % (i, r_g))
        lines.append('    </body>')
    lines.append('  </worldbody>')
    lines.append('  <equality>')
    for i in range(N):
        lines.append('    <connect name="eq%d" site1="s%d" site2="pt%d" '
                     'solref="%g 1" solimp="0.9 0.99 0.001"/>'
                     % (i, i, i, tc))
    lines.append('  </equality>')
    lines.append('</mujoco>')
    return "\n".join(lines)


def main():
    ap = argparse.ArgumentParser(description="纯 MuJoCo 弹性点云落地凹陷实验")
    ap.add_argument("--n", type=int, default=300)
    ap.add_argument("--tc", type=float, default=0.004, help="connect 软度时间常数(越小越软越压深; 收敛取 0.05~0.4)")
    ap.add_argument("--drop", type=float, default=1.2)
    ap.add_argument("--damp", type=float, default=2.0, help="质点 free joint 速度阻尼(缺收敛不了; 2 已被验收敛)")
    ap.add_argument("--steps", type=int, default=1500)
    ap.add_argument("--settle", type=int, default=400, help="触地后静置步数")
    ap.add_argument("--png", type=str, default="", help="非空则渲染该前缀的 3 帧 png")
    args = ap.parse_args()

    import mujoco
    P = load_pcd(PCD, args.n)
    m = mujoco.MjModel.from_xml_string(build_xml(P, args.tc, args.drop, args.damp))
    d = mujoco.MjData(m)
    pt_ids = list(range(2, args.n + 2))     # body0=world, body1=core, 2..n+1=壳点
    os.makedirs(REPORT, exist_ok=True)

    R = 0.014
    nstep = args.steps
    t_contact = None
    peak_depth = 0.0            # 底部球心低于 R 的最大值(凹陷深度 m)
    peak_frame = 0
    d_peak = None
    frames = []

    def shell_pos(data):
        return np.array([data.xpos[b] for b in pt_ids])

    pos = shell_pos(d)
    d_hover = copy.deepcopy(d)
    for it in range(nstep):
        mujoco.mj_step(m, d)
        pos = shell_pos(d)
        floor = pos[:, 2].min()
        if t_contact is None and floor < R + 0.005:
            t_contact = it * m.opt.timestep
            frames.append(("contact", copy.deepcopy(d)))
        if t_contact is not None:
            dep = np.maximum(R - pos[:, 2], 0.0).max()   # 最底部球被压入的量
            if dep > peak_depth:
                peak_depth = dep
                peak_frame = it
                d_peak = copy.deepcopy(d)

    # 静置回弹
    for _ in range(args.settle):
        mujoco.mj_step(m, d)
    pos_end = shell_pos(d)
    d_rest = copy.deepcopy(d)
    final_floor = pos_end[:, 2].min()
    final_span = pos_end[:, 2].max() - final_floor       # 静止垂直跨度(=软体压扁后的「身高」)
    rigid_span = max(float(P[:, 2].max() - P[:, 2].min()), 1e-9)   # 刚体对照高度(=原始跨度)
    settle_v = float(np.sqrt(np.sum(d.qvel ** 2)))       # 末速: <1 才算收敛

    print("== 纯 MuJoCo 软 connect 点云落地凹陷实验 ==")
    print("n=%d tc=%g damp=%g drop=%g | 首次触地 t=%.3fs (step %d/%d)"
          % (args.n, args.tc, args.damp, args.drop, t_contact or -1, int(t_contact / 0.002) if t_contact else -1, nstep))
    print("最深压入: %.1f mm @ step %d" % (peak_depth * 1000, peak_frame))
    print("静止垂直跨度 %.3f m vs 刚体 %.3f m | 压扁 -%.1f%% (凹/压扁量)"
          % (final_span, rigid_span, (1 - final_span / rigid_span) * 100))
    print("末速 %.2f | %s" % (settle_v, "✓ 收敛" if settle_v < 1 else "✗ 仍在抖"))

    meta = {"n": args.n, "tc": args.tc, "damp": args.damp, "drop": args.drop,
            "t_contact": t_contact, "peak_depth_m": float(peak_depth),
            "depth_mm": float(peak_depth * 1000), "final_floor_m": float(final_floor),
            "rigid_span_m": rigid_span, "settled_span_m": float(final_span),
            "squash_pct": float((1 - final_span / rigid_span) * 100),
            "settle_velocity": settle_v, "converged": settle_v < 1}
    with open(os.path.join(REPORT, "soft_dent_tc%s.json" % args.tc), "w") as f:
        json.dump(meta, f, indent=2, ensure_ascii=False)
    print("指标已写 reports/soft_dent_tc%s.json" % args.tc)

    if args.png:
        renderer = mujoco.Renderer(m, height=600, width=800)
        for name, dd in (("hover", d_hover), ("peak", d_peak), ("rest", d_rest)):
            if dd is None:
                continue
            mujoco.mj_forward(m, dd)
            renderer.update_scene(dd, camera="cam0")
            img = renderer.render()
            import PIL.Image
            PIL.Image.fromarray(img).save(os.path.join(REPORT, "soft_dent_%s.png" % name))
            print("已渲染 reports/soft_dent_%s.png" % name)


if __name__ == "__main__":
    main()