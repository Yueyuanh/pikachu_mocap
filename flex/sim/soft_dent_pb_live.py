#!/usr/bin/env python3
# -*- coding: utf-8 -*-
"""soft_dent_pb_live.py — PyBullet 实时交互仿真窗口(GUI)

弹 PyBullet 自带可视化：实时看刚性球砸到平铺地面的软垫 → 垫被压出局部凹坑 → 回弹。
操作：左键/右键/中键/滚轮拖视角。按键：R=重建场景重放(球重新下砸) · 空格=暂停。
窗口右上角 ⛶ 也可切换物理暂停。

依赖：需本地显示器(GUI 后端)。切换物理关联默认开启, 落得较自然。

用法: conda run -n mocap python flex/sim/soft_dent_pb_live.py \
          [--nx 8 --ny 8 --radius 0.35 --drop 1.2 --steps 1800]
  --check: DIRECT 无窗口自检(构建场景推几步, 验证逻辑不弹窗)。
"""
import argparse, json, os, sys, time
import numpy as np

HERE = os.path.dirname(os.path.abspath(__file__))
REPORT = os.path.join(HERE, "reports")
TMPOBJ = os.path.join(REPORT, "_pb_live_sheet.obj")


def gen_sheet_obj(nx, ny, size, path, base_z=0.05):
    xs = np.linspace(-size / 2, size / 2, nx)
    ys = np.linspace(-size / 2, size / 2, ny)
    ix, iy = np.meshgrid(xs, ys)
    verts = np.stack([ix.ravel(), iy.ravel(), np.full(nx * ny, base_z)], 1)
    with open(path, "w") as f:
        for v in verts:
            f.write("v %g %g %g\n" % (v[0], v[1], v[2]))
        for i in range(nx - 1):
            for j in range(ny - 1):
                a = j * nx + i; b = a + 1; c = a + nx; dd = c + 1
                f.write("f %d %d %d\n" % (a + 1, b + 1, c + 1))
                f.write("f %d %d %d\n" % (b + 1, dd + 1, c + 1))
    return verts


def build_scene(p, args, verts):
    """铺地面 + 软垫(仅此两样)。球在垫静置后再单独造, 免得提前落地错过凹陷段。"""
    plane_col = p.createCollisionShape(p.GEOM_PLANE, planeNormal=[0, 0, 1])
    p.createMultiBody(0, plane_col)
    soft = p.loadSoftBody(TMPOBJ, scale=1, mass=1.0, collisionMargin=0.015,
                          useNeoHookean=1, useBendingSprings=1,
                          springElasticStiffness=260, springDampingStiffness=8)
    return soft


def add_ball(p, args):
    ball = p.createCollisionShape(p.GEOM_SPHERE, radius=args.radius)
    return p.createMultiBody(2.0, ball, basePosition=[0, 0, args.drop])


def main():
    ap = argparse.ArgumentParser(description="PyBullet 实时软垫压痕")
    ap.add_argument("--nx", type=int, default=8)
    ap.add_argument("--ny", type=int, default=8)
    ap.add_argument("--size", type=float, default=0.9)
    ap.add_argument("--radius", type=float, default=0.35)
    ap.add_argument("--drop", type=float, default=1.2)
    ap.add_argument("--steps", type=int, default=1800)
    ap.add_argument("--sit", type=int, default=600, help="开场让软垫静置平铺的步数")
    ap.add_argument("--check", action="store_true", help="DIRECT 无窗口自检")
    args = ap.parse_args()

    import pybullet as p
    os.makedirs(REPORT, exist_ok=True)
    verts = gen_sheet_obj(args.nx, args.ny, args.size, TMPOBJ)

    mode = p.DIRECT if args.check else p.GUI
    p.connect(mode)
    p.setGravity(0, 0, -9.81)
    p.setTimeStep(1.0 / 240)
    p.setPhysicsEngineParameter(numSubSteps=8, numSolverIterations=40)

    soft = build_scene(p, args, verts)
    for _ in range(args.sit):        # 让软垫先平铺稳再开砸
        p.stepSimulation()

    def top_near_cx(r=None):
        r = args.radius * 0.8 if r is None else r
        d = p.getMeshData(soft)
        zs = [float(v[2]) for v in d[1]
              if hasattr(v, "__len__") and len(v) >= 3 and abs(v[0]) < r and abs(v[1]) < r]
        return min(zs) if zs else float("nan")

    if args.check:
        top0 = top_near_cx()
        ball = add_ball(p, args)     # 垫静置后再造球 → 完整复现「下砸→凹陷→回弹」
        top_peak = top0; t_contact = None
        for it in range(args.steps):
            p.stepSimulation()
            bp = p.getBasePositionAndOrientation(ball)[0]
            if t_contact is None and bp[2] < args.radius + 0.06:
                t_contact = it
            top_peak = min(top_peak, top_near_cx())
        print("CHECK OK: 软垫 %dx%d=%d 压点垫高 %.3f → 最低 %.3f | 凹痕 %.1f mm%s"
              % (args.nx, args.ny, len(verts), top0, top_peak,
                 (top0 - top_peak) * 1000,
                 " (球已触垫 @%d)" % t_contact if t_contact else " (球尚未触垫!)"))
        p.disconnect()
        return

    ball = add_ball(p, args)         # GUI 也走「先垫后球」

    # ---- 实时 GUI 循环 ----
    p.resetDebugVisualizerCamera(1.5, 50, -35, [0, 0, 0.25])
    print(">> PyBullet GUI 实时窗口 (按 R 重建重放 | 空格暂停 | 右上 ⛶ 暂停物理 | 关窗退出)")

    paused = False
    while True:
        keys = p.getKeyboardEvents()
        if ord('r') in keys and keys[ord('r')] & p.KEY_WAS_RELEASED:
            p.removeBody(ball); p.removeBody(soft)      # 重建 → 球重新下砸
            tprev = time.time()
            soft = build_scene(p, args, verts)
            for _ in range(args.sit):
                p.stepSimulation()
            ball = add_ball(p, args)
            print("   R: 已重建场景, 球重新下砸")
        if ord(' ') in keys and keys[ord(' ')] & p.KEY_WAS_RELEASED:
            paused = not paused
            print("   空格: %s" % ("暂停" if paused else "继续"))
        if not paused:
            for _ in range(3):                          # 每帧步进多点, 更流畅
                p.stepSimulation()
        time.sleep(1.0 / 240)


if __name__ == "__main__":
    main()