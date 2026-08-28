#!/usr/bin/env python3
# -*- coding: utf-8 -*-
"""soft_dent_pb.py — Part 2B：PyBullet 软体(FEM/deformable)落地压痕实验

背景: PyBullet 提供两种软体——
  1) 真 FEM: URDF <deformable> + .vtk 四面体体积网格(neohookean mu/lambda/damping)。
     pybullet_data 只随带 torus_deform.urdf 但缺 torus.vtk → 需自建 tet 网格。
  2) loadSoftBody(obj) 表面弹簧网(本实验用的方案 B): 读三角面 obj → 每顶点一质点,
     弹簧阻尼连接, 落地面即被压出凹痕。与 MuJoCo 的软 connect 点云是「另一套软体」对照。

本实验: 生成一张方形皮肤片 obj → loadSoftBody → 刚性球落到其上 → 记录接触点下方顶点的
        最大下压(凹痕深度 mm) + 回弹, 输出 reports/soft_dent_pb_*.json。

用法: conda run -n mocap python flex/sim/soft_dent_pb.py [--nx N --ny N --size S --drop D --settle T]
"""
import argparse, json, os, numpy as np

HERE = os.path.dirname(os.path.abspath(__file__))
REPORT = os.path.join(HERE, "reports")
TMPOBJ = os.path.join(HERE, "reports", "_pb_sheet.obj")


def gen_sheet_obj(nx, ny, size, path, base_z=0.05):
    """生成 nx×ny 顶点的方形皮肤片 obj(平铺三角面), XY 面, 高 base_z。返回 Nx3 顶点。"""
    xs = np.linspace(-size / 2, size / 2, nx)
    ys = np.linspace(-size / 2, size / 2, ny)
    ix, iy = np.meshgrid(xs, ys)
    verts = np.stack([ix.ravel(), iy.ravel(), np.full(nx * ny, base_z)], 1)
    with open(path, "w") as f:
        for v in verts:
            f.write("v %g %g %g\n" % (v[0], v[1], v[2]))
        for i in range(nx - 1):
            for j in range(ny - 1):
                a = j * nx + i                 # (0-based)
                b = a + 1
                c = a + nx
                d = c + 1
                f.write("f %d %d %d\n" % (a + 1, b + 1, c + 1))
                f.write("f %d %d %d\n" % (b + 1, d + 1, c + 1))
    return verts


def main():
    ap = argparse.ArgumentParser(description="PyBullet 软体落地压痕(方案B)")
    ap.add_argument("--nx", type=int, default=8)
    ap.add_argument("--ny", type=int, default=8)
    ap.add_argument("--size", type=float, default=0.9, help="皮肤片边长 m")
    ap.add_argument("--drop", type=float, default=1.2, help="球离地高度 m")
    ap.add_argument("--radius", type=float, default=0.35, help="压球半径 m")
    ap.add_argument("--steps", type=int, default=1500)
    ap.add_argument("--settle", type=int, default=2500)
    args = ap.parse_args()

    import pybullet as p
    os.makedirs(REPORT, exist_ok=True)
    verts = gen_sheet_obj(args.nx, args.ny, args.size, TMPOBJ)

    cid = p.connect(p.DIRECT)
    p.setGravity(0, 0, -9.81)
    p.setTimeStep(1.0 / 240)
    p.setPhysicsEngineParameter(numSubSteps=8, numSolverIterations=40)
    plane_col = p.createCollisionShape(p.GEOM_PLANE, planeNormal=[0, 0, 1])
    p.createMultiBody(0, plane_col)   # 刚性地面(z=0)

    # loadSoftBody 不接受 basePosition —— 位置已烘焙进 obj 顶点(base_z)。用 neohookean=1 更像 FEM。
    soft = p.loadSoftBody(TMPOBJ, scale=1, mass=1.0, collisionMargin=0.015,
                          useNeoHookean=1, useBendingSprings=1,
                          springElasticStiffness=260, springDampingStiffness=8)
    nv = verts.shape[0]
    bz = float(verts[0, 2])           # 软垫上表面(底面贴合地面 z=0 → 地面托住它, 不会整片塌)
    # 先让软垫静置平铺在地面上
    for _ in range(600):
        p.stepSimulation()

    def top_near_cx(cx=0.0, cy=0.0, r=None):
        # 球心 (cx,cy) 附近软垫「上表面」最低 z(凹痕要被球压出来的局部下凹)
        r = args.radius * 0.8 if r is None else r
        d = p.getMeshData(soft)
        zs = [float(v[2]) for v in d[1]
              if hasattr(v, "__len__") and len(v) >= 3 and abs(v[0] - cx) < r and abs(v[1] - cy) < r]
        return min(zs) if zs else float("nan")

    top0 = top_near_cx()              # 静置铺平后, 球压点处垫高(=垫厚 5cm, 压痕深理论上限)
    # 刚球从正上方落到垫中央
    ball = p.createCollisionShape(p.GEOM_SPHERE, radius=args.radius)
    ball_body = p.createMultiBody(2.0, ball, basePosition=[0, 0, args.drop])

    top_peak = top0
    peak_step = 0
    t_contact = None
    for it in range(args.steps):
        p.stepSimulation()
        znear = top_near_cx()
        bp = p.getBasePositionAndOrientation(ball_body)[0]
        if t_contact is None and bp[2] < args.radius + bz + 0.02:
            t_contact = it
        if znear < top_peak:
            top_peak = znear
            peak_step = it
    for _ in range(args.settle):
        p.stepSimulation()
    top_end = top_near_cx()
    dent_mm = (top0 - top_peak) * 1000          # 凹痕深度: 球压点垫高 - 最深
    dent_end_mm = (top0 - top_end) * 1000

    print("== PyBullet 软体(loadSoftBody 表面弹簧网)落地压痕 ==")
    print("软垫 %dx%d=%d 顶点 垫厚 %gcm | 球 r%gm 落高 %gm" % (args.nx, args.ny, nv, bz * 100, args.radius, args.drop))
    print("压点垫高 %.3f → 最深 %.3f | 凹痕最深 %.1f mm @ step %d" % (top0, top_peak, dent_mm, peak_step))
    print("末高 %.3f | 静置后凹痕 %.1f mm | %s"
          % (top_end, dent_end_mm,
             "✓ 回弹(明显)" if dent_mm - dent_end_mm > 2 else "→ 残留压痕"))

    meta = {"nx": args.nx, "ny": args.ny, "nverts": nv, "pad_thick_cm": bz * 100,
            "radius": args.radius, "drop": args.drop, "t_contact_step": t_contact,
            "top0_m": top0, "top_peak_m": float(top_peak),
            "dent_mm": float(dent_mm), "dent_end_mm": float(dent_end_mm),
            "rebound_mm": float(dent_mm - dent_end_mm)}
    with open(os.path.join(REPORT, "soft_dent_pb.json"), "w") as f:
        json.dump(meta, f, indent=2, ensure_ascii=False)
    print("指标已写 reports/soft_dent_pb.json")
    p.disconnect()


if __name__ == "__main__":
    main()