#!/usr/bin/env python3
# -*- coding: utf-8 -*-
"""
elastic_collision.py — 「皮卡丘点云外皮」弹性碰撞实验(阶段2)

第一性原理:
    恢复系数 e = v_after / v_incident(正碰, 出触=入触速率之比)。
    自由落体碰平面: h_rebound = e^2 * h_drop。
    弹性不损失 ⇔ e=1; 完全塑性(压实) ⇔ e=0。
    MuJoCo 默认接触为塑性。回弹由接触软参数 solref 的阻尼比 dampratio 决定:
        dampratio≈1   临界阻尼 → 塑性压实(e→0)
        dampratio<1   欠阻尼机械过冲 → 回弹(e→1 随着 dampratio→0)
    因此 dampratio 是「弹性旋钮」, 实测 h_rebound 并可反推 e_e2 = sqrt(h_reb/h_drop)。

实验A  基线:  撞击球 vs 刚性地板            (测刚体本征 e, 隔离外皮)
实验B  皮卡丘: 撞击球 vs 皮卡丘头部点云外皮   (测综合 e + 外皮应变)
实验C  能量:   同 B, 看系统能量时间线、碰撞损耗率 1-e^2。

用法; conda run -n mocap python flex/sim/elastic_collision.py --which B --dampratio 0.15 --drop 0.5
"""
import argparse
import json
import os
import sys

import numpy as np

HERE = os.path.dirname(os.path.abspath(__file__))
MODEL_DIR = os.path.join(HERE, "models")
REPORT_DIR = os.path.join(HERE, "reports")
os.makedirs(REPORT_DIR, exist_ok=True)

# 复用皮卡丘资产的读取/分骨函数(import 只定义函数, 不执行 main)
sys.path.insert(0, HERE)
from pikachu_cloud import OBJ, GLB, load_pcd, load_skeleton, CLOUD_R, SEED

# ── 全局实验参数 ──────────────────────────────────────────────────────────
DT = 0.001                  # 仿真步长 s(须细于接触弹性振荡周期)
DAMPRATIOS = [1.0, 0.5, 0.2, 0.05, 0.0]   # 弹性旋钮(1=临界阻尼完全塑性, 0=理想无阻尼弹簧,e→1)
BALL_R = 0.035              # 撞击球半径 m
BALL_MASS = 0.12            # 球质量 kg(默认); 实验改变量独立设置
DROP = 0.60                 # 球初始高度m(球心之离地; ~0.6m)
PER_BONE = 60               # 每骨参与碰撞的点云球数(降采样, 保证求解稳定)
HEAD = "head"


def load_pika_parts(pcd_path):
    """返回 (点云P: Nx3, 骨架 bones: dict, groups: dict<bone>->Nx3头部坐标)。"""
    import trimesh
    from pikachu_cloud import assign_bones
    P = load_pcd(pcd_path)
    print("  点云顶点: %d" % len(P))
    glb = trimesh.load(GLB, force=None)
    bones = load_skeleton(glb)          # bone -> 全局4x4
    groups, counts = assign_bones(P, bones)     # bone -> np.array 点(已按 PER_BONE=220 降采样)
    # 再压到每骨 PER_BONE(60, 保证可碰撞点数适中)
    for b in groups:
        g = groups[b]
        if len(g) > PER_BONE:
            rng = np.random.default_rng(SEED)
            idx = rng.choice(len(g), PER_BONE, replace=False)
            groups[b] = g[idx]
    return P, bones, groups


def build_mjcf(bones, groups, dampratio, with_cloud, drop=DROP):
    """MJCF: 地板 + (可选点云外皮, contype=1 可碰撞) + 撞击球(高处释放)。"""
    root_bone = "骨架"
    order = ["骨架", "base_link", "hip_L", "hip_R",
             "hip_pitch_L", "hip_pitch_R", "hip_knee_L", "hip_knee_R",
             "hip_ankle_L", "hip_ankle_R", "head", "arm_L", "arm_R",
             "arm_pitch_L", "arm_pitch_R"]
    bones_present = [b for b in order if b in bones and b in groups]
    body_pos = {b: bones[b][:3, 3].copy() for b in bones_present}
    origin = body_pos[list(bones_present)[0]].copy() if bones_present else np.zeros(3)
    for b in body_pos:
        body_pos[b] -= origin

    solref = "0.001 %g" % dampratio    # 相对时间常数, 只扫阻尼比; solimp dmin 压小唤醒弹性
    L = ['<?xml version="1.0"?>',
         '<mujoco model="pika_elastic">',
         '<compiler angle="radian"/>',
         '<option timestep="%g" gravity="0 0 -9.81" integrator="RK4"/>'
         % DT,
         '<visual><global elevation="-30" azimuth="120" offwidth="900" offheight="900"/></visual>',
         '<worldbody>']
    L.append('  <geom name="floor" type="plane" size="3 3 0.1" pos="0 0 -0.02" '
             'rgba="0.42 0.47 0.55 1" friction="0.0" condim="1"/>')
    # 光照(保证离屏不暗)
    L.append('  <light name="key" pos="1.2 1.8 2.6" dir="-0.35 -0.5 -1" '
             'diffuse="1 1 1" ambient="0.85 0.85 0.85"/>')
    L.append('  <light name="fill" pos="-1.6 0.2 1.2" dir="0.55 -0.1 -0.8" '
             'diffuse="0.8 0.8 0.85" ambient="0.6 0.6 0.6"/>')

    # ── 皮卡丘骨架刚体(关节冻结, 仅作点云载体) ──
    if with_cloud:
        L.append('  <body name="pika" pos="0 0 0">')
        # 本体胶囊(视觉参考, 不碰撞 contype=0 避免干扰)
        L.append('    <geom name="torso" type="capsule" fromto="0 0 0.12 0 0 0.32" '
                 'size="0.05" contype="0" conaffinity="0" rgba="0.95 0.80 0.30 0.35"/>')
        # 点云小球(可碰撞)
        for b in bones_present:
            for i, q in enumerate(groups[b]):
                lp = q - bones[b][:3, 3] - origin
                L.append('    <geom name="%s_pt_%02d" type="sphere" size="%g" '
                         'pos="%.4f %.4f %.4f" rgba="0.95 0.55 0.05 1" '
                         'condim="1" solref="%s" '
                         'contype="1" conaffinity="1"/>'
                         % (b, i, CLOUD_R, lp[0], lp[1], lp[2], solref))
        L.append('  </body>')
    # ── 撞击球, 从 drop 高度释放 ──
    L.append('  <body name="ball" pos="0 0 %g">' % drop)
    L.append('    <freejoint/>')
    L.append('    <geom name="ball_g" type="sphere" size="%g" '
             'rgba="0.90 0.28 0.20 1" mass="%g" condim="1" solref="%s" '
             ' friction="0.0"/>'
             % (BALL_R, BALL_MASS, solref))
    L.append('  </body>')
    # 相机(正面看皮卡丘, 略仰) —— 球落向头部
    L.append('  <camera name="cam" pos="0.4 -1.1 0.55" xyaxes="1 0 0 0 0.5 1"/>')
    L.append('</worldbody></mujoco>')
    return "\n".join(L) + "\n"


def run_sim(model, data, steps_max, record_head=True):
    """跑仿真; 记录 球z(反弹峰)、速度、系统能量、皮卡丘头部点云最大下压(应变)。
    返回 dict: {script, t_max, rebound_h, e_floor2, head_squash_mm, ke, pe, ke_loss}"""
    import mujoco
    ball_body = mujoco.mj_name2id(model, mujoco.mjtObj.mjOBJ_BODY, "ball")
    zbuf, vz, ebuf = [], [], []          # 球 z / 球 vz / 系统能量(PE+KE)
    max_overlap = 0.0                    # 接触重叠最大值(外皮应变代理)

    for step in range(steps_max):
        mj.mj_step(model, data)
        # 球 z(xpos 直读, 免 qpos 布局依赖)
        xball = data.xpos[ball_body][2]
        vball = data.cvel[ball_body][5] if ball_body < model.nbody else 0.0
        zbuf.append(xball)
        vz.append(vball)
        # 能量 PE+KE 相对地板(z=0)
        ke = mj_kinetic_energy(model, data)
        pe = total_pe(model, data)
        ebuf.append(ke + pe)
        # 接触重叠(外皮应变代理): 球正下方 与任何g 点云geom 的接触
        for c in data.contact[:data.ncon]:
            if c.dist < 0:
                max_overlap = max(max_overlap, float(-c.dist))
    zbuf = np.array(zbuf); vz = np.array(vz); ebuf = np.array(ebuf)
    # 物理判据: 首次"下落→弹起"的 v 符号翻转(负变正)。无翻转=完全塑性未起弹, 记 None
    bounce_idx = None
    for i in range(1, len(vz)):
        if vz[i - 1] < -0.1 and vz[i] > 0:
            bounce_idx = i
            break
    rebound_h = None
    e_floor2 = None
    if bounce_idx is not None:
        peak = float(max(zbuf[bounce_idx:]))       # 弹起后整段最高(多峰也取最高)
        rebound_h = peak - BALL_R                  # 球心最高-半径 → 离地高度
        drop_h = DROP - BALL_R
        if drop_h > 0 and rebound_h > 0:
            e_floor2 = float(np.sqrt(min(1.0, rebound_h / drop_h)))
    ke_loss = float(ebuf[-1] / ebuf[0]) if ebuf[0] > 1e-9 else float("nan")
    return {"rebound_h": rebound_h, "e_eff": e_floor2,
            "squash_mm": float(max_overlap * 1000.0),
            "z_min": float(zbuf.min()), "ke0": float(ebuf[0]),
            "ke_final_ratio": ke_loss}


def mj_kinetic_energy(model, data):
    import mujoco
    ke = 0.0
    for i in range(model.nbody):
        m = model.body_mass[i]
        v = data.cvel[i, 3:6]; w = data.cvel[i, 0:3]
        ke += 0.5 * m * np.dot(v, v)
    return float(ke)


def total_pe(model, data):
    pe = 0.0
    for i in range(model.nbody):
        m = model.body_mass[i]
        pe += m * 9.81 * float(data.xpos[i, 2])
    return float(pe)


def save_frame(model, data, out, camera="cam", w=900, h=900):
    """离屏渲染一帧存png(不读图, 仅供塞进报告)。"""
    from mujoco import Renderer
    try:
        rend = Renderer(model, w, h)
        rend.update_scene(data, camera)
        img = rend.render()
        from matplotlib import pyplot as plt
        plt.imsave(out, img)
        return True
    except Exception as e:
        print("[warn] 离屏渲染跳过:", type(e).__name__, e)
        return False


def main():
    ap = argparse.ArgumentParser(description="皮卡丘点云外皮弹性碰撞实验")
    ap.add_argument("--which", choices=["A", "B", "C"], default="B")
    ap.add_argument("--dampratio", type=float, default=None)
    ap.add_argument("--drop", type=float, default=DROP)
    ap.add_argument("--steps", type=int, default=3000)
    ap.add_argument("--pcd", type=str, default=os.path.join(MODEL_DIR, "pikachu_skin_real.pcd"))
    ap.add_argument("--shot", action="store_true",
                    help="离屏渲染几张关键帧到 reports/(实验A每档一张)")
    args = ap.parse_args()

    import mujoco
    with_cloud = args.which in ("B", "C")
    do_list = [args.dampratio] if args.dampratio is not None else DAMPRATIOS
    results = {}
    print("=" * 66)
    print("皮卡丘点云 弹性碰撞实验 | 方案 %s | 入射高 %.2f m | 球质量 %.3f kg"
          % (args.which, args.drop, BALL_MASS))
    print("点云外皮参与碰撞: %s" % with_cloud)
    print("=" * 66)

    bones = groups = None
    if with_cloud:
        _, bones, groups = load_pika_parts(args.pcd)
    for i, dr in enumerate(do_list):
        xml = build_mjcf(bones or {}, groups or {}, dr, with_cloud, args.drop)
        model = mujoco.MjModel.from_xml_string(xml)
        data = mujoco.MjData(model)
        mujoco.mj_resetData(model, data)
        r = run_sim(model, data, args.steps)
        results[dr] = r
        tag = {"A": "刚性基线球vs地板", "B": "皮卡丘点云外皮", "C": "能量"}[args.which]
        print("\ndampratio=%.2f (%s):" % (dr, tag))
        print("  球距地最高反弹 %.3f m | e_eff=%.3f | 点云/接触挤压 %.1f mm | 末能量比 %.3f"
              % (r["rebound_h"] if r["rebound_h"] else float("nan"),
                 r["e_eff"] if r["e_eff"] else float("nan"),
                 r["squash_mm"], r["ke_final_ratio"]))
        if args.shot:
            base = "%s_d%.2f" % (args.which, dr)
            # 存: 触地前(固定若干步) + 反弹峰值帧
            save_frame(model, data, os.path.join(REPORT_DIR, base + "_peak.png"))
            # 重跑到 mid
            data2 = mujoco.MjData(model); mujoco.mj_resetData(model, data2)
            for _ in range(min(args.steps // 3, 400)):
                mujoco.mj_step(model, data2)
            save_frame(model, data2, os.path.join(REPORT_DIR, base + "_impact.png"))
    out = os.path.join(REPORT_DIR, "elastic_%s.json" % args.which)
    with open(out, "w", encoding="utf-8") as f:
        json.dump({"which": args.which, "results": {str(k): v for k, v in results.items()}},
                  f, ensure_ascii=False, indent=1)
    print("\n[写] %s" % out)


DEMO = False


if __name__ == "__main__":
    main()