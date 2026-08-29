#!/usr/bin/env python3
# -*- coding: utf-8 -*-
"""soft_dent_live.py — MuJoCo 实时交互仿真窗口

弹原生 MuJoCo viewer：实时看「皮卡丘弹性点云」从高处落地 → 压陷 → 回弹的全过程。
操作：左键拖旋转 · 滚轮缩放 · 右键平移 · 空格=暂停/继续 · 双击=跟随。
按键：R = 重置回下落位形(反复看凹陷)。窗口关闭即退出。

依赖：需本地显示器(GUI)。mujoco.viewer 用 PySide6/glfw 后端渲染。

用法: conda run -n mocap python flex/sim/soft_dent_live.py \
          [--n 100 --tc 0.15 --damp 1.5 --drop 1.2 --speed 1]
  --speed: >1 快进(每物理步连走 N 步), <1 慢放, 省得等它慢慢落地。
"""
import argparse, os, sys, time
import numpy as np

HERE = os.path.dirname(os.path.abspath(__file__))
sys.path.insert(0, HERE)
import soft_dent as sd          # 复用 load_pcd + build_xml(已收敛的软 connect 构件)


def main():
    ap = argparse.ArgumentParser(description="MuJoCo 实时弹性点云落地凹陷")
    ap.add_argument("--n", type=int, default=100)
    ap.add_argument("--tc", type=float, default=0.15)
    ap.add_argument("--damp", type=float, default=1.5)
    ap.add_argument("--drop", type=float, default=1.2)
    ap.add_argument("--speed", type=float, default=1.0)
    ap.add_argument("--check", action="store_true",
                    help="headless 自检: 只验证模型构建+重置逻辑, 不弹窗")
    args = ap.parse_args()

    import mujoco

    P = sd.load_pcd(sd.PCD, args.n)
    m = mujoco.MjModel.from_xml_string(
        sd.build_xml(P, args.tc, args.drop, args.damp))
    d = mujoco.MjData(m)
    npts = args.n
    pt_ids = list(range(2, npts + 2))

    state0 = {}     # 重置基准: 把核心与壳点的位形深度拷贝
    state0["qpos"] = d.qpos.copy()
    state0["qvel"] = d.qvel.copy()

    def reset(key=0):
        if key and key not in (ord("r"), ord("R")):
            return                     # 只认 R/r 重置, 其余键交给 viewer(空格=暂停)
        d.qpos[:] = state0["qpos"]
        d.qvel[:] = state0["qvel"]
        mujoco.mj_forward(m, d)

    if args.check:
        # 无窗口: 简单推几步看它确实下落(reproduce soft_dent 的收敛验证)
        reset()
        for _ in range(1200):
            mujoco.mj_step(m, d)
        zmin = float(np.min([d.xpos[b][2] for b in pt_ids]))
        print("CHECK OK: n=%d tc=%g damp=%g drop=%g | 1200 步后壳点最低 z=%.3f m (%s)"
              % (npts, args.tc, args.damp, args.drop, zmin,
                 "已触地" if zmin < 0.02 else "仍在落"))
        reset()
        zmin0 = float(np.min([d.xpos[b][2] for b in pt_ids]))
        print("CHECK: 重置回落位形, 壳点最低 z=%.3f m" % zmin0)
        return
    import mujoco.viewer

    print(">> 弹 MuJoCo 实时窗口… (按 R 重置回落 | 空格暂停 | 关窗退出)")

    try:
        with mujoco.viewer.launch_passive(m, d,
                                          key_callback=reset,      # 按键时回调(空格/R 都到这)
                                          show_left_ui=True,
                                          show_right_ui=True) as view:
            step = 0
            while view.is_running():
                for _ in range(max(1, int(round(args.speed)))):
                    mujoco.mj_step(m, d)
                view.sync()
                time.sleep(max(0.004, m.opt.timestep / max(args.speed, 100)))
                step += 1
        print(">> 窗口已关闭。")
    except ImportError as e:
        print(">> mujoco.viewer 后端缺失(%s): 本环境无显示器/缺 PySide6 或 glfw。\n"
              "%s" % (e, "  头less 验证请用 flex/sim/soft_dent.py --png (离屏渲染)。"))
        sys.exit(2)


if __name__ == "__main__":
    main()