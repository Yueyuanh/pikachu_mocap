"""
Pikachu URDF(V025) → Blender rig 重定向映射逻辑（纯 Python，无 bpy 依赖）。

供 Pikachu_Retarget.py GUI 使用：读取 retarget_map.yaml，把单自由度 URDF 关节角
映射为 Blender 姿态骨骼的 XYZ 欧拉角。

核心公式:
    sink_axis_angle = joint_angle(度) * sign + bias
随后按该轴的 limit 钳位，写入 pose[bone][axis]。

示例:
    import retarget
    m = retarget.load_retarget_map("retarget_map.yaml")
    pose = retarget.apply_retarget({"left_arm_pitch_joint": 30.0}, m)
    # -> {"upper_arm_fk.L": [-180.0, -30.0, 0.0], ...}  (未启用的骨骼不出现/保持 0)
"""

from collections import OrderedDict

AXIS_INDEX = {"x": 0, "y": 1, "z": 2}


def load_retarget_map(path):
    """加载 retarget_map.yaml，返回 {joint: {bone, axis, sign, bias, limit, enabled}} 的有序 dict。

    limit: (lo_deg, hi_deg)。若项缺省 enabled，视为 True。
    """
    import yaml

    with open(path, "r", encoding="utf-8") as f:
        data = yaml.safe_load(f) or {}

    raw = data.get("retarget") or {}

    result = OrderedDict()
    for joint, cfg in raw.items():
        if cfg is None:
            continue
        limit = cfg.get("limit", [-180.0, 180.0])
        if not isinstance(limit, (list, tuple)) or len(limit) < 2:
            limit = [-180.0, 180.0]
        result[joint] = {
            "bone": cfg.get("bone", ""),
            "axis": str(cfg.get("axis", "y")).lower(),
            "sign": float(cfg.get("sign", 1.0)),
            "bias": float(cfg.get("bias", 0.0)),
            "limit": (float(limit[0]), float(limit[1])),
            "enabled": bool(cfg.get("enabled", True)),
        }
    return result


def joint_limits_from_map(rt_map):
    """返回 {joint: (lo_deg, hi_deg)}，供 GUI 滑动条量程使用。"""
    return {
        joint: cfg["limit"]
        for joint, cfg in rt_map.items()
    }


def apply_retarget(joint_angles_deg, rt_map):
    """把 {joint: 角度(度)} 映射为 Blender 姿态 {bone: [x, y, z]}。

    同一骨骼多个轴会叠加累加（如 head 骨 x/y/z）。未在 map 中、或 disabled 的
    关节被忽略。返回 OrderedDict，bone 键按首次出现顺序。
    """
    pose = OrderedDict()

    for joint, cfg in rt_map.items():
        if not cfg.get("enabled", True):
            continue
        if joint not in joint_angles_deg:
            continue

        deg = joint_angles_deg[joint]
        if deg is None:
            continue

        bone = cfg.get("bone", "")
        axis = cfg.get("axis", "y")
        sign = cfg.get("sign", 1.0)
        bias = cfg.get("bias", 0.0)
        lo, hi = cfg.get("limit", (-180.0, 180.0))

        raw = deg * sign + bias
        clamped = max(lo, min(hi, raw))

        idx = AXIS_INDEX.get(axis, 0)
        angles = pose.setdefault(bone, [0.0, 0.0, 0.0])
        angles[idx] += clamped

    return pose


def rad_to_deg(rad):
    return rad * 180.0 / 3.141592653589793


def apply_retarget_rad(joint_angles_rad, rt_map):
    """与 apply_retarget 相同，但输入为弧度；输出为骨骼欧拉角（度，供 Blender set_pose 使用）。

    map 中的 sign/bias/limit 均按『度』定义，因此先把弧度关节角换算为度再映射。
    """
    deg_map = {j: rad_to_deg(v) for j, v in joint_angles_rad.items()}
    return apply_retarget(deg_map, rt_map)


# ============================ 独立运行的自检 ============================

if __name__ == "__main__":
    import os
    import sys

    here = os.path.dirname(os.path.abspath(__file__))
    path = os.path.join(here, "config", "retarget_map.yaml")

    if not os.path.exists(path):
        print(f"未找到映射文件: {path}")
        sys.exit(1)

    m = load_retarget_map(path)
    print(f"加载到 {len(m)} 个映射项")

    # 冒烟：全零 + 一个正角度
    zero = {j: 0.0 for j in m}
    p = apply_retarget(zero, m)
    print("全零 ->", p)
    assert all(abs(v) < 1e-6 for vals in p.values() for v in vals) or not p, "全零应接近 0"

    sample = dict(zero)
    sample["left_arm_pitch_joint"] = 30.0
    sample["left_arm_roll_joint"] = 45.0
    sample["head_yaw_joint"] = 90.0  # 应被 limit 截断到 60
    p2 = apply_retarget(sample, m)
    print("示例 ->", p2)

    bone = m["left_arm_pitch_joint"]["bone"]
    axis_i = AXIS_INDEX[m["left_arm_pitch_joint"]["axis"]]
    assert abs(p2[bone][axis_i] - (-30.0)) < 1e-6, "sign=-1 应得 -30"
    head_i = AXIS_INDEX[m["head_yaw_joint"]["axis"]]
    assert abs(p2["head"][head_i] - 60.0) < 1e-6, "limit 截断应得 60"

    print("自检通过 ✔")