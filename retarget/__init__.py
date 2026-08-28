"""
retarget 包：Pikachu URDF(V025) → Blender 骨架 重定向映射逻辑（纯 Python，无 bpy 依赖）。

把 `retarget.py` 作为包导出，使
    import retarget as retarget_mod
    from retarget import AXIS_INDEX
两种导入方式都可用。config/ 下存放各目标骨架的映射 yaml。
"""

from .retarget import (
    AXIS_INDEX,
    apply_retarget,
    apply_retarget_rad,
    joint_limits_from_map,
    load_retarget_map,
    rad_to_deg,
)

__all__ = [
    "AXIS_INDEX",
    "apply_retarget",
    "apply_retarget_rad",
    "joint_limits_from_map",
    "load_retarget_map",
    "rad_to_deg",
]