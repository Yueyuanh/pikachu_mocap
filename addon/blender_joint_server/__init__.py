bl_info = {
    "name": "Skeleton Server",
    "author": "yue yue",
    "version": (1,0,0),
    "blender": (4,0,0),
    "location": "View3D > Sidebar",
    "category": "Animation",
}

import bpy
import os
from math import degrees

from . import server
from . import rig_sync


# ==========================
# 可驱动的 URDF 骨架多选（面板勾选集合）
# ==========================

def _is_urdf_armature(ob):
    """URDF importer 命名的骨架：骨名含 `.revolute.bone` 或 `.fixed.bone`。"""
    if ob is None or ob.type != 'ARMATURE':
        return False
    for b in ob.data.bones:
        if ".revolute.bone" in b.name or ".fixed.bone" in b.name:
            return True
    return False


def _sync_urdf_targets(scene):
    """把场景里现存的 URDF 骨架同步进勾选集合（新出现的自动加入，默认勾选；删除的移除）。"""
    coll = scene.skserver_urdf_targets
    present = [ob.name for ob in bpy.data.objects if _is_urdf_armature(ob)]
    existing = {t.name for t in coll}
    for name in present:
        if name not in existing:
            t = coll.add()
            t.name = name
            t.enabled = True
    drop = [i for i, t in enumerate(coll) if t.name not in present]
    for i in reversed(drop):
        coll.remove(i)


class SKSERVER_URDFTarget(bpy.types.PropertyGroup):
    name: bpy.props.StringProperty(name="Armature")
    enabled: bpy.props.BoolProperty(name="Enable", default=True)

# ==========================
# URDF 重定向目标骨（可选适配）
# ==========================

_URDF_TARGET_BONES = [
    # (URDF 关节, 目标骨骼, 轴)
    ("head_yaw_joint", "head", "z"),
    ("head_pitch_joint", "head", "y"),
    ("head_roll_joint", "head", "x"),
    ("left_arm_pitch_joint", "upper_arm_fk.L", "y"),
    ("left_arm_roll_joint", "upper_arm_fk.L", "x"),
    ("left_arm_yaw_joint", "upper_arm_fk.L", "z"),
    ("left_elbow_ankle_joint", "forearm_fk.L", "y"),
    ("right_arm_pitch_joint", "upper_arm_fk.R", "y"),
    ("right_arm_roll_joint", "upper_arm_fk.R", "x"),
    ("right_arm_yaw_joint", "upper_arm_fk.R", "z"),
    ("right_elbow_ankle_joint", "forearm_fk.R", "y"),
    ("left_hip_pitch_joint", "hips", "y"),
    ("left_hip_roll_joint", "hips", "x"),
    ("left_hip_yaw_joint", "hips", "z"),
    ("left_knee_joint", "foot_ik.L", "x"),
    ("left_ankle_joint", "foot_ik.L", "y"),
    ("right_hip_pitch_joint", "hips", "y"),
    ("right_hip_roll_joint", "hips", "x"),
    ("right_hip_yaw_joint", "hips", "z"),
    ("right_knee_joint", "foot_ik.R", "x"),
    ("right_ankle_joint", "foot_ik.R", "y"),
]


def check_urdf_target_bones(arm):
    """返回 (存在列表, 缺失列表)，供面板核对 retarget_map.yaml 的目标骨是否在 rig 中。"""
    if arm is None:
        return [], _URDF_TARGET_BONES
    existing = {b.name for b in arm.pose.bones}
    found = [(j, b, a) for (j, b, a) in _URDF_TARGET_BONES if b in existing]
    missing = [(j, b, a) for (j, b, a) in _URDF_TARGET_BONES if b not in existing]
    return found, missing


# ==========================
# 演示模式（隐藏骨骼与游标）
# ==========================

def _apply_demo_overlay(on):
    """on=True: 演示模式，隐藏所有 3D 视口的游标与骨骼；on=False: 恢复显示。"""
    show = not on
    try:
        wm = bpy.context.window_manager
        if wm is None:
            return
        for win in wm.windows:
            for area in win.screen.areas:
                if area.type == 'VIEW_3D':
                    for sp in area.spaces:
                        if sp.type == 'VIEW_3D':
                            sp.overlay.show_cursor = show
                            sp.overlay.show_bones = show
    except Exception:
        pass


def _demo_mode_update(self, context):
    _apply_demo_overlay(self.skserver_demo_mode)


# ==========================
# Log helpers
# ==========================

def _ensure_server_logs():
    if not hasattr(server, "log_messages"):
        server.log_messages = []
    if not hasattr(server, "add_log"):
        def _fallback_add_log(message):
            text = str(message)
            server.log_messages.append(text)
            if len(server.log_messages) > 50:
                del server.log_messages[:-50]
            if hasattr(server, "_mark_state_dirty"):
                server._mark_state_dirty()
        server.add_log = _fallback_add_log
    return server.log_messages


# ==========================
# Operator
# ==========================

class SKSERVER_OT_start(bpy.types.Operator):

    bl_idname = "skserver.start"
    bl_label = "Start Skeleton Server"

    def execute(self, context):

        server.start_server()

        if not bpy.app.timers.is_registered(rig_sync.blender_loop):
            bpy.app.timers.register(rig_sync.blender_loop, persistent=True)

        return {'FINISHED'}


class SKSERVER_OT_stop(bpy.types.Operator):

    bl_idname = "skserver.stop"
    bl_label = "Stop Skeleton Server"

    def execute(self, context):

        server.stop_server()

        if bpy.app.timers.is_registered(rig_sync.blender_loop):
            bpy.app.timers.unregister(rig_sync.blender_loop)

        return {'FINISHED'}


class SKSERVER_OT_reset(bpy.types.Operator):
    """一键 reset（同 Qt reset_all）：所有角色骨骼回 rest、对象位置回原始安装位置、旋转绝对归 0。"""

    bl_idname = "skserver.reset_all"
    bl_label = "Reset All"
    bl_description = "骨骼回 rest + 位置回原始安装点 + 旋转归 0（与 Qt 的 reset 一致）"

    def execute(self, context):

        rig_sync.reset_blender()

        return {'FINISHED'}


# ==========================
# Panel
# ==========================

class SKSERVER_PT_panel(bpy.types.Panel):

    bl_label = "Skeleton Server"

    bl_idname = "SKSERVER_PT_panel"

    bl_space_type = 'VIEW_3D'
    bl_region_type = 'UI'
    bl_category = "Skeleton"

    def draw(self, context):

        layout = self.layout

        # 演示模式：开→隐藏骨骼与游标；关→恢复显示
        layout.prop(context.scene, "skserver_demo_mode", text="演示模式", toggle=True)
        if context.scene.skserver_demo_mode:
            layout.label(text="→ 骨骼与游标已隐藏", icon="HIDE_OFF")
        layout.separator()

        layout.operator("skserver.start")
        layout.operator("skserver.stop")
        layout.operator("skserver.reset_all")

        layout.separator()

        if server.server_running:
            layout.label(text="Server: Running", icon="CHECKMARK")
        else:
            layout.label(text="Server: Stopped", icon="CANCEL")

        if server.client_connected:
            layout.label(text="Client: Connected", icon="LINKED")
        else:
            layout.label(text="Client: Waiting", icon="UNLINKED")

        layout.separator()

        arm = rig_sync.get_armature()

        if arm:
            layout.label(text=f"Armature: {arm.name}")
        else:
            layout.label(text="Armature: None")

        if hasattr(rig_sync, "get_active_pose_bone"):
            pb = rig_sync.get_active_pose_bone(context)
        else:
            pb = getattr(context, "active_pose_bone", None)

        if pb:
            if hasattr(rig_sync, "get_bone_angles"):
                x, y, z = rig_sync.get_bone_angles(pb)
            else:
                e = pb.rotation_euler
                x, y, z = degrees(e.x), degrees(e.y), degrees(e.z)
            layout.label(text=f"Active Bone: {pb.name}")
            layout.label(text=f"X: {x:.1f}  Y: {y:.1f}  Z: {z:.1f}")
        else:
            layout.label(text="Active Bone: None")

        layout.separator()
        layout.label(text="Drive URDF armatures (multi-select:)", icon="CHECKBOX_HLT")
        _sync_urdf_targets(context.scene)
        coll = context.scene.skserver_urdf_targets
        if len(coll) == 0:
            layout.label(text="(no URDF armature found)", icon="INFO")
        for t in coll:
            layout.prop(t, "enabled", text=t.name, toggle=True)
        if len(coll) > 1:
            layout.label(text="→ 勾选的会被 set_urdf_pose 驱动", icon="INFO")

        layout.separator()

        # URDF 重定向目标骨核对
        arm = rig_sync.get_armature()
        found, missing = check_urdf_target_bones(arm)
        layout.label(text=f"URDF target bones: {len(found)} ok / {len(missing)} missing")
        if missing:
            layout.label(text="Missing (check retarget_map.yaml):", icon="ERROR")
            for j, b, a in missing[:10]:
                layout.label(text=f"  {j} -> {b} ({a})")
            if len(missing) > 10:
                layout.label(text=f"  ... and {len(missing)-10} more")

        layout.separator()
        layout.prop(context.scene, "skserver_show_logs", text="Logs", toggle=True,
                    icon="DISCLOSURE_TRI_RIGHT")
        if context.scene.skserver_show_logs:
            logs = _ensure_server_logs()[-8:]
            if not logs:
                layout.label(text="(no logs)")
            for line in logs:
                layout.label(text=str(line)[:80])

        layout.separator()
        layout.prop(context.scene, "skserver_show_paths", text="Paths", toggle=True,
                    icon="DISCLOSURE_TRI_RIGHT")
        if context.scene.skserver_show_paths:
            layout.label(text=f"Addon: {os.path.abspath(__file__)[:80]}")
            if hasattr(server, "__file__"):
                layout.label(text=f"Server: {os.path.abspath(server.__file__)[:80]}")
            if hasattr(rig_sync, "__file__"):
                layout.label(text=f"RigSync: {os.path.abspath(rig_sync.__file__)[:80]}")


classes = [
    SKSERVER_OT_start,
    SKSERVER_OT_stop,
    SKSERVER_OT_reset,
    SKSERVER_PT_panel
]


def register():

    for c in classes:
        bpy.utils.register_class(c)
    bpy.utils.register_class(SKSERVER_URDFTarget)
    bpy.types.Scene.skserver_urdf_targets = bpy.props.CollectionProperty(type=SKSERVER_URDFTarget)
    bpy.types.Scene.skserver_demo_mode = bpy.props.BoolProperty(
        name="演示模式", default=False, update=_demo_mode_update)
    # 折叠区（默认隐藏）：Logs / Paths
    bpy.types.Scene.skserver_show_logs = bpy.props.BoolProperty(
        name="Logs", default=False)
    bpy.types.Scene.skserver_show_paths = bpy.props.BoolProperty(
        name="Paths", default=False)

    # 自动注册消息处理循环，保证 request_*/set_urdf_* 等双向命令能被处理
    if not bpy.app.timers.is_registered(rig_sync.blender_loop):
        bpy.app.timers.register(rig_sync.blender_loop, persistent=True)
    logs = _ensure_server_logs()
    server.add_log("Addon registered")
    server.add_log(f"Addon path: {os.path.abspath(__file__)}")
    if hasattr(server, "__file__"):
        server.add_log(f"Server path: {os.path.abspath(server.__file__)}")
    if hasattr(rig_sync, "__file__"):
        server.add_log(f"RigSync path: {os.path.abspath(rig_sync.__file__)}")


def unregister():

    if hasattr(bpy.types.Scene, "skserver_demo_mode"):
        del bpy.types.Scene.skserver_demo_mode
    if hasattr(bpy.types.Scene, "skserver_show_logs"):
        del bpy.types.Scene.skserver_show_logs
    if hasattr(bpy.types.Scene, "skserver_show_paths"):
        del bpy.types.Scene.skserver_show_paths
    bpy.utils.unregister_class(SKSERVER_URDFTarget)
    for c in classes:
        bpy.utils.unregister_class(c)
