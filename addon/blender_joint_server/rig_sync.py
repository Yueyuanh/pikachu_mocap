import bpy
import json
from math import radians, degrees

from . import server


ARMATURE_NAME = "rig"

def _tag_redraw():

    wm = bpy.context.window_manager

    if wm is None:
        return

    for window in wm.windows:

        screen = window.screen

        if screen is None:
            continue

        for area in screen.areas:

            if area.type == 'VIEW_3D':
                area.tag_redraw()


def get_armature(armature_name=ARMATURE_NAME):

    if armature_name:
        arm = bpy.data.objects.get(armature_name)
        if arm and arm.type == 'ARMATURE':
            return arm

    obj = bpy.context.object
    if obj and obj.type == 'ARMATURE':
        return obj

    for obj in bpy.data.objects:
        if obj.type == 'ARMATURE':
            return obj

    return None


def _send_debug(message):
    try:
        if hasattr(server, "add_log"):
            server.add_log(message)
        server.send_message({
            "type": "debug",
            "message": str(message)
        })
    except Exception:
        pass


def ensure_pose_mode(arm):

    bpy.context.view_layer.objects.active = arm

    if bpy.context.object.mode != 'POSE':
        try:
            bpy.ops.object.mode_set(mode='POSE')
        except Exception as e:
            for window in bpy.context.window_manager.windows:
                screen = window.screen
                for area in screen.areas:
                    if area.type != 'VIEW_3D':
                        continue
                    for region in area.regions:
                        if region.type != 'WINDOW':
                            continue
                        override = {
                            "window": window,
                            "screen": screen,
                            "area": area,
                            "region": region,
                            "scene": bpy.context.scene,
                            "view_layer": bpy.context.view_layer,
                            "active_object": arm,
                            "object": arm
                        }
                        try:
                            bpy.ops.object.mode_set(override, mode='POSE')
                            break
                        except Exception:
                            pass
            if bpy.context.object.mode != 'POSE':
                print("Pose mode error:", e)
                return False

    return True


def get_pose_bone(armature_name, bone_name):

    arm = get_armature(armature_name)

    if arm is None:
        raise ValueError(f"Armature {armature_name} not found")

    ensure_pose_mode(arm)

    bone = arm.pose.bones.get(bone_name)

    if bone is None:
        raise ValueError(f"Bone {bone_name} not found")

    return arm, bone


# ==========================
# 设置骨骼
# ==========================

def set_joint(bone_name, axis, angle):
    try:
        arm, bone = get_pose_bone(ARMATURE_NAME, bone_name)
    except Exception as e:
        print("Set joint error:", e)
        _send_debug(f"Set joint error: {e}")
        return

    bone.rotation_mode = 'XYZ'
    e = bone.rotation_euler
    angle = radians(angle)
    if axis == "x":
        e.x = angle
    elif axis == "y":
        e.y = angle
    elif axis == "z":
        e.z = angle

    bone.rotation_euler = e


def set_pose(pose, armature_name=None):
    if not pose:
        return

    # 带 armature 名 → 驱动指定骨架（多皮肤支持）；否则沿用默认 get_armature()（rig）
    arm = get_armature(armature_name) if armature_name else get_armature()
    if arm is None:
        _send_debug("Set pose failed: armature not found.")
        return

    # 纯数据写入：pose.bones.rotation_euler 在任何模式都可写，无需进入 pose mode /
    # 抢占 active 物体。去掉 ensure_pose_mode() 以免实时驱动时状态栏
    # 「Armature: rig / Active Bone」在多个骨架间来回闪。
    for bone_name, angles in pose.items():
        if not isinstance(angles, (list, tuple)) or len(angles) < 3:
            continue
        bone = arm.pose.bones.get(bone_name)
        if bone is None:
            continue
        bone.rotation_mode = 'XYZ'
        e = bone.rotation_euler
        e.x = radians(angles[0])
        e.y = radians(angles[1])
        e.z = radians(angles[2])
        bone.rotation_euler = e


# ==========================
# UI helpers
# ==========================

def _is_bone_selected(pbone):
    """兼容各 Blender 版本检查骨骼选中状态。

    - Blender 4.4+：姿态模式下用 PoseBone.bone_select
    - 旧版本：Bone.select（仅在编辑/老版本可用）
    """
    try:
        return pbone.bone_select
    except AttributeError:
        pass
    try:
        return pbone.bone.select
    except AttributeError:
        pass
    return False


def get_active_pose_bone(context=None):

    ctx = context or bpy.context
    pb = getattr(ctx, "active_pose_bone", None)

    if pb:
        return pb

    obj = ctx.object
    if obj and obj.type == 'ARMATURE' and obj.pose:
        for b in obj.pose.bones:
            if _is_bone_selected(b):
                return b

    return None


def get_bone_angles(pbone):

    e = pbone.rotation_euler
    return (
        degrees(e.x),
        degrees(e.y),
        degrees(e.z)
    )


# ==========================
# 获取骨骼姿态
# ==========================

def get_pose():

    arm = get_armature()

    if arm is None:
        return {}

    pose = {}

    for bone in arm.pose.bones:

        e = bone.rotation_euler

        pose[bone.name] = [
            degrees(e.x),
            degrees(e.y),
            degrees(e.z)
        ]

    return pose


# ==========================
# 获取骨骼树
# ==========================

def get_bone_tree():

    arm = get_armature()

    if arm is None:
        return []

    bones = []

    for b in arm.data.bones:

        bones.append({
            "name": b.name,
            "parent": b.parent.name if b.parent else None
        })

    return bones


# ==========================
# 获取场景结构（用于 socket 检查 Blender 里的 URDF / 皮肤）
# ==========================

def get_scene_info():
    """返回 Blender 当前场景结构：所有对象的名称/类型/父级，以及每个骨架的各骨名称/父/head/tail。

    用途：让外部（Qt / 脚本）通过 socket 检查 Blender 里导入的 URDF 与皮肤 rig，
    从而核对 retarget_map.yaml / blender_urdf_map.yaml 里的目标骨是否存在。
    """
    objects = []
    armatures = []

    for ob in bpy.data.objects:
        objects.append({
            "name": ob.name,
            "type": ob.type,
            "parent": ob.parent.name if ob.parent else None,
            "children": len(ob.children),
        })
        if ob.type == 'ARMATURE':
            bones = []
            for b in ob.data.bones:
                bones.append({
                    "name": b.name,
                    "parent": b.parent.name if b.parent else None,
                    "head": [float(v) for v in b.head_local],
                    "tail": [float(v) for v in b.tail_local],
                })
            armatures.append({"name": ob.name, "bones": bones})

    # body 0 之外也看下空物体/集合名，帮助识别 URDF 导入结构
    return {
        "objects": objects,
        "armatures": armatures,
        "collections": [c.name for c in bpy.data.collections],
    }


def set_urdf_joint(armature_name, bone_name, axis, angle_rad):
    """驱动导入到 Blender 的 URDF 骨架中的某个骨：围绕其某轴旋转（弧度）。

    与 set_joint 的区别：set_joint 针对皮肤"rig"且角度单位度；
    这里允许指定任意骨架名、任意骨、弧度，用来驱动 Blender 里的 URDF。
    """
    try:
        arm, bone = get_pose_bone(armature_name, bone_name)
    except Exception as e:
        _send_debug(f"set_urdf_joint error: {e}")
        return
    bone.rotation_mode = 'XYZ'
    e = bone.rotation_euler
    if axis == "x":
        e.x = angle_rad
    elif axis == "y":
        e.y = angle_rad
    elif axis == "z":
        e.z = angle_rad
    bone.rotation_euler = e


def _is_urdf_target_enabled(name):
    """是否驱动该骨架：仅当它在插件面板（Skeleton Server → Drive URDF armatures）里被勾选。

    未在勾选集合中出现的骨架默认放行（True），这样面板尚未同步 / 旧版 addon 时也能工作。
    """
    scene = getattr(bpy.context, "scene", None)
    if scene is None or not hasattr(scene, "skserver_urdf_targets"):
        return True
    for t in scene.skserver_urdf_targets:
        if t.name == name:
            return bool(t.enabled)
    return True


def set_urdf_pose(armature_name, pose):
    """批量驱动 URDF 骨架：pose = {bone: [x,y,z](rad)}。

    只驱动插件面板里勾选（enabled）的目标骨架，未勾选整骨架忽略，
    从而保证 meshcat 与“被勾选的 Blender URDF”保持一致。
    """
    if not _is_urdf_target_enabled(armature_name):
        return
    try:
        arm = get_armature(armature_name)
    except Exception as e:
        _send_debug(f"set_urdf_pose error: {e}")
        return
    # pose.bones.rotation_euler 为纯数据写入，不需 pose mode / 抢占 active（避免闪）
    if arm is None:
        _send_debug(f"set_urdf_pose: armature '{armature_name}' not found")
        return
    for bone_name, angles in (pose or {}).items():
        bone = arm.pose.bones.get(bone_name)
        if bone is None:
            continue
        bone.rotation_mode = 'XYZ'
        e = bone.rotation_euler
        e.x, e.y, e.z = angles[0], angles[1], angles[2]
        bone.rotation_euler = e


# 记录每个 armature 对象首次收到的原始安装位置，pos 增量在其上叠加（reset pos=0 即回到原位置）
_orig_loc = {}


def set_base(armature_name, pos, rpy_deg):
    """移动整个 armature 对象(Object)。

    pos   = 相对原始安装位置的增量(世界)：目标 location = 原位置 + pos（首帧记得原位置）。
    rpy   = 绝对 XYZ 欧拉(度)，直接覆盖 rotation_euler。
    设 object.location/rotation_euler（非骨），整棵骨架随对象一起动，天然是世界位姿。
    """
    ob = bpy.data.objects.get(armature_name)
    if ob is None:
        _send_debug(f"set_base: 对象 '{armature_name}' 不存在（Qt 的角色名与 Blender 场景对象名对不上）")
        return
    if pos is None and rpy_deg is None:
        return
    if armature_name not in _orig_loc:
        _orig_loc[armature_name] = tuple(ob.location)
    if pos is not None:
        base = _orig_loc[armature_name]
        ob.location = (float(base[0]) + float(pos[0]),
                       float(base[1]) + float(pos[1]),
                       float(base[2]) + float(pos[2]))
    if rpy_deg is not None:
        ob.rotation_mode = 'XYZ'
        ob.rotation_euler = (radians(float(rpy_deg[0])),
                             radians(float(rpy_deg[1])),
                             radians(float(rpy_deg[2])))


def reset_blender(armature_name=None):
    """一键 reset（与 Qt reset_all 一致）：所有角色骨骼回 rest、对象位置回原始安装位置、旋转绝对归 0。

    armature_name=None 时重置场景里全部 ARMATURE；指定则只重置该对象。
    """
    if armature_name:
        names = [armature_name]
    else:
        names = [ob.name for ob in bpy.data.objects if ob.type == 'ARMATURE']
    for nm in names:
        ob = bpy.data.objects.get(nm)
        if ob is None or ob.type != 'ARMATURE':
            continue
        if not ensure_pose_mode(ob):
            continue
        for pb in ob.pose.bones:
            pb.rotation_mode = 'XYZ'
            pb.rotation_euler = (0.0, 0.0, 0.0)
        # 位置回原(增量0) + 旋转绝对归 0
        set_base(nm, [0.0, 0.0, 0.0], [0.0, 0.0, 0.0])
    bpy.context.view_layer.update()
    _send_debug("Reset all armatures to rest + origin")


# ==========================
# 获取骨骼变换
# ==========================

def _matrix_to_list(mat):
    return [list(row) for row in mat]


def get_bone_transforms(bone_names=None, debug=False):

    arm = get_armature()

    if arm is None:
        if debug:
            _send_debug("Armature not found for transforms.")
        return []

    if not ensure_pose_mode(arm):
        if debug:
            _send_debug("Failed to enter pose mode.")
        return []

    bpy.context.view_layer.update()

    transforms = []
    name_filter = None
    if bone_names:
        try:
            name_filter = set()
            for name in bone_names:
                bone = arm.pose.bones.get(name)
                while bone:
                    name_filter.add(bone.name)
                    bone = bone.parent
        except Exception:
            name_filter = None

    for bone in arm.pose.bones:
        if name_filter is not None and bone.name not in name_filter:
            continue

        parent = bone.parent
        if parent:
            local = parent.matrix.inverted_safe() @ bone.matrix
            parent_name = parent.name
        else:
            local = bone.matrix.copy()
            parent_name = None

        world_matrix = arm.matrix_world @ bone.matrix
        head = arm.matrix_world @ bone.head
        tail = arm.matrix_world @ bone.tail

        transforms.append({
            "name": bone.name,
            "parent": parent_name,
            "head": [float(v) for v in head],
            "tail": [float(v) for v in tail],
            "matrix": _matrix_to_list(world_matrix),
            "local_matrix": _matrix_to_list(local)
        })

    return transforms


# ==========================
# 处理消息
# ==========================

def handle_message(msg):

    try:

        data = json.loads(msg)

        if data["type"] == "set_joint":

            set_joint(
                data["bone"],
                data["axis"],
                data["angle"]
            )
        elif data["type"] == "set_pose":

            pose = data.get("pose") or {}
            set_pose(pose, data.get("armature"))

        elif data["type"] == "request_pose":

            pose = get_pose()

            server.send_message({
                "type": "pose",
                "data": pose
            })

        elif data["type"] == "request_bones":

            bones = get_bone_tree()

            server.send_message({
                "type": "bones",
                "data": bones
            })

        elif data["type"] == "request_transforms":

            requested = data.get("bones") or []
            transforms = get_bone_transforms(requested, debug=True)
            _send_debug(f"Send transforms: {len(transforms)} (requested {len(requested)} + ancestors)")

            server.send_message({
                "type": "transforms",
                "data": transforms
            })

        elif data["type"] == "request_scene":

            server.send_message({
                "type": "scene",
                "data": get_scene_info()
            })

        elif data["type"] == "set_urdf_joint":

            set_urdf_joint(
                data.get("armature", ""),
                data.get("bone", ""),
                data.get("axis", "y"),
                float(data.get("angle_rad", 0.0)),
            )

        elif data["type"] == "set_urdf_pose":

            set_urdf_pose(data.get("armature", ""), data.get("pose") or {})

        elif data["type"] == "set_base":

            set_base(data.get("armature", ""),
                     data.get("pos"),
                     data.get("rpy_deg"))

        elif data["type"] == "reset":

            reset_blender(data.get("armature") or None)

    except Exception as e:

        print("Message error:", e)
        _send_debug(f"Message error: {e}")


# ==========================
# 主循环
# ==========================

def blender_loop():

    changed = False
    while not server.msg_queue.empty():

        msg = server.msg_queue.get()

        handle_message(msg)
        changed = True

    # 统一刷新：每帧最多一次 view_layer.update()（各 set_* 不再各自调用，
    # 每帧只触发一次重算绑定网格，显著降低 Qt 实时驱动时 Blender 的卡顿）
    if changed:
        bpy.context.view_layer.update()

    if server.state_dirty:
        server.state_dirty = False
        if not changed:
            bpy.context.view_layer.update()
        _tag_redraw()

    return 0.02
