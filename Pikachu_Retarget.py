"""
Pikachu_Retarget.py — Pikachu URDF(V025) → Blender rig 重定向 + 动作播放工具。

界面风格参照 Pikachu_Mocap.py，去掉了摄像头/MediaPipe 识别，加入了：
  - Pikachu FK 骨骼直接控制（Bone 模式）
  - URDF 关节控制（URDF 模式）
  - NPZ 动作回放（NPZ 模式：文件选择 / 播放暂停 / 倍速 / 进度条），逐帧驱动 URDF 关节
  - 将 URDF 关节角映射到 Blender 皮肤骨骼（retarget_map.yaml）与 Blender 内 URDF（blender_urdf_map.yaml）

连接状态实时显示在 Qt 左栏。所有映射由 "Sync to Blender" 开关控制是否上行。

核心公式:  sink_axis_angle = joint_angle * sign + bias            (retarget_map.yaml)
"""

import sys
import socket
import json
import os
import math

import numpy as np
from PySide6.QtWidgets import *
from PySide6.QtCore import Qt, QTimer

import retarget as retarget_mod
from retarget import AXIS_INDEX

# urdfpy(1.x) 依赖 numpy 已移除的别名(np.float 等)；numpy>=1.24 需兼容垫片（须早于 URDF 导入）
try:
    import numpy as _np
    import warnings as _w
    with _w.catch_warnings():
        _w.simplefilter("ignore")
        for _name, _type in [('float', float), ('int', int), ('bool', bool),
                             ('object', object), ('str', str), ('unicode', str),
                             ('complex', complex)]:
            if not hasattr(_np, _name):
                setattr(_np, _name, _type)
except Exception:
    pass


BASE_DIR = os.path.dirname(os.path.abspath(__file__))
HOST = "127.0.0.1"
PORT = 9999

URDF_DIR = os.path.join(BASE_DIR, "urdf")
if URDF_DIR not in sys.path:
    sys.path.append(URDF_DIR)
if BASE_DIR not in sys.path:  # 确保 retarget/ 包可导入（不管从哪启动）
    sys.path.append(BASE_DIR)

RETARGET_CFG = os.path.join(BASE_DIR, "retarget", "config")
BLENDER_URDF_MAP_PATH = os.path.join(RETARGET_CFG, "blender_urdf_map.yaml")
# 皮肤骨架注册表（per-armature、带 name；rig / self_rig_v2 / self_rig_v3 统一在这里管理）
# —— 皮肤映射的唯一来源；不再加载 retarget_map.yaml / retarget_map_self_rig_v2/v3.yaml。
BLENDER_SKIN_MAP_PATH = os.path.join(RETARGET_CFG, "blender_skin_map.yaml")
# 自定义皮肤骨架目标（Blender 里的真实骨架名，用于诊断/打印）
SELF_RIG_V2_ARMATURE = "Pikacuh_skin_self_rig_v2"
DEFAULT_URDF_PATH = os.path.join(
    BASE_DIR, "urdf", "robot", "Pikachu_links", "default", "27dof", "pikachu_sample_links_27dof.urdf"
)
DEFAULT_NPZ_DIR = (
    "/home/finnox/Pikachu/PikachuRobot/pikachu_playground/mjlab/src/mjlab/mocap/npz"
)
if "PIKACHU_MOCAP_NPZ" in os.environ:
    DEFAULT_NPZ_DIR = os.environ["PIKACHU_MOCAP_NPZ"]

# Pikachu 皮肤 FK 直接控制骨骼（x/y/z 范围，度）
DIRECT_BONES = [
    ("head", (-20, 20)), ("neck", (-10, 10)), ("chest", (-10, 10)),
    ("torso", (-10, 10)), ("hips", (-10, 10)),
    ("shoulder.L", (-10, 10)), ("shoulder.R", (-10, 10)),
    ("ear.L", (-90, 90)), ("ear.R", (-90, 90)),
    ("upper_arm_fk.L", (-90, 90)), ("upper_arm_fk.R", (-90, 90)),
    ("forearm_fk.L", (-90, 90)), ("forearm_fk.R", (-90, 90)),
    ("hand_fk.L", (-90, 90)), ("hand_fk.R", (-90, 90)),
    ("foot_ik.L", (-50, 50)), ("foot_ik.R", (-50, 50)),
    ("toe.L", (-50, 50)), ("toe.R", (-50, 50)),
    ("tail", (-90, 90)),
]

# Pikacuh_skin_self_rig_v2 自定义皮肤骨架的骨骼（name, (lo,hi) 度）——用于直接控制这张皮肤，方便适配映射
V2_BONES = [
    ("base_link", (-30, 30)),
    ("head", (-60, 60)),
    ("arm_L", (-120, 120)), ("arm_pitch_L", (-120, 120)),
    ("arm_R", (-120, 120)), ("arm_pitch_R", (-120, 120)),
    ("hip_L", (-30, 30)),
    ("hip_pitch_L", (-120, 120)), ("hip_knee_L", (-120, 120)), ("hip_ankle_L", (-90, 90)),
    ("hip_R", (-30, 30)),
    ("hip_pitch_R", (-120, 120)), ("hip_knee_R", (-120, 120)), ("hip_ankle_R", (-90, 90)),
]

def v2_bone_limits_from_map(rt_map_v2, bones):
    """把 self_rig_v2.yaml 的逐关节 limit 聚合到每根骨各轴；yaml 未覆盖的骨/轴用 bones 兜底。

    返回 {bone: {"x":(lo,hi), "y":.., "z":..}}；骨骼同名同轴多关节取最宽并集。
    """
    lims = {name: {"x": None, "y": None, "z": None} for name, _ in bones}
    fallback = {name: lim for name, lim in bones}
    for cfg in (rt_map_v2 or {}).values():
        b = cfg.get("bone")
        ax = cfg.get("axis")
        if not b or ax not in ("x", "y", "z"):
            continue
        lo, hi = float(cfg["limit"][0]), float(cfg["limit"][1])
        d = lims.setdefault(b, {"x": None, "y": None, "z": None})
        prev = d[ax]
        d[ax] = (min(lo, prev[0]) if prev else lo, max(hi, prev[1]) if prev else hi)
    out = {}
    for b in lims:
        fb = fallback.get(b, (-90.0, 90.0))
        out[b] = {ax: (lims[b][ax] if lims[b][ax] is not None else fb) for ax in ("x", "y", "z")}
    return out


# 通用别名：对任意皮肤骨架的 joints 聚合逐骨逐轴 limit（不止 self_rig_v2）
bone_limits_from_map = v2_bone_limits_from_map


def load_skin_registry(path=None):
    """读 blender_skin_map.yaml → 皮肤骨架注册表列表。

    每条: {"name": Blender 骨架名, "joints": {URDF关节: {bone,axis,sign,bias,limit}},
           "bones": [直接操控的骨名…], "cfg": 合并后的 base 配置, "source_fbx": 可选离线骨源}。

    结构兼容新旧两种格式（老单条 meta["armature"]/meta["joints"] 也能读）。enabled=False 或
    解析失败返回空列表（调用方据此回退）。map 里没填 base 键时给单位默认。
    """
    path = path or BLENDER_SKIN_MAP_PATH
    try:
        import yaml
        with open(path, "r", encoding="utf-8") as f:
            meta = yaml.safe_load(f) or {}
        if not meta.get("enabled", True):
            return []
        arms = meta.get("armatures") or []
        if not arms:
            arm, joints = meta.get("armature"), meta.get("joints")
            if arm and joints:
                arms = [{"name": arm, "joints": joints}]
        out = []
        for a in arms:
            if not isinstance(a, dict) or not a.get("name"):
                continue
            bp = a.get("base_pos") or {}
            br = a.get("base_rot") or {}
            cfg = dict(bp)
            cfg.setdefault("pos_retarget", ["x", "y", "z"])
            cfg.update(br)
            cfg.setdefault("pos_scale", [1.0, 1.0, 1.0])
            cfg.setdefault("pos_dir", [1.0, 1.0, 1.0])
            cfg.setdefault("rot_retarget", ["x", "y", "z"])
            cfg.setdefault("rot_scale", [1.0, 1.0, 1.0])
            cfg.setdefault("rot_offset", [0.0, 0.0, 0.0])
            out.append({
                "name": a.get("name"),
                "joints": a.get("joints") or {},
                "bones": list(a.get("bones") or []),
                "cfg": cfg,
                "source_fbx": a.get("source_fbx", ""),
            })
        return out
    except Exception as e:
        print("[skin registry] load failed:", e)
        return []


# NPZ 关节 dof 列序（= twin_server.py 的 PIKACHU_JOINT_NAMES，14 列）→ 对应 URDF 关节名
NPZ_COLUMNS_TO_URDF = [
    ("left_hip_pitch_joint", False), ("left_hip_roll_joint", False), ("left_hip_yaw_joint", False),
    ("left_knee_joint", False), ("left_ankle_joint", False),
    ("right_hip_pitch_joint", False), ("right_hip_roll_joint", False), ("right_hip_yaw_joint", False),
    ("right_knee_joint", False), ("right_ankle_joint", False),
    ("left_arm_pitch_joint", False), ("left_arm_roll_joint", True),
    ("right_arm_pitch_joint", False), ("right_arm_roll_joint", True),
]
# arm_roll 90° 对正（右=π/2-v，左=-π/2-v）
IDX_LEFT_ARM_ROLL = 11
IDX_RIGHT_ARM_ROLL = 13


def npz_row_to_urdf(row):
    """把第 i 帧的 joint_pos 行(14) 转成 {urdf_joint: 弧度}。

    兼容列数 < 14 的动作文件（如纯腿部 10-dof pawn）：只映射前 len(row) 列。
    前 10 列恰为腿部子集，故 10-dof 文件也能正确加载。
    """
    out = {}
    for col, (urdf_name, is_arm_roll) in enumerate(NPZ_COLUMNS_TO_URDF):
        if col >= len(row):
            break
        v = float(row[col])
        # if is_arm_roll:
        #     if col == IDX_RIGHT_ARM_ROLL:
        #         v = math.pi / 2.0 - v
        #     elif col == IDX_LEFT_ARM_ROLL:
        #         v = -math.pi / 2.0 - v
        out[urdf_name] = v
    return out


# ============================ npz base (root 位姿) 支持 ============================
# npz root = body_pos_w[:,0]（世界位置）+ body_quat_w[:,0]（世界四元数, wxyz）。
#   播 base: pos 取“相对首帧增量”(叠到角色原本位置)；rot 取“绝对 XYZ 欧拉(度)”。
# 统一 XYZ 欧拉约定：meshcat euler_matrix('rxyz') 与 Blender rotation_mode='XYZ'
#   都是 intrinsic XYZ（复合矩阵 R = Rz·Ry·Rx），故 meshcat / Blender 三方一致。
UNIT_BASE_CFG = {"pos_scale": [1.0, 1.0, 1.0], "pos_dir": [1.0, 1.0, 1.0]}


def _quat_wxyz_to_mat(q):
    """wxyz 四元数 → 3x3 旋转矩阵（单位化）。"""
    q = np.asarray(q, dtype=float)
    n = np.linalg.norm(q)
    if n < 1e-8:
        return np.eye(3)
    w, x, y, z = q / n
    xx, yy, zz = x * x, y * y, z * z
    xy, xz, yz = x * y, x * z, y * z
    wx, wy, wz = w * x, w * y, w * z
    return np.array([
        [1 - 2 * (yy + zz), 2 * (xy - wz), 2 * (xz + wy)],
        [2 * (xy + wz), 1 - 2 * (xx + zz), 2 * (yz - wx)],
        [2 * (xz - wy), 2 * (yz + wx), 1 - 2 * (xx + yy)],
    ])


def _mat_to_euler_xyz_rad(R):
    """旋转矩阵 → intrinsic XYZ 欧拉(rad)。R = Rz(γ)·Ry(β)·Rx(α)。"""
    R = np.asarray(R, dtype=float)
    beta = math.atan2(-R[2, 0], math.sqrt(R[0, 0] ** 2 + R[1, 0] ** 2))
    if abs(abs(beta) - math.pi / 2.0) > 1e-6:
        alpha = math.atan2(R[2, 1], R[2, 2])
        gamma = math.atan2(R[1, 0], R[0, 0])
    else:
        # 万向锁：令 gamma=0，α 由 (R[0,1],R[0,2]) 求
        alpha = math.atan2(R[0, 1], R[0, 2])
        gamma = 0.0
    return alpha, beta, gamma


def quat_to_rpy_xyz_deg(q):
    a, b, g = _mat_to_euler_xyz_rad(_quat_wxyz_to_mat(q))
    return [math.degrees(a), math.degrees(b), math.degrees(g)]


def npz_base_to(pos_t, quat_t, pos0, cfg=None):
    """换算每帧 base 的「原始」值（不做轴置换/缩放，留给各 armature 按自身 cfg 应用）。
    pos_t/pos0=[x,y,z](世界)，quat_t=[wxyz]。
    返回 (pos_delta, rpy_deg)：pos_delta=相对首帧位移(m)，rpy=绝对 XYZ 欧拉(度)。
    """
    pos_delta = (np.asarray(pos_t, float) - np.asarray(pos0, float))
    rpy_deg = quat_to_rpy_xyz_deg(quat_t)
    return [float(v) for v in pos_delta], rpy_deg


AXIS_IDX = {"x": 0, "y": 1, "z": 2}


def _retarget_axes(v, rt):
    """按 retarget(元素为源轴字母，如 [y,x,z]) 把 v=[x,y,z] 置换成目标 [x,y,z]。
    out[目标x]=v[retarget[0]]，out[目标y]=v[retarget[1]]，out[目标z]=v[retarget[2]]。
    retarget 缺失/非法则恒等。"""
    if not rt or len(rt) != 3:
        return list(v)
    try:
        idx = [AXIS_IDX[str(c).lower()] for c in rt]
    except (KeyError, TypeError):
        return list(v)
    return [v[idx[0]], v[idx[1]], v[idx[2]]]


def _base_pos_for_cfg(pos_delta, cfg):
    """该 armature 应用 own base 配置后的 Blender 目标位置增量：
    先按 pos_retarget 换轴，再逐轴 * pos_scale * pos_dir。"""
    tmp = _retarget_axes(pos_delta, cfg.get("pos_retarget"))
    scale = cfg.get("pos_scale", [1.0, 1.0, 1.0])
    direc = cfg.get("pos_dir", [1.0, 1.0, 1.0])
    return [tmp[i] * scale[i] * direc[i] for i in range(3)]


def _blender_base_rpy(cfg, rpy_deg):
    """npz 的绝对 rpy → Blender 目标 xyz 欧拉(度)：
    先按 rot_retarget 换轴，再 rot_scale 逐轴重映射(+1/-1) + rot_offset 该模型默认安装朝向。
    """
    tmp = _retarget_axes(rpy_deg, cfg.get("rot_retarget"))
    s = cfg.get("rot_scale", [1.0, 1.0, 1.0])
    off = cfg.get("rot_offset", [0.0, 0.0, 0.0])
    return [off[i] + s[i] * tmp[i] for i in range(3)]


def _blender_rpy_direct(cfg, rpy_deg):
    """手动滑块路径：滑块值即目标轴（不换轴），仅叠 rot_scale/rot_offset（安装朝向）。"""
    s = cfg.get("rot_scale", [1.0, 1.0, 1.0])
    off = cfg.get("rot_offset", [0.0, 0.0, 0.0])
    return [off[i] + s[i] * rpy_deg[i] for i in range(3)]


def _rot_offset(cfg):
    """台词模型默认 rest 朝向（XYZ 欧拉，度）；reset 归位到它而非 0。"""
    return list(cfg.get("rot_offset", [0.0, 0.0, 0.0]))


def _load_base_cfg_from_file(path):
    """读 yaml 顶层 base: 块；无则返回单位配置。"""
    try:
        import yaml
        with open(path, "r", encoding="utf-8") as f:
            meta = yaml.safe_load(f) or {}
        return dict(meta.get("base") or UNIT_BASE_CFG)
    except Exception:
        return dict(UNIT_BASE_CFG)


def _base_cfg_for_burdf(meta, name):
    """从 blender_urdf_map.yaml 取某 armature 的 base 块；无则单位配置。

    兼容两种写法：
      - 旧式  base:  { pos_scale, pos_dir, rot_scale, rot_offset }
      - 新式  base_pos {pos_retarget,pos_scale,pos_dir} + base_rot {rot_retarget,rot_scale,rot_offset}
        因为每个模型安装朝向/方向轴可能不一致，用 base_pos/base_rot 分开定向；
        pos_retarget/rot_retarget 为声明字段，实际逐轴方向靠 pos_dir/rot_scale(±1)。
    返回统一 cfg：{pos_scale, pos_dir, rot_scale, rot_offset}（缺省单位）。
    """
    for a in (meta or {}).get("armatures") or []:
        if a.get("name") != name:
            continue
        if a.get("base"):
            return dict(a["base"])
        if a.get("base_pos") or a.get("base_rot"):
            bp = a.get("base_pos") or {}
            br = a.get("base_rot") or {}
            return {
                "pos_retarget": bp.get("pos_retarget"),
                "pos_scale": bp.get("pos_scale", [1.0, 1.0, 1.0]),
                "pos_dir": bp.get("pos_dir", [1.0, 1.0, 1.0]),
                "rot_retarget": br.get("rot_retarget"),
                "rot_scale": br.get("rot_scale", [1.0, 1.0, 1.0]),
                "rot_offset": br.get("rot_offset", [0.0, 0.0, 0.0]),
            }
    return dict(UNIT_BASE_CFG)


# ============================ Socket 客户端 ============================

class BlenderClient:

    def __init__(self, on_message=None, on_connect=None):

        self.sock = None
        self.connected = False
        self.on_message = on_message
        self.on_connect = on_connect
        self._buffer = ""

        self._retry_timer = QTimer()
        self._retry_timer.setInterval(1000)
        self._retry_timer.timeout.connect(self._try_connect)

        self._recv_timer = QTimer()
        self._recv_timer.setInterval(30)
        self._recv_timer.timeout.connect(self._poll_socket)

        self._try_connect()

    def send(self, data):
        if not self.connected or self.sock is None:
            return
        try:
            self.sock.sendall((json.dumps(data) + "\n").encode())
        except Exception:
            self._handle_disconnect()

    def set_pose(self, pose, armature=None):
        if not pose:
            return
        msg = {"type": "set_pose", "pose": pose}
        if armature:
            msg["armature"] = armature
        self.send(msg)

    def set_urdf_pose(self, armature, pose):
        if not pose:
            return
        self.send({"type": "set_urdf_pose", "armature": armature, "pose": pose})

    def set_base(self, armature, pos, rpy_deg):
        """把 armature 对象根位姿设为 pos(增量,世界) + rpy(绝对 XYZ 欧拉,度)。"""
        if pos is None and rpy_deg is None:
            return
        self.send({"type": "set_base", "armature": armature,
                   "pos": list(pos) if pos is not None else [0.0, 0.0, 0.0],
                   "rpy_deg": list(rpy_deg) if rpy_deg is not None else [0.0, 0.0, 0.0]})

    def request_bones(self):
        self.send({"type": "request_bones"})

    def request_scene(self):
        self.send({"type": "request_scene"})

    def _try_connect(self):
        if self.connected:
            return
        try:
            self.sock = socket.socket(socket.AF_INET, socket.SOCK_STREAM)
            self.sock.settimeout(1.0)
            self.sock.connect((HOST, PORT))
            self.sock.setblocking(False)
            self.connected = True
            self._retry_timer.stop()
            self._recv_timer.start()
            print("Connected to Blender:", HOST, PORT)
            if self.on_connect:
                self.on_connect(True)
        except Exception:
            if not self._retry_timer.isActive():
                self._retry_timer.start()

    def _poll_socket(self):
        if not self.connected or self.sock is None:
            return
        try:
            data = self.sock.recv(4096)
        except BlockingIOError:
            return
        except Exception:
            self._handle_disconnect()
            return
        if not data:
            self._handle_disconnect()
            return
        self._buffer += data.decode(errors="replace")
        while "\n" in self._buffer:
            line, self._buffer = self._buffer.split("\n", 1)
            line = line.strip()
            if not line:
                continue
            try:
                msg = json.loads(line)
            except Exception:
                continue
            if self.on_message:
                self.on_message(msg)

    def _handle_disconnect(self, retry=True):
        was = self.connected
        self.connected = False
        self._recv_timer.stop()
        if self.sock is not None:
            try:
                self.sock.close()
            except Exception:
                pass
            self.sock = None
        if was and self.on_connect:
            self.on_connect(False)
        if retry and not self._retry_timer.isActive():
            self._retry_timer.start()


# ============================ 通用滑动条控件 ============================

class URDFJointWidget(QWidget):

    def __init__(self, name, lower, upper, on_change, on_sync_change,
                 use_degree=True, sync_checked=True):
        super().__init__()
        self.name = name
        self.lower = lower
        self.upper = upper
        self.on_change = on_change
        self.on_sync_change = on_sync_change
        self.use_degree = use_degree
        self.sync_checked = sync_checked

        layout = QVBoxLayout()
        layout.setContentsMargins(0, 0, 0, 0)
        layout.setSpacing(2)

        header = QHBoxLayout()
        header.setContentsMargins(0, 0, 0, 0)
        header.setSpacing(4)
        self.sync_checkbox = QCheckBox()
        self.sync_checkbox.setChecked(bool(self.sync_checked))
        self.sync_checkbox.toggled.connect(self._on_sync_toggled)
        title = QLabel(name)
        title.setStyleSheet("font-size: 12px; font-weight: 600;")
        header.addWidget(self.sync_checkbox, 0)
        header.addWidget(title, 1)
        # 角度显示 + (limits) 放在关节名称旁
        self.value_label = QLabel()
        self.value_label.setFixedWidth(170)
        self.value_label.setAlignment(Qt.AlignLeft | Qt.AlignVCenter)
        self.value_label.setStyleSheet("font-size: 11px;")
        self.update_value_label(0.0)
        header.addWidget(self.value_label, 0)
        layout.addLayout(header)

        slider_lay = QHBoxLayout()
        slider_lay.setContentsMargins(24, 0, 0, 0)
        self.slider = QSlider(Qt.Horizontal)
        self.slider.setMinimumWidth(120)
        if self.use_degree:
            a, b = int(lower * 180 / math.pi), int(upper * 180 / math.pi)
        else:
            a, b = int(lower * 100), int(upper * 100)
        self.slider.setMinimum(a)
        self.slider.setMaximum(b)
        self.slider.setValue(0)
        self.slider.valueChanged.connect(self._on_slider_change)
        slider_lay.addWidget(self.slider, 1)
        layout.addLayout(slider_lay)
        self.setLayout(layout)

    def _on_slider_change(self, value):
        angle_rad = value * math.pi / 180.0 if self.use_degree else value / 100.0
        self.update_value_label(angle_rad)
        if self.on_change:
            self.on_change(self.name, angle_rad)

    def _on_sync_toggled(self, checked):
        self.sync_checked = bool(checked)
        if self.on_sync_change:
            self.on_sync_change(self.name, checked)

    def update_value_label(self, angle_rad):
        angle_deg = angle_rad * 180.0 / math.pi
        ld, ud = self.lower * 180.0 / math.pi, self.upper * 180.0 / math.pi
        self.value_label.setText(f"{angle_deg:.1f}°  ({ld:.0f}~{ud:.0f})")

    def set_use_degree(self, use_degree):
        if self.use_degree == use_degree:
            return
        self.use_degree = use_degree
        cur_rad = self.slider.value() * math.pi / 180.0 if not use_degree else self.slider.value() / 100.0
        a, b = (int(self.lower * 180 / math.pi), int(self.upper * 180 / math.pi)) if use_degree \
            else (int(self.lower * 100), int(self.upper * 100))
        self.slider.blockSignals(True)
        self.slider.setMinimum(a)
        self.slider.setMaximum(b)
        self.slider.setValue(int(cur_rad * 180.0 / math.pi) if use_degree else int(cur_rad * 100.0))
        self.slider.blockSignals(False)
        self.update_value_label(cur_rad)

    def set_angle(self, angle_rad):
        v = int(angle_rad * 180.0 / math.pi) if self.use_degree else int(angle_rad * 100.0)
        self.slider.blockSignals(True)
        self.slider.setValue(v)
        self.slider.blockSignals(False)
        self.update_value_label(angle_rad)


class BoneJointWidget(QWidget):
    """单根 FK 骨的三轴滑动条（直接控制 Pikachu 骨骼）。"""

    def __init__(self, name, limits, on_change):
        """三轴滑动条, x/y/z 各轴独立 limit（limits = {"x":(lo,hi),"y":..,"z":..}）。
        兼容旧式单 (lo,hi) 元组：三轴共用该范围。"""
        super().__init__()
        if not isinstance(limits, dict):
            limits = {axis: limits for axis in ("x", "y", "z")}
        self.name = name
        self.on_change = on_change
        lay = QVBoxLayout()
        lay.setContentsMargins(0, 0, 0, 0)
        self.title = QLabel(f"Bone: {name}")
        self.title.setStyleSheet("font-size: 13px; font-weight: 600;")
        lay.addWidget(self.title)
        self.labels = {}
        for axis in ["x", "y", "z"]:
            row = QHBoxLayout()
            row.setContentsMargins(0, 0, 0, 0)
            lab = QLabel(axis.upper())
            lab.setFixedWidth(20)
            row.addWidget(lab)
            sl = QSlider(Qt.Horizontal)
            lo, hi = limits.get(axis, (-90, 90))
            sl.setRange(int(lo), int(hi))
            sl.setValue(0)
            sl.valueChanged.connect(lambda v, ax=axis: self._changed(ax, v))
            row.addWidget(sl, 1)
            lb = QLabel("0")
            lb.setFixedWidth(40)
            lb.setAlignment(Qt.AlignRight | Qt.AlignVCenter)
            row.addWidget(lb)
            lay.addLayout(row)
            self.labels[axis] = (sl, lb)
        self.setLayout(lay)

    def set_limits(self, limits):
        """切换骨骼时更新三轴各自范围（limit 来自 self_rig_v2.yaml 聚合/兜底）。"""
        for axis, (sl, lb) in self.labels.items():
            lo, hi = limits.get(axis, (-90, 90))
            sl.blockSignals(True)
            sl.setRange(int(lo), int(hi))
            lb.setText(str(sl.value()))
            sl.blockSignals(False)

    def set_bone_name(self, name):
        """选中列表中其它骨骼时更新标题（否则会一直显示首个骨骼名）。"""
        self.name = name
        self.title.setText(f"Bone: {name}")

    def _changed(self, axis, val):
        self.labels[axis][1].setText(str(val))
        if self.on_change:
            self.on_change(self.name, axis, val)

    def set_angles(self, angles):
        for i, axis in enumerate(["x", "y", "z"]):
            self.labels[axis][0].blockSignals(True)
            self.labels[axis][0].setValue(int(angles[i]) if angles else 0)
            self.labels[axis][0].blockSignals(False)


# ============================ 重定向映射面板 ============================

class RetargetPanel(QWidget):

    def __init__(self, rt_maps, on_reload):
        """rt_maps: {"rig": retarget_map, "self_rig_v2": retarget_map_self_rig_v2} 等多张映射，
        顶部下拉切换显示哪一张。"""
        super().__init__()
        self._rt_maps = dict(rt_maps) or {}
        self.on_reload = on_reload
        self.rt_map = self._current_map()
        lay = QVBoxLayout()
        lay.setContentsMargins(0, 0, 0, 0)
        # 标题行(含 map_combo / Reload)由外层 CollapsibleFrame 标题栏承载，这里只留 body
        self.map_combo = QComboBox()
        self._fill_map_combo()
        self.map_combo.currentIndexChanged.connect(self._rebuild_from_combo)

        self.table = QTableWidget(0, 6)
        self.table.setHorizontalHeaderLabels(["Joint", "Bone", "Axis", "Sign", "Bias", "Limit"])
        self.table.verticalHeader().setVisible(False)
        self.table.setEditTriggers(QAbstractItemView.NoEditTriggers)
        self.table.horizontalHeader().setStretchLastSection(True)
        lay.addWidget(self.table)

        self.pose_label = QLabel("Bone pose: (none)")
        self.pose_label.setWordWrap(True)
        lay.addWidget(self.pose_label)
        self.setLayout(lay)
        self.rebuild(self.rt_map)

    def _fill_map_combo(self):
        self.map_combo.clear()
        for key in self._rt_maps:
            self.map_combo.addItem(key, key)
        # 默认选中最新的（yaml 末尾）骨架（如 self_rig_v3）
        if self.map_combo.count():
            self.map_combo.setCurrentIndex(self.map_combo.count() - 1)

    def _current_map(self):
        key = self.map_combo.currentData() if hasattr(self, "map_combo") else None
        return self._rt_maps.get(key) if key else (next(iter(self._rt_maps.values()), None))

    def _rebuild_from_combo(self):
        self.rt_map = self._current_map()
        self.rebuild(self.rt_map)

    def set_maps(self, rt_maps):
        """Reload 后刷新映射数据，保留当前下拉选择。"""
        self._rt_maps = dict(rt_maps) or {}
        cur = self.map_combo.currentData()
        self.map_combo.blockSignals(True)
        self._fill_map_combo()
        if cur in self._rt_maps:
            self.map_combo.setCurrentIndex([k for k in self._rt_maps].index(cur))
        self.map_combo.blockSignals(False)
        self._rebuild_from_combo()

    def rebuild(self, rt_map):
        self.rt_map = rt_map
        self.table.setRowCount(len(rt_map))
        for r, (joint, c) in enumerate(rt_map.items()):
            lo, hi = c["limit"]
            vals = [joint, c["bone"], c["axis"], f"{c['sign']:+.0f}",
                    f"{c['bias']:.1f}", f"[{lo:.0f},{hi:.0f}]"]
            for col, v in enumerate(vals):
                self.table.setItem(r, col, QTableWidgetItem(v))

    def update_pose(self, bone_pose):
        if not bone_pose:
            self.pose_label.setText("Bone pose: (none)")
            return
        parts = [f"{b}=[{', '.join(f'{a:.1f}' for a in v)}]" for b, v in bone_pose.items()]
        self.pose_label.setText("Bone pose: " + "  ".join(parts))


class BlenderUrdfPanel(QWidget):
    """把 URDF 关节角映射到 Blender 内导入的 URDF 骨骼轴（blender_urdf_map.yaml）。"""

    def __init__(self, meta, on_reload):
        super().__init__()
        self.meta = meta  # {"enabled": bool, "armatures": [{name, joints:{urdf_joint:{bone,axis,sign,bias}}}]}
        self.on_reload = on_reload
        lay = QVBoxLayout()
        lay.setContentsMargins(0, 0, 0, 0)
        # 标题行(含 Reload)由外层 CollapsibleFrame 标题栏承载，这里只留 body
        self.info = QLabel("")
        self.info.setWordWrap(True)
        lay.addWidget(self.info)
        self.setLayout(lay)
        self.rebuild(meta)

    def rebuild(self, meta):
        self.meta = meta or {}
        arms = self._armature_list(meta)
        if not arms:
            self.info.setText("armature: (无)\n映射关节: 0 个")
            return
        lines = [f"共 {len(arms)} 套 URDF rig（勾选在 Blender 面板决定）:"]
        for a in arms:
            lines.append(f"  · {a.get('name', '?')}: {len(a.get('joints', {}))} 关节")
        self.info.setText("\n".join(lines))

    @staticmethod
    def _armature_list(meta):
        """兼容新旧两种格式：
        - 新：meta["armatures"] = [{name, joints}]
        - 旧：meta["armature"] + meta["joints"] 单条
        """
        arms = (meta or {}).get("armatures") or []
        if not arms:
            arm = (meta or {}).get("armature")
            joints = (meta or {}).get("joints")
            if arm and joints:
                arms = [{"name": arm, "joints": joints}]
        return arms


# ============================ NPZ 播放面板 ============================

class NPZPanel(QWidget):

    def __init__(self, on_play_state, on_speed):
        super().__init__()
        self.on_play_state = on_play_state
        self.on_speed = on_speed
        # 未加载 npz 前也要有初值，reset_all 里 set_frame(0)/set_total 前不崩
        self._total = 0
        self._fps = 30
        lay = QVBoxLayout()
        lay.setContentsMargins(0, 0, 0, 0)

        # 文件选择
        file_row = QHBoxLayout()
        self.file_combo = QComboBox()
        self.file_combo.currentIndexChanged.connect(self._load_selected)
        file_row.addWidget(self.file_combo, 1)
        browse = QPushButton("浏览…")
        browse.clicked.connect(self._browse)
        file_row.addWidget(browse)
        lay.addLayout(file_row)

        # 播放控制
        ctrl = QHBoxLayout()
        self.play_btn = QPushButton("▶ 播放")
        self.play_btn.setCheckable(True)
        self.play_btn.setChecked(False)
        self.play_btn.toggled.connect(self.on_play_state)
        ctrl.addWidget(self.play_btn)

        ctrl.addWidget(QLabel("倍速"))
        self.speed = QDoubleSpinBox()
        self.speed.setRange(0.05, 5.0)
        self.speed.setValue(1.0)
        self.speed.setSingleStep(0.1)
        self.speed.valueChanged.connect(self.on_speed)
        ctrl.addWidget(self.speed)
        lay.addLayout(ctrl)

        # 进度
        self.progress = QProgressBar()
        self.progress.setRange(0, 100)
        lay.addWidget(self.progress)
        self.frame_label = QLabel("frame 0 / 0")
        lay.addWidget(self.frame_label)

        lay.addStretch(1)
        self.setLayout(lay)

    def refresh_files(self, paths):
        """按文件夹分组填充 npz 下拉：每组先加一个不可选的「标题」项，再跟该目录的 .npz。
        用 QComboBox 标准 addItem(text, data)，data 存完整路径，_load_selected 不变。"""
        # 公共根：把每个 npz 所在目录相对根展开，根目录文件归 "(根)"
        try:
            base = os.path.commonpath([os.path.dirname(p) for p in paths])
        except Exception:
            base = ""
        groups = {}
        for p in paths:
            d = os.path.dirname(p)
            rel = os.path.relpath(d, base) if (base and d != base) else "(根)"
            groups.setdefault(rel, []).append(p)
        self.file_combo.blockSignals(True)
        self.file_combo.clear()
        for g in sorted(groups):
            self.file_combo.addItem(f"—— {g} ——", None)      # 标题：data=None，加载时跳过
            idx = self.file_combo.count() - 1
            it = self.file_combo.model().item(idx)
            if it is not None:
                it.setFlags(Qt.NoItemFlags)                   # 不可选中 / 不响应
            for p in sorted(groups[g]):
                self.file_combo.addItem(os.path.basename(p), p)
        self.file_combo.blockSignals(False)

    def set_total(self, total, fps):
        self.progress.setMaximum(max(1, total))
        self._total = total
        self._fps = fps

    def set_frame(self, idx):
        self.progress.setValue(idx)
        self.frame_label.setText(f"frame {idx} / {self._total}")

    def _load_selected(self, _):
        p = self.file_combo.currentData()
        if p:
            self.on_speed  # noop guard
            if getattr(self, "_on_load", None):
                self._on_load(p)

    def _browse(self):
        p = getattr(self, "_browse_fn", None)
        if p:
            p()


# ============================ 可折叠面板 ============================

class CollapsibleFrame(QFrame):
    """可折叠面板：标题栏含折叠箭头 + 标题 + (可选 header_widget，如 map 下拉) + 右侧 Reload，
    body(内容) 可点箭头折叠/展开；折叠后给上方控件腾出竖向空间。
    Reload 始终在标题栏显示，折叠时仍可点。"""

    def __init__(self, title, body, header_widget=None, on_reload=None, collapsed=False):
        super().__init__()
        self.setFrameShape(QFrame.StyledPanel)
        lay = QVBoxLayout()
        lay.setContentsMargins(6, 6, 6, 4)
        lay.setSpacing(2)
        header = QHBoxLayout()
        header.setContentsMargins(0, 0, 0, 0)
        header.setSpacing(4)
        self._arrow = QPushButton("▾")
        self._arrow.setFixedSize(24, 22)
        self._arrow.setCursor(Qt.PointingHandCursor)
        self._arrow.setToolTip("折叠 / 展开")
        self._arrow.setStyleSheet(
            "QPushButton { border: 1px solid #b0b0b0; border-radius: 3px;"
            " background: #f5f5f5; font-size: 12px; padding: 0; }"
            "QPushButton:hover { background: #e0e0e0; }")
        self._arrow.clicked.connect(self._toggle_collapsed)
        header.addWidget(self._arrow)
        lab = QLabel(title)
        lab.setStyleSheet("font-size: 13px; font-weight: 600;")
        header.addWidget(lab)
        if header_widget is not None:
            header.addWidget(header_widget)
        header.addStretch(1)
        if on_reload is not None:
            rb = QPushButton("Reload")
            rb.setFixedWidth(70)
            rb.clicked.connect(lambda: on_reload())
            header.addWidget(rb)
        lay.addLayout(header)
        self._body = body
        body.setContentsMargins(0, 0, 0, 0)
        lay.addWidget(body, 1)
        self.setLayout(lay)
        self.setMinimumHeight(30)   # 保住标题行，防止被 splitter 拖没
        self._collapsed = False
        if collapsed:
            self.set_collapsed(True)

    def set_collapsed(self, collapsed):
        self._collapsed = collapsed
        if self._body is not None:
            self._body.setVisible(not collapsed)
        self._arrow.setText("▶" if collapsed else "▾")
        self._resync_splitters()

    def toggle_collapsed(self):
        self.set_collapsed(not self._collapsed)

    def _resync_splitters(self):
        """折叠/展开后逐级重设所在 QSplitter 的 sizes：让本面板按当前所需高度
        (sizeHint) 收缩/恢复，其余空间按份分给兄弟 → 折叠腾出的竖向空间
        立即被相邻面板与上方面板吸收，无需手动拖分隔线。"""
        self.updateGeometry()
        node = self
        while node.parentWidget() is not None:
            node.updateGeometry()
            parent = node.parentWidget()
            if isinstance(parent, QSplitter):
                idx = parent.indexOf(node)
                want = max(1, node.sizeHint().height(), node.minimumHeight())
                sizes = parent.sizes()
                cur_total = sum(sizes)
                cur_other = cur_total - sizes[idx]
                n_other = parent.count() - 1
                for i in range(parent.count()):
                    if i == idx:
                        sizes[i] = want
                    else:
                        sizes[i] = (int(cur_other // n_other)
                                    if (n_other and cur_other > 0) else 0)
                parent.setSizes(sizes)
            node = parent

    def _toggle_collapsed(self):
        self.toggle_collapsed()


# ============================ 主窗口 ============================

class RetargetStudio(QWidget):

    def _make_panel(self, title, widget):
        frame = QFrame()
        frame.setFrameShape(QFrame.StyledPanel)
        lay = QVBoxLayout()
        lay.setContentsMargins(6, 6, 6, 6)
        header = QHBoxLayout()
        lab = QLabel(title)
        lab.setStyleSheet("font-size: 13px; font-weight: 600;")
        header.addWidget(lab)
        header.addStretch(1)
        lay.addLayout(header)
        lay.addWidget(widget, 1)
        frame.setLayout(lay)
        return frame

    def __init__(self, urdf_path=None, map_path=None, npz_dir=None):
        super().__init__()

        # map_path 参数保留仅为向后兼容；皮肤/URDF 映射现分别由
        # blender_skin_map.yaml / blender_urdf_map.yaml 唯一驱动。
        self.burdf_map_path = BLENDER_URDF_MAP_PATH
        self.npz_dir = npz_dir or DEFAULT_NPZ_DIR
        self.burdf_map = self._load_burdf_map()

        self.bone_angles = {name: [0, 0, 0] for name, _ in DIRECT_BONES}
        self.scene_bones = {}  # request_scene 返回的 armature名 -> [骨名]（Blender scene 热加载）
        # 皮肤骨架注册表 + 每套直接操控状态（angles/limits/骨骼），由 blender_skin_map.yaml 驱动
        self._build_skin_states()
        self._active_skin = None
        self.urdf_joint_angles_rad = {}
        self.urdf_joint_widgets_list = {}
        self.urdf_use_degree = True
        self.npz_joints = None     # (T,14)
        self.npz_base = None       # (body_pos_w, body_quat_w)（若有 base 数据）
        self.npz_root0 = None      # 首帧 root pos（世界）
        self.npz_frame = 0
        self._diag_base = 0
        self.playing = False

        self.client = BlenderClient(self.on_blender_message, self.on_connect_change)

        # URDF 模型 + Meshcat
        self.urdf_robot = None
        self.urdf_viewer = None
        if urdf_path and os.path.exists(urdf_path):
            try:
                from robot_model import RobotModel
                from robot_viewer import RobotViewer
                self.urdf_robot = RobotModel(urdf_path)
                self.urdf_viewer = RobotViewer(self.urdf_robot)
                print("URDF loaded:", urdf_path)
            except Exception as e:
                print("URDF load failed:", e)

        # ================= 布局 =================
        main_layout = QHBoxLayout()
        main_layout.setContentsMargins(12, 12, 12, 12)

        list_panel = QFrame()
        list_panel.setFrameShape(QFrame.StyledPanel)
        list_layout = QVBoxLayout()

        # 连接状态
        self.status_label = QLabel("● Blender: 未连接")
        self.status_label.setStyleSheet(
            "font-size: 13px; font-weight: 600; color: #a0a0a0;"
        )
        list_layout.addWidget(self.status_label)

        # 全局开关
        self.sync_toggle = QCheckBox("Sync to Blender (皮肤)")
        self.sync_toggle.setChecked(False)
        self.sync_toggle.toggled.connect(self._on_skin_toggle_all)
        list_layout.addWidget(self.sync_toggle)

        self.urdf_sync_toggle = QCheckBox("Drive Blender URDF")
        self.urdf_sync_toggle.setChecked(False)
        self.urdf_sync_toggle.toggled.connect(self._push_urdf_if_needed)
        list_layout.addWidget(self.urdf_sync_toggle)

        # 是否把 npz 的 base(位置/旋转) 映射到 Blender（Blender 实时挪动角色较卡，默认关）
        self.base_sync_toggle = QCheckBox("映射 Base (pos/rot → Blender)")
        self.base_sync_toggle.setChecked(False)
        list_layout.addWidget(self.base_sync_toggle)

        self.reset_btn = QPushButton("Reset All")
        self.reset_btn.clicked.connect(self.reset_all)
        list_layout.addWidget(self.reset_btn)

        self.reload_skin_btn = QPushButton("Reload 皮肤 (map + scene 热加载)")
        self.reload_skin_btn.clicked.connect(self._reload_skin_registry)
        list_layout.addWidget(self.reload_skin_btn)

        # 模式切换：皮肤骨架下拉（Bone/V2Bone/V3Bone 统一变成下拉切骨架）+ URDF/NPZ 模式按钮。
        # 三项用网格三列等宽，保证「皮肤 / URDF / NPZ」尽量等长。
        mode_lay = QGridLayout()
        mode_lay.setContentsMargins(0, 0, 0, 0)
        mode_lay.setSpacing(4)
        self.skin_combo = QComboBox()
        self._populate_skin_combo()
        self._active_skin = self.skin_combo.currentData()
        self.skin_combo.setSizePolicy(QSizePolicy.Expanding, QSizePolicy.Fixed)
        mode_lay.addWidget(self.skin_combo, 0, 0)
        self.mode_btns = {}
        for col, (txt, key) in enumerate([("URDF", "urdf"), ("NPZ", "npz")], start=1):
            btn = QPushButton(txt)
            btn.setCheckable(True)
            btn.setChecked(key == "urdf")
            btn.clicked.connect(lambda _=False, k=key: self._switch_mode(k))
            btn.setSizePolicy(QSizePolicy.Expanding, QSizePolicy.Fixed)
            btn.setStyleSheet("""
                QPushButton { font-size: 12px; padding: 4px 8px;
                              border: 1px solid #ccc; border-radius: 3px; }
                QPushButton:checked { background: #2d6cdf; color: white;
                                      border: 1px solid #2d6cdf; }
            """)
            mode_lay.addWidget(btn, 0, col)
            self.mode_btns[key] = btn
        for c in range(3):
            mode_lay.setColumnStretch(c, 1)
        list_layout.addLayout(mode_lay)

        # 模式面板（stack）
        self.mode_stack = QStackedWidget()

        # ── 皮肤 Bone 模式：直接转动所选皮肤骨架的骨骼（下拉切骨架）──
        skin_w = QWidget()
        skin_w_lay = QVBoxLayout()
        skin_w_lay.setContentsMargins(0, 0, 0, 0)
        tip = QLabel("直接转动所选皮肤骨架骨骼 → set_pose(bone, armature=所选骨架)")
        tip.setWordWrap(True)
        tip.setStyleSheet("font-size: 11px; color: #888;")
        skin_w_lay.addWidget(tip)
        self.skin_bone_list = QListWidget()
        self.skin_bone_list.currentRowChanged.connect(self._on_skin_bone_selected)
        skin_w_lay.addWidget(self.skin_bone_list, 1)
        self.skin_bone_joint_widget = BoneJointWidget("", {"x": (-120, 120), "y": (-120, 120), "z": (-120, 120)}, self._on_skin_bone_axis_change)
        skin_w_lay.addWidget(self.skin_bone_joint_widget)
        skin_w.setLayout(skin_w_lay)
        self.mode_stack.addWidget(skin_w)      # index 0

        # ── URDF 模式 ──
        urdf_w = QWidget()
        urdf_w_lay = QVBoxLayout()
        urdf_w_lay.setContentsMargins(0, 0, 0, 0)
        sel_row = QHBoxLayout()
        self.urdf_select_all = QCheckBox("Select All")
        self.urdf_select_all.toggled.connect(self._on_urdf_select_all)
        sel_row.addWidget(self.urdf_select_all)
        self.urdf_rad_btn = self._toggle_btn("rad", False, lambda: self._set_urdf_mode(False))
        self.urdf_deg_btn = self._toggle_btn("degree", True, lambda: self._set_urdf_mode(True))
        sel_row.addWidget(self.urdf_rad_btn)
        sel_row.addWidget(self.urdf_deg_btn)
        sel_row.addStretch(1)
        urdf_w_lay.addLayout(sel_row)

        body = QWidget()
        body_lay = QVBoxLayout()
        body_lay.setContentsMargins(0, 0, 0, 0)
        self._build_urdf_widgets(body_lay)
        body_lay.addStretch(1)
        body.setLayout(body_lay)
        scroll = QScrollArea()
        scroll.setWidgetResizable(True)
        scroll.setWidget(body)
        urdf_w_lay.addWidget(scroll, 1)
        urdf_w.setLayout(urdf_w_lay)
        self.mode_stack.addWidget(urdf_w)      # index 1

        # ── NPZ 模式 ──
        self.npz_panel = NPZPanel(self._on_play_state, lambda v: None)
        self.npz_panel._on_load = self._npz_load
        self.npz_panel._browse_fn = self._npz_browse
        self.mode_stack.addWidget(self.npz_panel)  # index 2

        # 骨架下拉切换 → 进入皮肤控制并刷新骨列表（reload 重建 combo 后也由它保持）
        self.skin_combo.currentIndexChanged.connect(self._on_skin_armature_selected)

        self._switch_mode("urdf")
        list_layout.addWidget(self.mode_stack, 1)

        list_panel.setLayout(list_layout)

        # 底部面板：重定向 + Blender URDF 映射
        # URDF→Skin 下拉统由 blender_skin_map.yaml 注册表驱动（rig / self_rig_v2 / self_rig_v3 …）
        self.retarget_panel = RetargetPanel(
            {nm: st["joints"] for nm, st in self.skin_states.items()}, self._reload_map)
        self.burdf_panel = BlenderUrdfPanel(self.burdf_map, self._reload_burdf_map)

        bottom = QWidget()
        bottom_lay = QVBoxLayout()
        bottom_lay.setContentsMargins(0, 0, 0, 0)
        # 底部两个可折叠面板，竖向放入可拖动的 QSplitter（鼠标拖手柄调节上下比例）
        bottom_split = QSplitter(Qt.Vertical)
        skin_frame = CollapsibleFrame(
            "URDF→Skin", self.retarget_panel,
            header_widget=self.retarget_panel.map_combo, on_reload=self._reload_map)
        urdf_frame = CollapsibleFrame(
            "URDF→Blender-URDF", self.burdf_panel,
            header_widget=None, on_reload=self._reload_burdf_map)
        bottom_split.addWidget(skin_frame)
        bottom_split.addWidget(urdf_frame)
        bottom_split.setSizes([320, 200])
        bottom_split.setChildrenCollapsible(False)
        bottom_lay.addWidget(bottom_split, 1)
        bottom.setLayout(bottom_lay)

        left_splitter = QSplitter(Qt.Vertical)
        left_splitter.addWidget(list_panel)
        left_splitter.addWidget(bottom)
        left_splitter.setStretchFactor(0, 3)
        left_splitter.setStretchFactor(1, 2)
        left_splitter.setChildrenCollapsible(False)   # 拖动分隔不会让下面区域消失
        left_splitter.setSizes([620, 300])

        viewer_widget = self.urdf_viewer if self.urdf_viewer else QLabel("No URDF viewer")
        # meshcat 下方叠 Base 手动排查面板（pos/rot 滑块，直连 meshcat + Blender，方便定位）
        base_panel = self._make_panel("Base 手动 (pos/rot 排查)", self._build_base_widget())
        right_stack = QWidget()
        _rv = QVBoxLayout(right_stack)
        _rv.setContentsMargins(0, 0, 0, 0)
        _rv.addWidget(viewer_widget, 1)
        _rv.addWidget(base_panel)
        right_panel = self._make_panel(
            "URDF Meshcat 3D" if self.urdf_viewer else "URDF (unavailable)",
            right_stack,
        )

        main_splitter = QSplitter(Qt.Horizontal)
        main_splitter.addWidget(left_splitter)
        main_splitter.addWidget(right_panel)
        # 右面板（URDF Meshcat 3D）默认只占 1/3，保持 splitter 可手动拖拽调整
        main_splitter.setStretchFactor(0, 2)
        main_splitter.setStretchFactor(1, 1)
        main_splitter.setSizes([700, 350])

        main_layout.addWidget(main_splitter)
        self.setLayout(main_layout)

        self.setWindowTitle("Pikachu Retarget Studio")
        self.resize(1280, 780)

        # 播放定时器
        self.play_timer = QTimer()
        self.play_timer.timeout.connect(self._npz_tick)

        # 交互刷新
        self.refresh_timer = QTimer()
        self.refresh_timer.setInterval(250)
        self.refresh_timer.timeout.connect(self._refresh_npz_files)
        self.refresh_timer.start()

        # NPZ 文件列表
        self.npz_paths = []
        self._refresh_npz_files()

    # ---------- 工具 ----------
    @staticmethod
    def _toggle_btn(text, checked, cb):
        btn = QPushButton(text)
        btn.setCheckable(True)
        btn.setChecked(checked)
        btn.clicked.connect(cb)
        btn.setStyleSheet("""
            QPushButton { font-size: 11px; padding: 4px 8px;
                          border: 1px solid #ccc; border-radius: 3px; }
            QPushButton:checked { background: #2d6cdf; color: white;
                                  border: 1px solid #2d6cdf; }
        """)
        return btn

    def _load_burdf_map(self):
        try:
            import yaml
            with open(self.burdf_map_path, "r", encoding="utf-8") as f:
                return yaml.safe_load(f) or {}
        except Exception:
            return {}

    # ---------- URDF 滑动条构建（按 hip → arm → head 分组，每组可折叠）----------
    def _build_urdf_widgets(self, layout):
        # 每组一个标题 + 关节列表 + 归属前缀（用于把未知关节兜进正确的组）
        GROUPS = [
            ("Hip", [
                "left_hip_pitch_joint", "left_hip_roll_joint", "left_hip_yaw_joint",
                "left_knee_joint", "left_ankle_joint",
                "right_hip_pitch_joint", "right_hip_roll_joint", "right_hip_yaw_joint",
                "right_knee_joint", "right_ankle_joint",
            ], ("hip", "knee", "ankle")),
            ("Arm", [
                "left_arm_pitch_joint", "left_arm_roll_joint", "left_arm_yaw_joint",
                "left_elbow_joint",
                "right_arm_pitch_joint", "right_arm_roll_joint", "right_arm_yaw_joint",
                "right_elbow_joint",
            ], ("arm", "elbow", "shoulder")),
            ("Head", [
                "head_yaw_joint", "head_pitch_joint", "head_roll_joint",
            ], ("head", "neck")),
            ("Ear", [
                "left_ear_pitch_joint", "left_ear_roll_joint",
                "right_ear_pitch_joint", "right_ear_roll_joint",
            ], ("ear",)),
            ("Tail", [
                "tail_pitch_joint", "tail_yaw_joint",
            ], ("tail",)),
        ]

        def joint_limit(name):
            if self.urdf_robot and name in self.urdf_robot.joint_limits:
                return self.urdf_robot.joint_limits[name]
            return -3.14, 3.14

        for group_title, order, terms in GROUPS:
            known = set(order)
            if self.urdf_robot:
                names = [n for n in order if n in self.urdf_robot.joint_limits]
                names += [n for n in self.urdf_robot.joint_names
                          if n not in known and any(n.startswith(t) for t in terms)]
            else:
                names = list(order)

            # 折叠头：勾选=展开（↑/↓ 箭头 + 文字）
            head = QToolButton()
            head.setText(group_title)
            head.setCheckable(True)
            head.setChecked(True)
            head.setToolButtonStyle(Qt.ToolButtonTextBesideIcon)
            head.setArrowType(Qt.DownArrow)
            head.setStyleSheet("QToolButton { font-size: 13px; font-weight: 700; "
                               "border: none; padding: 3px 0; }")
            layout.addWidget(head)

            # 组内容：包一层 QWidget，折叠时整体隐藏
            cont = QWidget()
            cont_lay = QVBoxLayout()
            cont_lay.setContentsMargins(8, 0, 0, 0)
            cont_lay.setSpacing(2)
            def make_widget(name):
                lower, upper = joint_limit(name)
                w = URDFJointWidget(name, lower, upper, self._on_urdf_joint_change,
                                    lambda n, c: None, use_degree=True, sync_checked=False)
                self.urdf_joint_widgets_list[name] = w
                return w

            # 左右同关节并排：list up left_* / right_*，按后缀配对
            lefts = {n[5:]: n for n in names if n.startswith("left_")}
            rights = {n[6:]: n for n in names if n.startswith("right_")}
            paired = set()
            for suffix in list(lefts.keys()):
                ls, rs = lefts.get(suffix), rights.get(suffix)
                row = QHBoxLayout()
                row.setContentsMargins(0, 0, 0, 0)
                row.setSpacing(10)
                row.addWidget(make_widget(ls), 1)
                paired.add(ls)
                if rs:
                    row.addWidget(make_widget(rs), 1)
                    paired.add(rs)
                cont_lay.addLayout(row)
            # 无左右配对的（如 head_*）单独整行
            for name in names:
                if name in paired:
                    continue
                cont_lay.addWidget(make_widget(name))
            cont.setLayout(cont_lay)
            layout.addWidget(cont)

            def _toggle(checked, cont=cont, head=head):
                cont.setVisible(checked)
                head.setArrowType(Qt.DownArrow if checked else Qt.RightArrow)
            head.toggled.connect(_toggle)
            layout.addSpacing(6)

    # ---------- 模式切换 ----------
    def _switch_mode(self, key):
        order = {"skin": 0, "urdf": 1, "npz": 2}
        self.mode_stack.setCurrentIndex(order[key])
        for k, b in self.mode_btns.items():
            b.setChecked(k == key)

    # ---------- 连接状态 ----------
    def on_connect_change(self, connected):
        if connected:
            self.status_label.setText("● Blender: 已连接")
            self.status_label.setStyleSheet("font-size: 13px; font-weight: 600; color: #22c55e;")
            self.client.request_scene()
        else:
            self.status_label.setText("● Blender: 未连接")
            self.status_label.setStyleSheet("font-size: 13px; font-weight: 600; color: #a0a0a0;")

    def on_blender_message(self, msg):
        t = msg.get("type")
        if t == "scene":
            self._on_scene(msg.get("data", {}))
        elif t == "bones":
            print("[Blender bones]", len(msg.get("data", [])))
        elif t == "debug":
            print("[Blender]", msg.get("message"))

    def _on_scene(self, data):
        objs = data.get("objects", [])
        arma = data.get("armatures", [])
        print(f"[Scene] objects={len(objs)} armatures={len(arma)}")
        # 热加载：把 Blender 场景里的真实骨骼写回对应皮肤骨架（scene 优先于 map 的 bones）
        for a in arma:
            nm = a.get("name", "")
            bones = [b["name"] for b in a.get("bones", []) if isinstance(b, dict)]
            if not nm or not bones:
                continue
            print(f"   armature '{nm}': {len(bones)} bones")
            self.scene_bones[nm] = bones
            st = self.skin_states.get(nm)
            if st:
                st["bones"] = bones
                for b in bones:
                    st["angles"].setdefault(b, [0, 0, 0])
                    st["limits"].setdefault(b, {"x": (-120, 120), "y": (-120, 120), "z": (-120, 120)})
        self._refresh_skin_bone_list()
        # 校验各皮肤骨架映射目标骨是否存在于任一 Blender armature（仅诊断）
        allbones = {b["name"] for a in arma for b in a["bones"]}
        if allbones:
            ref_bones = {c.get("bone") for st in self.skin_states.values()
                         for c in st["joints"].values() if c.get("bone")}
            missing = sorted(ref_bones - allbones)
            if missing:
                print("   skin 映射目标骨缺失:", missing)

    # ---------- 皮肤骨架直接控制（下拉切骨架；Bone/V2Bone/V3Bone 统一走这里）----------
    def _on_skin_armature_selected(self, idx):
        name = self.skin_combo.itemData(idx)
        if name not in self.skin_states:
            return
        self._set_active_skin(name)

    def _set_active_skin(self, name):
        if name not in self.skin_states:
            return
        self._active_skin = name
        st = self.skin_states[name]
        self.skin_bone_list.blockSignals(True)
        self.skin_bone_list.clear()
        for b in st["bones"]:
            self.skin_bone_list.addItem(b)
        self.skin_bone_list.blockSignals(False)
        if st["bones"]:
            self.skin_bone_list.setCurrentRow(0)
        self.skin_bone_joint_widget.set_angles([0, 0, 0])
        # 同步下拉框，切到皮肤控制页
        self.skin_combo.blockSignals(True)
        want = self.skin_combo.findData(name)
        if want >= 0 and self.skin_combo.currentIndex() != want:
            self.skin_combo.setCurrentIndex(want)
        self.skin_combo.blockSignals(False)
        self._switch_mode("skin")

    def _refresh_skin_bone_list(self):
        """scene 热加载改动了当前骨架的骨骼后刷新列表。"""
        st = self.skin_states.get(self._active_skin)
        if not st:
            return
        cur = self.skin_bone_list.currentRow()
        cur_name = st["bones"][cur] if 0 <= cur < len(st["bones"]) else None
        self.skin_bone_list.blockSignals(True)
        self.skin_bone_list.clear()
        for b in st["bones"]:
            self.skin_bone_list.addItem(b)
        self.skin_bone_list.blockSignals(False)
        if st["bones"]:
            if cur_name in st["bones"]:
                self.skin_bone_list.setCurrentRow(st["bones"].index(cur_name))
            else:
                self.skin_bone_list.setCurrentRow(0)
            self._on_skin_bone_selected(self.skin_bone_list.currentRow())

    def _on_skin_bone_selected(self, row):
        if row < 0:
            return
        st = self.skin_states.get(self._active_skin)
        if not st:
            return
        name = st["bones"][row]
        lim = st["limits"].get(name, {"x": (-120, 120), "y": (-120, 120), "z": (-120, 120)})
        self.skin_bone_joint_widget.set_bone_name(name)
        self.skin_bone_joint_widget.set_limits(lim)
        self.skin_bone_joint_widget.set_angles(st["angles"].get(name, [0, 0, 0]))

    def _on_skin_bone_axis_change(self, bone, axis, val):
        st = self.skin_states.get(self._active_skin)
        if not st:
            return
        angles = st["angles"].setdefault(bone, [0, 0, 0])
        angles[AXIS_INDEX[axis]] = val
        if self.client.connected:
            self.client.set_pose(dict(st["angles"]), armature=st["armature_name"])

    # ---------- URDF 控制 ----------
    def _on_urdf_joint_change(self, name, angle_rad):
        if self.playing:
            return
        self.urdf_joint_angles_rad[name] = angle_rad
        if self.urdf_robot:
            self.urdf_robot.set_joint(name, angle_rad)
        if self.urdf_viewer:
            self.urdf_viewer.update_robot()
        self._push_mapped_chains()

    def _push_mapped_chains(self):
        bone_pose = self._current_skin_pose()
        self.retarget_panel.update_pose(bone_pose)
        self._push_skins()
        self._push_urdf_if_needed()

    def _populate_skin_combo(self):
        """按 skin 注册表 name 填充下拉（不触发 currentIndexChanged）。

        默认选中最近的（yaml 里最后一个）骨架（如 self_rig_v3），随 yaml 末尾新项同步更新。
        """
        self.skin_combo.blockSignals(True)
        self.skin_combo.clear()
        for name in self.skin_states:
            self.skin_combo.addItem(name, name)
        # 默认选最后一项（最新 self_rig_v3；加新骨架则自动跟随 yaml 末尾）
        if self.skin_combo.count():
            self.skin_combo.setCurrentIndex(self.skin_combo.count() - 1)
        self.skin_combo.blockSignals(False)

    def _build_skin_states(self):
        """读 blender_skin_map.yaml 建 self.skin_states = {骨架名: {...}}。

        每套含 armature_name/joints/bones/limits/angles/base_cfg。joints 一律来自
        blender_skin_map.yaml 各自的 joints 块（rig 不再走 retarget_map.yaml）。
        bones 由 map 的 bones 初始化，连上 Blender 后由 scene 覆盖。
        """
        reg = load_skin_registry()
        self.skin_registry = reg
        self.skin_states = {}
        for a in reg:
            name = a["name"]
            limits = bone_limits_from_map(
                a["joints"], [(b, (-120, 120)) for b in a["bones"]])
            self.skin_states[name] = {
                "armature_name": name,
                "joints": a["joints"],
                "bones": list(a["bones"]),
                "limits": limits,
                "angles": {b: [0, 0, 0] for b in a["bones"]},
                "base_cfg": a["cfg"],
            }
        return reg

    def _skin_armatures(self):
        """供 set_pose 驱动的皮肤骨架注册表：(armature_name, rt_map)。

        每个骨架用各自的 retarget map 把 URDF 关节角写成该骨架的骨骼欧拉角。
        rig = 原 Rigify 皮肤（默认 / 旧版 addon 也兜得住）；
        其余为 blender_skin_map.yaml 里登记的自定义皮肤骨架（self_rig_v2 / self_rig_v3 …）。
        """
        return [(st["armature_name"], st["joints"]) for st in self.skin_states.values()]

    def _push_skins(self):
        """把当前 URDF 皮肤姿态推给所有注册的皮肤骨架（受 Sync to Blender 皮肤 开关限制）。"""
        if not (self.sync_toggle.isChecked() and self.client.connected):
            return
        for arm_name, m in self._skin_armatures():
            pose = self._current_skin_pose(m)
            if pose:
                self.client.set_pose(pose, armature=arm_name)

    def _urdf_armature_list(self):
        return BlenderUrdfPanel._armature_list(self.burdf_map)

    def _urdf_push_targets(self):
        """返回 [(blender 真实骨架名, joints)] 供 set_urdf_pose 驱动。

        连上 Blender 且 scene 里存在 URDF 骨架（骨名含 .revolute/.fixed.bone）时，
        用 scene 真实骨架名推送、并合并 map 全套 joints —— 否则 map 里历史虚拟骨架名
        （Pikachu_V025 / pikachu_sample_links / ...）与 Blender 场景对象名（如
        pikachu_urdf_T）对不上，addon 的 get_armature 会回退到「当前激活 / 第一个
        ARMATURE」，时而被写进皮肤骨架整台不动、时而被写进 URDF 却只有骨名对得上的
        部分能动 → 时好时坏。未连 Blender（离线兜底）时回退 map 原样。
        """
        map_arms = self._urdf_armature_list()
        if not map_arms:
            return []
        merged = {}
        for a in map_arms:
            merged.update(a.get("joints") or {})
        scene_urdf = [nm for nm, bn in self.scene_bones.items()
                      if any(".revolute.bone" in b or ".fixed.bone" in b for b in bn)]
        if scene_urdf:
            return [(nm, merged) for nm in scene_urdf]
        return [(a.get("name", ""), a.get("joints") or {}) for a in map_arms]

    def _build_pose_for_armature(self, joints, angles, zero=False):
        """把当前 urdf 关节角（弧度）换算成该 rig 的 pose = {bone: [x,y,z](rad)}。

        zero=True：把所有关节当作 0 弧度（用于 reset，落点 = bias 即 rest pose）。
        否则跳过尚未被驱动的关节（ang_rad 为 None）。
        """
        pose = {}
        for uj, cfg in (joints or {}).items():
            if zero:
                ang_rad = 0.0
            else:
                ang_rad = angles.get(uj)
                if ang_rad is None:
                    continue
            bone = cfg.get("bone", "")
            if not bone:
                continue
            axis = cfg.get("axis", "y")
            sign = float(cfg.get("sign", 1.0))
            bias = float(cfg.get("bias", 0.0))
            # 与 Blender 端 set_urdf_pose 接口契约一致：传弧度（bias 单位同为弧度）
            e_rad = ang_rad * sign + bias
            e = pose.setdefault(bone, [0.0, 0.0, 0.0])
            e[AXIS_INDEX.get(axis, 1)] += e_rad
        return pose

    def _push_urdf_if_needed(self):
        """实时把当前 URDF 关节角推到所有 map 里的 Blender rig（是否生效由 Blender 面板勾选过滤）。"""
        if not (self.urdf_sync_toggle.isChecked() and self.client.connected):
            # 诊断：区分「Drive Blender URDF 开关没开」与「addon 旧/未重启」
            if self.client.connected and not self.urdf_sync_toggle.isChecked():
                print("[URDF→Blender] 跳过：Drive Blender URDF 开关未勾选")
            return
        angles = self.urdf_joint_angles_rad
        sent = []
        for arm_name, joints in self._urdf_push_targets():
            pose = self._build_pose_for_armature(joints, angles)
            if pose:
                self.client.set_urdf_pose(arm_name, pose)
                sent.append(f"{arm_name}({len(pose)}骨)")
        if sent:
            print("[URDF→Blender] set_urdf_pose ->", ", ".join(sent))

    def _push_urdf_reset(self):
        """reset：强推所有 map 里的 Blender rig 到 rest pose（全关节 0 → 各轴 bias），
        不受 Drive Blender URDF 开关限制（reset 一致性优先），是否生效仍由面板勾选过滤。"""
        if not self.client.connected:
            return
        for arm_name, joints in self._urdf_push_targets():
            pose = self._build_pose_for_armature(joints, {}, zero=True)
            if pose:
                self.client.set_urdf_pose(arm_name, pose)

    def _current_skin_pose(self, rt_map=None):
        active = {n: self.urdf_joint_angles_rad.get(n, 0.0)
                  for n, w in self.urdf_joint_widgets_list.items() if w.sync_checkbox.isChecked()}
        if rt_map is None:
            # 无参调用（仅用于面板显示 pose 文本）默认取 rig 骨架的映射
            rig = self.skin_states.get("rig") if hasattr(self, "skin_states") else None
            rt_map = rig.get("joints", {}) if rig else {}
        return retarget_mod.apply_retarget_rad(active, rt_map)

    def _push_skin_pose(self):
        bone_pose = self.bone_angles  # 直接 FK 控制
        if self.sync_toggle.isChecked() and self.client.connected:
            self.client.set_pose(bone_pose)

    # ---------- URDF 模式工具 ----------
    def _on_urdf_select_all(self, checked):
        for w in self.urdf_joint_widgets_list.values():
            w.sync_checkbox.blockSignals(True)
            w.sync_checkbox.setChecked(checked)
            w.sync_checkbox.blockSignals(False)

    def _on_skin_toggle_all(self, checked):
        """勾 Sync to Blender(皮肤) = 打开 Drive Blender URDF + 全选所有 urdf 关节（等值 select all）；
        取消则同步取消所有 urdf 勾选并关掉 Drive Blender URDF。"""
        self.urdf_sync_toggle.blockSignals(True)
        self.urdf_sync_toggle.setChecked(bool(checked))
        self.urdf_sync_toggle.blockSignals(False)
        self._on_urdf_select_all(bool(checked))
        self._push_urdf_if_needed()
        self._push_mapped_chains()

    def _set_urdf_mode(self, use_degree):
        self.urdf_use_degree = use_degree
        self.urdf_rad_btn.setChecked(not use_degree)
        self.urdf_deg_btn.setChecked(use_degree)
        for w in self.urdf_joint_widgets_list.values():
            w.set_use_degree(use_degree)

    # ---------- NPZ 播放 ----------
    def _refresh_npz_files(self):
        paths = []
        if os.path.isdir(self.npz_dir):
            for root, _, files in os.walk(self.npz_dir):
                for f in sorted(files):
                    if f.lower().endswith(".npz"):
                        paths.append(os.path.join(root, f))
        if paths != self.npz_paths:
            self.npz_paths = paths
            self.npz_panel.refresh_files(paths)

    def _npz_browse(self):
        p, _ = QFileDialog.getOpenFileName(self, "选择 NPZ 动作文件", self.npz_dir, "*.npz")
        if p:
            self._npz_load(p)

    def _npz_load(self, path):
        try:
            d = np.load(path, allow_pickle=True)
            joints = np.asarray(d["joint_pos"], dtype=float)
            if joints.ndim != 2 or joints.shape[1] < 6:
                raise ValueError(f"joint_pos 需要 (T,>=6)，实际 {joints.shape}")
            if joints.shape[1] < 14:
                print(f"   (列数 {joints.shape[1]} < 14，仅映射腿部子集)")
            fps = float(np.asarray(d["fps"]).reshape(-1)[0]) if "fps" in d else 30.0
            self.npz_joints = joints
            self.npz_base = None
            self.npz_root0 = None
            # 读 root 位姿（body_pos_w/body_quat_w 第 0 个关节=root）。缺键则仅关节动、base 不动。
            if "body_pos_w" in d and "body_quat_w" in d:
                bp = np.asarray(d["body_pos_w"], dtype=float)
                bq = np.asarray(d["body_quat_w"], dtype=float)
                if bp.ndim >= 3 and bp.shape[1] >= 1 and bq.ndim >= 3:
                    self.npz_base = (bp, bq)
                    self.npz_root0 = bp[0, 0] if bp.shape[0] > 0 else None
                    print(f"   base: root pos 帧数={bp.shape[0] * bp.shape[1]} 首帧={np.round(self.npz_root0, 3)}")
            self.npz_frame = 0
            self.npz_panel.set_total(len(joints), fps)
            self.npz_panel.set_frame(0)
            self._npz_fps = fps
            print(f"NPZ 加载: {path}  frames={len(joints)}  fps={fps}")
        except Exception as e:
            print("NPZ 加载失败:", e)

    def _on_play_state(self, checked):
        self.playing = bool(checked)
        self.play_timer.stop()
        if self.playing:
            if self.npz_joints is None:
                self.play_btn_pause()
                return
            speed = max(0.05, self.npz_panel.speed.value())
            interval = int(1000.0 / (self._npz_fps * speed))
            self.play_timer.start(max(1, interval))

    def _npz_tick(self):
        if self.npz_joints is None:
            self._on_play_state(False)
            return
        row = self.npz_joints[self.npz_frame]
        urdf = npz_row_to_urdf(row)
        # 更新 URDF 滑动条 + 模型（不回写，避免反复触发信号）
        for name, w in self.urdf_joint_widgets_list.items():
            if name in urdf:
                w.set_angle(urdf[name])
        if self.urdf_robot:
            for n, v in urdf.items():
                self.urdf_robot.set_joint(n, v)
            self.urdf_viewer.update_robot()
        # 应用并上行
        for n, v in urdf.items():
            self.urdf_joint_angles_rad[n] = v
        self._push_mapped_chains()
        # base（root 位姿）同步到 meshcat + Blender 所有角色
        self._apply_npz_base()
        self.npz_frame += 1
        if self.npz_frame >= len(self.npz_joints):
            self.npz_frame = 0
        self.npz_panel.set_frame(self.npz_frame)

    def _build_base_widget(self):
        """Base 手动排查面板：pos X/Y/Z(cm) + rot X/Y/Z(°) 滑块 + 复位，直连 meshcat + Blender。"""
        self._bs = {}   # key -> QSlider
        self._bsl = {}  # key -> QLabel(值)
        wid = QWidget()
        lay = QVBoxLayout(wid)
        lay.setContentsMargins(2, 2, 2, 2)
        lay.setSpacing(2)

        def slr(key, label, lo, hi, unit, tip):
            h = QHBoxLayout()
            h.addWidget(QLabel(label))
            s = QSlider(Qt.Horizontal)
            s.setRange(lo, hi)
            s.setValue(0)
            s.setToolTip(tip)
            s.setFixedHeight(16)
            s.valueChanged.connect(self._on_base_manual)
            h.addWidget(s, 1)
            v = QLabel(f"0 {unit}")
            v.setFixedWidth(64)
            h.addWidget(v)
            lay.addLayout(h)
            self._bs[key] = s
            self._bsl[key] = v

        slr("px", "pos X", -500, 500, "cm", "角色位置 X 增量(cm)")
        slr("py", "pos Y", -500, 500, "cm", "角色位置 Y 增量(cm)")
        slr("pz", "pos Z", -500, 500, "cm", "角色位置 Z 增量(cm)")
        slr("rx", "rot X", -360, 360, "°", "角色绝对 XYZ 欧拉 X(°)")
        slr("ry", "rot Y", -360, 360, "°", "角色绝对 XYZ 欧拉 Y(°)")
        slr("rz", "rot Z", -360, 360, "°", "角色绝对 XYZ 欧拉 Z(°)")

        rb = QPushButton("Reset base (置 0)")
        rb.setFixedWidth(140)
        rb.clicked.connect(self._reset_base_manual)
        lay.addWidget(rb, 0, Qt.AlignLeft)
        return wid

    def _on_base_manual(self):
        pos = [self._bs["px"].value() / 100.0,
               self._bs["py"].value() / 100.0,
               self._bs["pz"].value() / 100.0]
        rpy = [self._bs["rx"].value(), self._bs["ry"].value(), self._bs["rz"].value()]
        self._bsl["px"].setText(f"{pos[0]*100:.0f} cm")
        self._bsl["py"].setText(f"{pos[1]*100:.0f} cm")
        self._bsl["pz"].setText(f"{pos[2]*100:.0f} cm")
        self._bsl["rx"].setText(f"{rpy[0]}°")
        self._bsl["ry"].setText(f"{rpy[1]}°")
        self._bsl["rz"].setText(f"{rpy[2]}°")
        self._apply_base_to_all(pos, rpy)

    def _reset_base_manual(self):
        for k, s in self._bs.items():
            s.setValue(0)
        self._on_base_manual()

    def _apply_npz_base(self):
        """把当前帧 root 位姿同步到 meshcat（+可选 Blender 所有角色）。
        同时让下方的 base 位置/角度滑块随动显示 npz 当前值（不触发手动回发）。"""
        if not self.npz_base or self.npz_root0 is None:
            return
        bp, bq = self.npz_base
        if self.npz_frame >= bp.shape[0]:
            return
        pos_delta, rpy_deg = npz_base_to(
            bp[self.npz_frame, 0], bq[self.npz_frame, 0], self.npz_root0)
        # 滑块 + meshcat 都播原始元数据（cm / °，不缩放）
        self._sync_base_sliders(pos_delta, rpy_deg)
        if self.urdf_viewer:
            self.urdf_viewer.follow_robot(True)
        self._apply_base_to_all(pos_delta, rpy_deg, remap=True)

    def _sync_base_sliders(self, pos_delta, rpy_deg):
        """播放 npz 时更新 base 手动面板的滑块/数值显示（cm 与 °）。
        用 blockSignals 防止 setValue 触发 _on_base_manual 造成回环覆写。"""
        for s in self._bs.values():
            s.blockSignals(True)
        vals = {
            "px": int(round(pos_delta[0] * 100.0)), "py": int(round(pos_delta[1] * 100.0)),
            "pz": int(round(pos_delta[2] * 100.0)),
            "rx": int(round(rpy_deg[0])), "ry": int(round(rpy_deg[1])), "rz": int(round(rpy_deg[2])),
        }
        for k, v in vals.items():
            s = self._bs.get(k)
            if s is None:
                continue
            s.setValue(v)
            lab = self._bsl.get(k)
            if lab is not None:
                unit = "cm" if k.startswith("p") else "°"
                lab.setText(f"{v} {unit}")
        for s in self._bs.values():
            s.blockSignals(False)

    def _skin_base_confs(self):
        """皮肤骨架 (name, base_cfg)：全部来自 blender_skin_map.yaml 的 per-armature base 块。"""
        return [(a["armature_name"], a["base_cfg"]) for a in self.skin_states.values()]

    def _burdf_base_confs(self):
        confs = []
        for a in self._urdf_armature_list():
            name = a.get("name")
            if name:
                confs.append((name, _base_cfg_for_burdf(self.burdf_map, name)))
        return confs

    def _apply_base_to_all(self, pos, rpy, remap=False):
        """把 base 位姿同步到 meshcat +（可选）Blender 所有角色。

        meshcat 永远播「原始 base 元数据」（npz 相对首帧位移 + 绝对 rpy），不做任何缩放/轴置换。
        只有 Blender 各 armature 才按各自 cfg 重映射（pos_retarget/pos_scale/pos_dir /
        rot_retarget/rot_scale/rot_offset）：remap=True(npx) 完整重映射；remap=False(手动) 仅叠 rot_offset。
        "映射 Base" 勾选才推给 Blender。"""
        if self.urdf_viewer:
            self.urdf_viewer.set_base_transform(pos, rpy)
        if not self.base_sync_toggle.isChecked():
            return
        if not self.client.connected:
            return
        for arm_name, cfg in self._skin_base_confs():
            if remap:
                bp_ = _base_pos_for_cfg(pos, cfg)
                br_ = _blender_base_rpy(cfg, rpy)
                if arm_name == SELF_RIG_V2_ARMATURE and self._diag_base % 20 == 0:
                    print(f"  [base→skin] f{self.npz_frame}: pos={[round(v,3) for v in bp_]} "
                          f"rpy={[round(v,1) for v in br_]}  pos_rt={cfg.get('pos_retarget')}")
                self._diag_base += 1
                self.client.set_base(arm_name, bp_, br_)
            else:
                self.client.set_base(arm_name, pos, _blender_rpy_direct(cfg, rpy))
        for name, cfg in self._burdf_base_confs():
            if remap:
                self.client.set_base(name, _base_pos_for_cfg(pos, cfg), _blender_base_rpy(cfg, rpy))
            else:
                self.client.set_base(name, pos, _blender_rpy_direct(cfg, rpy))

    def _reset_base(self):
        """播完/重置：meshcat 归 0；Blender 各角色 pos 归 0、旋转回到默认安装朝向(rot_offset)。
        reset 是显式复位，不受"映射 Base"开关限制。面板滑块一并归 0。"""
        self._sync_base_sliders([0.0, 0.0, 0.0], [0.0, 0.0, 0.0])
        if self.urdf_viewer:
            self.urdf_viewer.set_base_transform([0, 0, 0], [0, 0, 0])
            self.urdf_viewer.follow_robot(False)
        if self.client.connected:
            for arm_name, cfg in self._skin_base_confs():
                self.client.set_base(arm_name, [0, 0, 0], _rot_offset(cfg))
            for name, cfg in self._burdf_base_confs():
                self.client.set_base(name, [0, 0, 0], _rot_offset(cfg))

    def play_btn_pause(self):
        self.playing = False
        self.npz_panel.play_btn.setChecked(False)
        self.play_timer.stop()

    # ---------- 重载 / 复位 ----------
    def _reload_map(self):
        try:
            # 皮肤映射统一重读 blender_skin_map.yaml；再喂给 Retarget 面板表格显示。
            self._reload_skin_registry()
            self.retarget_panel.set_maps({nm: st["joints"] for nm, st in self.skin_states.items()})
            print("Retarget map reloaded")
        except Exception as e:
            print("Reload map failed:", e)

    def _reload_skin_registry(self):
        """统一重读 blender_skin_map.yaml：重建皮肤骨架注册表/限位/下拉，并用 Blender scene 覆盖骨骼。
        Retarget / Burdf 面板与「Reload 皮肤」按钮都调用，皮肤 map 改动即时生效。"""
        if not hasattr(self, "skin_states"):
            return
        prev = self._active_skin
        self._build_skin_states()
        if prev not in self.skin_states:
            # 默认回退到 yaml 里最后一项（最新 self_rig_v3）
            prev = next(reversed(self.skin_states), None)
        if prev:
            self._active_skin = prev
            # 重建下拉并同步回 prev（不触发 signal / 不切页）
            self._populate_skin_combo()
            self.skin_combo.blockSignals(True)
            w = self.skin_combo.findData(prev)
            if w >= 0:
                self.skin_combo.setCurrentIndex(w)
            self.skin_combo.blockSignals(False)
            self._refresh_skin_bone_list()
            self.skin_bone_joint_widget.set_angles([0, 0, 0])
        if self.client.connected:
            self.client.request_scene()
        print("  skin registry reloaded:", list(self.skin_states))

    def _reload_burdf_map(self):
        self.burdf_map = self._load_burdf_map()
        self.burdf_panel.rebuild(self.burdf_map)
        self._reload_skin_registry()
        print("Blender URDF map reloaded")

    def reset_all(self):
        # 停止 NPZ 播放并清进度
        self.play_btn_pause()
        self.npz_frame = 0
        self.npz_panel.set_frame(0)

        # 清零 URDF 关节（滑块 + 模型）
        self.urdf_joint_angles_rad = {}
        for name, w in self.urdf_joint_widgets_list.items():
            w.set_angle(0.0)
            if self.urdf_robot:
                self.urdf_robot.set_joint(name, 0.0)
        if self.urdf_viewer:
            self.urdf_viewer.update_robot()

        # 清零直接操控骨（所有皮肤骨架）+ UI
        for st in self.skin_states.values():
            for b in st["angles"]:
                st["angles"][b] = [0, 0, 0]
        self.skin_bone_joint_widget.set_angles([0, 0, 0])
        if hasattr(self, "skin_bone_list") and self.skin_bone_list.currentRow() >= 0:
            self._on_skin_bone_selected(self.skin_bone_list.currentRow())

        # 上行零姿态给 Blender（所有皮肤骨架 + blender-urdf）
        bone_pose = self._current_skin_pose()
        self.retarget_panel.update_pose(bone_pose)
        if self.client.connected:
            # 直接骨骼控制：把清零后的 st["angles"]（全 0）推给每个皮肤骨架，让 Blender 模型回中。
            # 注意直接操控的姿势存于 st["angles"]，与 URDF 推导的 rest pose 是两套，都要上行。
            for st in self.skin_states.values():
                self.client.set_pose(dict(st["angles"]), armature=st["armature_name"])
            for arm_name, m in self._skin_armatures():
                pose = self._current_skin_pose(m)
                if pose:
                    self.client.set_pose(pose, armature=arm_name)
            # reset 强推所有 map rig 到 rest，保持 meshcat 与勾选的 Blender URDF 一致
            self._push_urdf_reset()

        # base 归位（角色回原位置 / 朝向）
        self._reset_base()
        print("Reset all")

    def closeEvent(self, event):
        self.play_timer.stop()
        if hasattr(self, "urdf_viewer") and self.urdf_viewer is not None:
            try:
                self.urdf_viewer.close()
            except Exception:
                pass
        super().closeEvent(event)


if __name__ == "__main__":
    print("Skin map:", os.path.abspath(BLENDER_SKIN_MAP_PATH))
    urdf_path = DEFAULT_URDF_PATH
    if len(sys.argv) > 1:
        urdf_path = sys.argv[1]
    if "PIKACHU_URDF_PATH" in os.environ:
        urdf_path = os.environ["PIKACHU_URDF_PATH"]
    print("URDF path:", os.path.abspath(urdf_path))

    QApplication.setAttribute(Qt.AA_ShareOpenGLContexts, True)
    app = QApplication(sys.argv)
    w = RetargetStudio(urdf_path=urdf_path)
    w.show()
    sys.exit(app.exec())