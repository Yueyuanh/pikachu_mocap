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

RETARGET_MAP_PATH = os.path.join(BASE_DIR, "retarget_map.yaml")
BLENDER_URDF_MAP_PATH = os.path.join(BASE_DIR, "blender_urdf_map.yaml")
DEFAULT_URDF_PATH = os.path.join(
    BASE_DIR, "urdf", "robot", "Pikachu_V025", "urdf", "Pikachu_V025_flat_21dof.urdf"
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
        if is_arm_roll:
            if col == IDX_RIGHT_ARM_ROLL:
                v = math.pi / 2.0 - v
            elif col == IDX_LEFT_ARM_ROLL:
                v = -math.pi / 2.0 - v
        out[urdf_name] = v
    return out


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

    def set_pose(self, pose):
        if not pose:
            return
        self.send({"type": "set_pose", "pose": pose})

    def set_urdf_pose(self, armature, pose):
        if not pose:
            return
        self.send({"type": "set_urdf_pose", "armature": armature, "pose": pose})

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
        self.sync_checkbox = QCheckBox()
        self.sync_checkbox.setChecked(bool(self.sync_checked))
        self.sync_checkbox.toggled.connect(self._on_sync_toggled)
        title = QLabel(name)
        title.setStyleSheet("font-size: 12px; font-weight: 600;")
        header.addWidget(self.sync_checkbox, 0)
        header.addWidget(title, 1)
        layout.addLayout(header)

        slider_lay = QHBoxLayout()
        slider_lay.setContentsMargins(20, 0, 0, 0)
        self.slider = QSlider(Qt.Horizontal)
        self.slider.setMinimumWidth(200)
        self.slider.setMaximumWidth(500)
        if self.use_degree:
            a, b = int(lower * 180 / math.pi), int(upper * 180 / math.pi)
        else:
            a, b = int(lower * 100), int(upper * 100)
        self.slider.setMinimum(a)
        self.slider.setMaximum(b)
        self.slider.setValue(0)
        self.slider.valueChanged.connect(self._on_slider_change)
        slider_lay.addWidget(self.slider, 1)
        self.value_label = QLabel()
        self.value_label.setFixedWidth(190)
        self.value_label.setAlignment(Qt.AlignLeft | Qt.AlignVCenter)
        self.update_value_label(0.0)
        slider_lay.addWidget(self.value_label, 0)
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
        self.value_label.setText(f"{angle_rad:.2f}(rad) {angle_deg:.1f}(deg) ({ld:.0f},{ud:.0f})")

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

    def __init__(self, name, limit, on_change):
        super().__init__()
        self.name = name
        self.on_change = on_change
        lo, hi = limit
        lay = QVBoxLayout()
        lay.setContentsMargins(0, 0, 0, 0)
        title = QLabel(f"Bone: {name}")
        title.setStyleSheet("font-size: 13px; font-weight: 600;")
        lay.addWidget(title)
        self.labels = {}
        for axis in ["x", "y", "z"]:
            row = QHBoxLayout()
            row.setContentsMargins(0, 0, 0, 0)
            lab = QLabel(axis.upper())
            lab.setFixedWidth(20)
            row.addWidget(lab)
            sl = QSlider(Qt.Horizontal)
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

    def __init__(self, rt_map, on_reload):
        super().__init__()
        self.rt_map = rt_map
        self.on_reload = on_reload
        lay = QVBoxLayout()
        lay.setContentsMargins(0, 0, 0, 0)
        header = QHBoxLayout()
        title = QLabel("Retarget Map (URDF→Skin)")
        title.setStyleSheet("font-size: 13px; font-weight: 600;")
        header.addWidget(title)
        header.addStretch(1)
        reload_btn = QPushButton("Reload")
        reload_btn.setFixedWidth(70)
        reload_btn.clicked.connect(lambda: self.on_reload())
        header.addWidget(reload_btn)
        lay.addLayout(header)

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
        self.rebuild(rt_map)

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
        header = QHBoxLayout()
        title = QLabel("Blender URDF (urdf→blender-urdf)")
        title.setStyleSheet("font-size: 13px; font-weight: 600;")
        header.addWidget(title)
        header.addStretch(1)
        rb = QPushButton("Reload")
        rb.setFixedWidth(70)
        rb.clicked.connect(lambda: self.on_reload())
        header.addWidget(rb)
        lay.addLayout(header)
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
        self.file_combo.blockSignals(True)
        self.file_combo.clear()
        for p in paths:
            self.file_combo.addItem(p, p)
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

        self.map_path = map_path or RETARGET_MAP_PATH
        self.burdf_map_path = os.path.join(BASE_DIR, "blender_urdf_map.yaml")
        self.npz_dir = npz_dir or DEFAULT_NPZ_DIR
        self.rt_map = retarget_mod.load_retarget_map(self.map_path)
        self.burdf_map = self._load_burdf_map()

        self.bone_angles = {name: [0, 0, 0] for name, _ in DIRECT_BONES}
        self.urdf_joint_angles_rad = {}
        self.urdf_joint_widgets_list = {}
        self.urdf_use_degree = True
        self.npz_joints = None     # (T,14)
        self.npz_frame = 0
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
        list_layout.addWidget(self.sync_toggle)

        self.urdf_sync_toggle = QCheckBox("Drive Blender URDF")
        self.urdf_sync_toggle.setChecked(False)
        self.urdf_sync_toggle.toggled.connect(self._push_urdf_if_needed)
        list_layout.addWidget(self.urdf_sync_toggle)

        self.reset_btn = QPushButton("Reset All")
        self.reset_btn.clicked.connect(self.reset_all)
        list_layout.addWidget(self.reset_btn)

        # 模式切换
        mode_lay = QHBoxLayout()
        mode_lay.setContentsMargins(0, 0, 0, 0)
        self.mode_btns = {}
        for i, (txt, key) in enumerate([("Bone", "bone"), ("URDF", "urdf"), ("NPZ", "npz")]):
            btn = QPushButton(txt)
            btn.setCheckable(True)
            btn.setChecked(i == 1)
            btn.clicked.connect(lambda _=False, k=key: self._switch_mode(k))
            btn.setStyleSheet("""
                QPushButton { font-size: 12px; padding: 4px 8px;
                              border: 1px solid #ccc; border-radius: 3px; }
                QPushButton:checked { background: #2d6cdf; color: white;
                                      border: 1px solid #2d6cdf; }
            """)
            mode_lay.addWidget(btn)
            self.mode_btns[key] = btn
        list_layout.addLayout(mode_lay)

        # 模式面板（stack）
        self.mode_stack = QStackedWidget()

        # ── Bone 模式 ──
        bone_w = QWidget()
        bone_w_lay = QVBoxLayout()
        bone_w_lay.setContentsMargins(0, 0, 0, 0)
        self.bone_list = QListWidget()
        for name, _ in DIRECT_BONES:
            self.bone_list.addItem(name)
        self.bone_list.currentRowChanged.connect(self._on_bone_selected)
        bone_w_lay.addWidget(self.bone_list, 1)
        self.bone_joint_widget = BoneJointWidget(DIRECT_BONES[0][0], DIRECT_BONES[0][1], self._on_bone_axis_change)
        bone_w_lay.addWidget(self.bone_joint_widget)
        bone_w.setLayout(bone_w_lay)
        self.mode_stack.addWidget(bone_w)      # index 0

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

        self._switch_mode("urdf")
        list_layout.addWidget(self.mode_stack, 1)

        list_panel.setLayout(list_layout)

        # 底部面板：重定向 + Blender URDF 映射
        self.retarget_panel = RetargetPanel(self.rt_map, self._reload_map)
        self.burdf_panel = BlenderUrdfPanel(self.burdf_map, self._reload_burdf_map)

        bottom = QWidget()
        bottom_lay = QVBoxLayout()
        bottom_lay.setContentsMargins(0, 0, 0, 0)
        bottom_lay.addWidget(self._make_panel("URDF→Skin", self.retarget_panel), 3)
        bottom_lay.addWidget(self._make_panel("URDF→Blender-URDF", self.burdf_panel), 2)
        bottom.setLayout(bottom_lay)

        left_splitter = QSplitter(Qt.Vertical)
        left_splitter.addWidget(list_panel)
        left_splitter.addWidget(bottom)
        left_splitter.setStretchFactor(0, 3)
        left_splitter.setStretchFactor(1, 2)
        left_splitter.setSizes([500, 260])

        viewer_widget = self.urdf_viewer if self.urdf_viewer else QLabel("No URDF viewer")
        right_panel = self._make_panel(
            "URDF Meshcat 3D" if self.urdf_viewer else "URDF (unavailable)",
            viewer_widget,
        )

        main_splitter = QSplitter(Qt.Horizontal)
        main_splitter.addWidget(left_splitter)
        main_splitter.addWidget(right_panel)
        main_splitter.setStretchFactor(0, 1)
        main_splitter.setStretchFactor(1, 2)
        main_splitter.setSizes([560, 700])

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

    # ---------- URDF 滑动条构建 ----------
    def _build_urdf_widgets(self, layout):
        ORDER = [
            "head_yaw_joint", "head_pitch_joint", "head_roll_joint",
            "left_arm_pitch_joint", "left_arm_roll_joint", "left_arm_yaw_joint", "left_elbow_ankle_joint",
            "right_arm_pitch_joint", "right_arm_roll_joint", "right_arm_yaw_joint", "right_elbow_ankle_joint",
            "left_hip_pitch_joint", "left_hip_roll_joint", "left_hip_yaw_joint",
            "left_knee_joint", "left_ankle_joint",
            "right_hip_pitch_joint", "right_hip_roll_joint", "right_hip_yaw_joint",
            "right_knee_joint", "right_ankle_joint",
        ]
        if self.urdf_robot:
            known = set(ORDER)
            names = [n for n in ORDER if n in self.urdf_robot.joint_limits]
            names += [n for n in self.urdf_robot.joint_names if n not in known]
        else:
            names = list(self.rt_map.keys())
        for name in names:
            if self.urdf_robot and name in self.urdf_robot.joint_limits:
                lower, upper = self.urdf_robot.joint_limits[name]
            elif name in self.rt_map:
                lo, hi = self.rt_map[name]["limit"]
                lower, upper = math.radians(lo), math.radians(hi)
            else:
                lower, upper = -3.14, 3.14
            w = URDFJointWidget(name, lower, upper, self._on_urdf_joint_change,
                                lambda n, c: None, use_degree=True, sync_checked=False)
            self.urdf_joint_widgets_list[name] = w
            layout.addWidget(w)

    # ---------- 模式切换 ----------
    def _switch_mode(self, key):
        order = {"bone": 0, "urdf": 1, "npz": 2}
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
        for a in arma:
            print(f"   armature '{a['name']}': {len(a['bones'])} bones")
        # 校验 retarget_map 目标骨是否在任一 armature
        allbones = {b["name"] for a in arma for b in a["bones"]}
        missing = sorted({c["bone"] for c in self.rt_map.values() if c["bone"] and c["bone"] in ("",
                          "head", "upper_arm_fk.L")} - allbones) if allbones else []
        if allbones and missing:
            print("   retarget 目标骨缺失:", missing)

    # ---------- Bone 直接控制 ----------
    def _on_bone_selected(self, row):
        if row < 0:
            return
        name, limit = DIRECT_BONES[row]
        self.bone_joint_widget.name = name
        self.bone_joint_widget.set_angles(self.bone_angles.get(name, [0, 0, 0]))

    def _on_bone_axis_change(self, bone, axis, val):
        angles = self.bone_angles.get(bone, [0, 0, 0])
        angles[AXIS_INDEX[axis]] = val
        self.bone_angles[bone] = angles
        self._push_skin_pose()

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
        if self.sync_toggle.isChecked() and self.client.connected:
            self.client.set_pose(bone_pose)
        self._push_urdf_if_needed()

    def _urdf_armature_list(self):
        return BlenderUrdfPanel._armature_list(self.burdf_map)

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
            return
        angles = self.urdf_joint_angles_rad
        for a in self._urdf_armature_list():
            pose = self._build_pose_for_armature(a.get("joints", {}), angles)
            if pose:
                self.client.set_urdf_pose(a.get("name", ""), pose)

    def _push_urdf_reset(self):
        """reset：强推所有 map 里的 Blender rig 到 rest pose（全关节 0 → 各轴 bias），
        不受 Drive Blender URDF 开关限制（reset 一致性优先），是否生效仍由面板勾选过滤。"""
        if not self.client.connected:
            return
        for a in self._urdf_armature_list():
            pose = self._build_pose_for_armature(a.get("joints", {}), {}, zero=True)
            if pose:
                self.client.set_urdf_pose(a.get("name", ""), pose)

    def _current_skin_pose(self):
        active = {n: self.urdf_joint_angles_rad.get(n, 0.0)
                  for n, w in self.urdf_joint_widgets_list.items() if w.sync_checkbox.isChecked()}
        return retarget_mod.apply_retarget_rad(active, self.rt_map)

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
        self.npz_frame += 1
        if self.npz_frame >= len(self.npz_joints):
            self.npz_frame = 0
        self.npz_panel.set_frame(self.npz_frame)

    def play_btn_pause(self):
        self.playing = False
        self.npz_panel.play_btn.setChecked(False)
        self.play_timer.stop()

    # ---------- 重载 / 复位 ----------
    def _reload_map(self):
        try:
            self.rt_map = retarget_mod.load_retarget_map(self.map_path)
            self.retarget_panel.rebuild(self.rt_map)
            print("Retarget map reloaded")
        except Exception as e:
            print("Reload map failed:", e)

    def _reload_burdf_map(self):
        self.burdf_map = self._load_burdf_map()
        self.burdf_panel.rebuild(self.burdf_map)
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

        # 清零 FK 骨 + UI
        for name in list(self.bone_angles):
            self.bone_angles[name] = [0, 0, 0]
        self.bone_joint_widget.set_angles([0, 0, 0])

        # 上行零姿态给 Blender（皮肤 + blender-urdf）
        bone_pose = self._current_skin_pose()
        self.retarget_panel.update_pose(bone_pose)
        if self.client.connected:
            self.client.set_pose(bone_pose)
            # reset 强推所有 map rig 到 rest，保持 meshcat 与勾选的 Blender URDF 一致
            self._push_urdf_reset()
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
    print("Retarget map:", os.path.abspath(RETARGET_MAP_PATH))
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