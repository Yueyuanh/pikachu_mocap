import sys
import socket
import json
import ast
import os
import math
import cv2
import mediapipe as mp
import time
from typing import Any

from PySide6.QtWidgets import *
from PySide6.QtCore import Qt, QTimer
from PySide6.QtGui import QImage, QPixmap
from matplotlib.backends.backend_qtagg import FigureCanvasQTAgg as FigureCanvas
from matplotlib.figure import Figure


HOST = "127.0.0.1"
PORT = 9999
BASE_DIR = os.path.dirname(__file__)
CONFIG_PATH = os.path.join(BASE_DIR, "addon", "config", "blender_joint_config.yaml")
SKELETON_PATH = os.path.join(BASE_DIR, "addon", "scripts", "pikachu_skeleton.yaml")
HUMANOID_SKELETON_PATH = os.path.join(BASE_DIR, "addon", "scripts", "humanoid_skeleton.yaml")
PIKACHU_POSE_SKELETON_PATH = os.path.join(BASE_DIR, "addon", "scripts", "pikachu_pose_skeleton.yaml")
MEDIA_DIR = os.path.join(BASE_DIR, "pose", "MediaPipe")

if MEDIA_DIR not in sys.path:
    sys.path.append(MEDIA_DIR)

from MediaPipe_detect import MediaPipeDetector
from Humanoid_frame import HumanoidPlotter
from Pikachu_frame import PikachuPlotter
from transfer import map_humanoid_to_pikachu, convert_humanoid_to_urdf
from transfer import Humanoid2Skeleton, Humanoid2Urdf, HumanoidPoseData

# 添加urdf相关导入
URDF_DIR = os.path.join(BASE_DIR, "urdf")
if URDF_DIR not in sys.path:
    sys.path.append(URDF_DIR)

try:
    import yaml
    _HAVE_YAML = True
except Exception:
    yaml = None
    _HAVE_YAML = False


def _parse_yaml_value(value: str) -> Any:

    if value is None:
        return ""

    v = value.strip()
    if not v:
        return ""

    if (v.startswith('"') and v.endswith('"')) or (v.startswith("'") and v.endswith("'")):
        return v[1:-1]

    if (v.startswith("[") and v.endswith("]")) or (v.startswith("{") and v.endswith("}")):
        try:
            return ast.literal_eval(v)
        except Exception:
            pass

    try:
        return int(v)
    except ValueError:
        pass

    try:
        return float(v)
    except ValueError:
        pass

    return v


def _parse_axis_spec(value):

    if value is None:
        return 0, -180, 180

    if isinstance(value, (int, float)):
        return int(value), -180, 180

    text = str(value).strip()
    if not text:
        return 0, -180, 180

    parts = text.split(",", 1)
    angle = int(float(parts[0].strip())) if parts[0].strip() else 0

    lower = -180
    upper = 180

    if len(parts) > 1:
        lim = parts[1].strip()
        lim = lim.lstrip("(").rstrip(")")
        if "," in lim:
            a, b = lim.split(",", 1)
            try:
                lower = int(float(a.strip()))
            except ValueError:
                lower = -180
            try:
                upper = int(float(b.strip()))
            except ValueError:
                upper = 180

    if lower > upper:
        lower, upper = upper, lower

    angle = max(lower, min(upper, angle))

    return angle, lower, upper


def _load_yaml_fallback(text: str) -> dict:

    cfg = {"bones": []}
    current = None

    for raw in text.splitlines():
        line = raw.strip()
        if not line or line.startswith("#"):
            continue
        if line.startswith("bones:"):
            continue
        if line.startswith("-"):
            if current:
                cfg["bones"].append(current)
            current = {}
            rest = line[1:].strip()
            if rest and ":" in rest:
                key, value = rest.split(":", 1)
                current[key.strip()] = _parse_yaml_value(value)
            continue
        if ":" in line:
            key, value = line.split(":", 1)
            if current is not None:
                current[key.strip()] = _parse_yaml_value(value)
            else:
                cfg[key.strip()] = _parse_yaml_value(value)

    if current:
        cfg["bones"].append(current)

    return cfg


def _dump_yaml_fallback(cfg: dict) -> str:

    lines = ["bones:"]
    for bone in cfg.get("bones", []):
        name = bone.get("name", "")
        x = bone.get("x", "0,(-180,180)")
        y = bone.get("y", "0,(-180,180)")
        z = bone.get("z", "0,(-180,180)")
        lines.append(f"  - name: {name}")
        lines.append(f"    x: {x}")
        lines.append(f"    y: {y}")
        lines.append(f"    z: {z}")
    return "\n".join(lines) + "\n"


def _dump_skeleton_fallback(data: dict) -> str:

    lines = ["bones:"]
    for bone in data.get("bones", []):
        name = bone.get("name", "")
        parent = bone.get("parent") or ""
        head = bone.get("head", [0.0, 0.0, 0.0])
        tail = bone.get("tail", [0.0, 0.0, 0.0])
        matrix = bone.get("matrix")
        local_matrix = bone.get("local_matrix")
        lines.append(f"  - name: {name}")
        lines.append(f"    parent: {parent}")
        lines.append(f"    head: [{head[0]}, {head[1]}, {head[2]}]")
        lines.append(f"    tail: [{tail[0]}, {tail[1]}, {tail[2]}]")
        if matrix is not None:
            lines.append(f"    matrix: {json.dumps(matrix)}")
        if local_matrix is not None:
            lines.append(f"    local_matrix: {json.dumps(local_matrix)}")
    return "\n".join(lines) + "\n"


def _parse_vec3(value):

    if isinstance(value, (list, tuple)) and len(value) >= 3:
        return [float(value[0]), float(value[1]), float(value[2])]

    if value is None:
        return [0.0, 0.0, 0.0]

    text = str(value).strip().lstrip("[").rstrip("]")
    if not text:
        return [0.0, 0.0, 0.0]

    parts = [p.strip() for p in text.split(",") if p.strip()]
    if len(parts) < 3:
        return [0.0, 0.0, 0.0]

    try:
        return [float(parts[0]), float(parts[1]), float(parts[2])]
    except ValueError:
        return [0.0, 0.0, 0.0]


def _parse_matrix_value(value):
    if isinstance(value, list):
        return value
    if value is None:
        return None
    text = str(value).strip()
    if not text:
        return None
    if (text.startswith("[") and text.endswith("]")) or (text.startswith("{") and text.endswith("}")):
        try:
            return ast.literal_eval(text)
        except Exception:
            try:
                return json.loads(text)
            except Exception:
                return None
    return None


def load_skeleton():

    if not os.path.exists(SKELETON_PATH):
        return None

    try:
        with open(SKELETON_PATH, "r", encoding="utf-8") as f:
            data = f.read()
            if _HAVE_YAML:
                return yaml.safe_load(data) or {}
            return _load_yaml_fallback(data)
    except Exception as e:
        print("Skeleton load error:", e)
        return None


def save_skeleton_to(path, data: dict):

    try:
        normalized = {"bones": []}
        for bone in data.get("bones", []):
            if not isinstance(bone, dict):
                continue
            out = {
                "name": bone.get("name"),
                "parent": bone.get("parent"),
                "head": bone.get("head"),
                "tail": bone.get("tail")
            }
            matrix = bone.get("matrix")
            if isinstance(matrix, list):
                out["matrix"] = json.dumps(matrix)
            elif matrix is not None:
                out["matrix"] = str(matrix)
            local_matrix = bone.get("local_matrix")
            if isinstance(local_matrix, list):
                out["local_matrix"] = json.dumps(local_matrix)
            elif local_matrix is not None:
                out["local_matrix"] = str(local_matrix)
            normalized["bones"].append(out)
        with open(path, "w", encoding="utf-8") as f:
            if _HAVE_YAML:
                yaml.safe_dump(normalized, f, allow_unicode=True, sort_keys=False)
            else:
                f.write(_dump_skeleton_fallback(normalized))
    except Exception as e:
        print("Skeleton save error:", e)


def save_skeleton(data: dict):
    save_skeleton_to(SKELETON_PATH, data)


def load_config():

    if not os.path.exists(CONFIG_PATH):
        default = {
            "bones": [
                {"name": "upper_arm_fk.L", "x": "0,(-180,180)", "y": "0,(-180,180)", "z": "0,(-180,180)"},
                {"name": "forearm_fk.L", "x": "0,(-180,180)", "y": "0,(-180,180)", "z": "0,(-180,180)"},
                {"name": "hand_fk.L", "x": "0,(-180,180)", "y": "0,(-180,180)", "z": "0,(-180,180)"},
                {"name": "upper_arm_fk.R", "x": "0,(-180,180)", "y": "0,(-180,180)", "z": "0,(-180,180)"},
                {"name": "forearm_fk.R", "x": "0,(-180,180)", "y": "0,(-180,180)", "z": "0,(-180,180)"},
                {"name": "hand_fk.R", "x": "0,(-180,180)", "y": "0,(-180,180)", "z": "0,(-180,180)"}
            ]
        }
        save_config(default)
        return default

    try:
        with open(CONFIG_PATH, "r", encoding="utf-8") as f:
            data = f.read()
            if _HAVE_YAML:
                return yaml.safe_load(data) or {"bones": []}
            return _load_yaml_fallback(data)
    except Exception as e:
        print("Config load error:", e)
        return {"bones": []}


def save_config(cfg):

    try:
        with open(CONFIG_PATH, "w", encoding="utf-8") as f:
            if _HAVE_YAML:
                yaml.safe_dump(cfg, f, allow_unicode=True, sort_keys=False)
            else:
                f.write(_dump_yaml_fallback(cfg))
    except Exception as e:
        print("Config save error:", e)


class BlenderClient:

    def __init__(self, on_message=None):

        self.sock = None
        self.connected = False
        self.on_message = on_message
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
            self.sock.sendall((json.dumps(data)+"\n").encode())
        except Exception:
            self._handle_disconnect()

    def set_joint(self, bone, axis, angle):

        self.send({
            "type": "set_joint",
            "bone": bone,
            "axis": axis,
            "angle": angle
        })

    def set_pose(self, pose):
        if not pose:
            return
        self.send({
            "type": "set_pose",
            "pose": pose
        })

    def request_pose(self):
        self.send({"type": "request_pose"})

    def request_bones(self):
        self.send({"type": "request_bones"})

    def request_transforms(self, bones=None):
        payload = {"type": "request_transforms"}
        if bones:
            payload["bones"] = list(bones)
        self.send(payload)

    def _try_connect(self):

        if self.connected:
            return

        try:
            self.sock = socket.socket(socket.AF_INET, socket.SOCK_STREAM)
            self.sock.settimeout(1.0)
            print("Connecting to Blender server...")
            self.sock.connect((HOST,PORT))
            self.sock.setblocking(False)
            self.connected = True
            self._retry_timer.stop()
            self._recv_timer.start()
            print("Connected!")
        except Exception as e:
            if not self._retry_timer.isActive():
                self._retry_timer.start()
            print("Connect failed:", e)
            self._handle_disconnect(retry=False)

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
                print("Socket decode error:", line[:200])
                continue
            if self.on_message:
                self.on_message(msg)

    def _handle_disconnect(self, retry=True):

        self.connected = False
        self._recv_timer.stop()

        if self.sock is not None:
            try:
                self.sock.close()
            except Exception:
                pass
            self.sock = None

        if retry and not self._retry_timer.isActive():
            self._retry_timer.start()


class AxisSlider(QWidget):

    def __init__(self, label, on_change):

        super().__init__()

        layout = QHBoxLayout()

        layout.setContentsMargins(0, 0, 0, 0)

        self.label = QLabel(label)
        self.label.setFixedWidth(40)

        self.slider = QSlider(Qt.Horizontal)
        self.slider.setMinimum(-180)
        self.slider.setMaximum(180)
        self.slider.setValue(0)
        self.slider.valueChanged.connect(on_change)

        self.value = QLabel("0")
        self.value.setFixedWidth(40)
        self.value.setAlignment(Qt.AlignRight | Qt.AlignVCenter)

        self.range_label = QLabel("(-180,180)")
        self.range_label.setFixedWidth(90)
        self.range_label.setAlignment(Qt.AlignRight | Qt.AlignVCenter)

        layout.addWidget(self.label)
        layout.addWidget(self.slider)
        layout.addWidget(self.value)
        layout.addWidget(self.range_label)

        self.setLayout(layout)

    def set_value(self, val):
        self.slider.setValue(val)
        self.value.setText(str(val))

    def update_value_label(self, val):
        self.value.setText(str(val))

    def set_range(self, lower, upper):
        self.slider.setMinimum(lower)
        self.slider.setMaximum(upper)
        self.range_label.setText(f"({lower},{upper})")


class JointPanel(QWidget):

    def __init__(self, client, on_axis_change, get_angles, get_limits):

        super().__init__()

        self.client = client
        self.on_axis_change = on_axis_change
        self.get_angles = get_angles
        self.get_limits = get_limits
        self.bone = None
        self.axis_sliders = {}

        layout = QVBoxLayout()

        self.title = QLabel("Select a bone")
        self.title.setAlignment(Qt.AlignLeft | Qt.AlignVCenter)
        self.title.setStyleSheet("font-size: 16px; font-weight: 600;")

        layout.addWidget(self.title)

        for axis in ["x", "y", "z"]:
            slider = AxisSlider(axis.upper(), self._make_on_change(axis))
            self.axis_sliders[axis] = slider
            layout.addWidget(slider)

        layout.addStretch(1)

        self.setLayout(layout)

        self.setEnabled(False)

    def _make_on_change(self, axis):

        def _on_change(val):
            if not self.bone:
                return
            self.axis_sliders[axis].update_value_label(val)
            self.client.set_joint(self.bone, axis, val)
            self.on_axis_change(self.bone, axis, val)

        return _on_change

    def set_bone(self, bone):

        self.bone = bone
        self.title.setText(f"Bone: {bone}")
        angles = self.get_angles(bone)
        limits = self.get_limits(bone)
        self._set_limits(limits)
        self._set_angles(self._clamp_angles(angles, limits))
        self.setEnabled(True)

    def _set_angles(self, angles):

        for axis, val in zip(["x", "y", "z"], angles):
            slider = self.axis_sliders[axis]
            slider.slider.blockSignals(True)
            slider.set_value(val)
            slider.slider.blockSignals(False)

    def _set_limits(self, limits):

        for axis in ["x", "y", "z"]:
            lower, upper = limits.get(axis, (-180, 180))
            self.axis_sliders[axis].set_range(lower, upper)

    def _clamp_angles(self, angles, limits):

        result = []
        for axis, val in zip(["x", "y", "z"], angles):
            lower, upper = limits.get(axis, (-180, 180))
            result.append(max(lower, min(upper, int(val))))
        return result


class BoneItem(QWidget):

    def __init__(self, name, angles, on_select, on_toggle=None, checked=True):

        super().__init__()

        self.name = name

        layout = QHBoxLayout()
        layout.setContentsMargins(0, 0, 0, 0)

        self.sync_checkbox = QCheckBox()
        self.sync_checkbox.setChecked(bool(checked))
        self.sync_checkbox.setToolTip("Sync from pose")
        if on_toggle:
            self.sync_checkbox.toggled.connect(lambda val, n=name: on_toggle(n, val))

        self.button = QPushButton(name)
        self.button.clicked.connect(lambda checked=False: on_select(name))

        x, y, z = angles
        self.value = QLabel(f"X(:{x}  Y:{y}  Z:{z}")
        self.value.setAlignment(Qt.AlignRight | Qt.AlignVCenter)
        self.value.setFixedWidth(140)

        layout.addWidget(self.sync_checkbox, 0)
        layout.addWidget(self.button, 1)
        layout.addWidget(self.value, 0)

        self.setLayout(layout)

    def set_angles(self, angles):

        x, y, z = angles
        self.value.setText(f"X:{x}  Y:{y}  Z:{z}")

    def set_sync_checked(self, checked):
        self.sync_checkbox.setChecked(bool(checked))


class SkeletonPlot(QWidget):

    def __init__(self):

        super().__init__()

        self.fig = Figure(figsize=(5, 4))
        self.canvas = FigureCanvas(self.fig)
        self.ax = self.fig.add_subplot(111, projection="3d")
        self.ax.set_title("Skeleton")
        self.ax.set_xlabel("X")
        self.ax.set_ylabel("Y")
        self.ax.set_zlabel("Z")
        self.ax.grid(True)
        self.ax.view_init(elev=20, azim=-60)

        self.lines = {}
        self.axis_lines = {}
        self.axis_colors = ("#ff3b30", "#34c759", "#007aff")
        self.points = self.ax.scatter([], [], [], s=10, color="#34c759")
        self.bones = {}
        self.bone_order = []
        self.angles = {}
        self.axis_scale = 0.35
        self.axis_min = 0.1
        self.visible_bones = None

        layout = QVBoxLayout()
        layout.setContentsMargins(0, 0, 0, 0)
        layout.addWidget(self.canvas)
        self.setLayout(layout)

    def _set_axes_equal(self, points):

        if not points:
            return

        xs, ys, zs = zip(*points)
        min_x, max_x = min(xs), max(xs)
        min_y, max_y = min(ys), max(ys)
        min_z, max_z = min(zs), max(zs)

        max_range = max(
            max_x - min_x,
            max_y - min_y,
            max_z - min_z,
            1e-6
        )

        mid_x = (min_x + max_x) / 2.0
        mid_y = (min_y + max_y) / 2.0
        mid_z = (min_z + max_z) / 2.0

        half = max_range / 2.0
        self.ax.set_xlim(mid_x - half, mid_x + half)
        self.ax.set_ylim(mid_y - half, mid_y + half)
        self.ax.set_zlim(mid_z - half, mid_z + half)

    def _matmul(self, a, b):
        return [
            [
                a[0][0] * b[0][j] + a[0][1] * b[1][j] + a[0][2] * b[2][j]
                for j in range(3)
            ],
            [
                a[1][0] * b[0][j] + a[1][1] * b[1][j] + a[1][2] * b[2][j]
                for j in range(3)
            ],
            [
                a[2][0] * b[0][j] + a[2][1] * b[1][j] + a[2][2] * b[2][j]
                for j in range(3)
            ],
        ]

    def _matmul4(self, a, b):
        return [
            [
                a[i][0] * b[0][j] +
                a[i][1] * b[1][j] +
                a[i][2] * b[2][j] +
                a[i][3] * b[3][j]
                for j in range(4)
            ]
            for i in range(4)
        ]

    def _identity4(self):
        return [
            [1.0, 0.0, 0.0, 0.0],
            [0.0, 1.0, 0.0, 0.0],
            [0.0, 0.0, 1.0, 0.0],
            [0.0, 0.0, 0.0, 1.0],
        ]

    def _rot4_xyz(self, angles_deg):
        ax = math.radians(angles_deg[0])
        ay = math.radians(angles_deg[1])
        az = math.radians(angles_deg[2])
        r3 = self._euler_xyz(ax, ay, az)
        return [
            [r3[0][0], r3[0][1], r3[0][2], 0.0],
            [r3[1][0], r3[1][1], r3[1][2], 0.0],
            [r3[2][0], r3[2][1], r3[2][2], 0.0],
            [0.0, 0.0, 0.0, 1.0],
        ]

    def _transform_point(self, mat, point):
        x, y, z = point
        return (
            mat[0][0] * x + mat[0][1] * y + mat[0][2] * z + mat[0][3],
            mat[1][0] * x + mat[1][1] * y + mat[1][2] * z + mat[1][3],
            mat[2][0] * x + mat[2][1] * y + mat[2][2] * z + mat[2][3],
        )

    def _base_from_head_tail(self, head, tail):
        x_axis, y_axis, z_axis = self._basis_from_bone(head, tail)
        hx, hy, hz = head
        return [
            [x_axis[0], y_axis[0], z_axis[0], hx],
            [x_axis[1], y_axis[1], z_axis[1], hy],
            [x_axis[2], y_axis[2], z_axis[2], hz],
            [0.0, 0.0, 0.0, 1.0],
        ]

    def _build_order(self):
        children = {name: [] for name in self.bones}
        roots = []
        for name, data in self.bones.items():
            parent = data.get("parent")
            if parent and parent in self.bones:
                children[parent].append(name)
            else:
                roots.append(name)

        order = []
        visited = set()

        def _dfs(node):
            if node in visited:
                return
            visited.add(node)
            order.append(node)
            for child in children.get(node, []):
                _dfs(child)

        for root in roots:
            _dfs(root)
        for name in self.bones:
            if name not in visited:
                _dfs(name)

        self.bone_order = order
    def _normalize(self, v, eps=1e-8):
        x, y, z = v
        n = (x * x + y * y + z * z) ** 0.5
        if n < eps:
            return (0.0, 0.0, 0.0)
        return (x / n, y / n, z / n)

    def _cross(self, a, b):
        return (
            a[1] * b[2] - a[2] * b[1],
            a[2] * b[0] - a[0] * b[2],
            a[0] * b[1] - a[1] * b[0],
        )

    def _basis_from_matrix(self, mat):
        if not mat or len(mat) < 3 or len(mat[0]) < 3:
            return None
        x_axis = (mat[0][0], mat[1][0], mat[2][0])
        y_axis = (mat[0][1], mat[1][1], mat[2][1])
        z_axis = (mat[0][2], mat[1][2], mat[2][2])
        return (
            self._normalize(x_axis),
            self._normalize(y_axis),
            self._normalize(z_axis),
        )

    def _basis_from_bone(self, head, tail):
        hx, hy, hz = head
        tx, ty, tz = tail
        y_axis = self._normalize((tx - hx, ty - hy, tz - hz))
        up = (0.0, 0.0, 1.0)
        if abs(y_axis[2]) > 0.99:
            up = (0.0, 1.0, 0.0)
        x_axis = self._normalize(self._cross(up, y_axis))
        z_axis = self._normalize(self._cross(y_axis, x_axis))
        return x_axis, y_axis, z_axis

    def _euler_xyz(self, ax, ay, az):
        cx, sx = math.cos(ax), math.sin(ax)
        cy, sy = math.cos(ay), math.sin(ay)
        cz, sz = math.cos(az), math.sin(az)
        rx = [
            [1.0, 0.0, 0.0],
            [0.0, cx, -sx],
            [0.0, sx, cx],
        ]
        ry = [
            [cy, 0.0, sy],
            [0.0, 1.0, 0.0],
            [-sy, 0.0, cy],
        ]
        rz = [
            [cz, -sz, 0.0],
            [sz, cz, 0.0],
            [0.0, 0.0, 1.0],
        ]
        return self._matmul(rz, self._matmul(ry, rx))

    def update_angles(self, angles):
        self.angles = angles or {}
        self._redraw_scene()

    def set_visible_bones(self, bones):
        if bones:
            self.visible_bones = set(bones)
        else:
            self.visible_bones = None
        self._redraw_scene()

    def update_transforms(self, transforms):

        self.bones = {}

        for bone in transforms:
            name = bone.get("name")
            head = bone.get("head")
            tail = bone.get("tail")
            if not name or not head or not tail:
                continue
            if len(head) != 3 or len(tail) != 3:
                continue
            length = ((tail[0] - head[0]) ** 2 + (tail[1] - head[1]) ** 2 + (tail[2] - head[2]) ** 2) ** 0.5
            self.bones[name] = {
                "parent": bone.get("parent"),
                "head": head,
                "tail": tail,
                "matrix": bone.get("matrix"),
                "local_matrix": bone.get("local_matrix"),
                "length": length
            }

        self._build_order()
        self._redraw_scene()

    def _compute_pose(self):
        worlds = {}
        heads = {}
        tails = {}

        for name in self.bone_order:
            data = self.bones[name]
            angles = self.angles.get(name, [0.0, 0.0, 0.0])
            rot = self._rot4_xyz(angles)

            parent = data.get("parent")
            local = data.get("local_matrix")
            base = data.get("matrix")

            if parent and parent in worlds and local:
                world = self._matmul4(worlds[parent], self._matmul4(local, rot))
            else:
                if not base:
                    if local:
                        base = local
                    else:
                        base = self._base_from_head_tail(data["head"], data["tail"])
                world = self._matmul4(base, rot)

            worlds[name] = world
            heads[name] = self._transform_point(world, (0.0, 0.0, 0.0))
            tails[name] = self._transform_point(world, (0.0, data["length"], 0.0))

        return worlds, heads, tails

    def _redraw_scene(self):

        if not self.bones:
            for line in self.lines.values():
                line.set_data_3d([], [], [])
            for line_set in self.axis_lines.values():
                for line in line_set:
                    line.set_data_3d([], [], [])
            self.points._offsets3d = ([], [], [])
            self.canvas.draw_idle()
            return

        worlds, heads, tails = self._compute_pose()

        points = []
        seen = set()

        for name, head in heads.items():
            if self.visible_bones is not None and name not in self.visible_bones:
                continue
            tail = tails.get(name)
            if not tail:
                continue
            hx, hy, hz = head
            tx, ty, tz = tail
            points.append((hx, hy, hz))
            points.append((tx, ty, tz))
            line = self.lines.get(name)
            if line is None:
                line, = self.ax.plot([], [], [], linewidth=2, color="#007aff")
                self.lines[name] = line
            line.set_data_3d([hx, tx], [hy, ty], [hz, tz])
            seen.add(name)

        for name, line in list(self.lines.items()):
            if name not in seen:
                line.set_data_3d([], [], [])

        if points:
            xs, ys, zs = zip(*points)
            self.points._offsets3d = (xs, ys, zs)
            self._set_axes_equal(points)
        else:
            self.points._offsets3d = ([], [], [])

        axis_seen = set()
        for name, world in worlds.items():
            if self.visible_bones is not None and name not in self.visible_bones:
                continue
            head = heads.get(name)
            tail = tails.get(name)
            if not head or not tail:
                continue
            basis = self._basis_from_matrix(world)
            if basis is None:
                basis = self._basis_from_bone(head, tail)
            x_axis, y_axis, z_axis = basis

            hx, hy, hz = head
            length = ((tail[0] - hx) ** 2 + (tail[1] - hy) ** 2 + (tail[2] - hz) ** 2) ** 0.5
            axis_len = max(self.axis_min, length * self.axis_scale)

            line_set = self.axis_lines.get(name)
            if line_set is None:
                x_line, = self.ax.plot([], [], [], linewidth=1, color=self.axis_colors[0])
                y_line, = self.ax.plot([], [], [], linewidth=1, color=self.axis_colors[1])
                z_line, = self.ax.plot([], [], [], linewidth=1, color=self.axis_colors[2])
                line_set = (x_line, y_line, z_line)
                self.axis_lines[name] = line_set

            x_line, y_line, z_line = line_set
            x_line.set_data_3d(
                [hx, hx + axis_len * x_axis[0]],
                [hy, hy + axis_len * x_axis[1]],
                [hz, hz + axis_len * x_axis[2]],
            )
            y_line.set_data_3d(
                [hx, hx + axis_len * y_axis[0]],
                [hy, hy + axis_len * y_axis[1]],
                [hz, hz + axis_len * y_axis[2]],
            )
            z_line.set_data_3d(
                [hx, hx + axis_len * z_axis[0]],
                [hy, hy + axis_len * z_axis[1]],
                [hz, hz + axis_len * z_axis[2]],
            )
            axis_seen.add(name)

        for name, line_set in list(self.axis_lines.items()):
            if name not in axis_seen:
                for line in line_set:
                    line.set_data_3d([], [], [])

        self.canvas.draw_idle()


class URDFJointWidget(QWidget):

    def __init__(self, name, lower, upper, on_change, on_sync_change, use_degree=True, sync_checked=True):

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

        # 勾选框和标题行
        header_layout = QHBoxLayout()
        header_layout.setContentsMargins(0, 0, 0, 0)

        self.sync_checkbox = QCheckBox()
        self.sync_checkbox.setChecked(bool(self.sync_checked))
        self.sync_checkbox.toggled.connect(self._on_sync_toggled)

        title_label = QLabel(name)
        title_label.setStyleSheet("font-size: 12px; font-weight: 600;")

        header_layout.addWidget(self.sync_checkbox, 0)
        header_layout.addWidget(title_label, 1)

        layout.addLayout(header_layout)

        # 滑动条区域
        slider_layout = QHBoxLayout()
        slider_layout.setContentsMargins(20, 0, 0, 0)  # 左边距20，与勾选框对齐

        self.slider = QSlider(Qt.Horizontal)
        self.slider.setMinimumWidth(250)  # 增加滑动条最小宽度
        self.slider.setMaximumWidth(500)  # 增加滑动条最大宽度

        # 将弧度转换为度数（如果使用度数模式）
        if self.use_degree:
            lower_deg = int(lower * 180 / math.pi)
            upper_deg = int(upper * 180 / math.pi)
        else:
            lower_deg = int(lower * 100)  # 弧度模式，放大100倍提高精度
            upper_deg = int(upper * 100)

        self.slider.setMinimum(lower_deg)
        self.slider.setMaximum(upper_deg)
        self.slider.setValue(0)
        self.slider.valueChanged.connect(self._on_slider_change)

        slider_layout.addWidget(self.slider, 1)

        # 数值显示（移除angle字样，固定宽度）
        self.value_label = QLabel()
        self.value_label.setFixedWidth(200)  # 固定宽度，确保数字变化时不影响布局
        self.value_label.setAlignment(Qt.AlignLeft | Qt.AlignVCenter)
        self.update_value_label(0.0)

        slider_layout.addWidget(self.value_label, 0)

        layout.addLayout(slider_layout)

        self.setLayout(layout)

    def _on_slider_change(self, value):
        # 根据模式计算实际角度
        if self.use_degree:
            angle_deg = value
            angle_rad = value * math.pi / 180.0
        else:
            angle_rad = value / 100.0
            angle_deg = angle_rad * 180.0 / math.pi

        self.update_value_label(angle_rad)

        # 调用回调函数，传入弧度值
        if self.on_change:
            self.on_change(self.name, angle_rad)

    def _on_sync_toggled(self, checked):
        self.sync_checked = bool(checked)
        if self.on_sync_change:
            self.on_sync_change(self.name, checked)

    def update_value_label(self, angle_rad):
        """更新数值标签显示（移除angle字样）"""
        angle_deg = angle_rad * 180.0 / math.pi

        lower_deg = self.lower * 180.0 / math.pi
        upper_deg = self.upper * 180.0 / math.pi

        self.value_label.setText(f"{angle_rad:.2f}(rad) {angle_deg:.1f}(deg) ({lower_deg:.0f},{upper_deg:.0f})")

    def set_use_degree(self, use_degree):
        """切换rad/degree模式"""
        if self.use_degree == use_degree:
            return

        self.use_degree = use_degree

        # 保存当前值
        current_value = self.slider.value()
        current_angle_rad = current_value * math.pi / 180.0 if not use_degree else current_value / 100.0

        # 更新滑动条范围
        if use_degree:
            lower_deg = int(self.lower * 180 / math.pi)
            upper_deg = int(self.upper * 180 / math.pi)
        else:
            lower_deg = int(self.lower * 100)
            upper_deg = int(self.upper * 100)

        self.slider.blockSignals(True)
        self.slider.setMinimum(lower_deg)
        self.slider.setMaximum(upper_deg)

        # 设置当前值
        if use_degree:
            self.slider.setValue(int(current_angle_rad * 180.0 / math.pi))
        else:
            self.slider.setValue(int(current_angle_rad * 100.0))

        self.slider.blockSignals(False)

        # 更新显示
        self.update_value_label(current_angle_rad)

    def set_angle(self, angle_rad):
        """设置关节角度（弧度）"""
        if self.use_degree:
            value = int(angle_rad * 180.0 / math.pi)
        else:
            value = int(angle_rad * 100.0)

        self.slider.blockSignals(True)
        self.slider.setValue(value)
        self.slider.blockSignals(False)

        self.update_value_label(angle_rad)


class URDFJointPanel(QWidget):

    def __init__(self, robot_model, on_change):

        super().__init__()

        self.robot = robot_model
        self.on_change = on_change

        layout = QVBoxLayout()

        # 标题
        self.title = QLabel("URDF Info")
        self.title.setStyleSheet("font-size: 14px; font-weight: 600;")
        layout.addWidget(self.title)

        # 显示简化的URDF信息
        if self.robot:
            joint_count = len(self.robot.joint_names)

            info_text = QLabel(f"Total joints: {joint_count}")
            info_text.setStyleSheet("font-size: 12px; color: #666;")
            layout.addWidget(info_text)

            # 显示前几个joint的名称
            joint_names = self.robot.joint_names[:5]
            if joint_names:
                joints_text = QLabel(f"Sample joints: {', '.join(joint_names)}")
                joints_text.setStyleSheet("font-size: 11px; color: #888;")
                layout.addWidget(joints_text)

            if len(self.robot.joint_names) > 5:
                more_text = QLabel(f"... and {len(self.robot.joint_names) - 5} more")
                more_text.setStyleSheet("font-size: 11px; color: #888;")
                layout.addWidget(more_text)

        layout.addStretch(1)

        self.setLayout(layout)


class Studio(QWidget):

    def _make_panel(self, title, widget, actions=None, left_actions=None):

        frame = QFrame()
        frame.setFrameShape(QFrame.StyledPanel)
        layout = QVBoxLayout()
        layout.setContentsMargins(6, 6, 6, 6)
        header = QHBoxLayout()
        header.setContentsMargins(0, 0, 0, 0)

        # 左侧按钮
        if left_actions:
            for btn in left_actions:
                header.addWidget(btn)

        label = QLabel(title)
        label.setStyleSheet("font-size: 13px; font-weight: 600;")
        header.addWidget(label)
        header.addStretch(1)

        # 右侧按钮
        if actions:
            for btn in actions:
                header.addWidget(btn)

        layout.addLayout(header)
        layout.addWidget(widget, 1)
        frame.setLayout(layout)
        return frame

    def __init__(self, urdf_path=None):

        super().__init__()

        self.client = BlenderClient(self.on_blender_message)

        # 持有转换器实例，方便外部通过 set_bias() 微调角度
        self.skeleton_converter = Humanoid2Skeleton()
        self.urdf_converter = Humanoid2Urdf()

        # 创建URDF模型和查看器
        self.urdf_robot = None
        self.urdf_viewer = None
        if urdf_path and os.path.exists(urdf_path):
            try:
                from robot_model import RobotModel
                from robot_viewer import RobotViewer
                self.urdf_robot = RobotModel(urdf_path)
                self.urdf_viewer = RobotViewer(self.urdf_robot)
                print(f"URDF loaded: {urdf_path}")
            except Exception as e:
                print(f"Failed to load URDF: {e}")
        self.bone_angles = {}
        self.bone_items = {}
        self.bone_order = []
        self.bone_limits = {}
        self.bone_tree = []
        self.bone_sync = {}
        self._updating_select_all = False
        self._connected_once = False
        self._pending_save_skeleton = False
        self._skeleton_loaded = False
        self._skeleton_mtime = None
        self._requested_bones_count = None
        self._save_timeout_timer = QTimer()
        self._save_timeout_timer.setInterval(2000)
        self._save_timeout_timer.setSingleShot(True)
        self._save_timeout_timer.timeout.connect(self._on_save_timeout)
        self.last_pose_time = 0.0
        self.pose_interval = 1.0 / 12.0
        self.sync_threshold = 1
        self._last_sent_angles = {}
        self.config = load_config()
        bones = self.config.get("bones", [])

        main_layout = QHBoxLayout()
        main_layout.setContentsMargins(12, 12, 12, 12)

        list_panel = QFrame()
        list_panel.setFrameShape(QFrame.StyledPanel)
        list_layout = QVBoxLayout()

        self.status_label = QLabel("Status: Waiting")
        self.status_label.setStyleSheet("font-size: 13px;")
        list_layout.addWidget(self.status_label)

        self.sync_toggle = QCheckBox("Sync to Blender")
        self.sync_toggle.setChecked(False)
        self.sync_toggle.setToolTip("Auto sync MediaPipe pose to Blender")
        self.sync_toggle.toggled.connect(self._on_sync_toggled)
        list_layout.addWidget(self.sync_toggle)

        self.plot_sync_toggle = QCheckBox("Sync to Plot")
        self.plot_sync_toggle.setChecked(True)
        self.plot_sync_toggle.setToolTip("Preview MediaPipe pose in skeleton plot only")
        self.plot_sync_toggle.toggled.connect(self._on_plot_sync_toggled)
        list_layout.addWidget(self.plot_sync_toggle)

        self.reset_btn = QPushButton("Reset All")
        self.reset_btn.clicked.connect(self.reset_all)
        list_layout.addWidget(self.reset_btn)

        self.save_skeleton_btn = QPushButton("Save Skeleton")
        self.save_skeleton_btn.clicked.connect(self.save_skeleton_from_blender)
        list_layout.addWidget(self.save_skeleton_btn)

        list_title = QLabel("Bones")
        list_title.setStyleSheet("font-size: 16px; font-weight: 600;")
        list_layout.addWidget(list_title)

        # 添加Bone和URDF切换按钮
        mode_switch_layout = QHBoxLayout()
        mode_switch_layout.setContentsMargins(0, 0, 0, 0)

        self.bone_mode_btn = QPushButton("Bone")
        self.bone_mode_btn.setCheckable(True)
        self.bone_mode_btn.setChecked(True)
        self.bone_mode_btn.clicked.connect(lambda: self._switch_mode("bone"))
        self.bone_mode_btn.setStyleSheet("""
            QPushButton {
                font-size: 12px;
                padding: 4px 8px;
                border: 1px solid #ccc;
                border-radius: 3px;
            }
            QPushButton:checked {
                background: #2d6cdf;
                color: white;
                border: 1px solid #2d6cdf;
            }
        """)

        self.urdf_mode_btn = QPushButton("URDF")
        self.urdf_mode_btn.setCheckable(True)
        self.urdf_mode_btn.setChecked(False)
        self.urdf_mode_btn.clicked.connect(lambda: self._switch_mode("urdf"))
        self.urdf_mode_btn.setStyleSheet("""
            QPushButton {
                font-size: 12px;
                padding: 4px 8px;
                border: 1px solid #ccc;
                border-radius: 3px;
            }
            QPushButton:checked {
                background: #2d6cdf;
                color: white;
                border: 1px solid #2d6cdf;
            }
        """)

        mode_switch_layout.addWidget(self.bone_mode_btn)
        mode_switch_layout.addWidget(self.urdf_mode_btn)
        list_layout.addLayout(mode_switch_layout)

        # 创建URDF joint panel
        self.urdf_joint_panel = URDFJointPanel(
            self.urdf_robot,
            self._on_urdf_joint_change
        )

        list_scroll = QScrollArea()
        list_scroll.setWidgetResizable(True)

        # 创建列表stacked widget
        self.list_stacked_widget = QStackedWidget()

        # Bone列表container
        list_container = QWidget()
        list_container_layout = QVBoxLayout()

        self.select_all_checkbox = QCheckBox("Select All")
        self.select_all_checkbox.setChecked(False)
        self.select_all_checkbox.toggled.connect(self._on_select_all_changed)
        list_container_layout.addWidget(self.select_all_checkbox)

        self.joint_panel = JointPanel(
            self.client,
            self.on_axis_change,
            self.get_angles,
            self.get_limits
        )

        self.skeleton_plot = SkeletonPlot()

        self.camera_label = QLabel("Camera")
        self.camera_label.setAlignment(Qt.AlignCenter)
        self.camera_label.setMinimumSize(320, 240)
        self.camera_label.setStyleSheet("background: #111; color: #ddd;")

        hide_labels = [
            "LEFT_EYE_INNER",
            "LEFT_EYE_OUTER",
            "RIGHT_EYE_INNER",
            "RIGHT_EYE_OUTER",
            "MOUTH_LEFT",
            "MOUTH_RIGHT",
            "LEFT_EAR",
            "RIGHT_EAR",
            "RIGHT_HEEL",
            "LEFT_HEEL",
            "LEFT_PINKY",
            "RIGHT_PINKY",
            "LEFT_INDEX",
            "RIGHT_INDEX",
            "LEFT_THUMB",
            "RIGHT_THUMB",
        ]

        self.detector = MediaPipeDetector()
        humanoid_fig = Figure()
        pikachu_fig = Figure()
        self.humanoid_plotter = HumanoidPlotter(
            show_grid=True,
            show_global_axes=False,
            show_names=False,
            show_axes=False,
            hide_labels=hide_labels,
            figure=humanoid_fig,
        )
        self.humanoid_canvas = FigureCanvas(humanoid_fig)

        self.pikachu_plotter = PikachuPlotter(
            show_grid=True,
            show_global_axes=False,
            show_names=False,
            show_axes=False,
            hide_labels=hide_labels,
            config_path=os.path.join(MEDIA_DIR, "pikachu.yaml"),
            figure=pikachu_fig,
        )
        self.pikachu_canvas = FigureCanvas(pikachu_fig)

        self.humanoid_names_btn = QPushButton("N")
        self.humanoid_names_btn.setCheckable(True)
        self.humanoid_names_btn.setToolTip("Show names (-n)")
        self.humanoid_axes_btn = QPushButton("A")
        self.humanoid_axes_btn.setCheckable(True)
        self.humanoid_axes_btn.setToolTip("Show axes (-a)")
        self.humanoid_angles_btn = QPushButton("Ang")
        self.humanoid_angles_btn.setCheckable(True)
        self.humanoid_angles_btn.setToolTip("Show joint angles")
        self.humanoid_save_btn = QPushButton("S")
        self.humanoid_save_btn.setToolTip("Save humanoid skeleton")

        self.pikachu_names_btn = QPushButton("N")
        self.pikachu_names_btn.setCheckable(True)
        self.pikachu_names_btn.setToolTip("Show names (-n)")
        self.pikachu_axes_btn = QPushButton("A")
        self.pikachu_axes_btn.setCheckable(True)
        self.pikachu_axes_btn.setToolTip("Show axes (-a)")
        self.pikachu_save_btn = QPushButton("S")
        self.pikachu_save_btn.setToolTip("Save pikachu skeleton")

        for btn in [
            self.humanoid_names_btn,
            self.humanoid_axes_btn,
            self.humanoid_angles_btn,
            self.humanoid_save_btn,
            self.pikachu_names_btn,
            self.pikachu_axes_btn,
            self.pikachu_save_btn,
        ]:
            btn.setFixedSize(24, 20)
            btn.setStyleSheet(                "QPushButton { font-size: 11px; padding: 0 2px; }"
                "QPushButton:checked { background: #2d6cdf; color: white; }"
            )

        self.humanoid_names_btn.toggled.connect(self.humanoid_plotter.set_show_names)
        self.humanoid_axes_btn.toggled.connect(self.humanoid_plotter.set_show_axes)
        self.humanoid_angles_btn.setFixedSize(32, 20)
        self.humanoid_angles_btn.toggled.connect(self.humanoid_plotter.set_show_angles)
        self.humanoid_save_btn.clicked.connect(self.save_humanoid_skeleton)
        self.pikachu_names_btn.toggled.connect(self.pikachu_plotter.set_show_names)
        self.pikachu_axes_btn.toggled.connect(self.pikachu_plotter.set_show_axes)
        self.pikachu_save_btn.clicked.connect(self.save_pikachu_skeleton)

        default_sync = self.select_all_checkbox.isChecked()

        for bone_cfg in bones:
            name = bone_cfg.get("name", "").strip()
            if not name:
                continue
            angles = [
                bone_cfg.get("x", 0),
                bone_cfg.get("y", 0),
                bone_cfg.get("z", 0)
            ]
            ax = _parse_axis_spec(angles[0])
            ay = _parse_axis_spec(angles[1])
            az = _parse_axis_spec(angles[2])
            parsed_angles = [ax[0], ay[0], az[0]]
            self.bone_angles[name] = parsed_angles
            self.bone_sync[name] = default_sync
            self.bone_limits[name] = {
                "x": (ax[1], ax[2]),
                "y": (ay[1], ay[2]),
                "z": (az[1], az[2])
            }
            item = BoneItem(
                name,
                parsed_angles,
                self.joint_panel.set_bone,
                self._on_bone_sync_changed,
                checked=default_sync
            )
            self.bone_items[name] = item
            self.bone_order.append(name)
            list_container_layout.addWidget(item)

        self.skeleton_plot.set_visible_bones(self.bone_order)

        list_container_layout.addStretch(1)
        list_container.setLayout(list_container_layout)

        # URDF joint列表container（显示所有URDF joint的滑动条）
        urdf_list_container = QWidget()
        urdf_list_container_layout = QVBoxLayout()

        # URDF joint标题和全选框
        urdf_header_layout = QHBoxLayout()
        urdf_header_layout.setContentsMargins(0, 0, 0, 0)

        urdf_title = QLabel("URDF Joints")
        urdf_title.setStyleSheet("font-size: 16px; font-weight: 600;")

        self.urdf_select_all_checkbox = QCheckBox("Select All")
        self.urdf_select_all_checkbox.setChecked(False)
        self.urdf_select_all_checkbox.toggled.connect(self._on_urdf_select_all_changed)

        urdf_header_layout.addWidget(urdf_title, 1)
        urdf_header_layout.addWidget(self.urdf_select_all_checkbox, 0)

        urdf_list_container_layout.addLayout(urdf_header_layout)

        # rad/degree切换开关
        urdf_mode_layout = QHBoxLayout()
        urdf_mode_layout.setContentsMargins(0, 0, 0, 0)

        self.urdf_rad_btn = QPushButton("rad")
        self.urdf_rad_btn.setCheckable(True)
        self.urdf_rad_btn.setChecked(False)
        self.urdf_rad_btn.clicked.connect(lambda: self._set_urdf_mode(False))
        self.urdf_rad_btn.setStyleSheet("""
            QPushButton {
                font-size: 11px;
                padding: 4px 8px;
                border: 1px solid #ccc;
                border-radius: 3px;
            }
            QPushButton:checked {
                background: #2d6cdf;
                color: white;
                border: 1px solid #2d6cdf;
            }
        """)

        self.urdf_degree_btn = QPushButton("degree")
        self.urdf_degree_btn.setCheckable(True)
        self.urdf_degree_btn.setChecked(True)
        self.urdf_degree_btn.clicked.connect(lambda: self._set_urdf_mode(True))
        self.urdf_degree_btn.setStyleSheet("""
            QPushButton {
                font-size: 11px;
                padding: 4px 8px;
                border: 1px solid #ccc;
                border-radius: 3px;
            }
            QPushButton:checked {
                background: #2d6cdf;
                color: white;
                border: 1px solid #2d6cdf;
            }
        """)

        urdf_mode_layout.addWidget(self.urdf_rad_btn)
        urdf_mode_layout.addWidget(self.urdf_degree_btn)
        urdf_mode_layout.addStretch(1)

        urdf_list_container_layout.addLayout(urdf_mode_layout)

        # 添加所有URDF joint的滑动条（从头到脚排序）
        self.urdf_joint_widgets_list = {}
        self.urdf_use_degree = True

        _HEAD_TO_TOE_ORDER = [
            "head_yaw_joint", "head_pitch_joint", "head_roll_joint",
            "left_arm_pitch_joint", "left_arm_roll_joint", "left_arm_yaw_joint", "left_elbow_ankle_joint",
            "right_arm_pitch_joint", "right_arm_roll_joint", "right_arm_yaw_joint", "right_elbow_ankle_joint",
            "left_hip_pitch_joint", "left_hip_roll_joint", "left_hip_yaw_joint", "left_knee_joint", "left_ankle_joint",
            "right_hip_pitch_joint", "right_hip_roll_joint", "right_hip_yaw_joint", "right_knee_joint", "right_ankle_joint",
        ]

        if self.urdf_robot:
            known = set(_HEAD_TO_TOE_ORDER)
            sorted_names = [n for n in _HEAD_TO_TOE_ORDER if n in self.urdf_robot.joint_limits]
            sorted_names += [n for n in self.urdf_robot.joint_names if n not in known]
            for name in sorted_names:
                lower, upper = self.urdf_robot.joint_limits[name]

                widget = URDFJointWidget(
                    name,
                    lower,
                    upper,
                    self._on_urdf_joint_change,
                    self._on_urdf_joint_sync_changed,
                    use_degree=self.urdf_use_degree,
                    sync_checked=False
                )

                self.urdf_joint_widgets_list[name] = widget
                urdf_list_container_layout.addWidget(widget)

        urdf_list_container_layout.addStretch(1)
        urdf_list_container.setLayout(urdf_list_container_layout)

        # 添加两个container到stacked widget
        self.list_stacked_widget.addWidget(list_container)  # 索引0: Bone列表
        self.list_stacked_widget.addWidget(urdf_list_container)  # 索引1: URDF joint列表

        list_scroll.setWidget(self.list_stacked_widget)

        list_layout.addWidget(list_scroll)
        list_panel.setLayout(list_layout)

        left_splitter = QSplitter(Qt.Vertical)
        left_splitter.addWidget(list_panel)

        # 创建joint panel的stacked widget
        self.joint_panel_stacked_widget = QStackedWidget()
        self.joint_panel_stacked_widget.addWidget(self.joint_panel)  # 索引0: Bone joint panel
        self.joint_panel_stacked_widget.addWidget(self.urdf_joint_panel)  # 索引1: URDF joint panel

        left_splitter.addWidget(self.joint_panel_stacked_widget)
        left_splitter.setStretchFactor(0, 4)  # 上半部分占4/5
        left_splitter.setStretchFactor(1, 1)  # 下半部分占1/5

        # 设置初始大小，使joint panel区域不会太大
        left_splitter.setSizes([640, 160])

        grid_container = QFrame()
        grid_container.setFrameShape(QFrame.StyledPanel)
        grid_layout = QGridLayout()
        grid_layout.setContentsMargins(6, 6, 6, 6)
        grid_layout.setSpacing(8)
        grid_layout.addWidget(self._make_panel("Camera", self.camera_label), 0, 0)
        grid_layout.addWidget(
            self._make_panel(
                "Pose 3D",
                self.humanoid_canvas,
                [self.humanoid_axes_btn, self.humanoid_angles_btn, self.humanoid_names_btn, self.humanoid_save_btn]
            ),
            0,
            1
        )
        grid_layout.addWidget(
            self._make_panel(
                "Pikachu 3D" if self.urdf_viewer is None else "URDF Meshcat",
                self.urdf_viewer if self.urdf_viewer else self.pikachu_canvas,
                [] if self.urdf_viewer else [self.pikachu_axes_btn, self.pikachu_names_btn, self.pikachu_save_btn]
            ),
            1,
            0
        )
        grid_layout.addWidget(self._make_panel("Skeleton", self.skeleton_plot), 1, 1)
        grid_layout.setRowStretch(0, 1)
        grid_layout.setRowStretch(1, 1)
        grid_layout.setColumnStretch(0, 1)
        grid_layout.setColumnStretch(1, 1)
        grid_container.setLayout(grid_layout)

        main_splitter = QSplitter(Qt.Horizontal)
        main_splitter.addWidget(left_splitter)
        main_splitter.addWidget(grid_container)
        main_splitter.setStretchFactor(0, 1)
        main_splitter.setStretchFactor(1, 3)
        main_splitter.setSizes([360, 920])

        main_layout.addWidget(main_splitter)
        self.setLayout(main_layout)

        self.setWindowTitle("Skeleton Studio")

        self.resize(1280, 720)

        self.status_timer = QTimer()
        self.status_timer.setInterval(500)
        self.status_timer.timeout.connect(self.update_status)
        self.status_timer.start()

        self.cap = cv2.VideoCapture(0)
        if not self.cap.isOpened():
            self.camera_label.setText("Camera: open failed")
        self.mp_drawing = mp.solutions.drawing_utils
        self.mp_pose = mp.solutions.pose
        self.camera_timer = QTimer()
        self.camera_timer.setInterval(30)
        self.camera_timer.timeout.connect(self.update_camera)
        self.camera_timer.start()

        self.skeleton_watch_timer = QTimer()
        self.skeleton_watch_timer.setInterval(1000)
        self.skeleton_watch_timer.timeout.connect(self.check_skeleton_file)
        self.skeleton_watch_timer.start()

        self.check_skeleton_file()

        if self.bone_order:
            self.joint_panel.set_bone(self.bone_order[0])

    def update_status(self):
        sync_state = "ON" if self.sync_toggle.isChecked() else "OFF"
        plot_state = "ON" if self.plot_sync_toggle.isChecked() else "OFF"
        if self.client.connected:
            self.status_label.setText(
                f"Status: Connected | Blender: {sync_state} | Plot: {plot_state}"
            )
            if not self._connected_once:
                self._connected_once = True
        else:
            self.status_label.setText(
                f"Status: Waiting | Blender: {sync_state} | Plot: {plot_state}"
            )
            self._connected_once = False

    def update_camera(self):

        if self.cap is None or not self.cap.isOpened():
            return

        ret, frame = self.cap.read()
        if not ret:
            return

        results = self.detector.process(frame)
        landmarks = self.detector.extract_landmarks(results)

        if landmarks:
            self.humanoid_plotter.update(landmarks)
            self.pikachu_plotter.update(landmarks)

        if results.pose_landmarks:
            self.mp_drawing.draw_landmarks(
                frame,
                results.pose_landmarks,
                self.mp_pose.POSE_CONNECTIONS,
                landmark_drawing_spec=self.mp_drawing.DrawingSpec(
                    color=(52, 199, 89),
                    thickness=1,
                    circle_radius=2
                ),
                connection_drawing_spec=self.mp_drawing.DrawingSpec(
                    color=(255, 122, 0),
                    thickness=1
                )
            )

        should_preview = self.plot_sync_toggle.isChecked() or self.sync_toggle.isChecked()
        if should_preview and self.humanoid_plotter.last_angles:
            now = time.time()
            if now - self.last_pose_time >= self.pose_interval:
                self.last_pose_time = now

                # ── Bone（Skeleton）模式同步 ────────────────────────────────
                if self.bone_order:
                    humanoid_data = HumanoidPoseData(angles=self.humanoid_plotter.last_angles)
                    skeleton_data = self.skeleton_converter.convert(humanoid_data, self.bone_order)
                    mapped = skeleton_data.bone_angles
                    if mapped:
                        pose_payload = {}
                        any_changed = False
                        for bone in self.bone_order:
                            if not self.bone_sync.get(bone, True):
                                continue
                            angles = mapped.get(bone)
                            if angles is None:
                                continue
                            limits = self.get_limits(bone)
                            clamped = [
                                max(limits["x"][0], min(limits["x"][1], angles[0])),
                                max(limits["y"][0], min(limits["y"][1], angles[1])),
                                max(limits["z"][0], min(limits["z"][1], angles[2])),
                            ]
                            new_angles = [int(round(a)) for a in clamped]
                            prev_angles = self.bone_angles.get(bone)
                            if prev_angles != new_angles:
                                self.bone_angles[bone] = new_angles
                                any_changed = True
                                item = self.bone_items.get(bone)
                                if item:
                                    item.set_angles(new_angles)

                            last_sent = self._last_sent_angles.get(bone)
                            if last_sent is None or any(
                                abs(new_angles[i] - last_sent[i]) >= self.sync_threshold
                                for i in range(3)
                            ):
                                pose_payload[bone] = new_angles
                                self._last_sent_angles[bone] = new_angles

                        if any_changed:
                            if self.joint_panel.bone:
                                self.joint_panel._set_angles(self.bone_angles[self.joint_panel.bone])
                            self.skeleton_plot.update_angles(self.bone_angles)

                        if self.sync_toggle.isChecked() and pose_payload and self.client.connected:
                            self.client.set_pose(pose_payload)

                # ── URDF 模式同步 ───────────────────────────────────────────
                if self.urdf_robot and self.urdf_joint_widgets_list:
                    synced_joints = [
                        n for n, w in self.urdf_joint_widgets_list.items()
                        if w.sync_checkbox.isChecked()
                    ]
                    if synced_joints:
                        joint_limits = {
                            n: self.urdf_robot.joint_limits[n] for n in synced_joints
                        }
                        humanoid_data = HumanoidPoseData(angles=self.humanoid_plotter.last_angles)
                        urdf_data = self.urdf_converter.convert(
                            humanoid_data, synced_joints, joint_limits
                        )
                        urdf_result = urdf_data.joint_angles
                        for jname, angle_rad in urdf_result.items():
                            widget = self.urdf_joint_widgets_list.get(jname)
                            if widget:
                                widget.set_angle(angle_rad)
                            self.urdf_robot.set_joint(jname, angle_rad)
                        if self.urdf_viewer and urdf_result:
                            self.urdf_viewer.update_robot()

        rgb = cv2.cvtColor(frame, cv2.COLOR_BGR2RGB)
        rgb = cv2.flip(rgb, 1)  # 水平镜像
        h, w, ch = rgb.shape
        qimg = QImage(rgb.data, w, h, ch * w, QImage.Format_RGB888)
        pix = QPixmap.fromImage(qimg)
        self.camera_label.setPixmap(
            pix.scaled(self.camera_label.size(), Qt.KeepAspectRatio, Qt.SmoothTransformation)
        )

    def save_humanoid_skeleton(self):
        bones = self.humanoid_plotter.export_skeleton()
        if not bones:
            print("Humanoid skeleton save skipped: no pose data yet.")
            return
        save_skeleton_to(HUMANOID_SKELETON_PATH, {"bones": bones})
        print(f"Saved humanoid skeleton: {HUMANOID_SKELETON_PATH}")

    def save_pikachu_skeleton(self):
        bones = self.pikachu_plotter.export_skeleton()
        if not bones:
            print("Pikachu skeleton save skipped: no pose data yet.")
            return
        save_skeleton_to(PIKACHU_POSE_SKELETON_PATH, {"bones": bones})
        print(f"Saved pikachu skeleton: {PIKACHU_POSE_SKELETON_PATH}")

    def request_transforms(self):
        if self.client.connected:
            self.client.request_transforms()

    def request_pose(self):
        if self.client.connected:
            self.client.request_pose()

    def on_blender_message(self, msg):

        msg_type = msg.get("type")

        if msg_type == "bones":
            self.bone_tree = msg.get("data", [])
        elif msg_type == "pose":
            self._apply_pose(msg.get("data", {}))
        elif msg_type == "debug":
            print("[Blender]", msg.get("message"))
        elif msg_type == "transforms":
            transforms = msg.get("data", [])
            if not transforms:
                print("Transforms empty or missing.")
            else:
                preview = transforms[:3]
                print(f"Transforms received: {len(transforms)} bones. Preview: {preview}")
            root_names = [b.get("name") for b in transforms if b.get("parent") in (None, "")]
            if root_names:
                allowed = list(dict.fromkeys(self.bone_order + root_names))
                self.skeleton_plot.set_visible_bones(allowed)
            self.skeleton_plot.update_transforms(transforms)
            self.skeleton_plot.update_angles(self.bone_angles)
            if self._pending_save_skeleton and transforms:
                self._pending_save_skeleton = False
                if (
                    self._requested_bones_count is not None and
                    len(transforms) != self._requested_bones_count
                ):
                    if len(transforms) > self._requested_bones_count:
                        print(
                            "Info: received",
                            len(transforms),
                            "bones (includes ancestors of",
                            self._requested_bones_count,
                            ")"
                        )
                    else:
                        print(
                            "Warning: requested",
                            self._requested_bones_count,
                            "bones but received",
                            len(transforms)
                        )
                save_skeleton({"bones": transforms})
                self._skeleton_loaded = True
                self._skeleton_mtime = os.path.getmtime(SKELETON_PATH)
                self._requested_bones_count = None
            elif self._pending_save_skeleton and not transforms:
                print("Save Skeleton requested but transforms are empty.")

    def on_axis_change(self, bone, axis, val):

        angles = self.bone_angles.get(bone, [0, 0, 0])

        if axis == "x":
            angles[0] = val
        elif axis == "y":
            angles[1] = val
        elif axis == "z":
            angles[2] = val

        self.bone_angles[bone] = angles

        item = self.bone_items.get(bone)
        if item:
            item.set_angles(angles)
        self.skeleton_plot.update_angles(self.bone_angles)

        self._last_sent_angles[bone] = list(angles)

    def get_angles(self, bone):

        return self.bone_angles.get(bone, [0, 0, 0])

    def get_limits(self, bone):

        return self.bone_limits.get(bone, {
            "x": (-180, 180),
            "y": (-180, 180),
            "z": (-180, 180)
        })

    def _apply_pose(self, pose):

        if not pose:
            return

        for name, angles in pose.items():
            if name not in self.bone_angles:
                continue
            limits = self.get_limits(name)
            updated = [
                max(limits["x"][0], min(limits["x"][1], int(round(angles[0])))),
                max(limits["y"][0], min(limits["y"][1], int(round(angles[1])))),
                max(limits["z"][0], min(limits["z"][1], int(round(angles[2])))),
            ]
            self.bone_angles[name] = updated
            item = self.bone_items.get(name)
            if item:
                item.set_angles(updated)

        if self.joint_panel.bone and self.joint_panel.bone in pose:
            angles = self.bone_angles[self.joint_panel.bone]
            self.joint_panel._set_angles(angles)

    def save_skeleton_from_blender(self):

        if not self.client.connected:
            print("Save Skeleton failed: not connected to Blender.")
            return
        if not self.bone_order:
            print("Save Skeleton failed: joint_config.yaml has no bones.")
            return
        print("Save Skeleton: requesting transforms from Blender...")
        self._pending_save_skeleton = True
        bones = self.bone_order
        self._requested_bones_count = len(bones)
        self.client.request_transforms(bones=bones)
        self._save_timeout_timer.start()

    def _on_save_timeout(self):
        if self._pending_save_skeleton:
            print("Save Skeleton timeout: no transforms received.")
            self._pending_save_skeleton = False
            self._requested_bones_count = None

    def _apply_skeleton_data(self, data):

        if not data:
            return
        bones = data.get("bones", [])
        if not bones:
            return
        by_name = {}
        for bone in bones:
            if not isinstance(bone, dict):
                continue
            matrix = bone.get("matrix")
            matrix = _parse_matrix_value(matrix)
            local_matrix = _parse_matrix_value(bone.get("local_matrix"))
            name = bone.get("name")
            if not name:
                continue
            by_name[name] = {
                "name": bone.get("name"),
                "parent": bone.get("parent"),
                "head": _parse_vec3(bone.get("head")),
                "tail": _parse_vec3(bone.get("tail")),
                "matrix": matrix,
                "local_matrix": local_matrix
            }
        normalized = list(by_name.values())
        root_names = [n for n, b in by_name.items() if not b.get("parent")]
        allowed = set(self.bone_order)
        allowed.update(root_names)
        self.skeleton_plot.set_visible_bones(sorted(allowed))
        self.skeleton_plot.update_transforms(normalized)
        self.skeleton_plot.update_angles(self.bone_angles)
        self._skeleton_loaded = True

    def check_skeleton_file(self):

        if not os.path.exists(SKELETON_PATH):
            return

        mtime = os.path.getmtime(SKELETON_PATH)
        if self._skeleton_mtime is not None and mtime == self._skeleton_mtime:
            return

        data = load_skeleton()
        if data:
            self._apply_skeleton_data(data)
            self._skeleton_mtime = mtime

    def reset_all(self):

        for name in self.bone_order:
            limits = self.get_limits(name)
            angles = [
                max(limits["x"][0], min(limits["x"][1], 0)),
                max(limits["y"][0], min(limits["y"][1], 0)),
                max(limits["z"][0], min(limits["z"][1], 0))
            ]
            self.bone_angles[name] = angles
            self._last_sent_angles[name] = list(angles)
            item = self.bone_items.get(name)
            if item:
                item.set_angles(angles)

            for axis in ["x", "y", "z"]:
                self.client.set_joint(name, axis, 0)

        if self.joint_panel.bone:
            self.joint_panel._set_angles(self.bone_angles[self.joint_panel.bone])
        self.skeleton_plot.update_angles(self.bone_angles)

        # 重置所有 URDF 关节到 0
        for jname, widget in self.urdf_joint_widgets_list.items():
            widget.set_angle(0.0)
            if self.urdf_robot:
                self.urdf_robot.set_joint(jname, 0.0)
        if self.urdf_viewer and self.urdf_joint_widgets_list:
            self.urdf_viewer.update_robot()

    def closeEvent(self, event):
        if hasattr(self, "camera_timer"):
            self.camera_timer.stop()
        if hasattr(self, "cap") and self.cap is not None:
            self.cap.release()
        if hasattr(self, "humanoid_plotter") and self.humanoid_plotter is not None:
            self.humanoid_plotter.close()
        if hasattr(self, "pikachu_plotter") and self.pikachu_plotter is not None:
            self.pikachu_plotter.close()
        if hasattr(self, "urdf_viewer") and self.urdf_viewer is not None:
            self.urdf_viewer.close()
        super().closeEvent(event)

    def _on_sync_toggled(self, enabled):
        state = "ON" if enabled else "OFF"
        print(f"Auto sync: {state}")
        self._last_sent_angles = {}
        self.last_pose_time = 0.0

    def _on_plot_sync_toggled(self, enabled):
        state = "ON" if enabled else "OFF"
        print(f"Plot preview: {state}")
        self.last_pose_time = 0.0

    def _on_select_all_changed(self, enabled):
        if self._updating_select_all:
            return
        self._updating_select_all = True
        for name, item in self.bone_items.items():
            item.sync_checkbox.blockSignals(True)
            item.sync_checkbox.setChecked(enabled)
            item.sync_checkbox.blockSignals(False)
            self.bone_sync[name] = bool(enabled)
        self._last_sent_angles = {}
        self._updating_select_all = False

    def _on_bone_sync_changed(self, bone, enabled):
        self.bone_sync[bone] = bool(enabled)
        if enabled:
            self.joint_panel.set_bone(bone)
        if self._updating_select_all:
            return
        self._last_sent_angles.pop(bone, None)
        all_checked = all(self.bone_sync.values()) if self.bone_sync else False
        self._updating_select_all = True
        self.select_all_checkbox.setChecked(all_checked)
        self._updating_select_all = False

    def _toggle_urdf_panel(self):
        """切换URDF控制面板的展开/折叠状态"""
        if self.urdf_collapse_btn.isChecked():
            # 展开
            self.urdf_control_panel.setMaximumWidth(300)
            self.urdf_control_panel.setMinimumWidth(200)
            self.urdf_collapse_btn.setText("▼")
        else:
            # 折叠
            self.urdf_control_panel.setMaximumWidth(0)
            self.urdf_control_panel.setMinimumWidth(0)
            self.urdf_collapse_btn.setText("▶")

    def _toggle_urdf_view(self):
        """切换URDF Meshcat和关节控制面板的显示"""
        if self.urdf_toggle_btn.isChecked():
            # 显示关节控制面板
            self.urdf_stacked_widget.setCurrentIndex(1)
        else:
            # 显示meshcat viewer
            self.urdf_stacked_widget.setCurrentIndex(0)

    def _switch_mode(self, mode):
        """切换Bone和URDF模式"""
        if mode == "bone":
            self.bone_mode_btn.setChecked(True)
            self.urdf_mode_btn.setChecked(False)
            self.list_stacked_widget.setCurrentIndex(0)  # 显示Bone列表
            self.joint_panel_stacked_widget.setCurrentIndex(0)  # 显示Bone joint panel
        elif mode == "urdf":
            self.bone_mode_btn.setChecked(False)
            self.urdf_mode_btn.setChecked(True)
            self.list_stacked_widget.setCurrentIndex(1)  # 显示URDF joint列表
            self.joint_panel_stacked_widget.setCurrentIndex(1)  # 显示URDF joint panel

    def _set_urdf_mode(self, use_degree):
        """切换URDF列表的rad/degree模式"""
        self.urdf_use_degree = use_degree

        self.urdf_rad_btn.setChecked(not use_degree)
        self.urdf_degree_btn.setChecked(use_degree)

        # 更新所有URDF joint widget的模式
        for widget in self.urdf_joint_widgets_list.values():
            widget.set_use_degree(use_degree)

    def _on_urdf_joint_change(self, name, angle_rad):
        """处理URDF关节角度变化（接收弧度值）"""
        # 更新URDF模型（传入弧度）
        if self.urdf_robot:
            self.urdf_robot.set_joint(name, angle_rad)

        # 更新URDF查看器（传入弧度）
        if self.urdf_viewer:
            self.urdf_viewer.update_robot()

    def _on_urdf_joint_sync_changed(self, name, checked):
        """处理URDF关节同步勾选框变化"""
        # 更新全选框状态
        all_checked = all(w.sync_checkbox.isChecked() for w in self.urdf_joint_widgets_list.values())
        self.urdf_select_all_checkbox.blockSignals(True)
        self.urdf_select_all_checkbox.setChecked(all_checked)
        self.urdf_select_all_checkbox.blockSignals(False)

    def _on_urdf_select_all_changed(self, checked):
        """处理URDF Select All勾选框变化"""
        for widget in self.urdf_joint_widgets_list.values():
            widget.sync_checkbox.blockSignals(True)
            widget.sync_checkbox.setChecked(checked)
            widget.sync_checkbox.blockSignals(False)


if __name__ == "__main__":

    print("Qt script:", os.path.abspath(__file__))
    print("CONFIG_PATH:", os.path.abspath(CONFIG_PATH))
    print("SKELETON_PATH:", os.path.abspath(SKELETON_PATH))

    # URDF文件路径（可以通过命令行参数或环境变量设置）
    urdf_path = os.path.join(BASE_DIR, "urdf", "robot", "Pikachu_V025", "urdf", "Pikachu_V025_flat_21dof.urdf")

    # 检查命令行参数
    if len(sys.argv) > 1:
        urdf_path = sys.argv[1]

    # 检查环境变量
    if "PIKACHU_URDF_PATH" in os.environ:
        urdf_path = os.environ["PIKACHU_URDF_PATH"]

    print("URDF_PATH:", os.path.abspath(urdf_path))

    # QtWebEngine 要求共享 OpenGL 上下文,必须早于 QApplication 创建时设置,
    # 否则 meshcat 内嵌的 QWebEngineView 无法初始化 WebGL。
    QApplication.setAttribute(Qt.AA_ShareOpenGLContexts, True)

    app = QApplication(sys.argv)

    w = Studio(urdf_path=urdf_path)

    w.show()

    sys.exit(app.exec())
