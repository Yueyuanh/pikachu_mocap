import os
import numpy as np
import trimesh
import trimesh.creation
import meshcat
import meshcat.geometry as mg
import meshcat.transformations as mt

from PySide6.QtWidgets import QWidget, QVBoxLayout
from PySide6.QtWebEngineWidgets import QWebEngineView
from PySide6.QtCore import QUrl


def _rpy_xyz_deg_to_mat4(rx_deg, ry_deg, rz_deg):
    """intrinsic XYZ 欧拉(度) → 4x4 旋转，复合 R = Rz·Ry·Rx（与 Blender rotation_mode='XYZ' 一致）。
    不依赖 meshcat euler_matrix 的欧拉约定，保证 meshcat 与 Blender 三方数学相同。
    """
    a, b, g = np.radians([rx_deg, ry_deg, rz_deg])
    ca, sa = np.cos(a), np.sin(a)
    cb, sb = np.cos(b), np.sin(b)
    cg, sg = np.cos(g), np.sin(g)
    Rz = np.array([[cg, -sg, 0, 0], [sg, cg, 0, 0], [0, 0, 1, 0], [0, 0, 0, 1]])
    Ry = np.array([[cb, 0, sb, 0], [0, 1, 0, 0], [-sb, 0, cb, 0], [0, 0, 0, 1]])
    Rx = np.array([[1, 0, 0, 0], [0, ca, -sa, 0], [0, sa, ca, 0], [0, 0, 0, 1]])
    return Rz @ Ry @ Rx


def _look_at(eye, target, up):
    """从 eye 看向 target 的世界位姿 4x4（meshcat 的 transformations 没有内置 look_at，自建）。
    相机主轴指向 -Z（OpenGL/meshcat 惯例）。
    """
    eye = np.asarray(eye, float)
    target = np.asarray(target, float)
    up = np.asarray(up, float)
    zn = eye - target                       # 相机向 +Z 指向背后（meshcat camera 默认 -Z 朝前）
    zn = zn / (np.linalg.norm(zn) + 1e-12)
    xn = np.cross(up, zn)
    xn = xn / (np.linalg.norm(xn) + 1e-12)
    yn = np.cross(zn, xn)
    M = np.eye(4)
    M[:3, :3] = np.column_stack([xn, yn, zn])
    M[:3, 3] = eye
    return M


class RobotViewer(QWidget):

    def __init__(self, robot_model):

        super().__init__()

        self.robot = robot_model

        # 创建meshcat viewer
        self.viewer = meshcat.Visualizer()

        # 获取meshcat的URL
        self.meshcat_url = self.viewer.url()

        # 创建Qt布局
        layout = QVBoxLayout(self)

        # 创建QWebEngineView来加载meshcat网页
        self.web_view = QWebEngineView()
        self.web_view.setUrl(QUrl(self.meshcat_url))

        # 当网页加载完成后，设置默认相机视角
        self.web_view.loadFinished.connect(self._on_web_view_loaded)

        layout.addWidget(self.web_view)

        # 注意: meshcat.geometry没有Grid属性，所以这里不添加地面网格

        self.mesh_items = {}
        self._scale_matrix = np.eye(4)          # robot 根显示缩放（load_robot 里更新为 1.5×）
        self._base_pos = [0.0, 0.0, 0.0]        # base 位置增量（世界）
        self._base_rpy = [0.0, 0.0, 0.0]        # base 绝对朝向 XYZ 欧拉（度）
        self._ground_offset = 0.16              # meshcat 默认把 robot 抬高，让地面和脚底平齐
        self._follow = False                    # 相机是否跟随机器人
        self._cam_h = 1.5                       # 跟随相机高度（相对地面）
        self._cam_d = 1.5                       # 跟随相机水平距离

        print(f"Meshcat viewer initialized. URL: {self.meshcat_url}")

        self.load_robot()

    def _base_link_world_pos(self):
        """返回 base_link 在世界里的显示位置（= 根节点缩放/抬高/位姿 × base_link FK）。

        base_link 是 URDF 根链接；meshcat 把它的显示值 = robot 根节点变换(1.5×缩放+ground_offset+base)
        再乘 base_link 的 FK。默认下即 (0,0,~0.24)，相机默认应该对准它而不是世界中心 (0,0,16 的地面点)。
        """
        try:
            fk = self.robot.compute_fk()
            bl = next((l for l in fk if l.name == 'base_link'), None)
            Tbl = fk[bl] if bl is not None else np.eye(4)
        except Exception:
            Tbl = np.eye(4)
        # 根节点变换（与 _update_robot_root 一致）：缩放 × 位姿(含 ground_offset) , 单位取到 4x4
        pos = np.asarray(self._base_pos, float) + np.array([0.0, 0.0, self._ground_offset])
        Troot = self._scale_matrix @ mt.translation_matrix(pos) @ _rpy_xyz_deg_to_mat4(*self._base_rpy)
        return (Troot @ Tbl)[:3, 3]

    def _on_web_view_loaded(self, success):
        """当meshcat网页加载完成后，设置默认相机视角：对准 base_link 并抬高相机"""
        if success:
            # 计算 base_link 世界显示位置，抬高相机、注视到 base_link
            t = self._base_link_world_pos()
            eye_off = np.array([0.65, 0.6, 0.9])   # 相机眼位相对 base_link 的偏移（更远、更高）
            ex, ey, ez = t + eye_off
            tx, ty, tz = t
            # 使用JavaScript调整OrbitControls的参数，保持meshcat交互
            js_code = (
                "// 等待一小段时间确保viewer和controls都已初始化\n"
                "setTimeout(function() {\n"
                "    // 尝试获取viewer对象\n"
                "    if (typeof viewer !== 'undefined' && viewer && viewer.controls) {\n"
                "        // 设置更近的初始距离\n"
                "        viewer.controls.minDistance = 1.0;\n"
                "        viewer.controls.maxDistance = 10.0;\n"
                "        // 相机抬高并对准 base_link（而非世界中心的地面点）\n"
                f"        viewer.camera.position.set({ex:.3f}, {ey:.3f}, {ez:.3f});\n"
                f"        viewer.camera.lookAt({tx:.3f}, {ty:.3f}, {tz:.3f});\n"
                "        viewer.controls.update();\n"
                "    }\n"
                "}, 500);\n"
            )
            self.web_view.page().runJavaScript(js_code)

    def load_robot(self):

        for link in self.robot.robot.links:

            if not link.visuals:
                continue

            visual = link.visuals[0]
            geom = visual.geometry

            # mesh: 加载 .stl/.obj；box/cylinder/sphere: 用 trimesh 按几何尺寸现造网格，
            #        统一走 mesh 渲染路径（修复 27dof 纯 box 模型此前只剩尾巴显示的缺陷）。
            mesh = None
            if geom.mesh is not None:
                mesh_path = os.path.abspath(os.path.join(self.robot.base_dir, geom.mesh.filename))
                if not os.path.exists(mesh_path):
                    print("Mesh not found:", mesh_path)
                    continue
                mesh = trimesh.load(mesh_path)
            elif geom.box is not None:
                mesh = trimesh.creation.box(extents=geom.box.size)
            elif geom.cylinder is not None:
                mesh = trimesh.creation.cylinder(radius=geom.cylinder.radius,
                                                 height=geom.cylinder.length, sections=32)
            elif geom.sphere is not None:
                mesh = trimesh.creation.icosphere(subdivisions=2, radius=geom.sphere.radius)

            if mesh is None:
                continue

            # 应用visual的origin变换
            if visual.origin is not None:
                mesh = mesh.apply_transform(visual.origin)

            # 创建meshcat的TriangularMeshGeometry
            mesh_geom = mg.TriangularMeshGeometry(mesh.vertices, mesh.faces)

            # 为每个link创建一个场景节点
            self.viewer["robot"][link.name].set_object(mesh_geom)

            # 存储mesh信息（meshcat不需要像pyqtgraph那样存储mesh_item）
            self.mesh_items[link.name] = {
                'mesh': mesh,
                'visual_origin': visual.origin
            }

        # 初始化时更新所有link的位置和姿态
        self.update_robot()

        # 设置robot节点的缩放，使机器人显示得更大
        # 使用缩放矩阵放大1.5倍
        self._scale_matrix = np.eye(4)
        self._scale_matrix[0, 0] = 1.5
        self._scale_matrix[1, 1] = 1.5
        self._scale_matrix[2, 2] = 1.5
        # 根节点 = 显示缩放 × base 位姿（含 offset），挂在 update_robot 一起刷新
        self._update_robot_root()

    def update_robot(self):

        fk = self.robot.compute_fk()

        for link, T in fk.items():

            if link.name not in self.mesh_items:
                continue

            # meshcat使用4x4变换矩阵
            # T是urdfpy返回的变换矩阵，可以直接使用
            self.viewer["robot"][link.name].set_transform(T)

        # 刷新 robot 根节点（叠加 base 位姿 + 显示缩放）
        self._update_robot_root()

    def set_base_transform(self, pos, rpy_deg, ground_offset=None):
        """叠加 base 位姿到 robot 根节点。pos=位置增量(世界)，rpy=绝对朝向 XYZ 欧拉(度)。"""
        if ground_offset is not None:
            self._ground_offset = float(ground_offset)
        self._base_pos = list(pos) if pos is not None else [0.0, 0.0, 0.0]
        self._base_rpy = list(rpy_deg) if rpy_deg is not None else [0.0, 0.0, 0.0]
        self._update_robot_root()

    def follow_robot(self, on, height=1.5, dist=1.5):
        """开启/关闭相机跟随机器人（npz 播 base 时让人物居中、视角跟随）。"""
        self._follow = bool(on)
        self._cam_h = float(height)
        self._cam_d = float(dist)
        if on:
            self._maybe_follow_camera()

    def _maybe_follow_camera(self):
        """把默认相机移到 base_link 上方，看向 base_link（近似跟随）。"""
        if not self._follow:
            return
        t = self._base_link_world_pos()
        eye = np.array([t[0] + self._cam_d, t[1] - self._cam_d * 0.6, t[2] + self._cam_h])
        target = t
        look = _look_at(eye, target, np.array([0.0, 0.0, 1.0]))
        self.viewer["/Cameras/default/rotated"].set_transform(look)

    def _update_robot_root(self):
        """robot 根节点变换 = 显示缩放 × base 位姿（R 用 Rz·Ry·Rx，与 Blender XYZ 一致），
        并恒定抬高 _ground_offset 使脚底贴地。"""
        pos = np.asarray(self._base_pos, float) + np.array([0.0, 0.0, self._ground_offset])
        T = mt.translation_matrix(pos) @ _rpy_xyz_deg_to_mat4(*self._base_rpy)
        self.viewer["robot"].set_transform(self._scale_matrix @ T)
        # 相机跟随（若开启）
        try:
            self._maybe_follow_camera()
        except Exception:
            pass