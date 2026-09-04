#!/usr/bin/env python3
# -*- coding: utf-8 -*-
"""
pikachu_link_tuner_server.py — Pikachu 连杆调校台的后端.

给 urdf/robot/Pikachu_links/pikachu_link_tuner.html 提供一个本地静态文件服务:

    pikachu_link_tuner.html  调校台页面(打开浏览器鉴这个)
    pikachu_sample_links.xacro  页面 boot() 通过 fetch() 加载的同目录数据源

说明(为什么需要这层):
    HTML 是纯前端工具;它在启动时会 `fetch('pikachu_sample_links.xacro')`
    加载同目录 xacro,用 file:// 直接打开时该 fetch 会被浏览器 CORS 拦截,
    只能回退到内置示例并弹提示。用本 server 经 http:// 提供后,fetch 正常,
    可加载真实 xacro。

    同时本 server 提供两组保存接口,让调校结果直接写回所在目录,而不依赖
    浏览器 File System Access API:

        POST /api/save      {"filename","content"|"b64","ext"}     写单个文件
        POST /api/save_dir  {"dir","files":[{filename, content|b64}]} 建文件夹写多文件

    filename/dir 都会做安全清洗:仅允许 [A-Za-z0-9._ -,],拒绝 '/'、'\\'、
    '..' 等路径穿越写法;dir 只能在根目录下新建一个子文件夹。

    另提供 MJCF 一键导出(参考 EasyMJCF 的思路,但这里的连杆都是纯 <box>,
    无 mesh,因此跳过 STL/package 处理,直接在进程内完成):

        POST /api/export_mjcf   {"xacro":"<当前 xacro 文本>"}
            → { ok, basename, nbody, njnt, ngeom,
                urdf(纯几何 urdf 文本), mjcf(适用于 muJoCo 的 xml 文本) }

        流程:进程内 xacro.process_file 展开宏 → 对缺 <inertial> 的连杆按 box
        尺寸+density 计算质量/转动惯量并注入 → 注入 <mujoco><compiler
        balanceinertia=.../></mujoco> 编译选项 → mujoco.MjModel.from_xml_path
        解析 → mj_saveLastXML 序列化回标准 MJCF 文本。(非纯 box 的 xacro 也可
        用 muJoCo 官方 URDF 扩展解析,但本工具面向简单连杆:不生成 mesh 目录,
        不做 STL 压缩。)

        导出导 MJCF 需要服务器进程所在 Python 有 xacro 与 mujoco 两个包
        (本机即 mocap 环境)。缺失时该接口返回 4xx 并提示如何启动。

用法:
    python pikachu_link_tuner_server.py [--port 8080] [--dir PATH] [--host 127.0.0.1]

    --port     监听端口;不指定则自动找空闲端口,并把实际端口打印到 stdout
    --dir      根目录;默认本文件所在目录(即 Pikachu_links)
    --host     绑定地址;默认 127.0.0.1(仅本机,避免暴露到局域网)
    --mocap    若不在 mocap 环境里启动,用它能相容调用 export_mjcf 的 Python;
               导出 MJCF 时优先用该解释器(见 _DST_PY)。默认空 = 用当前进程。

安全:
    - 静态资源仅放行 GET/HEAD，写操作只允许明确的 /api/* POST
    - 只服务本文件同目录(或 --dir 指定目录)下的文件,禁止目录遍历,不列目录
"""

import argparse
import base64
import http.server
import json
import math
import os
import re
import shutil
import socket
import subprocess
import sys
import tempfile
import threading
import time
import xml.etree.ElementTree as ET

HERE = os.path.dirname(os.path.abspath(__file__))
MAX_REQUEST_BYTES = 24 * 1024 * 1024

# 与 HTML/示例文件配套的 MIME 表
MIME = {
    ".html": "text/html; charset=utf-8",
    ".htm": "text/html; charset=utf-8",
    ".xacro": "application/xml; charset=utf-8",
    ".urdf": "application/xml; charset=utf-8",
    ".xml": "application/xml; charset=utf-8",
    ".json": "application/json; charset=utf-8",
    ".text": "text/plain; charset=utf-8",
    ".txt": "text/plain; charset=utf-8",
    ".png": "image/png",
    ".svg": "image/svg+xml",
    ".js": "text/javascript; charset=utf-8",
    ".css": "text/css; charset=utf-8",
}


# ============================================================================
# MJCF 一键导出
# ----------------------------------------------------------------------------
# 参考 EasyMJCF 的思路,但本工具的连杆全是纯 <box>(无 mesh),所以跳过
# STL 简化 / package:// 替换 / mesh 目录,直接把转换逻辑收敛在 mjcf_convert.py
# 单一文件里: xacro → urdf → 补 <inertial>(按 box 尺寸算质量/转动惯量) →
# 注入 mujoco compiler 选项 → mujoco.MjModel 解析 → mj_saveLastXML 序列化回
# 标准 MJCF 文本。
#
# 若本进程能 import xacro+mujoco 就进程内叫 mjcf_convert;否则 fork 到
# 探测到的 mocap 解释器子进程跑同一份 mjcf_convert.py —— 两者单一真相源。
# 依赖都缺失时接口返回 4xx 并给出启动提示。
# ============================================================================

MJCF_CONVERT = os.path.join(HERE, "mjcf_convert.py")


# 显式指定(经 --mocap)的导出用解释器;为空则自动探测 / 用本进程
_DST_PY = ""


def _find_mocap_python():
    """返回能导出 MJCF 的解释器路径;本进程够用则返回 ''(用本进程);
    两者都不可用则返回 None。"""
    use = (_DST_PY or "").strip()
    if not use:
        try:
            import xacro  # noqa: F401
            import mujoco  # noqa: F401
            return ""
        except ImportError:
            use = None
    if not use:
        try:
            import xacro  # noqa: F401
            import mujoco  # noqa: F401
            return ""
        except ImportError:
            pass
    if not use:
        for base in (os.environ.get("CONDA_PREFIX", ""),
                     os.path.expanduser("~/miniconda3"),
                     os.path.expanduser("~/miniconda"),
                     os.path.expanduser("~/.miniconda3")):
            cand = os.path.join(base, "envs", "mocap", "bin", "python")
            if os.path.isfile(cand) and os.access(cand, os.X_OK):
                use = cand
                break
    return use


def _default_mjcf_filename(basename):
    """输出文件命名: <原名>.xml;取不到原名给 pikachu_links.xml。"""
    b = basename.strip() if isinstance(basename, str) and basename.strip() else "pikachu_links"
    return re.sub(r"[^A-Za-z0-9._-]", "_", b) + ".xml"


def convert_xacro_to_mjcf(xacro_text, basename=""):
    """把 xacro(或纯 urdf)文本转换为 {mjcf, urdf, nbody, njnt, ngeom, basename}。

    返回 dict;ok=False 时带 error 说明。
    """
    use = _find_mocap_python()
    if use is None:
        return {"ok": False, "error": "本进程缺少 xacro/mujoco,且未找到可用的 mocap 环境"}
    if use:
        # 子进程:在 mocap 解释器里跑同一份 mjcf_convert.py,避免在系统 python3
        # 下报缺包。stdin/stdout 只属于这个子进程,不影响服务器线程。
        payload = json.dumps({"xacro": xacro_text, "basename": basename}).encode("utf-8")
        try:
            p = subprocess.run([use, MJCF_CONVERT], input=payload,
                               capture_output=True, timeout=180)
        except Exception as e:
            return {"ok": False, "error": "调用 %s 失败: %s" % (use, e)}
        raw = (p.stdout or b"").decode("utf-8", "replace").strip().splitlines()
        if not raw:
            return {"ok": False,
                     "error": "子进程无输出: %s" % (p.stderr or b"").decode("utf-8", "replace")[-600:]}
        try:
            res = json.loads(raw[-1])
        except Exception:
            return {"ok": False, "error": "子进程输出解析失败: %s" % raw[-1][-400:]}
        return res

    # 本进程直接转换(进程内 import 同一份 mjcf_convert)
    try:
        import importlib.util as _ilu
        spec = _ilu.spec_from_file_location("mjcf_convert", MJCF_CONVERT)
        mod = _ilu.module_from_spec(spec)
        spec.loader.exec_module(mod)
        return mod.convert(xacro_text, basename)
    except Exception as e:
        return {"ok": False,
                 "error": "进程内导入 mjcf_convert 失败: %s: %s" % (type(e).__name__, e)}


def _export_mjcf_endpoint(data):
    """POST /api/export_mjcf 的处理;带依赖缺失时的友好提示。"""
    xacro_text = data.get("xacro")
    if not xacro_text or not isinstance(xacro_text, str):
        return 400, {"ok": False, "error": "缺少 xacro 文本"}
    if not MJCF_CONVERT or not os.path.isfile(MJCF_CONVERT):
        return 500, {"ok": False, "error": "找不到 mjcf_convert.py"}
    if _find_mocap_python() is None:
        return 400, {"ok": False,
                     "error": "导出 MJCF 需要 xacro + mujoco,当前解释器缺失,"
                              "且未找到 mocap 环境。请用 "
                              "`conda activate mocap && python pikachu_link_tuner_server.py` "
                              "启动后在页面重试。"}
    res = convert_xacro_to_mjcf(xacro_text, data.get("basename", ""))
    if res.get("ok"):
        res["filename"] = _default_mjcf_filename(res.get("basename", ""))
        return 200, res
    return 400, res


# ============================================================================
# meshcat 3D + npz 动作加载
# ----------------------------------------------------------------------------
# HTML 前端的 3D 显示改成 meshcat:前端把每帧(每条 link 的世界位姿 + 质心 +
# 足端支撑多边形 + 重心落点)POST 到 /api/scene,本进程在后台持有一个
# meshcat.Visualizer(自带网页),把同 URL 嵌进页面 iframe;这样 3D 是真正的
# meshcat 浅蓝主题,前端只需算 FK(与 2D 图纸同一份 JS),后端只负责渲染。
#
# npz 动作仿照 Pikachu_Retarget 的加载(同一份 14 列 NPZ_COLUMNS_TO_URDF),
# 并针对"27dof 已是 T-pose 展开、npz arm_roll 以下垂=0"做角度偏置:
#     θ = v - π/2   (左右对称,几何推导 + 数值核验)
# meshcat + numpy 只在 mocap 环境可用;若当前进程缺少,相关接口返回 4xx 提示。
# ============================================================================

# npz 库目录;Retarget 同款
DEFAULT_NPZ_DIR = os.path.abspath(
    "/home/finnox/Pikachu/PikachuRobot/pikachu_playground/mjlab/src/mjlab/mocap/npz")
_NPZ_DIR = DEFAULT_NPZ_DIR

# npz joint_pos 的 14 列顺序 -> URDF 关节名(与 Pikachu_Retarget 的
# NPZ_COLUMNS_TO_URDF 完全一致;只有腿 10 + 臂 4,无肘/头/耳/尾)
NPZ_COLS_14 = [
    "left_hip_pitch_joint", "left_hip_roll_joint", "left_hip_yaw_joint",
    "left_knee_joint", "left_ankle_joint",
    "right_hip_pitch_joint", "right_hip_roll_joint", "right_hip_yaw_joint",
    "right_knee_joint", "right_ankle_joint",
    "left_arm_pitch_joint", "left_arm_roll_joint",
    "right_arm_pitch_joint", "right_arm_roll_joint",
]

# 27dof xacro 的全部可动关节(按遍历顺序;q前端按 name 对应)
ALL_27_NAMES = [
    "left_hip_pitch_joint", "left_hip_roll_joint", "left_hip_yaw_joint",
    "left_knee_joint", "left_ankle_joint",
    "right_hip_pitch_joint", "right_hip_roll_joint", "right_hip_yaw_joint",
    "right_knee_joint", "right_ankle_joint",
    "left_arm_pitch_joint", "left_arm_roll_joint", "left_arm_yaw_joint", "left_elbow_joint",
    "right_arm_pitch_joint", "right_arm_roll_joint", "right_arm_yaw_joint", "right_elbow_joint",
    "head_pitch_joint", "head_yaw_joint", "head_roll_joint",
    "left_ear_pitch_joint", "left_ear_roll_joint",
    "right_ear_pitch_joint", "right_ear_roll_joint",
    "tail_pitch_joint", "tail_yaw_joint",
]
NAME2IDX27 = {n: i for i, n in enumerate(ALL_27_NAMES)}

_MC = None          # meshcat 单例: {"viewer","url","links"}
_MC_LOCK = threading.Lock()


def _meshcat_available():
    try:
        import meshcat  # noqa: F401
        return True
    except Exception:
        return False


def _ensure_meshcat():
    """惰性启动一个 meshcat.Visualizer(自带网页 / 自有端口),返回单例 dict。"""
    global _MC
    with _MC_LOCK:
        if _MC is None:
            import numpy as np  # noqa: F401   (meshcat 依赖 numpy)
            import meshcat
            vis = meshcat.Visualizer()          # 自动起服务器,不开浏览器
            _MC = {"viewer": vis, "url": vis.url(), "links": None}
        return _MC


def _reset_meshcat(vis):
    for sub in ("pikachu", "fx"):
        try:
            vis[sub].delete()
        except Exception:
            pass


def _to_color_int(c):
    """'#rrggbb' / '#rgb' / int -> int,直立为浅蓝灰。"""
    if isinstance(c, int):
        return c
    s = str(c or "").strip()
    if not s.startswith("#"):
        return 0x9fb4cc
    s = s.lstrip("#")
    if len(s) == 3:
        s = "".join(ch * 2 for ch in s)
    try:
        return int(s, 16)
    except ValueError:
        return 0x9fb4cc


def _map_npz_to_27(row, arm_bias):
    """npz 单帧 14 列 -> {urdf_joint: 弧度};对 arm_roll 应用 T-pose 角度偏置。"""
    out = {}
    mode = arm_bias or "v-90"
    for i, name in enumerate(NPZ_COLS_14):
        if i >= len(row):
            break
        v = float(row[i])
        if name.endswith("arm_roll_joint"):
            if mode == "v-90":       # 推荐:新 27dof 装配位=水平外展,减 π/2 回垂挂
                v = v - math.pi / 2.0
            elif mode == "90-v":
                v = math.pi / 2.0 - v
            # 'direct' 原样
        out[name] = v
    return out


def _quat_wxyz_to_mat(q):
    q = [float(x) for x in q]
    qn = math.sqrt(sum(x * x for x in q))
    if qn < 1e-9:
        return [[1, 0, 0], [0, 1, 0], [0, 0, 1]]
    w, x, y, z = (v / qn for v in q)
    xx, yy, zz = x * x, y * y, z * z
    xy, xz, yz = x * y, x * z, y * z
    wx, wy, wz = w * x, w * y, w * z
    return [
        [1 - 2 * (yy + zz), 2 * (xy - wz), 2 * (xz + wy)],
        [2 * (xy + wz), 1 - 2 * (xx + zz), 2 * (yz - wx)],
        [2 * (xz - wy), 2 * (yz + wx), 1 - 2 * (xx + yy)],
    ]


def _mat_to_euler_xyz_deg(R):
    beta = math.atan2(-R[2][0], math.hypot(R[0][0], R[1][0]))
    if abs(abs(beta) - math.pi / 2.0) > 1e-6:
        alpha = math.atan2(R[2][1], R[2][2])
        gamma = math.atan2(R[1][0], R[0][0])
    else:
        alpha = math.atan2(R[0][1], R[0][2])
        gamma = 0.0
    return [math.degrees(alpha), math.degrees(beta), math.degrees(gamma)]


def _npz_files(data):
    d = (data or {}).get("dir") or _NPZ_DIR
    if not os.path.isdir(d):
        d = _NPZ_DIR
    try:
        fs = sorted(f for f in os.listdir(d) if f.lower().endswith(".npz"))
    except OSError:
        fs = []
    return {"ok": True, "dir": os.path.abspath(d), "files": fs}


def _npz_parse(data):
    """载入 npz,把 joint_pos(T,14) 映射成 (T,27) 弧度,并算根位姿(base)。"""
    import numpy as np
    path = str((data or {}).get("path") or "")
    if not path or not os.path.isfile(path):
        return 400, {"ok": False, "error": "无效 npz 路径"}
    if not path.lower().endswith(".npz"):
        return 400, {"ok": False, "error": "不是 .npz 文件"}
    arm_bias = (data or {}).get("armBias", "v-90")
    try:
        z = np.load(path, allow_pickle=True)
    except Exception as e:
        return 400, {"ok": False, "error": f"npz 读取失败: {e}"}
    if "joint_pos" not in z:
        return 400, {"ok": False, "error": "缺少 joint_pos"}
    jp = np.asarray(z["joint_pos"])
    if jp.ndim != 2:
        return 400, {"ok": False, "error": "joint_pos 需为二维 (T,14)"}
    T = int(jp.shape[0])
    j27 = np.zeros((T, len(ALL_27_NAMES)), dtype=float)
    for t in range(T):
        for name, val in _map_npz_to_27(jp[t], arm_bias).items():
            idx = NAME2IDX27.get(name)
            if idx is not None:
                j27[t, idx] = val

    base_pos = base_rpy = None
    if "body_pos_w" in z and "body_quat_w" in z:
        bp, bq = np.asarray(z["body_pos_w"]), np.asarray(z["body_quat_w"])
        if bp.ndim == 3 and bq.ndim == 3 and int(bp.shape[0]) == T and bp.shape[1] >= 1:
            pos0 = bp[0, 0]
            base_pos = (bp[:, 0] - pos0).astype(float).tolist()
            base_rpy = [_mat_to_euler_xyz_deg(_quat_wxyz_to_mat(q)) for q in bq[:, 0]]
    fps = 30.0
    if "fps" in z:
        try:
            fps = float(np.asarray(z["fps"]).reshape(-1)[0])
        except Exception:
            fps = 30.0

    return 200, {"ok": True, "fps": fps, "n": T,
                 "jointNames": ALL_27_NAMES, "joints": j27.tolist(),
                 "basePos": base_pos, "baseRpy": base_rpy, "armBias": arm_bias}


def _handle_scene(data):
    """POST /api/scene:把前端送来的 link 世界位姿 + 覆盖层推给 meshcat。"""
    import numpy as np
    import meshcat.geometry as mg
    mc = _ensure_meshcat()
    vis = mc["viewer"]
    links = data.get("links") or []
    full = bool(data.get("full"))
    names = [l.get("name") for l in links]
    if full or mc["links"] != names:
        _reset_meshcat(vis)
        root = vis["pikachu"]
        for l in links:
            root[l["name"]].set_object(
                mg.Box([float(x) for x in l["size"]]),
                mg.MeshLambertMaterial(color=_to_color_int(l.get("color"))))
        mc["links"] = names
    for l in links:
        node = vis["pikachu"][l["name"]]
        R = np.array(l["r9"], dtype=float).reshape(3, 3)
        p = np.array(l["pos"], dtype=float)
        T = np.eye(4)
        T[:3, :3] = R
        T[:3, 3] = p
        node.set_transform(T)
    _push_overlays(vis, data)
    return 200, {"ok": True, "url": mc["url"]}


def _push_overlays(vis, data):
    """地平面 + 质心球 + 重心落点/下落线 + 足端支撑多边形(含平衡与否着色)。

    脚底平面 z0 由连杆包围盒自动求出(全模型最低 box 角点),不依赖前端传值;
    重心距脚底的高度 = com.z - z0,用于下落线长度。
    """
    import numpy as np
    import meshcat.geometry as mg
    fx = vis["fx"]

    fx["ground"].set_object(
        mg.Box([1.4, 1.4, 0.003]),
        mg.MeshBasicMaterial(color=0xbfd6f2, opacity=0.14, transparent=True))
    fx["ground"].set_transform(_tr([0, 0, -0.0015]))   # 顶面 ≈ z=0，机器人脚底贴地

    # 前端用完整旋转后的 8 角包围盒精确求得脚底 z，优先直接使用。
    # 旧版的 pos.z-size.z/2 在 link 旋转后会算错，只作兼容回退。
    z0 = (data.get("feet") or {}).get("z")
    if z0 is None:
        for l in (data.get("links") or []):
            try:
                s = l.get("size"); p = l.get("pos")
                if not s or not p:
                    continue
                b = float(p[2]) - float(s[2]) / 2.0
                if z0 is None or b < z0:
                    z0 = b
            except Exception:
                continue
    z0 = float(z0 or 0.0)

    com = data.get("com")
    if com:
        fx["com"].set_object(mg.Sphere(0.010),
                             mg.MeshLambertMaterial(color=0xe0442e))
        fx["com"].set_transform(_tr(com))
        drop_h = max(com[2] - z0, 0.004)
        fx["comDrop"].set_object(
            mg.Box([0.0016, 0.0016, drop_h]),
            mg.MeshBasicMaterial(color=0xe0442e, opacity=0.45, transparent=True))
        fx["comDrop"].set_transform(_tr([com[0], com[1], z0 + drop_h / 2.0]))
        fx["comGround"].set_object(
            mg.Box([0.024, 0.024, 0.0015]),
            mg.MeshBasicMaterial(color=0xe0442e, opacity=0.85, transparent=True))
        fx["comGround"].set_transform(_tr([com[0], com[1], z0 + 0.002]))

    poly = (data.get("feet") or {}).get("poly2d") if data.get("feet") else None
    if poly and len(poly) >= 3:
        z_face = z0   # 脚底平面 = 自动算出的最低 box z
        stable = bool(data.get("balance"))
        color = 0x2ea869 if stable else 0xd64545
        verts = [list(p) + [z_face + 0.004] for p in poly]
        faces = [[0, i + 1, i + 2] for i in range(len(verts) - 2)]
        fx["footPoly"].set_object(
            mg.TriangularMeshGeometry(np.array(verts, dtype=float).reshape(-1, 3),
                                      np.array(faces, dtype=int).reshape(-1, 3)),
            mg.MeshBasicMaterial(color=color, opacity=0.38, transparent=True))
    else:
        try:
            fx["footPoly"].delete()
        except Exception:
            pass


def _tr(p):
    import numpy as np
    import meshcat.transformations as mt
    return mt.translation_matrix([float(x) for x in p])


class LinkTunerHandler(http.server.SimpleHTTPRequestHandler):
    """静态文件处理器:仅 GET,站点根锁定在 directory(由 server 注入)。

    directory 用类属性承载(server.__init__ 时赋值),避免在 handler 的
    __init__ 里依赖尚未初始化的 self.server。基类 translate_path 已自带
    "../" 过滤,杜绝目录遍历。
    """

    server_version = "PikachuLinkTuner/2.0"
    directory = os.path.abspath(HERE)  # 默认根;server 实例化时会被覆盖

    def __init__(self, *args, **kwargs):
        # 基于类属性取目录,而非 self.server(那会在基类 __init__ 后才可用)
        kwargs["directory"] = type(self).directory
        super().__init__(*args, **kwargs)

    # 只放行 GET;HEAD 由基类分派到 do_GET,一并保留(无副作用)
    def do_GET(self):
        if self.path == "/api/health":
            return self._send_json(200, {
                "ok": True,
                "service": self.server_version,
                "meshcat": _meshcat_available(),
                "mjcf": _find_mocap_python() is not None,
                "root": os.path.basename(self.directory),
            })
        if self.path == "/api/meshcat":
            if not _meshcat_available():
                return self._send_json(400, {"ok": False,
                                             "error": "meshcat 不可用,请用 "
                                                      "`conda activate mocap && "
                                                      "python pikachu_link_tuner_server.py` "
                                                      "启动后在页面重试。"})
            try:
                mc = _ensure_meshcat()
            except Exception as e:
                return self._send_json(500, {"ok": False, "error": f"meshcat 启动失败: {e}"})
            return self._send_json(200, {"ok": True, "url": mc["url"]})
        if self.path.startswith("/api/"):
            self._send_json(404, {"ok": False, "error": "no such api"})
            return
        return super().do_GET()

    # 保存接口: /api/save 与 /api/save_dir;导出: /api/export_mjcf
    # 3D/motion: /api/scene /api/npz_files /api/npz_parse
    def do_POST(self):
        if self.path == "/api/save":
            return self._api_save()
        if self.path == "/api/save_dir":
            return self._api_save_dir()
        data, err = self._read_json()
        if err:
            return self._send_json(400, {"ok": False, "error": err})
        if self.path == "/api/export_mjcf":
            code, payload = _export_mjcf_endpoint(data)
        elif self.path == "/api/scene":
            if not _meshcat_available():
                code, payload = 400, {"ok": False,
                                      "error": "meshcat 3D 需要 mocap 环境,"
                                               "请用 `conda activate mocap && "
                                               "python pikachu_link_tuner_server.py` 重启。"}
            else:
                code, payload = _handle_scene(data)
        elif self.path == "/api/npz_files":
            code, payload = 200, _npz_files(data)
        elif self.path == "/api/npz_parse":
            code, payload = _npz_parse(data)
        else:
            code, payload = 404, {"ok": False, "error": "no such api"}
        self._send_json(code, payload)

    # ---------- 保存 API ----------

    def _read_json(self):
        try:
            length = int(self.headers.get("Content-Length", 0))
            if length > MAX_REQUEST_BYTES:
                return None, f"请求过大: {length} bytes (上限 {MAX_REQUEST_BYTES})"
            body = self.rfile.read(length) if length else b""
            return json.loads(body.decode("utf-8")), None
        except Exception as e:
            return None, f"JSON 解析失败: {e}"

    def _send_json(self, code, payload):
        data = json.dumps(payload, ensure_ascii=False).encode("utf-8")
        self.send_response(code)
        self.send_header("Content-Type", "application/json; charset=utf-8")
        self.send_header("Content-Length", str(len(data)))
        self.send_header("Cache-Control", "no-store")
        self.end_headers()
        self.wfile.write(data)

    @staticmethod
    def _safe_name(name, label="文件名"):
        """清洗为安全的单段文件名:仅 [A-Za-z0-9._ -,],拒绝路径分隔与 '..'。"""
        if not name or not isinstance(name, str):
            raise ValueError(f"{label}为空")
        name = name.strip()
        if "/" in name or "\\" in name or ".." in name:
            raise ValueError(f"{label}不能包含路径或 '..'")
        cleaned = re.sub(r"[^A-Za-z0-9._\- ]", "_", name)
        cleaned = cleaned.strip(". ")  # 去首尾点/空格,避免 '..' 或结尾点问题
        if not cleaned or cleaned in (".", ".."):
            raise ValueError(f"{label}不合法")
        return cleaned

    @staticmethod
    def _atomic_write(dest, payload):
        """在目标目录内先写临时文件，再原子替换，避免导出中断留下半个文件。"""
        folder = os.path.dirname(dest)
        fd, tmp = tempfile.mkstemp(prefix=".tuner-", suffix=".tmp", dir=folder)
        try:
            with os.fdopen(fd, "wb") as f:
                f.write(payload)
                f.flush()
                os.fsync(f.fileno())
            os.replace(tmp, dest)
        except Exception:
            try:
                os.unlink(tmp)
            except OSError:
                pass
            raise

    def _unique_output_dir(self, dirname):
        """批量交付绝不覆盖历史结果；目录已存在时加时间戳。"""
        folder = os.path.join(self.directory, dirname)
        if not os.path.exists(folder):
            return dirname, folder
        stamp = time.strftime("%Y%m%d-%H%M%S")
        candidate = f"{dirname}-{stamp}"
        folder = os.path.join(self.directory, candidate)
        serial = 2
        while os.path.exists(folder):
            candidate = f"{dirname}-{stamp}-{serial}"
            folder = os.path.join(self.directory, candidate)
            serial += 1
        return candidate, folder

    @staticmethod
    def _to_bytes(entry):
        """从条目取内容:优先 b64(binary),否则 content(text)。"""
        if "b64" in entry and entry.get("b64"):
            return base64.b64decode(entry["b64"])
        content = entry.get("content", "")
        if isinstance(content, str):
            return content.encode("utf-8")
        if isinstance(content, (bytes, bytearray)):
            return bytes(content)
        raise ValueError("缺少 content 或 b64 内容")

    def _api_save(self):
        data, err = self._read_json()
        if err:
            return self._send_json(400, {"ok": False, "error": err})
        try:
            filename = self._safe_name(data.get("filename"))
            payload = self._to_bytes(data)
        except (ValueError, KeyError, TypeError) as e:
            return self._send_json(400, {"ok": False, "error": str(e)})
        dest = os.path.join(self.directory, filename)
        try:
            self._atomic_write(dest, payload)
        except OSError as e:
            return self._send_json(500, {"ok": False, "error": f"写入失败: {e}"})
        self._send_json(200, {"ok": True, "path": filename, "bytes": len(payload)})

    def _api_save_dir(self):
        data, err = self._read_json()
        if err:
            return self._send_json(400, {"ok": False, "error": err})
        try:
            dirname = self._safe_name(data.get("dir"), "目录名")
        except (ValueError, TypeError) as e:
            return self._send_json(400, {"ok": False, "error": str(e)})
        files = data.get("files") or []
        if not isinstance(files, list) or not files:
            return self._send_json(400, {"ok": False, "error": "files 为空"})
        prepared = []
        try:
            for entry in files:
                if not isinstance(entry, dict):
                    raise ValueError("files 中存在非对象条目")
                filename = self._safe_name(entry.get("filename"))
                prepared.append((filename, self._to_bytes(entry)))
            if len({name for name, _ in prepared}) != len(prepared):
                raise ValueError("files 中存在重名文件")
        except (ValueError, KeyError, TypeError) as e:
            return self._send_json(400, {"ok": False, "error": str(e)})
        dirname, folder = self._unique_output_dir(dirname)
        stage = tempfile.mkdtemp(prefix=".tuner-batch-", dir=self.directory)
        saved = []
        try:
            for filename, payload in prepared:
                self._atomic_write(os.path.join(stage, filename), payload)
                saved.append(filename)
            os.replace(stage, folder)
        except Exception as e:
            shutil.rmtree(stage, ignore_errors=True)
            return self._send_json(500, {"ok": False, "error": f"批量写入失败，未产生交付目录: {e}"})
        self._send_json(200, {"ok": True, "dir": dirname, "files": saved,
                              "count": len(saved)})

    # 不列目录,不给目录列表页
    def list_directory(self, path):
        self.send_error(403, "Directory listing disabled")
        return None

    def guess_type(self, path):
        ext = os.path.splitext(path)[1].lower()
        return MIME.get(ext, "application/octet-stream")


class ThreadingHTTPServerV4(http.server.ThreadingHTTPServer):
    """注入根目录到 handler 取用,并默认 IPv4(避免系统解析到 ::1 的差异)。"""

    address_family = socket.AF_INET

    def __init__(self, address, handler, root):
        self.root = os.path.abspath(root)
        handler.directory = self.root  # 注入根目录给 handler 类属性
        super().__init__(address, handler)


def find_free_port(preferred=None, host="127.0.0.1"):
    """返回一个可用端口:优先 preferred,占用则由系统分配。"""
    port = preferred
    while True:
        with socket.socket(socket.AF_INET, socket.SOCK_STREAM) as s:
            s.setsockopt(socket.SOL_SOCKET, socket.SO_REUSEADDR, 1)
            try:
                s.bind((host, port if port else 0))
                return s.getsockname()[1]
            except OSError:
                if port is None:
                    raise
                port = None  # 首选端口被占 → 交给系统分配


def build_server(root=HERE, host="127.0.0.1", port=None):
    """创建(未启动)的服务器对象;port 为 None 时自动分配空闲端口。"""
    root = os.path.abspath(root)
    if not os.path.isdir(root):
        raise NotADirectoryError(f"目录不存在: {root}")
    tuner = os.path.join(root, "pikachu_link_tuner.html")
    if not os.path.isfile(tuner):
        raise FileNotFoundError(f"未找到 {tuner} —— 请确认 --dir 指向含 html 的目录")

    port = find_free_port(port, host)
    server = ThreadingHTTPServerV4((host, port), LinkTunerHandler, root)
    return server, server.server_address[1], root


def main(argv=None):
    ap = argparse.ArgumentParser(
        prog="pikachu_link_tuner_server",
        description="Pikachu 连杆调校台的本地静态后端。",
    )
    ap.add_argument("--port", type=int, default=None,
                    help="监听端口;缺省自动找空闲端口")
    ap.add_argument("--dir", default=HERE,
                    help=f"根目录;缺省本文件所在目录 ({HERE})")
    ap.add_argument("--host", default="127.0.0.1",
                    help="绑定地址;缺省 127.0.0.1(仅本机)")
    ap.add_argument("--npz-dir", default=DEFAULT_NPZ_DIR,
                    help=f"npz 动作库目录;缺省 {DEFAULT_NPZ_DIR}")
    ap.add_argument("--mocap", metavar="PY", default="",
                    help="能导出 MJCF 的 Python(需含 xacro+mujoco,如 mocap 环境的"
                         "bin/python)。缺省自动探测;缺省也用不了则导出 MJCF 报错")
    args = ap.parse_args(argv)
    globals()["_DST_PY"] = args.mocap  # 供 _find_mocap_python 取用
    globals()["_NPZ_DIR"] = os.path.abspath(args.npz_dir) if args.npz_dir else DEFAULT_NPZ_DIR

    try:
        server, port, root = build_server(args.dir, args.host, args.port)
    except (NotADirectoryError, FileNotFoundError) as e:
        print(f"[错误] {e}", file=sys.stderr)
        return 2

    url = f"http://{args.host}:{port}/pikachu_link_tuner.html"
    print(f"[server] 根目录: {root}", flush=True)
    print(f"[server] 调校台: {url}  (Ctrl+C 退出)", flush=True)
    print(f"[server] npz 动作库: {_NPZ_DIR}  (--npz-dir 可改)", flush=True)
    if _meshcat_available():
        print("[server] meshcat 3D / npz 播放: 可用", flush=True)
    else:
        print("[server] meshcat 3D / npz 播放: 不可用 —— 需 meshcat+numpy(本机 mocap"
              f"环境)。请用 conda activate mocap && python {os.path.basename(__file__)} "
              "重启以获得 3D 显示", flush=True)
    if _find_mocap_python() is not None:
        print("[server] MJCF 一键导出: 可用 (xacro + mujoco)", flush=True)
    else:
        print("[server] MJCF 一键导出: 不可用 —— 需 xacro+mujoco。"
              "请在 mocap 环境启动: conda activate mocap && "
              f"python {os.path.basename(__file__)}", flush=True)
    try:
        server.serve_forever()
    except KeyboardInterrupt:
        print("\n[server] 已退出")
    finally:
        server.server_close()
    return 0


if __name__ == "__main__":
    raise SystemExit(main())
