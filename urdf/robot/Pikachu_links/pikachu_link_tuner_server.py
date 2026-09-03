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
    - 仅放行 GET
    - 只服务本文件同目录(或 --dir 指定目录)下的文件,禁止目录遍历,不列目录
"""

import argparse
import base64
import http.server
import json
import os
import re
import socket
import subprocess
import sys
import tempfile
import xml.etree.ElementTree as ET

HERE = os.path.dirname(os.path.abspath(__file__))

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


class LinkTunerHandler(http.server.SimpleHTTPRequestHandler):
    """静态文件处理器:仅 GET,站点根锁定在 directory(由 server 注入)。

    directory 用类属性承载(server.__init__ 时赋值),避免在 handler 的
    __init__ 里依赖尚未初始化的 self.server。基类 translate_path 已自带
    "../" 过滤,杜绝目录遍历。
    """

    server_version = "PikachuLinkTuner/1.0"
    directory = os.path.abspath(HERE)  # 默认根;server 实例化时会被覆盖

    def __init__(self, *args, **kwargs):
        # 基于类属性取目录,而非 self.server(那会在基类 __init__ 后才可用)
        kwargs["directory"] = type(self).directory
        super().__init__(*args, **kwargs)

    # 只放行 GET;HEAD 由基类分派到 do_GET,一并保留(无副作用)
    def do_GET(self):
        if self.path.startswith("/api/"):
            self._send_json(404, {"ok": False, "error": "no such api"})
            return
        return super().do_GET()

    # 保存接口: /api/save 与 /api/save_dir;导出: /api/export_mjcf
    def do_POST(self):
        if self.path == "/api/save":
            self._api_save()
        elif self.path == "/api/save_dir":
            self._api_save_dir()
        elif self.path == "/api/export_mjcf":
            data, err = self._read_json()
            if err:
                return self._send_json(400, {"ok": False, "error": err})
            code, payload = _export_mjcf_endpoint(data)
            self._send_json(code, payload)
        else:
            self._send_json(404, {"ok": False, "error": "no such api"})

    # ---------- 保存 API ----------

    def _read_json(self):
        try:
            length = int(self.headers.get("Content-Length", 0))
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
        # 先用 basename 杜绝多级路径
        name = os.path.basename(name.replace("\\", "/"))
        cleaned = re.sub(r"[^A-Za-z0-9._\- ]", "_", name)
        cleaned = cleaned.strip(". ")  # 去首尾点/空格,避免 '..' 或结尾点问题
        if not cleaned or cleaned in (".", ".."):
            raise ValueError(f"{label}不合法")
        return cleaned

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
            with open(dest, "wb") as f:
                f.write(payload)
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
        folder = os.path.join(self.directory, dirname)
        try:
            os.makedirs(folder, exist_ok=True)
        except OSError as e:
            return self._send_json(500, {"ok": False, "error": f"无法建目录 {dirname}: {e}"})
        saved = []
        for entry in files:
            if not isinstance(entry, dict):
                continue
            try:
                filename = self._safe_name(entry.get("filename"))
                if os.path.sep in filename or os.path.altsep and os.path.altsep in filename:
                    continue  # 双保险,绝不写路径
                payload = self._to_bytes(entry)
                with open(os.path.join(folder, filename), "wb") as f:
                    f.write(payload)
                saved.append(filename)
            except Exception:
                continue
        if not saved:
            return self._send_json(500, {"ok": False, "error": "没有文件成功写入"})
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
    ap.add_argument("--mocap", metavar="PY", default="",
                    help="能导出 MJCF 的 Python(需含 xacro+mujoco,如 mocap 环境的"
                         "bin/python)。缺省自动探测;缺省也用不了则导出 MJCF 报错")
    args = ap.parse_args(argv)
    globals()["_DST_PY"] = args.mocap  # 供 _find_mocap_python 取用

    try:
        server, port, root = build_server(args.dir, args.host, args.port)
    except (NotADirectoryError, FileNotFoundError) as e:
        print(f"[错误] {e}", file=sys.stderr)
        return 2

    url = f"http://{args.host}:{port}/pikachu_link_tuner.html"
    print(f"[server] 根目录: {root}", flush=True)
    print(f"[server] 调校台: {url}  (Ctrl+C 退出)", flush=True)
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