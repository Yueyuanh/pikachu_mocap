#!/usr/bin/env python3
# -*- coding: utf-8 -*-
"""
pikachu_link_tuner.py — Pikachu 连杆调校台的一键启动脚本.

负责:
    1. 定位本脚本所在目录(即 Pikachu_links,内含 html / xacro)
    2. 启动后端 pikachu_link_tuner_server.py(自动找空闲端口)
    3. 等端口就绪后,调用系统浏览器打开 pikachu_link_tuner.html

用法:
    python pikachu_link_tuner.py             # 默认方式,自动选端口并开浏览器
    python pikachu_link_tuner.py --port 9001 # 指定端口
    python pikachu_link_tuner.py --no-browser# 只起服务,不开浏览器
    python pikachu_link_tuner.py --dir PATH  # 换目录(必须含 html)

端口占用: 首选 --port,被占则自动换空闲端口并打印实际地址。
停止: Ctrl+C 同时退出后端(子进程一并终止)。
"""

import argparse
import json
import os
import socket
import subprocess
import sys
import time
import urllib.request
import webbrowser

HERE = os.path.dirname(os.path.abspath(__file__))
SERVER = os.path.join(HERE, "pikachu_link_tuner_server.py")


def is_port_open(host, port):
    with socket.socket(socket.AF_INET, socket.SOCK_STREAM) as s:
        s.settimeout(0.3)
        return s.connect_ex((host, port)) == 0


def pick_free_port(host="127.0.0.1"):
    """用临时 socket 找一个空闲端口(SO_REUSEADDR 不生效,确保真正空闲)。"""
    with socket.socket(socket.AF_INET, socket.SOCK_STREAM) as s:
        s.bind((host, 0))
        return s.getsockname()[1]


def wait_until_open(host, port, timeout=8.0):
    """阻塞直到后端在 (host, port) 可连接,或超时返回 False。"""
    deadline = time.time() + timeout
    while time.time() < deadline:
        if is_port_open(host, port):
            return True
        time.sleep(0.12)
    return False


def is_tuner_server(host, port):
    """只复用真正的 Pikachu tuner 服务，避免 --port 碰到其它网页时打开错地址。"""
    try:
        with urllib.request.urlopen(
            f"http://{host}:{port}/api/health", timeout=0.5
        ) as response:
            data = json.loads(response.read().decode("utf-8"))
        return bool(data.get("ok") and str(data.get("service", "")).startswith("PikachuLinkTuner/"))
    except Exception:
        return False


def main(argv=None):
    ap = argparse.ArgumentParser(
        prog="pikachu_link_tuner",
        description="Pikachu 连杆调校台启动脚本(起后端 + 开浏览器)。",
    )
    ap.add_argument("--port", type=int, default=None,
                    help="首选端口;被占自动换空闲端口")
    ap.add_argument("--host", default="127.0.0.1",
                    help="后端绑定地址;缺省 127.0.0.1(仅本机)")
    ap.add_argument("--dir", default=HERE,
                    help=f"根目录;缺省本文件所在目录 ({HERE})")
    ap.add_argument("--no-browser", action="store_true",
                    help="不自动打开浏览器")
    args = ap.parse_args(argv)

    if not os.path.isfile(SERVER):
        print(f"[错误] 未找到后端脚本: {SERVER}", file=sys.stderr)
        return 2

    # 固定端口:指定则用之;否则本脚本探测一个空闲端口传后端,URL 立即可知
    port = args.port or pick_free_port(args.host)

    # 显式端口已占用时，只复用带正确健康接口的 tuner；其它服务不能误当后端。
    occupied = args.port is not None and is_port_open(args.host, port)
    already_up = occupied and is_tuner_server(args.host, port)
    if occupied and not already_up:
        old_port = port
        port = pick_free_port(args.host)
        print(f"[info] 端口 {old_port} 被非 tuner 服务占用，改用 {port}。")

    cmd = [sys.executable, SERVER, "--dir", args.dir, "--host", args.host,
           "--port", str(port)]

    if already_up:
        print("[info] 检测到后端已在运行,直接复用。")
        proc = None
    else:
        print(f"[启动] 启动后端 (端口 {port}) …")
        # 让后端进程追随本进程:本进程退出(或被 Ctrl+C)即随之后台结束
        # 继承终端输出：既保留能力/错误日志，也避免 PIPE 长时间无人读取导致后端阻塞。
        proc = subprocess.Popen(cmd)
        if proc.poll() is not None:
            print("[后端启动失败]", file=sys.stderr)
            return proc.returncode or 1
        if not wait_until_open(args.host, port):
            print("后端端口不可达。", file=sys.stderr)
            proc.terminate()
            return 1
        print("[启动] 后端已就绪。")

    url = f"http://{args.host}:{port}/pikachu_link_tuner.html"
    print()
    print("  调校台 → " + url)
    print("  停止     → Ctrl+C")
    print()

    if not args.no_browser:
        webbrowser.open(url)

    try:
        if proc:
            proc.wait()
    except KeyboardInterrupt:
        print("\n[退出] 关闭后端 …")
        if proc:
            proc.terminate()
            try:
                proc.wait(timeout=3)
            except subprocess.TimeoutExpired:
                proc.kill()
    return 0


if __name__ == "__main__":
    raise SystemExit(main())
