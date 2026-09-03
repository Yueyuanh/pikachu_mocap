#!/usr/bin/env python3
# -*- coding: utf-8 -*-
"""
mjcf_convert.py — 把 Pikachu 连杆 xacro(或纯 urdf)一键转成适用于 muJoCo 的 MJCF。

这是「MJCF 一键导出」转换逻辑的单一真相源;既可由 pikachu_link_tuner_server.py
进程内 import 调用,也可作为脚本被(如 mocap 环境的)解释器子进程运行:

    子进程(读 stdin 的 JSON,写 stdout 的 JSON):
        echo '{"xacro":"...","basename":"pikachu"}' | python mjcf_convert.py
    进程内:
        import mjcf_convert; mjcf_convert.convert(xacro_text, basename)

参考 EasyMJCF 的思路,但本工具的连杆全是纯 <box>(无 mesh),因此这里跳过
STL 简化 / package:// 替换 / mesh 目录,只留「必要的转换」:

  xacro 展开(宏) → 对缺 <inertial> 的连杆按 box 尺寸+密度补质量/转动惯量 →
  注入 mujoco compiler 选项(balanceinertia 等)→ mujoco.MjModel 解析 →
  mj_saveLastXML 序列化回标准 MJCF 文本。

依赖: xacro、mujoco(通常都在 mocap 环境)。
"""

import json
import os
import sys
import tempfile
import xml.etree.ElementTree as ET

try:
    import xacro
except Exception:
    xacro = None
import mujoco

# 连杆默认密度(kg/m^3)。前端 tuner 的连杆是塑料质感近似,可用 --density 覆盖。
DENSITY = 1500.0


def expand_xacro(text):
    """把 xacro 文本展开成纯 urdf 字符串;纯 urdf 则原样返回。"""
    if xacro is not None:
        try:
            with tempfile.NamedTemporaryFile("w", suffix=".xacro", delete=False) as f:
                f.write(text)
                path = f.name
            try:
                doc = xacro.process_file(path)
            finally:
                os.unlink(path)
            return doc.toxml(), None
        except Exception as e:
            return None, "xacro 展开失败: %s" % e
    return text, None  # 无 xacro 包 → 当纯 urdf 处理


def inject_inertials_and_mujoco(root):
    """给缺 <inertial> 的连杆按 box 尺寸注入质量/转动惯量,再加 mujoco compiler 块。

    - box size 是 URDF 的全尺寸;质量 = 密度×体积;绕质心(半尺寸)转动惯量
      Ixx=m/12*(sy²+sz²) 等,质心取 geometry 的 origin。
    - compiler 的 balanceinertia 会再兜底把过小的惯量拉回 mjMINVAL 以上。
    """
    for link in root.findall("link"):
        if link.find("inertial") is not None:
            continue
        gp = link.find("collision") or link.find("visual")
        if gp is None:
            continue
        geom = gp.find("geometry")
        box = geom.find("box") if geom is not None else None
        if box is None:
            continue  # 非 box(如 mesh/cylinder)跳过,交给 mujoco 自身处理
        sx, sy, sz = [float(v) for v in box.get("size").split()]  # 全尺寸
        origin = gp.find("origin")
        xyz = [0.0, 0.0, 0.0]
        rpy = [0.0, 0.0, 0.0]
        if origin is not None:
            if origin.get("xyz"):
                xyz = [float(v) for v in origin.get("xyz").split()]
            if origin.get("rpy"):
                rpy = [float(v) for v in origin.get("rpy").split()]
        mass = DENSITY * sx * sy * sz
        ixx = mass / 12.0 * (sy * sy + sz * sz)
        iyy = mass / 12.0 * (sx * sx + sz * sz)
        izz = mass / 12.0 * (sx * sx + sy * sy)
        inert = ET.SubElement(link, "inertial")
        ET.SubElement(inert, "origin",
                      {"xyz": "%g %g %g" % tuple(xyz), "rpy": "%g %g %g" % tuple(rpy)})
        ET.SubElement(inert, "mass", {"value": "%g" % mass})
        ET.SubElement(inert, "inertia",
                      {"ixx": "%g" % ixx, "ixy": "0", "ixz": "0",
                       "iyy": "%g" % iyy, "iyz": "0", "izz": "%g" % izz})
    mc = ET.Element("mujoco")
    ET.SubElement(mc, "compiler", {
        "balanceinertia": "true",
        "discardvisual": "false",
        "fusestatic": "false",
    })
    root.insert(0, mc)  # 放在 <robot> 根下、首个子元素位置(EasyMJCF 同套路)


def add_visual_and_collision(mjcf_text):
    """让导出的每个 link 同时具备「视觉层」与「碰撞层」几何，任何可视化软件都能看到本体。

    背景：URDF 的 <visual> 经 MuJoCo 转出后落在 group=1（视觉层，MuJoCo 默认渲染 0/1 两层）；
    但不少 URDF/MJCF 导入器只把 group=1 当"视觉"、把 group=0 当"碰撞(常隐藏)"。因此这里对每个
    <geom> 产出两块：
      · 碰撞层: group=0  contype/conaffinity=1  —— 参与碰撞（且在显碰撞层的工具可见）
      · 视觉层: group=1  contype/conaffinity=0  —— 彩色模型（视觉层工具可见）
    惯量由 <inertial> 提供（MuJoCo 下 inertial 优先、geom density 被忽略、无双算）；
    但**有些 MJCF 解析器不是读 <inertial>，而是按 geom 的 density×体积累加算 body 质量**，
    若全为密度 0 它们会把每个运动 body 算成 0 质量 → 报 "mass ... must be positive" 且物理/视觉都空白。
    因此给碰撞层补一个「匹配正密度」(= 该 body 的 inertial 质量 ÷ 几何体积)，让两类加载器都得正质量；
    视觉层始终 density=0（group=1 在任意 MJCF 实现里都不计质量，se False 双算）。"""
    import copy
    try:
        root = ET.fromstring(mjcf_text)
    except Exception:
        return mjcf_text  # 解析失败就原样返回，不阻断主体转换
    parent = {}

    def walk(e, par):
        for ch in e:
            parent[id(ch)] = par
            walk(ch, ch)

    walk(root, None)

    def geom_volume(g):
        size = g.get("size")
        if size is None:
            return None
        try:
            sx, sy, sz = [float(v) for v in size.split()]
        except Exception:
            return None
        return 8.0 * sx * sy * sz  # MJCF 的 box size 是半尺寸→全尺寸 2x

    def body_mass(body_el):
        inert = body_el.find("inertial") if body_el is not None else None
        if inert is None or inert.get("mass") is None:
            return None
        try:
            return float(inert.get("mass"))
        except Exception:
            return None

    changed = 0
    for g in list(root.iter("geom")):
        if not g.attrib:
            continue
        par = parent.get(id(g))
        if par is None:
            continue
        # 这块作为碰撞层（含碰撞 + 也可显碰撞层）
        g.set("group", "0"); g.set("contype", "1"); g.set("conaffinity", "1")
        # 碰撞层给「匹配正密度」：让按 density 累加的加载器也得到正质量
        mass = body_mass(par)
        vol = geom_volume(g)
        if mass is not None and vol:
            g.set("density", "%.6f" % (mass / vol))
        # 再补一块彩色视觉层（无碰撞、不计质量，density=0）
        twin = copy.deepcopy(g)
        twin.set("group", "1"); twin.set("contype", "0"); twin.set("conaffinity", "0")
        twin.set("density", "0")
        par.append(twin)
        changed += 1
    if not changed:
        return mjcf_text
    return ET.tostring(root, encoding="unicode")


def convert(xacro_text, basename=""):
    """主入口: xacro/urdf 文本 → {ok, mjcf, urdf, nbody, njnt, ngeom, basename}。"""
    urdf, err = expand_xacro(xacro_text)
    if err:
        return {"ok": False, "error": err}
    try:
        root = ET.fromstring(urdf)
        if root.tag != "robot":
            return {"ok": False, "error": "根元素不是 <robot>: %s" % root.tag}
        inject_inertials_and_mujoco(root)
        tmp = tempfile.mkdtemp(prefix="mjcf_")
        urdf_path = os.path.join(tmp, "m.urdf")
        xml_path = os.path.join(tmp, "m.xml")
        ET.ElementTree(root).write(urdf_path, encoding="utf-8")
        model = mujoco.MjModel.from_xml_path(urdf_path)
        mujoco.mj_saveLastXML(xml_path, model)
        mjcf_text = open(xml_path, encoding="utf-8").read()
        mjcf_text = add_visual_and_collision(mjcf_text)  # 每组几何同时给视觉层(group1)与碰撞层(group0)
        return {
            "ok": True,
            "mjcf": mjcf_text,
            "urdf": urdf,
            "basename": (basename or "pikachu_links"),
            "nbody": model.nbody,
            "njnt": model.njnt,
            "ngeom": model.ngeom,
        }
    except Exception as e:
        import traceback
        return {"ok": False,
                "error": "%s: %s" % (type(e).__name__, e),
                "tb": traceback.format_exc()[-1200:]}


def main():
    """脚本入口: 读 stdin 的 {"xacro","basename"},写一行 JSON 到 stdout。"""
    data = json.load(sys.stdin)
    out = convert(data.get("xacro", ""), data.get("basename", ""))
    print(json.dumps(out))
    sys.stdout.flush()


if __name__ == "__main__":
    main()