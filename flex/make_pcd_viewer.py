#!/usr/bin/env python3
# -*- coding: utf-8 -*-
"""
make_pcd_viewer.py — 把 ASCII PCL 点云(.pcd)打包成自包含 three.js 查看器 (单文件 html)

解决"点云加载出来一坨"的渲染适配问题:
  · 前端按包围盒自动居中 + 缩放到高 1.0, 相机始终对准目标;
  · 点 size 设小(固态), 按高度 高度着色(脚蓝→头橙)以免糊成一团;
  · 数据以 base64 的 Float32Array 内嵌, 天然紧凑, 双击 html 即看无需 server。

用法: conda run -n mocap python flex/make_pcd_viewer.py [in.pcd] [--out out.html] [--size 0.008]
"""
import argparse
import base64
import os

import numpy as np

HERE = os.path.dirname(os.path.abspath(__file__))
DEFAULT_IN = os.path.join(HERE, "models", "pikachu_skin.pcd")
DEFAULT_OUT = os.path.join(HERE, "pcd_viewer.html")


def load_pcd_ascii(path):
    """读 ASCII PCD, 返回 Nx3 float32 点。"""
    pts = []
    with open(path) as f:
        for line in f:
            s = line.strip()
            if not s or s.startswith("#") or s[0].isalpha() and not s[0][0].isdigit() and s[0] not in "-.":
                if s and len(s.split()) == 3:
                    pts.append([float(x) for x in s.split()])
                continue
            parts = s.split()
            if len(parts) == 3:
                try:
                    pts.append([float(x) for x in parts])
                except ValueError:
                    pass
    return np.asarray(pts, dtype=np.float32)


def main():
    ap = argparse.ArgumentParser(description="打包 pcd -> 自包含 three.js 查看器")
    ap.add_argument("src", nargs="?", default=DEFAULT_IN)
    ap.add_argument("--out", default=DEFAULT_OUT)
    ap.add_argument("--size", type=float, default=0.008)
    args = ap.parse_args()

    P = load_pcd_ascii(args.src)
    if len(P) == 0:
        raise SystemExit("未解析到点")
    b64 = base64.b64encode(P.tobytes()).decode()
    n = len(P)

    center = P.mean(0)
    span = P.max(0) - P.min(0)
    print("点 %d | 中心 %s | 跨度 %s | base64 %.2f MB"
          % (n, center.round(3), span.round(3), len(b64) / 1048576))

    html = r"""<!doctype html>
<html lang="zh-CN"><head>
<meta charset="utf-8"><meta name="viewport" content="width=device-width,initial-scale=1">
<title>皮卡丘点云查看器 · %(n)d 点</title>
<style>
  *{box-sizing:border-box} html,body{margin:0;height:100%%;background:#0e141d;font:13px/1.5 system-ui,"PingFang SC",sans-serif;color:#dce6f2;overflow:hidden}
  canvas{display:block} #hud{position:fixed;top:10px;left:10px;background:rgba(12,18,26,.8);padding:8px 12px;border-radius:8px;z-index:3;line-height:1.7;font-size:12px;color:#a9b6c9}
  #hud b{color:#ffd76a} .tools{position:fixed;bottom:12px;left:50%%;transform:translateX(-50%%);display:flex;gap:8px;z-index:3}
  .tools button{background:#22304a;color:#dbe4f0;border:1px solid #35445f;padding:7px 14px;border-radius:7px;cursor:pointer;font-size:12.5px}
  .tools button:hover{background:#33446a}
</style></head><body>
<canvas id="gl"></canvas>
<div id="hud">皮卡丘点云 · <b>%(n)d</b> 点<br>高度着色(蓝→红) · 滚轮缩放 · 拖拽旋转</div>
<div class="tools">
  <button onclick="fit('auto')">自动视角</button>
  <button onclick="fit('side')">侧面</button>
  <button onclick="fit('front')">正面</button>
  <button onclick="fit('top')">俯视</button>
  <button onclick="sb(0.004)">更小点</button>
  <button onclick="sb(0.012)">更大点</button>
</div>

<script type="importmap">{"imports":{
  "three":"https://cdn.jsdelivr.net/npm/three@0.160.0/build/three.module.js",
  "three/addons/":"https://cdn.jsdelivr.net/npm/three@0.160.0/examples/jsm/"
}}</script>
<script type="module">
import * as THREE from 'three';
import { OrbitControls } from 'three/addons/controls/OrbitControls.js';

const raw=__B64__;                      // base64 Float32Array
const u8=Uint8Array.from(atob(raw),c=>c.charCodeAt(0));
const P=new Float32Array(u8.buffer);    // little-endian float32
const N=P.length/3;

// 居中 + 缩放到高度≈1, 消除"一坨"(尺度/偏移)
const pos=new Float32Array(P), n=N*3;
const box=new THREE.Box3(new THREE.Vector3(1e9,1e9,1e9),new THREE.Vector3(-1e9,-1e9,-1e9));
for(let i=0;i<n;i+=3) box.expandByPoint(new THREE.Vector3(pos[i],pos[i+1],pos[i+2]));
const c=box.getCenter(new THREE.Vector3());
const spanY=box.max.y-box.min.y, s=1/spanY;
for(let i=0;i<n;i+=3){ pos[i  ]=(pos[i  ]-c.x)*s; pos[i+1]=(pos[i+1]-c.y)*s; pos[i+2]=(pos[i+2]-c.z)*s; }
const ymin=box.min.y, ymax=box.max.y;

// 高度着色: 蓝(脚) → 青 → 黄 → 红(头), 明度随高度增, 避免糊成一团
let yLo= 1e9, yHi=-1e9;
for(let i=0;i<n;i+=3){ if(pos[i+1]<yLo)yLo=pos[i+1]; if(pos[i+1]>yHi)yHi=pos[i+1]; }
const col=new Float32Array(n);
const hsl=new THREE.Color();
for(let i=0;i<n;i+=3){
  const t=(pos[i+1]-yLo)/(yHi-yLo+1e-9);
  hsl.setHSL(0.62-0.62*t,0.9,0.3+0.5*t);   // 蓝(0.62)→红(0)
  col[i  ]=hsl.r; col[i+1]=hsl.g; col[i+2]=hsl.b;
}

const scene=new THREE.Scene(); scene.background=new THREE.Color('#0e141d');
const cam=new THREE.PerspectiveCamera(50,1,0.01,20); cam.position.set(1.6,1.1,1.8);
const renderer=new THREE.WebGLRenderer({canvas:document.getElementById('gl'),antialias:true});
renderer.setPixelRatio(Math.min(devicePixelRatio,2));
scene.add(new THREE.AmbientLight(0xffffff,1.0));
const key=new THREE.DirectionalLight(0xffffff,1.4); key.position.set(2,3,3); scene.add(key);
const grid=new THREE.GridHelper(2.4,16,0x3a4a68,0x232e44); grid.position.y=yLo; scene.add(grid);
const ctrl=new OrbitControls(cam,renderer.domElement); ctrl.target.set(0,(yLo+yHi)/2,0); ctrl.update();

const geo=new THREE.BufferGeometry();
geo.setAttribute('position',new THREE.BufferAttribute(pos,3));
geo.setAttribute('color',new THREE.BufferAttribute(col,3));
const mat=new THREE.PointsMaterial({size:%(size)g,vertexColors:true,sizeAttenuation:true});
const points=new THREE.Points(geo,mat); scene.add(points);

function fit(v){
  if(v==='side'){ cam.position.set(0,0.8,2.4); }
  else if(v==='front'){ cam.position.set(2.4,0.8,0); }
  else if(v==='top'){ cam.position.set(0,2.6,0.02); }
  else { cam.position.set(1.6,1.1,1.8); }
  ctrl.target.set(0,(yLo+yHi)/2,0); ctrl.update();
}
function sb(s){ mat.size=s; }

function resize(){ const w=innerWidth,h=innerHeight; renderer.setSize(w,h,false); cam.aspect=w/h; cam.updateProjectionMatrix(); }
window.addEventListener('resize',resize); resize();
(function loop(){ requestAnimationFrame(loop); ctrl.update(); renderer.render(scene,cam); })();
window.fit=fit; window.sb=sb; window.shadeUpdate=shadeUpdate;
</script>
</body></html>
""" % {"n": n, "size": args.size, "b64": b64}

    # 注入 base64(替换占位符 __B64__): repr 自带单引号, base64 无引号/反斜杠, 直接内插为 JS 字符串字面量
    html = html.replace("__B64__", repr(b64))
    with open(args.out, "w", encoding="utf-8") as f:
        f.write(html)
    print("已写: %s  %.2f MB" % (args.out, os.path.getsize(args.out) / 1048576))


if __name__ == "__main__":
    main()