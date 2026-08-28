#!/usr/bin/env python3
# -*- coding: utf-8 -*-
"""
make_selfrig_viewer.py — 由 selfrig_data.json 生成自包含 three.js 真蒙皮驱动查看器 selfrig_viewer.html

点云每一顶点带 top-4 多骨蒙皮权重(关节处平滑混合)。前端:
  · FK 沿骨链累积旋转(滑条每骨绕 X);
  · LBS = Σ_b w_b · ( R_b·(p_rest − rest_b) + world_b )   ← 真正加权蒙皮, 不是「最近单骨」
  · 每骨一个基准色相, 顶点色 = 按权重混合该骨色 → 一眼看出蒙皮跟着骨架走。
双击 html 即看, 数据内嵌; headless 可抓 title('movedY') 程序化验证。

用法: conda run -n mocap python flex/make_selfrig_viewer.py
"""
import json
import os
import sys

HERE = os.path.dirname(os.path.abspath(__file__))
SRC = os.path.join(HERE, "selfrig_data.json")
OUT = os.path.join(HERE, "selfrig_viewer.html")


def main():
    d = json.load(open(SRC, encoding="utf-8"))
    bones, pts, weights = d["bones"], d["pts"], d["weights"]
    nb, N = len(bones), len(pts)
    # 定长打包每点 top-k: boneIdx,w 交错
    flat = []
    for w in weights:
        base = len(w)
        for bi, ww in w:
            flat.append(bi); flat.append(ww)
        if base < 4:
            flat.extend([0, 0.0] * (4 - base))
    js_flat = json.dumps(flat)
    js_bones = json.dumps(bones, ensure_ascii=False)
    js_pts = json.dumps(pts)

    html = r"""<!doctype html>
<html lang="zh-CN"><head><meta charset="utf-8">
<meta name="viewport" content="width=device-width,initial-scale=1">
<title>皮卡丘点云 · 真蒙皮 LBS 驱动</title>
<style>
 *{box-sizing:border-box} html,body{margin:0;height:100%;font:14px/1.5 system-ui,"PingFang SC",sans-serif;color:#e8ecf4;background:#0f141d}
 .wrap{display:flex;height:100%} #view{flex:1;min-width:0;position:relative}
 #hud{position:absolute;top:10px;left:10px;background:rgba(15,20,29,.8);padding:6px 12px;border-radius:8px;font-size:12px;color:#9aa6b8;z-index:2;max-width:58%;line-height:1.6}
 .tools{position:absolute;left:10px;bottom:10px;display:flex;gap:8px;z-index:2}
 .tools button{background:#22304a;color:#dbe4f0;border:1px solid #35445f;padding:6px 12px;border-radius:7px;cursor:pointer;font-size:12px}
 .panel{width:292px;flex:none;background:#151b26;border-left:1px solid #232c3b;overflow:auto;padding:12px}
 .panel h1{font-size:15px;margin:2px 0 4px} .tip{font-size:12px;color:#8b96a8;margin:0 0 10px;line-height:1.5}
 .grp{margin-bottom:8px} .grp>.t{font-weight:600;font-size:12px;color:#7fd0ff;margin:6px 0 2px}
 .jnt{display:flex;align-items:center;gap:8px;margin:3px 0}
 .jnt label{width:112px;font-size:12px;color:#c7d0dd;flex:none;overflow:hidden;white-space:nowrap;text-overflow:ellipsis}
 .jnt input[type=range]{flex:1} .jnt .deg{width:38px;text-align:right;font-size:11px;color:#8b96a8;font-variant-numeric:tabular-nums}
 #stat{font-size:11px;color:#7ecd8c;white-space:pre-wrap;margin-top:4px}
 .legend{margin-top:10px;border-top:1px solid #232c3b;padding-top:8px}
 .legend .row{display:flex;align-items:center;gap:6px;font-size:11px;color:#9aa6b8;margin:2px 0}
 .legend .sw{width:12px;height:12px;border-radius:3px}
</style></head><body>
<div class="wrap">
 <div id="view"><canvas id="gl"></canvas><div id="hud">加载…</div>
  <div class="tools"><button onclick="toggleBone()">骨骼线:开</button>
   <button onclick="resetPose()">复位</button><button onclick="autoPose()">摆动</button></div>
 </div>
 <div class="panel"><h1>皮卡丘点云 · 真蒙皮(LBS)</h1>
  <p class="tip">每顶点绑定 <b>top-4 骨骼蒙皮权重</b>(从 FBX 顶点组插值)。拖滑条转骨 → FK 累积 → 每点按权重混合各骨变换。关节处颜色/位移平滑过渡 = 真蒙皮, 非「最近骨」。</p>
  <div id="stat"></div><div id="sliders"></div></div>
</div>
<script type="importmap">{"imports":{
 "three":"https://cdn.jsdelivr.net/npm/three@0.160.0/build/three.module.js",
 "three/addons/":"https://cdn.jsdelivr.net/npm/three@0.160.0/examples/jsm/"}}</script>
<script type="module">
window.addEventListener('error',e=>{document.title='ERR '+String(e.message||'').slice(0,60);
  const st=document.getElementById('stat'); if(st)st.innerHTML='<pre>'+String((e.error&&e.error.stack)||e.message)+'</pre>';});
import * as THREE from 'three';
import { OrbitControls } from 'three/addons/controls/OrbitControls.js';

const BONES=__BONES__, PTS=__PTS__, FLAT=__FLAT__;
const N=PTS.length, nb=BONES.length, W=4;
// bones 数组序即 id
const rest=[]; BONES.forEach((b,i)=>rest[i]=new THREE.Vector3(b.pos[0],b.pos[1],b.pos[2]));
const childRel=[]; BONES.forEach((b,i)=>childRel[i]= b.parent>=0? rest[i].clone().sub(rest[b.parent]) : new THREE.Vector3());
// BFS 拓扑序
// 拓扑序: 从"所有根"(parent<0 可多个, 如 base_link + hip_L + hip_R) 一起 BFS, 再兜底补漏
const order=[]; (function(){const q=BONES.map((b,i)=>b.parent<0?i:-1).filter(i=>i>=0);
  while(q.length){const id=q.shift(); if(order.includes(id))continue; order.push(id);
    BONES.forEach((c,j)=>{ if(c.parent===id && !order.includes(j)) q.push(j); });}
  BONES.forEach((_,i)=>{ if(!order.includes(i)) order.push(i); });})();
// 每点权重解包 (flat: boneIdx0,w0, boneIdx1,w1, ..., 4组, 空组=(-1,0))
const bw=[]; for(let i=0;i<N;i++){ const row=[]; for(let k=0;k<W;k++){ const bi=FLAT[(i*W+k)*2], w=FLAT[(i*W+k)*2+1]; row.push([bi, w]); } bw.push(row); }

const scene=new THREE.Scene(); scene.background=new THREE.Color('#141924');
const cam=new THREE.PerspectiveCamera(50,1,.01,100); cam.position.set(1.8,1.4,2.2);
const renderer=new THREE.WebGLRenderer({canvas:document.getElementById('gl'),antialias:true});
renderer.outputColorSpace=THREE.SRGBColorSpace;
scene.add(new THREE.AmbientLight(0xffffff,1.0));
scene.add(new THREE.DirectionalLight(0xffffff,1.6));
const grid=new THREE.GridHelper(4,20,0x33405a,0x232b3d); grid.position.y=0; scene.add(grid);
const ctrl=new OrbitControls(cam,renderer.domElement); ctrl.target.set(0,1.1,0); ctrl.update();

// 几何
const geo=new THREE.BufferGeometry();
const posAttr=new Float32Array(N*3); geo.setAttribute('position',new THREE.BufferAttribute(posAttr,3));
const col=new Float32Array(N*3); geo.setAttribute('color',new THREE.BufferAttribute(col,3));
// 每骨基准色相(离散且区分)
const hues=[0.02,0.10,0.16,0.20,0.28,0.34,0.40,0.46,0.52,0.58,0.64,0.70,0.76,0.82];
function tint(bi){ const c=new THREE.Color(); c.setHSL(hues[bi%hues.length],0.95,0.52); return c; }
const tints=BONES.map((_,i)=>tint(i));
const points=new THREE.Points(geo,new THREE.PointsMaterial({size:0.012,vertexColors:true,sizeAttenuation:true}));
scene.add(points);
const boneSegs=new THREE.LineSegments(new THREE.BufferGeometry(),new THREE.LineBasicMaterial({color:0x6fd0ff}));
scene.add(boneSegs);

const rotDeg=new Float32Array(nb), DEG=Math.PI/180;
const boneR=[], boneWorld=[];
BONES.forEach((_,i)=>{boneR.push(null); boneWorld.push(new THREE.Vector3());});
const _v=new THREE.Vector3(), _a=new THREE.Vector3();

function fk(){
  for(const id of order){ const b=BONES[id];
    const Rl=new THREE.Matrix4().makeRotationX(rotDeg[id]*DEG);
    boneR[id]= b.parent>=0? boneR[b.parent].clone().multiply(Rl) : Rl;
    if(b.parent<0){ boneWorld[id].copy(rest[id]); }
    else{ _v.copy(childRel[id]).applyMatrix4(boneR[b.parent]).add(boneWorld[b.parent]); boneWorld[id].copy(_v); }
  }
  // 顶点色: 每点按权重混合骨色
  for(let i=0;i<N;i++){
    let r=0,g=0,b=0;
    for(let k=0;k<W;k++){ const [bi,wdi]=bw[i][k]; if(bi<0)continue; const t=tints[bi]; r+=wdi*t.r; g+=wdi*t.g; b+=wdi*t.b; }
    col[i*3]=r; col[i*3+1]=g; col[i*3+2]=b;
  }
  col.needsUpdate=true;
  // LBS 位置
  for(let i=0;i<N;i++){
    _a.set(PTS[i][0],PTS[i][1],PTS[i][2]);
    const out=new THREE.Vector3();
    for(let k=0;k<W;k++){ const [bi,wdi]=bw[i][k]; if(bi<0||wdi<1e-6)continue;
      _v.copy(_a).sub(rest[bi]).applyMatrix4(boneR[bi]).add(boneWorld[bi]).multiplyScalar(wdi);
      out.add(_v); }
    posAttr[i*3]=out.x; posAttr[i*3+1]=out.y; posAttr[i*3+2]=out.z;
  }
  posAttr.needsUpdate=true;
  const seg=[];
  for(let i=0;i<nb;i++){ const b=BONES[i]; if(b.parent>=0){ const p=boneWorld[b.parent], c=boneWorld[i];
    seg.push(p.x,p.y,p.z, c.x,c.y,c.z); } }
  boneSegs.geometry.setAttribute('position',new THREE.Float32BufferAttribute(seg,3));
}
function movedY(){ let s=0; for(let i=0;i<N;i++) s+=Math.abs(posAttr[i*3+1]-PTS[i][1]); return s; }
const hud=document.getElementById('hud'), stat=document.getElementById('stat');
function refresh(){ fk(); document.title='pcd-skin-movedY='+movedY().toFixed(2);
  hud.textContent='点云 '+N+' · 14 骨真蒙皮 · 拖滑条转骨 → 权重混合 LBS 随动'; }

// 滑条面板
const holder=document.getElementById('sliders');
const groups={'躯干/头':['base_link','head'],'腿_L':['hip_L','hip_pitch_L','hip_knee_L','hip_ankle_L'],
 '腿_R':['hip_R','hip_pitch_R','hip_knee_R','hip_ankle_R'],'臂_L':['arm_L','arm_pitch_L'],
 '臂_R':['arm_R','arm_pitch_R']};
for(const gname in groups){ const sec=document.createElement('div'); sec.className='grp';
  sec.innerHTML='<div class="t">'+gname+'</div>';
  for(const bn of groups[gname]){ const id=BONES.findIndex(x=>x.name===bn);
    const row=document.createElement('div'); row.className='jnt';
    row.innerHTML='<label>'+bn+'</label><input type="range" min="-70" max="70" step="1" value="0"><span class="deg">0°</span>';
    const rng=row.querySelector('input'), dg=row.querySelector('.deg');
    rng.oninput=()=>{ rotDeg[id]=+rng.value; dg.textContent=rng.value+'°'; refresh(); };
    sec.appendChild(row);} holder.appendChild(sec);}
// 图例
const leg=document.createElement('div'); leg.className='legend';
leg.innerHTML='<div class="t" style="color:#7fd0ff">骨骼权重配色</div>';
BONES.forEach((b,i)=>{ const c=tints[i]; leg.innerHTML+='<div class="row"><span class="sw" style="background:rgb('+(c.r*255|0)+','+(c.g*255|0)+','+(c.b*255|0)+')"></span>'+(b.name)+(b.parent<0?' (根)':'')+'</div>';});
(stat.parentElement||holder).appendChild(leg);

window.toggleBone=()=>{ boneSegs.visible=!boneSegs.visible; document.querySelector('.tools button').textContent='骨骼线:'+(boneSegs.visible?'开':'关'); };
window.resetPose=()=>{ rotDeg.fill(0); document.querySelectorAll('#sliders input').forEach(i=>{i.value=0; i.nextElementSibling.textContent='0°';}); refresh(); };
let _auto=null;
window.autoPose=()=>{ if(_auto){clearInterval(_auto);_auto=null;resetPose();return;} const raws=[...document.querySelectorAll('#sliders .jnt')]; let t=0;
  _auto=setInterval(()=>{ t+=0.05;
    for(const r of raws){ const nm=r.querySelector('label').textContent; const id=BONES.findIndex(x=>x.name===nm);
      const v=Math.round(Math.sin(t*2+id)*45); r.querySelector('input').value=v; r.querySelector('.deg').textContent=v+'°'; rotDeg[id]=v; }
    refresh(); if(t>6){clearInterval(_auto);_auto=null;} },40);
  document.querySelector('.tools button').textContent='骨骼线:开'; };

// 测试姿态 ?pose=1
if(new URLSearchParams(location.search).has('pose')){
  const setR=(nm,v)=>{ const id=BONES.findIndex(x=>x.name===nm); if(id>=0)rotDeg[id]=v; };
  setR('hip_knee_L',48); setR('hip_pitch_L',25); setR('arm_pitch_R',38); setR('arm_R',18); setR('head',12);
  document.querySelectorAll('#sliders input').forEach(i=>{ const nm=i.parentElement.querySelector('label').textContent;
    const id=BONES.findIndex(x=>x.name===nm); i.value=rotDeg[id]||0; i.nextElementSibling.textContent=(rotDeg[id]||0)+'°'; });
  refresh();
}
function resize(){ renderer.setPixelRatio(Math.min(devicePixelRatio,2));
  const w=document.getElementById('view').clientWidth, h=document.getElementById('view').clientHeight;
  renderer.setSize(w,h,false); cam.aspect=w/h; cam.updateProjectionMatrix(); }
window.addEventListener('resize',resize);
function loop(){ requestAnimationFrame(loop); ctrl.update(); renderer.render(scene,cam); }
resize(); refresh(); loop();
</script></body></html>
"""
    html = html.replace("__BONES__", js_bones).replace("__PTS__", js_pts).replace("__FLAT__", js_flat)
    with open(OUT, "w", encoding="utf-8") as f:
        f.write(html)
    print("已写", OUT, " %.2f MB | 点 %d 骨 %d 权重对(定长) %d"
          % (os.path.getsize(OUT) / 1048576, N, nb, len(flat)))


if __name__ == "__main__":
    main()