#!/usr/bin/env python3
# -*- coding: utf-8 -*-
"""
make_selfrig_viewer.py — 由 selfrig_data.json 生成自包含 three.js 真蒙皮驱动查看器 selfrig_viewer.html

点云每一顶点带 top-4 多骨蒙皮权重(关节处平滑混合)。前端:
  · 每个可动 DOF 一根滑条(参考 Pikachu_Retarget 的 retarget_map_self_rig_v2.yaml,
    每关节 = (骨, 轴, sign, 限位[lo,hi] 度)), 滑条钳位到限位;
  · FK: 每骨把其全部可动轴旋转合成一个局部旋转矩阵(localR), 沿骨链累积成世界旋转 boneR;
  · LBS = Σ_b w_b · ( R_b·(p_rest − rest_b) + world_b )    ← 真正加权蒙皮, 非「最近单骨」
  · 每骨一个基准色相, 顶点色 = 按权重混合该骨色。
双击 html 即看, 数据内嵌; headless 可抓 title('movedY') 程序化验证。

多轴自由度(19-dof, 与 retarget 一致):
  head        x(pitch) y(yaw) z(roll)
  arm_pitch_L x(pitch) y(yaw) z(roll)
  arm_pitch_R x(pitch) y(yaw) z(roll)
  hip_pitch_L x(pitch) y(yaw) z(roll)
  hip_pitch_R x(pitch) y(yaw) z(roll)
  hip_knee_L  x(pitch)      / hip_knee_R  x
  hip_ankle_L x(pitch)      / hip_ankle_R x

用法: conda run -n mocap python flex/selfrig/make_selfrig_viewer.py
"""
import json
import os

HERE = os.path.dirname(os.path.abspath(__file__))
SRC = os.path.join(HERE, "selfrig_data.json")
OUT = os.path.join(HERE, "selfrig_viewer.html")

# ── 关节自由度列表(参考 retarget_map_self_rig_v2.yaml)──────────────────
# (关节名, 骨名, 轴, sign, 限位lo, 限位hi) 度 —— 与 Pikachu_Retarget 的
# sink_axis_angle = joint_angle*sign + bias 语义一致(bias=0)。
DOFS = [
    ("head_pitch", "head", "x", -1, -60, 60),
    ("head_yaw",   "head", "y", +1, -60, 60),
    ("head_roll",  "head", "z", -1, -60, 60),
    # 左臂: arm_pitch_L 承载 肩 pitch/roll/yaw(与 retarget 一致)
    ("left_arm_pitch", "arm_pitch_L", "x", -1, -60, 180),
    ("left_arm_yaw",   "arm_pitch_L", "y", +1, -90, 90),
    ("left_arm_roll",  "arm_pitch_L", "z", +1, 0, 90),
    # 右臂
    ("right_arm_pitch", "arm_pitch_R", "x", -1, -60, 180),
    ("right_arm_yaw",   "arm_pitch_R", "y", +1, -90, 90),
    ("right_arm_roll",  "arm_pitch_R", "z", +1, 0, 90),
    # 左腿: hip_pitch_L 承载 大腿 pitch/roll/yaw, 另 knee/ankle 单轴
    ("left_hip_pitch", "hip_pitch_L", "x", +1, -140, 0),
    ("left_hip_yaw",   "hip_pitch_L", "y", +1, -5, 15),
    ("left_hip_roll",  "hip_pitch_L", "z", -1, -5, 15),
    ("left_knee",      "hip_knee_L",  "x", -1, 0, 90),
    ("left_ankle",     "hip_ankle_L", "x", -1, -60, 30),
    # 右腿
    ("right_hip_pitch", "hip_pitch_R", "x", -1, -140, 0),
    ("right_hip_yaw",   "hip_pitch_R", "y", +1, -15, 5),
    ("right_hip_roll",  "hip_pitch_R", "z", -1, -5, 15),
    ("right_knee",      "hip_knee_R",  "x", +1, 0, 90),
    ("right_ankle",     "hip_ankle_R", "x", +1, -60, 30),
]


def main():
    d = json.load(open(SRC, encoding="utf-8"))
    bones, pts, weights = d["bones"], d["pts"], d["weights"]
    nb, N = len(bones), len(pts)
    # 定长打包每点 top-k: boneIdx,w 交错
    flat = []
    for w in weights:
        base = len(w)
        for bi, ww in w:
            flat.append(bi)
            flat.append(ww)
        if base < 4:
            flat.extend([0, 0.0] * (4 - base))
    js_flat = json.dumps(flat)
    js_bones = json.dumps(bones, ensure_ascii=False)
    js_pts = json.dumps(pts)
    # 把 DOF 的骨名解析为骨骼索引
    bone_id = {b["name"]: i for i, b in enumerate(bones)}
    dofs = []
    for jn, bn, ax, sgn, lo, hi in DOFS:
        bid = bone_id.get(bn)
        if bid is None:
            print("!! 跳过未知骨 %s(%s)" % (bn, jn))
            continue
        dofs.append([jn, bid, ax, sgn, lo, hi])
    js_dofs = json.dumps(dofs, ensure_ascii=False)

    html = r"""<!doctype html>
<html lang="zh-CN"><head><meta charset="utf-8">
<meta name="viewport" content="width=device-width,initial-scale=1">
<title>皮卡丘点云 · 真蒙皮 19-DOF LBS 驱动</title>
<style>
 *{box-sizing:border-box} html,body{margin:0;height:100%;font:14px/1.5 system-ui,"PingFang SC",sans-serif;color:#e8ecf4;background:#0f141d}
 .wrap{display:flex;height:100%} #view{flex:1;min-width:0;position:relative}
 #hud{position:absolute;top:10px;left:10px;background:rgba(15,20,29,.85);padding:6px 12px;border-radius:8px;font-size:12px;color:#9aa6b8;z-index:2;max-width:58%;line-height:1.6}
 .tools{position:absolute;left:10px;bottom:10px;display:flex;gap:8px;z-index:2}
 .tools button{background:#22304a;color:#dbe4f0;border:1px solid #35445f;padding:6px 12px;border-radius:7px;cursor:pointer;font-size:12px}
 .panel{width:320px;flex:none;background:#151b26;border-left:1px solid #232c3b;overflow:auto;padding:12px}
 .panel h1{font-size:15px;margin:2px 0 4px} .tip{font-size:12px;color:#8b96a8;margin:0 0 10px;line-height:1.5}
 .grp{margin-bottom:8px} .grp>.t{font-weight:600;font-size:12px;color:#7fd0ff;margin:6px 0 2px}
 .jnt{display:flex;align-items:center;gap:8px;margin:3px 0}
 .jnt label{width:136px;font-size:12px;color:#c7d0dd;flex:none;overflow:hidden;white-space:nowrap;text-overflow:ellipsis}
 .jnt .lim{color:#6b7688;font-size:10px;flex:none}
 .jnt input[type=range]{flex:1} .jnt .deg{width:42px;text-align:right;font-size:11px;color:#8b96a8;font-variant-numeric:tabular-nums}
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
 <div class="panel"><h1>皮卡丘点云 · 真蒙皮 19-DOF(LBS)</h1>
  <p class="tip">21 个关节自由度(参考 Pikachu_Retarget 映射): 每 DOF = (骨,轴,sign,限位)。
   骨可绕多轴(x/y/z)旋转并合成局部矩阵; 拖滑条 → FK 累积 → 每点按 top-4 权重混合各骨变换。</p>
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

const BONES=__BONES__, PTS=__PTS__, FLAT=__FLAT__, DOFS=__DOFS__;
const N=PTS.length, nb=BONES.length, W=4;
// bones 数组序即 id
const rest=[]; BONES.forEach((b,i)=>rest[i]=new THREE.Vector3(b.pos[0],b.pos[1],b.pos[2]));
const childRel=[]; BONES.forEach((b,i)=>childRel[i]= b.parent>=0? rest[i].clone().sub(rest[b.parent]) : new THREE.Vector3());
// 拓扑序: 从所有根一起 BFS(可多根), 兜底补漏
const AXIS_IDX={x:0,y:1,z:2};
// 每骨的局部旋转累积对象: rot[bone] = {x:deg, y:deg, z:deg} (已乘 sign)
const rot=[]; BONES.forEach(()=>rot.push({x:0,y:0,z:0}));
// 每个 DOF 一个滑条/值槽: dofVal[i] = 当前关节角(度, 未乘 sign)
const dofVal=new Float64Array(DOFS.length);
const dofBone=DOFS.map(d=>d[1]), dofAxis=DOFS.map(d=>AXIS_IDX[d[2]]), dofSign=DOFS.map(d=>d[3]),
      dofLo=DOFS.map(d=>d[4]), dofHi=DOFS.map(d=>d[5]);

const order=[]; (function(){const q=BONES.map((b,i)=>b.parent<0?i:-1).filter(i=>i>=0);
  while(q.length){const id=q.shift(); if(order.includes(id))continue; order.push(id);
    BONES.forEach((c,j)=>{ if(c.parent===id && !order.includes(j)) q.push(j); });}
  BONES.forEach((_,i)=>{ if(!order.includes(i)) order.push(i); });})();
// 每点权重解包 (flat: boneIdx0,w0 ... 4组)
const bw=[]; for(let i=0;i<N;i++){ const row=[]; for(let k=0;k<W;k++){ const bi=FLAT[(i*W+k)*2], w=FLAT[(i*W+k)*2+1]; row.push([bi, w]); } bw.push(row); }

const scene=new THREE.Scene(); scene.background=new THREE.Color('#141924');
const cam=new THREE.PerspectiveCamera(50,1,.01,100); cam.position.set(1.8,1.4,2.2);
const renderer=new THREE.WebGLRenderer({canvas:document.getElementById('gl'),antialias:true});
renderer.outputColorSpace=THREE.SRGBColorSpace;
scene.add(new THREE.AmbientLight(0xffffff,1.0));
scene.add(new THREE.DirectionalLight(0xffffff,1.6));
const grid=new THREE.GridHelper(4,20,0x33405a,0x232b3d); grid.position.y=0; scene.add(grid);
const ctrl=new OrbitControls(cam,renderer.domElement); ctrl.target.set(0,1.1,0); ctrl.update();

const geo=new THREE.BufferGeometry();
const posAttr=new Float32Array(N*3); geo.setAttribute('position',new THREE.BufferAttribute(posAttr,3));
const col=new Float32Array(N*3); geo.setAttribute('color',new THREE.BufferAttribute(col,3));
const hues=[0.02,0.10,0.16,0.20,0.28,0.34,0.40,0.46,0.52,0.58,0.64,0.70,0.76,0.82];
function tint(bi){ const c=new THREE.Color(); c.setHSL(hues[bi%hues.length],0.95,0.52); return c; }
const tints=BONES.map((_,i)=>tint(i));
const points=new THREE.Points(geo,new THREE.PointsMaterial({size:0.012,vertexColors:true,sizeAttenuation:true}));
scene.add(points);
const boneSegs=new THREE.LineSegments(new THREE.BufferGeometry(),new THREE.LineBasicMaterial({color:0x6fd0ff}));
scene.add(boneSegs);

const DEG=Math.PI/180;
const boneR=[], boneWorld=[];
BONES.forEach((_,i)=>{boneR.push(null); boneWorld.push(new THREE.Vector3());});
// 每骨局部旋转矩阵 = 绕其各轴旋转的合成(Rx*Ry*Rz)
const _R=new THREE.Matrix4();
function localMatrix(id){
  _R.identity();
  const r=rot[id];
  if(r.x!==0) _R.multiply(new THREE.Matrix4().makeRotationX(r.x*DEG));
  if(r.y!==0) _R.multiply(new THREE.Matrix4().makeRotationY(r.y*DEG));
  if(r.z!==0) _R.multiply(new THREE.Matrix4().makeRotationZ(r.z*DEG));
  return _R.clone();
}
const _v=new THREE.Vector3(), _a=new THREE.Vector3();
function fk(){
  // 把每个 DOF 的滑条值 ×sign 累加进对应骨对应轴
  for(const id of order) rot[id].x=rot[id].y=rot[id].z=0;
  for(let i=0;i<DOFS.length;i++){ const b=dofBone[i]; const v=dofVal[i]*dofSign[i];
    rot[b][['x','y','z'][dofAxis[i]]] += v; }
  for(const id of order){ const b=BONES[id];
    const L=localMatrix(id);
    boneR[id]= b.parent>=0? boneR[b.parent].clone().multiply(L) : L;
    if(b.parent<0){ boneWorld[id].copy(rest[id]); }
    else{ _v.copy(childRel[id]).applyMatrix4(boneR[b.parent]).add(boneWorld[b.parent]); boneWorld[id].copy(_v); }
  }
  // 顶点色: 按权重混合骨色
  for(let i=0;i<N;i++){
    let r=0,g=0,b=0;
    for(let k=0;k<W;k++){ const [bi,wdi]=bw[i][k]; if(bi<0)continue; const t=tints[bi]; r+=wdi*t.r; g+=wdi*t.g; b+=wdi*t.b; }
    col[i*3]=r; col[i*3+1]=g; col[i*3+2]=b;
  }
  geo.attributes.color.needsUpdate=true;
  // LBS 位置
  for(let i=0;i<N;i++){
    _a.set(PTS[i][0],PTS[i][1],PTS[i][2]);
    const out=new THREE.Vector3();
    for(let k=0;k<W;k++){ const [bi,wdi]=bw[i][k]; if(bi<0||wdi<1e-6)continue;
      _v.copy(_a).sub(rest[bi]).applyMatrix4(boneR[bi]).add(boneWorld[bi]).multiplyScalar(wdi);
      out.add(_v); }
    posAttr[i*3]=out.x; posAttr[i*3+1]=out.y; posAttr[i*3+2]=out.z;
  }
  geo.attributes.position.needsUpdate=true;
  const seg=[];
  for(let i=0;i<nb;i++){ const b=BONES[i]; if(b.parent>=0){ const p=boneWorld[b.parent], c=boneWorld[i];
    seg.push(p.x,p.y,p.z, c.x,c.y,c.z); } }
  boneSegs.geometry.setAttribute('position',new THREE.Float32BufferAttribute(seg,3));
}
function poseStats(){
  let s=0,maxv=0,act=0; const bins=[0,0,0,0,0,0];
  for(let i=0;i<N;i++){ const dx=posAttr[i*3]-PTS[i][0], dy=posAttr[i*3+1]-PTS[i][1], dz=posAttr[i*3+2]-PTS[i][2];
    const m=Math.sqrt(dx*dx+dy*dy+dz*dz); s+=m; if(m>maxv)maxv=m; if(m>0.05)act++;
    const b=Math.min(6-1,(m/0.1)|0); if(m>0.01)bins[b]++; }
  return 'mean='+(s/N*1000|0)+'mm max='+(maxv*1000|0)+'mm act>5cm='+((act/N*1000)|0)+'‰';
}
function movedY(){ let s=0; for(let i=0;i<N;i++) s+=Math.abs(posAttr[i*3+1]-PTS[i][1]); return s; }
const hud=document.getElementById('hud'), stat=document.getElementById('stat');
function refresh(){ fk();
  const moving=DOFS.filter((_,i)=>dofVal[i]!==0).length;
  document.title='pcd-skin-19dof movedY='+movedY().toFixed(2)+' activeDOF='+moving;
  hud.textContent='点云 '+N+' · 19-DOF 真蒙皮 · 拖滑条转关节(带限位) → 权重混合 LBS 随动 · 已动 '+moving+'/19 DOF'; }

// 滑条面板: 按身体部位分组, 每 DOF 一行, 标注限位
const holder=document.getElementById('sliders');
const GROUPS={'头部':['head'],'臂_L':['arm_pitch_L'],'臂_R':['arm_pitch_R'],
 '腿_L':['hip_pitch_L','hip_knee_L','hip_ankle_L'],'腿_R':['hip_pitch_R','hip_knee_R','hip_ankle_R']};
const groupOf={};
for(const g in GROUPS) for(const bn of GROUPS[g]) groupOf[bn]=g;
const groupsOrder=['头部','臂_L','臂_R','腿_L','腿_R'];
const rows=[];
const mkGroups={};
groupsOrder.forEach(g=>{ mkGroups[g]=document.createElement('div'); mkGroups[g].className='grp';
  mkGroups[g].innerHTML='<div class="t">'+g+'</div>'; holder.appendChild(mkGroups[g]); });
DOFS.forEach((d,i)=>{
  const [name,boneIdx,ax,sgn,lo,hi]=d; const g=groupOf[BONES[boneIdx].name]||'腿_L';
  const row=document.createElement('div'); row.className='jnt';
  row.innerHTML='<label>'+name.split('_').slice(1).join(' ') + ' ('+ax.toUpperCase()+fy(sgn)+')</label>'+
    '<span class="lim">['+lo+'…'+hi+']</span><input type="range" min="'+lo+'" max="'+hi+'" step="1" value="0"><span class="deg">0°</span>';
  const rng=row.querySelector('input'), dg=row.querySelector('.deg');
  rng.oninput=()=>{ dofVal[i]=+rng.value; dg.textContent=rng.value+'°'; refresh(); };
  mkGroups[g].appendChild(row); rows.push({rng,dg,i}); });
function fy(s){ return s>=0?'+':'-'; }

const leg=document.createElement('div'); leg.className='legend';
leg.innerHTML='<div class="t" style="color:#7fd0ff">骨骼权重配色</div>';
BONES.forEach((b,i)=>{ const c=tints[i]; leg.innerHTML+='<div class="row"><span class="sw" style="background:rgb('+(c.r*255|0)+','+(c.g*255|0)+','+(c.b*255|0)+')"></span>'+(b.name)+(b.parent<0?' (根)':'')+'</div>';});
(stat.parentElement||holder).appendChild(leg);

window.toggleBone=()=>{ boneSegs.visible=!boneSegs.visible; document.querySelector('.tools button').textContent='骨骼线:'+(boneSegs.visible?'开':'关'); };
window.resetPose=()=>{ dofVal.fill(0); rows.forEach(r=>{r.rng.value=0; r.rng.nextElementSibling.textContent='0°';}); refresh(); };
let _auto=null;
window.autoPose=()=>{ if(_auto){clearInterval(_auto);_auto=null;resetPose();return;} let t=0;
  _auto=setInterval(()=>{ t+=0.05;
    rows.forEach((r,i)=>{ const lo=dofLo[r.i], hi=dofHi[r.i]; const mid=(lo+hi)/2, amp=(hi-lo)/2;
      const v=Math.round(mid+Math.sin(t*2+i)*amp*0.6); r.rng.value=v; r.rng.nextElementSibling.textContent=v+'°'; dofVal[r.i]=v; });
    refresh(); if(t>6){clearInterval(_auto);_auto=null;} },40); };

// 测试姿态 ?pose=1: 抬左腿 knee+hip_pitch、摆右臂、转头
if(new URLSearchParams(location.search).has('pose')){
  const setD=(nameDeg,v)=>{ const i=DOFS.findIndex(d=>d[0]===nameDeg); if(i>=0){ dofVal[i]=v; } };
  setD('left_knee',45); setD('left_hip_pitch',25); setD('right_arm_pitch',40);
  setD('head_yaw',18); setD('right_hip_yaw',8);
  for(const r of rows){ r.rng.value=dofVal[r.i]; r.rng.nextElementSibling.textContent=dofVal[r.i]+'°'; }
  refresh();
  let st; try{ st=poseStats(); }catch(e){ st='STATERR '+e.message; }
  document.title += ' | ' + st;
  const sdiv=document.getElementById('stat'); if(sdiv) sdiv.textContent=st;
}
function resize(){ renderer.setPixelRatio(Math.min(devicePixelRatio,2));
  const w=document.getElementById('view').clientWidth, h=document.getElementById('view').clientHeight;
  renderer.setSize(w,h,false); cam.aspect=w/h; cam.updateProjectionMatrix(); }
window.addEventListener('resize',resize);
function loop(){ requestAnimationFrame(loop); ctrl.update(); renderer.render(scene,cam); }
resize(); refresh(); loop();
</script></body></html>
"""
    html = (html.replace("__BONES__", js_bones)
                .replace("__PTS__", js_pts)
                .replace("__FLAT__", js_flat)
                .replace("__DOFS__", js_dofs))
    with open(OUT, "w", encoding="utf-8") as f:
        f.write(html)
    print("已写", OUT, " %.2f MB | 点 %d 骨 %d DOF %d" %
          (os.path.getsize(OUT) / 1048576, N, nb, len(dofs)))


if __name__ == "__main__":
    main()