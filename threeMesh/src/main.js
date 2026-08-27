/**
 * main.js —— 预研主程序：同一 three.js 场景里
 *   · 左边：纯 three.js 载入 URDF(assets/urdf.urdf + STL 壳)，FK 逐骨
 *   · 右边：skin.glb 蒙皮(DEF-* 变形骨)
 * 一张滑块表把 URDF 关节角同时写到 URDF FK 节点 + 蒙皮 DEF 骨，验证随动。
 *
 * 随动映射(CONFIG)是「最小可用占位」——真正 21 关节的 bone/axis/sign 由你之后细对。
 * 每个条目 JS 运行时会自动从骨格的父子位置算出「铰链轴」，所以基本不用手调方向。
 */
import * as THREE from 'three';
import { OrbitControls } from '/lib/OrbitControls.js';
import { GLTFLoader } from '/lib/GLTFLoader.js';
import { loadURDF } from './urdf_loader.js';

// 离屏验证埋点（跑普通浏览器无影响）
window.__boot = { step: 'imports-done' };

// ---------- 随动映射（占位，待细对） ----------
// 注意：GLTFLoader 会把导出骨名里的 `.` 剥掉（DEF-shin.L → DEF-shinL），所以这里用无点版。
// 名字对应同一套骨架所有副本（glTF 里 4 套蒙皮各引一份），下面按名驱动全部。
const CONFIG = [
  { urdf: 'left_knee_joint',         bone: 'DEF-shinL',        ui: '左膝 left_knee → DEF-shin' },
  { urdf: 'left_ankle_joint',        bone: 'DEF-footL',        ui: '左踝 left_ankle → DEF-foot' },
  { urdf: 'left_hip_pitch_joint',    bone: 'DEF-thighL',       ui: '左髋 pitch → DEF-thigh' },
  { urdf: 'left_elbow_ankle_joint',  bone: 'DEF-forearmL',     ui: '左肘 elbow → DEF-forearm' },
  { urdf: 'left_arm_pitch_joint',    bone: 'DEF-upper_armL',   ui: '左肩 pitch → DEF-upper_arm' },
  { urdf: 'right_knee_joint',        bone: 'DEF-shinR',        ui: '右膝 → DEF-shin' },
  { urdf: 'right_arm_pitch_joint',   bone: 'DEF-upper_armR',   ui: '右肩 pitch → DEF-upper_arm' },
  { urdf: 'head_pitch_joint',        bone: 'head',             ui: '头 pitch → head' },
];

// ---------- 场景 ----------
const canvas = document.getElementById('gl');
const renderer = new THREE.WebGLRenderer({ canvas, antialias: true });
renderer.setPixelRatio(Math.min(devicePixelRatio, 2));
renderer.outputColorSpace = THREE.SRGBColorSpace;
renderer.toneMapping = THREE.ACESFilmicToneMapping;
renderer.shadowMap.enabled = true;

const scene = new THREE.Scene();
scene.background = new THREE.Color(0x10131a);

const camera = new THREE.PerspectiveCamera(50, 1, 0.01, 200);
camera.position.set(1.2, 1.5, 4.6);

const controls = new OrbitControls(camera, canvas);
controls.target.set(0, 0.7, 0);
controls.enableDamping = true;

scene.add(new THREE.AmbientLight(0xffffff, 0.6));
scene.add(new THREE.HemisphereLight(0xdfe8ff, 0x3a2f1a, 0.5));
const key = new THREE.DirectionalLight(0xffffff, 1.4);
key.position.set(2, 4, 3);
scene.add(key);

const grid = new THREE.GridHelper(8, 20, 0x3a4a5e, 0x26303e);
scene.add(grid);

// ---------- 布局：左 URDF / 右蒙皮 ----------
const URDF_GROUP = new THREE.Group();
URDF_GROUP.position.x = -1.7;
scene.add(URDF_GROUP);

const SKIN_GROUP = new THREE.Group();
SKIN_GROUP.position.x = 1.7;
SKIN_GROUP.position.y = 0.9;    // 让 Pikachu 脚踩地
scene.add(SKIN_GROUP);

const status = document.getElementById('status');
const jointsDom = document.getElementById('joints');

// ---------- 运行时给蒙皮骨算铰链 ----------
const _v = new THREE.Vector3();
const _b1 = new THREE.Vector3();
const _b2 = new THREE.Vector3();
const _q = new THREE.Quaternion();

// glTF 里同名的骨在 4 套蒙皮里各有副本；target 保存所有副本，驱动时全带上。
const targets = new Map(); // name -> {bones:[], restQs:[], axisLocal, sign}
function makeTarget(name, bones, allBones) {
  if (targets.has(name)) return targets.get(name);
  const primary = bones.find((b) => b.parent && b.parent.type === 'Bone') || bones[0];
  let axisLocal = new THREE.Vector3(0, 1, 0);

  if (primary && primary.parent) {
    // hip/knee/远端三个世界点，用两段骨向量叉积求铰链轴
    primary.parent.getWorldPosition(_b1);   // 近端(髋)
    primary.getWorldPosition(_b2);          // 转轴(膝)
    const seg1 = _b1.clone().sub(_b2).normalize();

    // 找该骨的远端后代作 seg2 端点（选中离转轴最远的一个）
    let tip = null, best = 0;
    for (const b of allBones) {
      if (b.parent === primary) {
        b.getWorldPosition(_v);
        const d = _v.distanceTo(_b2);
        if (d > best) { best = d; tip = _v.clone(); }
      }
    }
    if (tip) {
      const seg2 = tip.sub(_b2).normalize();
      const h = new THREE.Vector3().crossVectors(seg1, seg2);
      if (h.lengthSq() > 1e-6) {
        h.normalize();
        h.applyQuaternion(primary.getWorldQuaternion(new THREE.Quaternion()).invert());
        axisLocal = h;
      }
    }
  }

  const t = { bones, restQs: bones.map((b) => b.quaternion.clone()), axisLocal, sign: 1 };
  targets.set(name, t);
  return t;
}

// ---------- 主加载 ----------
const gltf = new GLTFLoader();
window.__boot.step = 'promise-started';
Promise.all([
  gltf.loadAsync('/assets/skin.glb'),
  loadURDF('/assets/urdf.urdf'),
]).then(([skinModel, urdf]) => {
  window.__boot.step = 'promise-resolved';
  const skin = skinModel.scene;
  skin.traverse((o) => { if (o.isMesh) o.castShadow = true; });
  SKIN_GROUP.add(skin);

  // 收集所有蒙皮骨
  const skelBones = [];
  skin.traverse((o) => { if (o.isSkinnedMesh) skelBones.push(...o.skeleton.bones); });
  const byName = new Map(skelBones.map((b) => [b.name, b]));
  window.__bonesTotal = skelBones.length; // 离屏验证埋点

  // 挂 URDF FK 根
  URDF_GROUP.add(urdf.root);

  // 构建控制条
  let ok = 0;
  for (const c of CONFIG) {
    const matches = skelBones.filter((b) => b.name === c.bone);
    const j = urdf.joints.get(c.urdf);
    if (!j) { logLine(`跳过：URDF 无 ${c.urdf}`); continue; }
    if (!matches.length) { logLine(`跳过：蒙皮无 ${c.bone}`); continue; }
    const target = makeTarget(c.bone, matches, skelBones);
    ok++;
    buildSlider(c, j, target);
  }
  logLine(`就绪：${ok}/${CONFIG.length} 组 URDF⇄蒙皮随动已装配`);
  logLine('拖动左栏滑块 → 同时驱动 URDF FK + 蒙皮 DEF 骨');
  window.__dbg = { joints: urdf.joints, targets };
}, (e) => {
  window.__boot.step = 'load-failed: ' + (e && (e.stack || e.message));
  logLine('加载失败：' + (e && e.message));
});

function buildSlider(c, j, target) {
  if (c.ui) {
    const l = document.createElement('label');
    l.innerHTML = `<span class="name">${c.ui}</span>`;
    jointsDom.appendChild(l);
  }
  const row = document.createElement('label');
  const span = document.createElement('span');
  span.className = 'pair'; span.textContent = '0°';
  const input = document.createElement('input');
  input.type = 'range';
  const deg = (r) => (r * 180 / Math.PI).toFixed(1) + '°';
  const lo = Math.max(j.min, -1.6), hi = Math.min(j.max, 1.6);
  input.min = lo; input.max = hi; input.step = 0.005; input.value = 0;
  input.oninput = () => {
    const rad = parseFloat(input.value);
    span.textContent = deg(rad);
    j.setAngle(rad);                                   // URDF FK
    if (rad === 0) {                                   // 归零
      for (let i = 0; i < target.bones.length; i++) target.bones[i].quaternion.copy(target.restQs[i]);
    } else {                                           // 蒙皮（同角度驱所有副本）
      _q.setFromAxisAngle(target.axisLocal, rad * target.sign);
      for (let i = 0; i < target.bones.length; i++) target.bones[i].quaternion.copy(target.restQs[i]).multiply(_q);
    }
    renderer.render(scene, camera);
  };
  row.appendChild(span); row.appendChild(input);
  jointsDom.appendChild(row);
}

function logLine(s) { status.textContent += '\n' + s; }
logLine('正在加载 URDF + skin.glb…');

// ---------- 渲染循环 ----------
function resize() {
  const w = canvas.clientWidth || window.innerWidth;
  const h = canvas.clientHeight || window.innerHeight;
  if (canvas.width !== w || canvas.height !== h) {
    renderer.setSize(w, h, false);
    camera.aspect = w / h;
    camera.updateProjectionMatrix();
  }
}
window.addEventListener('resize', resize);
resize();
function loop() {
  resize();
  controls.update();
  renderer.render(scene, camera);
  requestAnimationFrame(loop);
}
loop();