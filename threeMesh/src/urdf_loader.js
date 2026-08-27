/**
 * urdf_loader.js —— 用纯 three.js DOM 解析一套 URDF：
 *   1. XML(map) 拆出 links / joints / 每关节 origin(xyz,rpy) / axis / 视觉 mesh(STL)
 *   2. 每个 link 建一个 THREE.Group，挂上它的 STL 视觉壳
 *   3. 每个 joint 建 pivot 组(origin) + spin 组(绕 axis 转)，把子 link 挂进 spin
 *   这样 FK 由场景层级自动算：转 spin = 转该 link 及以下全部后代。
 * 返回 { root, joints: Map<jointName, Joint> }，Joint.setAngle(rad) 驱动。
 */
import * as THREE from 'three';

const _eps = 1e-6;

/**
 * 手写二进制 STL 解析（Open3D 导出的这种：80 字节头 + uint32 三角数 + 每三角
 * 法向3 + 顶点3 + 2字节属性）。three 的 STLLoader 会把这类头误判成 ASCII 导致
 * parseInt 出天文数字，所以这里直接按确定格式读。
 * 属性高 5 位若为 0x8000，则低 15 位是 Open3D 5-bit BGR 顶点色。
 */
function parseBinaryStl(buf) {
  const dv = new DataView(buf);
  const count = dv.getUint32(80, true);
  const positions = new Float32Array(count * 9);
  const normals = new Float32Array(count * 9);
  const colors = new Float32Array(count * 9);
  const indices = new Uint32Array(count * 3);
  let p = 84, vi = 0;
  for (let t = 0; t < count; t++) {
    const nx = dv.getFloat32(p, true), ny = dv.getFloat32(p + 4, true), nz = dv.getFloat32(p + 8, true); p += 12;
    for (let v = 0; v < 3; v++) {
      const x = dv.getFloat32(p, true), y = dv.getFloat32(p + 4, true), z = dv.getFloat32(p + 8, true); p += 12;
      positions[vi * 3] = x; positions[vi * 3 + 1] = y; positions[vi * 3 + 2] = z;
      normals[vi * 3] = nx; normals[vi * 3 + 1] = ny; normals[vi * 3 + 2] = nz;
      vi++;
    }
    const attr = dv.getUint16(p, true); p += 2;
    // Open3D 顶点色（5-bit BGR）放属性高 15 位
    let r = 0.72, g = 0.75, b = 0.8;
    if (attr & 0x8000) {
      const col = attr & 0x7fff;
      r = ((col >> 10) & 0x1f) / 31; g = ((col >> 5) & 0x1f) / 31; b = (col & 0x1f) / 31;
    }
    const base = t * 9;
    for (let v = 0; v < 9; v++) colors[base + v] = v % 3 === 0 ? r : (v % 3 === 1 ? g : b);
    indices[t * 3] = t * 3; indices[t * 3 + 1] = t * 3 + 1; indices[t * 3 + 2] = t * 3 + 2;
  }
  const geo = new THREE.BufferGeometry();
  geo.setAttribute('position', new THREE.BufferAttribute(positions, 3));
  geo.setAttribute('normal', new THREE.BufferAttribute(normals, 3));
  geo.setAttribute('color', new THREE.BufferAttribute(colors, 3));
  geo.setIndex(new THREE.BufferAttribute(indices, 1));
  geo.computeBoundingSphere();
  return geo;
}

function vec(ns, ax, ay, az, def) {
  if (ns == null) return new THREE.Vector3(...def);
  return new THREE.Vector3(
    parseFloat(ns.getAttribute(ax) || def[0]),
    parseFloat(ns.getAttribute(ay) || def[1]),
    parseFloat(ns.getAttribute(az) || def[2]),
  );
}

/** URDF rpy（弧度，绕固定轴 Z→Y→X）转 quaternion。 */
function rpyQuat(r, p, y) {
  const q = new THREE.Quaternion();
  const e = new THREE.Euler(r, p, y, 'ZYX');
  return q.setFromEuler(e);
}

async function loadStl(url) {
  const buf = await (await fetch(url)).arrayBuffer();
  return parseBinaryStl(buf);
}

/**
 * @param {string} urdfUrl   /lib 之外绝对 http 路径，如 /assets/urdf.urdf
 */
export async function loadURDF(urdfUrl) {
  const absUrl = new URL(urdfUrl, location.origin).href;
  const doc = new DOMParser().parseFromString(
    await (await fetch(absUrl)).text(), 'text/xml',
  );
  const base = new URL('.', absUrl).href;

  // ---- pass1: link 组 + 视觉 STL ----
  const links = new Map();
  const linkNodes = [...doc.querySelectorAll('link')];
  await Promise.all(linkNodes.map(async (ls) => {
    const name = ls.getAttribute('name');
    const g = new THREE.Group();
    g.name = name;
    links.set(name, g);
    // 每个 <visual>/<geometry>mesh 都挂上去（相对 urdf 目录的 filename）
    const jobs = [...ls.querySelectorAll('geometry mesh')].map(async (m) => {
      const fname = m.getAttribute('filename') || '';
      let url;
      try { url = new URL(fname, base).href; }
      catch { url = base + fname.replace(/^[./]+/, ''); }
      const sc = m.querySelector('scale');
      const s = sc ? new THREE.Vector3(
        parseFloat(sc.getAttribute('x') || 1),
        parseFloat(sc.getAttribute('y') || 1),
        parseFloat(sc.getAttribute('z') || 1),
      ) : new THREE.Vector3(1, 1, 1);
      const geo = await loadStl(url);
      if (!geo.boundingSphere) geo.computeBoundingSphere();
      const mesh = new THREE.Mesh(geo, new THREE.MeshStandardMaterial({
        vertexColors: true, metalness: 0.05, roughness: 0.65,
      }));
      mesh.scale.copy(s);
      // 视觉 origin 偏移
      const vo = ls.querySelector('visual origin');
      if (vo) mesh.position.copy(vec(vo, 'xyz', 'y', 'z', [0, 0, 0]));
      g.add(mesh);
    });
    await Promise.all(jobs);
  }));

  // ---- pass2: joints → pivot + spin 层级 ----
  const baseLink = doc.querySelector('link[name="base_link"]')?.getAttribute('name');
  const joints = new Map();
  for (const js of doc.querySelectorAll('joint')) {
    const name = js.getAttribute('name');
    const type = js.getAttribute('type');
    const parent = js.querySelector('parent')?.getAttribute('link');
    const child = js.querySelector('child')?.getAttribute('link');
    const o = js.querySelector('origin');
    const pos = vec(o, 'xyz', 'y', 'z', [0, 0, 0]);
    const rpy = o ? [o.getAttribute('rpy') || '0'] : ['0'];
    const rp = (rpy[0] || '0 0 0').split(/\s+/).map(Number);
    const axis = vec(js.querySelector('axis'), 'xyz', 'y', 'z', [1, 0, 0]);
    let lim = null;
    const l = js.querySelector('limit');
    if (l) lim = [
      parseFloat(l.getAttribute('lower') || -Math.PI),
      parseFloat(l.getAttribute('upper') || Math.PI),
    ];

    const pivot = new THREE.Group();   // 放在关节原点（含 rpy 偏置）
    pivot.name = name + '#pivot';
    pivot.position.copy(pos);
    pivot.quaternion.copy(rpyQuat(rp[0], rp[1], rp[2]));

    const spin = new THREE.Group();    // 绕轴旋转的节点
    spin.name = name + '#spin';
    pivot.add(spin);

    // 找到父 link 组
    let parentGroup = links.get(parent);
    if (!parentGroup) parentGroup = links.get(name);
    const childGroup = links.get(child);
    if (childGroup) spin.add(childGroup);   // 子 link 移到 spin 下，随旋转一起动

    // 若第一次见到这个父 link，先给它挂上
    if (parentGroup && !parentGroup.children.includes(pivot)) parentGroup.add(pivot);

    const j = {
      name, type, axis: axis.clone().normalize(),
      min: lim ? lim[0] : -Math.PI, max: lim ? lim[1] : Math.PI,
      _spin: spin, _angle: 0,
      setAngle(rad) {
        this._angle = rad;
        if (rad === 0) spin.quaternion.identity();
        else spin.quaternion.setFromAxisAngle(j.axis, rad);
      },
      getAngle() { return this._angle; },
    };
    joints.set(name, j);
  }

  const root = links.get(baseLink) || links.values().next().value;
  return { root, joints };
}