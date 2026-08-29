# FLEX 软体方案选型 · 实验记录

> 目标：把皮卡丘点云外皮当成「物理属性点云」，在物理引擎里落地接触后产生**可测量的凹陷/压扁**。
> 现状约束：本机 MuJoCo 3.12.0 无原生软体（`<flex>/<softbody>/<deformable>` 都被编译掉，报 Schema violation）。
> 结论先行：走 **A. 纯 MuJoCo 软 equality（connect）弹性点云** —— 已收敛、有量化凹陷记录（Part 2A 完成）；B. PyBullet FEM 待续（Part 2B）。

---

## 选型对比

| 方案 | 原理 | 原生软体 | 现状 |
|------|------|---------|------|
| **A. 纯 MuJoCo 软 connect** | 每个壳点=自由质点，用带 `solref/solimp` 的软 `connect` 约束把它耦合到刚性内核定位 site；落地地面压力把底部点压陷，弹簧拉回 | ❌ 无（不需原生软体）| ✅ **已收敛**，压扁 9%，凹陷 5.7mm |
| B. PyBullet 软体 FEM | `loadSoftBody`（FEM tetras / deformable），真正的可变形网格 | ✅ 有 | ⏳ 待做 |
| C. MuJoCo `<flex>` | 原生粒子软体 | ❌ 本构建编译掉了（需 CUDA 重建）| 排除 |

---

## Part 2A —— 纯 MuJoCo 软 connect 弹性点云（完成 ✅）

文件：`flex/sim/soft_dent.py`　输出：`flex/sim/reports/soft_dent_*.png/json`

### 物理骨架

```
worldbody
├─ ground (plane)
└─ core  (free joint + 阻尼)          ← 刚性内核，只做「定位参考点」
│     ├─ site s0..sN-1               ← 壳点 rest 位（在核上的局部坐）
│     └─ coreviz (小球, 视觉示意)
└─ pt0..ptN-1                        ← 每个壳点 = 自由质点(带阻尼) + 小球 geom
       └─ 第 i 个 <connect site1=s_i site2=pt_i solref=... solimp=...>
```

- 每个壳点一只球体（r=0.014m），一处**软 connect** 把它的球心拉向核上对应 site。
- `solref="tc 1"` 越小越软（越易被压陷）；`solimp` 设阻尼/刚度系数。
- 落地后地面把底部点顶向上、压向核 → 外皮凹陷；松释弹簧拉回。

### 踩坑记录（都有实证，别再踩）

1. **MuJoCo 没有 `<equal>` 元素** → 用 `<connect>`（`weld` 是刚焊会抖，`connect` 才是软弹簧）。
2. `<connect>` 不能同时给 body+site 锚点 → 只用 `site1`/`site2` 一对。
3. `autolimits` 取值是 `true/false`，不是 `"on"`。
4. 壳点 body 必须有 `<joint>`，否则定死在世界系不落体 → 加 `type="free"`。
5. body 索引偏移：body0=world、body1=core，壳点从 **body2** 起。（曾用 `range(1,n+1)` 把 core 也算进壳点）
6. **内核球要小**（r≈0.05）：太大时把出生时嵌在几何内的壳点卡住 → 出生即深穿透爆炸。
7. drop 要大于半高（点云 z 半跨度 0.58）否则出生即在土里 → 爆炸；用 drop≥1.2。
8. **真正的收敛全靠关节速度阻尼**：没有阻尼时自由质点弹簧网络永远震荡（末速 v 高达 46~865）。但 `<freejoint/>` 不接受 damping/class → 必须写显式 `<joint type="free" damping="N"/>`；`<default><joint damping=../>` 对 freejoint 也不稳。**阻尼 ~1.5–2 + tc∈[0.05,0.4] 收敛**（末速<1）。
9. 相机：MuJoCo 没有 `trackcom` 默认相机；用 `<camera mode="targetbody" target="core">`。加 `<visual><global offwidth= offheight=>` 才能大于 640 宽离屏渲染。
10. `conda run -n mocap python - <<EOF` 不吃 stdin（静默）→ 用临时 `.py` 文件跑。

### 一次收敛记录（reports/soft_dent_tc0.15.json）

| 参数 | 值 |
|------|----|
| 点数 n / 软度 tc / 阻尼 damp / 落高 drop | 100 / 0.15 / 1.5 / 1.2 m |
| 首次触地 t | 2.006 s |
| 最深压入（底部球被压）| **5.7 mm** |
| 刚体垂直跨度 → 静止垂直跨度 | 1.249 → **1.136 m**，**压扁 9.0 %** |
| 末速 / 收敛 | 0.13 / ✓ |

off-screen 关键帧 hover（浮空）→ peak（触地）→ rest（静置）像素校验：
hover→rest 差 **58%**（落地身姿大变），peak→rest 仅 **5%**（同处静置期）——三帧时序正确。

### 复现

```bash
conda run -n mocap python flex/sim/soft_dent.py \
  --n 100 --tc 0.15 --damp 1.5 --drop 1.2 --settle 1800 --steps 3000 --png soft_dent
```

---

## Part 2B —— PyBullet 软体 Fem（完成 ✅）

文件：`flex/sim/soft_dent_pb.py`　输出：`flex/sim/reports/soft_dent_pb.json`

### PyBullet 软体接口测绘（都实证过）

PyBullet soft body 有两条路：
1. **真 FEM**：URDF `<deformable>` + `.vtk` 四面体体积网格（neohookean `mu/lambda/damping`）。
   `pybullet_data` 只随带 `torus_deform.urdf` 却**缺 `torus.vtk`** → 真要 FEM 得自建 tet 网格（本实验没走到，留作边界）。
2. **loadSoftBody(obj)**（本实验走通的方案 B）：读三角面 obj → 每顶点一质点、弹簧阻尼连 → 可形变表面。用它作「另一套软体」与 MuJoCo 的软 connect 点云对照。

**PyBullet 3.2.7 接口真相（全踩过）：**
- `soft` 相关只有 `loadSoftBody` + `createSoftBodyAnchor`（没有 `createSoftBodyShape`）。
- `loadSoftBody` **不接受 `basePosition/baseOrientation`**（报 Cannot convert）→ 位置要**烘焙进 obj 顶点**。
- 合法 kwarg 只有：`scale / mass / useNeoHookean / useBendingSprings / springElasticStiffness / springDampingStiffness / collisionMargin / repulsionStiffness / useSelfCollision`。
- `createSoftBodyAnchor` 签名是 **5 参**：`(softId, nodeIndex, bodyUniqueId, bodyLinkIndex, anchorPos)`。
- `getMeshData(soft)` → `(flag, positions_tuple)`，每项 `(x,y,z)`；顶点数会**膨胀**（64→294），解析每个 3 元组即用。
- 顶面软网被球压时会「把网顶成山」反向抬升 → 要做局部凹坑，正确姿势是**软垫平铺地面、球从上方压**（教科书 deformable-dent），凹痕量 `压点垫高 − 最深`。
- 无离屏渲染：DIRECT 模式直接量顶点 z，数值验证、不读图。

### 一次收敛记录（reports/soft_dent_pb.json）

| 参数 | 值 |
|------|----|
| 软垫 8×8=64 顶点 · 厚 5cm · neohookean·弹簧刚 260 | 球 r=0.35m 落高 1.2m |
| 球压点垫高基线 | 0.100 m |
| **凹痕最深（球压点垫高下降）** | **23.3 mm** @ step 205 |
| 静置后残留凹痕 / 回弹 | 10.9 mm / **回弹 12.4 mm** |

局部弹性压痕 + 回弹 → 这是「软体躺在地面被球压凹、松开回弹」的经典表现。比 MuJoCo 点云（纯表面压扁）多了**体积/厚度压缩**（球把垫压薄了）。

### 复现

```bash
conda run -n mocap python flex/sim/soft_dent_pb.py --nx 8 --ny 8 --radius 0.35 --drop 1.2
```

## 实时可视化（交互窗口，非离屏）

上面 A/B 是离屏数值实验，另有配套**弹窗实时看**的交互版（需本地显示器 `DISPLAY`；本机已确认 `DISPLAY=:0` 且 muojoc/viewer+PySide6+glfw 齐全）：

```bash
# A. MuJoCo 实时: 弹性点云落地→压陷→回弹(右键拖看任意角度)
conda run -n mocap python flex/sim/soft_dent_live.py --n 100 --tc 0.15 --damp 1.5
#    操作: 空格=暂停  R=重置回落(反复看凹陷)  关窗退出
# B. PyBullet 实时: 球砸软垫→凹坑→回弹
conda run -n mocap python flex/sim/soft_dent_pb_live.py --radius 0.35 --drop 1.2
#    操作: 拖动视角  R=重建重放  空格=暂停  右上⛶=暂停物理
# 无显示器自检(不弹窗): 两个脚本都加 --check
```

### 对比小结

| | A. MuJoCo 软 connect 点云 | B. PyBullet loadSoftBody 软网 |
|---|---|---|
| 变形模式 | 表面弹簧点、压扁/凹陷 | 厚度压缩的局部凹坑，有回弹 |
| 原生软体 | 无（用软约束拼）| 有（弹簧网/neohookean）|
| 深度实测 | 压扁 9%、压入 5.7mm | 凹痕 23.3mm→回弹 12.4mm |
| 接口难度 | 中（一堆 XML 坑）| 高（kwarg/anchor/signature 处处踩）|
| 参数直觉 | 直观（tc/damp）| 玄（stiffness/还是点网）|
| 真 FEM 体积 | ❌ | ⚠️ 需自建 .vtk 四面体 |

---
见 `flex/README.md` 总览。