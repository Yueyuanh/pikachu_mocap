# Pikachu 连杆调校台 · 网页可视化重构规格

> 本文档是「把现有单文件网页工具整体重做」的完整规格，交给另一模型实现。
> 目标是：在不读现有源码、只看本文档的情况下，重建一个行为等价、工程更严谨的
> 网页端 3D 可视化与调校工具。
>
> - 产品：`Pikachu 连杆调校台`（URDF / xacro 连杆调校 + meshcat 实时 3D）
- 现状形态：一个 `pikachu_link_tuner.html`（前端，全是单页 JS）+ 一个
  `pikachu_link_tuner_server.py`（本地静态 + JSON API 后端）。
- 运行环境：Linux + `conda activate mocap`（meshcat、numpy、xacro、mujoco 仅在此环境）。

---

## 0. 一句话需求

> 一个本地网页调校台：加载 Pikachu 的 27-DOF URDF/xacro 模型，左侧用**参数滑动条 +
> 逐关节角度滑动条**实时改连杆尺寸/关节角，右侧用 **meshcat** 渲染实时 3D，并自动叠加
> **质心、质量、惯量、重心落到脚底的垂线、足端支撑多边形（平衡判定）**，整体浅蓝 meshcat
> 风格；初始姿态把机器人抬到脚底贴地；可一键导出 URDF / Xacro / MJCF / PNG / JSON。

---

## 1. 用户与使用场景

| 角色 | 场景 | 关键诉求 |
|---|---|---|
| 机器人调结构的人 | 改某根连杆的长/宽/高、改臂/耳/尾关节活动范围，立即看 3D 与重心变化 | 改一个数“手感立刻出来”，不弹插件、不重启 |
| 验证平衡的人 | 手动摆一个关节姿态、或回放一段 npz 动作 | 立即看到重心落点是否落在足端支撑多边形内 |
| 导出给下游的人 | 定稿后一键出 URDF/MJCF 喂给仿真/打印 | 导出与屏幕所见一致 |

**核心体验承诺**：
1. 任何滑动条拖动 → 450ms 内 3D 更新（40ms 防抖推送，本地无网络往返）。
2. 初始加载即脚底贴地，无需手动调相机/坐标。
3. 3D 里能一眼读出「稳不稳」：重心竖线 + 彩色支撑多边形。

---

## 2. 功能需求（可验收）

### F1 模型加载
- 站点根目录为后端所在目录；启动即从 `default/27dof/pikachu_sample_links_27dof.xacro`
  自动加载（http 方式，`fetch`），失败依次回退
  `default/pikachu_sample_links.xacro` → `pikachu_sample_links.xacro` → 内置 `<script type="text/plain">` 示例。
- 也支持 `file://` 或手动载入 `.xacro/.urdf/.xml` 文件（FileReader）。

### F2 参数面板（左侧）
- 解析 xacro 里的 `xacro:property` 数值，按分组（躯干/头/腿·骨盆/大腿/小腿/脚/臂/臂·限位/头·限位/其它）生成滑动条 + 数值框。
- 拖动滑动条即重解析 xacro（`expand`）→ 重建模型 → 刷新 3D/2D。（尺寸相关）
- 修改限位参数会作用到同名校的关节 limit（见 F3）。

### F3 关节面板（左侧，27-DOF）
- 对每个可动关节（revolute）生成一张卡片：名称、锁定勾选、`min/max/effort` 数值框、
  **一个角度滑动条**（弧度，范围 ≈ 限位的±3 倍或 ±0.5）。
- 拖动角度滑动条 → 写入全局 `Angle[关节名]` → FK 重算 → 推送 meshcat。这是当前唯一“驱动”
  手段（npz 回放已在 UI 移除，见 §10 备注）。
- 「锁定」勾选 = 该 child link 在导出/3D 中隐藏，不参与导出。

### F4 3D 视图（右侧，meshcat iframe）
- 顶栏：质心/惯量显示开关、落足重心/支撑多边形开关、「重置位姿」按钮、质心读数。
- 3D 内常驻：地面（顶面 z=0）、逐 link 的 colored box、可选：质心球、重心→脚底垂线、
  落足圆点、足端支撑多边形（绿=稳定 / 红=不稳定）。

### F5 质心/惯量
- 总质量、质心坐标、每 link 惯量，实时显示；读取行人一眼看懂「重心离脚底多高」「落足在哪」。

### F6 平衡判定
- 支撑多边形 = 两脚掌底部（最低角点）xy 的凸包；重心落点在其内且距边界 > 2mm 判为「稳定」。

### F7 初始贴地
- 任何一次模型加载/「重置位姿」后，自动按当前 FK 算全模型最低 box 角点 z，把整体 base Z 抬/降到
  脚底恰好 **z=0**（地面顶面）。

### F8 导出（底部）
- 保存 Xacro、导出 URDF、导出 MJCF、导出 2D 尺寸标注 PNG、一键导出全部（URDF+Xacro+MJCF+PNG+JSON）。
- 优先写选定目录（后端 `/api/save_dir`），失败退浏览器下载。

---

## 3. 人性化 / UX 设计约束

1. **即时反馈**：滑动条 `input` 事件（非 `change`）驱动；推送防抖 40ms。
2. **浅蓝 meshcat 主题**：CSS 变量控制，主背景 `#eef3fa`，强调 `#2f6fd6`；支持明/暗切换按钮。
3. **可读读数**：质心读数用等宽字体 + 绿色/红色 ``●`` 稳定圆点，附余量 mm。
4. **操作可逆**：「重置位姿」一键回 T-pose 贴地、「重置默认」一键还原示例源码。
5. **错误温和**：后端不可用（缺 meshcat）时各按钮 toast 中文提示，不白屏、不弹窗。
6. **布局自适应**：窄屏（≤860px）降级为单列；所有图/表内部 `overflow-x:auto`，页面不横向滚动。
7. **一致性**：坐标全部 URDF 约定（Z 轴向上）；读数字单位毫米；角度单位弧度（限位除外用度）。
8. 无障碍基线：所有控件可获得焦点（focus-visible outline）；按钮/勾选有 title 提示。

---

## 4. 架构与模块职责

```
浏览器 (单页 HTML+JS)
  ├─ expand()/parseRobot()   解析 xacro→Model(links+joints)
  ├─ computeTFs(model)       正逆运动学 FK，Angles→每 link 世界 {pos,R}
  ├─ worldTFs()              ×BaseT(根位姿偏移) 后返回世界系 TF
  ├─ systemCOM/balanceInfo   质心/支持多边形/平衡
  ├─ groundFeet()            初始贴地 base 偏移(T0Z)
  ├─ push3D()                防抖 POST /api/scene
  └─ iframe.meshcatFrame     src=meshcat url，浏览器本地渲染 meshcat

本机后端 python http.server (端口可指定，缺省自动)
  ├─ GET  /(静态)            pikachu_link_tuner.html 等
  ├─ POST /api/scene         把每 link 世界盒 + 覆盖层推到独占 meshcat 实例
  ├─ POST /api/npz_files     列 npz 动作库
  ├─ POST /api/npz_parse     解析 npz 帧→27 关节角(含 arm bias)
  ├─ POST /api/save | /api/save_dir    写文件到指定/选定目录
  ├─ POST /api/export_mjcf   后端 xacro→MJCF 一键导出
  └─ 进程内独占 meshcat.Visualizer()（自带网页，最新帧持续渲染）
```

**关键原则**：前端是 FK/平衡/CG 的单一事实源；后端只做 meshcat 渲染与文件导出，
不重复算 FK。这样浏览器算、后端画，二者解耦。

---

## 5. 数据模型

### 5.1 Model（前端 parseRobot 产物）
```
Model = {
  links: { <link名>: { size:[sx,sy,sz],   // 米(全尺寸)
                       origin:[x,y,z],     // visual box 中心相对 link 原点(米)
                       color:'r g b a' } },
  joints: [ { name, type:'revolute'|'fixed', parent, child,
              axis:[x,y,z], origin:{xyz, rpy}, limit:{lower,upper,effort},
              rpy:[...], apos:[...] } ]
}
```

### 5.2 27-DOF 关节清单（顺序即遍历顺序，前端按 `name` 对应）
| 分区 | 关节 |
|---|---|
| 腿 ×4 ×(5) | hip_pitch / hip_roll / hip_yaw / knee / ankle |
| 臂 ×2 ×(4) | arm_pitch / arm_roll / arm_yaw / elbow |
| 头 ×3 | head_pitch / head_yaw / head_roll |
| 耳 ×2 ×(2) | ear_pitch / ear_roll |
| 尾 ×2 | tail_pitch / tail_yaw |

> **关键装配约定**：新 27dof 的 `arm_roll` 默认是 **T-pose 展开**（臂水平外展，近平伸），
> `origin rpy = ±1.57` 已把装配位“烤”进几何；所以 0 角时手臂向外平伸，而非下垂（见 §7 arm bias）。

### 5.3 npz 动作格式（复用 Pikachu_Retarget 的 14 列）
```
npz keys: joint_pos(T,14), body_pos_w(T,17,3), body_quat_w(T,17,4 wxyz), fps
14 列顺序 = NPZ_COLS_14:
  左 hip_pitch, hip_roll, hip_yaw, knee, ankle,
  右 hip_pitch, hip_roll, hip_yaw, knee, ankle,
  左 arm_pitch,     arm_roll,
  右 arm_pitch,     arm_roll
(无肘/头/耳/尾；它们 npz 回放时保持 0)
```

---

## 6. 关键算法（必须与现实现等价）

### 6.1 角度 → 末端 FK
每个关节对其子 link 施加：先乘 `rpyToMat(origin.rpy)`（关节装配旋转），再乘
`axisRotMat(axis, θ)`（Rodrigues 绕关节轴转角度）。逐深度优先累乘得每 link 世界 `{pos,R}`。

### 6.2 世界系 + 根位姿
`worldTFs = computeTFs × BaseT`，其中 `BaseT={pos:[dx,dy,dz], R:3x3}`（默认 null=无偏移，
初始贴地时 `pos.z=T0Z`）。

### 6.3 质心 / 惯量（与 Mujoco 公式一致）
- 自动惯量（无覆盖时）：`m=ρ·sx·sy·sz`，`ixx=m/12(sy²+sz²)` 等（半轴公式，全尺寸）。
- 每 link 可由 `Inert[link]` 覆盖 `mass/ixx/iyy/izz` 中任一字段，缺省用自动值。
- 总体质心 = Σ(m·c)/Σm，`c = linkCOM = tf.pos + R·origin`（质心=visual origin=inertial origin）。
- `footZ` = 全模型所有 box 角点的最低 world z。

### 6.4 平衡判定
```
对每个 *_ankle_link 的 box，取其所有角点的 z==最低z 的点 → 投影到 xy → 凸包(monotone chain)
支持多边形 = 两脚掌凸包坐标；重心落点 = (com.x, com.y)
稳定 ⇔ 凸包顶点≥3 且 落点在内 且 到最近边距 > 0.002m
```

### 6.5 初始贴地（groundFeet）
```
footZ_T = 当前(全 0 角)FK 下的全模型最低 box 角点 z
T0Z = -footZ_T；BaseT := {pos:[0,0,T0Z], R:identity}
```
脚底即 z=0=地面顶面。改尺寸/Apply/重置都要重算。

### 6.6 npz arm bias（本工具核心适配点）
- 新 27dof：`arm_roll` 装配位=T-pose 外展，0 角=手平伸。
- 旧 npz：`arm_roll` 值 v 以下垂=0 计。
- 适配：`θ = v − π/2`（左右相同；几何推导 + 实测 frame0 得 θ≈−1.571 ≈ −π/2 吻合，手垂下）。
- 备选模式：`90−v`、`direct`。解析接口返回所用模式。

---

## 7. meshcat 渲染方案（后端）

**可用原语**（该版本 meshcat 无 `PolygonGeometry`/`LineSegments`）：`Box`,`Sphere`,
`TriangularMeshGeometry(verts(n,3), faces(n,3), color=)`。多边形用三角扇填充。

**每进程独占单例**：
```
mc = meshcat.Visualizer()        # 自动起自有 tornado，端口自选(≈7001+)
url = mc.url()                   # http://127.0.0.1:<port>/static/
```
- 前端把 `iframe.src = url`；此后任何 `set_transform/set_object` 即时可见。
- 场景结构：`vis["pikachu"][<link>]` 每 link 一个 box；`vis["fx"]` 覆盖层。
- 刷新策略：link 集合/尺寸/颜色变了才重建（`set_object`），位置/姿态每帧 `set_transform`（廉价）。
- 每 link 盒：`Box(size)` + `MeshLambertMaterial(color)`；`set_transform` 用 4×4 numpy。

**覆盖层 `fx`**：
| 层 | 几何 | 说明 |
|---|---|---|
| ground | Box(1.4,1.4,0.003) 中心 z=-0.0015，opacity≈0.14 | 顶面≈z=0 |
| com | Sphere(0.010) 红 | 质心球 |
| comDrop | 细长 Box，从质心垂直垂到脚底 z0 | 重心→脚底竖线 |
| comGround | 小圆盘(方块) Box≈0.024 红 | 落足点 |
| footPoly | TriangularMeshGeometry 三角扇 | 稳定绿 0x2ea869 / 不稳红 0xd64545，opacity 0.38 |

**脚底平面 z0（后端自动算，不依赖前端）** = 所有 link box 的 `pos.z − size.z/2` 最小值。
用于 comDrop 长度与 footPoly 面高。保证与「贴地地面」一致。

---

## 8. API 契约（精确 schema，重做必须兼容）

基础：`Content-Type: application/json`；BaseURL=`http://<host>:<port>/`。Host 可 `127.0.0.1`；端口 `--port` 或缺省自动找空闲端口。

### 8.1 `GET /api/meshcat`
```
200  {"ok":true,"url":"http://127.0.0.1:<p>/static/"}
400  {"ok":false,"error":"meshcat 不可用,请用 conda activate mocap ..."}   # 缺 meshcat
```
前端：拿到 url → 首帧成功后设 iframe src + 隐藏“载入中”。

### 8.2 `POST /api/scene`（实时 3D 核心，防抖 40ms）
请求：
```
{
  "full": bool,            // link 集合/尺寸/颜色 是否变化需重建
  "links": [ { "name","size":[3],"pos":[3],"r9":[9] /*R 展开行优先*/,
                "color":"#rrggbb" } , ... ],
  "com":[3],               // 质心(世界)，可选
  "feet": {"z":num, "poly2d":[[x,y],...]},   // 支撑多边形，可选
  "balance": bool
}
```
响应：`200 {"ok":true,"url":"..."}`；缺 meshcat → `400 {"ok":false,"error":...}`。
后端行为：`full` 或 link 集合变了 → 重建 `pikachu/*`；逐 link `set_transform`；调用覆盖层。

### 8.3 `POST /api/npz_files`
请求 `{"dir":str?}`；缺省用 `--npz-dir`。响应
```
{"ok":true,"dir":abs, "files":["Pikachu-Transition-Down.npz", ...]}  # 按名排序,仅 .npz
```

### 8.4 `POST /api/npz_parse`
请求 `{"path":abs, "armBias":"v-90"|"90-v"|"direct"}`
响应：
```
{"ok":true,"fps":50,"n":316,"jointNames":[ALL_27_NAMES],
 "joints":[[θ0_0..θ0_26], ...],       # (n,27) 弧度
 "basePos":[ [x,y,z],... ]|null, "baseRpy":[[度...],...]|null,
 "armBias":"v-90"}
```
错误（路径无效/非 npz/读失败/缺 joint_pos/维度错）→ `400 {"ok":false,"error":...}`。

### 8.5 保存/导出 （可保持兼容）
`POST /api/save`（写当前文件）、`/api/save_dir`（选定目录）、`/api/export_mjcf`（后端 xacro→MJCF）。
规则：文件名清洗 `_safe_name`（仅 `[A-Za-z0-9._ -]`，拒 `..`）；导出 MJCF 需 `xacro+mujoco` 环境，若缺 → 明确报错文案。

---

## 9. 前端状态与流程示意

```
全局: Model, Props, Angle{joint:rad}, Inert{link:{...}}, BaseT, T0Z, JointConf{joint:{fixed,lower,upper,effort}}, DENSITY=1500

boot():
  setSize/resize 绑定
  绑定: 重置位姿 → resetPose(); 明暗 → 主题切换
  meshcat: GET /api/meshcat → 记 url
  npz: (回放 UI 已移除) 见 §10
  加载 xacro: tryLoad(candidates) → loadSrc(text) → rebuildScene()
  rebuildScene(): expand→parseRobot→Model; buildJointControls; buildInertControls;
                  updateStats; groundFeet(); renderAll()

renderAll()/updateView():
  tfs = worldTFs()
  push3D(tfs)          # 防抖 40ms POST /api/scene
  render2D(Model)      # 左侧 2D 侧视简图(可选保留)
  updateComReadout()   # 质心读数 + 距脚底高 + 稳定性

resetPose():
  Angle 全 0 → groundFeet()(重算贴地偏移) → 同步滑块/读数 → updateView()
```

JS 依赖：约 1380 行单 `<script>`，`node --check` 可静态验语法。

---

## 10. 功能取舍与待办备注

- 【已移除】npz 回放：上一版有「npz 文件/帧/播放」控件，本轮按“手动关节优先”移除 UI 与相关 JS；
  后端 `/api/npz_files|parse` **保留**（渲染与其它工具仍可用）。npz 的 arm bias 逻辑仍在后端，
  如需恢复回放 UI 只需接回前端事件。
- 【只留 slider】逐关节角度滑动条是当前唯一驱动手段；如需多通道，可把 §8.4 parse 结果再灌回
  `Angle[]` + 触发 updateView，即还原回放。

---

## 11. 环境与工程约束（重做必读）

| 约束 | 说明 |
|---|---|
| Python 环境 | 用 `~/miniconda3/envs/mocap/bin/python` 启动 server（python:multiprocessing, meshcat, numpy, xacro, mujoco 齐全）。系统 python 无 meshcat → `/api/scene|meshcat|export_mjcf` 返回 4xx 提示而非崩溃。 |
| meshcat 依赖 numpy | 但**不要在函数外 import numpy** 会造成跨函数 NameError 的历史坑；每个用 `np.*` 的独立函数要**函数内 `import numpy as np`**。 |
| `Visualizer` 版本 | 该版本**无** `open_browser` 参数；签名 `Visualizer(zmq_url=None, window=None, server_args=[])`。用 `Visualizer()` + 手动设 iframe src。 |
| 无 PolygonGeometry | 多边形填充用 `TriangularMeshGeometry` 三角扇。 |
| URDF 轴约定 | 世界 Z 向上；地面顶面 = z=0；所有视觉初态自动贴地。 |
| 不要读图 | 代码/验收全用数值与接口验证，禁止依赖分析截图。 |

---

## 12. 验收标准（交付前逐项核验）

### 12.1 UX 验收
1. 打开 `http://127.0.0.1:<port>/pikachu_link_tuner.html` → 浅蓝主题、自动加载 27dof、meshcat iframe 出现机器人 **且脚底贴 z=0**；无任何白屏/报错 toast。
2. 拖任意关节角度滑动条 → 3D 立即跟随（<0.5s）；拖连杆尺寸滑动条 → 模型重建+3D 刷新。
3. 「重置位姿」→ 全关节归零、脚底贴地；「重置默认」→ 恢复示例源码。
4. 质心读数显示 Σm、COM(x,y,z)、**距脚底高**、精度 mm；稳定/不稳定用绿/红圆点。
5. 摆一个明显“劈腿张臂”姿势 → 支撑多边形变红“不稳定”；恢复站姿→绿“稳定”。
6. 窄窗口 → 布局单列可滚动；页面本体不横向滚动。

### 12.2 工程验证（curl 可复现）
1. `GET /api/meshcat` → `{"ok":true,"url":...}`。
2. `GET /` 与静态资源 → 200。
3. `POST /api/scene`（含 links+com+feet+balance）→ 200 `{"ok":true,"url":...}`；改 full 重建不报错；
   **连续推送**（同一集合改 pos）只用 `set_transform`，不重建。
4. `POST /api/npz_files` → 列 5 个 npz；`POST /api/npz_parse`(Pikachu-Transition-Down) →
   `n=316, fps=50`，frame0 两 `arm_roll` ≈ **−1.571**（手臂垂下，验 bias）。
5. 数值核验：对给定几何，`systemCOM` 总质量 = Σρ·尺寸积，质心≈visual origin 加权；
   `footZ` = 全模型最低 box 角点；T-pose 下 T0Z=+0.161（当前几何）→ 脚底=0。
6. 平衡判定对固定姿势有稳定/不稳定两种可复现结果（切换边界≈2mm）。
7. 后端在**系统 python**（无 meshcat）启动时：`/api/scene|meshcat|export_mjcf` 返回 400 文案提示，
   纯静态页浏览不受影响。
8. 关闭浏览器等进程后 server 可重启、meshcat 单例重建，无端口/僵尸冲突。

---

## 13. 建议的重构拆分（若允许改结构）

- 单体 HTML → 拆 `index.html / style.css / app.js / fk.js / balance.js / npz.js`（或 webpack 打包）。
- server 拆分：`server.py`（路由）+ `meshcat_vis.py` + `npz_load.py` + `export_mjcf.py`，模块化 + 单测。
- 前端加单元可测的纯函数层（FK/COM/凸包/贴地）——这些是本项目最值得回归验证的核心。
- API 加版本前缀 `/api/v1/*` 便于演进；/api/scene 加 `seq` 序号丢弃过期帧（防抖竞态）。

---

## 附：启动命令
```bash
conda activate mocap
cd urdf/robot/Pikachu_links
python pikachu_link_tuner_server.py --port 8123
# 浏览器打开 http://127.0.0.1:8123/pikachu_link_tuner.html
# 其它参数: --dir <静态根> --host 127.0.0.1 --npz-dir <动作库目录> --mocap <python>
```