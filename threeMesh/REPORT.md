# threeMesh/ —— URDF + SKin 随动预研报告

> 预研目标：回答「用 meshcat / three.js 能不能实现 **URDF 机器人** 与 **蒙皮 Pikachu(skin.glb)** 的随动」。
> 结论先行：**能，但要放弃 meshcat，改用纯 three.js 作为统一宿主**。本目录即是最小可运行验证。

- 日期：2026-08-28
- 位置：`threeMesh/`（浏览器端纯 JS，无需 Blender / 无需 Python 主程序）

---

## 0. 一句话结论

meshcat 体内没有 `SkinnedMesh`（只有静网格 + 刚性 `set_transform`），**做不了连续蒙皮**。
改用 **纯 three.js**：同一场景里左边加载 URDF FK 参考骨架（STL 壳逐骨）、右边加载 `skin.glb` 蒙皮，
一张滑块表把同一个关节角同时写给双方 —— **URDF 关节旋转 + 蒙皮 DEF 变形骨同步动**。已离屏验证通过。

---

## 1. 我做了什么

| 文件 | 作用 |
|------|------|
| `index.html` | 左侧操纵面板 + 右侧 3D 视口（`.gl` 画布） |
| `src/urdf_loader.js` | **纯 three.js 手写 URDF FK 加载器**：DOMParser 读 XML → 每 link 建 `THREE.Group` 挂 STL 壳 → 每 joint 建 `pivot+spin` 层级，转 `spin` = 转该 link 及全部后代。返回 `{root, joints(Map)}`，`joint.setAngle(rad)` 驱动 |
| `src/main.js` | 场景/相机/光照/OrbitControls；`GLTFLoader` 读 `skin.glb`；**CONFIG 随动表** 把 URDF 关节 ↔ 蒙皮 DEF 骨映射；滑块同时驱动两边 |
| `server.py` | 极简 `http.server`（端口 8767，`no-store`）。必须用 http 打开——ESM 模块被 `file://` 的 CORS 挡掉 |
| `assets/` | `urdf.urdf`(21 关节) + `skin.glb`(1.76MB，从 Blender 导出) |
| `meshes/` | 25 个 binary STL 视觉壳（URDF 内相对路径 `../meshes/*.STL` 引用） |
| `lib/` | 本地 three r0.160 + `OrbitControls/GLTFLoader`，`utils/BufferGeometryUtils.js`（GLTFLoader 的相对依赖） |

### 随动映射 CONFIG（最小占位，**待你细对**）
```js
{ urdf:'left_knee_joint',        bone:'DEF-shinL',       ui:'左膝 → DEF-shin' }
{ urdf:'left_ankle_joint',       bone:'DEF-footL',       ui:'左踝 → DEF-foot' }
{ urdf:'left_hip_pitch_joint',   bone:'DEF-thighL',      ui:'左髋pitch → DEF-thigh' }
{ urdf:'left_elbow_ankle_joint', bone:'DEF-forearmL',    ui:'左肘 → DEF-forearm' }
{ urdf:'left_arm_pitch_joint',   bone:'DEF-upper_armL',  ui:'左肩pitch → DEF-upper_arm' }
{ urdf:'right_knee_joint',       bone:'DEF-shinR',       ... }
{ urdf:'right_arm_pitch_joint',  bone:'DEF-upper_armR',  ... }
{ urdf:'head_pitch_joint',       bone:'head',            ... }
```
> ⚠️ 骨名用的是 **GLTFLoader 剥点后的名字**（`DEF-shin.L` → `DEF-shinL`）。
> 每个名字在 4 套蒙皮里各有副本（`__bonesTotal=1844`），`main.js` 按名驱动**全部副本**，所以实际绑定身体的哪套都会动。

---

## 2. 关键发现（这部分是你后面改 map 最需要的）

1. **蒙皮主体 `Cube.003` 绑在 skin[0]=`rig`(Blender Rigify 生成)，不是 metarig**。
   变形靠 **Rigify 的 `DEF-*` 链**（`DEF-thighL/DEF-shinL/DEF-footL/DEF-upper_armL/DEF-forearmL`…）。
   metarig 的人类骨名（`thigh.L/shin.L`）**不驱动身体**——之前以为是它，实际不是。

2. **GLTFLoader 会把导出骨名里的 `.` 剥掉**：`DEF-shin.L` → `DEF-shinL`，`head`保留。映射 bone 名要按无点版。

3. **驱动蒙皮骨 = 直接转 `DEF-*` 骨即可**（绕开 Rigify 的 IK/FK/MCH 层）。three.js 无约束，
   glTF 是纯 rest pose，转 DEF 骨就是干净的局部旋转 → 网格正确变形。

4. **铰链轴可自动求**：`main.js` 在运行时用「父骨世界点 / 本骨世界点 / 最远子骨世界点」三点的
   两段叉积计算出该关节的转轴，再转回骨局部系。所以 CONFIG 基本只需写 `urdf↔bone` 名，**方向基本不用手调**。
   实测左膝铰链轴 local `(-1,0,0)`（横向）→ 弯膝正确。

5. **蒙皮有 4 套骨架**（每根 skinned mesh 各引一份 rig）。同名骨多副本 → 驱动时要全带上（已处理）。

---

## 3. 纯 three.js 加载 URDF 时踩的坑（探索记录）

| 坑 | 现象 | 解决 |
|----|------|------|
| **ESM 模块 CORS** | `file://` 打开 module 直接不跑（`import` 被拒） | 必须走 http（给了 `server.py`） |
| **importmap 子路径** | addons 平铺在 `lib/` 根，`three/addons/controls/...` 404 → 模块整段不加载 | 改 importmap 只留 `"three"`，addons 用绝对 URL `/lib/OrbitControls.js` 导入 |
| **GLTFLoader 相对依赖** | `import '../utils/BufferGeometryUtils.js'` 不走 importmap | 拷到 `utils/` 命中相对路径 |
| **STLLoader 误判 ASCII** | Open3D 二进制 STL 头被 three 的自动探测当 ASCII，`parseInt` 出 16GB → 爆内存 | **手写二进制 STL 解析器**（80 字节头 + uint32 计数 + 每三角 50 字节），顺带读 Open3D 5-bit BGR 顶点色 |
| **URDF 相对路径** | urdf 里写 `../meshes/*.STL`（相对 urdf 所在目录的兄弟），放错层 → 404→把 HTML 当 STL 解析 | `meshes/` 放成 `assets/` 的兄弟（`/meshes/`） |

---

## 4. 怎么跑/怎么看

```bash
cd threeMesh
python server.py          # 然后浏览器开 http://127.0.0.1:8767/
```
离屏验证（无头）：
```bash
# 装配验证：读页面 JS 状态 `__boot/__dbg`，不读图
# 驱动验证：把左膝滑块拉到 0.6，断言 URDF 角=0.6 且 DEF-shin 骨四元数变化
```
（验证脚本在 `/tmp/verify_threemesh.py`、`/tmp/drive_threemesh.py`，用 mocap 环境 PySide6 QWebEngine。）

---

## 5. 限制 & 下一步

- **这是「预研」，不是交付**：CONFIG 只映射了 8 组「示意」，完整 21 关节的 `bone/axis/sign/bias/limit`
  由你之后在我的 `blender_urdf_map.yaml`/`retarget_map.yaml` 经验基础上细对。
- **轴方向/符号**：`main.js` 自动算轴，但 `sign` 默认 +1，个别骨骼若反了改 `target.sign` 或在 CONFIG 加 `sign` 字段即可。
- **下一步候选**：
  1. 把 Qt `Pikachu_Retarget.py` 的 socket 数据接到这个页面（WebSocket 或 QtWebChannel），让真实 NPZ 动画驱动蒙皮；
  2. 完整 21 关节 CONFIG；加 `bias/limit`；
  3. 把左/右肩手臂的 IK 姿势对正（和主程序 arm_roll ±90° 同一套约定）后再映到 `DEF-*`；
  4. 性能：25 个 STL 约 40MB 三角壳偏重，正式版可简化视觉或换 glTF。