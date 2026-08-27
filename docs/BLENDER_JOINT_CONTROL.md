# Pikachu 关节控制指南 —— 给其他 Agent 的工作文档

> 本文档描述 **Pikachu_Mocap.py 如何通过 `blender_joint_server` socket 控制 Blender 里的 Pikachu 骨架（armature `rig`）**，以及头顶的三套骨骼结构、控制点与数值约定。任何想程序化驱动皮卡丘关节（读姿态、设姿态、导出骨架）的 agent 先读这篇。

---

## 0. 一句话总结

GUI 在 **`localhost:9999`**（不是 9876、不是 9877）开一个**换行分隔的 JSON TCP 连接**，往 Blender addon `blender_joint_server` 发命令；Blender 侧把命令应用到 `rig` 骨骼的 `pose.bones[<name>].rotation_euler`（**XYZ 欧拉，度**）。每个可控关节都写死在 `addon/config/blender_joint_config.yaml`。

---

## 1. 三套骨骼结构 —— 千万别混淆

这个项目里有三套"骨骼"，名称/用途完全不同：

| 结构 | 文件 | 内容 | 角色 |
|------|------|------|------|
| **A. Rig 骨骼树** | `addon/scripts/pikachu_skeleton.yaml` | Blender 骨架 `rig` 的 **54 根骨骼**完整层级（DEF-/MCH-/ORG-/tweak_/FK/IK 前缀） | 模型真实骨架 |
| **B. 可控关节（控制点）** | `addon/config/blender_joint_config.yaml` | **21 个能设角度的骨骼** + 每轴度极限 | **agent 唯一需要操作的集合** |
| **C. 人体关键点** | `pose/MediaPipe/pikachu.yaml` + `addon/scripts/pikachu_pose_skeleton.yaml` | MediaPipe **33 个人体关键点**（NOSE/…/LEFT_HIP/…/FOOT_INDEX） | 动捕输入，映射到 B |

- **调试/驱动皮卡丘 = 用 B（控制点），对照 A（层级）**。
- C 是摄像头动捕的输入侧，agent 一般不直接碰。

---

## 2. 端口与连接（重要！）

| 端口 | 是谁 | 用途 |
|------|------|------|
| **9999** | `blender_joint_server`（Skeleton Server） | **本系统的控制通道** ← 只连这个 |
| 9876 | 自定义 `mcp_to_blender_server` | 另一套纯 Python-exec 服务，别混淆 |
| 9877 | 标准 blender-mcp addon | Claude 的 Blender MCP 工具，非本系统 |

**协议**：一条消息一行，JSON + `\n`。Python `BlenderClient.send()` 的实现：

```python
self.sock.sendall((json.dumps(data) + "\n").encode())
```

---

## 3. 命令协议（核心）

通过 `addon/blender_joint_server/server.py`（TCP server）+ `rig_sync.py`（处理 `handle_message`）实现。共 5 个命令：

### 3.1 设单根骨骼单轴
```json
{"type": "set_joint", "bone": "upper_arm_fk.L", "axis": "y", "angle": -30}
```
Blender: `rig.pose.bones["upper_arm_fk.L"].rotation_euler.y = radians(-30)`

### 3.2 设整帧姿态（**实时动捕主路径**）
```json
{"type": "set_pose", "pose": {
    "head":          [0, 5, 0],
    "upper_arm_fk.L":[30, 0, -10],
    "forearm_fk.L":  [0, -45, 0],
    "tail":          [0, 20, 0]
}}
```
`pose` 是 `{bone: [x, y, z]}`，三个值都是**度**。Blender 逐骨 `rotation_euler = (rad(x), rad(y), rad(z))`。

### 3.3 读当前姿态
```json
{"type": "request_pose"}
```
→ 返回 `{"type":"pose", "data": {"<bone>": [x,y,z], ...}}`（全部骨骼，度）。

### 3.4 读骨骼树
```json
{"type": "request_bones"}
```
→ 返回 `{"type":"bones", "data": [{"name": "...", "parent": "...|null"}, ...]}`。

### 3.5 读骨骼变换（导出骨架用）
```json
{"type": "request_transforms", "bones": ["upper_arm_fk.L", "tail"]}
```
→ 返回 `{"type":"transforms", "data": [{"name","parent","head","tail","matrix","local_matrix"}, ...]}`，带请求骨骼的所有祖先（`get_bone_transforms` 会沿 parent 爬升补全）。world matrix = `arm.matrix_world @ bone.matrix`。

### 数值约定
- **角度 = 度**；Blender 内部转弧度（`radians()`）。
- 欧拉顺序 **XYZ**（`bone.rotation_mode='XYZ'`）。物理轴≠UI 轴，正负号以 Blender 编辑为准。
- GUI `set_pose` 前还会 **clamp 到 config 极限 + round 成整数**；外部驱动建议同样姿势。

---

## 4. 可控关节清单（B：控制点）+ 每轴度极限

来源 `addon/config/blender_joint_config.yaml`，格式 `x: 0,(-M,M)`。

| 骨骼名 | X（°） | Y（°） | Z（°） | 对应身体部位 |
|--------|---------|---------|---------|--------------|
| `head` | -20..20 | -20..20 | -20..20 | 头 |
| `neck` | -10..10 | -10..10 | -10..10 | 脖子 |
| `chest` | -10..10 | -10..10 | -10..10 | 胸口 |
| `torso` | -10..10 | -10..10 | -10..10 | 躯干 |
| `hips` | -10..10 | -10..10 | -10..10 | 髋 |
| `shoulder.L` / `shoulder.R` | -10..10 | -10..10 | -10..10 | 肩 |
| `ear.L` / `ear.R` | -90..90 | -90..90 | -90..90 | 耳朵 |
| `upper_arm_fk.L` / `.R` | -90..90 | -90..90 | -90..90 | 上臂（FK） |
| `forearm_fk.L` / `.R` | -90..90 | -90..90 | -90..90 | 前臂（FK） |
| `hand_fk.L` / `.R` | -90..90 | -90..90 | -90..90 | 手（FK） |
| `foot_ik.L` / `.R` | -50..50 | -50..50 | -50..50 | 脚（IK） |
| `toe.L` / `.R` | -50..50 | -50..50 | -50..50 | 脚趾 |
| `tail` | -90..90 | -90..90 | -90..90 | 尾巴 |

> 注：`addon/scripts/joint_config.yaml` 是**另一份更保守**的配置，只含 6 个 FK 手臂骨骼（±180）。GUI 真正用的是 **`addon/config/blender_joint_config.yaml`**（`CONFIG_PATH`）。改极限请改这份。
>
> 手指骨（`f_index.01.*`、`thumb.*`、`f_middle.*`…）当前**被注释掉**，未启用。

---

## 5. Rig 骨骼树（A：54 根）

从 `pikachu_skeleton.yaml` 解析出的完整层级：

```
root
 ├─ DEF-spine → .001 → .002 → .003 → .004 → .005 → .006
 │    ├─ ear.L / ear.R
 ├─ tail (parent=DEF-spine.002)
 ├─ torso
 │    ├─ chest, hips
 │    ├─ MCH-spine.001 → MCH-spine → tweak_spine → ORG-spine
 │    │    ├─ ORG-thigh.L → ORG-shin.L → ORG-foot.L → MCH-toe.L → toe.L
 │    │    └─ ORG-thigh.R → ORG-shin.R → ORG-foot.R → MCH-toe.R → toe.R
 │    └─ MCH-spine.002 → MCH-spine.003
 │         ├─ MCH-ROT-neck → neck → MCH-ROT-head → head
 │         ├─ tweak_spine.003 → ORG-spine.003
 │         │    ├─ ORG-shoulder.L → MCH-upper_arm_parent.L → upper_arm_fk.L → forearm_fk.L → MCH-hand_fk.L → hand_fk.L
 │         │    ├─ ORG-shoulder.R → MCH-upper_arm_parent.R → upper_arm_fk.R → forearm_fk.R → MCH-hand_fk.R → hand_fk.R
 │         │    ├─ shoulder.L / shoulder.R
 └─ MCH-foot_ik_socket.R → foot_ik.R
└─ MCH-foot_ik_socket.L → foot_ik.L
```

命名规范（帮助辨识）：
- **`DEF-`** = 形变骨（蒙皮权重用），spine 链是 DEF。
- **`MCH-`** = 机制/控制骨（spine 旋转、foot_ik_socket、toe、hand_fk、upper_arm_parent）。
- **`ORG-`** = 归位/骨架原始骨（ORG-spine、ORG-thigh/shin/foot、ORG-shoulder）。
- **`tweak_`** = 微调骨。
- **`*_fk`** = FK（正向运动学，手臂直接转）。
- **`foot_ik` / `toe`** = 腿部 IK 方案。
- `root` 无父，是所有 DEF/torso 的根；`MCH-foot_ik_socket.*` 单独、parent=null（IK socket 挂世界）。

---

## 6. MediaPipe 关键点 → 关节的对应（C，仅供了解）

hub 在 `transfer/transfer.py`（`map_humanoid_to_pikachu`）；URDF 侧映射表见 `transfer/Transfer.md`：

| 部位 | URDF joint | 轴 | MediaPipe 源 |
|------|-----------|----|--------------|
| 头 | head_pitch/yaw/roll_joint | Y/Z/X | HEAD 区域关键点 |
| 左臂 | left_arm_pitch/roll/yaw_joint | Y/X/Z | LEFT_SHOULDER |
| 左肘 | left_elbow_joint | Y | LEFT_ELBOW |
| 左髋 | left_hip_pitch/roll/yaw_joint | Y/X/Z | LEFT_HIP |
| 左膝 | left_knee_joint | Y | LEFT_KNEE |
| 左踝 | left_ankle_joint | X | LEFT_ANKLE |
| … | （右侧对称） | | |

动捕主循环（`update_camera`）：
```
摄像头帧 → MediaPipe → HumanoidPoseData → 映射成 pikachu 角度
  → 逐骨 clamp 到 config 极限 → round 成 int
  → 与 _last_sent_angles 比较（差>=sync_threshold 才重发）做去重
  → 过滤 bone_sync 勾选 → client.set_pose({bone:[x,y,z]})
```

---

## 7. 直接程序化驱动皮卡丘（最小可用示例）

不需要跑 GUI，连上 9999 就能驱动：

```python
import socket, json

s = socket.socket(); s.connect(("127.0.0.1", 9999))

def send(payload):
    s.sendall((json.dumps(payload) + "\n").encode())

# 举手
send({"type":"set_pose", "pose":{
    "upper_arm_fk.L": [45, 0, 0],
    "forearm_fk.L":   [0, -90, 0],
    "tail":           [0, 30, 0],
}})

# 读回姿态
send({"type":"request_pose"})
buf=b""
while True:
    line = s.recv(4096).decode(errors="replace")
    buf += line
    if "\n" in buf:
        msg, buf = buf.split("\n",1)
        print(json.loads(msg)); break
```

---

## 8. Clamp / 去重 / 同步逻辑（GUI 的做法，供对齐）

`Pikachu_Mocap.py` 关键行为：
- `on_axis_change(bone, axis, val)`：手动拖单轴 → **`set_joint(bone, axis, val)`**（`JointPanel._make_on_change` → line ~535）。
- 实时动捕帧：走 `set_pose` 整帧（见上 §6）。
- `reset_all()`：所有 bone_order 骨清零 → 逐骨 `set_joint(name, axis, 0)` + URDF 归零；并清 `_last_sent_angles`。
- 去重：`sync_threshold`（默认某整度数差值）内不重发，避免刷屏。
- 每个骨骼有独立同步开关 `bone_sync`；关掉就不进 `set_pose` payload。

---

## 9. 常见坑（踩过）

1. **端口**：只连 **9999**。9876 是自定义 Python-exec 服务、9877 是 blender-mcp，都跟本链路无关，别发 `set_pose` 过去。
2. **骨骼名必须精确匹配** `rig.pose.bones`。少了 `.L/.R`、`.fk` 大小写等都会静默被跳过（`set_pose` 里 `if bone is None: continue`）。
3. **角度是度**。直接传弧度会错得离谱；Blender 端会自动 `radians()`。
4. **XYZ 欧拉**：三个轴顺序固定，是 Blender 的 XYZ mode，不是自定义轴序。
5. **`rig_sync` 需要 Pose 模式**：`ensure_pose_mode` 会自动切，但若曾有手动 break 需重开 addon。
6. 改极限改 `addon/config/blender_joint_config.yaml`，不是 `joint_config.yaml`。
7. Blender addon 用 `addon/blender_joint_server.zip` 安装。

---

## 10. 文件地图

| 文件 | 作用 |
|------|------|
| `Pikachu_Mocap.py` | GUI 主程序（控制逻辑全在这） |
| `addon/blender_joint_server/{server,rig_sync,__init__}.py` | Blender 端 socket server + 命令处理（对端协议真身） |
| `addon/config/blender_joint_config.yaml` | **可控关节 + 极限（改这里）** |
| `addon/scripts/pikachu_skeleton.yaml` | 54 骨 rig 层级 + rest 矩阵 |
| `addon/scripts/pikachu_pose_skeleton.yaml` | MediaPipe 关键点位置（含头/眼/手关节） |
| `addon/scripts/joint_config.yaml` | 旧 FK 手臂极限（保守，GUI 未用） |
| `pose/MediaPipe/pikachu.yaml` | MediaPipe 33 关键点定义 |
| `transfer/Transfer.md` | URDF joint ↔ 人体关键点映射表 |
| `urdf/` | URDF 机器人模型 + 查看器（meshcat） |
| `assets/` | .blend + 毛发烘焙缓存（blendcache_*） |

> 要导出带材质/毛发/骨骼的模型：Blender 打开 `assets` 里的 .blend，导出时勾选 Geometry or Data · Materials · Modifiers · Armature。毛发是 `blendcache_pikachu_role` 里的烘焙缓存，需先模拟(bake)再导出或带缓存打包。