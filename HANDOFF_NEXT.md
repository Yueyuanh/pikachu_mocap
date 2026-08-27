# HANDOFF_NEXT — 交接给下一个 Claude 对话

**日期**: 2026-08-28
**项目**: `/home/finnox/Pikachu/PikachuRobot/pikachu_mocap`（分支 main，git 未提交）

---

## 1. 环境与启动

- **Python**: `/home/finnox/miniconda3/envs/mocap/bin/python`（PySide6 / urdfpy / meshcat / yaml 齐全）。
- **测试**（无显示环境）：`QT_QPA_PLATFORM=offscreen PYTHONIOENCODING=utf-8`
- **启动 GUI**: `cd /home/finnox/Pikachu/PikachuRobot/pikachu_mocap && python Pikachu_Retarget.py`
- URDF 默认: `urdf/robot/Pikachu_V025/urdf/Pikachu_V025_flat_21dof.urdf`
- NPZ 默认目录: `/home/finnox/Pikachu/PikachuRobot/pikachu_playground/mjlab/src/mjlab/mocap/npz`（可用环境变量 `PIKACHU_MOCAP_NPZ` 覆盖）

---

## 2. 已完成（可编译、离屏跑通）

| 文件 | 内容 |
|------|------|
| `Pikachu_Retarget.py` | 三模式 GUI。**Bone**（FK 直控 20 骨 x/y/z）、**URDF**（21 关节）+Meshcat、**NPZ**（回放）。含连接状态 label、`Sync to Blender`(皮肤)+`Drive Blender URDF` 双开关、底部两映射面板(Reload)、npz 文件选择+播放暂停+倍速0.05-5x+进度条。 |
| `retarget_map.yaml` | 21 关节 → 皮肤骨映射（bone/axis/sign/bias/limit），可编辑。 |
| `blender_urdf_map.yaml` | URDF → Blender 内 URDF 映射。**⚠️ 当前是占位（bone 名误用皮肤骨名，未核实真实骨架）**，见任务 2。 |
| `retarget.py` | 纯逻辑 `load_retarget_map` / `apply_retarget_rad` / `rad_to_deg` 等，无 bpy。 |
| `addon/blender_joint_server/` | 已加 `rig_sync.get_scene_info/set_urdf_joint/set_urdf_pose`；消息 `request_scene`/`set_urdf_pose` 分支；`__init__` 自动注册 timer + URDF 目标骨核对；`server.py` 新客户端替换旧连接。zip 已重建。 |

### 数据流
- **URDF 滑块** → `urdf_robot.set_joint` + meshcat 更新 → `apply_retarget_rad`(retarget_map) → `set_pose`
- **NPZ 每帧行** → `npz_row_to_urdf` → URDF → 映射上行
- 皮肤走 `set_pose`（retarget_map.yaml）；Blender 内 URDF 走 `set_urdf_pose`（blender_urdf_map.yaml），独立开关。

---

## 3. 关键实现细节（改这里先读）

### NPZ 列序（= twin_server.py 的 PIKACHU_JOINT_NAMES，14 列，弧度）
```
0  left_hip_pitch      6  right_hip_pitch
1  left_hip_roll       7  right_hip_roll
2  left_hip_yaw        8  right_knee
3  left_knee           9  right_ankle
4  left_ankle         10  left_arm_pitch
5  right_hip_yaw  →   11  left_arm_roll     (right=π/2−v, left=−π/2−v)
                     12  right_arm_pitch
                     13  right_arm_roll
```
> ⚠️ **10-dof（纯腿）容错**：`joint_pos` 只有 10 列时，**只映射腿部前 10 个关节，不传 arm**；且**不做** arm_roll 90° 偏置（因为 11/13 索引不存在）。已按 twin_server.py 语义实现：`npz_row_to_urdf(row)` 中 `col >= len(row)` 自动 break。10 列文件全部在 `pikachu_walk_base_npz/` 子目录。

### npz 结构
`joint_pos`(T×14 或 T×10) 弧度 + `fps`(1,) + `body_pos_w`(T,Nbody,3)/`body_quat_w`(T,Nbody,4)`。

### 已知坑
- **urdfpy 在 numpy≥1.24 坏** → GUI 顶部内置 numpy 别名垫片（在 URDF import 前）。
- npz 加载校验 `shape[1] >= 6`，<14 时打印提示仅映射腿部。

---

## 4. ❗你接下来的任务

1. **请用户在 Blender 重装/重载 `addon/blender_joint_server.zip`**（否则 socket 上无新命令）。服务器端口 9999。
2. 对已运行的 Blender，`request_scene` 拉取真实场景（objects/armatures/bones 含 head/tail 坐标）。**找出 Blender 里导入的 URDF 骨架对象名和其骨名**（区别于皮肤 rig 的 `upper_arm_fk.*`）。
3. 用真实骨名+坐标**重写 `blender_urdf_map.yaml`**（armature 名 + 每个 URDF 关节映射到该骨架的 bone/axis/sign/bias）。
4. 端到端验证：npz 播放时皮肤 rig 与 Blender 内 URDF 都动。
5. 可选：Bone 模式目前只发 `set_pose` 到皮肤；确认是否也同步到 URDF 骨架。

### 探测命令（复制即用）
```bash
cd /home/finnox/Pikachu/PikachuRobot/pikachu_mocap
/home/finnox/miniconda3/envs/mocap/bin/python -c "
import socket, time
s = socket.create_connection(('127.0.0.1', 9999)); s.settimeout(5)
s.sendall(b'{\"type\":\"request_scene\"}\n')
time.sleep(1.5)
try:
    d = s.recv(65536); print(d.decode(errors='replace'))
except Exception as e:
    print('no reply (addon 未重载？):', e)
"
```
> 返回空/超时 → addon 未重载，回到任务 1。

---

## 5. 快速冒烟（改完仓库自检）
```bash
/home/finnox/miniconda3/envs/mocap/bin/python -m py_compile Pikachu_Retarget.py retarget.py

# 离屏起 GUI + 10-dof npz + reset 验证
QT_QPA_PLATFORM=offscreen PYTHONIOENCODING=utf-8 \
  /home/finnox/miniconda3/envs/mocap/bin/python -c "
import Pikachu_Retarget as M
from PySide6.QtWidgets import QApplication
import sys
app = QApplication(sys.argv); w = M.RetargetStudio()
print('urdf joints:', len(w.urdf_joint_widgets_list), 'bones:', len(M.DIRECT_BONES))
"
```

---

## 6. 可选：把交接做成 artifact
如需把这份交接转成可分享的网页（claude.ai/artifacts），直接让我/下个对话用 Artifact 发布本 md。

---

## 7. ✅ 本轮（2026-08-28，blender-mcp）已完成 —— 任务 1~4

> 以下为接手进度记录，前面原始任务说明仍保留；若展示给下一个对话，可直接覆盖任务清单。

- **任务 1（addon 生效）**：已装 addon 磁盘已是新版（含 `request_scene`/`set_urdf_pose`），但 Blender 进程内存是旧版 → 已用 blender-mcp 触发 `stop_server()` + addon_disable/enable 重载。新代码确认加载，timer(`blender_loop`) 与 socket 9999 线程已起。`request_scene` 现返回完整 scene。
  - ⚠️ **若重启 Blender 后未自动 start**：`register()` 只注册 timer，`start_server()` 需手动（面板 Skeleton→Start 或直接 `server.start_server()`）。
- **任务 2（真实骨架）**：Blender 场景 4 个 armature —— `rig`(331,皮肤/set_pose)、`metarig`(51,人体参考)、`Pikachu_V025`(19,**URDF 导入,仅腿10 revolute、臂 .fixed、无 head**)、`pikachu_sample_links`(20,全 revolute、含单一 head)。**没有任何与 21dof 一一对应的骨架**（缺 `head_yaw/pitch/roll` 与 `elbow_ankle`）。
- **任务 3（重写 bm）**：按用户决策「不重建、复用 Blender 现有、缺的省略、腿>手>头」→ `blender_urdf_map.yaml` 改为 `armature: "Pikachu_V025"`，仅 10 个腿关节，axis:x sign:+1（逐骨核对：Pikachu_V025 腿骨长轴 == URDF 关节轴）。肘/头已省略。
- **任务 4（端到端）**：修了一个单位 bug —— `Pikachu_Retarget.py _push_urdf_if_needed` 原把角度×180/π 变度发给 `set_urdf_pose`，而 Blender 端按弧度用，会放大 57.3×；已改为直接传弧度。验证：手动 socket、GUI 完整路径、npz 第 0 帧（`Pikachu_Walk2_S1.npz`）均精确驱动 `Pikachu_V025` 腿骨，且同帧驱动皮肤 `rig`（set_pose）。

### 剩余
- **连续 npz 播放**：单帧链路已通，连续播放是 GUI 交互（QTimer），建议真机播放确认手感。
- **任务 5（可选）**：Bone 模式目前只 `set_pose` 到皮肤，未同步 URDF 骨架（`_push_skin_pose` 未发 set_urdf_pose）。

---

## 8. ✅ 本轮（2026-08-28，多 rig 多选 + reset 一致性）已完成代码，待 Blender 重启后验证

> 需求（用户原话）：「reset all之后blender中的urdf并没有reset,要始终保证meshcat中的urdf和blender中的urdf一致，然后因为我在blender中有多个实验的urdf，请你帮我在插件界面添加一个多项选择框，可以选择多个urdf rig」。
> 用户拍板：**多选框放 Blender 插件面板**（非 Qt）；**预配两个 rig**（Pikachu_V025 + pikachu_sample_links）一起支持。

- **blender_urdf_map.yaml**：重构为 per-armature —— `armatures: [ {name:"Pikachu_V025", joints:{…10腿}}, {name:"pikachu_sample_links", joints:{…10腿+8臂}} ]`。
  - 腿：两套同名同轴同 sign（axis:x sign:+1，骨长轴==关节轴）。
  - 臂（仅 sample_links）：`left/right_arm_pitch/roll/yaw_joint` 同 axis:x sign:+1；`left/right_elbow_ankle_joint` 映射到 `left/right_elbow_joint.revolute.bone`（此骨架骨名是 elbow 非 elbow_ankle）。
  - head：两套都省略（V025 无 head 骨；sample_links 仅单一 head_joint 骨，无法表示 head_yaw/pitch/roll 三维 → 按“缺则省略”、头优先级最低）。
- **Qt Pikachu_Retarget.py**：
  - `BlenderUrdfPanel.rebuild/_armature_list`：兼容新旧格式（armatures 列表 / armature+joints 单条）；面板显示套数+各自关节数。
  - `_push_urdf_if_needed`：改为对 map 里**每一套 armature** 各发一个 `set_urdf_pose`（仍受 Drive Blender URDF 开关 + 连接判定）。
  - 新增 `_build_pose_for_armature(joints, angles, zero)` 与 `_push_urdf_reset`。
  - **reset_all**：改成调 `_push_urdf_reset()`，**强推所有 map rig 到 rest pose（全关节 0→各轴 bias）**，不受 Drive 开关限制，解决“reset 后 blender urdf 不 reset”的根因（旧代码因 toggle guard + 空 pose 短路而不发）。
  - **勾选决策在哪端**：Qt 对 map 里所有 rig 全发；最终谁真的动由 **Blender 面板勾选过滤**（见下），从而保证 meshcat 与“被勾选的 Blender URDF” 一致（reset 同理）。
- **Blender addon（blender_joint_server）**：
  - 新增 `SKSERVER_URDFTarget` PropertyGroup + `Scene.skserver_urdf_targets` 集合；`_is_urdf_armature`（骨名含 `.revolute.bone`/`.fixed.bone`）+ `_sync_urdf_targets`（自动同步场景 URDF 骨架，缺省勾选）。
  - 面板 `Skeleton Server` 新增 “Drive URDF armatures (multi-select)” 段，每个候选一套 checkbox（`layout.prop(t,"enabled",toggle=True)`）。
  - `rig_sync.set_urdf_pose` 开头加 `_is_urdf_target_enabled(armature_name)` 过滤：**未勾选整骨架忽略**；未在集合里出现的名字默认放行（True，兼容旧版/面板未同步）。
  - 项目 zip 已重建（`addon/blender_joint_server.zip`），并已同步 `/home/finnox/.config/blender/5.2/scripts/addons/blender_joint_server/`。
- **验证状态**：py_compile 全过；离屏 Qt 确认解析出 2 套 armature（V025=10 关节、sample_links=18 关节）且换算/`zero` reset 冒烟通过；MCP 重载 addon 后 `scene.skserver_urdf_targets` 属性注册成功、`_sync_urdf_targets` 列出 (pikachu_sample_links,T)/(Pikachu_V025,T)。
- ⚠️ **唯一未完成：live socket 验证被旧 addon 残留端口阻塞**——原地 reload 时旧 server 线程未停、仍占 9999（`Address already in use`），新版 start_server 起不来。**需重启 Blender** 让新 addon 干净接管 9999，再做端到端（勾选/去勾选 + reset 一致性）。重启后若未自动 start：面板 Skeleton→Start。