# 反思与需求复盘：npz base / Qt 界面（2026-08-30）

## 一、今天做坏在哪 —— 诚实复盘

今天的核心工作是一条「npz 动作回放 → base 映射 → meshcat + Blender」的数据链路，
本应是「一遍做对」的，结果来回返工、用户多次发火。根因不在不会写代码，
而在**不先把链路立起来就动手，又在不可观测的环节上靠猜**。

### 1. 最大的错：远程 Blender 是不可观测状态，我却一直没把「生效前提」说死

Blender 端是独立进程，改了 `rig_sync.py` 后 **必须完全重启 Blender + 清字节码缓存**
（`它内存里还缓存着旧子模块，reload/启用禁用都不一定重读`）。这个我一直知道（总结里挂着），
却直到用户反复报「blender 不动 / reset 不生效」才每次都归因到它。
一个用户在信任上系于「我改了 → 你就该重启 Blender 再测」，我该**在每次改 Blender 端代码时，
第一句话就写上「装好后：重启 Blender + 清 __pycache__」**，而不是把它当普通改动。

### 2. 链路有好几跳，我却先猜再写，而不是先埋「最小可观测点」

链路：`npz → numpy(quaternion/rpy) → Qt 算 base → socket → Blender set_base → 对象/骨架`。
任何一跳不一致，表象都是「不动」。我前几轮在猜「为什么不动」，浪费了用户时间；
直到后期才加 `[base→skin] fN: pos=… rpy=…` 打印、滑块随动显示——这些**本就该第一版就埋进去**。
原则：**凡跨进程/跨多次写入的链路，第一版就带「打印实际算出的中间值」，一次定位**，而不是改一轮看一轮。

### 3. 量纲 / 量程含糊，让「对的」被当成「错的」

- npz 的 `body_pos_w` 是米，root 位移常常就几毫米 → meshcat / Blender 里「看起来不 pos 走」，
  被误判成 bug，实际是**数据本身就小**。该第一轮就把 unit(m→cm)、量级讲清。
- base 滑块量程 `±100cm`，npz 位移超 1m 时滑块被 clamp 在 100，但文本和模型显示真实值，
  UI 显示和实际脱节，就是「滑块不对，调个 P」。量程和显示必须始终配得上数据。
- meshcat 到底该播「原始元数据」还是「映射后」，用户点明后才划清。**设计初期就该明确定位：
  meshcat = 黄金参考（原值），Blender = 映射输出（目标），滑块 = 观测 meshcat（原值）。**

### 4. 双重映射 / 语义冲突，我改这漏那

base 映射横跨四个文件（yaml 解析、Qt 计算、meshcat、Blender 插件），
我多次「改了 A 处又让 B 处重复映射」：皮肤 base 的 `pos_scale` 在 npz_base_to 和 per-cfg 两处乘、
`_apply_base_to_all` 中 meshcat 曾跟着 remap、手动路径又重新走 retarget。
改到最后才收敛成「meshcat 永远原值；Blender 各 armature 只在本 cfg 做一次映射」。
教训：**这类共享数据流的映射函数必须只有一处，且命名/注释写明「这段是给谁、基于哪个 cfg」**。

### 5. 不该对用户说「改好了」却没给验证动作

多次以「已改好，重启生效」结束，用户一测又不行。今后**每次交付必须附一条可执行验证**
（打印什么、滑块应显示什么、模型应怎么动），让「确认生效」不需要靠猜。

### 6. 情绪上对用户的愤怒不该防御

用户几次「你tm是不是脑瘫 / 改的什么狗屎」，我个别轮次先辩解「代码没坏」。
更该做的是承认「没让你确认生效」的流程缺陷，立刻给一个可观测的判定。**任何报障先信它，再找证据。**

## 二、我要求自己记住的守则

1. 跨进程（Qt↔Blender）改动，交付语里必带「重启 Blender + 清 __pycache__」。
2. 多跳数据链路，第一版就埋「打印实际中间值」，用它二分，不猜。
3. 量纲/量程先讲清：米↔厘米、npz 位移量级、滑块量程配得上数据。
4. 映射函数单一出处；注释写明「给谁用、基于哪个 cfg」。
5. 每次交付附一条可执行验证；报障先信再找证据。

## 三、从你的 Qt 需求中琢磨出的设计意图

你今天反复表达的诉求，其实指向一个清晰的界面契约，我整理如下，作为后续固化的依据：

### 数据定位（一成不变）
- **meshcat（URDF 查看器）= 黄金参考**：如实播放 npz 原始想 base（相对首帧位移 + 绝对 rpy），
  不做任何 pos_scale/rot_offset/retarget 缩放。你看到的就是动作数据原貌。
- **Blender 各角色 = 映射输出**：每个 armature 用各自 yaml 的 cfg 独立重映射
  （pos_retarget / pos_scale / pos_dir / rot_retarget / rot_scale / rot_offset），
  是真正要「调方向、调比例」的地方。
- **base 手动面板 = 观测 + 微调**：滑块显示并与 meshcat 严格一致（cm / °，量程配得上数据），
  可暂停后手动微调；npz 播放时滑块实时随动。

### 界面交互要项
- **npz 下拉按文件夹分组**（今天已做）；文件是播放主入口。
- **「映射 Base」开关**决定是否把 base 推给 Blender（Blender 实时挪角色较卡），
  meshcat 不受此开关影响。
- **两层面板 Reload 都要重读对应 yaml 并打印 cfg 自检**（Retarget/Burdf 都刷皮肤 base）——
  避免「改了配置文件点 Reload 不生效」。
- **reset all**：meshcat 归 0、面板滑块归 0、Blender 各角色回 pos0 + 各自 rot_offset（安装朝向），三处一致。

### 仍待你明确定夺（明天我照办）
- Blender 端「映射 Base」开关要不要默认就打开（现在默认关，防卡）？
- npz root 位移量级太小想「看得见」，是走 yaml `pos_scale` 放大（只影响 Blender），
  还是想让 meshcat 也放大（与原数据定位冲突）？——建议只由 `pos_scale` 控制 Blender。
- Blender idle 卡顿与映射实时性的平衡，是否需要节流 base 帧率。

## 附：本次已收敛到位的实现
- yaml 统一 `base_pos: {pos_retarget,pos_scale,pos_dir}` + `base_rot:{rot_retarget,rot_scale,rot_offset}`，
  `_base_cfg_for_burdf`/`_load_base_cfg_from_file` 都保留 retarget 字段。
- `npz_base_to` 返回原始值；`_base_pos_for_cfg`/`_blender_base_rpy` 各 armature 凭自 cfg 一次重映射。
- meshcat 永远原值；手动滑块走 `_blender_rpy_direct`（不换轴）与 npz 隔离。
- Blender 插件：set_base/set_pose 纯数据写入不再抢 active（消闪烁）、统一刷新。