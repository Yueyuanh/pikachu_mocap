# Pikachu 27DOF 尺寸契约

`pikachu_sample_links_27dof.xacro` 是 27DOF 简化 box-link 模型的源文件。模型采用 SI 单位、右手坐标系：`x` 向前、`y` 向左、`z` 向上；工程尺寸统一在关节角全为 0 的 T-pose 下测量。

## 唯一真相源

只用以下 14 个 `measure_*` 属性表达外部实测。关节位置和 box 长度必须由它们派生，不应在宏或 joint 中再次写相同常量。

| ID | 属性 | 定义 | 基准值 |
|---|---|---|---:|
| M01 | `measure_ear_root_height` | `base_link` 原点到耳根轴的 z 差 | 449 mm |
| M02 | `measure_ear_root_half_width` | 中心 z 轴到单侧耳根轴的绝对 y 距离 | 116 mm |
| M03 | `measure_head_pitch_height` | `base_link` 原点到 `head_pitch` 轴的 z 差 | 219 mm |
| M04 | `measure_shoulder_half_width` | 中心 z 轴到共点肩 pitch/roll 轴的绝对 y 距离 | 116 mm |
| M05 | `measure_arm_plane_offset_x` | 腿关节平面到肩关节平面的 x 差 | 83 mm |
| M06 | `measure_upper_arm_length` | 共点肩轴到 elbow 轴的欧氏距离 | 100 mm |
| M07 | `measure_forearm_length` | elbow 轴到 `elbow_link` box 远端 | 66 mm |
| M08 | `measure_hip_roll_half_width` | 中心 z 轴到 hip-roll 轴的绝对 y 距离 | 100 mm |
| M09 | `measure_torso_length` | hip-pitch 轴到 arm-pitch 轴的 z 差 | 166 mm |
| M10 | `measure_thigh_length` | hip-pitch 轴到 knee 轴的 z 差 | 83 mm |
| M11 | `measure_shank_length` | knee 轴到 ankle 轴的 z 差 | 86 mm |
| M12 | `measure_foot_length_x` | `ankle_link` box 的 x 边长 | 93 mm |
| M13 | `measure_tail_axis_height_from_ankle` | ankle 轴到 tail-pitch 轴的 z 差 | 166 mm |
| M14 | `measure_tail_center_offset_x` | 腿关节平面到 tail box 中心的绝对 x 距离 | 143 mm |

这里把用户定义的“尾宽”解释为侧视图中的前后偏置，而不是尾巴 box 的横向厚度；物理横向宽度由独立属性 `tail_wid` 控制。

## 调参和验收

1. 从仓库根目录运行 `python3 urdf/robot/Pikachu_links/pikachu_link_tuner.py`。
2. 优先修改“工程尺寸”分组。页面每次重建模型后，都会从关节帧和 box 几何反算 M01–M14。
3. “实算 / 目标 / 偏差”应全部为通过；默认容差属性 `tune_measurement_tolerance=0.0005` m。
4. 切换正面和侧面图核对测量基准。关节姿态可用于运动范围、碰撞趋势和支撑检查，但不会改变零位尺寸验收。
5. 一键导出会同时保存 Xacro、展开 URDF、二维图纸、link 尺寸 JSON 和 validation JSON；有尺寸超差或拓扑错误时会阻止正式导出。

## 扩展新机型

复制模型后设置新的 `tune_profile`，并在 tuner 的 `MEASUREMENT_PROFILES` 中注册对应的参考点、目标属性和实算函数。不要沿用不匹配的 profile；未注册 profile 会触发交付错误。
