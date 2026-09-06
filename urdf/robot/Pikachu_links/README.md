# Pikachu Link Tuner

这是一个面向简化 box-link URDF/xacro 的本地工程调参台。它把几何参数、关节位姿/限位、惯量设置、3D 装配、2D 尺寸和交付校验放在同一条工作流中。

## 启动

基础模式不依赖前端构建工具：

```bash
python3 urdf/robot/Pikachu_links/pikachu_link_tuner.py
```

如果需要 meshcat 3D 和 MJCF 编译，请用同时安装了 `meshcat`、`xacro`、`mujoco` 的环境启动：

```bash
conda activate mocap
python urdf/robot/Pikachu_links/pikachu_link_tuner.py
```

没有 meshcat 时会自动切换到浏览器内的 canvas 3D，几何调参、2D 验证和 URDF 导出仍可使用。

## 推荐工作流

1. 载入 xacro/URDF，确认顶部显示的 link/joint/box 数量符合预期。
2. 对 27DOF 模型，优先修改“工程尺寸”中的 14 个 `measure_*` 主参数；厚度、碰撞余量等再到部件分组微调。
3. 确认工程验证中的 14 项“实算 / 目标 / 偏差”全部在容差内，再用正面/侧面 2D 标注与参考图复核。
4. 在“关节”页扫描可动范围。“导出为 fixed”会保留整个 child link 子树，只固定该关节。
5. 在“惯量”页设置基准密度，只对实测质量/惯量做单 link 覆盖。
6. 检查底部“工程验证”。Error 会阻止交付；Warning 可交付，但会记入报告。
7. 用“一键导出全部”生成一个新的交付目录。服务端不覆盖同名历史目录。

支持 `Ctrl/Cmd+Z`、`Ctrl/Cmd+Shift+Z` 撤销/重做；“恢复基线”回到本次载入或应用源码时的状态。

## 交付内容

一键导出默认包含：

- `*.xacro`：保留宏、注释和集中参数的可继续编辑源文件。
- `*.urdf`：已展开、已写入惯量与关节设置的交付文件。
- `*.xml`：可选 MJCF，仅在后端具备 MuJoCo 依赖时生成。
- `*_2d.png`：带关键尺寸表的高分辨率图纸。
- `*_links.json`：每个 box link 的三轴尺寸。
- `*_validation.json`：校验结果、关键尺寸、参数基线与所有覆盖，用于复核和回溯。

## 验证范围

前端交付门禁会检查单根无环连杆树、parent/child 完整性、box 尺寸、关节轴与限位、正质量、惯量三角不等式、左右对称差、COM 与脚底支撑区。`pikachu_27dof_v1` profile 还会在零位下独立实算 14 项工程尺寸；默认容差为 ±0.5 mm，超差即阻止交付。当前姿态只影响姿态、COM 和支撑校验，不会改变标称尺寸结论。工作台启动时还会自检 X/Y/Z 轴旋转矩阵和盒体惯量公式。

27DOF 的测量基准、坐标语义和扩展约定见 [default/27dof/README.md](default/27dof/README.md)。

浏览器预览器只展开本项目使用的 xacro 子集（property、macro 和算术表达式）。使用 include、if/unless 或复杂 Python 表达式的通用 xacro，应先用 ROS `xacro` 展开后再载入，或在导出时使用安装了 `xacro` 的后端环境复核。

## 测试

```bash
pytest -q urdf/robot/Pikachu_links/test_pikachu_link_tuner.py urdf/robot/Pikachu_links/test_pikachu_27dof_contract.py
```
