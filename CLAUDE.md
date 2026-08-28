# CLAUDE.md

This file provides guidance to Claude Code (claude.ai/code) when working with code in this repository.


不要读图！不要读图！不要读图！

## Project Overview

Pikachu Motion Capture — a real-time system that captures human poses via MediaPipe and transfers them to a Pikachu character skeleton. Supports Blender integration and URDF robot model visualization.

> 🎬 **想程序化驱动 / 导出 Pikachu 关节？先读 [`docs/BLENDER_JOINT_CONTROL.md`](docs/BLENDER_JOINT_CONTROL.md)** —— 里面是 socket 控制协议（端口 9999）、21 个可控关节+极限、54 骨 rig 层级、以及一套可复制的驱动示例。

## Running the Project

```bash
# Main GUI application (requires webcam)
python Pikachu_Mocap.py

# URDF → Blender rig retarget tool (no webcam; URDF sliders + Meshcat 3D)
# 映射规则见 retarget_map.yaml（joint -> bone/axis/sign/bias/limit）
python Pikachu_Retarget.py

# MediaPipe pose detection demo
python pose/MediaPipe/main.py [--names] [--axes] [--grid]

# URDF robot viewer with joint sliders
cd urdf && python main.py

# Transfer module smoke test
python -m transfer.main
```

## Dependencies

No `requirements.txt` exists. Dependencies are managed via conda. Key libraries:

- **mediapipe**, **opencv-python** — pose detection
- **PySide6** / **PyQt6** — GUI
- **numpy**, **scipy** — math
- **matplotlib**, **trimesh**, **meshcat** — visualization
- **urdfpy** — URDF parsing
- **pyyaml** — config files

## Architecture

### Pose Pipeline

```
Camera → MediaPipe → HumanoidPoseData → SkeletonPoseData / UrdfPoseData
```

The `transfer/` module handles all coordinate-system conversions:
- `humanoid2skeleton.py` — MediaPipe landmarks → Pikachu bone rotations
- `humanoid2urdf.py` — MediaPipe landmarks → URDF joint angles
- `urdf2skeleton.py` — URDF joint angles → Pikachu skeleton

Data classes live in `transfer/data_structures.py`. Math utilities (Euler angles, rotation matrices, angle normalization) are in `transfer/utils.py`.

### Main Application (`Pikachu_Mocap.py`)

PyQt6/PySide6 GUI that:
- Reads webcam via OpenCV
- Runs MediaPipe pose detection
- Displays real-time humanoid and Pikachu skeleton visualizations side-by-side
- Sends joint data to Blender via TCP socket on `localhost:9999`
- Loads skeleton/joint config from YAML files at startup

### Blender Integration (`addon/`)

Blender addon (`blender_joint_server/`) listens on port 9999 for live rig sync. Install via `addon/blender_joint_server.zip`.

### URDF Viewer (`urdf/`)

Standalone PyQt GUI with per-joint sliders. Loads `urdf/robot/Pikachu_V025/` model. Uses meshcat for 3D rendering.

### Configuration Files (YAML)

| File | Purpose |
|------|---------|
| `addon/scripts/pikachu_skeleton.yaml` | Pikachu bone hierarchy and rest-pose matrices |
| `addon/scripts/pikachu_pose_skeleton.yaml` | Pose-specific skeleton config |
| `addon/scripts/humanoid_skeleton.yaml` | Humanoid skeleton structure |
| `addon/scripts/joint_config.yaml` | Joint angle limits/constraints |
| `pose/MediaPipe/pikachu.yaml` | MediaPipe keypoint → Pikachu bone mapping |
| `urdf/robot/Pikachu_V025/config/joint_names_Pikachu_V025.yaml` | URDF joint names |
