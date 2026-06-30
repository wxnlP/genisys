# 手眼标定 — 采集与求解工具

```
examples/hand_eye_calibration/
├── __init__.py          # 包标记
├── common.py            # 公共模块：Zenoh 位姿读取、RPC 控制、矩阵工具
├── collect_data.py      # 数据采集脚本（机械臂走姿态 → 拍照 → 检测标定板）
├── calibrate.py         # 标定求解脚本（AX=XB → T_tool_to_camera）
├── README.md            # 本文件
├── requirements.txt     # 额外依赖
└── proto/               # 生成的 protobuf 代码
    ├── __init__.py
    ├── pose_pb2.py
    ├── header_pb2.py
    ├── quaternion_pb2.py
    └── vector3_pb2.py
```

## 使用方法

### 1. 安装依赖

```bash
# 在项目根目录下
pip install opencv-python opencv-contrib-python numpy scipy
pip install -r monitor/requirements.txt   # 已有 zenoh + protobuf
```

### 2. 相机内参标定

先标定相机内参（张正友法），保存为 `camera_matrix.npy` 和 `dist_coeffs.npy`：

```python
import cv2, numpy as np
# ... 标定流程 ...
np.save("camera_matrix.npy", K)
np.save("dist_coeffs.npy", dist)
```

### 3. 采集数据

```bash
# 确保 robot 程序已启动，Zenoh 网络正常
# 棋盘格标定板固定在机械臂前方视野内
python -m examples.hand_eye_calibration.collect_data \
    --camera-id 0 \
    --num-poses 20 \
    --output calibration_data.json \
    --show-preview
```

机械臂会自动走 20 个随机姿态，每步拍照并记录位姿。

### 4. 求解手眼矩阵

```bash
python -m examples.hand_eye_calibration.calibrate \
    --data calibration_data.json \
    --method daniilidis \
    --output T_tool_to_camera.npy
```

### 5. 在代码中使用

```python
import numpy as np

T_tool_to_camera = np.load("T_tool_to_camera.npy")

# 相机看到一个点 P_cam，转换到基座标系
P_cam = np.array([x, y, z, 1.0])
P_base = T_base_to_tool @ T_tool_to_camera @ P_cam
```

## 文件说明

| 文件 | 用途 |
|------|------|
| `common.py` | `movej()` RPC 调用、`PoseReader` Zenoh 位姿读取、矩阵工具函数 |
| `collect_data.py` | 生成随机姿态 → 控制机械臂运动 → 读取末端位姿 → 拍照 → 检测标定板 → 保存 |
| `calibrate.py` | 加载数据 → `cv2.calibrateHandEye()` 求解 → 重投影验证 → 保存结果 |
| `proto/` | 从 `pose.proto` 生成的 protobuf Python 代码，用于解码 `/geniarm/pose_current` |
