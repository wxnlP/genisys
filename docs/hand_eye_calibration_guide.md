# 手眼标定 — 操作流程与计算方法

> 适用于 geniarm 六轴机械臂 + 2D/深度相机（Eye-in-Hand 方案）
> 目标：标定相机与机械臂末端之间的空间变换关系 $T_{tool\to camera}$，实现视觉引导抓取

---

## 1. 背景概念

### 1.1 什么是手眼标定

手眼标定解决的是：**相机坐标系与机械臂坐标系之间的空间变换关系**。

对于 **Eye-in-Hand（眼在手上）** 方案（相机固定在机械臂末端），标定目标是求一个 $4\times4$ 齐次变换矩阵 $X = T_{tool\to camera}$，使得：

$$
P_{base} = T_{base\to tool} \cdot T_{tool\to camera} \cdot P_{camera}
$$

即：相机里看到一个物体的坐标 $P_{camera}$，能转换到机械臂基座标系 $P_{base}$ 下，机械臂才能去抓取。

### 1.2 标定方程 $AX = XB$

标定方程推导：标定板固定在外部，在任意两个机械臂位姿 $i, j$ 下，标定板在基座下的坐标恒定，因此：

$$
T_{base\to tool}^{\,i} \cdot X \cdot T_{camera\to board}^{\,i} = T_{base\to tool}^{\,j} \cdot X \cdot T_{camera\to board}^{\,j}
$$

整理得：

$$
A_{ij} \cdot X = X \cdot B_{ij}
$$

其中：
- $A_{ij} = (T_{base\to tool}^{\,j})^{-1} \cdot T_{base\to tool}^{\,i}$ — 末端在两个位姿间的**相对运动**
- $B_{ij} = T_{camera\to board}^{\,j} \cdot (T_{camera\to board}^{\,i})^{-1}$ — 标定板在两个位姿间的**相对运动**
- $X = T_{tool\to camera}$ — 待求的**手眼变换矩阵**（固定不变）

将 $A, X, B$ 写成旋转-平移形式并展开 $AX = XB$，得到两个方程：

$$
\begin{cases}
R_A R_X = R_X R_B & \text{(1) 旋转方程}\\
R_A t_X + t_A = R_X t_B + t_X & \text{(2) 平移方程}
\end{cases}
$$

### 1.3 求解方法概述

| 方法 | 原理 | 特点 |
|------|------|------|
| **Tsai-Lenz** | 两步法：先解旋转再解平移 | 经典，速度快 |
| **Park-Martin** | 用 Lie 代数将旋转方程线性化 | 精度较高 |
| **Horaud** | 四元数法 | 精度较高 |
| **Daniilidis** | **对偶四元数**同时求解旋转和平移 | **精度最高，推荐** |

OpenCV 的 `cv2.calibrateHandEye()` 直接支持以上四种方法。

---

## 2. 坐标系定义

```
┌──────────────────────────────────────────────────────────────┐
│ 符号                       │ 含义                            │
├──────────────────────────────────────────────────────────────┤
│ T_base_to_tool             │ 基座 → 末端法兰 (4×4)          │
│ T_tool_to_camera (X)       │ 末端法兰 → 相机 (4×4)  ★待求  │
│ T_camera_to_board          │ 相机 → 标定板 (4×4)            │
│ T_base_to_board            │ 基座 → 标定板 (4×4)  固定不变  │
└──────────────────────────────────────────────────────────────┘
```

**在 geniarm 项目中的具体话题/接口**：

| 数据 | 来源 | 获取方式 |
|------|------|----------|
| $T_{base\to tool}$ | 机械臂正运动学 | Zenoh 话题 `/geniarm/pose_current` |
| $T_{camera\to board}$ | 相机拍标定板 → PnP 解算 | OpenCV `solvePnP()` |
| $T_{tool\to camera}$ | 手眼标定求解 | OpenCV `calibrateHandEye()` |

---

## 3. 2D 相机完整标定流程

### 3.1 准备工作

**硬件**：
- 棋盘格标定板（推荐 ChArUco 板，允许部分遮挡）
- 标定板贴在硬平板上，确保平整

**软件**：
```bash
pip install opencv-python opencv-contrib-python numpy scipy
# 如使用 Zenoh 读取机械臂位姿：
pip install eclipse-zenoh protobuf
```

**相机内参标定**（先用张正友法标定好）：

```python
import cv2
import numpy as np
import glob

CHECKERBOARD = (9, 6)     # 棋盘格内角点数 (宽, 高)
SQUARE_SIZE = 0.030        # 格边长 (米)

objp = np.zeros((CHECKERBOARD[0]*CHECKERBOARD[1], 3), np.float32)
objp[:, :2] = np.mgrid[0:CHECKERBOARD[0], 0:CHECKERBOARD[1]].T.reshape(-1, 2)
objp *= SQUARE_SIZE

objpoints, imgpoints = [], []
for fname in glob.glob('calib_images/*.jpg'):
    img = cv2.imread(fname)
    gray = cv2.cvtColor(img, cv2.COLOR_BGR2GRAY)
    ret, corners = cv2.findChessboardCorners(gray, CHECKERBOARD, None)
    if ret:
        objpoints.append(objp)
        corners = cv2.cornerSubPix(gray, corners, (11,11), (-1,-1),
                    (cv2.TERM_CRITERIA_EPS + cv2.TERM_CRITERIA_MAX_ITER, 30, 0.001))
        imgpoints.append(corners)

ret, K, dist, rvecs, tvecs = cv2.calibrateCamera(objpoints, imgpoints, gray.shape[::-1], None, None)
print(f"重投影误差: {ret:.4f} 像素")
np.save("camera_matrix.npy", K)
np.save("dist_coeffs.npy", dist)
```

**采集要求**：
- 15~20 张图像
- 标定板覆盖画面**九宫格区域**
- 标定板与像平面夹角 $\ge 20^\circ$
- 重投影误差应 $< 0.5$ 像素

### 3.2 数据采集

机械臂走 **15~20 个不同姿态**，每个姿态记录一对数据：

```
对每个姿态 i:
  1. MoveJ 运动到目标关节角度
  2. 等待机器人稳定 (0.5~1s)
  3. 读取当前末端位姿 T_base_to_tool[i] (从 /geniarm/pose_current)
  4. 相机拍照
  5. 检测棋盘格角点
  6. solvePnP 解算 T_camera_to_board[i]
  7. 保存数据对
```

**姿态设计原则**（至关重要）：

| 原则 | 要求 | 反面例子 |
|------|------|---------|
| 旋转变化充分 | 绕 $x,y,z$ 各转 $\pm 30^\circ \sim 60^\circ$ | 只平移不旋转 → 方程退化 |
| 平移变化充分 | 在 $0.2\sim 0.5$ m 范围内移动 | 只在一个点微动 |
| 标定板始终在视野内 | 运动前用仿真验证路径 | 走到视野外 → 数据无效 |
| 标定板与像面夹角 | $\ge 20^\circ$ | 正对相机 → PnP 退化 |

**推荐的姿态序列**（可直接使用）：

```python
# 基于 home 位姿附近生成 20 个姿态
# 每个姿态 = home_joints + 随机扰动
import numpy as np

home = [0.0, 0.0, 0.0, 0.0, 0.0, 0.0]
poses = []

np.random.seed(42)
for _ in range(20):
    q = home.copy()
    # J2, J3 大幅变化（改变相机位置和朝向）
    q[1] += np.random.uniform(-1.0, 1.5)   # -57° ~ +86°
    q[2] += np.random.uniform(-2.0, 0.5)   # -115° ~ +28°
    # J1, J4, J5, J6 中等变化
    q[0] += np.random.uniform(-0.8, 0.8)
    q[3] += np.random.uniform(-1.0, 1.0)
    q[4] += np.random.uniform(-0.5, 0.5)
    q[5] += np.random.uniform(-0.5, 0.5)
    poses.append(q)
```

### 3.3 求解 $AX=XB$

```python
from cv2 import calibrateHandEye, HAND_EYE_DANIILIDIS

# 整理数据
R_g2b, t_g2b = [], []   # 末端位姿的旋转和平移
R_t2c, t_t2c = [], []   # 标定板位姿的旋转和平移

for i in range(n):
    R, t = decompose(T_base_to_tool[i])
    R_g2b.append(R); t_g2b.append(t)
    R, t = decompose(T_camera_to_board[i])
    R_t2c.append(R); t_t2c.append(t)

# 求解 (推荐 Daniilidis 方法)
R_X, t_X = calibrateHandEye(
    R_g2b, t_g2b, R_t2c, t_t2c,
    method=HAND_EYE_DANIILIDIS
)

T_tool_to_camera = np.eye(4)
T_tool_to_camera[:3, :3] = R_X
T_tool_to_camera[:3, 3]  = t_X.flatten()

print("手眼变换矩阵:\n", T_tool_to_camera)
```

### 3.4 验证

```python
# 验证：对每个位姿计算 T_base_to_board，应几乎恒定
T_pred_list = []
for i in range(n):
    T_pred = T_base_to_tool[i] @ T_tool_to_camera @ T_camera_to_board[i]
    T_pred_list.append(T_pred)

T_ref = np.median(T_pred_list, axis=0)  # 中值作为参考值

rot_errors, trans_errors = [], []
for T_pred in T_pred_list:
    R_err = T_pred[:3,:3] @ T_ref[:3,:3].T
    angle = np.degrees(np.arccos(np.clip((np.trace(R_err)-1)/2, -1, 1)))
    dist  = np.linalg.norm(T_pred[:3,3] - T_ref[:3,3]) * 1000  # mm
    rot_errors.append(angle)
    trans_errors.append(dist)

print(f"旋转误差: 均值={np.mean(rot_errors):.3f}°, 最大={np.max(rot_errors):.3f}°")
print(f"平移误差: 均值={np.mean(trans_errors):.3f} mm, 最大={np.max(trans_errors):.3f} mm")
```

**验收标准**（Eye-in-Hand）：
- 旋转误差均值 $< 1^\circ$
- 平移误差均值 $< 5$ mm
- 物理验证（末端去碰视觉识别的点）偏差 $< 10$ mm

---

## 4. 深度相机标定

### 4.1 与 2D 相机的异同

| 对比项 | 2D 相机 | 深度相机 |
|--------|---------|---------|
| 核心原理 | $AX=XB$（完全相同） | $AX=XB$（完全相同） |
| 相机内参 | 仅 RGB 内参 | RGB 内参 + 深度内参 + **RGB-D 外参** |
| 标定板检测 | 棋盘格角点（亚像素） | 同 2D（用 RGB 通道） |
| 手眼标定方式 | `solvePnP` + `calibrateHandEye` | 推荐同 2D 路线（用 RGB） |
| 额外步骤 | 无 | **必须先做 RGB-D 联合标定** |
| 精度瓶颈 | 角点检测精度 | 深度噪声 + RGB-D 对齐误差 |

### 4.2 RGB-D 联合标定（深度相机独有步骤）

深度相机的 RGB 传感器和 Depth 传感器之间存在固定刚体变换 $T_{depth\to color}$，这个必须标定。

```python
# 使用 OpenCV 双目标定
ret, K1, dist1, K2, dist2, R, T, E, F = cv2.stereoCalibrate(
    objpoints,           # 世界坐标
    imgpoints_left,      # RGB 图像角点
    imgpoints_right,     # 红外/深度图像角点
    K1, dist1,           # RGB 内参
    K2, dist2,           # 深度内参
    gray.shape[::-1],
    flags=cv2.CALIB_FIX_INTRINSIC
)
T_depth_to_color = build_transform(R, T)
```

> **注意**：大多数商用深度相机（Intel RealSense、Orbbec Astra、Microsoft Kinect）出厂时已提供 RGB-D 外参。如果精度要求高，建议自行重新标定。

### 4.3 推荐路线：走 2D 路线

使用深度相机的 **RGB 图像**走第 3 节的标准 2D 流程，完全不使用深度数据。

```
手眼标定结果: T_tool_to_color_camera
```

使用时，深度数据先通过 $T_{depth\to color}$ 映射到 RGB 坐标系，再用 $T_{tool\to color}$ 转换到机械臂坐标系：

$$
P_{base} = T_{base\to tool} \cdot T_{tool\to color} \cdot T_{depth\to color}^{-1} \cdot P_{depth}
$$

### 4.4 备选路线：纯 3D 点云标定

适用于 RGB 图像质量差或没有 RGB 的深度相机。使用已知半径的球体作为标定物。

```
对每个位姿 i:
  1. 机械臂带着球走到位姿 → 球心在基座下的坐标 = T_base_to_tool[i] 的平移
  2. 深度相机采集点云 → RANSAC 球拟合 → 球心在相机下的坐标
  3. 得到一对 3D 点对应
```

用 Umeyama 算法或 ICP 配准求解 $T_{tool\to camera}$：

```python
# Umeyama 点集配准
from scipy.spatial import procrustes

# pts_base: N×3, pts_cam: N×3
R, t = umeyama_alignment(pts_cam.T, pts_base.T)
T_tool_to_camera = build_transform(R, t)
```

**优缺点对比**：

| 路线 | 精度 | 鲁棒性 | 实现难度 | 推荐场景 |
|------|------|--------|---------|---------|
| 棋盘格（2D 路线） | 高（亚像素） | 需良好光照 | 简单 | **大多数情况** |
| 纯 3D 点云 | 受深度噪声限制 | 不受光照影响 | 较复杂 | RGB 不可用时 |

---

## 5. Eye-to-Hand 标定（眼在手外）

如果相机固定在外部，标定方程变为 $AX = ZB$，改用 OpenCV 的 `calibrateRobotWorldHandEye()`：

```python
cv2.calibrateRobotWorldHandEye(
    R_base2gripper, t_base2gripper,   # 末端在基座下的位姿
    R_target2cam,   t_target2cam,     # 标定板在相机下的位姿
    method=cv2.CALIB_HAND_EYE_DANIILIDIS
)
# 返回: R_base2cam, t_base2cam, R_gripper2target, t_gripper2target
```

---

## 6. 数学原理详解

### 6.1 $AX=XB$ 的 Tsai-Lenz 解法

**Step 1**：解旋转方程 $R_A R_X = R_X R_B$

将 $R_A$ 和 $R_B$ 转换为旋转轴-角表示。$R_X$ 将 $B$ 的旋转轴旋转到 $A$ 的旋转轴方向。用至少 3 组数据构成超定方程组，用 SVD 求解。

**Step 2**：解平移方程

由 $(R_A - I) t_X = R_X t_B - t_A$，堆叠所有数据得：

$$
\begin{bmatrix} R_{A,1} - I \\ R_{A,2} - I \\ \vdots \end{bmatrix} t_X = \begin{bmatrix} R_X t_{B,1} - t_{A,1} \\ R_X t_{B,2} - t_{A,2} \\ \vdots \end{bmatrix}
$$

最小二乘求解 $t_X$。

### 6.2 Daniilidis 对偶四元数解法

将对偶四元数 $\hat{q} = q + \epsilon q'$ 同时表示旋转和平移。$AX = XB$ 转化为：

$$
\hat{a} \star \hat{x} = \hat{x} \star \hat{b}
$$

整理为齐次线性方程组 $M \hat{x} = 0$，用 SVD 求解。**同时求解旋转和平移**，误差不累积。

### 6.3 为什么需要多个姿态

$R_X$ 有 3 个自由度，每组 $(R_A, R_B)$ 提供 2 个独立约束，理论上 $\ge 2$ 组即可。实践中要求 $\ge 15$ 组是因为：
1. **噪声平均**：多组数据降低角点检测和 PnP 的噪声
2. **避免退化**：如果所有 $R_A$ 共轴，$R_X$ 绕该轴的分量不可观
3. **工作空间覆盖**：不同位姿的约束互补

### 6.4 姿态质量评价

```python
def pose_quality_score(R_A_ij, R_B_ij):
    """评估一对姿态的标定价值 (0~1)"""
    angle_A = np.arccos((np.trace(R_A_ij) - 1) / 2)
    angle_B = np.arccos((np.trace(R_B_ij) - 1) / 2)
    score = min(1.0, (angle_A + angle_B) / (2 * np.pi / 3))
    return score
```

低于 0.3 的位姿对标定贡献很小，应丢弃。

---

## 7. 常见问题

### Q1: 标定后误差很大？

| 现象 | 可能原因 | 解决 |
|------|---------|------|
| 重投影平移误差 $>10$ mm | 姿态不够多样 | 增加姿态数，扩大旋转范围 |
| 旋转误差 $>2^\circ$ | 标定板角点检测不准 | 检查对焦、光照、标定板平整度 |
| 个别姿态误差特别大 | 该姿态 PnP 退化 | 删除该数据点重新求解 |
| 整体偏差大 | 机械臂绝对定位精度差 | 先做运动学标定 |

### Q2: 什么时候需要重新标定？

- 相机拆装后 → **必须重新标定**
- 机械臂基座移动后 → **必须重新标定**
- 正常使用 → 每 3~6 个月验证一次

### Q3: 标定板用什么？

- **打印在纸上贴硬平板**：够用，确保平整
- **陶瓷/玻璃棋盘格**：工业级，精度更高
- **ChArUco 板**：部分遮挡也能检测，**推荐**

### Q4: 深度相机标定精度比 2D 差多少？

深度相机的手眼标定精度通常比 2D 相机低 30~50%，主要因为：
1. 深度噪声（$\pm 1\sim 3$ mm vs 2D 亚像素 ≈ 0.1 mm）
2. RGB-D 对齐误差
3. 纯 3D 路线中球拟合精度受点云密度影响

### Q5: 与静力学标定的关系？

两者是独立的：

| | 静力学标定 | 手眼标定 |
|--|-----------|---------|
| 目的 | 重力补偿参数辨识 | 相机-机器人坐标变换 |
| 输出 | 连杆质量×质心 | $T_{tool\to camera}$ |
| 依赖 | 力矩传感器 | 相机+标定板 |
| 顺序 | 先做（保证运动精度） | 后做 |

建议：**先完成静力学标定**，装上相机后做末端负载辨识，最后做手眼标定。

---

## 8. 参考文献

1. Tsai, R. Y., & Lenz, R. K. (1989). "A new technique for fully autonomous and efficient 3D robotics hand/eye calibration." *IEEE Transactions on Robotics and Automation*.
2. Daniilidis, K. (1999). "Hand-eye calibration using dual quaternions." *The International Journal of Robotics Research*.
3. Park, F. C., & Martin, B. J. (1994). "Robot sensor calibration: solving AX= XB on the Euclidean group." *IEEE Transactions on Robotics and Automation*.
4. OpenCV: `cv2.calibrateHandEye()` — [官方文档](https://docs.opencv.org/master/d9/d0c/group__calib3d.html#gaebfc1c9f7434196a374c382abf43439b)
