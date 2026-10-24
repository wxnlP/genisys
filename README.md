# Genisys —— 基于 AimRT 的六轴机械臂控制系统

## 项目简介

本项目是一套基于 **AimRT** 框架的六自由度协作机械臂实时控制系统，支持本体 + 夹爪一体式控制。系统提供基础的通信和算法能力，配合 **ArmMonitor** 可视化工具做轨迹调试和录制。

### 支持的机械臂构型

系统支持常见的两种构型机械臂：

| 构型 | IK 方法 | 说明 |
|:---|:---|:---|
| **类 UR 构型** | UR 解析解 | 参考 [reBot-DevArm](https://github.com/Seeed-Projects/reBot-DevArm) 、[Panthera-HT](https://github.com/HighTorque-Robotics/Panthera-HT_Main)，系统提供 GeniArm 适配。 |
| **Piper 构型** | Piper 解析解 | 参考 [Dummy-Robot](https://github.com/peng-zhihui/Dummy-Robot)，系统提供 S6h4d 适配。 |

两套构型共用同一套控制栈（ControlModule + ActuatorModule），仅需替换 yaml 配置文件即可切换。此外，得益于 [genisys-core](https://github.com/wxnlP/genisys-core) 的解藕设计，开发者可以拓展适配自己的机械臂，无论你是 UART 、CAN、Ethernet，只需按照 actuator 模块的接口做适配，即可丝滑对接 control 模块。

### 目录

```
genisys
├── build.sh                   # 构建脚本
├── compile.sh                 # 增量编译
├── setup.sh                   # ROS环境变量设置
├── pkg_start.sh               # 启动脚本
├── CMakeLists.txt             # 根 CMake 配置
├── cmake                      # CMake 工具模块
├── examples                   # Python RPC 调用示例
├── monitor                    # ArmMonitor 数据监控与采集软件
└── src
    ├── install/linux/bin/cfg  # 运行配置文件
    ├── module/
    │   ├── actuator_module    # 执行器模块
    │   ├── control_module     # 控制模块 
    │   └── genisys-core       # 底层驱动 SDK       
    │		├── actuator       # 电机驱动
    │       └── arm_control    # 运动控制
    └── protocols/             # 自定义协议

```

## 快速开始

### 环境配置

环境要求：

- **OS**: Ubuntu 22.04/Ubuntu 24.04
- **编译器**: GCC 11+，Rust编译器（Zenoh）
- **构建工具**: CMake 3.24+,
- **通信硬件**: SocketCAN 

环境配置：

- 安装rust编译器

```bash
sudo apt update
sudo apt install build-essential cmake pkg-config libssl-dev
curl --proto '=https' --tlsv1.2 -sSf https://sh.rustup.rs | sh
source $HOME/.cargo/env
```

- Ubuntu 22.04需要更新CMake版本

> 注意根据自己的系统选择后缀，如 x86_64、aarch64。 

```bash
# 拉取安装脚本
wget https://cmake.org/files/v3.28/cmake-3.28.6-linux-aarch64.sh

# 添加可执行权限
chmod +x cmake-3.28.6-linux-aarch64.sh
sudo sh cmake-3.28.6-linux-aarch64.sh --prefix=/opt/cmake

# 安装
cd /opt/cmake 
sudo mv ./cmake-3.28.6-linux-aarch64/* ./
sudo rm -rf ./cmake-3.28.6-linux-aarch64

# 添加环境变量
echo 'export PATH=/opt/cmake/bin:$PATH' >> ~/.bashrc
source ~/.bashrc
```

### 构建

```bash
# 首次使用需配置当前用户的调度策略权限
echo "$(id -un)   -   rtprio   90" | sudo tee /etc/security/limits.d/99-robot-realtime.conf
```

```bash
# 首次构建，需要下载依赖可能较慢，建议内存8G以上或申请8G的虚拟内存，防止编译崩溃
./build.sh
```

### 运行

```bash
# 交互式选择配置文件
./pkg_start.sh

# 或直接指定
./pkg_start.sh geniarm_real_cfg.yaml   # GeniArm 真机模式
./pkg_start.sh geniarm_sim_cfg.yaml    # GeniArm 仿真模式
```

> 切换机械臂构型需替换 yaml 配置文件。

---

## 模块说明

### ControlModule

负责高层轨迹规划与运动控制：

| 组件 | 功能 |
|:---|:---|
| `MoveControlServiceImpl` | HTTP RPC 服务，提供 MoveJ / MoveJPose / MoveL / MoveJMulti / MoveJPoseMulti / MoveLMulti 以及夹爪 GripperPos / GripperForce 原语；运动指令**受理即返回**，完成状态经 `/geniarm/task_context` 通知 |
| `ArmController` | 轨迹规划（quintic/cubic/lspb）、逆运动学 (Piper/UR)、运动状态机与任务记录 |
| `GripperController` | 夹爪运动规划 |
| `ControlLoop` | 500Hz 控制循环，轮询轨迹插值点并下发 `/geniarm/joint_cmd` |
| `PosePublishLoop` | 500Hz 发布循环，发布期望 TCP 位姿 `/geniarm/tcp_pose_expected` 与当前法兰/TCP 位姿 `/geniarm/flange_pose`、`/geniarm/tcp_pose` |
| `TaskPublishLoop` | 50Hz 任务状态发布循环，把最近的任务记录发布到 `/geniarm/task_context` |
| `JointStateCallback` | 接收执行器反馈，更新控制器状态 |

### ActuatorModule

负责底层电机驱动与状态反馈：

| 组件 | 功能 |
|:---|:---|
| `JointCmdCallback` | 接收控制指令，下发到电机 |
| `PublishLoop` | 500Hz 发布循环，采集电机状态并上报 |
| `GeniArmCtrl` | 六轴机械臂控制器 (CANFD) |
| `GeniArmGripper` | 夹爪控制器 (CANFD) |

### SimModule

基于 **MuJoCo** 物理引擎的仿真环境，可在无硬件条件下离线验证控制算法。当前提供 GeniArm 仿真配置，替换 yaml 配置即可仿真其他构型。


### Monitor

基于 **PyQt5 + pyqtgraph** 的桌面监控应用，位于 `monitor/` 目录：

```bash
cd genisys
pip install -r monitor/requirements.txt
python -m monitor.main
```

详见 [monitor/README.md](monitor/README.md)。

---

## 通信架构

```
┌──────────────┐     ┌──────────────┐     ┌──────────────────┐
│  HTTP Client │     │  echo_plugin │     │    ArmMonitor    │
│  (curl/GUI)  │     │   (DEBUG)    │     │  (Data Capture)  │
└──────┬───────┘     └──────┬───────┘     └────────┬─────────┘
       │ RPC                │ sub (channel)        │ sub (zenoh)
       ▼                    ▼                      ▼
┌──────────────────────────────────────────────────────────┐
│                   AimRT Runtime                          │
│                                                          │
│  ┌─ net_plugin ─────────────┐  ┌─ zenoh_plugin ────────┐ │
│  │  HTTP : 50080            │  │  Zenoh Native         │ │
│  └──────────────────────────┘  └───────────────────────┘ │
│                                                          │
│  ┌─ ControlModule ──────────────────────────────────┐    │
│  │  MoveControlService (RPC)                        │    │
│  │  ArmController · GripperController               │    │
│  │  ControlLoop (500Hz) ------→ /geniarm/joint_cmd  │    │
│  │  PosePublishLoop (500Hz) --→ tcp_pose_expected   │    │
│  │  │                      --→ tcp_pose             │    │
│  │  │                      --→ flange_pose          │    │
│  │  TaskPublishLoop (50Hz) --→ task_context         │    │
│  │  JointStateCallback  ←--- /geniarm/joint_state   │    │
│  └──────────────────────┬───────────────────────────┘    │
│                         │ channel (local / zenoh)        │
│  ┌──────────────────────▼───────────────────────────┐    │
│  │  ActuatorModule  / MujocoSimModule               │    │
│  │  JointCmdCallback ←-- /geniarm/joint_cmd         │    │
│  │  Publish/SimLoop --→ /geniarm/joint_state        │    │
│  └──────────────────────────────────────────────────┘    │
└──────────────────────────────────────────────────────────┘
```

### Channel 话题列表

| Topic | 消息类型 | 说明 |
|:---|:---|:---|
| `/geniarm/joint_cmd` | `aimrt.protocols.sensor.JointCommandArray` (protobuf) | 关节控制指令 (MIT) |
| `/geniarm/joint_state` | `aimrt.protocols.sensor.JointStateArray` (protobuf) | 关节状态反馈 |
| `/geniarm/tcp_pose_expected` | `genisys.protocols.PoseStamped` (protobuf) | **期望 TCP 位姿** |
| `/geniarm/tcp_pose` | `genisys.protocols.PoseStamped` (protobuf) | **当前 TCP 位姿** |
| `/geniarm/flange_pose` | `genisys.protocols.PoseStamped` (protobuf) | **当前法兰位姿** |
| `/geniarm/servoj_cmd` | `genisys_msgs/msg/ServoJCommand` (ROS2 Message) | 关节空间伺服指令 |
| `/geniarm/servoj_pose_cmd` | `genisys_msgs/msg/ServoJPoseCommand` (ROS2 Message) | 笛卡尔位姿伺服指令 |
| `/geniarm/servol_cmd` | `genisys_msgs/msg/ServoLCommand` (ROS2 Message) | 笛卡尔直线伺服指令 |
| `/geniarm/task_context` | `genisys_msgs/msg/TaskContext` (ROS2 Message) | 任务状态事件，按 `task_id` 归并 |

### RPC 接口

服务名: `MoveControlService` | 后端: HTTP (端口 50080)

| 方法 | 请求 | 功能 |
|:---|:---|:---|
| `MoveJ` | 关节角度[6] + 速度 + 加速度 | 关节空间点到点运动 |
| `MoveJPose` | TCP 位姿(xyz+wxyz) + 速度 + 加速度 | 逆解后关节空间点到点运动 |
| `MoveL` | TCP 位姿(xyz+wxyz) + 速度 + 加速度 | 笛卡尔空间直线运动 |
| `MoveJMulti` | 多组关节角度 + 过渡半径 + v + a | 多路径点关节运动 |
| `MoveJPoseMulti` | 多组 TCP 位姿 + 过渡半径 + v + a | 多路径点位姿运动（逆解后关节空间） |
| `MoveLMulti` | 多组 TCP 位姿 + 过渡半径 + v + a | 多路径点笛卡尔运动 |
| `GripperPos` | 位置[0~1] + 速度 + 加速度 + 力矩 | 夹爪位置控制（开合） |
| `GripperForce` | 目标力矩 + 速度亏欠阈值 + 接近速度 + 接近加速度 + 闭合释放 | 力控抓取（碰到后按力矩夹住） |

### 运动指令与完成通知

运动类 RPC 是**非阻塞**的：受理成功立即返回 `error.code = 0`，并带回

| 字段 | 含义 |
|:---|:---|
| `task_id` | 本次任务编号，从 1 单调递增；被拒绝的请求不占用编号 |
| `duration_ms` | 规划给出的预计执行时长 |

不受理时返回 `error.code = -1`。系统采用严格模式：同一时刻只允许一条运动任务，当前状态不是空闲时新请求直接失败，不排队。

运动是否完成、是否成功，通过 `/geniarm/task_context` 通知，客户端按 `task_id` 归并：

| state | 值 | 含义 | code |
|:---|:---|:---|:---|
| `RUNNING` | 0 | 已受理，正在生产轨迹帧 | - |
| `FINISHED` | 3 | 轨迹播完，且收敛到到位容差内 | 0 |
| `TOLER_ERROR` | 2 | 轨迹播完，但未收敛到容差 | -3 |
| `SAMPLE_ERROR` | 1 | 采样阶段出错（逆解失败、越限等） | -2 |

`header.stamp` 是这条事件产生的时刻，`end_timestamp` 是任务结束时刻（`RUNNING` 事件里为 0）。

## Python RPC 调用示例

项目提供八种 RPC 调用的 Python 示例脚本，位于 `examples/` 目录：

> 这些脚本只检查 RPC 的**受理结果**。运动指令受理即返回，脚本退出不代表动作已经完成；要等运动结束，需要订阅 `/geniarm/task_context` 并按 `task_id` 匹配。

| 示例 | 文件 | 说明 |
|:---|:---|:---|
| `MoveJ` | [`examples/movej_demo.py`](examples/movej_demo.py) | 关节空间点到点运动 |
| `MoveJPose` | [`examples/movej_pose_demo.py`](examples/movej_pose_demo.py) | 目标 TCP 位姿的点到点运动 (逆解后关节空间插值) |
| `MoveL` | [`examples/movel_demo.py`](examples/movel_demo.py) | TCP 位姿的笛卡尔空间直线运动 |
| `MoveJMulti` | [`examples/movej_multi_demo.py`](examples/movej_multi_demo.py) | 多路径点关节运动 |
| `MoveJPoseMulti` | [`examples/movej_pose_multi_demo.py`](examples/movej_pose_multi_demo.py) | 多路径点 TCP 位姿运动 (逆解后关节空间插值) |
| `MoveLMulti` | [`examples/movel_multi_demo.py`](examples/movel_multi_demo.py) | 多路径点 TCP 位姿笛卡尔运动 |
| `Go Home` | [`examples/go_home.py`](examples/go_home.py) | 一键回原点 (`MoveJ`) |
| `GripperPos / GripperForce` | [`examples/gripper_demo.py`](examples/gripper_demo.py) | 夹爪测试：`--pos pos v a e` 位置控制，`--force target switch v a release` 力控抓取 |

依赖: `pip install requests`

```bash
# 启动控制系统后，在另一个终端运行
python3 examples/movej_demo.py
python3 examples/movej_pose_demo.py
python3 examples/movel_demo.py
python3 examples/movej_multi_demo.py
python3 examples/movej_pose_multi_demo.py
python3 examples/movel_multi_demo.py
python3 examples/go_home.py
python3 examples/gripper_demo.py
```

## 许可

[LICENSE](LICENSE)

