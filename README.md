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
├── src
    ├── install/linux/bin/cfg  # 运行配置文件
    ├── module/
    │   ├── actuator_module    # 执行器模块
    │   ├── control_module     # 控制模块 
    │   └── genisys-core    # 底层驱动 SDK       
    │		├── actuator       # 电机驱动
    │       └── arm_control    # 运动控制
    └── protocols/             # 自定义协议

```

## 快速开始

### 环境要求

- **OS**: Ubuntu 22.04+
- **编译器**: GCC 11+ / Clang 14+ (C++20)，Rust编译器
- **构建工具**: CMake 3.24+,
- **硬件 (真机模式)**: SocketCAN 接口, DM4340P / DM4310 电机

由于本系统主要使用 AimRT 的 zenoh 后端处理外部订阅，所以需要在编译前安装Rust编译器：
```bash
# 安装rust编译器
sudo apt update
sudo apt install build-essential cmake pkg-config libssl-dev
curl --proto '=https' --tlsv1.2 -sSf https://sh.rustup.rs | sh
source $HOME/.cargo/env
```

### 构建


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
| `MoveControlServiceImpl` | HTTP RPC 服务，提供 MoveJ / MoveL / MoveJMuti / MoveLMuti / Gripper 五种运动原语 |
| `ArmController` | 轨迹规划（quintic/cubic/lspb）、逆运动学 (Piper/UR)|
| `GripperController` | 夹爪运动规划 |
| `ControlLoop` | 500Hz 控制循环，轮询轨迹插值点并下发 `/geniarm/joint_cmd` |
| `PublishLoop` | 1000Hz 发布循环，发布期望和当前笛卡尔位姿到 `/geniarm/pose_expected`、`/geniarm/pose_current` |
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

| 组件 | 功能 |
|:---|:---|
| `AimrtSubscriber` | 经 AIMRT（zenoh 后端）精确订阅话题并解码 protobuf，运行时隔离在子进程 |
| `DataModel` | 支持采样快照与 CSV 导出 |
| `DataPanel` | 左侧表格面板，实时显示各关节状态，右键添加绘图曲线 |
| `PlotPanel` | 右侧 pyqtgraph 曲线面板，支持轴独立缩放、曲线高亮、暂停浏览 |

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
│  │  ControlLoop (500Hz) ---→ /geniarm/joint_cmd     │    │
│  │  PublishLoop (1000Hz) --→ /geniarm/pose_expected │    │
│  │  │                    --→ /geniarm/pose_current  │    │
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

| Topic | 消息类型 | 方向 | 频率 | 说明 |
|:---|:---|:---|:---|:---|
| `/geniarm/joint_cmd` | `aimrt.protocols.sensor.JointCommandArray` | ControlModule → Actuator/SimModule | 500 Hz | 关节控制指令 (位置/速度/刚度/阻尼) |
| `/geniarm/joint_state` | `aimrt.protocols.sensor.JointStateArray` | Actuator/SimModule → ControlModule | 500 Hz | 关节状态反馈 |
| `/geniarm/pose_expected` | `genisys.protocols.PoseStamped` | ControlModule → (echo/monitor) | 1000 Hz | **期望笛卡尔位姿** (由当前关节指令正解算出) |
| `/geniarm/pose_current` | `genisys.protocols.PoseStamped` | ControlModule → (echo/monitor) | 1000 Hz | **当前实际笛卡尔位姿** (由关节反馈正解算出) |

### RPC 接口

服务名: `MoveControlService` | 后端: HTTP (端口 50080)

| 方法 | 请求 | 功能 |
|:---|:---|:---|
| `MoveJ` | 关节角度[6] + 速度 + 加速度 | 关节空间点到点运动 |
| `MoveL` | 末端位姿(xyz+wxyz) + 速度 + 加速度 | 笛卡尔空间直线运动 |
| `MoveJMuti` | 多组关节角度 + 过渡半径 + v + a | 多路径点关节运动 |
| `MoveLMuti` | 多组末端位姿 + 过渡半径 + v + a | 多路径点笛卡尔运动 |
| `Gripper` | 位置[0~1] + 速度 + 加速度 + 力矩 | 夹爪开合控制 |

## Python RPC 调用示例

项目提供六种 RPC 调用的 Python 示例脚本，位于 `examples/` 目录：

| 示例 | 文件 | 说明 |
|:---|:---|:---|
| MoveJ | [`examples/movej_demo.py`](examples/movej_demo.py) | 关节空间点到点运动 |
| MoveL | [`examples/movel_demo.py`](examples/movel_demo.py) | 笛卡尔空间直线运动 |
| MoveJMuti | [`examples/movej_muti_demo.py`](examples/movej_muti_demo.py) | 多路径点关节运动 |
| MoveLMuti | [`examples/movel_muti_demo.py`](examples/movel_muti_demo.py) | 多路径点笛卡尔运动 |
| Go Home | [`examples/go_home.py`](examples/go_home.py) | 一键回原点 (MoveJ) |
| Gripper | [`examples/gripper_demo.py`](examples/gripper_demo.py) | 夹爪开合测试 |

依赖: `pip install requests`

```bash
# 启动控制系统后，在另一个终端运行
python3 examples/movej_demo.py
python3 examples/movel_demo.py
python3 examples/movej_muti_demo.py
python3 examples/movel_muti_demo.py
python3 examples/go_home.py
python3 examples/gripper_demo.py
```

## 许可

[LICENSE](LICENSE)

