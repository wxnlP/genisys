# 基于 AimRT 的六轴机械臂控制系统

## 项目简介

本项目是一套基于 **AimRT** 框架的六自由度协作机械臂实时控制系统，支持本体 + 夹爪一体式控制。系统 **HTTP RPC** 轨迹规划接口，支持真机运行与 MuJoCo 仿真两种模式，配套基于 **Zenoh** 的数据监控与采集工具。

### 支持的机械臂构型

得益于 [robotic_arm_sdk](https://github.com/wxnlP/robotic_arm_sdk) 的解藕设计，系统支持常见的两种构型机械臂：

| 构型 | IK 方法 | 说明 |
|:---|:---|:---|
| **类 UR 构型** | UR 解析解 | 参考 [reBot-DevArm](https://github.com/Seeed-Projects/reBot-DevArm) 、[Panthera-HT](https://github.com/HighTorque-Robotics/Panthera-HT_Main)，系统提供 GeniArm 适配。 |
| **Piper 构型** | Piper 解析解 | 参考 [Dummy-Robot](https://github.com/peng-zhihui/Dummy-Robot)，系统提供 S6h4d 适配。 |

两套构型共用同一套控制栈（ControlModule + ActuatorModule），仅需替换 yaml 配置文件即可切换。

### 系统架构

>  以 GeniArm 为例。

```
┌─ HTTP Client ──────────────────────────────────────────────────────┐
│         │ RPC (MoveJ / MoveL / MoveJMuti / MoveLMuti)              │
│         ▼ :50080                                                   │
│  ┌─ AimRT Runtime ───────────────────────────────────────────┐     │
│  │                                                           │     │
│  │  ┌─ ControlModule ────────────────────────────────────┐   │     │
│  │  │  MoveControlService (RPC)                          │   │     │
│  │  │  ArmController (轨迹规划 · IK · 插值)                │   │     │
│  │  │  ControlLoop (500Hz) ──→ /geniarm/joint_cmd        │   │     │
│  │  │  JointStateCallback ←── /geniarm/joint_state       │   │     │
│  │  └────────────────────────────────────────────────────┘   │     │
│  │         │ channel (local)              │ channel (local)  │     │
│  │         ▼                              │                  │     │
│  │  ┌─ ActuatorModule ───────────────────────────────────┐   │     │
│  │  │  JointCmdCallback ←── /geniarm/joint_cmd           │   │     │
│  │  │  PublishLoop (500Hz) ──→ /geniarm/joint_state      │   │     │
│  │  │  GeniArmCtrl · GeniArmGripper                      │   │     │
│  │  └────────────────────────────────────────────────────┘   │     │
│  │         │ CANFD (can0)                                    │     │
│  └─────────┼─────────────────────────────────────────────────┘     │
│            ▼                                                       │
│  ┌─ 硬件层 ──────────────────────────────────────────────────────┐  │
│  │  DM4340P ×3 (joint1-3)  DM4310 ×4 (joint4-6 + gripper)       │  │
│  └──────────────────────────────────────────────────────────────┘  │
└────────────────────────────────────────────────────────────────────┘
```

### 目录

```
robotic_arm_aimrt/
├── build.sh                   # 首次构建（含依赖下载）
├── compile.sh                 # 增量编译
├── setup.sh                   # 环境变量设置
├── pkg_start.sh               # 启动脚本（交互式选配置）
├── format.sh                  # 代码格式化
├── test.sh                    # 运行测试
├── CMakeLists.txt              # 根 CMake 配置
├── cmake/                      # CMake 工具模块
├── examples/                   # Python RPC 调用示例
├── monitor/                    # ArmMonitor 桌面监控工具 (Python)
│   ├── README.md
│   ├── config.yaml
│   ├── main.py
│   ├── core/                   #   数据层
│   ├── ui/                     #   界面层
│   └── proto/                  #   Protobuf 定义
├── src/
│   ├── CMakeLists.txt
│   ├── install/linux/bin/cfg/  # 运行配置文件
│   ├── module/
│   │   ├── actuator_module/    # 执行器模块
│   │   │   └── cfg/
│   │   ├── control_module/     # 控制模块
│   │   │   └── cfg/
│   │   └── robotic_arm_sdk/   # 底层驱动 SDK
│   │       ├── actuator/       #   电机驱动
│   │       └── arm_control/    #   运动控制
│   ├── pkg/arm_control_pkg/    # 模块打包
│   └── protocols/              # Protobuf 协议定义
└── build/                      # 构建输出
    └── cfg/                    # 运行时配置
```

## 快速开始

### 环境要求

- **OS**: Ubuntu 22.04+
- **编译器**: GCC 11+ / Clang 14+ (C++20)
- **构建工具**: CMake 3.24+, Ninja
- **硬件 (真机模式)**: SocketCAN 接口, DM4340P / DM4310 电机

### 构建

```bash
# 首次构建（含依赖下载）
./build.sh

# 增量编译
./compile.sh
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
| `ControlLoop` | 500Hz 控制循环，轮询轨迹插值点并下发 |
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

基于 **MuJoCo** 物理引擎的仿真环境，可在无硬件条件下离线验证控制算法。当前提供 GeniArm 仿真配置（`geniarm_sim_cfg.yaml`），替换 yaml 配置即可仿真其他构型。

### Monitor

基于 **PyQt5 + pyqtgraph** 的桌面监控应用，位于 `monitor/` 目录：

| 组件 | 功能 |
|:---|:---|
| `ZenohSubscriber` | zenoh 多话题订阅，protobuf 解码 JointStateArray / JointCommandArray |
| `DataModel` | 支持采样快照与 CSV 导出 |
| `DataPanel` | 左侧表格面板，实时显示各关节状态，右键添加绘图曲线 |
| `PlotPanel` | 右侧 pyqtgraph 曲线面板，支持轴独立缩放、曲线高亮、暂停浏览 |

```bash
cd robotic_arm_aimrt
pip install -r monitor/requirements.txt
python -m monitor.main
```

详见 [monitor/README.md](monitor/README.md)。

---

## 通信架构（GeniArm）

```
                    ┌──────────────┐
                    │  HTTP Client │  curl / Postman / Web 前端
                    └──────┬───────┘
                           │ RPC (JSON/Protobuf)
                    ┌──────▼───────┐
                    │  net_plugin  │  :50080
                    └──────┬───────┘
                           │
              ┌────────────┴────────────┐
              │                         │
     ┌────────▼────────┐       ┌────────▼────────┐
     │  ControlModule  │       │  ActuatorModule  │
     │                 │       │                  │
     │  pub:           │       │  pub:            │
     │  /geniarm/      │──────→│  /geniarm/       │
     │  joint_cmd      │ local │  joint_state     │
     │                 │←──────│                  │
     │  sub:           │       │  sub:            │
     │  /geniarm/      │       │  /geniarm/       │
     │  joint_state    │       │  joint_cmd       │
     └─────────────────┘       └────────┬─────────┘
                                        │ CANFD
                                ┌───────▼────────┐
                                │  SocketCAN Bus │
                                │     can0       │
                                └───────┬────────┘
                                        │
                         ┌──────────────┼──────────────┐
                         │              │              │
                    ┌────▼────┐   ┌─────▼─────┐  ┌────▼────┐
                    │ joint1-3│   │ joint4-6  │  │ gripper │
                    │ DM4340P │   │ DM4310    │  │ DM4310  │
                    └─────────┘   └───────────┘  └─────────┘
```

> 上图为 GeniArm 构型示意。不同构型通过替换 yaml 配置文件切换，Topic 前缀与 MDH 参数随之变化。

| 通道 | Topic（GeniArm 示例） | 方向 | 频率 |
|:---|:---|:---|:---|
| Channel (local) | `/geniarm/joint_cmd` | ControlModule → ActuatorModule | 500 Hz |
| Channel (local) | `/geniarm/joint_state` | ActuatorModule → ControlModule | 500 Hz |
| RPC (HTTP) | `MoveControlService` | Client → ControlModule | 按需 |

---

## RPC 接口

服务名: `MoveControlService` | 后端: HTTP (端口 50080)

| 方法 | 请求 | 功能 |
|:---|:---|:---|
| `MoveJ` | 关节角度[6] + 速度 + 加速度 | 关节空间点到点运动 |
| `MoveL` | 末端位姿(xyz+wxyz) + 速度 + 加速度 | 笛卡尔空间直线运动 |
| `MoveJMuti` | 多组关节角度 + 过渡半径 + v + a | 多路径点关节运动 |
| `MoveLMuti` | 多组末端位姿 + 过渡半径 + v + a | 多路径点笛卡尔运动 |
| `Gripper` | 位置[0~1] + 速度 + 加速度 + 力矩 | 夹爪开合控制 |

### Python 调用示例

项目提供六种 RPC 调用的 Python 示例脚本，位于 `examples/` 目录：

| 示例 | 文件 | 说明 |
|:---|:---|:---|
| MoveJ | [`examples/movej_demo.py`](examples/movej_demo.py) | 关节空间点到点运动 |
| MoveL | [`examples/movel_demo.py`](examples/movel_demo.py) | 笛卡尔空间直线运动 |
| MoveJMuti | [`examples/movej_muti_demo.py`](examples/movej_muti_demo.py) | 多路径点关节运动 (圆弧过渡) |
| MoveLMuti | [`examples/movel_muti_demo.py`](examples/movel_muti_demo.py) | 多路径点笛卡尔运动 (圆弧过渡) |
| Go Home | [`examples/go_home.py`](examples/go_home.py) | 一键回原点 (MoveJ) |
| Gripper | [`examples/gripper_demo.py`](examples/gripper_demo.py) | 夹爪往复开合测试 |

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

---

## 硬件配置（GeniArm ）

| 关节 | 电机型号 | CAN ID | 反馈 ID | 运动范围 |
|:---|:---|:---|:---|:---|
| joint1 | DM4340P | 0x01 | 0x11 | ±150° |
| joint2 | DM4340P | 0x02 | 0x12 | 0° ~ 212° |
| joint3 | DM4340P | 0x03 | 0x13 | -212° ~ 0° |
| joint4 | DM4310 | 0x04 | 0x14 | ±90° |
| joint5 | DM4310 | 0x05 | 0x15 | ±90° |
| joint6 | DM4310 | 0x06 | 0x16 | ±90° |
| gripper | DM4310 | 0x07 | 0x17 | — |

> Piper (s6h4d) 构型使用不同的关节限位与 MDH 参数，详见 `src/module/robotic_arm_sdk/cfg/s6h4d_body_cfg.yaml`。

控制模式: **MIT** (阻抗控制, stiffness + damping) / POS (位置) / VEL (速度)

## 许可

[LICENSE](LICENSE)

