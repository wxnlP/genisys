# GeniArm — 基于 AimRT 的六轴机械臂控制系统

[TOC]

## 项目简介

GeniArm 是一个基于 **AimRT** 框架的六自由度协作机械臂实时控制系统，支持本体 + 夹爪一体式控制。系统采用 **CANFD 总线**驱动 DM 系列电机，提供 **HTTP RPC** 轨迹规划接口，支持真机运行与 MuJoCo 仿真两种模式。

### 系统架构

```
┌─ HTTP Client ──────────────────────────────────────────────────────┐
│         │ RPC (MoveJ / MoveL / MoveJMuti / MoveLMuti)              │
│         ▼ :50080                                                   │
│  ┌─ AimRT Runtime ───────────────────────────────────────────┐     │
│  │                                                            │     │
│  │  ┌─ ControlModule ────────────────────────────────────┐   │     │
│  │  │  MoveControlService (RPC)                          │   │     │
│  │  │  ArmController (轨迹规划 · IK · 插值)              │   │     │
│  │  │  ControlLoop (500Hz) ──→ /geniarm/joint_cmd        │   │     │
│  │  │  JointStateCallback ←── /geniarm/joint_state       │   │     │
│  │  └────────────────────────────────────────────────────┘   │     │
│  │         │ channel (local)              │ channel (local)  │     │
│  │         ▼                              │                  │     │
│  │  ┌─ ActuatorModule ───────────────────────────────────┐   │     │
│  │  │  JointCmdCallback ←── /geniarm/joint_cmd           │   │     │
│  │  │  PublishLoop (500Hz) ──→ /geniarm/joint_state     │   │     │
│  │  │  GeniArmCtrl · GeniArmGripper                      │   │     │
│  │  └────────────────────────────────────────────────────┘   │     │
│  │         │ CANFD (can0)                                     │     │
│  └─────────┼──────────────────────────────────────────────────┘     │
│            ▼                                                        │
│  ┌─ 硬件层 ──────────────────────────────────────────────────────┐  │
│  │  DM4340P ×3 (joint1-3)  DM4310 ×4 (joint4-6 + gripper)       │  │
│  └────────────────────────────────────────────────────────────────┘  │
└──────────────────────────────────────────────────────────────────────┘
```

---

## 快速开始

### 环境要求

- **OS**: Ubuntu 22.04+
- **编译器**: GCC 11+ / Clang 14+ (C++20)
- **构建工具**: CMake 3.24+, Ninja
- **硬件 (真机模式)**: SocketCAN 接口 (can0), DM4340P / DM4310 电机

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
./pkg_start.sh geniarm_real_cfg.yaml   # 真机模式
./pkg_start.sh geniarm_sim_cfg.yaml    # 仿真模式
```

---

## 模块说明

### ControlModule — 控制模块

负责高层轨迹规划与运动控制：

| 组件 | 功能 |
|:---|:---|
| `MoveControlServiceImpl` | HTTP RPC 服务，提供 MoveJ / MoveL / MoveJMuti / MoveLMuti 四种运动原语 |
| `ArmController` | 轨迹规划、逆运动学 (IK)、时间缩放 (quintic/cubic) |
| `GripperController` | 夹爪运动规划 |
| `ControlLoop` | 500Hz 控制循环，轮询轨迹插值点并下发 |
| `JointStateCallback` | 接收执行器反馈，更新控制器状态 |

### ActuatorModule — 执行器模块

负责底层电机驱动与状态反馈：

| 组件 | 功能 |
|:---|:---|
| `JointCmdCallback` | 接收控制指令，下发到电机 |
| `PublishLoop` | 500Hz 发布循环，采集电机状态并上报 |
| `GeniArmCtrl` | 六轴机械臂控制器 (CANFD) |
| `GeniArmGripper` | 夹爪控制器 (CANFD) |

### SimModule — 仿真模块 (开发中)

基于 **MuJoCo** 物理引擎的仿真环境，用于离线验证控制算法。

---

## 通信架构

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

| 通道 | Topic | 方向 | 频率 |
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

---

## 硬件配置

| 关节 | 电机型号 | CAN ID | 反馈 ID | 运动范围 |
|:---|:---|:---|:---|:---|
| joint1 | DM4340P | 0x01 | 0x11 | ±150° |
| joint2 | DM4340P | 0x02 | 0x12 | 0° ~ 212° |
| joint3 | DM4340P | 0x03 | 0x13 | -212° ~ 0° |
| joint4 | DM4310 | 0x04 | 0x14 | ±90° |
| joint5 | DM4310 | 0x05 | 0x15 | ±90° |
| joint6 | DM4310 | 0x06 | 0x16 | ±90° |
| gripper | DM4310 | 0x07 | 0x17 | — |

控制模式: **MIT** (阻抗控制, stiffness + damping) / POS (位置) / VEL (速度)

---

## 目录结构

```
robotic_arm_aimrt/
├── build.sh                   # 首次构建（含依赖下载）
├── compile.sh                 # 增量编译
├── setup.sh                   # 环境变量设置
├── pkg_start.sh               # 启动脚本（交互式选配置）
├── CMakeLists.txt              # 根 CMake 配置
├── cmake/                      # CMake 工具模块
├── src/
│   ├── CMakeLists.txt
│   ├── install/linux/bin/cfg/  # 源配置文件
│   ├── module/
│   │   ├── actuator_module/    # 执行器模块
│   │   │   └── cfg/
│   │   ├── control_module/     # 控制模块
│   │   │   └── cfg/
│   │   ├── sim_module/         # 仿真模块 (MuJoCo)
│   │   └── robotic_arm_sdk/   # 底层驱动 SDK
│   │       ├── actuator/       #   电机驱动
│   │       └── arm_control/    #   运动控制
│   ├── pkg/arm_control_pkg/    # 模块打包
│   └── protocols/              # Protobuf 协议定义
└── build/                      # 构建输出
    └── cfg/                    # 运行时配置
```

---

## 许可

[LICENSE](LICENSE)

