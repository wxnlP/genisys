# AGENTS.md — genisys 工程速读

> 面向新会话的 agent。先读本文件，再按任务打开对应文件；不要一上来全仓库扫描。

## 0. 一句话

基于 **AimRT** 的六轴机械臂实时控制栈：`ControlModule` 负责轨迹规划 / RPC / 500Hz 控制循环，`ActuatorModule` 负责 CANFD 电机下发与反馈（仿真时换成官方 `MujocoSimModule`），运动学与轨迹算法在 **`genisys-core` 子模块**里。对外主要接口是 **HTTP RPC**（`:50080`）和 AimRT channel（local / zenoh）。

## 1. 事实表

| 项 | 值 |
|:---|:---|
| 语言/标准 | C++20、Python 3（示例/监控）、Proto3、ROS2 msg/srv |
| 构建 | CMake ≥ 3.24、GCC 11+、Eigen3、yaml-cpp；首次 `build.sh` 拉 AimRT / MuJoCo / 依赖（gitee、gh-proxy 镜像） |
| 系统 | Ubuntu 22.04 / 24.04；真机用 SocketCAN（CANFD） |
| 通信 | AimRT channel：`local` + `zenoh`；RPC 后端：HTTP `127.0.0.1:50080` |
| 当前构型 | `geniarm`（UR 构型，`ik_mode: UR` → `ParallelWrist`） |
| 分支 | 父仓库 `dev`；子模块 `genisys-core` 跟踪 `dev` |
| 格式化 | clang-format 15，Google 风格，`ColumnLimit: 0`（见 `.clang-format`） |
| 注释/日志 | 中文为主；`AIMRT_INFO("... {}", x)` 的 fmt 风格 |

## 2. 阅读优先级：任务 → 先看哪几个文件

| 任务 | 先读 |
|:---|:---|
| 改 RPC、控制循环、任务状态 | [`control_module.cc`](src/module/control_module/control_module.cc)、[`control_module.h`](src/module/control_module/control_module.h) |
| 改逆解 / 选解 / 轨迹 / 状态机 | [`arm_controller.cc`](src/module/genisys-core/arm_control/src/arm_controller.cc)、[`arm_controller.h`](src/module/genisys-core/arm_control/include/arm_controller.h) |
| 改解析 FK/IK | [`kinematic.cc`](src/module/genisys-core/arm_control/src/kinematic.cc)、[`tool_frame.cc`](src/module/genisys-core/arm_control/src/tool_frame.cc) |
| 加/改接口 | [`genisys_msgs`](src/protocols/ros2/genisys_msgs)、同目录 [`CMakeLists.txt`](src/protocols/ros2/genisys_msgs/CMakeLists.txt) |
| 改配置 / 换构型 | `src/install/linux/bin/cfg/geniarm_real_cfg.yaml`、`src/module/control_module/cfg/geniarm_body_cfg.yaml`、[`ADAPTATION.md`](src/module/genisys-core/ADAPTATION.md) |
| 写客户端 | [`examples/`](examples)（`requests.post` 直连 HTTP JSON） |
| 驱动 / CAN | [`actuator_module.cc`](src/module/actuator_module/actuator_module.cc)、[`genisys-core/actuator`](src/module/genisys-core/actuator) |
| 监控 / 数据 | [`monitor/README.md`](monitor/README.md) |

**不要读**（浪费 token，或不是源码）：`build/`、`.cache/`、`src/module/genisys-core/actuator/third_party/`、`src/install/linux/bin/model|meshes`、`monitor/ui/`、`monitor/proto/`、`**/__pycache__/`。`test/` 是 gitignore 的草稿区。

## 3. 目录地图

```text
genisys/
├── CMakeLists.txt / cmake/          # 根构建；GetAimRT / GetMujocoSim / GetGTest / NamespaceTool
├── build.sh                         # 首次全量 configure + build + install
├── compile.sh                       # 增量 cmake --build --target install
├── pkg_start.sh                     # 从 build/ 启动 aimrt_main，交互选择 cfg
├── setup.sh                         # source build/install/share/genisys_msgs/local_setup.bash
├── socketcan_config.sh              # 配置 CAN 接口
├── examples/                        # 8 个 Python HTTP RPC 示例
├── monitor/                         # ArmMonitor：PyQt5 + pyqtgraph，zenoh 订阅
├── src/
│   ├── pkg/arm_control_pkg/         # 动态库入口，AIMRT_PKG_MAIN 注册 ControlModule/ActuatorModule
│   ├── module/
│   │   ├── control_module/          # 外层：RPC + 3 个循环 + 回调；cfg/ 是机器人本体配置
│   │   ├── actuator_module/         # 外层：CANFD 指令下发 / 500Hz 状态上报
│   │   └── genisys-core/            # 子模块（独立仓库）：算法 + 驱动
│   │       ├── arm_control/         #   运动学、雅可比、路径、时间缩放、轨迹、ArmController
│   │       └── actuator/            #   GeniArm(CANFD) / 夹爪 + dm_driver、serial
│   ├── protocols/
│   │   ├── pb/genisys/              # Proto：Pose/PoseStamped 在用；move_control.proto 是历史遗留
│   │   └── ros2/genisys_msgs/       # ROS2 msg/srv + aimrt rpc gencode（当前 RPC 的真身）
│   └── install/linux/bin/           # 运行 cfg、MuJoCo 模型/网格、启动脚本、录制脚本
└── build/                           # 产物：aimrt_main、libarm_control_pkg.so、cfg/、log/、install/
```

## 4. 构建 / 运行 / 验证

```
./build.sh                              # 首次；内存建议 ≥8G
# 仿真需要 MuJoCo 组件；build.sh 默认传 OFF，用追加参数覆盖：
./build.sh -DGENISYS_BUILD_MUJOCO_SIM=ON

./compile.sh                            # 日常增量

./pkg_start.sh geniarm_sim_*.yaml     # 仿真
./pkg_start.sh geniarm_real_*.yaml    # 真机（需要 CAN 与 rtprio 权限）
```

- `pkg_start.sh` 实际执行：`cd build && ./aimrt_main --cfg_file_path=./cfg/<cfg>.yaml`。

- **必须从 `build/` 找 cfg**：`ArmManager`/`ActuatorManager` 会自己拼 `"./cfg/" + <config_file>`，所以工作目录必须是 `build/`。

- 日志：控制台 + `build/log/*.log`（`rotate_file`，4MB × 10）。

## 5. 运行时架构

### 模块与循环

| **模块 / 循环**   | **职责 / 频率** | **核心作用**                                                 |
| ----------------- | --------------- | ------------------------------------------------------------ |
| `ControlModule`   | 500Hz           | 创建 RPC server、3 个循环、订阅 joint_state / 3 个 servo 通道 |
| `ActuatorModule`  | 500Hz           | 订阅 `/geniarm/joint_cmd` 下发电机，发布 `/geniarm/joint_state` |
| `ControlLoop`     | 500Hz           | 取轨迹采样点 → 组 `JointCommandArray` → `/geniarm/joint_cmd` |
| `PosePublishLoop` | 500Hz           | 发布 `/geniarm/tcp_pose_expected`、`/geniarm/tcp_pose`、`/geniarm/flange_pose` |
| `TaskPublishLoop` | 50Hz            | 把任务历史 / 当前任务发到 `/geniarm/task_context`            |

### RPC（HTTP `:50080`）

方法名 = srv 文件名；URL 形如：`http://127.0.0.1:50080/rpc/genisys_msgs/srv/MoveJ`

- **运动 RPC 语义**：受理即返回，不阻塞到运动完成。成功返回 `error.code = 0` 及 `task_id`。
- 完成/失败通过 `/geniarm/task_context` 通知（按 `task_id` 归并）。
- 常见方法：`MoveJ`、`MoveJPose`、`MoveL`、`MoveJMulti`、`MoveJPoseMulti`、`MoveLMulti`、`GripperPos`、`GripperForce`。

## 6. 配置系统（两层）

1. **顶层运行 cfg**：`src/install/linux/bin/cfg/*.yaml`（AimRT 插件、executor、channel 绑定等）。
2. **模块 cfg**：
   - `src/module/control_module/cfg/geniarm_body_*.yaml`（限位、MDH、`ik_mode`、`tool` 等）。
   - `src/module/actuator_module/cfg/geniarm_actuator_*.yaml`（CAN 端口、电机 ID 等）。
   - *注意：限位 / MDH / tool 的 yaml 单位是**度**，代码里会转弧度。*

## 7. 运动学 / IK 关键避坑指南

- **IK/FK 返回值**：`KinematicSolver::IK` / `FK` 现在都是 `void`，不再用返回值判断失败。判断有没有解**必须看 `solveFlag[8][3]`**（`0`=不可达，`±1`=可用）。
- **位姿基准**：输入给 `IK` 的是**法兰位姿**；`Move*` 输入是 **TCP**，进入 IK 前需先 `tool_frame_.Remove(tcp)`。
- **幽灵解与漏解**：Piper（`SphericalWrist`）可能存在不可达位姿给出非零 flag 的情况，建议对选中的解做一次 FK 反代校验；限位为硬约束。
- **关节区间**：geniarm 限位（度）中 J2/J3 是不对称的 `[-212, 0]` 和 `[0, 212]` 区间，2π 归一化时切勿简单按 ±180 处理。