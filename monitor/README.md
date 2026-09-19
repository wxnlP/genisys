# ArmMonitor

实时机械臂状态监控工具。通过 AIMRT（zenoh 后端）订阅机器人数据，以表格和曲线图实时展示关节位置/速度/力矩及末端位姿等数据。

## 快速开始

```bash
# 安装依赖（aimrt_py 需按平台安装 wheel，见 requirements.txt）
pip install -r monitor/requirements.txt

# 启动
python -m monitor.main
```

启动后连接 AIMRT 会话（🟢 Connect），数据流入后右击表格单元格 → **Plot …** 添加要查看的数据曲线。

## 基本操作

| 操作 | 说明 |
|------|------|
| 🟢 **Connect** / 🔴 **Disconnect** | 连接/断开 AIMRT 会话 |
| ▶ **继续** / ⏸ **暂停** | 开始/停止采样与绘图 |
| 🗑 **清除曲线** | 移除所有曲线，回到初始状态 |
| 🔄 **重置数据** | 清空历史数据，曲线重新开始绘制 |
| ▶️ **开始录制** / ⏹ **停止录制** | 通过机器人的 record_playback 插件落盘；🛠 **录制配置** 可改参数与 RPC 地址 |
| ⚙ **操作** | 显示/隐藏全部曲线；**导出/加载标签配置**（`.plottag`） |
| **右击表格** | 添加/移除曲线到绘图区 |
| **右击图例** | 隐藏/显示/删除该曲线 |

### 数据面板

| 表格 | 内容 |
|------|------|
| **Joint State / Command** | 各关节位置、速度、力矩 |
| **Pose Current / Expected** | 末端位置（XYZ）+ 姿态（RPY） |

### 绘图区

- 单击 X/Y 轴标签 → 选中该轴，滚轮单独缩放
- 双击绘图区 → 恢复 Y 轴自动范围
- 图例左键 → 高亮曲线；右键 → 隐藏/显示/删除

## 配置

改完重启 monitor 生效。配置文件两个：

| 文件 | 内容 |
|------|------|
| `aimrt_cfg.yaml` | AIMRT 运行时配置 + 本程序参数（见下） |
| `zenoh_native_config.json5` | zenoh 连接配置（对端端点、组播开关） |

`aimrt_cfg.yaml` 上半部分是 AIMRT 运行时配置（插件、日志级别、执行器、channel
后端），下半部分的 `MonitorSubscriberPyModule` 段是本程序自己的参数。AIMRT 会把
这一段单独交给 worker 模块读取。

| 参数 | 默认 | 说明 |
|------|------|------|
| `topics.*` | `/geniarm/...` | 订阅话题，键名决定消息类型（见下） |
| `refresh_interval_ms` | `2` | 绘图刷新间隔（ms），实际下限为 10 |
| `initial_x_window_s` | `10.0` | 初始 X 轴宽度（秒） |
| `y_range_min` | `0.001` | Y 轴最小范围 |
| `ui_scale` | `1.0` | 界面缩放因子 |

`topics` 的**键名**必须属于以下四种之一，它同时也决定了 protobuf 消息类型：

| 键名 | 消息类型 |
|------|----------|
| `joint_state` | `aimrt.protocols.sensor.JointStateArray` |
| `joint_cmd` | `aimrt.protocols.sensor.JointCommandArray` |
| `pose_current` | `robotic_arm.protocols.PoseStamped` |
| `pose_expected` | `robotic_arm.protocols.PoseStamped` |

机器人端 `ControlModule` 用 `using Pose = robotic_arm::protocols::PoseStamped`，
所以线上类型是 `PoseStamped`，不是 `Pose`。

插件的 `path` 填文件名即可（如 `libaimrt_zenoh_plugin.so`），程序会把 aimrt_py
所在目录加入 `LD_LIBRARY_PATH`，不必写绝对路径。

> `aimrt.configurator.temp_cfg_path` 是 AIMRT 存放模块配置临时副本的目录。默认值
> `./cfg/tmp` 会让 monitor 的工作目录里凭空多出一个 `cfg/` 目录，所以这里改到了
> 系统临时目录。

## 远程连接

monitor 与控制器之间的 zenoh 连接由 `zenoh_native_config.json5` 描述：

- **同网段**：`scouting.multicast.enabled` 设为 `true`，不必填写端点。
- **跨网段**：设为 `false`，并在 `connect.endpoints` 填入控制器的固定监听地址，
  例如 `["tcp/10.200.63.14:7447"]`。

控制器端必须有固定监听端口：AIMRT 的 zenoh 插件默认走 zenoh 默认配置，peer
模式监听**随机临时端口**，没有固定端口就无从连接。仓库已提供两份配置，并由
`geniarm_*.yaml` 的 `native_cfg_path` 引用：

| 文件 | 端口 |
|------|------|
| `src/install/linux/bin/cfg/zenoh_native_config.json5` | 真机 7447 |
| `src/install/linux/bin/cfg/zenoh_native_config_sim.json5` | 仿真 7448 |

> 安全提示：这套系统里 zenoh 不只是"读"通道——robot 通过 zenoh 订阅
> `/geniarm/servoj_cmd` 等话题并直接驱动机器人运动，且没有鉴权。把 zenoh
> 端口暴露到不可信网络前，请先确认网络边界，或配置 zenoh 的 `access_control`。

## 实现说明

- 数据源是 AIMRT python 运行时 + zenoh 后端，与机器人使用同一套 AIMRT / zenoh
  版本。只订阅 `aimrt_cfg.yaml` 列出的话题，组帧与 protobuf 解码在 C++ 侧完成。
- AIMRT 运行时运行在独立子进程（`monitor/core/aimrt_worker.py`），通过管道把
  解码后的消息送回 GUI。这样每个会话都能干净地重建，zenoh 配置错误也只终止
  worker 而不会影响 GUI。

