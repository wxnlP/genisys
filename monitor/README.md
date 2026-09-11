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
| 🟢 **Connect** / 🔴 **Disconnect** | 连接/断开 zenoh连接 |
| ▶ **继续** / ⏸ **暂停** | 开始/停止数据采样 |
| 🗑 **清除曲线** | 移除所有曲线，回到初始状态 |
| 🔄 **重置数据** | 清空历史数据，曲线重新开始绘制 |
| 💾 **保存CSV** | 将当前曲线数据导出为 CSV |
| 📏 **选区** → 💾 **导出选区** | 选择时间范围后导出该区间数据 |
| ⚙ **操作** | 显示/隐藏全部曲线；**导出/加载标签配置**（`.plottag`）；**导出/导入轨迹**（`.npz`） |
| **右击表格** | 添加/移除曲线到绘图区 |

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

编辑 `config.yaml` 后重启生效。主要参数：

| 参数 | 默认 | 说明 |
|------|------|------|
| `topics.*` | `/geniarm/...` | 订阅话题，键名决定消息类型（见下） |
| `aimrt.multicast_scouting` | `true` | 同网段靠组播零配置发现 robot |
| `aimrt.connect_endpoints` | `[]` | 主动连到对端，如 `["tcp/192.168.1.100:7447"]` |
| `aimrt.listen_endpoints` | `[]` | 被对端连入时填写固定端口 |
| `aimrt.native_cfg_path` | `""` | 高级：使用自己的 zenoh json5 |
| `aimrt.log_level` | `warn` | AIMRT 运行时日志级别 |
| `refresh_interval_ms` | `10` | 绘图刷新间隔（ms） |
| `initial_x_window_s` | `10.0` | 初始 X 轴宽度（秒） |

`topics` 的**键名**必须属于以下四种之一，它同时也决定了 protobuf 消息类型：

| 键名 | 消息类型 |
|------|----------|
| `joint_state` | `aimrt.protocols.sensor.JointStateArray` |
| `joint_cmd` | `aimrt.protocols.sensor.JointCommandArray` |
| `pose_current` | `robotic_arm.protocols.PoseStamped` |
| `pose_expected` | `robotic_arm.protocols.PoseStamped` |

机器人端 `ControlModule` 用 `using Pose = robotic_arm::protocols::PoseStamped`，
所以线上类型是 `PoseStamped`，不是 `Pose`。

## 远程连接

同一台机器或本地子网组播能通时，**无需任何配置**。

跨机器 / 跨网段时，先确认 robot 端有一个固定监听端口——AIMRT 的 zenoh 插件
默认用 zenoh 默认配置，peer 模式监听的是**随机临时端口**，直接填 7447 通常连不通。
需要在 robot 端提供 `native_cfg_path` 指定一个 json5，例如：

```json5
{
  mode: "peer",
  listen: { endpoints: { peer: ["tcp/0.0.0.0:7447"] } },
}
```

并在 `src/install/linux/bin/cfg/*.yaml` 的 `zenoh_plugin` 下取消注释
`native_cfg_path` 指向它。之后 monitor 端填：

```yaml
aimrt:
  connect_endpoints: ["tcp/<robot_ip>:7447"]
```

> 安全提示：这套系统里 zenoh 不只是"读"通道——robot 通过 zenoh 订阅
> `/geniarm/servoj_cmd` 等话题并直接驱动机器人运动，且没有鉴权。把 zenoh
> 端口暴露到不可信网络前，请先确认网络边界，或配置 zenoh 的 `access_control`。

## 实现说明

- 数据源是 **AIMRT python 运行时 + zenoh 后端**，与机器人使用同一套
  AIMRT / zenoh 版本；不再使用 zenoh-python 直接订阅。
- 只订阅 `config.yaml` 中列出的话题（旧实现用 `**` 全量订阅），无关流量不会
  进入 Python；组帧、上下文元数据解析和 protobuf 解码都在 C++ 侧完成。
- AIMRT 运行时跑在**独立子进程**（`monitor/core/aimrt_worker.py`）中，通过管道
  把解码后的消息送回 GUI。这不是为了好看：`aimrt_py` 1.7.0 在声明过带 Python
  回调的 channel 订阅者之后，`Core.Shutdown()` 必定段错误（最小 1 话题复现），
  进程内无法安全地拆除并重建会话。用子进程可以彻底绕开这个析构路径，也让
  zenoh 配置错误只杀死 worker、不会带崩 GUI。
- `monitor/core/zenoh_client.py` 是旧的 zenoh-python 实现，已不再被默认使用，
  保留仅作回退参考（如需使用请自行安装 `eclipse-zenoh`）。

