# ArmMonitor

实时机械臂状态监控工具。通过 zenoh 订阅机器人数据，以表格和曲线图实时展示关节位置/速度/力矩及末端位姿等数据。

## 快速开始

```bash
# 安装依赖
pip install -r monitor/requirements.txt

# 启动
python -m monitor.main
```

启动后连接 zenoh 网络（🟢 Connect），数据流入后右击表格单元格 → **Plot …** 添加要查看的数据曲线。

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
| `topics.*` | `/geniarm/...` | 订阅话题表达式 |
| `refresh_interval_ms` | `10` | 绘图刷新间隔（ms） |
| `initial_x_window_s` | `10.0` | 初始 X 轴宽度（秒） |
| `zenoh_connect_endpoints` | `[]` | 远程连接时填写 robot 的 tcp 端点 |

## 远程连接

monitor 与 robot 跨机器时，在 `config.yaml` 中填写：

```yaml
zenoh_connect_endpoints:
  - "tcp/<robot_ip>:7447"
```

同一机器或本地子网无需此配置。

