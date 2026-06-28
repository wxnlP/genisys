# ArmMonitor

实时机械臂关节状态监控工具。通过 zenoh 订阅机器人发布的 JointStateArray / JointCommandArray 数据，以表格和曲线图实时展示各关节的位置、速度、力矩。

> **Design by** lzh &nbsp;|&nbsp; **Implemented by** Claude

## 目录结构

```
monitor/
├── README.md              # 本文件
├── requirements.txt       # Python 依赖
├── config.yaml            # 用户配置（话题、刷新率等）
├── main.py                # 入口

├── core/                  # 数据层
│   ├── __init__.py
│   ├── config.py          # 配置加载器
│   ├── data_model.py      # 时序数据存储（不限量，手动清空）
│   ├── zenoh_client.py    # Zenoh 订阅 + AIMRT 帧解码
│   └── zenoh_scout.py     # 网络发现辅助脚本

├── ui/                    # 界面层
│   ├── __init__.py
│   ├── main_window.py     # 主窗口布局 + 信号连接
│   ├── data_panel.py      # 左侧数据表格面板
│   └── plot_panel.py      # 右侧曲线图面板

└── proto/
    ├── __init__.py
    ├── header_pb2.py          # protobuf 定义（生成）
    ├── joint_state_pb2.py     # JointStateArray 消息
    └── joint_command_pb2.py   # JointCommandArray 消息
```

## 使用方法

```bash
# 1. 进入项目根目录，激活虚拟环境
cd /path/to/robotic_arm_aimrt
source .venv/bin/activate

# 2. 安装依赖
pip install -r monitor/requirements.txt

# 3. （可选）按需修改配置
#    编辑 monitor/config.yaml 中的话题、刷新率等参数

# 4. 启动
python -m monitor.main

# 5. 点击工具栏「● Connect」连接 zenoh 网络
#    数据开始流入后，右击表格单元格 → 选择「Plot …」添加曲线
```

> **注意**：应用启动后默认处于暂停状态（⏸），点击 **▶ 继续** 开始采样和绘图。

## 配置参数

| 参数 | 默认值 | 说明 |
|------|--------|------|
| `topics.joint_state` | `/geniarm/joint_state` | 关节实际状态话题 |
| `topics.joint_cmd` | `/geniarm/joint_cmd` | 关节期望指令话题 |
| `refresh_interval_ms` | `10` | 绘图刷新间隔（ms） |
| `initial_x_window_s` | `10.0` | 初始 X 轴视图宽度（秒） |
| `y_range_min` | `0.001` | Y 轴最小范围，防止微小偏差时曲线不可辨 |
| `csv_export_window_s` | `0` | CSV 导出时间窗口（秒），0 = 不限量 |
| `zenoh_connect_endpoints` | `[]` | 远程 zenoh 端点列表，跨机器时填写（如 `["tcp/192.168.1.100:7447"]`） |

编辑 `config.yaml` 后重启应用生效。

## 界面功能

| 操作 | 说明 |
|------|------|
| **● Connect** | 连接 zenoh 网络，开始接收数据 |
| **◉ Disconnect** | 断开连接 |
| **右击表格** | 选择要绘制的关节数据字段（再次右击可移除曲线） |
| **↺ 重置数据** | 清空历史数据，曲线从 0 重画，自动继续采样 |
| **✕ 清除曲线** | 移除所有曲线和图例，清空数据，自动暂停 |
| **💾 Save CSV** | 将当前绘制曲线的数据导出为 CSV 文件 |
| **☰ 操作** | 显示全部 / 隐藏全部曲线（括号内为当前曲线数） |
| **▶ 继续 / ⏸ 暂停** | 冻结/恢复数据采样与自动跟随 |
| **单击 X/Y 轴标签** | 选中坐标轴（橙色高亮），滚轮缩放该轴 |
| **单击绘图区空白** | 取消坐标轴选中 |
| **双击绘图区** | 恢复 Y 轴自动范围 |
| **图例左键** | 高亮/取消高亮曲线（加粗 + 置顶） |
| **图例右键** | 隐藏 / 显示 / 删除曲线 |
| **图例悬停** | 鼠标变为手型指针 |

### 工具栏信息

- **Hz**：实时采样率显示
- **Samples**：累计采样数
- **状态栏**：连接状态、订阅话题、运行时长

## 数据流

1. `ZenohSubscriber` 订阅 `config.yaml` 中配置的所有话题（默认 `joint_state` + `joint_cmd`），peer mode
2. 收到每条消息 → protobuf 解码 → 按 source 名称 emit `data_received` Qt 信号（线程安全）
3. `MainWindow._on_data_received` → `DataModel.ingest()` 写入环形缓冲区 + `DataPanel.update_from_message()` 刷新表格
4. DAQ 定时器（10ms 固定）→ `DataModel.sample_snapshot()` 以统一时间戳采样所有缓冲区
5. UI 定时器（由 `refresh_interval_ms` 控制）→ `PlotPanel.update_curves()` 将最新时序数据推入 pyqtgraph 曲线

## 数据存储

数据**不限量**累积，支持以下管理方式：

- 点击 **↺ 重置数据** 手动清空历史，曲线自动从 0 重画
- 点击 **✕ 清除曲线** 清空数据并移除所有曲线
- 点击 **💾 Save CSV** 导出当前绘制曲线的数据（可通过 `csv_export_window_s` 限制导出范围）

长时间运行注意内存 — 定期重置即可。

## 依赖

| 包 | 最低版本 | 用途 |
|----|----------|------|
| PyQt5 | 5.15 | GUI 框架 |
| pyqtgraph | 0.14 | 实时绘图 |
| eclipse-zenoh | 1.9 | zenoh 通信 |
| protobuf | 5 | 消息解码 |
| pyyaml | 6.0 | 配置文件解析 |

## 界面主题

蓝白科技风（blue-white tech theme）：
- 主色 `#2563eb`（蓝），背景 `#f8fafc`（白）
- 等宽字体栈：JetBrains Mono → Fira Code → Consolas → monospace
- 表格蓝色表头 + 斑马纹交替行色
