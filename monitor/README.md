ArmMonitor

实时机械臂关节状态监控工具。通过 zenoh 订阅机器人发布的 JointState / JointCommand / Pose 数据，以表格和曲线图实时展示各关节的位置、速度、力矩，以及末端位姿。

> **Design by** lzh(wxnlP) &nbsp;|&nbsp; **Implemented by** Claude and Copilot  
> **Version:** 0.1.1

## 目录结构

```
monitor/
├── requirements.txt       # Python 依赖
├── config.yaml            # 用户配置（话题、刷新率等）
├── main.py                # 入口
├── core/                  # 数据层
├── ui/                    # 界面层
└── proto/                 # protobuf 消息定义（生成）
    ├── header_pb2.py          # 公共消息头
    ├── joint_state_pb2.py     # JointStateArray 消息
    ├── joint_command_pb2.py   # JointCommandArray 消息
    ├── pose_pb2.py            # PoseStamped 消息
    ├── quaternion_pb2.py      # 四元数定义
    └── vector3_pb2.py         # 三维向量定义
```

## 使用方法

```bash
# 1. 安装依赖
pip install -r monitor/requirements.txt

# 2. （可选）按需修改配置
#    编辑 monitor/config.yaml 中的话题、刷新率等参数

# 3. 启动
python -m monitor.main

# 4. 点击工具栏「● Connect」连接 zenoh 网络
#    数据开始流入后，右击表格单元格 → 选择「Plot …」添加曲线
```

> **注意**：应用启动后默认处于暂停状态（⏸），点击 **▶ 继续** 开始采样和绘图。  
> 按 **Ctrl+C** 可优雅退出。

## 配置参数

| 参数 | 默认值 | 说明 |
|------|--------|------|
| `topics.joint_state` | `/geniarm/joint_state` | 关节实际状态话题 |
| `topics.joint_cmd` | `/geniarm/joint_cmd` | 关节期望指令话题 |
| `topics.pose_current` | `/geniarm/pose_current` | 末端实际位姿话题 |
| `topics.pose_expected` | `/geniarm/pose_expected` | 末端期望位姿话题 |
| `refresh_interval_ms` | `10` | 绘图刷新间隔（ms） |
| `initial_x_window_s` | `10.0` | 初始 X 轴视图宽度（秒） |
| `y_range_min` | `0.001` | Y 轴最小范围，防止微小偏差时曲线不可辨 |
| `csv_export_window_s` | `0` | CSV 导出时间窗口（秒），0 = 不限量 |
| `zenoh_connect_endpoints` | `[]` | 远程 zenoh 端点列表，跨机器时填写 |
| `ui_scale` | `1.0` | UI 缩放因子，跟随系统 DPI |

编辑 `config.yaml` 后重启应用生效。

## 界面功能

### 工具栏

| 操作 | 说明 |
|------|------|
| **● Connect** | 连接 zenoh 网络（peer mode），开始接收数据 |
| **◉ Disconnect** | 断开连接，断开后显示各来源消息统计 |
| **☰ 操作 (N)** | 显示全部 / 隐藏全部曲线（括号内为当前曲线数） |
| **↺ 重置数据** | 清空历史数据，逻辑时钟归零，曲线从 0 重画，**自动继续采样** |
| **✕ 清除曲线** | 移除所有曲线和图例，清空数据，**自动暂停**，等待添加新曲线 |
| **💾 Save CSV** | 将当前绘制曲线的数据导出为 CSV 文件 |
| **📏 选区** | 切换范围选择游标，用于截取特定时间区间 |
| **💾 导出选区** | 仅导出选中时间范围内的数据（需先启用 📏 选区） |
| **▶ 继续 / ⏸ 暂停** | 冻结/恢复数据采样与自动跟随 |

### 右击表格

| 操作 | 说明 |
|------|------|
| **右击关节名** | 为该关节的所有字段（pos / vel / eff）添加/移除曲线 |
| **右击数值列** | 为该关节的单个字段添加/移除曲线 |
| **° / rad 切换** | 每张表格右上角按钮，切换角度/弧度显示 |

### 左侧数据面板

数据面板分四张表格，共用同一垂直滚动条：

| 表格 | 内容 |
|------|------|
| **Joint State** | 各关节实际位置、速度、力矩 |
| **Joint Command** | 各关节期望位置、速度、力矩 |
| **Pose Current** | 末端实际位置（X Y Z）+ 姿态（Roll / Pitch / Yaw） |
| **Pose Expected** | 末端期望位置（X Y Z）+ 姿态（Roll / Pitch / Yaw） |

### 绘图交互

| 操作 | 说明 |
|------|------|
| **单击 X/Y 轴标签** | 选中坐标轴（橙色高亮），滚轮缩放该轴 |
| **单击绘图区空白** | 取消坐标轴选中 |
| **双击绘图区** | 恢复 Y 轴自动范围 |
| **图例左键** | 高亮/取消高亮曲线（加粗 + 置顶 Z-order） |
| **图例右键** | 隐藏 / 显示 / 删除曲线 |
| **图例悬停** | 鼠标变为手型指针 |
| **📏 选区游标** | 两条蓝色竖线 + 灰色区域，可拖拽选择时间范围 |

### 状态栏信息

- **Hz**：实时采样率（每秒刷新）
- **Samples**：每秒消息数统计
- **连接状态**：已连接（绿色）/ 断开（灰色）/ 错误（红色）
- **订阅话题**：当前配置的所有话题表达式
- **⏱ 运行时长**：连接持续时长

## 远程连接

当 monitor 和 robot 运行在不同机器上时：

1. **robot 端**：确保 zenoh 监听 `tcp/0.0.0.0:7447`（配置在 `zenoh_native_config.json5` 的 `listen.endpoints`）
2. **monitor 端**：在 `config.yaml` 中填写 robot IP：
   
   ```yaml
   zenoh_connect_endpoints:
     - "tcp/192.168.1.100:7447"
   ```

同一机器或本地子网无需此配置。

