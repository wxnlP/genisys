# ArmMonitor

实时机械臂关节状态监控工具。通过 zenoh 订阅机器人发布的 JointStateArray 数据，以表格和曲线图实时展示各关节的位置、速度、力矩。

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
    ├── header_pb2.py      # protobuf 定义（生成）
    └── joint_state_pb2.py
```

## 使用方法

```bash
# 1. 安装依赖
pip install -r requirements.txt

# 2. （可选）按需修改配置
#    编辑 config.yaml 中的 topic、刷新间隔等参数

# 3. 启动
python -m monitor.main

# 4. 点击工具栏「Connect」连接 zenoh 网络
#    数据开始流入后，右击表格单元格 → 选择「Plot …」添加曲线
```

## 配置参数

| 参数 | 默认值 | 说明 |
|------|--------|------|
| `topics.joint_state` | `/geniarm/joint_state` | 关节实际状态话题 |
| `topics.joint_cmd` | `/geniarm/joint_cmd` | 关节期望指令话题 |
| `refresh_interval_ms` | `33` | 绘图刷新间隔（ms），33ms ≈ 30fps |
| `initial_x_window_s` | `10.0` | 初始 X 轴视图宽度（秒） |

编辑 `config.yaml` 后重启应用生效。

## 界面功能

| 操作 | 说明 |
|------|------|
| **Connect** | 连接 zenoh 网络，开始接收数据 |
| **Disconnect** | 断开连接 |
| **右击表格** | 选择要绘制的关节数据字段 |
| **↺ 重置数据** | 清空历史数据，曲线从 0 重画 |
| **✕ 清除曲线** | 移除所有曲线和图例，数据保留 |
| **☰ 操作** | 显示全部/隐藏全部曲线 |
| **⏸ 暂停/继续** | 冻结画面，自由浏览历史数据 |
| **单击 X/Y 轴** | 选中坐标轴（橙色高亮），滚轮缩放该轴 |
| **单击绘图区** | 取消坐标轴选中 |
| **图例左键** | 高亮/取消高亮曲线 |
| **图例右键** | 隐藏/显示/删除曲线 |

## 数据存储

数据**不限量**累积，点击 **↺ 重置数据** 手动清空。
长时间运行注意内存 —— 定期重置即可。
