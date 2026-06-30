# 静力学标定 — 数据采集脚本

逐关节采集机械臂在不同位姿下的关节角度与力矩数据，用于辨识动力学参数。

## 使用方法

### 1. 启动机器人

确保 GeniArm 控制程序已启动。

### 2. 启动 Monitor

```bash
python -m monitor.main
```

点击工具栏 **「● Connect」** 连接，再点击 **「▶ 继续」** 开始采样。

### 3. 运行采集脚本

每个关节一个脚本，按顺序依次执行：

```bash
# 关节 1（肩关节旋转） — 约 2 分钟
python examples/calibration/j1_collect.py

# 关节 2（肩关节俯仰）⭐ 最关键 — 约 5 分钟
python examples/calibration/j2_collect.py

# 关节 3（肘关节）— 约 8 分钟（两组 q₂ 位姿）
python examples/calibration/j3_collect.py

# 关节 4~6（腕关节）— 各约 2 分钟
python examples/calibration/j4_collect.py
python examples/calibration/j5_collect.py
python examples/calibration/j6_collect.py
```

### 4. 导出数据

每个脚本运行完毕后，在 monitor 中：

1. 右击数据表格 → 选择要记录的字段（position + effort）
2. 点击 **「💾 Save CSV」**
3. 建议文件名：`calibration_j1.csv` ~ `calibration_j6.csv`

### 5. 重复数据验证（推荐）

为了验证参数的可重复性，建议对 **关节 2 或关节 3** 重复采集一次，
对比两次的力矩曲线是否一致。

## 文件说明

| 文件 | 说明 |
|------|------|
| `common.py` | 公共模块（RPC 调用、角度转换） |
| `j1_collect.py` | 关节 1：-150° ~ +150°，步进 10°，31 点/方向 |
| `j2_collect.py` | 关节 2：5° ~ 180°，步进 5°，36 点/方向 ⭐ |
| `j3_collect.py` | 关节 3：-150° ~ -5°，步进 5°，两组 q₂ 位姿 |
| `j4_collect.py` | 关节 4：-90° ~ +90°，步进 15°，13 点/方向 |
| `j5_collect.py` | 关节 5：-90° ~ +90°，步进 15°，13 点/方向 |
| `j6_collect.py` | 关节 6：-90° ~ +90°，步进 15°，13 点/方向 |

## 输出 CSV 格式说明

monitor 导出的 CSV 应包含以下字段（每行一个采样点）：

```
timestamp, j1_pos, j2_pos, j3_pos, j4_pos, j5_pos, j6_pos,
           j1_eff, j2_eff, j3_eff, j4_eff, j5_eff, j6_eff
```

> 注意：如果 monitor 导出的格式不同，需要在后续参数辨识脚本中做相应调整。
