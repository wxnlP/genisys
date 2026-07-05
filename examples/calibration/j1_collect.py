#!/usr/bin/env python3
"""
关节 1 数据采集 — 肩关节旋转

操作方式：
  1. 启动本脚本
  2. 在 monitor 中点击「● Connect」连接
  3. 点击「▶ 继续」开始采样
  4. 观察本脚本输出，等待采集完成
  5. 在 monitor 中右击表格 → 选择要记录的字段
  6. 点击「💾 Save CSV」导出数据

关节 1 绕竖直轴旋转，重力矩理论上不变，
主要用于验证数据一致性和检测零偏。

限位: -150° ~ +150°
步进: 10°
点数: 31 个
"""

import sys
import os
sys.path.insert(0, os.path.dirname(__file__))

from common import movej, deg2rad, wait_stable, print_step, go_home, wait_for_save

# ---- 配置参数 ----------------------------------------------------------
JOINT_ID = 1
START_DEG = -150.0  # 起始角度（度）
END_DEG = 150.0     # 终止角度（度）
STEP_DEG = 10.0     # 步进（度）
WAIT_SEC = 0.8      # 每步等待稳定时间（秒）
VEL = 0.15          # MoveJ 速度 (rad/s) — 低速，确保准静态
ACC = 0.3            # MoveJ 加速度 (rad/s²) — 小加速度，减少惯性力

# 固定其他关节的位姿
FIXED_POSE = [0.0, 0.0, 0.0, 0.0, 0.0, 0.0]
#              J1,  J2,  J3,  J4,  J5,  J6
#             (无,  0°,  0°,  0°,  0°,  0°)
#             J1 由循环控制，None 表示被循环覆盖

# ---- 计算角度序列 ------------------------------------------------------
import numpy as np
angles_fwd = np.arange(START_DEG, END_DEG + STEP_DEG / 2, STEP_DEG)
angles_rev = np.arange(END_DEG, START_DEG - STEP_DEG / 2, -STEP_DEG)
total = len(angles_fwd) + len(angles_rev)


def main():
    print("=" * 60)
    print("静力学标定 — 关节 1 数据采集")
    print("=" * 60)
    print(f"  范围: {START_DEG:.0f}° ~ {END_DEG:.0f}°  步进: {STEP_DEG:.0f}°")
    print(f"  正向点数: {len(angles_fwd)}  反向点数: {len(angles_rev)}")
    print(f"  总计采点: {total}")
    print(f"  等待时间: {WAIT_SEC}s/点")
    print(f"  速度: {VEL} rad/s  加速度: {ACC} rad/s²")
    print()

    # ---- 预移到起始角（不计入数据）----------------------------------------
    start_pose = list(FIXED_POSE)
    start_pose[JOINT_ID - 1] = deg2rad(START_DEG)
    print(f"\n>>> 预移到起始位置 J{JOINT_ID}={START_DEG:.0f}° ...")
    if not movej(start_pose, v=1, a=0.5):
        print("    [ERROR] 预移失败，终止")
        return
    print(">>> 已就位\n")

    print(f"请在 monitor 中点击「● Connect」并「▶ 继续」开始记录")
    input("准备好后按 Enter 开始采集 ...\n")

    count = 0

    # ---- 正向采集 ------------------------------------------------------
    print(">>> 正向采集（递增）")
    print("-" * 40)
    for i, deg in enumerate(angles_fwd):
        target = list(FIXED_POSE)
        target[JOINT_ID - 1] = deg2rad(deg)
        count += 1
        print_step(JOINT_ID, deg, count, total)
        if not movej(target, v=VEL, a=ACC):
            print("    [ERROR] 运动失败，终止")
            return
        wait_stable(WAIT_SEC)

    # ---- 反向采集 ------------------------------------------------------
    print("\n>>> 反向采集（递减，检查迟滞）")
    print("-" * 40)
    for i, deg in enumerate(angles_rev):
        target = list(FIXED_POSE)
        target[JOINT_ID - 1] = deg2rad(deg)
        count += 1
        print_step(JOINT_ID, deg, count, total)
        if not movej(target, v=VEL, a=ACC):
            print("    [ERROR] 运动失败，终止")
            return
        wait_stable(WAIT_SEC)

    # ---- 保存数据 & 回 home ---------------------------------------------
    print()
    print(f"✅ 关节 {JOINT_ID} 采集完成！共 {count} 个点")
    wait_for_save()

    go_home()
    print("建议文件名: calibration_j1.csv")


if __name__ == "__main__":
    main()
