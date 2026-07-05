#!/usr/bin/env python3
"""
关节 5 数据采集 — 腕关节偏转

腕部连杆质量小、力臂短，信号较弱。
采取水平伸出位姿（q₂=90°, q₃=-90°）放大重力效应。

限位: -90° ~ +90°
步进: 15°
点数: 26 个（正反向各 13）
"""

import sys
import os
sys.path.insert(0, os.path.dirname(__file__))

from common import movej, deg2rad, wait_stable, print_step, go_home, wait_for_save

# ---- 配置参数 ----------------------------------------------------------
JOINT_ID = 5
START_DEG = -90.0    # 起始角度（度）
END_DEG = 90.0       # 终止角度（度）
STEP_DEG = 15.0      # 步进（度）
WAIT_SEC = 1.0       # 每步等待稳定时间（秒）
VEL = 0.1             # MoveJ 速度 (rad/s) — 超低速，信号弱惯性噪声影响大
ACC = 0.2             # MoveJ 加速度 (rad/s²)

# 水平伸出位姿，放大腕部重力效应
FIXED_POSE = [0.0, deg2rad(90.0), deg2rad(-90.0), 0.0, None, 0.0]
#              J1,      J2,          J3,          J4,   J5,  J6
#              0°,     90°,         -90°,         0°,  (动),  0°

# ---- 计算角度序列 ------------------------------------------------------
import numpy as np
angles_fwd = np.arange(START_DEG, END_DEG + STEP_DEG / 2, STEP_DEG)
angles_rev = np.arange(END_DEG, START_DEG - STEP_DEG / 2, -STEP_DEG)
total = len(angles_fwd) + len(angles_rev)


def main():
    print("=" * 60)
    print("静力学标定 — 关节 5 数据采集")
    print("=" * 60)
    print(f"  范围: {START_DEG:.0f}° ~ {END_DEG:.0f}°  步进: {STEP_DEG:.0f}°")
    print(f"  正向点数: {len(angles_fwd)}  反向点数: {len(angles_rev)}")
    print(f"  总计采点: {total}")
    print(f"  等待时间: {WAIT_SEC}s/点  (信号弱，延长等待)")
    print(f"  速度: {VEL} rad/s  加速度: {ACC} rad/s²")
    print(f"  位姿: 手臂水平伸出 (q₂=90°, q₃=-90°)")
    print()

    # ---- 预先移动到起始位姿（不计入数据）----------------------------------
    if not go_home():
        return
    print(">>> 移动到腕部采集起始位姿 ...")
    start_pose = [0.0, deg2rad(90.0), deg2rad(-90.0),
                  0.0, deg2rad(START_DEG), 0.0]
    if not movej(start_pose, v=0.3, a=0.5):
        print("    [ERROR] 移动到起始位姿失败")
        return
    wait_stable(1.0)

    print(f"请在 monitor 中点击「● Connect」并「▶ 继续」开始记录")
    input("准备好后按 Enter 开始采集 ...\n")

    count = 0

    # ---- 正向采集 ------------------------------------------------------
    print("\n>>> 正向采集（递增）")
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
    print("建议文件名: calibration_j5.csv")


if __name__ == "__main__":
    main()
