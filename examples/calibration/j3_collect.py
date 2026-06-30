#!/usr/bin/env python3
"""
关节 3 数据采集 — 肘关节

关键技巧：在两组不同的 q₂ 位姿下分别采集，
以避免 q₂ 和 q₃ 的参数耦合。

限位: -212° ~ 0°（安全取值 -150° ~ -5°）
步进: 5°
组数: 2 组（q₂=45°, q₂=90°）
点数: 约 120 个
"""

import sys
import os
sys.path.insert(0, os.path.dirname(__file__))

from common import movej, deg2rad, wait_stable, print_step, go_home

# ---- 配置参数 ----------------------------------------------------------
JOINT_ID = 3
START_DEG = -150.0   # 起始角度（度），避免底部干涉
END_DEG = -5.0       # 终止角度（度），留安全余量
STEP_DEG = 5.0       # 步进（度）
WAIT_SEC = 0.8       # 每步等待稳定时间（秒）
VEL = 0.15           # MoveJ 速度 (rad/s) — 低速，确保准静态
ACC = 0.3             # MoveJ 加速度 (rad/s²) — 小加速度，减少惯性力

# 两组不同的 q₂ 固定位姿（避免参数耦合）
Q2_FIX_VALUES = [45.0, 90.0]

# ---- 计算角度序列 ------------------------------------------------------
import numpy as np
angles_fwd = np.arange(START_DEG, END_DEG + STEP_DEG / 2, STEP_DEG)
angles_rev = np.arange(END_DEG, START_DEG - STEP_DEG / 2, -STEP_DEG)
points_per_pass = len(angles_fwd) + len(angles_rev)
total = points_per_pass * len(Q2_FIX_VALUES)


def collect_pass(q2_fix_deg: str, angles_fwd, angles_rev, count_start: int) -> int:
    """采集一轮（正向+反向）"""
    count = count_start

    print(f"\n--- q₂ = {q2_fix_deg:5.1f}° ---")
    print(">>> 正向采集（递增）")
    print("-" * 40)

    FIXED_POSE = [0.0, deg2rad(q2_fix_deg), None, 0.0, 0.0, 0.0]
    #              J1,       J2,           J3,    J4,  J5,  J6
    #              0°,      固定值,        (动),  0°,  0°,  0°

    for i, deg in enumerate(angles_fwd):
        target = list(FIXED_POSE)
        target[JOINT_ID - 1] = deg2rad(deg)
        count += 1
        print_step(JOINT_ID, deg, count, total)
        if not movej(target, v=VEL, a=ACC):
            print("    [ERROR] 运动失败，终止")
            return -1
        wait_stable(WAIT_SEC)

    print("\n>>> 反向采集（递减）")
    print("-" * 40)
    for i, deg in enumerate(angles_rev):
        target = list(FIXED_POSE)
        target[JOINT_ID - 1] = deg2rad(deg)
        count += 1
        print_step(JOINT_ID, deg, count, total)
        if not movej(target, v=VEL, a=ACC):
            print("    [ERROR] 运动失败，终止")
            return -1
        wait_stable(WAIT_SEC)

    return count


def main():
    print("=" * 60)
    print("静力学标定 — 关节 3 数据采集")
    print("=" * 60)
    print(f"  范围: {START_DEG:.0f}° ~ {END_DEG:.0f}°  步进: {STEP_DEG:.0f}°")
    print(f"  q₂ 固定位姿: {Q2_FIX_VALUES}")
    print(f"  每组点数: {points_per_pass}  总点数: {total}")
    print(f"  等待时间: {WAIT_SEC}s/点")
    print(f"  速度: {VEL} rad/s  加速度: {ACC} rad/s²")
    print(f"\n请在 monitor 中点击「● Connect」并「▶ 继续」开始记录")
    input("准备好后按 Enter 开始采集 ...")
    print()

    # ---- 回 home -------------------------------------------------------
    if not go_home():
        return

    count = 0

    # ---- 分两组采集 ----------------------------------------------------
    for q2_fix in Q2_FIX_VALUES:
        # 先移动到该组的起始姿态
        print(f"\n>>> 移动到 q₂ = {q2_fix:.0f}° 的起始位姿 ...")
        start_pose = [0.0, deg2rad(q2_fix), deg2rad(START_DEG), 0.0, 0.0, 0.0]
        if not movej(start_pose, v=0.3, a=0.5):
            print("    [ERROR] 移动到起始位姿失败")
            return
        wait_stable(1.0)

        count = collect_pass(q2_fix, angles_fwd, angles_rev, count)
        if count < 0:
            return

    # ---- 回 home -------------------------------------------------------
    print()
    go_home()
    print("=" * 60)
    print(f"✅ 关节 {JOINT_ID} 采集完成！共 {count} 个点")
    print("=" * 60)
    print("请在 monitor 中点击「💾 Save CSV」导出数据，")
    print("建议文件名: calibration_j3.csv")


if __name__ == "__main__":
    main()
