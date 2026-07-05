#!/usr/bin/env python3
"""
关节 2 数据采集 — 肩关节俯仰（最关键）

这是标定中最重要的关节：
  - 承载连杆 2~6 的全部重力（总质量 ≈ 7.9 kg）
  - 力矩信号最强（峰值约 11.6 N·m）
  - τ₂ 呈正弦变化，τ₃ 也随 q₂ 摆动，提供耦合约束

限位: 0° ~ 212°（安全取值 5° ~ 180°）
步进: 5°
点数: 72 个（正反向各 36）
"""

import sys
import os
sys.path.insert(0, os.path.dirname(__file__))

from common import movej, deg2rad, wait_stable, print_step, go_home, wait_for_save

# ---- 配置参数 ----------------------------------------------------------
JOINT_ID = 2
START_DEG = 5.0     # 起始角度（度），留安全余量
END_DEG = 80.0     # 终止角度（度），留安全余量
STEP_DEG = 5.0      # 步进（度）
WAIT_SEC = 0.8      # 每步等待稳定时间（秒）— 手臂姿态变化大，多等一会
VEL = 0.15          # MoveJ 速度 (rad/s) — 低速，确保准静态
ACC = 0.3            # MoveJ 加速度 (rad/s²) — 小加速度，减少惯性力

# 固定其他关节的位姿
FIXED_POSE = [0.0, None, 0.0, 0.0, 0.0, 0.0]
#              J1,   J2,  J3,  J4,  J5,  J6
#              0°,  (动), 0°,  0°,  0°,  0°

# ---- 计算角度序列 ------------------------------------------------------
import numpy as np
angles_fwd = np.arange(START_DEG, END_DEG + STEP_DEG / 2, STEP_DEG)
angles_rev = np.arange(END_DEG, START_DEG - STEP_DEG / 2, -STEP_DEG)
total = len(angles_fwd) + len(angles_rev)


def main():
    print("=" * 60)
    print("静力学标定 — 关节 2 数据采集 ⭐")
    print("=" * 60)
    print(f"  范围: {START_DEG:.0f}° ~ {END_DEG:.0f}°  步进: {STEP_DEG:.0f}°")
    print(f"  正向点数: {len(angles_fwd)}  反向点数: {len(angles_rev)}")
    print(f"  总计采点: {total}")
    print(f"  等待时间: {WAIT_SEC}s/点")
    print(f"  速度: {VEL} rad/s  加速度: {ACC} rad/s²")
    print(f"\n警告: 关节 2 力矩变化大，请确保周围无障碍物！")
    print()

    # ---- 预移到起始角（不计入数据）----------------------------------------
    start_pose = list(FIXED_POSE)
    start_pose[JOINT_ID - 1] = deg2rad(START_DEG)
    print(f"\n>>> 预移到起始位置 J{JOINT_ID}={START_DEG:.0f}° ...")
    if not movej(start_pose, v=VEL, a=ACC):
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
    print("建议文件名: calibration_j2.csv")


if __name__ == "__main__":
    main()
