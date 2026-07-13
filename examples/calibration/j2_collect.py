#!/usr/bin/env python3
"""
关节 2 数据采集 — 肩关节俯仰（最关键）

这是标定中最重要的关节：
  - 承载连杆 2~6 的全部重力（总质量 ≈ 7.9 kg）
  - 力矩信号最强（峰值约 11.6 N·m）

为避免与关节 3 参数耦合，在 3 组不同的 q3 位姿下分别采集。
每组 q3 配独立的 j2 范围（肘弯越大，j2 碰撞风险越高，范围需收缩）：
  - q3 =   0°  → j2   5° ~  80°
  - q3 = -60°  → j2   5° ~  50°
  - q3 = -120° → j2   5° ~  30°
"""

import sys
import os
sys.path.insert(0, os.path.dirname(__file__))

from common import movej, deg2rad, wait_stable, print_step, go_home, wait_for_save

# ---- 配置参数 ----------------------------------------------------------
JOINT_ID = 2
STEP_DEG = 5.0      # 步进（度）
WAIT_SEC = 1.0      # 每步等待稳定时间（秒），含大臂大姿态变化
VEL = 0.15          # MoveJ 速度 (rad/s) — 低速，确保准静态
ACC = 0.3           # MoveJ 加速度 (rad/s²) — 小加速度，减少惯性力

# 每组 (q3 固定值, j2 起始角, j2 终止角)
# j2 5°~80° 仅在 q3=0° 时安全，肘弯后需缩范围
Q3_CONFIG = [
    (-60.0,  5.0,  125.0),
    (-120.0, 5.0,  155.0),
    (-180.0, 5.0,  175.0),
]

# ---- 计算角度序列 ------------------------------------------------------
import numpy as np


def generate_angles(start_deg, end_deg, step_deg):
    fwd = np.arange(start_deg, end_deg + step_deg / 2, step_deg)
    rev = np.arange(end_deg, start_deg - step_deg / 2, -step_deg)
    return fwd, rev


def collect_pass(q3_fix_deg: float, angles_fwd, angles_rev, count_start: int, total: int) -> int:
    """采集一轮（正向+反向）"""
    count = count_start

    print(f"\n--- q₃ = {q3_fix_deg:5.1f}° ---")
    print(">>> 正向采集（递增）")
    print("-" * 40)

    FIXED_POSE = [0.0, None, deg2rad(q3_fix_deg), 0.0, 0.0, 0.0]
    #              J1,   J2,    J3,           J4,  J5,  J6
    #              0°,  (动),  固定值,        0°,  0°,  0°

    for deg in angles_fwd:
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
    for deg in angles_rev:
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
    # 统计总点数
    n_config = len(Q3_CONFIG)
    total = 0
    for _, start_deg, end_deg in Q3_CONFIG:
        fwd, rev = generate_angles(start_deg, end_deg, STEP_DEG)
        total += len(fwd) + len(rev)

    print("=" * 60)
    print("静力学标定 — 关节 2 数据采集 ⭐")
    print("=" * 60)
    print(f"  步进: {STEP_DEG:.0f}°")
    print(f"  姿态数: {n_config} 组")
    for q3, s, e in Q3_CONFIG:
        n_pts = int((e - s) / STEP_DEG) * 2
        print(f"    q₃ = {q3:+.0f}°  →  j₂ = {s:.0f}° ~ {e:.0f}°  ({n_pts} 点)")
    print(f"  总计: {total} 点")
    print(f"  等待时间: {WAIT_SEC}s/点")
    print(f"  速度: {VEL} rad/s  加速度: {ACC} rad/s²")
    print()

    print("请在 monitor 中点击「● Connect」并「▶ 继续」开始记录")
    print()

    count = 0
    for idx, (q3_fix, start_deg, end_deg) in enumerate(Q3_CONFIG):
        print(f"\n{'='*60}")
        print(f"第 {idx+1}/{n_config} 段：q₃ = {q3_fix:+.0f}°")

        # 回家 → 移到本段起始位姿
        print(">>> 回到 home ...")
        if not go_home():
            return
        print(">>> 移动到本段起始姿态 ...")
        start_pose = [0.0, deg2rad(start_deg), deg2rad(q3_fix),
                      0.0, 0.0, 0.0]
        if not movej(start_pose, v=0.3, a=0.5):
            print("    [ERROR] 移动到起始位姿失败")
            return
        print(">>> 已就位\n")

        input(f"准备好后按 Enter 开始采集第 {idx+1} 段 ...\n")

        fwd, rev = generate_angles(start_deg, end_deg, STEP_DEG)
        count = collect_pass(q3_fix, fwd, rev, count, total)
        if count < 0:
            return

    # ---- 保存数据 & 回 home ---------------------------------------------
    print()
    print(f"✅ 关节 {JOINT_ID} 采集完成！共 {count} 个点")
    wait_for_save()

    go_home()
    print("建议文件名: calibration_j2.csv")


if __name__ == "__main__":
    main()
    main()
