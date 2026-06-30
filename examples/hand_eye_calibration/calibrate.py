#!/usr/bin/env python3
"""
手眼标定 — 求解与验证

从采集数据文件加载 ``T_base_to_tool`` 和 ``T_camera_to_board``，
求解 $AX=XB$ 得到手眼变换矩阵 $T_{tool\\to camera}$，并输出验证指标。

用法:
    python -m examples.hand_eye_calibration.calibrate
    python -m examples.hand_eye_calibration.calibrate --data calibration_data.json
    python -m examples.hand_eye_calibration.calibrate --method park --output my_result.npy
"""

from __future__ import annotations

import argparse
from pathlib import Path

import cv2
import numpy as np

from hand_eye_calibration.common import (
    load_calibration_data,
    decompose_transform,
    build_transform,
    rotation_error,
    translation_error,
)


# cv2.calibrateHandEye 方法映射
METHODS = {
    "tsai": cv2.CALIB_HAND_EYE_TSAI,
    "park": cv2.CALIB_HAND_EYE_PARK,
    "horaud": cv2.CALIB_HAND_EYE_HORAUD,
    "daniilidis": cv2.CALIB_HAND_EYE_DANIILIDIS,
}


def solve_hand_eye(data: dict,
                   method: int = cv2.CALIB_HAND_EYE_DANIILIDIS,
                   ) -> np.ndarray:
    """求解 $AX=XB$ 得到手眼变换矩阵。

    Args:
        data: 包含 ``T_base_to_tool`` 和 ``T_camera_to_board`` 的字典
        method: OpenCV 标定方法

    Returns:
        4×4 手眼变换矩阵 $T_{tool\\to camera}$
    """
    T_bt_list = data["T_base_to_tool"]
    T_cb_list = data["T_camera_to_board"]
    n = len(T_bt_list)

    R_g2b, t_g2b = [], []   # 末端位姿（基座→末端）
    R_t2c, t_t2c = [], []   # 标定板位姿（相机→标定板）

    for i in range(n):
        R, t = decompose_transform(T_bt_list[i])
        R_g2b.append(R)
        t_g2b.append(t)

        R, t = decompose_transform(T_cb_list[i])
        R_t2c.append(R)
        t_t2c.append(t)

    # 求解 AX=XB
    R_X, t_X = cv2.calibrateHandEye(
        R_g2b, t_g2b,
        R_t2c, t_t2c,
        method=method,
    )

    T = build_transform(R_X, t_X)
    return T


def validate(data: dict, T_tool_to_camera: np.ndarray) -> dict:
    """验证标定结果：计算重投影误差。

    Returns:
        包含各项误差指标的字典
    """
    T_bt_list = data["T_base_to_tool"]
    T_cb_list = data["T_camera_to_board"]
    n = len(T_bt_list)

    # 对每个位姿计算 T_base_to_board = T_bt @ X @ T_cb
    # 理论上应该恒定（标定板固定）
    T_pred_list = []
    for i in range(n):
        T_pred = T_bt_list[i] @ T_tool_to_camera @ T_cb_list[i]
        T_pred_list.append(T_pred)

    # 以中值作为参考值
    T_ref = np.median(T_pred_list, axis=0)

    rot_errs = []
    trans_errs = []
    for T_pred in T_pred_list:
        re = rotation_error(T_pred[:3, :3], T_ref[:3, :3])
        te = translation_error(T_pred[:3, 3], T_ref[:3, 3])
        rot_errs.append(re)
        trans_errs.append(te)

    # 各轴平移误差
    t_ref = T_ref[:3, 3]
    t_errors_axis = []
    for T_pred in T_pred_list:
        t_errors_axis.append(np.abs(T_pred[:3, 3] - t_ref) * 1000)  # mm
    t_errors_axis = np.array(t_errors_axis)

    return {
        "T_tool_to_camera": T_tool_to_camera,
        "rot_errors_deg": np.array(rot_errs),
        "trans_errors_mm": np.array(trans_errs),
        "trans_errors_x_mm": t_errors_axis[:, 0],
        "trans_errors_y_mm": t_errors_axis[:, 1],
        "trans_errors_z_mm": t_errors_axis[:, 2],
        "rot_mean": float(np.mean(rot_errs)),
        "rot_max": float(np.max(rot_errs)),
        "trans_mean": float(np.mean(trans_errs)),
        "trans_max": float(np.max(trans_errs)),
    }


def print_results(results: dict):
    """格式化打印标定结果。"""
    T = results["T_tool_to_camera"]

    print(f"\n{'='*60}")
    print("标定结果")
    print(f"{'='*60}")
    print(f"\n手眼变换矩阵 T_tool_to_camera:")
    print(f"  [[{T[0,0]:.6f}, {T[0,1]:.6f}, {T[0,2]:.6f}, {T[0,3]:.6f}],")
    print(f"   [{T[1,0]:.6f}, {T[1,1]:.6f}, {T[1,2]:.6f}, {T[1,3]:.6f}],")
    print(f"   [{T[2,0]:.6f}, {T[2,1]:.6f}, {T[2,2]:.6f}, {T[2,3]:.6f}],")
    print(f"   [{T[3,0]:.6f}, {T[3,1]:.6f}, {T[3,2]:.6f}, {T[3,3]:.6f}]]")

    print(f"\n{'='*60}")
    print("验证结果（重投影误差）")
    print(f"{'='*60}")
    print(f"  旋转误差:")
    print(f"    均值: {results['rot_mean']:.3f}°")
    print(f"    最大: {results['rot_max']:.3f}°")
    print(f"    标准差: {np.std(results['rot_errors_deg']):.3f}°")
    print(f"  平移误差:")
    print(f"    均值: {results['trans_mean']:.3f} mm")
    print(f"    最大: {results['trans_max']:.3f} mm")
    print(f"    标准差: {np.std(results['trans_errors_mm']):.3f} mm")
    print(f"  X轴平移误差: 均值={np.mean(results['trans_errors_x_mm']):.3f} mm")
    print(f"  Y轴平移误差: 均值={np.mean(results['trans_errors_y_mm']):.3f} mm")
    print(f"  Z轴平移误差: 均值={np.mean(results['trans_errors_z_mm']):.3f} mm")

    # 判定
    print(f"\n{'='*60}")
    ok_rot = results["rot_mean"] < 1.0
    ok_trans = results["trans_mean"] < 10.0
    if ok_rot and ok_trans:
        print("✅ 标定结果合格！")
    else:
        if not ok_rot:
            print(f"⚠️  旋转误差偏大 ({results['rot_mean']:.2f}° > 1.0°)，建议增加姿态数重采")
        if not ok_trans:
            print(f"⚠️  平移误差偏大 ({results['trans_mean']:.2f} mm > 10 mm)，建议检查标定板检测精度")
    print(f"{'='*60}\n")


def main():
    parser = argparse.ArgumentParser(description="手眼标定求解")
    parser.add_argument("--data", type=str, default="calibration_data.json",
                        help="采集数据文件路径")
    parser.add_argument("--method", type=str, default="daniilidis",
                        choices=list(METHODS.keys()),
                        help="标定求解方法 (默认 daniilidis)")
    parser.add_argument("--output", type=str, default="T_tool_to_camera.npy",
                        help="结果输出路径 (.npy)")
    args = parser.parse_args()

    # ── 加载数据 ────────────────────────────────────────────────────
    data_path = Path(args.data)
    if not data_path.exists():
        print(f"❌ 数据文件不存在: {data_path}")
        print(f"   请先运行采集脚本: python -m examples.hand_eye_calibration.collect_data")
        return

    data = load_calibration_data(data_path)
    n = len(data["T_base_to_tool"])
    if n < 3:
        print(f"❌ 有效数据不足 {n} 组，至少需要 3 组")
        return

    # ── 求解 ────────────────────────────────────────────────────────
    method = METHODS[args.method]
    method_name = args.method
    print(f"\n求解方法: {method_name}")

    T = solve_hand_eye(data, method=method)

    # ── 验证 ────────────────────────────────────────────────────────
    results = validate(data, T)
    print_results(results)

    # ── 保存 ────────────────────────────────────────────────────────
    np.save(args.output, T)
    print(f"结果已保存到 {args.output}")
    print(f"  加载方式: T = np.load('{args.output}')")

    # 同时输出在程序中如何使用
    print(f"\n{'='*60}")
    print("在代码中使用:")
    print(f"{'='*60}")
    print(f"""
    import numpy as np
    T_tool_to_camera = np.load('{args.output}')

    # 相机中看到一个点 P_cam (x, y, z)，转换到基座标系:
    P_cam = np.array([x, y, z, 1.0])
    P_base = T_base_to_tool @ T_tool_to_camera @ P_cam
    # P_base[:3] 即为基座标系下的坐标
""")


if __name__ == "__main__":
    main()
