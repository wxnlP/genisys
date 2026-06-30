#!/usr/bin/env python3
"""
手眼标定 — 数据采集

工作流程:
  1. 准备好棋盘格标定板，固定在机械臂前方
  2. 确保 robot 程序已启动，Zenoh 网络正常
  3. 运行本脚本：
     python -m examples.hand_eye_calibration.collect_data
  4. 机械臂自动走 20 个随机姿态，每步拍照并记录位姿
  5. 数据保存到 ``calibration_data.json``

依赖:
    pip install opencv-python opencv-contrib-python eclipse-zenoh protobuf numpy
"""

from __future__ import annotations

import argparse
import time
from pathlib import Path

import cv2
import numpy as np

from hand_eye_calibration.common import (
    PoseReader,
    go_home,
    movej,
    save_calibration_data,
    get_board_object_points,
    build_transform,
    CHECKERBOARD,
    SQUARE_SIZE,
)


# ── 默认参数 ──────────────────────────────────────────────────────────
NUM_POSES = 20          # 采集姿态数
WAIT_SEC = 0.8          # 每步等待稳定时间（秒）
MOVE_VEL = 0.3          # MoveJ 速度 (rad/s)
MOVE_ACC = 0.5          # MoveJ 加速度 (rad/s²)

# Home 附近的随机范围（弧度）
RANDOM_RANGE = {
    "j1": (-0.8, 0.8),
    "j2": (-1.0, 1.5),   # 大幅变化 → 改变相机位姿
    "j3": (-2.0, 0.5),   # 大幅变化
    "j4": (-1.0, 1.0),
    "j5": (-0.5, 0.5),
    "j6": (-0.5, 0.5),
}


def generate_poses(n: int, seed: int = 42) -> list[list[float]]:
    """生成 n 个随机关节角度姿态（基于 home 附近）。"""
    rng = np.random.default_rng(seed)
    poses = []
    for _ in range(n):
        q = [
            rng.uniform(*RANDOM_RANGE["j1"]),
            rng.uniform(*RANDOM_RANGE["j2"]),
            rng.uniform(*RANDOM_RANGE["j3"]),
            rng.uniform(*RANDOM_RANGE["j4"]),
            rng.uniform(*RANDOM_RANGE["j5"]),
            rng.uniform(*RANDOM_RANGE["j6"]),
        ]
        poses.append(q)
    return poses


def detect_board(image, K, dist, objp):
    """检测棋盘格并用 PnP 解算 T_camera_to_board。

    Returns:
        (T_camera_to_board: 4×4, corners) 或 (None, None)
    """
    gray = cv2.cvtColor(image, cv2.COLOR_BGR2GRAY)
    ret, corners = cv2.findChessboardCorners(gray, CHECKERBOARD, None)
    if not ret:
        return None, None

    # 亚像素精化
    criteria = (cv2.TERM_CRITERIA_EPS + cv2.TERM_CRITERIA_MAX_ITER, 30, 0.001)
    corners = cv2.cornerSubPix(gray, corners, (11, 11), (-1, -1), criteria)

    # PnP
    ret, rvec, tvec = cv2.solvePnP(objp, corners, K, dist)
    if not ret:
        return None, None

    R, _ = cv2.Rodrigues(rvec)
    return build_transform(R, tvec), corners


def draw_board(image, corners, T_camera_to_board, K, dist):
    """在图像上绘制标定板检测结果（调试用）。"""
    img = image.copy()
    if corners is not None:
        cv2.drawChessboardCorners(img, CHECKERBOARD, corners, True)
    if T_camera_to_board is not None:
        # 绘制坐标系轴
        axis_length = 0.05  # 5cm
        axis_pts = np.float32([
            [0, 0, 0],
            [axis_length, 0, 0],
            [0, axis_length, 0],
            [0, 0, axis_length],
        ]).reshape(-1, 3)
        rvec, _ = cv2.Rodrigues(T_camera_to_board[:3, :3])
        tvec = T_camera_to_board[:3, 3]
        img_pts, _ = cv2.projectPoints(axis_pts, rvec, tvec, K, dist)
        img_pts = img_pts.reshape(-1, 2).astype(int)
        # 画轴
        origin = tuple(img_pts[0])
        cv2.line(img, origin, tuple(img_pts[1]), (0, 0, 255), 3)  # X: 红色
        cv2.line(img, origin, tuple(img_pts[2]), (0, 255, 0), 3)  # Y: 绿色
        cv2.line(img, origin, tuple(img_pts[3]), (255, 0, 0), 3)  # Z: 蓝色
    return img


def main():
    parser = argparse.ArgumentParser(description="手眼标定数据采集")
    parser.add_argument("--camera-id", type=int, default=0,
                        help="相机设备 ID (默认 0)")
    parser.add_argument("--num-poses", type=int, default=NUM_POSES,
                        help=f"采集姿态数 (默认 {NUM_POSES})")
    parser.add_argument("--output", type=str, default="calibration_data.json",
                        help="输出文件路径")
    parser.add_argument("--image-dir", type=str, default="calib_images",
                        help="保存图像的目录 (默认 calib_images/)")
    parser.add_argument("--camera-matrix", type=str,
                        default="camera_matrix.npy",
                        help="相机内参矩阵文件 (.npy)")
    parser.add_argument("--dist-coeffs", type=str, default="dist_coeffs.npy",
                        help="畸变系数文件 (.npy)")
    parser.add_argument("--square-size", type=float, default=SQUARE_SIZE,
                        help="棋盘格格边长 (米)")
    parser.add_argument("--show-preview", action="store_true",
                        help="显示实时预览窗口")
    args = parser.parse_args()

    output_path = Path(args.output)
    image_dir = Path(args.image_dir)
    image_dir.mkdir(exist_ok=True)

    # ── 加载相机内参 ────────────────────────────────────────────────
    print("=" * 60)
    print("手眼标定 — 数据采集")
    print("=" * 60)
    print(f"\n加载相机内参: {args.camera_matrix}, {args.dist_coeffs}")
    try:
        K = np.load(args.camera_matrix)
        dist = np.load(args.dist_coeffs)
        print(f"  内参矩阵 K:\n{K}")
    except FileNotFoundError as e:
        print(f"❌ 未找到相机内参文件: {e}")
        print("   请先用内参标定脚本标定相机，生成 .npy 文件。")
        return

    # ── 连接机械臂与 Zenoh ──────────────────────────────────────────
    print("\n连接机械臂和 Zenoh ...")
    try:
        pose_reader = PoseReader()
        print("  ✅ 已连接到 Zenoh，可读取末端位姿")
    except Exception as e:
        print(f"  ❌ 连接失败: {e}")
        return

    # ── 打开相机 ────────────────────────────────────────────────────
    print(f"\n打开相机 (ID={args.camera_id}) ...")
    cap = cv2.VideoCapture(args.camera_id)
    if not cap.isOpened():
        print("  ❌ 无法打开相机")
        pose_reader.stop()
        return
    print("  ✅ 相机已打开")

    # ── 准备标定板物点 ──────────────────────────────────────────────
    objp = get_board_object_points(square_size=args.square_size)
    print(f"\n标定板: {CHECKERBOARD[0]}×{CHECKERBOARD[1]} 内角点, "
          f"格边长 {args.square_size*1000:.0f} mm")

    # ── 生成姿态 ────────────────────────────────────────────────────
    poses = generate_poses(args.num_poses)
    print(f"计划采集 {len(poses)} 个姿态")

    # ── 回 home ─────────────────────────────────────────────────────
    if not go_home():
        print("❌ 回 home 失败，退出")
        cap.release()
        pose_reader.stop()
        return

    # ── 采集主循环 ──────────────────────────────────────────────────
    T_base_to_tool_list = []
    T_camera_to_board_list = []
    image_paths = []
    success_count = 0

    input("\n准备好后按 Enter 开始采集 ...\n")

    for i, joint_target in enumerate(poses):
        print(f"\n--- 姿态 {i+1}/{len(poses)} ---")

        # 1. 运动到目标关节角度
        print(f"  运动到: {[f'{q:.3f}' for q in joint_target]}")
        if not movej(joint_target, v=MOVE_VEL, a=MOVE_ACC):
            print(f"  ⚠️ 运动失败，跳过")
            continue
        time.sleep(WAIT_SEC)

        # 2. 读取末端位姿
        try:
            T_base_to_tool = pose_reader.read()
            print(f"  末端位姿: pos=({T_base_to_tool[0,3]:.4f}, "
                  f"{T_base_to_tool[1,3]:.4f}, {T_base_to_tool[2,3]:.4f})")
        except TimeoutError:
            print(f"  ⚠️ 读取位姿超时，跳过")
            continue

        # 3. 拍照
        ret, frame = cap.read()
        if not ret:
            print(f"  ⚠️ 拍照失败，跳过")
            continue

        # 4. 检测标定板
        T_camera_to_board, corners = detect_board(frame, K, dist, objp)
        if T_camera_to_board is None:
            print(f"  ⚠️ 未检测到标定板，跳过 (调整机械臂位姿使标定板在视野内)")
            if args.show_preview:
                cv2.imshow("Preview", frame)
                cv2.waitKey(100)
            continue

        # 5. 保存图像
        img_name = f"pose_{success_count:02d}.jpg"
        img_path = image_dir / img_name
        debug_img = draw_board(frame, corners, T_camera_to_board, K, dist)
        cv2.imwrite(str(img_path), debug_img)
        print(f"  图像已保存: {img_path}")

        # 6. 记录数据
        T_base_to_tool_list.append(T_base_to_tool)
        T_camera_to_board_list.append(T_camera_to_board)
        image_paths.append(str(img_path))
        success_count += 1
        print(f"  ✅ 采集成功 (累计 {success_count})")

        # 预览
        if args.show_preview:
            cv2.imshow("Preview", debug_img)
            key = cv2.waitKey(500)
            if key == 27:  # ESC
                print("\n  用户中断")
                break

    # ── 回 home ─────────────────────────────────────────────────────
    go_home()

    # ── 保存 ────────────────────────────────────────────────────────
    print(f"\n{'='*60}")
    if success_count < 3:
        print(f"❌ 有效数据只有 {success_count} 组，至少需要 3 组，标定失败。")
        print("   请调整标定板位置或机械臂姿态后重试。")
    else:
        save_calibration_data(
            output_path,
            T_base_to_tool_list,
            T_camera_to_board_list,
            image_paths,
        )
        print(f"✅ 采集完成！共 {success_count} 组有效数据")
        print(f"   运行标定: python -m examples.hand_eye_calibration.calibrate")

    # ── 清理 ────────────────────────────────────────────────────────
    cap.release()
    pose_reader.stop()
    if args.show_preview:
        cv2.destroyAllWindows()


if __name__ == "__main__":
    main()
