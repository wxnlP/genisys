"""
手眼标定 — 公共模块

提供 RPC 机械臂控制、Zenoh 位姿读取、矩阵工具函数。
复用 ``examples/calibration/common.py`` 中的 ``movej`` / ``go_home`` 接口。
"""

from __future__ import annotations

import json
import time
from pathlib import Path
from typing import Optional

import numpy as np
import requests

# ── RPC 端点 ──────────────────────────────────────────────────────────
RPC_URL = "http://127.0.0.1:50080/rpc/robotic_arm.protocols.MoveControlService"


def movej(target: list[float], v: float = 0.3, a: float = 0.5,
          timeout: float = 30) -> bool:
    """MoveJ RPC — 关节空间运动。

    Args:
        target: 目标关节角度 [q1..q6]（弧度）
        v: 速度 (rad/s)
        a: 加速度 (rad/s²)
        timeout: 请求超时（秒）

    Returns:
        True 成功；False 失败
    """
    payload = {
        "position": {"position": target},
        "v": v,
        "a": a,
    }
    try:
        resp = requests.post(f"{RPC_URL}/MoveJ", json=payload, timeout=timeout)
        resp.raise_for_status()
        result = resp.json()
        code = result.get("error", {}).get("code", -1)
        return int(code) == 0
    except requests.exceptions.RequestException as e:
        print(f"    [ERROR] RPC 失败: {e}")
        return False


def go_home(v: float = 1.0, a: float = 0.5) -> bool:
    """回到 home 位姿（所有关节归零）。"""
    print("\n>>> 正在回到 home 位姿 ...")
    ok = movej([0.0, 0.0, 0.0, 0.0, 0.0, 0.0], v=v, a=a)
    if ok:
        print(">>> 已回到 home\n")
    else:
        print(">>> 回 home 失败\n")
    return ok


# ── Zenoh 末端位姿读取 ────────────────────────────────────────────────

class PoseReader:
    """从 Zenoh 订阅 ``/geniarm/pose_current``，读取当前末端位姿。

    用法::

        reader = PoseReader()
        T = reader.read()          # 等待并返回最新的 4×4 矩阵
        T = reader.read(timeout=5.0)
        reader.stop()
    """

    TOPIC = "/geniarm/pose_current"

    def __init__(self, zenoh_config: Optional[dict] = None):
        import zenoh
        cfg = zenoh.Config()
        if zenoh_config:
            for k, v in zenoh_config.items():
                cfg.insert_json5(k, v)
        self._session = zenoh.open(cfg)
        self._latest: Optional[np.ndarray] = None
        self._sub = self._session.declare_subscriber(self.TOPIC, self._on_pose)
        # 等待第一个数据到达
        self._wait_for_data()

    def _on_pose(self, sample):
        """Zenoh 回调：解码 PoseStamped protobuf → 4×4 矩阵。"""
        from hand_eye_calibration.proto.pose_pb2 import PoseStamped
        msg = PoseStamped()
        msg.ParseFromString(sample.payload)
        pose = msg.pose
        # 四元数
        q = pose.orientation
        # 位置
        t = pose.position
        self._latest = self._quat_to_matrix(q.w, q.x, q.y, q.z, t.x, t.y, t.z)

    def _wait_for_data(self, timeout: float = 3.0):
        """阻塞等待第一个位姿数据。"""
        t0 = time.monotonic()
        while self._latest is None:
            if time.monotonic() - t0 > timeout:
                raise TimeoutError(
                    f"等待 {self.TOPIC} 数据超时 {timeout}s，"
                    f"请确认 robot 程序已启动且 Zenoh 网络正常。"
                )
            time.sleep(0.05)

    def read(self, timeout: float = 5.0) -> np.ndarray:
        """读取当前最新末端位姿。

        Returns:
            4×4 齐次变换矩阵 $T_{base\\to tool}$
        """
        t0 = time.monotonic()
        while self._latest is None:
            if time.monotonic() - t0 > timeout:
                raise TimeoutError(f"读取位姿超时 {timeout}s")
            time.sleep(0.01)
        return self._latest.copy()

    def stop(self):
        """清理 Zenoh session。"""
        try:
            self._sub.undeclare()
        except Exception:
            pass
        try:
            self._session.close()
        except Exception:
            pass

    # ------------------------------------------------------------------
    # 工具方法
    # ------------------------------------------------------------------

    @staticmethod
    def _quat_to_matrix(qw, qx, qy, qz, tx, ty, tz) -> np.ndarray:
        """四元数 + 平移 → 4×4 齐次变换矩阵。"""
        # 归一化
        norm = np.sqrt(qw*qw + qx*qx + qy*qy + qz*qz)
        qw, qx, qy, qz = qw/norm, qx/norm, qy/norm, qz/norm
        # 旋转矩阵
        R = np.array([
            [1 - 2*(qy*qy + qz*qz),     2*(qx*qy - qw*qz),     2*(qx*qz + qw*qy)],
            [    2*(qx*qy + qw*qz), 1 - 2*(qx*qx + qz*qz),     2*(qy*qz - qw*qx)],
            [    2*(qx*qz - qw*qy),     2*(qy*qz + qw*qx), 1 - 2*(qx*qx + qy*qy)],
        ])
        T = np.eye(4)
        T[:3, :3] = R
        T[:3, 3] = [tx, ty, tz]
        return T


# ── 矩阵工具函数 ───────────────────────────────────────────────────────

def build_transform(R: np.ndarray, t: np.ndarray) -> np.ndarray:
    """组装 4×4 齐次变换矩阵。"""
    T = np.eye(4)
    T[:3, :3] = R
    T[:3, 3] = t.flatten()
    return T


def decompose_transform(T: np.ndarray) -> tuple[np.ndarray, np.ndarray]:
    """分解 4×4 矩阵 → (R: 3×3, t: 3×1)。"""
    return T[:3, :3], T[:3, 3].reshape(3, 1)


def rotation_error(R1: np.ndarray, R2: np.ndarray) -> float:
    """计算两个旋转矩阵之间的角度误差（度）。"""
    R_err = R1 @ R2.T
    cos_angle = np.clip((np.trace(R_err) - 1) / 2, -1, 1)
    return float(np.degrees(np.arccos(cos_angle)))


def translation_error(t1: np.ndarray, t2: np.ndarray) -> float:
    """计算两个平移向量之间的欧氏距离（mm）。"""
    return float(np.linalg.norm(t1 - t2) * 1000)


# ── 标定板参数 ─────────────────────────────────────────────────────────

# 默认棋盘格配置（可在采集脚本中覆盖）
CHECKERBOARD = (9, 6)     # 内角点数 (宽, 高)
SQUARE_SIZE = 0.030        # 格边长（米）


def get_board_object_points(checkerboard=CHECKERBOARD,
                            square_size=SQUARE_SIZE) -> np.ndarray:
    """生成棋盘格 3D 物点坐标（z=0 平面）。"""
    objp = np.zeros((checkerboard[0] * checkerboard[1], 3), np.float32)
    objp[:, :2] = np.mgrid[0:checkerboard[0],
                            0:checkerboard[1]].T.reshape(-1, 2)
    objp *= square_size
    return objp


# ── 数据序列化 ─────────────────────────────────────────────────────────

def save_calibration_data(filepath: str | Path,
                          T_base_to_tool_list: list[np.ndarray],
                          T_camera_to_board_list: list[np.ndarray],
                          image_paths: Optional[list[str]] = None):
    """保存采集数据到 JSON 文件。"""
    data = {
        "T_base_to_tool": [T.tolist() for T in T_base_to_tool_list],
        "T_camera_to_board": [T.tolist() for T in T_camera_to_board_list],
    }
    if image_paths:
        data["image_paths"] = image_paths
    with open(filepath, "w") as f:
        json.dump(data, f, indent=2)
    print(f"✅ 数据已保存到 {filepath} ({len(T_base_to_tool_list)} 组)")


def load_calibration_data(filepath: str | Path) -> dict:
    """从 JSON 文件加载采集数据。"""
    with open(filepath) as f:
        data = json.load(f)
    for key in ("T_base_to_tool", "T_camera_to_board"):
        data[key] = [np.array(m) for m in data[key]]
    print(f"📂 加载 {filepath} ({len(data['T_base_to_tool'])} 组)")
    return data


# ── 角度转换 ───────────────────────────────────────────────────────────

def deg2rad(deg: float) -> float:
    return deg * np.pi / 180.0


def rad2deg(rad: float) -> float:
    return rad * 180.0 / np.pi
