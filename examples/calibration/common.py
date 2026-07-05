"""
静力学标定 — 公共模块

提供 RPC 调用、角度转换等通用功能。
"""

from __future__ import annotations

import json
import time
from typing import List, Optional

import requests

# RPC 端点
RPC_URL = "http://127.0.0.1:50080/rpc/robotic_arm.protocols.MoveControlService/MoveJ"


def movej(target: List[float], v: float = 0.3, a: float = 0.5,
          timeout: float = 120) -> bool:
    """调用 MoveJ RPC，向机器人发送关节空间运动指令。

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
        # 本地地址不走代理，避免 Clash/VPN 等拦截 RPC 请求
        session = requests.Session()
        session.trust_env = False
        resp = session.post(RPC_URL, json=payload, timeout=timeout)
        resp.raise_for_status()
        result = resp.json()
        code = result.get("error", {}).get("code", -1)
        return int(code) == 0
    except requests.exceptions.RequestException as e:
        print(f"    [ERROR] RPC 失败: {e}")
        return False


def deg2rad(deg: float) -> float:
    """角度 → 弧度"""
    return deg * 3.141592653589793 / 180.0


def rad2deg(rad: float) -> float:
    """弧度 → 角度"""
    return rad * 180.0 / 3.141592653589793





def wait_stable(seconds: float = 0.8):
    """等待机器人稳定"""
    time.sleep(seconds)


def print_step(joint_id: int, angle_deg: float, index: int, total: int):
    """打印当前采点信息（monitor 记录时的参考标记）"""
    print(f"  [{index:3d}/{total}]  J{joint_id} = {angle_deg:+7.2f}°"
          f"  ({deg2rad(angle_deg):+.4f} rad)")


def go_home(v: float = 1.0, a: float = 0.5) -> bool:
    """回到 home 位姿（所有关节归零）"""
    print("\n>>> 正在回到 home 位姿 ...")
    ok = movej([0.0, 0.0, 0.0, 0.0, 0.0, 0.0], v=v, a=a)
    if ok:
        print(">>> 已回到 home\n")
    else:
        print(">>> 回 home 失败\n")
    return ok


def wait_for_save():
    """采集完成后暂停，等用户导出数据后按 Enter 回零"""
    print(f"\n✅ 采集完成！请在 monitor 中点击「💾 Save CSV」导出数据")
    input("导出完成后按 Enter 回零 ...\n")
