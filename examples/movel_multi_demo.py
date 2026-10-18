#!/usr/bin/env python3
"""
MoveLMulti Demo — 多路径点笛卡尔空间运动

向 GeniArm 发送 MoveLMulti RPC 请求，控制机械臂末端依次经过
多个位姿路径点，路径点之间以圆弧过渡。

用法:
    python3 movel_multi_demo.py
"""

import requests
import json
import math


def rpy_to_quat(roll_deg, pitch_deg, yaw_deg):
    """Convert RPY (degrees) to quaternion (w, x, y, z)."""
    r = math.radians(roll_deg / 2)
    p = math.radians(pitch_deg / 2)
    y = math.radians(yaw_deg / 2)
    return {
        "w": math.cos(r) * math.cos(p) * math.cos(y) + math.sin(r) * math.sin(p) * math.sin(y),
        "x": math.sin(r) * math.cos(p) * math.cos(y) - math.cos(r) * math.sin(p) * math.sin(y),
        "y": math.cos(r) * math.sin(p) * math.cos(y) + math.sin(r) * math.cos(p) * math.sin(y),
        "z": math.cos(r) * math.cos(p) * math.sin(y) - math.sin(r) * math.sin(p) * math.cos(y),
    }


# 末端姿态 (RPY 欧拉角, 单位: 度)
ROLL_DEG = -108.0
PITCH_DEG = 0.0
YAW_DEG = -90.0

ORI = rpy_to_quat(ROLL_DEG, PITCH_DEG, YAW_DEG)

URL_MOVELMULTI = "http://127.0.0.1:50080/rpc/genisys_msgs/srv/MoveLMulti"

# 多个末端位姿路径点: 位置 (x, y, z) + 四元数 (w, x, y, z)
payload = {
    "pose": [
        {
            "position": {"x": 0.30, "y": -0.10, "z": 0.15},
            "orientation": ORI
        },
        {
            "position": {"x": 0.30, "y": 0.10, "z": 0.15},
            "orientation": ORI
        },
        {
            "position": {"x": 0.30, "y": 0.00, "z": 0.20},
            "orientation": ORI
        },
    ],
    "r": 0.05,  # 过渡半径 (m)
    "v": 0.3,   # 末端线速度 (m/s)
    "a": 0.2    # 末端线加速度 (m/s^2)
}

print("=" * 60)
print("MoveLMulti Demo — 多路径点笛卡尔运动")
print("=" * 60)
print("Waypoints:")
for i, wp in enumerate(payload["pose"]):
    p = wp["position"]
    o = wp["orientation"]
    print(f"  [{i}] pos=({p['x']:.2f}, {p['y']:.2f}, {p['z']:.2f}), "
          f"ori=({o['w']}, {o['x']}, {o['y']}, {o['z']})")
print(f"Blend radius: {payload['r']} m")
print(f"Velocity: {payload['v']} m/s, Acceleration: {payload['a']} m/s^2")
print(f"Sending to {URL_MOVELMULTI} ...")

try:
    resp = requests.post(URL_MOVELMULTI, json=payload, timeout=60)
    resp.raise_for_status()
    result = resp.json()
    print(f"\nResponse: {json.dumps(result, indent=2)}")

    code = result.get("error", {}).get("code", -1)
    if int(code) == 0:
        print("\n✅ MoveLMulti succeeded!")
    else:
        print(f"\n❌ MoveLMulti failed (code={code}): {result.get('error', {}).get('msg', 'unknown')}")

except requests.exceptions.ConnectionError:
    print("\n❌ 无法连接到 GeniArm (127.0.0.1:50080)，请确认程序已启动。")
except requests.exceptions.Timeout:
    print("\n❌ 请求超时，轨迹执行时间过长。")
except requests.exceptions.RequestException as e:
    print(f"\n❌ 请求失败: {e}")
