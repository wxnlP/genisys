#!/usr/bin/env python3
"""
MoveLMuti Demo — 多路径点笛卡尔空间运动

向 GeniArm 发送 MoveLMuti RPC 请求，控制机械臂末端依次经过
多个位姿路径点，路径点之间以圆弧过渡。

用法:
    python3 movel_muti_demo.py
"""

import requests
import json

# RPC endpoint
URL = "http://127.0.0.1:50080/rpc/robotic_arm.protocols.MoveControlService/MoveLMuti"

# 多个末端位姿路径点: 位置 (x, y, z) + 四元数 (w, x, y, z)
payload = {
    "pose": [
        {
            "position": {"x": 0.40, "y": 0.00, "z": 0.30},
            "orientation": {"w": 1.0, "x": 0.0, "y": 0.0, "z": 0.0}
        },
        {
            "position": {"x": 0.30, "y": 0.15, "z": 0.25},
            "orientation": {"w": 1.0, "x": 0.0, "y": 0.0, "z": 0.0}
        },
        {
            "position": {"x": 0.20, "y": 0.00, "z": 0.20},
            "orientation": {"w": 1.0, "x": 0.0, "y": 0.0, "z": 0.0}
        },
    ],
    "r": 0.05,  # 过渡半径 (m)
    "v": 0.3,   # 末端线速度 (m/s)
    "a": 0.2    # 末端线加速度 (m/s^2)
}

print("=" * 60)
print("MoveLMuti Demo — 多路径点笛卡尔运动")
print("=" * 60)
print("Waypoints:")
for i, wp in enumerate(payload["pose"]):
    p = wp["position"]
    o = wp["orientation"]
    print(f"  [{i}] pos=({p['x']:.2f}, {p['y']:.2f}, {p['z']:.2f}), "
          f"ori=({o['w']}, {o['x']}, {o['y']}, {o['z']})")
print(f"Blend radius: {payload['r']} m")
print(f"Velocity: {payload['v']} m/s, Acceleration: {payload['a']} m/s^2")
print(f"Sending to {URL} ...")

try:
    resp = requests.post(URL, json=payload, timeout=60)
    resp.raise_for_status()
    result = resp.json()
    print(f"\nResponse: {json.dumps(result, indent=2)}")

    if result.get("error", {}).get("code", -1) == 0:
        print("\n✅ MoveLMuti succeeded!")
    else:
        print(f"\n❌ MoveLMuti failed: {result.get('error', {}).get('msg', 'unknown')}")

except requests.exceptions.ConnectionError:
    print("\n❌ 无法连接到 GeniArm (127.0.0.1:50080)，请确认程序已启动。")
except requests.exceptions.Timeout:
    print("\n❌ 请求超时，轨迹执行时间过长。")
except requests.exceptions.RequestException as e:
    print(f"\n❌ 请求失败: {e}")
