#!/usr/bin/env python3
"""
MoveJMuti Demo — 多路径点关节空间运动

向 GeniArm 发送 MoveJMuti RPC 请求，控制机械臂依次经过
多个关节角度路径点，路径点之间以圆弧过渡。

用法:
    python3 movej_muti_demo.py
"""

import requests
import json

# RPC endpoint
URL = "http://127.0.0.1:50080/rpc/robotic_arm.protocols.MoveControlService/MoveJMuti"

# 多个关节角度路径点 [joint1, joint2, joint3, joint4, joint5, joint6] (弧度)
payload = {
    "position": [
        {"position": [0.0, 0.5, -1.0, 0.3, 0.0, 0.0]},   # 起点
        {"position": [0.5, 1.0, -1.5, 0.5, 0.2, 0.1]},   # 中间点1
        {"position": [1.0, 1.5, -2.0, 0.8, 0.4, 0.2]},   # 终点
    ],
    "r": 0.1,   # 过渡半径 (rad)
    "v": 1.0,   # 速度 (rad/s)
    "a": 0.5    # 加速度 (rad/s^2)
}

print("=" * 60)
print("MoveJMuti Demo — 多路径点关节运动")
print("=" * 60)
print("Waypoints:")
for i, wp in enumerate(payload["position"]):
    print(f"  [{i}] {wp['position']}")
print(f"Blend radius: {payload['r']} rad")
print(f"Velocity: {payload['v']} rad/s, Acceleration: {payload['a']} rad/s^2")
print(f"Sending to {URL} ...")

try:
    resp = requests.post(URL, json=payload, timeout=60)
    resp.raise_for_status()
    result = resp.json()
    print(f"\nResponse: {json.dumps(result, indent=2)}")

    if result.get("error", {}).get("code", -1) == 0:
        print("\n✅ MoveJMuti succeeded!")
    else:
        print(f"\n❌ MoveJMuti failed: {result.get('error', {}).get('msg', 'unknown')}")

except requests.exceptions.ConnectionError:
    print("\n❌ 无法连接到 GeniArm (127.0.0.1:50080)，请确认程序已启动。")
except requests.exceptions.Timeout:
    print("\n❌ 请求超时，轨迹执行时间过长。")
except requests.exceptions.RequestException as e:
    print(f"\n❌ 请求失败: {e}")
