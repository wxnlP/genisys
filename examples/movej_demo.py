#!/usr/bin/env python3
"""
MoveJ Demo — 关节空间点到点运动

向 GeniArm 发送 MoveJ RPC 请求，控制机械臂从当前位置
以关节空间插值方式运动到目标关节角度。

用法:
    python3 movej_demo.py
"""

import requests
import json

# RPC endpoint
URL = "http://127.0.0.1:50080/rpc/robotic_arm.protocols.MoveControlService/MoveJ"

# 目标关节角度 [joint1, joint2, joint3, joint4, joint5, joint6] (弧度)
# home 位置附近的一个姿态
payload = {
    "position": {
        "position": [0.0, 0.5, -1.0, 0.3, 0.0, 0.0]
    },
    "v": 1.0,   # 速度 (rad/s)
    "a": 0.5    # 加速度 (rad/s^2)
}

print("=" * 60)
print("MoveJ Demo — 关节空间运动")
print("=" * 60)
print(f"Target joints: {payload['position']['position']}")
print(f"Velocity: {payload['v']} rad/s, Acceleration: {payload['a']} rad/s^2")
print(f"Sending to {URL} ...")

try:
    resp = requests.post(URL, json=payload, timeout=30)
    resp.raise_for_status()
    result = resp.json()
    print(f"\nResponse: {json.dumps(result, indent=2)}")

    if result.get("error", {}).get("code", -1) == 0:
        print("\n✅ MoveJ succeeded!")
    else:
        print(f"\n❌ MoveJ failed: {result.get('error', {}).get('msg', 'unknown')}")

except requests.exceptions.ConnectionError:
    print("\n❌ 无法连接到 GeniArm (127.0.0.1:50080)，请确认程序已启动。")
except requests.exceptions.Timeout:
    print("\n❌ 请求超时，轨迹执行时间过长。")
except requests.exceptions.RequestException as e:
    print(f"\n❌ 请求失败: {e}")
