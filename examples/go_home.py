#!/usr/bin/env python3
"""
Go Home Demo — 回原点

向 GeniArm 发送 MoveJ RPC 请求，控制机械臂从当前位置
以关节空间插值方式运动到原点姿态（所有关节回到零位）。

用法:
    python3 go_home.py
"""

import requests
import json

URL = "http://127.0.0.1:50080/rpc/robotic_arm.protocols.MoveControlService/MoveJ"

# 原点：所有关节角度归零
payload = {
    "position": {
        "position": [0.0, 0.0, 0.0, 0.0, 0.0, 0.0]
    },
    "v": 1.0,   # 速度 (rad/s)
    "a": 0.5    # 加速度 (rad/s^2)
}

print("=" * 60)
print("Go Home Demo — 回原点")
print("=" * 60)
print(f"Target: all joints → 0.0 rad")
print(f"Velocity: {payload['v']} rad/s, Acceleration: {payload['a']} rad/s^2")
print(f"Sending to {URL} ...")

try:
    resp = requests.post(URL, json=payload, timeout=30)
    resp.raise_for_status()
    result = resp.json()
    print(f"\nResponse: {json.dumps(result, indent=2)}")

    code = result.get("error", {}).get("code", -1)
    if int(code) == 0:
        print("\n✅ 已回到原点!")
    else:
        print(f"\n❌ 回原点失败 (code={code}): {result.get('error', {}).get('msg', 'unknown')}")

except requests.exceptions.ConnectionError:
    print("\n❌ 无法连接到 GeniArm (127.0.0.1:50080)，请确认程序已启动。")
except requests.exceptions.Timeout:
    print("\n❌ 请求超时，轨迹执行时间过长。")
except requests.exceptions.RequestException as e:
    print(f"\n❌ 请求失败: {e}")
