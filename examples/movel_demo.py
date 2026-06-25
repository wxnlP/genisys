#!/usr/bin/env python3
"""
MoveL Demo — 笛卡尔空间直线运动

向 GeniArm 发送 MoveL RPC 请求，控制机械臂末端沿直线
从当前位置运动到目标位姿。

用法:
    python3 movel_demo.py
"""

import requests
import json

# RPC endpoint
URL = "http://127.0.0.1:50080/rpc/robotic_arm.protocols.MoveControlService/MoveL"

# 目标末端位姿: 位置 (x, y, z) + 四元数 (w, x, y, z)
# 此处为机械臂正前方约 0.4m 处，姿态保持竖直
payload = {
    "pose": {
        "position": {
            "x": 0.4,
            "y": 0.0,
            "z": 0.3
        },
        "orientation": {
            "w": 1.0,
            "x": 0.0,
            "y": 0.0,
            "z": 0.0
        }
    },
    "v": 0.5,   # 末端线速度 (m/s)
    "a": 0.3    # 末端线加速度 (m/s^2)
}

print("=" * 60)
print("MoveL Demo — 笛卡尔直线运动")
print("=" * 60)
print(f"Target pose: pos=({payload['pose']['position']['x']}, "
      f"{payload['pose']['position']['y']}, "
      f"{payload['pose']['position']['z']}), "
      f"ori=({payload['pose']['orientation']['w']}, "
      f"{payload['pose']['orientation']['x']}, "
      f"{payload['pose']['orientation']['y']}, "
      f"{payload['pose']['orientation']['z']})")
print(f"Velocity: {payload['v']} m/s, Acceleration: {payload['a']} m/s^2")
print(f"Sending to {URL} ...")

try:
    resp = requests.post(URL, json=payload, timeout=30)
    resp.raise_for_status()
    result = resp.json()
    print(f"\nResponse: {json.dumps(result, indent=2)}")

    code = result.get("error", {}).get("code", -1)
    if int(code) == 0:
        print("\n✅ MoveL succeeded!")
    else:
        print(f"\n❌ MoveL failed (code={code}): {result.get('error', {}).get('msg', 'unknown')}")

except requests.exceptions.ConnectionError:
    print("\n❌ 无法连接到 GeniArm (127.0.0.1:50080)，请确认程序已启动。")
except requests.exceptions.Timeout:
    print("\n❌ 请求超时，轨迹执行时间过长。")
except requests.exceptions.RequestException as e:
    print(f"\n❌ 请求失败: {e}")
