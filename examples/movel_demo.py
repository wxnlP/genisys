#!/usr/bin/env python3
"""
MoveL Demo — 笛卡尔空间直线运动

先用 MoveJ 到安全姿态，再用 MoveL 做小范围直线运动。

用法:
    python3 movel_demo.py
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

URL_MOVEJ = "http://127.0.0.1:50080/rpc/robotic_arm.protocols.MoveControlService/MoveJ"
URL_MOVEL = "http://127.0.0.1:50080/rpc/robotic_arm.protocols.MoveControlService/MoveL"

# 1. 先用 MoveJ 到一个安全的展开姿态
pose_joint = {
    "position": {
        "position": [0.0, 0.8, -0.5, 0.0, 0.0, 0.0]
    },
    "v": 0.8,
    "a": 0.4
}

print("=" * 60)
print("Step 1: MoveJ — 展开到安全姿态")
print("=" * 60)
print(f"Target joints: {pose_joint['position']['position']}")
resp = requests.post(URL_MOVEJ, json=pose_joint, timeout=30)
result = resp.json()
code = result.get("error", {}).get("code", -1)
print(f"Result: {'✅' if int(code)==0 else '❌'} (code={code})")

if int(code) != 0:
    print("MoveJ 失败，无法继续")
    exit(1)

# 2. 笛卡尔空间直线运动
payload = {
    "pose": {
        "position": {"x": 0.30, "y": 0.0, "z": 0.15},
        "orientation": ORI
    },
    "v": 0.3,
    "a": 0.2
}

print("\n" + "=" * 60)
print("Step 2: MoveL — 直线运动到目标")
print("=" * 60)
print(f"Target pose: pos=({payload['pose']['position']['x']}, "
      f"{payload['pose']['position']['y']}, "
      f"{payload['pose']['position']['z']})")
print(f"Velocity: {payload['v']} m/s, Acceleration: {payload['a']} m/s^2")
print(f"Sending to {URL_MOVEL} ...")

resp = requests.post(URL_MOVEL, json=payload, timeout=30)
resp.raise_for_status()
result = resp.json()
print(f"Response: {json.dumps(result, indent=2)}")

code = result.get("error", {}).get("code", -1)
if int(code) == 0:
    print("\n✅ MoveL succeeded!")
else:
    print(f"\n❌ MoveL failed (code={code}): {result.get('error', {}).get('msg', 'unknown')}")
