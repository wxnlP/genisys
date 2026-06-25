#!/usr/bin/env python3
"""
Gripper Demo — 夹爪往复运动

向 GeniArm 发送 Gripper RPC 请求，控制夹爪往复开合 2~3 次，
每次使用不同速度以观察速度变化效果，最后回到 0 位（夹紧）。

用法:
    python3 gripper_demo.py
"""

import requests
import json
import time

URL = "http://127.0.0.1:50080/rpc/robotic_arm.protocols.MoveControlService/Gripper"

# 往复运动序列: (目标位置, 速度, 加速度, 力矩, 描述)
# position: 0.0=夹紧, 1.0=全开
sequence = [
    (1.0,  0.5, 1.0, 0.5, "张开 (慢速)"),
    (0.0,  0.5, 1.0, 0.5, "夹紧 (慢速)"),
    (1.0,  1.0, 2.0, 0.5, "张开 (中速)"),
    (0.0,  1.0, 2.0, 0.5, "夹紧 (中速)"),
    (1.0,  2.0, 3.0, 0.5, "张开 (快速)"),
    (0.0,  2.0, 3.0, 0.5, "夹紧 (快速) → 归零"),
]

print("=" * 60)
print("Gripper Demo — 夹爪往复运动")
print("=" * 60)

for i, (pos, vel, accel, eff, desc) in enumerate(sequence, 1):
    payload = {
        "position": pos,
        "velocity": vel,
        "acceleration": accel,
        "effort": eff,
    }

    print(f"\n[{i}/{len(sequence)}] {desc}")
    print(f"    target={pos:.1f}, v={vel:.1f}, a={accel:.1f}, effort={eff:.1f}")
    print(f"    Sending ...", end="", flush=True)

    try:
        resp = requests.post(URL, json=payload, timeout=30)
        resp.raise_for_status()
        result = resp.json()
        code = result.get("error", {}).get("code", -1)
        if int(code) == 0:
            print(" ✅")
        else:
            msg = result.get("error", {}).get("msg", "unknown")
            print(f" ❌ (code={code}): {msg}")
            break
    except requests.exceptions.ConnectionError:
        print("\n❌ 无法连接到 GeniArm (127.0.0.1:50080)，请确认程序已启动。")
        break
    except requests.exceptions.Timeout:
        print("\n❌ 请求超时。")
        break
    except requests.exceptions.RequestException as e:
        print(f"\n❌ 请求失败: {e}")
        break

    time.sleep(0.3)

print("\n" + "=" * 60)
print("Gripper Demo 完成。夹爪已归零（夹紧）。")
print("=" * 60)
