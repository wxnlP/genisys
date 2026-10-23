#!/usr/bin/env python3
"""
MoveJPose Demo — 目标位姿的点到点运动

向 GeniArm 发送 MoveJPose RPC 请求，由控制端对目标位姿做逆解后，
以关节空间插值方式运动到该位姿（v/a 为关节速度、关节加速度）。

用法:
    python3 movej_pose_demo.py                        # 全部使用默认参数
    python3 movej_pose_demo.py --pose 0.30 0 0.30 0 0 0
    python3 movej_pose_demo.py --pose 0.30,0,0.30,0,0,0 --v 30 --a 60
    python3 movej_pose_demo.py --v 30                 # 只改速度，位姿仍用默认

参数分两组，可各自独立覆盖（未指定的沿用默认值）:
    1. 运动目标: --pose x y z roll pitch yaw
                 x/y/z 单位 m；roll/pitch/yaw 为 ZYX 欧拉角，单位 度
    2. 运动参数: --v / --a               关节速度 度/秒、加速度 度/秒^2

写法说明: --pose 的 6 个数值支持「空格分隔」或「逗号分隔」两种写法。
          注意逗号形式若以 '-' 开头（例如需要负的姿态角）会被 argparse 当成选项,
          此时请改用空格写法，或写成 --pose=... 形式。

注意: 本脚本对外统一使用「度」和「度/秒」，内部自动换算成协议要求的
      弧度、弧度/秒后再发送。
"""

import argparse
import json
import math
import sys

import requests

# ---------------------------------------------------------------------------
# RPC endpoint
# ---------------------------------------------------------------------------
URL = "http://127.0.0.1:50080/rpc/genisys_msgs/srv/MoveJPose"
TIMEOUT = 30.0

# ---------------------------------------------------------------------------
# 默认参数
# ---------------------------------------------------------------------------
# 末端目标位姿: x y z (m) + roll pitch yaw (ZYX 欧拉角, 度)
# 这是 TCP 位姿；无夹爪时 TCP 仍带默认虚拟工具旋转（rpy=[90,-90,0]），
# 因此零位时 TCP 轴与基座对齐，且 flange_pose 与 tcp_pose 不再相等。
DEFAULT_POSE = (0.30, 0.0, 0.30, 0.0, 0.0, 0.0)

# 关节运动参数 (度制)
DEFAULT_VEL_DEG = 57.2958   # = 1.0 rad/s
DEFAULT_ACC_DEG = 28.6479   # = 0.5 rad/s^2


# ---------------------------------------------------------------------------
# 数学工具
# ---------------------------------------------------------------------------
def zyx_to_quat(roll_deg, pitch_deg, yaw_deg):
    """ZYX 欧拉角(度) 转 四元数 (w, x, y, z)。

    对应旋转矩阵 R = Rz(yaw) · Ry(pitch) · Rx(roll)，
    与 RViz2 的 "Roll Pitch Yaw (zyx)" 约定一致。
    """
    r = math.radians(roll_deg / 2)    # roll  = 绕 X 轴
    p = math.radians(pitch_deg / 2)   # pitch = 绕 Y 轴
    y = math.radians(yaw_deg / 2)     # yaw   = 绕 Z 轴
    return {
        "w": math.cos(r) * math.cos(p) * math.cos(y) + math.sin(r) * math.sin(p) * math.sin(y),
        "x": math.sin(r) * math.cos(p) * math.cos(y) - math.cos(r) * math.sin(p) * math.sin(y),
        "y": math.cos(r) * math.sin(p) * math.cos(y) + math.sin(r) * math.cos(p) * math.sin(y),
        "z": math.cos(r) * math.cos(p) * math.sin(y) - math.sin(r) * math.sin(p) * math.cos(y),
    }


def parse_float_list(value, count, label):
    """把 count 个 float 解析成列表。

    同时兼容:
        --pose 0.30 0 0.30 0 0 0            (空格分隔 -> nargs 列表)
        --pose 0.30,0,0.30,0,0,0            (逗号分隔 -> 单元素列表)
        --pose "0.30 0 0.30 0 0 0"          (整串空格分隔 -> 单元素列表)
    """
    if isinstance(value, str):
        items = [x for x in value.replace(",", " ").split() if x]
    else:
        items = [x for item in value for x in str(item).replace(",", " ").split() if x]

    if len(items) != count:
        raise argparse.ArgumentTypeError(
            f"{label} 需要 {count} 个数值，实际收到 {len(items)} 个: {items}"
        )

    try:
        return [float(x) for x in items]
    except ValueError as exc:
        raise argparse.ArgumentTypeError(f"{label} 必须是数字: {exc}") from exc


def parse_pose(value):
    return parse_float_list(value, 6, "--pose")


# ---------------------------------------------------------------------------
# 命令行
# ---------------------------------------------------------------------------
def build_parser():
    parser = argparse.ArgumentParser(
        description="MoveJPose Demo — 目标位姿的点到点运动 (位置 m, 姿态 ZYX 欧拉角 度)",
        formatter_class=argparse.RawDescriptionHelpFormatter,
        epilog=(
            "示例:\n"
            "  %(prog)s\n"
            "  %(prog)s --pose 0.30 0 0.30 0 0 0\n"
            "  %(prog)s --pose 0.30,0,0.30,0,0,0 --v 30 --a 60\n"
            "  %(prog)s --pose 0.35 0 0.30 0 0 0   # 只改目标位姿\n"
            "  %(prog)s --v 30                      # 只改速度\n"
            "  %(prog)s --url http://10.0.0.5:50080 --pose 0.30 0 0.30 0 0 0\n"
        ),
    )

    # 第 1 组: 运动目标 = 位姿 (m + 度)
    g_target = parser.add_argument_group(
        "运动目标 (x y z 单位 m; roll pitch yaw 为 ZYX 欧拉角, 单位 度)"
    )
    g_target.add_argument(
        "--pose", "-p", type=str, nargs="+", default=None, metavar="V",
        help="目标位姿，依次为 x y z roll pitch yaw，如 '--pose 0.30 0 0.30 0 0 0' "
             f"(默认: {' '.join(str(v) for v in DEFAULT_POSE)})",
    )

    # 第 2 组: 运动参数 = 关节速度 / 加速度 (度制)
    g_motion = parser.add_argument_group("运动参数 (关节速度/加速度, 度制)")
    g_motion.add_argument(
        "--v", type=float, default=None, metavar="DEG_S",
        help=f"关节速度，单位 度/秒 (默认: {DEFAULT_VEL_DEG})",
    )
    g_motion.add_argument(
        "--a", type=float, default=None, metavar="DEG_S2",
        help=f"关节加速度，单位 度/秒^2 (默认: {DEFAULT_ACC_DEG})",
    )

    # 其他
    g_other = parser.add_argument_group("其他")
    g_other.add_argument("--url", default=URL, help=f"RPC 地址 (默认: {URL})")
    g_other.add_argument(
        "--timeout", type=float, default=TIMEOUT,
        help=f"请求超时秒数 (默认: {TIMEOUT})",
    )
    g_other.add_argument(
        "--dry-run", action="store_true",
        help="只打印将要发送的 payload，不实际发起请求",
    )
    g_other.add_argument(
        "--json", dest="json_out", action="store_true",
        help="以 JSON 输出结果 (便于脚本调用/自动化测试)，不打印横幅文本",
    )
    return parser


def resolve_pose(args):
    """解析 --pose (x y z roll pitch yaw)，未指定则用默认值。"""
    if args.pose is None:
        return list(DEFAULT_POSE), []
    return parse_pose(args.pose), ["--pose"]


def main():
    parser = build_parser()
    args = parser.parse_args()

    # --- 第 1 组: 运动目标 -------------------------------------------------
    try:
        pose, pose_overridden = resolve_pose(args)
    except argparse.ArgumentTypeError as exc:
        parser.error(str(exc))

    x, y, z, roll, pitch, yaw = pose
    ori = zyx_to_quat(roll, pitch, yaw)

    # --- 第 2 组: 运动参数 (度制 -> 弧度制) --------------------------------
    v_deg = args.v if args.v is not None else DEFAULT_VEL_DEG
    a_deg = args.a if args.a is not None else DEFAULT_ACC_DEG
    motion_overridden = [
        name for name, val in (("--v", args.v), ("--a", args.a)) if val is not None
    ]

    # 内部换算: 协议要求 弧度 / 弧度每秒
    v_rad = math.radians(v_deg)
    a_rad = math.radians(a_deg)

    payload = {
        "pose": {
            "position": {"x": x, "y": y, "z": z},
            "orientation": ori,
        },
        "v": v_rad,
        "a": a_rad,
    }

    if args.dry_run:
        if args.json_out:
            print(json.dumps(payload, indent=2))
            return 0
        print("[dry-run] payload (实际发送内容, 弧度制):")
        print(json.dumps(payload, indent=2))
        print("\n[dry-run] 未发起实际请求。")
        return 0

    quiet = args.json_out
    if not quiet:
        print("=" * 60)
        print("MoveJPose Demo — 目标位姿的点到点运动")
        print("=" * 60)
        print(f"目标位置 (m): x={x}, y={y}, z={z}  "
              f"{'| 覆盖: ' + ','.join(pose_overridden) if pose_overridden else '| 默认'}")
        print(f"目标姿态 (ZYX 欧拉角, 度): roll={roll}, pitch={pitch}, yaw={yaw}")
        print(f"四元数 (w,x,y,z): {[round(ori[k], 6) for k in ('w', 'x', 'y', 'z')]}")
        print(f"关节速度: {round(v_deg, 4)} 度/秒  ({round(v_rad, 6)} rad/s)")
        print(f"关节加速度: {round(a_deg, 4)} 度/秒^2  ({round(a_rad, 6)} rad/s^2)")
        print(f"覆盖项: 目标={'是' if pose_overridden else '否(默认)'}"
              f"{'[' + ','.join(pose_overridden) + ']' if pose_overridden else ''}, "
              f"运动参数={'是' if motion_overridden else '否(默认)'}"
              f"{'[' + ','.join(motion_overridden) + ']' if motion_overridden else ''}")
        print(f"Sending to {args.url} ...")

    try:
        resp = requests.post(args.url, json=payload, timeout=args.timeout)
        resp.raise_for_status()
        result = resp.json()

        code = result.get("error", {}).get("code", -1)
        # code 可能是字符串 "0" 或整数 0，统一转为 int 比较
        ok = int(code) == 0
        msg = result.get("error", {}).get("msg", "unknown")

        if quiet:
            print(json.dumps({
                "ok": ok, "code": code, "msg": msg,
                "payload": payload, "response": result,
            }))
            return 0 if ok else 1

        print(f"\nResponse: {json.dumps(result, indent=2)}")
        if ok:
            print("\n✅ MoveJPose succeeded!")
            return 0

        print(f"\n❌ MoveJPose failed (code={code}): {msg}")
        return 1

    except requests.exceptions.ConnectionError:
        if quiet:
            print(json.dumps({"error": f"无法连接到 {args.url}"}))
        else:
            print(f"\n❌ 无法连接到 GeniArm ({args.url})，请确认程序已启动。")
        return 1
    except requests.exceptions.Timeout:
        if quiet:
            print(json.dumps({"error": "请求超时"}))
        else:
            print("\n❌ 请求超时，轨迹执行时间过长。")
        return 1
    except requests.exceptions.RequestException as e:
        if quiet:
            print(json.dumps({"error": f"请求失败: {e}"}))
        else:
            print(f"\n❌ 请求失败: {e}")
        return 1


if __name__ == "__main__":
    sys.exit(main())
