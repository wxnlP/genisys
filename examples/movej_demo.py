#!/usr/bin/env python3
"""
MoveJ Demo — 关节空间点到点运动

向 GeniArm 发送 MoveJ RPC 请求，控制机械臂从当前位置
以关节空间插值方式运动到目标关节角度。

用法:
    python3 movej_demo.py                        # 全部使用默认参数
    python3 movej_demo.py --joints 0 30 -60 15 0 0
    python3 movej_demo.py --joints 0,30,-60,15,0,0 --v 30 --a 60
    python3 movej_demo.py --v 30                 # 只改速度，关节角仍用默认

参数分两组，可各自独立覆盖（未指定的沿用默认值）:
    1. 运动目标: --joints / --j1..--j6   关节角，单位 度 (deg)
    2. 运动参数: --v / --a               关节速度 度/秒、加速度 度/秒^2

写法说明: 多个数值支持「空格分隔」或「逗号分隔」两种写法。
          注意逗号形式若以 '-' 开头 (如 -30,0,0,0,0,0) 会被 argparse 当成选项,
          此时请改用空格写法 --joints -30 0 0 0 0 0。

注意: 本脚本对外统一使用「度」和「度/秒」，内部自动换算成协议要求的
      弧度、弧度/秒后再发送，你只需要按更直观的度数来写。
"""

import argparse
import json
import math
import sys

import requests

# ---------------------------------------------------------------------------
# RPC endpoint
# ---------------------------------------------------------------------------
URL = "http://127.0.0.1:50080/rpc/ros2_proto/srv/MoveJ"
TIMEOUT = 30.0

# ---------------------------------------------------------------------------
# 默认参数 (单位: 度 / 度每秒)，home 位置附近的一个姿态
# ---------------------------------------------------------------------------
DEFAULT_JOINTS_DEG = [0.0, 28.6479, -57.2958, 17.1887, 0.0, 0.0]
DEFAULT_VEL_DEG = 57.2958   # = 1.0 rad/s
DEFAULT_ACC_DEG = 28.6479   # = 0.5 rad/s^2
JOINT_COUNT = 6


def parse_joint_list(value, count=JOINT_COUNT):
    """把 6 个关节角解析成 float 列表。

    同时兼容三种写法:
        --joints 0 30 -60 15 0 0      (空格分隔 -> 列表)
        --joints "0,30,-60,15,0,0"    (逗号分隔 -> 字符串)
        --joints "0 30 -60 15 0 0"    (整串空格分隔 -> 字符串)
    """
    if isinstance(value, str):
        # 逗号或空格都当作分隔符, 支持一次传入整串
        items = [x for x in value.replace(",", " ").split() if x]
    else:
        # --joints 用 nargs="+" 接收, 可能是 ["0","30",...]
        # 也可能是 ["0,30,-60,15,0,0"] / ["0 30 -60 15 0 0"], 统一拆平
        items = [x for item in value for x in str(item).replace(",", " ").split() if x]

    if len(items) != count:
        raise argparse.ArgumentTypeError(
            f"需要 {count} 个关节角，实际收到 {len(items)} 个: {items}"
        )

    try:
        return [float(x) for x in items]
    except ValueError as exc:
        raise argparse.ArgumentTypeError(f"关节角必须是数字: {exc}") from exc


def build_parser():
    parser = argparse.ArgumentParser(
        description="MoveJ Demo — 关节空间点到点运动（关节角单位: 度）",
        formatter_class=argparse.RawDescriptionHelpFormatter,
        epilog=(
            "示例:\n"
            "  %(prog)s\n"
            "  %(prog)s --joints 0 30 -60 15 0 0\n"
            "  %(prog)s --joints 0,30,-60,15,0,0 --v 30 --a 60\n"
            "  %(prog)s --j2 30 --j3 -60            # 只改第 2、3 轴\n"
            "  %(prog)s --v 30                      # 只改速度\n"
            "  %(prog)s --url http://10.0.0.5:50080 --joints 0 30 -60 15 0 0\n"
        ),
    )

    # 第 1 组: 运动目标 = 关节角 (度)
    g_target = parser.add_argument_group("运动目标 (关节角, 单位: 度)")
    g_target.add_argument(
        "--joints", "-j", type=str, nargs="+", default=None, metavar="DEG",
        help=f"6 个关节角(度)，用空格或逗号分隔，如 '0 30 -60 15 0 0' "
             f"(默认: {DEFAULT_JOINTS_DEG})",
    )
    for i in range(JOINT_COUNT):
        g_target.add_argument(
            f"--j{i + 1}", type=float, default=None, metavar="DEG",
            help=f"单独指定第 {i + 1} 轴关节角(度)，可与其他 --jN 混用",
        )

    # 第 2 组: 运动参数 = 速度 / 加速度 (度制)
    g_motion = parser.add_argument_group("运动参数 (速度/加速度, 度制)")
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


def resolve_joints(args):
    """把 --joints 和 --jN 合并成最终的 6 轴角度列表(度)。"""
    joints_deg = list(DEFAULT_JOINTS_DEG)
    overridden = []

    if args.joints is not None:
        joints_deg = parse_joint_list(args.joints)
        overridden.append("--joints")

    for i in range(JOINT_COUNT):
        value = getattr(args, f"j{i + 1}")
        if value is not None:
            joints_deg[i] = value
            overridden.append(f"--j{i + 1}")

    return joints_deg, overridden


def main():
    parser = build_parser()
    args = parser.parse_args()

    # --- 第 1 组: 运动目标 -------------------------------------------------
    try:
        joints_deg, joint_overridden = resolve_joints(args)
    except argparse.ArgumentTypeError as exc:
        parser.error(str(exc))

    # --- 第 2 组: 运动参数 (度制 -> 弧度制) --------------------------------
    v_deg = args.v if args.v is not None else DEFAULT_VEL_DEG
    a_deg = args.a if args.a is not None else DEFAULT_ACC_DEG
    motion_overridden = [
        name for name, val in (("--v", args.v), ("--a", args.a)) if val is not None
    ]

    # 内部换算: 协议要求 弧度 / 弧度每秒
    joints_rad = [math.radians(v) for v in joints_deg]
    v_rad = math.radians(v_deg)
    a_rad = math.radians(a_deg)

    payload = {
        "position": {"position": joints_rad},
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
        print("MoveJ Demo — 关节空间运动")
        print("=" * 60)
        print(f"目标关节角 (度): {[round(v, 4) for v in joints_deg]}")
        print(f"目标关节角 (弧度): {[round(v, 6) for v in joints_rad]}")
        print(f"速度: {round(v_deg, 4)} 度/秒  ({round(v_rad, 6)} rad/s)")
        print(f"加速度: {round(a_deg, 4)} 度/秒^2  ({round(a_rad, 6)} rad/s^2)")
        print(f"覆盖项: 目标={'是' if joint_overridden else '否(默认)'}"
              f"{'[' + ','.join(joint_overridden) + ']' if joint_overridden else ''}, "
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
            print("\n✅ MoveJ succeeded!")
            return 0

        print(f"\n❌ MoveJ failed (code={code}): {msg}")
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
