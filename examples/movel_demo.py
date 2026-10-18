#!/usr/bin/env python3
"""
MoveL Demo — 笛卡尔空间直线运动

向 GeniArm 发送 MoveL RPC 请求，控制机械臂末端从当前位置沿直线运动到目标位姿。

用法:
    python3 movel_demo.py                                     # 全部使用默认参数
    python3 movel_demo.py --pose 0.30 0 0.15 -108 0 -90
    python3 movel_demo.py --pose 0.30,0,0.15,-108,0,-90 --v 0.2
    python3 movel_demo.py --v 0.2                             # 只改运动参数

参数分两组，可各自独立覆盖（未指定的沿用默认值）:
    1. 运动目标: --pose x y z roll pitch yaw
                 x/y/z 单位 m；roll/pitch/yaw 为 ZYX 欧拉角，单位 度
    2. 运动参数: --v (m/s) / --a (m/s^2)

欧拉角约定: ZYX 顺规 (等价于 extrinsic XYZ), 即 R = Rz(yaw)·Ry(pitch)·Rx(roll),
            与 RViz2 中 "Roll Pitch Yaw (zyx)" 的显示一致。

写法说明: --pose 的 6 个数值支持「空格分隔」或「逗号分隔」两种写法。
          注意逗号形式若以 '-' 开头 (如 -108,0,-90) 会被 argparse 当成选项,
          此时请改用空格写法，或写成 --pose=0.3,0,0.15,-108,0,-90。
"""

import argparse
import json
import math
import sys

import requests

# ---------------------------------------------------------------------------
# RPC endpoint
# ---------------------------------------------------------------------------
URL_MOVEL = "http://127.0.0.1:50080/rpc/genisys_msgs/srv/MoveL"
TIMEOUT = 30.0

# ---------------------------------------------------------------------------
# 默认参数
# ---------------------------------------------------------------------------
# 末端目标位姿: x y z (m) + roll pitch yaw (ZYX 欧拉角, 度)
DEFAULT_POSE = (0.30, 0.0, 0.15, -108.0, 0.0, -90.0)
POSE_FIELDS = ("x", "y", "z", "roll", "pitch", "yaw")

# MoveL 运动参数
DEFAULT_V = 0.3     # m/s
DEFAULT_A = 0.2     # m/s^2


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


# 兼容旧调用名
rpy_to_quat = zyx_to_quat


def parse_float_list(value, count, label):
    """把 count 个 float 解析成列表。

    同时兼容:
        --pose 0.30 0 0.15 -108 0 -90       (空格分隔 -> nargs 列表)
        --pose 0.30,0,0.15,-108,0,-90       (逗号分隔 -> 单元素列表)
        --pose "0.30 0 0.15 -108 0 -90"     (整串空格分隔 -> 单元素列表)
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
        description="MoveL Demo — 笛卡尔空间直线运动 (位置 m, 姿态 ZYX 欧拉角 度)",
        formatter_class=argparse.RawDescriptionHelpFormatter,
        epilog=(
            "示例:\n"
            "  %(prog)s\n"
            "  %(prog)s --pose 0.30 0 0.15 -108 0 -90\n"
            "  %(prog)s --pose 0.30,0,0.15,-108,0,-90 --v 0.2 --a 0.1\n"
            "  %(prog)s --pose 0.35 0 0.20 -108 0 -90   # 只改目标位姿\n"
            "  %(prog)s --v 0.2                          # 只改运动参数\n"
        ),
    )

    # 第 1 组: 运动目标
    g_target = parser.add_argument_group(
        "运动目标 (x y z 单位 m; roll pitch yaw 为 ZYX 欧拉角, 单位 度)"
    )
    g_target.add_argument(
        "--pose", "-p", type=str, nargs="+", default=None, metavar="V",
        help="目标位姿，依次为 x y z roll pitch yaw，如 '--pose 0.30 0 0.15 -108 0 -90' "
             f"(默认: {' '.join(str(v) for v in DEFAULT_POSE)})",
    )

    # 第 2 组: 运动参数
    g_motion = parser.add_argument_group("运动参数 (速度/加速度)")
    g_motion.add_argument(
        "--v", type=float, default=None, metavar="M_S",
        help=f"直线运动速度 (m/s) (默认: {DEFAULT_V})",
    )
    g_motion.add_argument(
        "--a", type=float, default=None, metavar="M_S2",
        help=f"直线运动加速度 (m/s^2) (默认: {DEFAULT_A})",
    )

    # 其他
    g_other = parser.add_argument_group("其他")
    g_other.add_argument("--url-movel", default=URL_MOVEL,
                         help=f"MoveL RPC 地址 (默认: {URL_MOVEL})")
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


def post_rpc(url, payload, timeout):
    """发送一次 RPC，返回 (是否成功, 原始响应 dict, 错误信息)。"""
    resp = requests.post(url, json=payload, timeout=timeout)
    resp.raise_for_status()
    result = resp.json()
    code = result.get("error", {}).get("code", -1)
    ok = int(code) == 0
    msg = result.get("error", {}).get("msg", "unknown")
    return ok, result, msg


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

    # --- 第 2 组: 运动参数 -------------------------------------------------
    v = args.v if args.v is not None else DEFAULT_V
    a = args.a if args.a is not None else DEFAULT_A
    motion_overridden = [
        name for name, val in (("--v", args.v), ("--a", args.a)) if val is not None
    ]

    movel_payload = {
        "pose": {
            "position": {"x": x, "y": y, "z": z},
            "orientation": ori,
        },
        "v": v,
        "a": a,
    }

    if args.dry_run:
        if args.json_out:
            print(json.dumps(movel_payload, indent=2))
            return 0

        print("[dry-run] MoveL payload:")
        print(json.dumps(movel_payload, indent=2))
        print("\n[dry-run] 未发起实际请求。")
        return 0

    # --json: 只在结束时输出一行结构化结果, 不打印横幅
    quiet = args.json_out
    json_summary = {
        "movel": {"ok": None, "code": None, "msg": None},
        "movel_payload": movel_payload,
    }

    # 覆盖情况摘要 (两组分别汇报)
    if not quiet:
        print(f"参数覆盖: 运动目标={'是' if pose_overridden else '否(默认)'}"
              f"{'[' + ','.join(pose_overridden) + ']' if pose_overridden else ''} | "
              f"运动参数={'是' if motion_overridden else '否(默认)'}"
              f"{'[' + ','.join(motion_overridden) + ']' if motion_overridden else ''}")

    # ------------------------------------------------------------------
    # MoveL 直线运动
    # ------------------------------------------------------------------
    if not quiet:
        print("\n" + "=" * 60)
        print("MoveL — 直线运动到目标")
        print("=" * 60)
        print(f"目标位置 (m): x={x}, y={y}, z={z}  "
              f"{'| 覆盖: ' + ','.join(pose_overridden) if pose_overridden else '| 默认'}")
        print(f"目标姿态 (ZYX 欧拉角, 度): roll={roll}, pitch={pitch}, yaw={yaw}")
        print(f"四元数 (w,x,y,z): {[round(ori[k], 6) for k in ('w', 'x', 'y', 'z')]}")
        print(f"速度: {v} m/s, 加速度: {a} m/s^2  "
              f"{'(已覆盖)' if motion_overridden else '(默认)'}")
        print(f"Sending to {args.url_movel} ...")

    try:
        ok, result, msg = post_rpc(args.url_movel, movel_payload, args.timeout)
    except requests.exceptions.ConnectionError:
        if quiet:
            print(json.dumps({"error": f"无法连接到 {args.url_movel}"}))
        else:
            print(f"\n❌ 无法连接到 GeniArm ({args.url_movel})，请确认程序已启动。")
        return 1
    except requests.exceptions.RequestException as e:
        if quiet:
            print(json.dumps({"error": f"请求失败: {e}"}))
        else:
            print(f"\n❌ 请求失败: {e}")
        return 1

    json_summary["movel"] = {
        "ok": ok,
        "code": result.get("error", {}).get("code"),
        "msg": msg,
    }

    if quiet:
        print(json.dumps(json_summary))
        return 0 if ok else 1

    print(f"Response: {json.dumps(result, indent=2)}")
    if ok:
        print("\n✅ MoveL succeeded!")
        return 0

    print(f"\n❌ MoveL failed (code={result.get('error', {}).get('code')}): {msg}")
    return 1


if __name__ == "__main__":
    sys.exit(main())
