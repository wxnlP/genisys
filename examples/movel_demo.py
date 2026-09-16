#!/usr/bin/env python3
"""
MoveL Demo — 笛卡尔空间直线运动

先用 MoveJ 到安全姿态，再用 MoveL 做小范围直线运动。

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

此外, 作为前置动作的 MoveJ 安全姿态也支持独立覆盖: --pre-joints / --pre-v / --pre-a,
不想执行前置 MoveJ 可加 --skip-pre。
"""

import argparse
import json
import math
import sys

import requests

# ---------------------------------------------------------------------------
# RPC endpoint
# ---------------------------------------------------------------------------
URL_MOVEJ = "http://127.0.0.1:50080/rpc/robotic_arm.protocols.MoveControlService/MoveJ"
URL_MOVEL = "http://127.0.0.1:50080/rpc/robotic_arm.protocols.MoveControlService/MoveL"
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

# 前置 MoveJ 安全姿态 (关节角度, 度)
DEFAULT_PRE_JOINTS_DEG = [0.0, 45.8366, -28.6479, 0.0, 0.0, 0.0]
DEFAULT_PRE_V_DEG = 45.8366  # = 0.8 rad/s
DEFAULT_PRE_A_DEG = 22.9183  # = 0.4 rad/s^2

JOINT_COUNT = 6


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


def parse_joint_list(value, count=JOINT_COUNT):
    return parse_float_list(value, count, "--pre-joints")


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
            "  %(prog)s --skip-pre                       # 跳过前置 MoveJ\n"
            "  %(prog)s --pre-joints 0 45 -30 0 0 0      # 前置安全姿态也自定义\n"
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

    # 前置 MoveJ 安全姿态
    g_pre = parser.add_argument_group("前置 MoveJ 安全姿态 (关节角, 度)")
    g_pre.add_argument(
        "--pre-joints", type=str, nargs="+", default=None, metavar="DEG",
        help=f"前置 MoveJ 的 6 个关节角 (度) (默认: {DEFAULT_PRE_JOINTS_DEG})",
    )
    g_pre.add_argument(
        "--pre-v", type=float, default=None, metavar="DEG_S",
        help=f"前置 MoveJ 速度 (度/秒) (默认: {DEFAULT_PRE_V_DEG})",
    )
    g_pre.add_argument(
        "--pre-a", type=float, default=None, metavar="DEG_S2",
        help=f"前置 MoveJ 加速度 (度/秒^2) (默认: {DEFAULT_PRE_A_DEG})",
    )
    g_pre.add_argument(
        "--skip-pre", action="store_true",
        help="跳过前置 MoveJ，直接执行 MoveL",
    )

    # 其他
    g_other = parser.add_argument_group("其他")
    g_other.add_argument("--url-movej", default=URL_MOVEJ,
                         help=f"MoveJ RPC 地址 (默认: {URL_MOVEJ})")
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


def resolve_pre_joints(args):
    """合并 --pre-joints (前置 MoveJ 关节角)。"""
    joints_deg = list(DEFAULT_PRE_JOINTS_DEG)
    overridden = []
    if args.pre_joints is not None:
        joints_deg = parse_joint_list(args.pre_joints)
        overridden.append("--pre-joints")
    return joints_deg, overridden


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
        pre_joints_deg, pre_overridden = resolve_pre_joints(args)
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

    # --- 前置 MoveJ 运动参数 -----------------------------------------------
    pre_v_deg = args.pre_v if args.pre_v is not None else DEFAULT_PRE_V_DEG
    pre_a_deg = args.pre_a if args.pre_a is not None else DEFAULT_PRE_A_DEG

    movej_payload = {
        "position": {"position": [math.radians(vv) for vv in pre_joints_deg]},
        "v": math.radians(pre_v_deg),
        "a": math.radians(pre_a_deg),
    }
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
            print(json.dumps({
                "movej_payload": None if args.skip_pre else movej_payload,
                "movel_payload": movel_payload,
            }, indent=2))
            return 0

        if not args.skip_pre:
            print("[dry-run] Step 1 MoveJ payload:")
            print(json.dumps(movej_payload, indent=2))
        print("\n[dry-run] Step 2 MoveL payload:")
        print(json.dumps(movel_payload, indent=2))
        print("\n[dry-run] 未发起实际请求。")
        return 0

    # --json: 只在结束时输出一行结构化结果, 不打印横幅
    quiet = args.json_out
    json_summary = {
        "movej": {"skipped": bool(args.skip_pre), "ok": None, "code": None, "msg": None},
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
    # Step 1: 前置 MoveJ 到安全姿态
    # ------------------------------------------------------------------
    if args.skip_pre:
        if not quiet:
            print("\n[跳过] 前置 MoveJ (--skip-pre)")
    else:
        if not quiet:
            print("\n" + "=" * 60)
            print("Step 1: MoveJ — 展开到安全姿态")
            print("=" * 60)
            print(f"目标关节角 (度): {[round(vv, 4) for vv in pre_joints_deg]}  "
                  f"{'| 覆盖: ' + ','.join(pre_overridden) if pre_overridden else '| 默认'}")
            print(f"速度: {round(pre_v_deg, 4)} 度/秒, 加速度: {round(pre_a_deg, 4)} 度/秒^2")
        try:
            ok, result, msg = post_rpc(args.url_movej, movej_payload, args.timeout)
        except requests.exceptions.ConnectionError:
            if quiet:
                print(json.dumps({"error": f"无法连接到 {args.url_movej}"}))
            else:
                print(f"\n❌ 无法连接到 GeniArm ({args.url_movej})，请确认程序已启动。")
            return 1
        except requests.exceptions.RequestException as e:
            if quiet:
                print(json.dumps({"error": f"Step 1 请求失败: {e}"}))
            else:
                print(f"\n❌ Step 1 请求失败: {e}")
            return 1

        json_summary["movej"] = {
            "skipped": False, "ok": ok,
            "code": result.get("error", {}).get("code"), "msg": msg,
        }
        if not quiet:
            print(f"Result: {'✅' if ok else '❌'} (code={result.get('error', {}).get('code')})")
        if not ok:
            if quiet:
                print(json.dumps(json_summary))
            else:
                print(f"MoveJ 失败，无法继续 ({msg})")
            return 1

    # ------------------------------------------------------------------
    # Step 2: MoveL 直线运动
    # ------------------------------------------------------------------
    if not quiet:
        print("\n" + "=" * 60)
        print("Step 2: MoveL — 直线运动到目标")
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
