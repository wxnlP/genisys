#!/usr/bin/env python3
"""
Gripper Demo — 夹爪开合

向 GeniArm 发送 Gripper RPC 请求，控制夹爪运动到指定开合位置。

用法:
    python3 gripper_demo.py                                    # 全部使用默认参数
    python3 gripper_demo.py --gripper 1.0 1.0 2.0 0.5
    python3 gripper_demo.py --gripper 0,0.5,1.0,0.5            # 夹紧
    python3 gripper_demo.py --pos 0.5                          # 只改目标位置

参数分两组，可各自独立覆盖（未指定的沿用默认值）:
    1. 开合目标: --gripper pos v a e
                 也可用 --pos/--v/--a/--e 单独指定某一项

参数说明 (取值均为归一化值, 未标注单位):
    pos  目标开合位置, 0.0=夹紧, 1.0=全开 (默认 1.0)
    v    运动速度                          (默认 1.0)
    a    运动加速度                        (默认 2.0)
    e    力矩 / 电流上限                    (默认 0.5)

写法说明: --gripper 的 4 个数值支持「空格分隔」或「逗号分隔」两种写法。
          注意逗号形式若以 '-' 开头 (如 -1,0,1,0.5) 会被 argparse 当成选项,
          此时请改用空格写法，或写成 --gripper=... 形式。
"""

import argparse
import json
import sys

import requests

# ---------------------------------------------------------------------------
# RPC endpoint
# ---------------------------------------------------------------------------
URL = "http://127.0.0.1:50080/rpc/robotic_arm.protocols.MoveControlService/Gripper"
TIMEOUT = 30.0

# ---------------------------------------------------------------------------
# 默认参数: pos v a e
# ---------------------------------------------------------------------------
DEFAULT_GRIPPER = (1.0, 1.0, 2.0, 0.5)
# 缩写 -> 协议字段名
FIELD_MAP = (
    ("pos", "position"),
    ("v", "velocity"),
    ("a", "acceleration"),
    ("e", "effort"),
)
SHORTCUT_HELP = {
    "pos": "目标开合位置 (0.0=夹紧, 1.0=全开)",
    "v": "运动速度",
    "a": "运动加速度",
    "e": "力矩 / 电流上限",
}


def parse_float_list(value, count, label):
    """把 count 个 float 解析成列表。

    同时兼容:
        --gripper 1.0 1.0 2.0 0.5        (空格分隔 -> nargs 列表)
        --gripper 1.0,1.0,2.0,0.5        (逗号分隔 -> 单元素列表)
        --gripper "1.0 1.0 2.0 0.5"      (整串空格分隔 -> 单元素列表)
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


def parse_gripper(value):
    return parse_float_list(value, 4, "--gripper")


def build_parser():
    parser = argparse.ArgumentParser(
        description="Gripper Demo — 夹爪开合 (pos: 0.0=夹紧, 1.0=全开)",
        formatter_class=argparse.RawDescriptionHelpFormatter,
        epilog=(
            "示例:\n"
            "  %(prog)s\n"
            "  %(prog)s --gripper 1.0 1.0 2.0 0.5        # 全开 (pos v a e)\n"
            "  %(prog)s --gripper 0 0.5 1.0 0.5          # 夹紧\n"
            "  %(prog)s --gripper 1.0,1.0,2.0,0.5        # 逗号写法\n"
            "  %(prog)s --pos 0.5                        # 只改目标位置\n"
            "  %(prog)s --pos 0.5 --v 2.0                # 混用单项\n"
            "  %(prog)s --url http://10.0.0.5:50080 --gripper 1.0 1.0 2.0 0.5\n"
        ),
    )

    # 第 1 组: 开合目标
    g_target = parser.add_argument_group("开合目标 (pos v a e)")
    g_target.add_argument(
        "--gripper", "-g", type=str, nargs="+", default=None, metavar="V",
        help="夹爪参数，依次为 pos v a e，如 '--gripper 1.0 1.0 2.0 0.5' "
             f"(默认: {' '.join(str(v) for v in DEFAULT_GRIPPER)})",
    )
    for short, _field in FIELD_MAP:
        g_target.add_argument(
            f"--{short}", type=float, default=None, metavar="V",
            help=f"{short}: {SHORTCUT_HELP[short]}，可与其他单项混用",
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


def resolve_gripper(args):
    """合并 --gripper 与单项参数，返回 (协议字段字典, 覆盖项列表)。"""
    values = list(DEFAULT_GRIPPER)
    overridden = []

    if args.gripper is not None:
        values = parse_gripper(args.gripper)
        overridden.append("--gripper")

    for i, (short, _field) in enumerate(FIELD_MAP):
        value = getattr(args, short)
        if value is not None:
            values[i] = value
            overridden.append(f"--{short}")

    return {field: v for (_s, field), v in zip(FIELD_MAP, values)}, overridden


def main():
    parser = build_parser()
    args = parser.parse_args()

    try:
        gripper, overridden = resolve_gripper(args)
    except argparse.ArgumentTypeError as exc:
        parser.error(str(exc))

    payload = dict(gripper)

    if args.dry_run:
        if args.json_out:
            print(json.dumps(payload, indent=2))
            return 0
        print("[dry-run] payload (实际发送内容):")
        print(json.dumps(payload, indent=2))
        print("\n[dry-run] 未发起实际请求。")
        return 0

    quiet = args.json_out

    if not quiet:
        print("=" * 60)
        print("Gripper Demo — 夹爪开合")
        print("=" * 60)
        print(f"目标位置: {gripper['position']}  (0.0=夹紧, 1.0=全开)")
        print(f"速度: {gripper['velocity']}, 加速度: {gripper['acceleration']}, "
              f"力矩: {gripper['effort']}")
        print(f"覆盖项: {'是' if overridden else '否(默认)'}"
              f"{'[' + ','.join(overridden) + ']' if overridden else ''}")
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
            print("\n✅ Gripper succeeded!")
            return 0

        print(f"\n❌ Gripper failed (code={code}): {msg}")
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
            print("\n❌ 请求超时。")
        return 1
    except requests.exceptions.RequestException as e:
        if quiet:
            print(json.dumps({"error": f"请求失败: {e}"}))
        else:
            print(f"\n❌ 请求失败: {e}")
        return 1


if __name__ == "__main__":
    sys.exit(main())
