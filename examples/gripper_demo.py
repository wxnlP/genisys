#!/usr/bin/env python3
"""
Gripper Demo — 夹爪位置控制 / 力控

两个模式，二选一（都不给则是位置控制 + 全默认值）：

    --pos    位置控制  (GripperPos)
    --force  力控抓取  (GripperForce)

参数按顺序跟在开关后面，可以少给（缺的用默认值），也可以用逗号分隔。

    --pos    position  velocity  acceleration  effort
             目标位置   运动速度    运动加速度      力矩上限
             默认:     1.0       1.0         2.0       0.5
             说明:     0.0=夹紧, 1.0=全开

    --force  target_torque  switch_velocity  approach_velocity  approach_accel  closed_release
             力矩控制值      速度亏欠阈值      接近速度            接近加速度       闭合时释放力矩
             默认:  1.0 N·m        0.2 rad/s       1.0 rad/s           1.0 rad/s^2     true

用法:
    python3 gripper_demo.py                                  # 位置控制，全默认（全开）
    python3 gripper_demo.py --pos                            # 同上
    python3 gripper_demo.py --pos 0.5                        # 只给目标位置，其余默认
    python3 gripper_demo.py --pos 0 0.5 1.0 0.5              # 夹紧（全量参数）
    python3 gripper_demo.py --pos 0,0.5,1.0,0.5              # 逗号写法
    python3 gripper_demo.py --force                          # 力控抓取，全默认
    python3 gripper_demo.py --force 0.3                      # 只给夹持力矩，其余默认
    python3 gripper_demo.py --force 0.3 0.05 1.0 1.0 true    # 力控（全量参数）
    python3 gripper_demo.py --force 0.3 0.05 1.0 1.0 false   # 关闭闭合释放保护

注意: 逗号形式若以 '-' 开头 (如 -1,0,1,0.5) 会被 argparse 当成选项,
      此时请改用空格写法，或写成 --pos=... 形式。
"""

import argparse
import json
import sys

import requests

# ---------------------------------------------------------------------------
# RPC endpoint
# ---------------------------------------------------------------------------
URL_POS = "http://127.0.0.1:50080/rpc/genisys_msgs/srv/GripperPos"
URL_FORCE = "http://127.0.0.1:50080/rpc/genisys_msgs/srv/GripperForce"
TIMEOUT = 30.0

# ---------------------------------------------------------------------------
# 参数定义: 协议字段名 / 默认值 / 帮助文本，顺序即命令行顺序
# ---------------------------------------------------------------------------
POS_FIELDS = ("position", "velocity", "acceleration", "effort")
POS_DEFAULTS = (1.0, 1.0, 2.0, 0)
POS_HELP = (
    "目标位置 (0.0=夹紧, 1.0=全开)",
    "运动速度",
    "运动加速度",
    "力矩 / 电流上限",
)

FORCE_FIELDS = (
    "target_torque",
    "switch_velocity",
    "approach_velocity",
    "approach_accel",
    "closed_release",
)
FORCE_DEFAULTS = (0.5, 0.2, 1.0, 1.0, True)
FORCE_HELP = (
    "力矩控制值 N·m (夹持力矩)",
    "速度亏欠阈值 rad/s (实际速度落后规划速度这么多算碰到)",
    "接近速度 rad/s",
    "接近加速度 rad/s^2",
    "位置接近闭合时释放力矩 (true/false)",
)

TRUE_WORDS = {"1", "true", "t", "yes", "y", "on"}
FALSE_WORDS = {"0", "false", "f", "no", "n", "off"}


def split_tokens(value):
    """把 nargs 列表 / 逗号串 / 整串空格 三种写法统一拆成字符串列表。"""
    if isinstance(value, str):
        return [x for x in value.replace(",", " ").split() if x]
    return [x for item in value for x in str(item).replace(",", " ").split() if x]


def order_help(fields, helps):
    return "；".join(f"{i + 1}={f}({h})" for i, (f, h) in enumerate(zip(fields, helps)))


def resolve_pos(value):
    tokens = split_tokens(value)
    if len(tokens) > len(POS_FIELDS):
        raise argparse.ArgumentTypeError(
            f"--pos 最多 {len(POS_FIELDS)} 个参数，收到 {len(tokens)} 个。顺序: "
            + order_help(POS_FIELDS, POS_HELP)
        )
    values = list(POS_DEFAULTS)
    for i, token in enumerate(tokens):
        try:
            values[i] = float(token)
        except ValueError as exc:
            raise argparse.ArgumentTypeError(
                f"--pos 第 {i + 1} 个参数 ({POS_FIELDS[i]}) 必须是数字: {token}"
            ) from exc
    return dict(zip(POS_FIELDS, values))


def resolve_force(value):
    tokens = split_tokens(value)
    if len(tokens) > len(FORCE_FIELDS):
        raise argparse.ArgumentTypeError(
            f"--force 最多 {len(FORCE_FIELDS)} 个参数，收到 {len(tokens)} 个。顺序: "
            + order_help(FORCE_FIELDS, FORCE_HELP)
        )
    values = list(FORCE_DEFAULTS)
    for i, token in enumerate(tokens):
        if FORCE_FIELDS[i] == "closed_release":
            word = token.strip().lower()
            if word in TRUE_WORDS:
                values[i] = True
            elif word in FALSE_WORDS:
                values[i] = False
            else:
                raise argparse.ArgumentTypeError(
                    f"--force 第 {i + 1} 个参数 (closed_release) 需要 true/false，收到: {token}"
                )
            continue
        try:
            values[i] = float(token)
        except ValueError as exc:
            raise argparse.ArgumentTypeError(
                f"--force 第 {i + 1} 个参数 ({FORCE_FIELDS[i]}) 必须是数字: {token}"
            ) from exc
    return dict(zip(FORCE_FIELDS, values))


def build_parser():
    parser = argparse.ArgumentParser(
        description="Gripper Demo — 夹爪位置控制 (--pos) / 力控抓取 (--force)",
        formatter_class=argparse.RawDescriptionHelpFormatter,
        epilog=(
            "参数顺序:\n"
            f"  --pos    {'  '.join(POS_FIELDS)}\n"
            f"           {'  '.join(str(v) for v in POS_DEFAULTS)}  (默认值)\n"
            f"  --force  {'  '.join(FORCE_FIELDS)}\n"
            f"           {'  '.join(str(v) for v in FORCE_DEFAULTS)}  (默认值)\n"
        ),
    )

    modes = parser.add_argument_group("模式（二选一）")
    modes.add_argument(
        "--pos", nargs="*", default=None, metavar="V",
        help="位置控制。按顺序可给: " + order_help(POS_FIELDS, POS_HELP)
             + f"。默认: {' '.join(str(v) for v in POS_DEFAULTS)}",
    )
    modes.add_argument(
        "--force", nargs="*", default=None, metavar="V",
        help="力控抓取。按顺序可给: " + order_help(FORCE_FIELDS, FORCE_HELP)
             + f"。默认: {' '.join(str(v) for v in FORCE_DEFAULTS)}",
    )

    other = parser.add_argument_group("其他")
    other.add_argument(
        "--url", default=None,
        help=f"覆盖完整 RPC 地址 (默认按模式自动选择: {URL_POS} / {URL_FORCE})",
    )
    other.add_argument(
        "--timeout", type=float, default=TIMEOUT,
        help=f"请求超时秒数 (默认: {TIMEOUT})",
    )
    other.add_argument(
        "--dry-run", action="store_true",
        help="只打印将要发送的 payload，不实际发起请求",
    )
    other.add_argument(
        "--json", dest="json_out", action="store_true",
        help="以 JSON 输出结果 (便于脚本调用/自动化测试)，不打印横幅文本",
    )
    return parser


def main():
    parser = build_parser()
    args = parser.parse_args()

    if args.pos is not None and args.force is not None:
        parser.error("--pos 与 --force 不能同时使用，请二选一")

    try:
        if args.force is not None:
            mode = "force"
            payload = resolve_force(args.force)
            url = args.url or URL_FORCE
        else:
            mode = "pos"
            payload = resolve_pos(args.pos or [])
            url = args.url or URL_POS
    except argparse.ArgumentTypeError as exc:
        parser.error(str(exc))

    if args.dry_run:
        if args.json_out:
            print(json.dumps({"mode": mode, "url": url, "payload": payload}, indent=2))
            return 0
        print(f"[dry-run] mode={mode}  url={url}")
        print("[dry-run] payload (实际发送内容):")
        print(json.dumps(payload, indent=2))
        print("\n[dry-run] 未发起实际请求。")
        return 0

    quiet = args.json_out

    if not quiet:
        print("=" * 60)
        print("Gripper Demo — 夹爪力控" if mode == "force" else "Gripper Demo — 夹爪位置控制")
        print("=" * 60)
        if mode == "force":
            print(f"力矩控制值: {payload['target_torque']} N·m  "
                  f"速度亏欠阈值: {payload['switch_velocity']} rad/s")
            print(f"接近速度: {payload['approach_velocity']} rad/s  "
                  f"接近加速度: {payload['approach_accel']} rad/s^2  "
                  f"闭合释放: {'开' if payload['closed_release'] else '关'}")
        else:
            print(f"目标位置: {payload['position']}  (0.0=夹紧, 1.0=全开)")
            print(f"速度: {payload['velocity']}, 加速度: {payload['acceleration']}, "
                  f"力矩: {payload['effort']}")
        print(f"Sending to {url} ...")

    try:
        resp = requests.post(url, json=payload, timeout=args.timeout)
        resp.raise_for_status()
        result = resp.json()

        code = result.get("error", {}).get("code", -1)
        # code 可能是字符串 "0" 或整数 0，统一转为 int 比较
        ok = int(code) == 0
        msg = result.get("error", {}).get("msg", "unknown")

        if quiet:
            print(json.dumps({
                "ok": ok, "code": code, "msg": msg,
                "mode": mode, "url": url,
                "payload": payload, "response": result,
            }))
            return 0 if ok else 1

        print(f"\nResponse: {json.dumps(result, indent=2)}")
        if ok:
            print(f"\n✅ {mode} succeeded!")
            return 0

        print(f"\n❌ {mode} failed (code={code}): {msg}")
        return 1

    except requests.exceptions.ConnectionError:
        if quiet:
            print(json.dumps({"error": f"无法连接到 {url}"}))
        else:
            print(f"\n❌ 无法连接到 GeniArm ({url})，请确认程序已启动。")
        return 1


if __name__ == "__main__":
    sys.exit(main())
