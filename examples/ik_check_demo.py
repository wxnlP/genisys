#!/usr/bin/env python3
"""
IKCheck Demo — 批量逆解查询

向 GeniArm 发送一批位姿，服务端对每个位姿做逆解（默认参考当前实测关节），
返回每个位姿是否有解，以及最优解关节角。位姿可以用 TCP 或法兰（flange）表达，
每个目标可以单独指定是否启用 FK 反代校验（do_fk）。

用法:
    python3 ik_check_demo.py
    python3 ik_check_demo.py --n 1000          # 批量性能测试（重复同一个可达位姿）
    python3 ik_check_demo.py --url http://127.0.0.1:50080/rpc/genisys_msgs/srv/IKCheck
"""

import argparse
import json
import time

import requests

DEFAULT_URL = "http://127.0.0.1:50080/rpc/genisys_msgs/srv/IKCheck"

FRAME_TCP = 1
FRAME_FLANGE = 2

STATUS_NAME = {
    0: "OK",
    1: "NO_SOLUTION",
    2: "INVALID_INPUT",
    3: "FK_VERIFY_WARN",
    4: "OTHER_ERRORS",
}


def make_pose(x, y, z, qw=1.0, qx=0.0, qy=0.0, qz=0.0):
    return {
        "position": {"x": x, "y": y, "z": z},
        "orientation": {"w": qw, "x": qx, "y": qy, "z": qz},
    }


def call_ik_check(url, targets, timeout=60.0):
    resp = requests.post(url, json={"request": targets}, timeout=timeout)
    resp.raise_for_status()
    return resp.json().get("results", [])


def print_results(results, targets):
    frames = {t["id"]: t["frame"] for t in targets}
    fk_flags = {t["id"]: t.get("do_fk", False) for t in targets}
    print(f"{'id':>5} {'frame':>6} {'fk':>3} {'status':<14} {'solved':<7} joint[rad]")
    print("-" * 96)
    for r in results:
        status = STATUS_NAME.get(int(r.get("status", -1)), "UNKNOWN")
        frame = {FRAME_TCP: "TCP", FRAME_FLANGE: "FLANGE"}.get(
            frames.get(r.get("id")), "-"
        )
        fk = "Y" if fk_flags.get(r.get("id")) else "N"
        joint = r.get("joint", {}).get("position", [])
        joint_text = (
            "[" + ", ".join(f"{v:8.4f}" for v in joint) + "]"
            if r.get("solved")
            else "-"
        )
        print(
            f"{r.get('id', 0):>5} {frame:>6} {fk:>3} {status:<14} "
            f"{str(r.get('solved', False)):<7} {joint_text}"
        )


def main():
    parser = argparse.ArgumentParser(description="IKCheck 批量逆解查询示例")
    parser.add_argument("--url", default=DEFAULT_URL, help=f"RPC 地址 (默认: {DEFAULT_URL})")
    parser.add_argument("--n", type=int, default=0, help="性能测试的批量大小；0 表示功能演示")
    parser.add_argument("--timeout", type=float, default=60.0, help="请求超时秒数")
    args = parser.parse_args()

    if args.n <= 0:
        targets = [
            # 可达的 TCP 位姿，启用 FK 反代
            {"id": 1, "frame": FRAME_TCP, "do_fk": True,
             "pose": make_pose(0.30, 0.0, 0.30)},
            # 同一个位姿按法兰解释（工具带旋转，结果通常不同）
            {"id": 2, "frame": FRAME_FLANGE, "do_fk": True,
             "pose": make_pose(0.30, 0.0, 0.30)},
            # 明显不可达
            {"id": 3, "frame": FRAME_TCP, "do_fk": True,
             "pose": make_pose(1.50, 0.0, 0.30)},
            # 未指定坐标系：服务端返回 INVALID_INPUT
            {"id": 4, "frame": 0, "do_fk": True,
             "pose": make_pose(0.30, 0.0, 0.30)},
            # 关掉 FK 反代：只信解析解 + 关节限位，速度更快
            {"id": 5, "frame": FRAME_TCP, "do_fk": False,
             "pose": make_pose(0.30, 0.0, 0.30)},
        ]
        print(f"Sending {len(targets)} targets to {args.url} ...")
        results = call_ik_check(args.url, targets, args.timeout)
        print_results(results, targets)
        return 0

    targets = [
        {"id": i, "frame": FRAME_TCP, "do_fk": True,
         "pose": make_pose(0.30, 0.0, 0.30)}
        for i in range(args.n)
    ]
    print(f"Batch n={args.n} to {args.url} ...")
    t0 = time.perf_counter()
    results = call_ik_check(args.url, targets, args.timeout)
    elapsed_ms = (time.perf_counter() - t0) * 1000.0

    ok = sum(1 for r in results if r.get("solved"))
    ids_ok = all(int(r.get("id", -1)) == i for i, r in enumerate(results))
    print(f"返回 {len(results)} 项，有解 {ok} 项，id 顺序对齐: {ids_ok}")
    print(f"客户端往返耗时 {elapsed_ms:.2f} ms（{elapsed_ms / max(1, args.n):.2f} ms/pose）")
    return 0


if __name__ == "__main__":
    raise SystemExit(main())
