#!/usr/bin/env python3
"""测 pitch 的重力（定 GIMBAL_PITCH_GRAVITY_FF 用）：模式 1 下让 pitch 慢慢匀速往上扫一趟、再往下扫一趟，按角度分格。
匀速时 J·dω/dt = 0，电流 = 重力 ± 摩擦（摩擦跟着运动方向反号）：上、下两趟取平均就是重力，差的一半是摩擦。
再拟合 重力 = A·cos(pitch)：gimbal.c 里前馈是 GIMBAL_PITCH_GRAVITY_FF × cos(编码器 pitch)，A 就是该填的值。

用法（程序在跑、云台在模式 1，pitch 的增益要能稳住、带积分；gimbal_watch.py 开着也行）。路径写全：
    python C:/Project/RM_Infantry/Test_Module/Gimbal/Gimbal_test_RS05/tools/pitch_gravity.py
    pitch_gravity.py --low -28 --high 25 --speed 5 --bin 4
- 电流用发给电机的总电流（速度环 + 现在的重力前馈），所以现在前馈给多少都不影响结果。
- 匀速是电脑一点点改 pitch_deg（约每 10 ms 一步）：板子上的斜坡每一步一下子走完，速度前馈是一串很短的脉冲，
  电流会带尖，所以每格取中位数。
- 扫完回到开始时的目标。保护和 pid_step.py 一样（振荡、顶限幅 → 切模式 0）。csv 存在 %TEMP%/pid_step/。
"""

import argparse
import csv
import math
import os
import struct
import sys
import tempfile
import time

sys.path.insert(0, os.path.dirname(os.path.abspath(__file__)))
import gimbal_watch as gw  # noqa: E402
import pid_step as ps  # noqa: E402

SKIP_DEG = 1.5      # 每一趟开头这么多度不算（起步加速、静摩擦脱开）


def sweep(bench, guard, offset, start, end, speed, label, rows):
    """电脑把 pitch_deg 从 start 匀速改到 end，边改边采。"""
    direction = 1.0 if end > start else -1.0
    t0 = time.perf_counter()
    while True:
        cmd = start + direction * speed * (time.perf_counter() - t0)
        done = (cmd - end) * direction >= 0
        if done:
            cmd = end
        bench.write_test_f32(offset, cmd)
        s = bench.sampler.read()
        if s["mode"] != 1:
            raise ps.Trip(("不在模式 1 了（被切走，或者电机掉线、锁住）", []))
        problem = guard.check(s, True)
        if problem:
            bench.relax()
            raise ps.Trip((f"{problem}，已切模式 0", []))
        a = s["pitch"]
        rows.append({"dir": label, "tick": s["tick"], "cmd": cmd, "angle": a["angle"], "rate": a["rate"],
                     "target": a["target"], "current": a["current"],
                     "I_int": a["speed_pid"]["Ki"] * a["speed_pid"]["integral"]})
        if done:
            return


def hold(bench, guard, seconds):
    end = time.perf_counter() + seconds
    while time.perf_counter() < end:
        s = bench.sampler.read()
        problem = guard.check(s, True)
        if problem:
            bench.relax()
            raise ps.Trip((f"{problem}，已切模式 0", []))


def median(values):
    v = sorted(values)
    n = len(v)
    return None if n == 0 else (v[n // 2] if n % 2 else 0.5 * (v[n // 2 - 1] + v[n // 2]))


def fit(points):
    """最小二乘：G = A·cos θ。points 是 (θ°, G)，返回 (A, 残差 RMS)。
    不拟合常数偏置：±30° 里 cos 只在 0.87~1 之间变，A 和偏置分不开（2026-11-12 合成数据试过）。"""
    cs = [math.cos(math.radians(t)) for t, _ in points]
    gs = [g for _, g in points]
    a = sum(c * g for c, g in zip(cs, gs)) / sum(c * c for c in cs)
    rms = (sum((a * c - g) ** 2 for c, g in zip(cs, gs)) / len(points)) ** 0.5
    return a, rms


def main():
    parser = argparse.ArgumentParser(description="测 pitch 的重力（定重力前馈用）")
    parser.add_argument("--low", type=float, default=-28.0, help="往上扫的起点（编码器角，度）")
    parser.add_argument("--high", type=float, default=25.0, help="往上扫的终点")
    parser.add_argument("--speed", type=float, default=5.0, help="扫的速度，度/秒")
    parser.add_argument("--bin", type=float, default=4.0, help="分格宽度，度")
    parser.add_argument("--elf", default=gw.DEFAULT_ELF)
    parser.add_argument("--openocd", default=gw.DEFAULT_OPENOCD)
    parser.add_argument("--cfg", default=gw.DEFAULT_CFG)
    parser.add_argument("--port", type=int, default=6666)
    args = parser.parse_args()
    args.axis = "pitch"
    sys.stdout.reconfigure(encoding="utf-8", errors="replace")

    bench = ps.Bench(args)
    try:
        run(bench, args)
    finally:
        bench.close()


def run(bench, args):
    s = bench.state()
    problems = gw.layout_problems(s)
    if problems:
        sys.exit("ELF 和板子里的程序对不上：\n  " + "\n  ".join(problems))
    test = struct.unpack(gw.TEST_FORMAT, bench.ocd.read_bytes(*bench.symbols["gimbal_test"]))
    if s["mode"] != 1 or s["lock"] != 0 or test[0] != 1:
        sys.exit(f"云台要先在模式 1（编码器）、没锁住（现在指令 {test[0]}、实际 {s['mode']}、锁 {s['lock']}）")
    if test[7] != 0.0:
        sys.exit("pitch 的方波开着（pitch_square_deg 不是 0），先关掉")
    p = s["pitch"]
    if not (p["min_deg"] + 1.0 <= args.low < args.high <= p["max_deg"] - 1.0):
        sys.exit(f"扫的范围 {args.low:g} ~ {args.high:g} 要在软限位 {p['min_deg']:g} ~ {p['max_deg']:g} 里面、各留 1° 以上")
    if p["speed_pid"]["Ki"] == 0.0:
        sys.exit("pitch 速度环 Ki 是 0：匀速时稳态误差会让电流不等于负载，先给一点 Ki")
    print(f"pitch 增益：速度环 Kp {p['speed_pid']['Kp']:g} Ki {p['speed_pid']['Ki']:g}，角度环 Kp {p['angle_pid']['Kp']:g}；"
          f"从 {args.low:g}° 到 {args.high:g}°，{args.speed:g}°/s，来回约 {2 * (args.high - args.low) / args.speed + 5:.0f} s")

    offset = gw.TEST_PITCH_OFFSET
    start_cmd = test[2]
    limit = p["speed_pid"]["outMax"]
    guard = ps.Guard("pitch", limit)
    rows = []
    reason = None
    try:
        bench.write_test_f32(offset, args.low)
        hold(bench, guard, 1.5)
        sweep(bench, guard, offset, args.low, args.high, args.speed, "up", rows)
        hold(bench, guard, 1.0)
        sweep(bench, guard, offset, args.high, args.low, args.speed, "down", rows)
    except ps.Trip as trip:
        reason = trip.args[0][0]
    except KeyboardInterrupt:
        reason = "Ctrl+C"
    finally:
        if reason is None or "模式 0" not in reason:
            bench.write_test_f32(offset, start_cmd)

    path = os.path.join(tempfile.gettempdir(), "pid_step", time.strftime("%H%M%S") + "_pitch_gravity.csv")
    os.makedirs(os.path.dirname(path), exist_ok=True)
    with open(path, "w", newline="", encoding="utf-8") as f:
        w = csv.DictWriter(f, fieldnames=list(rows[0].keys()) if rows else ["dir"])
        w.writeheader()
        w.writerows(rows)
    if reason:
        print(f"!! 没测完：{reason}")
        print(f"csv：{path}")
        return
    report(rows, args.low, args.high, args.bin, args.speed)
    print(f"csv：{path}")


def bin_table(rows, low, high, width):
    """每格：(中心角, 往上的中位数, 往下的中位数, 往上样本数, 往下样本数)；每一趟开头 SKIP_DEG 度不算。"""
    table = []
    lo = low
    while lo < high - 1e-6:
        hi = min(lo + width, high)
        up = [r["current"] for r in rows if r["dir"] == "up" and lo <= r["angle"] < hi and r["cmd"] >= low + SKIP_DEG]
        down = [r["current"] for r in rows if r["dir"] == "down" and lo <= r["angle"] < hi
                and r["cmd"] <= high - SKIP_DEG]
        if len(up) >= 5 and len(down) >= 5:
            table.append(((lo + hi) / 2, median(up), median(down), len(up), len(down)))
        lo += width
    return table


def report(rows, low, high, width, speed):
    table = bin_table(rows, low, high, width)
    print()
    print(f"{'角度':>6} {'往上 A':>8} {'往下 A':>8} {'重力 A':>8} {'摩擦 A':>8} {'样本':>9}")
    for center, mu, md, nu, nd in table:
        print(f"{center:+6.1f} {mu:+8.3f} {md:+8.3f} {(mu + md) / 2:+8.3f} {(mu - md) / 2:+8.3f} {nu:4d}/{nd:<4d}")
    points = [(c, (mu + md) / 2) for c, mu, md, _, _ in table]
    if len(points) < 3:
        print("有效的格太少，没法拟合")
        return None
    a0, rms0 = fit(points)
    friction = [(mu - md) / 2 for _, mu, md, _, _ in table]
    print()
    print(f"拟合 重力 = A·cos(pitch)：A = {a0:.3f} A，残差 RMS {rms0:.3f} A")
    print(f"摩擦（匀速 {speed:g}°/s 时）：{min(friction):.3f} ~ {max(friction):.3f} A，平均 {sum(friction) / len(friction):.3f} A")
    print(f"→ GIMBAL_PITCH_GRAVITY_FF 按第一个拟合填 {a0:.3f}（重力是确定的，按 100% 给，剩下的偏差交给速度环积分）")
    return a0, rms0, friction


if __name__ == "__main__":
    main()
