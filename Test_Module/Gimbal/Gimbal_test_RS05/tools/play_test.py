#!/usr/bin/env python3
"""晃动测试：yaw、pitch 两个电机都“定死”（模式 1 编码器闭环 + 一组硬一点的参数），用手扳炮管，
看炮管（C 板 IMU）相对电机（编码器）能动多少：电机不动、IMU 在动，就是电机到炮管之间的结构在晃。
再看“IMU − 编码器”的差和电流的关系：电流几乎不变、差一下跳一截是间隙；差跟着电流成比例地变是弹性。

用法（程序在跑；没有 OpenOCD 在跑就自己起一个，用完关掉）。路径写全：
    python C:/Project/RM_Infantry/Test_Module/Gimbal/Gimbal_test_RS05/tools/play_test.py
        切模式 1、换硬参数、pitch 放到水平，然后录 45 s：炮管抬到水平、云台变硬以后，轻轻上下扳、再左右扳
    play_test.py --seconds 30 --pitch-deg 0 --yaw-gains 0.025,0.25,8 --pitch-gains 0.02,0.2,12
- 硬参数是 2026-11-12 模式 1 下试过不振的（yaw 0.025 / 0.25、pitch 0.02 / 0.2 / 12）。**不要在模式 2 下用**（pitch 会振）。
- 录完不改回去：两个轴留在模式 1、硬参数，方便继续看。要放开：gimbal_watch.py set mode 0；C 板复位后回到 gimbal_config.h。
- 只能轻扳：电流限幅 1 A（约 0.67 N·m），扳得太狠电机顶不住、编码器会跟着动，结果里会标出来（那些采样不算“定死”）。
- 保护：编码器转速 400 ms 里换向 6 次以上（每次超过 20°/s，比手扳快得多）算自激，切模式 0。
- csv 存在 %TEMP%/pid_step/。
"""

import argparse
import csv
import os
import struct
import sys
import tempfile
import time

sys.path.insert(0, os.path.dirname(os.path.abspath(__file__)))
import gimbal_watch as gw  # noqa: E402
import pid_step as ps  # noqa: E402

LOCKED_DEG = 0.1        # 编码器离开定住的位置不到这么多，算“定死”
FLIP_DPS = 20.0


def parse_gains(text):
    kp, ki, akp = (float(v) for v in text.split(","))
    return {"speed_kp": kp, "speed_ki": ki, "angle_kp": akp}


def percentile(values, q):
    v = sorted(values)
    return v[min(len(v) - 1, max(0, int(round(q * (len(v) - 1)))))]


def analyze(rows, axis):
    """axis 是 "yaw" 或 "pitch"。差 = (IMU − 编码器) − 开始时的差，开始时 = 前 0.5 s 的中位数。"""
    t0 = rows[0]["t"]
    start = [r for r in rows if r["t"] - t0 < 0.5] or rows[:5]
    enc_ref = sorted(r[f"{axis}_enc"] for r in start)[len(start) // 2]
    diff_ref = sorted(ps.wrap180(r[f"{axis}_imu"] - r[f"{axis}_enc"]) for r in start)[len(start) // 2]
    for r in rows:
        r[f"{axis}_d"] = ps.wrap180(r[f"{axis}_imu"] - r[f"{axis}_enc"]) - diff_ref
        r[f"{axis}_e"] = ps.wrap180(r[f"{axis}_enc"] - enc_ref)
    enc = [r[f"{axis}_e"] for r in rows]
    imu = [ps.wrap180(r[f"{axis}_imu"] - rows[0][f"{axis}_imu"]) for r in rows]
    locked = [r for r in rows if abs(r[f"{axis}_e"]) < LOCKED_DEG]
    name = {"yaw": "yaw（左右）", "pitch": "pitch（上下）"}[axis]
    print(f"\n{name}：")
    print(f"  电机（编码器）：{min(enc):+.2f} ~ {max(enc):+.2f}°；在 ±{LOCKED_DEG}° 以内（定死）的采样占 "
          f"{100.0 * len(locked) / len(rows):.0f}%")
    print(f"  炮管（IMU）：{min(imu):+.2f} ~ {max(imu):+.2f}°")
    if len(locked) < 20:
        print("  定死的采样太少（扳得太狠，电机被扳动了），没法分开结构的晃动")
        return
    d = [r[f"{axis}_d"] for r in locked]
    cur = [r[f"{axis}_I"] for r in locked]
    print(f"  电机定死时，炮管相对电机：{min(d):+.2f} ~ {max(d):+.2f}°（5%~95%：{percentile(d, 0.05):+.2f} ~ "
          f"{percentile(d, 0.95):+.2f}°），这时电流 {min(cur):+.3f} ~ {max(cur):+.3f} A")
    # 按电流分格看差：电流几乎不变而差变一大截 = 间隙；差随电流成比例变 = 弹性
    lo, hi = percentile(cur, 0.02), percentile(cur, 0.98)
    width = max(0.02, (hi - lo) / 8.0)
    print(f"  {'电流 A':>14} {'样本':>5} {'差 最小':>8} {'中位':>7} {'最大':>7}")
    edge = lo
    while edge < hi + 1e-9:
        cell = [r[f"{axis}_d"] for r in locked if edge <= r[f"{axis}_I"] < edge + width]
        if len(cell) >= 5:
            print(f"  {edge:+.3f}~{edge + width:+.3f} {len(cell):5d} {min(cell):+8.2f} "
                  f"{sorted(cell)[len(cell) // 2]:+7.2f} {max(cell):+7.2f}")
        edge += width
    n = len(locked)
    mc, md = sum(cur) / n, sum(d) / n
    scc = sum((c - mc) ** 2 for c in cur)
    if scc > 1e-9:
        k = sum((c - mc) * (v - md) for c, v in zip(cur, d)) / scc
        rms = (sum((v - md - k * (c - mc)) ** 2 for c, v in zip(cur, d)) / n) ** 0.5
        print(f"  按直线拟合：每 1 A 差 {k:+.2f}°（弹性），拟合后剩下的散布 ±{rms:.2f}°（间隙和滞回大概在这里面）")


def main():
    parser = argparse.ArgumentParser(description="晃动测试：两个电机定死，看炮管相对电机晃多少")
    parser.add_argument("--seconds", type=float, default=45.0)
    parser.add_argument("--pitch-deg", type=float, default=0.0, help="pitch 定在哪（编码器角）")
    parser.add_argument("--yaw-gains", default="0.025,0.25,8", help="yaw 速度环 Kp,Ki,角度环 Kp")
    parser.add_argument("--pitch-gains", default="0.02,0.2,12", help="pitch 速度环 Kp,Ki,角度环 Kp")
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
    ocd, symbols = bench.ocd, bench.symbols
    s = bench.state()
    problems = gw.layout_problems(s)
    if problems:
        sys.exit("ELF 和板子里的程序对不上：\n  " + "\n  ".join(problems))
    if not s["valid"]:
        print("注意：IMU 现在无效（还在标定 / 加热？），IMU 那一列没意义")
    test_address = symbols["gimbal_test"][0]

    # 先切模式 1，再换硬参数（模式 2 下 pitch 用这组会振）
    ocd.write(test_address + gw.TEST_MODE_OFFSET, 8, [1])
    time.sleep(0.3)
    s = bench.state()
    if s["mode"] != 1:
        sys.exit(f"没进模式 1（实际模式 {s['mode']}、锁 {s['lock']}），先看 gimbal_watch.py --once")
    for axis, text in (("yaw", args.yaw_gains), ("pitch", args.pitch_gains)):
        for key, value in parse_gains(text).items():
            bench.set_gain(ps.GAINS[axis][key], value)
    bench.write_test_f32(gw.TEST_PITCH_OFFSET, args.pitch_deg)
    time.sleep(1.5)
    print(f"\n已定死：模式 1，yaw {args.yaw_gains}、pitch {args.pitch_gains}，pitch 在 {args.pitch_deg:g}°。"
          f"现在开始录 {args.seconds:g} s，轻轻扳炮管：上下、再左右。")

    found, start, end = ps.find_block(args.elf)

    def part(raw, name):
        address, size = found[name]
        return raw[address - start:address - start + size]

    rows = []
    flips = {"yaw": [], "pitch": []}
    last_sign = {"yaw": 0, "pitch": 0}
    t0 = time.perf_counter()
    next_print = 0.0
    reason = None
    try:
        while time.perf_counter() - t0 < args.seconds:
            raw = ocd.read_bytes(start, end - start)
            yaw = gw.unpack_axis(part(raw, "gimbal_yaw"))
            pitch = gw.unpack_axis(part(raw, "gimbal_pitch"))
            imu = struct.unpack(gw.INS_FORMAT, part(raw, "gimbal_imu"))
            mode = part(raw, "gimbal_mode")[0]
            t = time.perf_counter() - t0
            rows.append({"t": t, "tick": yaw["fb_tick"], "mode": mode,
                         "yaw_enc": yaw["angle"], "yaw_imu": imu[1], "yaw_I": yaw["current"], "yaw_rate": yaw["rate"],
                         "pitch_enc": pitch["angle"], "pitch_imu": imu[2], "pitch_I": pitch["current"],
                         "pitch_rate": pitch["rate"]})
            if mode != 1:
                reason = f"不在模式 1 了（实际模式 {mode}：电机掉线、被切走？）"
                break
            for axis, a in (("yaw", yaw), ("pitch", pitch)):
                sign = 1 if a["rate"] > FLIP_DPS else (-1 if a["rate"] < -FLIP_DPS else 0)
                if sign and last_sign[axis] and sign != last_sign[axis]:
                    flips[axis].append(t)
                if sign:
                    last_sign[axis] = sign
                flips[axis] = [x for x in flips[axis] if t - x <= 0.4]
                if len(flips[axis]) >= 6:
                    bench.relax()
                    reason = f"{axis} 编码器转速 400 ms 里换向 {len(flips[axis])} 次，像是自激，已切模式 0"
                    break
            if reason:
                break
            if t >= next_print:
                r = rows[-1]
                print(f"  {t:4.0f} s  pitch 编码器 {r['pitch_enc']:+7.2f} IMU {r['pitch_imu']:+7.2f} 电流 {r['pitch_I']:+.2f} | "
                      f"yaw 编码器 {r['yaw_enc']:+7.2f} IMU {r['yaw_imu']:+7.2f} 电流 {r['yaw_I']:+.2f}")
                next_print += 1.0
    except KeyboardInterrupt:
        reason = "Ctrl+C"

    path = os.path.join(tempfile.gettempdir(), "pid_step", time.strftime("%H%M%S") + "_play.csv")
    os.makedirs(os.path.dirname(path), exist_ok=True)
    with open(path, "w", newline="", encoding="utf-8") as f:
        w = csv.DictWriter(f, fieldnames=list(rows[0].keys()))
        w.writeheader()
        w.writerows(rows)
    dt = (rows[-1]["t"] - rows[0]["t"]) / max(1, len(rows) - 1) * 1000.0
    print(f"\n{len(rows)} 个采样，平均 {dt:.1f} ms 一个" + (f"；!! {reason}" if reason else ""))
    for axis in ("pitch", "yaw"):
        analyze(rows, axis)
    print(f"\ncsv：{path}")
    print("两个轴还在模式 1、硬参数（定死）。要放开：gimbal_watch.py set mode 0；复位回到 gimbal_config.h 的参数。")


if __name__ == "__main__":
    main()
