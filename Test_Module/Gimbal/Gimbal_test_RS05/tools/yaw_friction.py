#!/usr/bin/env python3
"""Gimbal_test_RS05：测 yaw 的摩擦（匀速转时的电流），给 gimbal_config.h 里的 yaw 摩擦前馈定参数。

原理：编码器模式（模式 1）下转盘匀速转时 J·dω/dt = 0，电机出的电流全用来抵摩擦。摩擦只看电机相对底盘的转动，
所以机身放着不动、转盘匀速转，和转盘对地不动、底盘反着转是一回事，不用转底盘。

用法（程序已经烧进板子在跑，yaw 电机接好、有 24 V）。在哪个目录下运行都行，路径写全：
    python C:/Project/RM_Infantry/Test_Module/Gimbal/Gimbal_test_RS05/tools/yaw_friction.py
        切到模式 1，转盘按固件的斜坡速度先正转、再反转，每趟取中间匀速的一整圈，算平均电流，
        追加到 tools/yaw_friction.csv。测完切回原来的模式。开始前要输入 y 确认。
    python C:/Project/RM_Infantry/Test_Module/Gimbal/Gimbal_test_RS05/tools/yaw_friction.py fit
        把 yaw_friction.csv 里的结果拟合成 电流 = i_c·sgn(ω) + b·ω + i_0，给出 gimbal_config.h 里该填的数。

- 转速就是固件的斜坡速度（gimbal_config.h 的 GIMBAL_RAMP_DEG_PER_MS，0.1 即 100°/s），脚本改不了。
  要测别的转速：改它、重新编译烧录，再跑一次，结果都追加在同一个 csv 里，最后一起 fit。
  斜坡比 GIMBAL_SPEED_LIMIT_DPS（角度环输出限幅）的 0.75 倍还快时，要把它一起调大，不然电机跟不上，脚本会停下。
  测完记得把两个都改回来。
- 每趟转一圈多（匀速段一整圈，加上起转、刹车的路程；最多 --max-turn 度），转盘上不能接线、不能有东西挡着。
  yaw 有限位（gimbal_config.h 的 YAW_MIN/MAX 不是 ±180）时不能用。
- 换了机械（装上云台负载、滑环……）摩擦就变了：把 csv 删掉（或者用 --log 换一个文件）重新测。
- 怎么转：脚本一直把 gimbal_test.yaw_deg 写在斜坡目标前面 90°，斜坡就一直按最大速度走；快到头时写终点。
- 只取匀速段：一趟开始 0.4 s 以后、离终点还有 15° 加 0.1 s 路程之前，并且转速和斜坡速度差不到 10% 的样本。
  电流用发给电机的那个（gimbal_yaw.motor.current_a，乘方向），速度环和前馈都在里面，所以前馈开着测也一样；
  另外单独平均速度环的输出，两者之差就是前馈，用来检查前馈的符号。
- 保护：跟随误差超过 45°、转速超出斜坡速度太多、电机掉线或报故障、云台退到无力、斜坡 1 s 不动
  → 马上切模式 0（无力）并退出。Ctrl+C：把目标停在当前位置（电机停住），再切回原来的模式。
- 连接（先连 127.0.0.1:6666，没有就自己起一个 OpenOCD）、变量地址、结构体布局都用同一目录的 gimbal_watch.py，
  那边改了这里跟着用。CLion 的调试会话开着时连不上，先停掉；可以和 gimbal_watch.py 同时开。
"""

import argparse
import csv
import math
import os
import re
import statistics
import struct
import sys
import time
from datetime import datetime

HERE = os.path.dirname(os.path.abspath(__file__))
if HERE not in sys.path:
    sys.path.insert(0, HERE)
import gimbal_watch as gw  # noqa: E402

DEFAULT_LOG = os.path.join(HERE, "yaw_friction.csv")
CSV_FIELDS = ("time", "ramp_dps", "v_dps", "i_a", "i_pid_a", "i_lo_a", "i_hi_a", "i_std_a", "cruise_deg", "samples")
CONFIG_H = os.path.join(gw.PROJECT_DIR, "App", "gimbal", "gimbal_config.h")

LEAD_DEG = 90.0          # 目标写在斜坡前面多少度：要小于 180（不限位时按最短方向走），又要够大，斜坡追不上
SETTLE_S = 0.4           # 一趟开始以后多久才算匀速（起转、转速稳下来）
BRAKE_DEG = 15.0         # 离终点还剩 BRAKE_DEG + BRAKE_S 秒的路程时就不算了
BRAKE_S = 0.1
SPEED_TOL = 0.10         # 匀速段的转速和斜坡速度最多差多少（比例）
BIN_DEG = 10.0           # 按位置分格，看摩擦随位置怎么变
FOLLOW_LIMIT_DEG = 45.0  # 保护：跟随误差（目标 − 角度）。起转时会落后 v²/(2·角加速度)，300°/s 时约 8°
STALL_S = 1.0            # 保护：斜坡目标这么久没往前走
OFFLINE_S = 0.2          # 保护：电机反馈的时间戳这么久没变
RAMP_MARGIN = 0.75       # 斜坡速度最多是角度环输出限幅的多少
FF_RATIO = 0.8           # fit 给建议值时最多按测出来的多少给


class Abort(Exception):
    """要马上停下（切模式 0）的情况。"""


def wrap180(deg):
    return (deg + 180.0) % 360.0 - 180.0


def f32_word(value):
    return struct.unpack("<I", struct.pack("<f", value))[0]


def read_axis(ocd, sym):
    """一次读完 gimbal_yaw。电流按云台正方向（俯视逆时针为正），和 rate、angle 同号。"""
    axis = struct.unpack(gw.AXIS_FORMAT, ocd.read_bytes(*sym["gimbal_yaw"]))
    motor = struct.unpack(gw.MOTOR_FORMAT, axis[0])
    return {"t": time.perf_counter(), "min": axis[3], "max": axis[4],
            "angle": axis[7], "rate": axis[8], "target": axis[9],
            "current": axis[1] * motor[2], "pid": gw.unpack_pid(axis[6])["out"],
            "enable": motor[1], "fault": motor[7], "fb_tick": motor[8]}


def write_yaw(ocd, sym, deg):
    ocd.write(sym["gimbal_test"][0] + gw.TEST_YAW_OFFSET, 32, [f32_word(deg)])


def write_mode(ocd, sym, mode):
    ocd.write(sym["gimbal_test"][0] + gw.TEST_MODE_OFFSET, 8, [mode])     # 枚举只占 1 字节，按 8 位写


def read_config_macros():
    """源码里的斜坡和角度环输出限幅，只用来在提示里对照（板子里的程序可能是别的版本）。"""
    values = {}
    try:
        with open(CONFIG_H, encoding="utf-8", errors="replace") as f:
            text = f.read()
    except OSError:
        return values
    for name in ("GIMBAL_RAMP_DEG_PER_MS", "GIMBAL_SPEED_LIMIT_DPS"):
        match = re.search(rf"^[ \t]*#define[ \t]+{name}[ \t]+\(?([-+]?[0-9.]+)[fF]?\)?", text, re.M)
        if match:
            values[name] = float(match.group(1))
    return values


class Guard:
    """每个样本都查一遍，不对就抛 Abort。"""

    def __init__(self, speed_bound):
        self.speed_bound = speed_bound
        self.fb_tick = None
        self.fb_change = None

    def check(self, s):
        if s["fault"]:
            raise Abort(f"yaw 电机报故障 0x{s['fault']:02X}（gimbal_watch.py --once 能看是哪一位）")
        if not s["enable"]:
            raise Abort("云台退到了无力：gimbal_mode 被改了，或者闭环时电机掉过线、云台被锁住")
        if s["fb_tick"] != self.fb_tick:
            self.fb_tick, self.fb_change = s["fb_tick"], s["t"]
        elif s["t"] - self.fb_change > OFFLINE_S:
            raise Abort(f"yaw 电机 {OFFLINE_S * 1000:.0f} ms 没回话")
        lag = wrap180(s["target"] - s["angle"])
        if abs(lag) > FOLLOW_LIMIT_DEG:
            raise Abort(f"电机跟不上：目标和角度差了 {lag:+.1f}°（电流限幅、GIMBAL_SPEED_LIMIT_DPS 够不够？转盘被挡住了？）")
        if abs(s["rate"]) > self.speed_bound:
            raise Abort(f"转速 {s['rate']:+.0f}°/s，超过了 {self.speed_bound:.0f}°/s")


def run_pass(ocd, sym, guard, sign, cover_deg, max_turn_deg, speed_limit):
    """往 sign（+1 / −1）方向转一趟。返回 (样本, 斜坡速度, 匀速段起点, 匀速段终点)，位置都是这一趟走过的度数。"""
    first = read_axis(ocd, sym)
    guard.check(first)
    start = unwrapped = last = first["target"]
    t0 = first["t"]
    samples = []
    ramp = end = cruise_lo = cruise_hi = None
    written = None
    end_written = False
    moved = (t0, 0.0)
    next_print = t0 + 1.0
    while True:
        s = read_axis(ocd, sym)
        guard.check(s)
        unwrapped += wrap180(s["target"] - last)      # 两次采样之间斜坡走不到 180°
        last = s["target"]
        s["progress"] = progress = sign * (unwrapped - start)
        s["elapsed"] = s["t"] - t0
        samples.append(s)

        if progress > moved[1] + 0.5:
            moved = (s["t"], progress)
        elif s["t"] - moved[0] > STALL_S:
            raise Abort(f"斜坡目标 {STALL_S:.0f} s 没往前走：gimbal_test.yaw_deg 被别的程序改了？")

        if end is None and s["elapsed"] >= SETTLE_S:
            # 斜坡速度用固件的时间（电机反馈的 tick）算：斜坡每毫秒走 GIMBAL_RAMP_DEG_PER_MS 度。
            # 从 0.05 s 起算，那时第一个目标早就写进去了
            ref = next(x for x in samples if x["elapsed"] >= 0.05)
            dt_ms = s["fb_tick"] - ref["fb_tick"]
            if dt_ms <= 0:
                raise Abort("电机反馈的时间戳没在走")
            ramp = (progress - ref["progress"]) / dt_ms * 1000.0
            if ramp < 1.0:
                raise Abort(f"斜坡几乎没动（{ramp:.2f}°/s）")
            if ramp > RAMP_MARGIN * speed_limit:
                raise Abort(f"斜坡 {ramp:.0f}°/s 太接近 GIMBAL_SPEED_LIMIT_DPS（{speed_limit:.0f}°/s）："
                            f"速度环目标限在它以内，电机会跟不上。把它调到 {ramp / RAMP_MARGIN:.0f} 以上再测")
            guard.speed_bound = 1.5 * ramp + 50.0
            brake = BRAKE_DEG + ramp * BRAKE_S
            cruise_lo = progress
            end = min(progress + cover_deg + brake, max_turn_deg)
            cruise_hi = end - brake

        remaining = None if end is None else end - progress
        if remaining is not None and remaining <= LEAD_DEG:
            if not end_written:
                write_yaw(ocd, sym, wrap180(start + sign * end))
                end_written = True
        else:
            goal = unwrapped + sign * LEAD_DEG
            if written is None or abs(goal - written) >= 1.0:
                write_yaw(ocd, sym, wrap180(goal))
                written = goal
        if remaining is not None and remaining < 0.05:
            break

        if s["t"] >= next_print:
            print(f"    {sign * progress:+7.0f}°  转速 {s['rate']:+7.1f} °/s  电流 {s['current']:+.3f} A", flush=True)
            next_print = s["t"] + 1.0

    # 斜坡到了终点：等电机停稳，下一趟从静止开始
    t_end = time.perf_counter()
    quiet = None
    while time.perf_counter() - t_end < 2.0:
        s = read_axis(ocd, sym)
        guard.check(s)
        if abs(s["rate"]) < 3.0:
            quiet = quiet or s["t"]
            if s["t"] - quiet > 0.2:
                break
        else:
            quiet = None
    return samples, ramp, cruise_lo, cruise_hi


def summarize(samples, sign, ramp, lo, hi):
    """匀速段的平均。电流按位置分格平均后的最小、最大用绝对值（两个方向好比）。"""
    window = [s for s in samples if lo <= s["progress"] <= hi]
    cruise = [s for s in window if abs(sign * s["rate"] - ramp) <= SPEED_TOL * ramp]
    if len(cruise) < 20:
        raise Abort(f"匀速段只有 {len(cruise)} 个样本（窗口里 {len(window)} 个）：转速一直不稳？")
    bins = {}
    for s in cruise:
        bins.setdefault(int((s["angle"] + 180.0) // BIN_DEG), []).append(s["current"])
    bin_means = [abs(statistics.fmean(v)) for v in bins.values() if len(v) >= 3]
    duration = cruise[-1]["t"] - cruise[0]["t"]
    return {
        "ramp_dps": ramp,
        "v_dps": statistics.fmean(s["rate"] for s in cruise),
        "v_std": statistics.pstdev(s["rate"] for s in cruise),
        "i_a": statistics.fmean(s["current"] for s in cruise),
        "i_pid_a": statistics.fmean(s["pid"] for s in cruise),
        "i_std_a": statistics.pstdev(s["current"] for s in cruise),
        "i_lo_a": min(bin_means) if bin_means else float("nan"),
        "i_hi_a": max(bin_means) if bin_means else float("nan"),
        "cruise_deg": hi - lo,
        "samples": len(cruise),
        "rejected": len(window) - len(cruise),
        "hz": len(window) / duration if duration > 0 else 0.0,
    }


def print_result(index, sign, r):
    ff = r["i_a"] - r["i_pid_a"]
    print(f"  第 {index} 趟（{'+' if sign > 0 else '−'}）：斜坡 {r['ramp_dps']:.1f}°/s，匀速段 {r['cruise_deg']:.0f}°、"
          f"{r['samples']} 个样本（约 {r['hz']:.0f} Hz）")
    print(f"    转速 {r['v_dps']:+.1f} °/s（±{r['v_std']:.1f}），电流 {r['i_a']:+.4f} A（±{r['i_std_a']:.3f}；"
          f"按 {BIN_DEG:.0f}° 分格平均 {r['i_lo_a']:.3f} ~ {r['i_hi_a']:.3f} A），其中速度环 {r['i_pid_a']:+.4f}、前馈 {ff:+.4f}")
    if r["rejected"] > 0.2 * (r["samples"] + r["rejected"]):
        print(f"    注意：匀速段里有 {r['rejected']} 个样本转速差了 {SPEED_TOL:.0%} 以上，转速不太稳")
    if abs(ff) > 0.005 and ff * sign < 0:
        print("    注意：前馈和转速反号，补摩擦的方向反了（应该和转速同号）。查 gimbal.c 里 yaw 摩擦前馈的符号")


def append_log(path, results):
    new = not os.path.exists(path)
    with open(path, "a", newline="", encoding="utf-8") as f:
        writer = csv.writer(f)
        if new:
            writer.writerow(CSV_FIELDS)
        stamp = datetime.now().isoformat(timespec="seconds")
        for r in results:
            writer.writerow([stamp] + [f"{r[k]:.6g}" if k != "samples" else r[k] for k in CSV_FIELDS[1:]])


def read_test(ocd, sym):
    return struct.unpack(gw.TEST_FORMAT, ocd.read_bytes(*sym["gimbal_test"]))


def measure(args):
    sym = gw.find_symbols(args.elf)
    ocd, proc = gw.connect(args)
    original_mode = None
    finished = False
    try:
        state = gw.read_state(ocd, sym)
        problems = gw.layout_problems(state)
        if problems:
            sys.exit("读到的数和 ELF 对不上，什么都没写：\n  " + "\n  ".join(problems)
                     + "\nELF 和板子里的程序可能不是同一次编译的：重新烧一遍，或者用 --elf 指定烧进去的那个。")
        test = read_test(ocd, sym)
        mode, square = test[0], test[6]
        if mode == 2:
            sys.exit("现在是模式 2（IMU）：C 板应该装在转盘上，这个测试要转好几圈，会把线绕上。先 set mode 0，把 C 板拿下来。")
        if square != 0.0:
            sys.exit(f"gimbal_test.yaw_square_deg 是 {square:g}，方波会让目标跳。先改成 0。")
        if state["max_deg"] - state["min_deg"] < 360.0:
            sys.exit(f"yaw 有限位（{state['min_deg']:g} ~ {state['max_deg']:g}），这个测试要转一圈以上，不能用。")
        offline = state["fb_tick"] == 0 or gw.tick_age(state, state["fb_tick"]) > 100
        if offline or state["fault"]:
            sys.exit("yaw 电机没回话或者报故障，先用 gimbal_watch.py --once 查。")
        speed_limit = state["angle_pid"]["outMax"]
        source = read_config_macros()
        ramp_src = source.get("GIMBAL_RAMP_DEG_PER_MS")
        print(f"yaw：实际模式 {state['mode']}，角度 {state['angle']:+.1f}°，电流限幅 {state['speed_pid']['outMax']:g} A，"
              f"角度环输出限幅 {speed_limit:g}°/s。"
              + ("" if ramp_src is None else f"源码里的斜坡是 {ramp_src:g}°/ms（{ramp_src * 1000:g}°/s）。"))
        print(f"转盘要按斜坡速度先正转、再反转，一共 {args.passes} 趟，每趟一圈多（最多 {args.max_turn:g}°）。"
              "转盘上不能接线、不能有东西挡着。")
        print("确认请输入 y 回车：", end="", flush=True)
        answer = gw.read_stdin_line()
        if answer is None or answer.strip().lower() != "y":
            sys.exit("没动。")

        original_mode = mode
        if mode != 1:
            write_mode(ocd, sym, 1)
            time.sleep(0.3)             # 切模式时 yaw_deg 被同步成当前角，RS05 使能约 4 ms
        actual = ocd.read(sym["gimbal_mode"][0], 8, 1)[0]
        s = read_axis(ocd, sym)
        if actual != 1 or not s["enable"]:
            raise Abort(f"切到模式 1 以后，实际模式是 {actual}、电机 enable {s['enable']}（云台被锁住了？）")

        guard = Guard(speed_limit + 100.0)
        results = []
        sign = 1
        for index in range(1, args.passes + 1):
            print(f"  第 {index} 趟（{'+' if sign > 0 else '−'}）……", flush=True)
            samples, ramp, lo, hi = run_pass(ocd, sym, guard, sign, args.cover, args.max_turn, speed_limit)
            result = summarize(samples, sign, ramp, lo, hi)
            print_result(index, sign, result)
            results.append(result)
            sign = -sign
        append_log(args.log, results)
        finished = True

        ramp = statistics.fmean(r["ramp_dps"] for r in results)
        print(f"\n记进了 {args.log}（{len(results)} 行）。")
        if ramp_src is not None and abs(ramp - ramp_src * 1000.0) > 0.05 * ramp_src * 1000.0:
            print(f"注意：实测斜坡 {ramp:.1f}°/s，源码里是 {ramp_src * 1000:g}°/s，板子里的程序可能不是现在的源码。")
        if min(r["cruise_deg"] for r in results) < 359.0:
            print(f"注意：匀速段没转满一圈（--max-turn {args.max_turn:g} 不够），按位置的最小、最大不全。")
        print("换个转速再测：改 gimbal_config.h 的 GIMBAL_RAMP_DEG_PER_MS（比 GIMBAL_SPEED_LIMIT_DPS 的 0.75 倍快时"
              "要一起调大），重新编译烧录后再跑一次。测完两三个转速以后拟合：")
        print(f"    python {os.path.abspath(__file__)} fit")
    except Abort as e:
        original_mode = None                    # 停在模式 0，finally 里不要再切回原来的模式
        try:
            write_mode(ocd, sym, 0)
            done = "已切到模式 0（无力）"
        except (RuntimeError, ConnectionError, OSError):
            done = "切模式 0 也没写进去，赶紧手动断电或者在 gimbal_watch.py 里 set mode 0"
        sys.exit(f"停下了：{e}。{done}。")
    except KeyboardInterrupt:
        if original_mode is not None:
            try:
                write_yaw(ocd, sym, read_axis(ocd, sym)["target"])     # 目标停在当前位置，电机停住
                time.sleep(0.3)
            except (RuntimeError, ConnectionError, OSError):
                pass
            print("\nCtrl+C：停下了，这一次没记。")
    except (RuntimeError, ConnectionError, OSError) as e:
        log_hint = f"\nOpenOCD 日志：{proc.log_path}" if proc is not None else ""
        sys.exit(f"读写失败：{e}{log_hint}（电机会走到最后写的目标就停，最多 {LEAD_DEG:g}°）")
    finally:
        if original_mode is not None and original_mode != 1:
            try:
                write_mode(ocd, sym, original_mode)
                if finished:
                    print(f"切回了模式 {original_mode}。")
            except (RuntimeError, ConnectionError, OSError):
                pass
        if proc is not None:
            try:
                ocd.command("shutdown")
            except (ConnectionError, OSError):
                pass
            try:
                proc.wait(timeout=3)
            except Exception:                   # noqa: BLE001  退出时尽量收拾，不再报错
                proc.terminate()


# ---------- 拟合 ----------

def solve(rows, columns):
    """最小二乘：rows 是 (自变量 dict, 因变量)，columns 是要拟合的列名。正规方程，最多 3 个未知数。"""
    n = len(columns)
    a = [[sum(x[i] * x[j] for x, _ in rows) for j in columns] for i in columns]
    b = [sum(x[i] * y for x, y in rows) for i in columns]
    for c in range(n):                          # 高斯消元，选主元
        p = max(range(c, n), key=lambda r: abs(a[r][c]))
        if abs(a[p][c]) < 1e-12:
            return None
        a[c], a[p], b[c], b[p] = a[p], a[c], b[p], b[c]
        for r in range(n):
            if r != c:
                k = a[r][c] / a[c][c]
                a[r] = [a[r][j] - k * a[c][j] for j in range(n)]
                b[r] -= k * b[c]
    return {columns[c]: b[c] / a[c][c] for c in range(n)}


def fit(args):
    if not os.path.exists(args.log):
        sys.exit(f"没有 {args.log}：先不带参数跑一次测量。")
    with open(args.log, newline="", encoding="utf-8") as f:
        rows = list(csv.DictReader(f))
    if not rows:
        sys.exit(f"{args.log} 里没有数据。")
    data = []
    for r in rows:
        v, i = float(r["v_dps"]), float(r["i_a"])
        data.append({"v": v, "i": i, "lo": float(r["i_lo_a"]), "ramp": float(r["ramp_dps"])})
    speeds = sorted({round(d["ramp"], 1) for d in data})
    both = any(d["v"] > 0 for d in data) and any(d["v"] < 0 for d in data)
    spread = max(abs(d["v"]) for d in data) / min(abs(d["v"]) for d in data)
    print(f"{args.log}：{len(data)} 行，斜坡 {'、'.join(f'{s:g}' for s in speeds)}°/s，"
          + ("正反两个方向都有" if both else "只有一个方向"))

    columns = ["i_c"]
    notes = []
    if spread >= 1.3:
        columns.append("b")
    else:
        notes.append("只有一个转速，库仑项和粘滞项分不开：粘滞项按 0 算，库仑项就是这个转速下的摩擦。至少再测一个转速")
    if both:
        columns.append("i_0")
    else:
        notes.append("只有一个方向，分不出正反不对称的部分，按对称算")
    pairs = [({"i_c": math.copysign(1.0, d["v"]), "b": d["v"], "i_0": 1.0}, d["i"]) for d in data]
    p = solve(pairs, columns)
    if p is None:
        sys.exit("拟合不出来（数据退化了？）")
    i_c, b, i_0 = p["i_c"], p.get("b", 0.0), p.get("i_0", 0.0)
    resid = [y - sum(p[c] * x[c] for c in columns) for x, y in pairs]
    print(f"拟合：电流 = i_c·sgn(ω) + b·ω + i_0，i_c = {i_c:.4f} A，b = {b:.6f} A/(°/s)，i_0 = {i_0:+.4f} A"
          f"（正反不对称的部分，前馈里不补）；残差 RMS {math.sqrt(statistics.fmean(e * e for e in resid)):.4f} A")
    for text in notes:
        print(f"  注意：{text}")
    for d, e in zip(data, resid):
        print(f"  {d['v']:+8.1f} °/s：{d['i']:+.4f} A（拟合差 {e:+.4f}），分格平均最小 {d['lo']:.4f} A")

    if b < 0.0:
        print(f"  注意：粘滞项是负的（{b:.6f}，转得越快摩擦越小），前馈里按 0 给，库仑项按最慢那个转速的摩擦算")
        slowest = min(data, key=lambda d: abs(d["v"]))
        i_c, b = abs(slowest["i"]), 0.0
    if i_c <= 0.0:
        sys.exit("库仑项不是正的：测得不对（电流和转速反号？），不给建议值。")
    # 补的不能超过任何一个位置、任何一个方向的实际摩擦：比例取 FF_RATIO 和 各行(最小那格 / 前馈模型) 里小的那个
    safe = min((d["lo"] / (i_c + b * abs(d["v"])) for d in data if math.isfinite(d["lo"])), default=FF_RATIO)
    ratio = min(FF_RATIO, safe)
    print(f"\n建议（按 {ratio:.0%} 给：最多 {FF_RATIO:.0%}，并且每个位置、每个方向都不超过实测；"
          f"按位置最小那格算出来最多能给 {safe:.0%}）：")
    print(f"    #define GIMBAL_YAW_FRICTION_FF         {ratio * i_c:.4f}f    /* 库仑项，A */")
    print(f"    #define GIMBAL_YAW_VISCOUS_FF          {ratio * b:.6f}f  /* 粘滞项，A/(°/s) */")
    if abs(i_0) > 0.2 * i_c:
        print(f"  注意：正反不对称（i_0 = {i_0:+.4f} A）超过库仑项的 20%：正转、反转摩擦差得多，看看是不是有线在拽。")


def main():
    parser = argparse.ArgumentParser(description="Gimbal_test_RS05：匀速转测 yaw 摩擦；fit 拟合出前馈参数")
    parser.add_argument("action", nargs="?", choices=("measure", "fit"), default="measure",
                        help="measure（默认）：转几趟测电流；fit：拟合 csv 里的结果")
    parser.add_argument("--log", default=DEFAULT_LOG, help="结果文件（默认 tools/yaw_friction.csv）")
    parser.add_argument("--passes", type=int, default=2, help="转几趟，正反交替（默认 2）")
    parser.add_argument("--cover", type=float, default=360.0, help="每趟匀速段转多少度（默认 360，一整圈）")
    parser.add_argument("--max-turn", type=float, default=900.0, help="每趟最多转多少度（默认 900）")
    parser.add_argument("--elf", default=gw.DEFAULT_ELF, help="烧进板子的那个 ELF（默认 build/Debug/Gimbal_test_RS05.elf）")
    parser.add_argument("--openocd", default=gw.DEFAULT_OPENOCD, help="没有在跑的 OpenOCD 时用来起一个")
    parser.add_argument("--cfg", default=gw.DEFAULT_CFG, help="OpenOCD 的板子配置")
    parser.add_argument("--port", type=int, default=6666, help="OpenOCD 的 TCL 端口")
    sys.stdout.reconfigure(encoding="utf-8", errors="replace")
    sys.stderr.reconfigure(encoding="utf-8", errors="replace")
    args = parser.parse_args()
    if args.passes < 1 or args.cover <= 0.0 or args.max_turn <= 0.0:
        parser.error("--passes 至少 1，--cover、--max-turn 要是正数")
    if args.action == "fit":
        fit(args)
    else:
        measure(args)


if __name__ == "__main__":
    main()
