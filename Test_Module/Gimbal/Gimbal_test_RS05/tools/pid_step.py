#!/usr/bin/env python3
"""云台阶跃测试（调 PID 用）：经 OpenOCD 高速读 gimbal.c 里的变量，给 yaw 或 pitch 来回打阶跃，
算斜坡落后、上升、超调、卡住、稳定时间、稳态误差和电流，每次存一份 csv。

用法（程序在跑、云台已经在模式 1 或 2；gimbal_watch.py 开着也行，会连上它的 OpenOCD）。路径写全：
    python C:/Project/RM_Infantry/Test_Module/Gimbal/Gimbal_test_RS05/tools/pid_step.py yaw 10
        yaw 目标先 +10°、再 −10° 回来，每段看 2.5 s
    pid_step.py yaw 10 --speed-kp 0.02 --speed-ki 0.1 --angle-kp 4
        先改这个轴的增益再测（运行中改，板子复位后回到 gimbal_config.h 的值）
    pid_step.py pitch 5 --hold 3 --repeat 2 --trace
- 不切模式，用现在的实际模式：模式 1 看编码器角和电机转速，模式 2 看 IMU（和 gimbal.c 里用的反馈一样）。模式 0 或锁住时不动。
- 保护：斜坡走完 100 ms 以后，这个轴最近 300 ms 里有 60% 以上的时间电流顶在限幅上，或者 800 ms 里转速换向 5 次以上
  （每次超过 20°/s），算振荡：这次改过增益就先改回去，300 ms 后还在就切模式 0；没改增益就直接切模式 0。
  另一个轴最近 300 ms 有 60% 以上顶限幅也切模式 0。改完增益先停着看 0.5 s，也按这个查。
- “卡住”：斜坡走完 50 ms 以后，离最终目标还差 0.15° 以上，但 60 ms 内角度变化不到 0.03°（比 0.5°/s 还慢）。
- csv 在 %TEMP%/pid_step/ 下，一行一个采样（约每 3~5 ms 一个，看 DAPLink）。
- 连接、结构体布局、改增益（改 Ki 时先处理积分）都用 gimbal_watch.py 里的，改了布局两边一起改。
"""

import argparse
import csv
import os
import re
import shutil
import struct
import subprocess
import sys
import tempfile
import time

sys.path.insert(0, os.path.dirname(os.path.abspath(__file__)))
import gimbal_watch as gw  # noqa: E402

CMD_FORMAT = "<Bxxxff"          # GimbalCmd_t：mode, yaw_deg, pitch_deg（gimbal.h）
# gimbal.c 里的静态变量，地址是连着的（nm -n 看过），一次读一整块
BLOCK = {"gimbal_yaw": struct.calcsize(gw.AXIS_FORMAT), "gimbal_pitch": struct.calcsize(gw.AXIS_FORMAT),
         "gimbal_cmd": struct.calcsize(CMD_FORMAT), "gimbal_imu": struct.calcsize(gw.INS_FORMAT),
         "gimbal_mode": 1, "gimbal_fault": 1}
GAINS = {"yaw": {"speed_kp": "speed_kp", "speed_ki": "speed_ki", "angle_kp": "angle_kp"},
         "pitch": {"speed_kp": "pitch_speed_kp", "speed_ki": "pitch_speed_ki", "angle_kp": "pitch_angle_kp"}}

STUCK_ERR_DEG = 0.15
STUCK_MOVE_DEG = 0.03
STUCK_WINDOW_MS = 60
SETTLE_TOL_DEG = 0.2
SAT_RATIO = 0.97                # |电流| ≥ 限幅 × 这个数算顶在限幅上
TRACE_MS = (0, 20, 40, 60, 80, 100, 130, 160, 200, 250, 300, 400, 500, 600, 800, 1000, 1300, 1600, 2000, 2500,
            3000, 4000)


def wrap180(deg):
    return (deg + 180.0) % 360.0 - 180.0


def find_block(elf):
    nm = shutil.which("arm-none-eabi-nm")
    out = subprocess.run([nm, "-S", elf], capture_output=True, text=True, check=True).stdout
    found = {}
    for line in out.splitlines():
        parts = line.split()
        if len(parts) == 4 and parts[3] in BLOCK:
            found[parts[3]] = (int(parts[0], 16), int(parts[1], 16))
    missing = [name for name in BLOCK if name not in found]
    wrong = [f"{name} {found[name][1]} 字节（按 {size} 读）" for name, size in BLOCK.items()
             if name in found and found[name][1] != size]
    if missing or wrong:
        sys.exit("gimbal.c 的变量对不上：" + "、".join(missing + wrong))
    start = min(a for a, _ in found.values())
    end = max(a + n for a, n in found.values())
    if end - start > 1024:
        sys.exit(f"gimbal.c 的变量不挨着了（跨 {end - start} 字节），脚本按一块读，要改")
    return found, start, end


def read_config_float(name):
    path = os.path.join(gw.PROJECT_DIR, "App", "gimbal", "gimbal_config.h")
    with open(path, encoding="utf-8", errors="replace") as f:
        match = re.search(rf"^#define[ \t]+{name}[ \t]+\(?([-+]?[0-9.]+)f?\)?", f.read(), re.M)
    return float(match.group(1)) if match else None


class Sampler:
    """经 DAPLink 读的时间和字数成正比（2026-11-12 实测，gimbal_watch 同时开着：1 个字 2.0 ms、9 个 3.0、44 个 7.8、
    98 个 15.1）。所以每个采样只读被测的那个轴（44 个字）；模式、指令、IMU、另一个轴的电流每 4 个采样读一次。
    反馈直接用 PID 结构体里的 actual：就是 gimbal.c 当时喂给 PID 的（模式 2 是 IMU，模式 1 是编码器）。
    时间用电机反馈的 tick（RS05 每毫秒回一帧），不另读 uwTick。"""

    SLOW_EVERY = 4

    def __init__(self, ocd, block, start, end, axis):
        self.ocd, self.block, self.start, self.end, self.axis = ocd, block, start, end, axis
        self.other = "pitch" if axis == "yaw" else "yaw"
        self.count = 0
        self.slow = None

    def part(self, raw, base, name):
        address, size = self.block[name]
        return raw[address - base:address - base + size]

    def read_slow(self):
        base = self.block["gimbal_cmd"][0]
        raw = self.ocd.read_bytes(base, self.end - base)
        cmd = struct.unpack(CMD_FORMAT, self.part(raw, base, "gimbal_cmd"))
        imu = struct.unpack(gw.INS_FORMAT, self.part(raw, base, "gimbal_imu"))
        # 另一个轴只读电机里的 current_a（RS05_Motor_t 偏移 4），乘方向（GimbalAxis_t 偏移 28）就是按云台正方向的电流
        other_address = self.block[f"gimbal_{self.other}"][0]
        words = self.ocd.read(other_address, 32, 8)
        current = struct.unpack("<f", struct.pack("<I", words[1]))[0]
        direction = struct.unpack("<b", struct.pack("<I", words[7])[:1])[0]
        self.slow = {"cmd": {"yaw": cmd[1], "pitch": cmd[2]},
                     "imu": {"yaw": (imu[1], imu[3]), "pitch": (imu[2], imu[4])},
                     "mode": self.part(raw, base, "gimbal_mode")[0], "fault": self.part(raw, base, "gimbal_fault")[0],
                     "other_current": direction * current}

    def read(self):
        if self.slow is None or self.count % self.SLOW_EVERY == 0:
            self.read_slow()
        self.count += 1
        a = gw.unpack_axis(self.ocd.read_bytes(*self.block[f"gimbal_{self.axis}"]))
        s = dict(self.slow)
        s.update({"tick": a["fb_tick"], self.axis: a, self.other: {"current": self.slow["other_current"]}})
        return s


def feedback(s, axis):
    """gimbal.c 当时喂给两个环的反馈：模式 2 是 IMU 的角和角速度，模式 1 是编码器角和电机转速。"""
    return s[axis]["angle_pid"]["actual"], s[axis]["speed_pid"]["actual"]


class Guard:
    def __init__(self, axis, limit):
        self.axis, self.limit = axis, limit
        self.history = []           # (tick, 这个轴顶限幅, 另一个轴顶限幅)
        self.flips = []
        self.last_sign = 0

    def check(self, s, armed):
        other = "pitch" if self.axis == "yaw" else "yaw"
        tick = s["tick"]
        # 本轴只统计保护开启（armed）以后的采样：阶跃本身加速、刹车会顶限幅一百多毫秒，
        # 2026-11-12 一开始连开启前的也算进去，10° 阶跃一开启就误报了一次
        self.history.append((tick, armed and abs(s[self.axis]["current"]) >= SAT_RATIO * self.limit,
                             abs(s[other]["current"]) >= SAT_RATIO * self.limit, armed))
        self.history = [h for h in self.history if tick - h[0] <= 300]
        _, rate = feedback(s, self.axis)
        sign = 1 if rate > 20.0 else (-1 if rate < -20.0 else 0)
        if sign != 0 and self.last_sign != 0 and sign != self.last_sign:
            self.flips.append(tick)
        if sign != 0:
            self.last_sign = sign
        self.flips = [t for t in self.flips if tick - t <= 800]
        full = len(self.history) >= 20 and tick - self.history[0][0] >= 250
        if full and sum(h[2] for h in self.history) >= 0.6 * len(self.history):
            return f"{other} 最近 300 ms 有 60% 以上电流顶在限幅上"
        if not armed:
            return None
        own = [h for h in self.history if h[3]]
        if len(own) >= 20 and tick - own[0][0] >= 250 and sum(h[1] for h in own) >= 0.6 * len(own):
            return f"{self.axis} 保护开启后的 300 ms 里有 60% 以上电流顶在限幅上"
        if len(self.flips) >= 5:
            return f"{self.axis} 800 ms 里转速换向 {len(self.flips)} 次（每次超过 20°/s）"
        return None


class Trip(Exception):
    pass


class Bench:
    """连着板子的东西放一起：读、写指令、改增益、出事时的处理。"""

    def __init__(self, args):
        self.args = args
        self.ocd, self.proc = gw.connect(args)
        self.symbols = gw.find_symbols(args.elf)
        block, start, end = find_block(args.elf)
        self.sampler = Sampler(self.ocd, block, start, end, args.axis)
        self.const = gw.read_constants()
        self.changed = {}           # 这次改过的增益：名字 → 改之前的值

    def state(self):
        return gw.read_state(self.ocd, self.symbols)

    def write_test_f32(self, offset, value):
        self.ocd.write(self.symbols["gimbal_test"][0] + offset, 32, [struct.unpack("<I", struct.pack("<f", value))[0]])

    def relax(self):
        self.ocd.write(self.symbols["gimbal_test"][0] + gw.TEST_MODE_OFFSET, 8, [0])

    def set_gain(self, name, value):
        gw.write_value(self.ocd, self.symbols, self.state(), self.const, name, False, value, full=False)

    def apply_gains(self, axis, wanted):
        s = self.state()
        a = s if axis == "yaw" else s["pitch"]
        now = {"speed_kp": a["speed_pid"]["Kp"], "speed_ki": a["speed_pid"]["Ki"], "angle_kp": a["angle_pid"]["Kp"]}
        for key, value in wanted.items():
            if value is None or abs(value - now[key]) < 1e-9:
                continue
            self.changed[GAINS[axis][key]] = now[key]
            self.set_gain(GAINS[axis][key], value)

    def revert_gains(self):
        for name, value in self.changed.items():
            self.set_gain(name, value)
        self.changed = {}

    def record(self, guard, duration_s, armed_after_ms, mark):
        """采 duration_s 秒；armed_after_ms 毫秒以后保护才查本轴的振荡和顶限幅。出事抛 Trip。"""
        samples = []
        t0 = None
        end = time.perf_counter() + duration_s
        while time.perf_counter() < end:
            s = self.sampler.read()
            s["mark"] = mark
            samples.append(s)
            if t0 is None:
                t0 = s["tick"]
            if s["mode"] == 0:
                raise Trip(("板子自己进了模式 0（电机掉线、IMU 失效，或者有人切了）", samples))
            problem = guard.check(s, s["tick"] - t0 >= armed_after_ms)
            if problem is None:
                continue
            if not self.changed:
                self.relax()
                raise Trip((f"{problem}；这次没改增益，已切模式 0", samples))
            print(f"\n!! 保护：{problem}，先把增益改回去")
            self.revert_gains()
            self.watch_after_revert(guard, mark, samples)
            raise Trip((f"{problem}；增益已改回，稳住了", samples))
        return samples

    def watch_after_revert(self, guard, mark, samples):
        """改回增益以后看 600 ms：最后 300 ms 里还快过 20°/s，或者 30% 以上的时间顶限幅，就切模式 0。"""
        after = []
        end = time.perf_counter() + 0.6
        while time.perf_counter() < end:
            s = self.sampler.read()
            s["mark"] = mark
            after.append(s)
        samples += after
        tail = [x for x in after if x["tick"] >= after[-1]["tick"] - 300]
        fastest = max(abs(feedback(x, guard.axis)[1]) for x in tail)
        saturated = sum(abs(x[guard.axis]["current"]) >= SAT_RATIO * guard.limit for x in tail) / len(tail)
        if fastest > 20.0 or saturated > 0.3 or after[-1]["mode"] == 0:
            self.relax()
            raise Trip((f"改回增益后最后 300 ms 还在动（最快 {fastest:.0f}°/s、顶限幅 {saturated:.0%}），已切模式 0",
                        samples))

    def close(self):
        if self.proc is not None:
            try:
                self.ocd.command("shutdown")
            except (ConnectionError, OSError):
                pass
            self.proc.wait(timeout=5)


def analyze(samples, axis, new_cmd, limit, ramp):
    """一段阶跃：从斜坡后的目标开始动的那个采样算起，0 点按“已经走了多少 / 斜坡速度”往前推到指令变的那一刻。"""
    first = samples[0][axis]["target"]
    i0 = next((i for i, s in enumerate(samples) if abs(wrap180(s[axis]["target"] - first)) > 1e-3), None)
    if i0 is None:
        return None
    rows = []
    t0 = samples[i0]["tick"] - round(abs(wrap180(samples[i0][axis]["target"] - first)) / ramp)
    ref = samples[-1][axis]["target"]           # 斜坡最后停在的目标（被限位截了就是限位）
    fb_start, _ = feedback(samples[i0], axis)
    sign = 1.0 if wrap180(ref - fb_start) >= 0 else -1.0
    for s in samples[i0:]:
        fb, rate = feedback(s, axis)
        a = s[axis]
        rows.append({"t": s["tick"] - t0, "fb": fb, "rate": rate, "e": sign * wrap180(ref - fb),
                     "lag": sign * wrap180(a["target"] - fb), "target": a["target"], "I": a["current"],
                     "Ii": a["speed_pid"]["Ki"] * a["speed_pid"]["integral"], "spd_tgt": a["speed_pid"]["target"]})
    r = {"rows": rows, "ref": ref, "clamped": abs(wrap180(ref - new_cmd)) > 0.05, "size": abs(wrap180(ref - fb_start)),
         "count": len(rows), "dt": (rows[-1]["t"] / max(1, len(rows) - 1))}
    r["t_ramp"] = next((x["t"] for x in rows if abs(wrap180(x["target"] - ref)) < 0.01), None)
    ramp_rows = [x for x in rows if r["t_ramp"] is None or x["t"] <= r["t_ramp"]]
    r["lag_max"] = max(x["lag"] for x in ramp_rows)
    r["t90"] = next((x["t"] for x in rows if x["e"] <= 0.1 * r["size"]), None)
    r["overshoot"] = max(0.0, -min(x["e"] for x in rows))
    out = [x["t"] for x in rows if abs(x["e"]) > SETTLE_TOL_DEG]
    r["settle"] = 0 if not out else (None if out[-1] == rows[-1]["t"] else
                                     next(x["t"] for x in rows if x["t"] > out[-1]))
    tail = [x for x in rows if x["t"] >= rows[-1]["t"] - 300]
    r["e_end"] = sum(x["e"] for x in tail) / len(tail)
    r["e_end_span"] = (min(x["e"] for x in tail), max(x["e"] for x in tail))
    r["I_end"] = sum(x["I"] for x in tail) / len(tail)
    rest = [x["I"] for x in rows if x["t"] >= rows[-1]["t"] - 500]
    mean = sum(rest) / len(rest)
    r["I_noise"] = (sum((v - mean) ** 2 for v in rest) / len(rest)) ** 0.5
    r["I_peak"] = max(abs(x["I"]) for x in rows)
    r["sat_ms"] = sum(rows[k]["t"] - rows[k - 1]["t"] for k in range(1, len(rows))
                      if abs(rows[k]["I"]) >= SAT_RATIO * limit)
    # 卡住：斜坡走完 50 ms 以后，差 0.15° 以上、60 ms 内几乎没动
    stuck = []
    j = 0
    for k, x in enumerate(rows):
        while rows[j]["t"] < x["t"] - STUCK_WINDOW_MS:
            j += 1
        after_ramp = r["t_ramp"] is not None and x["t"] >= r["t_ramp"] + 50
        moved = abs(wrap180(x["fb"] - rows[j]["fb"]))
        stuck.append(after_ramp and x["t"] - rows[j]["t"] >= STUCK_WINDOW_MS - 10 and
                     abs(x["e"]) > STUCK_ERR_DEG and moved < STUCK_MOVE_DEG)
    spans = []
    k = 0
    while k < len(rows):
        if stuck[k]:
            m = k
            while m + 1 < len(rows) and stuck[m + 1]:
                m += 1
            first = window_start(rows, k)
            # 和上一段重叠或者只隔不到 30 ms（中间蹭了一下）就并成一段
            if spans and rows[first]["t"] <= spans[-1][1] + 30:
                first = rows.index(spans.pop()[2])
            peak = max(rows[first:m + 1], key=lambda q: abs(q["I"]))
            spans.append((rows[first]["t"], rows[m]["t"], rows[first], rows[m], peak))
            k = m + 1
        else:
            k += 1
    r["stuck"] = spans
    return r


def window_start(rows, k):
    """卡住的那一段从 60 ms 窗口的开头算（窗口里一直没动）。"""
    t = rows[k]["t"] - STUCK_WINDOW_MS
    return next(q for q in range(k + 1) if rows[q]["t"] >= t)


def fmt_ms(value):
    return "—" if value is None else f"{value:.0f} ms"


def print_result(r, label, mode, hold_ms, trace):
    if r is None:
        print(f"[{label}] 没找到指令变化的那个采样")
        return
    print(f"[{label}] 模式 {mode}，{r['count']} 个采样，平均 {r['dt']:.1f} ms 一个"
          + (f"；目标被限位截在 {r['ref']:+.2f}°" if r["clamped"] else ""))
    print(f"  斜坡 {fmt_ms(r['t_ramp'])} 走完，斜坡里最多落后 {r['lag_max']:.2f}°")
    settle = fmt_ms(r["settle"]) if r["settle"] is not None else f"{hold_ms:.0f} ms 内没稳定"
    print(f"  到 90%：{fmt_ms(r['t90'])}；超调 {r['overshoot']:.2f}°；稳定到 ±{SETTLE_TOL_DEG}°：{settle}")
    print(f"  最后 0.3 s：误差 {r['e_end']:+.3f}°（{r['e_end_span'][0]:+.3f} ~ {r['e_end_span'][1]:+.3f}），"
          f"电流 {r['I_end']:+.3f} A；最后 0.5 s 电流抖动 ±{r['I_noise']:.3f} A（标准差）")
    print(f"  电流峰值 {r['I_peak']:.2f} A，顶限幅共 {r['sat_ms']:.0f} ms")
    if r["stuck"]:
        total = sum(span[1] - span[0] for span in r["stuck"])
        print(f"  卡住 {len(r['stuck'])} 次，共 {total:.0f} ms：")
        for a, b, first, last, peak in r["stuck"]:
            print(f"    {a:5.0f} ~ {b:5.0f} ms，差 {first['e']:+.2f}° → {last['e']:+.2f}°，"
                  f"电流 {first['I']:+.3f} → {last['I']:+.3f} A、中间最大 {peak['I']:+.3f}"
                  f"（积分那份 {first['Ii']:+.3f} → {last['Ii']:+.3f}）")
    else:
        print("  没卡住")
    if trace:
        print(f"  {'t ms':>6} {'目标':>8} {'误差':>7} {'转速':>7} {'速度目标':>8} {'电流':>7} {'积分份':>7}")
        rows = r["rows"]
        for t in TRACE_MS:
            x = min(rows, key=lambda q: abs(q["t"] - t))
            if abs(x["t"] - t) > 15 and t > 0:
                continue
            print(f"  {x['t']:6.0f} {x['target']:+8.2f} {x['e']:+7.2f} {x['rate']:+7.1f} {x['spd_tgt']:+8.1f} "
                  f"{x['I']:+7.3f} {x['Ii']:+7.3f}")


def save_csv(path, axis, steps):
    with open(path, "w", newline="", encoding="utf-8") as f:
        w = csv.writer(f)
        w.writerow(["step", "tick", "mode", "cmd", "target", "fb", "fb_rate", "enc_deg", "enc_rate", "speed_target",
                    "speed_out", "I_int", "current", "other_current", "imu_yaw", "imu_pitch"])
        other = "pitch" if axis == "yaw" else "yaw"
        for s in steps:
            a = s[axis]
            fb, rate = feedback(s, axis)
            # mode、cmd、imu、另一个轴的电流是每 4 个采样读一次的，中间几行是最近一次的值
            w.writerow([s["mark"], s["tick"], s["mode"], f"{s['cmd'][axis]:.3f}", f"{a['target']:.3f}", f"{fb:.4f}",
                        f"{rate:.2f}", f"{a['angle']:.3f}", f"{a['rate']:.2f}", f"{a['speed_pid']['target']:.2f}",
                        f"{a['speed_pid']['out']:.4f}", f"{a['speed_pid']['Ki'] * a['speed_pid']['integral']:.4f}",
                        f"{a['current']:.4f}", f"{s[other]['current']:.4f}", f"{s['imu']['yaw'][0]:.4f}",
                        f"{s['imu']['pitch'][0]:.4f}"])


def main():
    parser = argparse.ArgumentParser(description="云台阶跃测试（调 PID 用）")
    parser.add_argument("axis", choices=("yaw", "pitch"))
    parser.add_argument("step", type=float, help="阶跃幅度（度）：先 +step 再 −step 回来")
    parser.add_argument("--hold", type=float, default=2.5, help="每段看几秒")
    parser.add_argument("--repeat", type=int, default=1, help="来回几趟")
    parser.add_argument("--speed-kp", type=float)
    parser.add_argument("--speed-ki", type=float)
    parser.add_argument("--angle-kp", type=float)
    parser.add_argument("--trace", action="store_true", help="每段打一张按时间的表")
    parser.add_argument("--elf", default=gw.DEFAULT_ELF)
    parser.add_argument("--openocd", default=gw.DEFAULT_OPENOCD)
    parser.add_argument("--cfg", default=gw.DEFAULT_CFG)
    parser.add_argument("--port", type=int, default=6666)
    args = parser.parse_args()
    sys.stdout.reconfigure(encoding="utf-8", errors="replace")

    bench = Bench(args)
    try:
        run(bench, args)
    finally:
        bench.close()


def run(bench, args):
    s = bench.state()
    problems = gw.layout_problems(s)
    if problems:
        sys.exit("ELF 和板子里的程序对不上（是不是重新编译后没烧？）：\n  " + "\n  ".join(problems))
    test = struct.unpack(gw.TEST_FORMAT, bench.ocd.read_bytes(*bench.symbols["gimbal_test"]))
    if s["mode"] not in (1, 2) or s["lock"] != 0 or test[0] != s["mode"]:
        sys.exit(f"云台要先在模式 1 或 2、没锁住（现在指令 {test[0]}、实际 {s['mode']}、锁 {s['lock']}）")
    if test[6] != 0.0 or test[7] != 0.0:
        sys.exit("gimbal_test 的方波开着（yaw_square_deg / pitch_square_deg 不是 0），先关掉")
    axis = args.axis
    a = s if axis == "yaw" else s["pitch"]
    limit = a["speed_pid"]["outMax"]
    ramp = read_config_float("GIMBAL_RAMP_DEG_PER_MS") or 0.1
    offset = gw.TEST_YAW_OFFSET if axis == "yaw" else gw.TEST_PITCH_OFFSET
    start_cmd = test[1] if axis == "yaw" else test[2]
    mode = s["mode"]

    bench.apply_gains(axis, {"speed_kp": args.speed_kp, "speed_ki": args.speed_ki, "angle_kp": args.angle_kp})
    s = bench.state()
    a = s if axis == "yaw" else s["pitch"]
    print(f"{axis} 现在的增益：速度环 Kp {a['speed_pid']['Kp']:g} Ki {a['speed_pid']['Ki']:g}，"
          f"角度环 Kp {a['angle_pid']['Kp']:g}；电流限幅 {limit:g} A；模式 {mode}")
    if bench.changed:
        print("  这次改之前：" + "，".join(f"{k} {v:g}" for k, v in bench.changed.items()))

    guard = Guard(axis, limit)
    samples = []
    results = []
    plan = []
    for _ in range(args.repeat):
        plan += [start_cmd + args.step, start_cmd]
    armed_after = abs(args.step) / ramp + 100.0
    reason = None
    try:
        if bench.changed:
            samples += bench.record(guard, 0.5, 0.0, "settle")
        for k, cmd in enumerate(plan):
            bench.write_test_f32(offset, cmd)
            part = bench.record(guard, args.hold, armed_after, k)
            samples += part
            results.append((k, cmd, part))
    except Trip as trip:
        reason, part = trip.args[0]
        samples += part
    except KeyboardInterrupt:
        reason = "Ctrl+C"

    os.makedirs(os.path.join(tempfile.gettempdir(), "pid_step"), exist_ok=True)
    name = time.strftime("%H%M%S") + f"_{axis}_{args.step:g}.csv"
    path = os.path.join(tempfile.gettempdir(), "pid_step", name)
    save_csv(path, axis, samples)

    prev = start_cmd
    for k, cmd, part in results:
        print()
        r = analyze(part, axis, cmd, limit, ramp)
        print_result(r, f"{cmd - prev:+.2f}°", mode, args.hold * 1000.0, args.trace)
        prev = cmd
    print()
    if reason:
        print(f"!! 没测完：{reason}")
    s = bench.state()
    a = s if axis == "yaw" else s["pitch"]
    print(f"结束时 {axis} 的增益：速度环 Kp {a['speed_pid']['Kp']:g} Ki {a['speed_pid']['Ki']:g}，"
          f"角度环 Kp {a['angle_pid']['Kp']:g}；实际模式 {s['mode']}")
    print(f"csv：{path}")


if __name__ == "__main__":
    main()
