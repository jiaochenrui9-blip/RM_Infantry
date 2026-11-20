#!/usr/bin/env python3
"""Gimbal_test_RS05 的时序读数脚本：经 OpenOCD（DAPLink）不停核读 robot.c 开头那组时序测量量。

看什么：陀螺仪实际出样本的频率（ODR）、DRDY 周期的抖动、Robot_Loop 每拍拿到几个新样本、
Robot_Loop 开始时最新样本的年龄、INS_Update 和整个 Robot_Loop 的耗时。

用法（程序已经烧进板子在跑；CLion 用 Run 烧录，不开调试会话）。在哪个目录下运行都行，路径写全：
    python C:/Project/RM_Infantry/Test_Module/Gimbal/Gimbal_test_RS05/tools/timing_watch.py
下面简写成 timing_watch.py：
    timing_watch.py                 一直读，每 0.5 s 打一行；Ctrl+C 退出，退出时打总结
    timing_watch.py --seconds 60    读 60 s，打总结后退出
    timing_watch.py --age           只看样本年龄：每 20 ms 打一行带位置的条，看它怎么漂（Ctrl+C 退出）
    timing_watch.py --csv           同时把每次读数存成 csv（%TEMP%/timing_watch/时分秒.csv）

每列的意思：
- T_C：BMI088 的温度（ins 加热 PID 的 actual，℃）。陀螺仪的 ODR 由它自己的振荡器定，可能跟着温度变。
- ODR_Hz：板子每数满 1000 个 DRDY，用 TIM2（1 MHz）记一次这 1000 个周期一共多少 us，ODR = 1e9 / 它，约每秒更新一次。
  dev% 是相对 1000 Hz 的偏差，手册表 5 写的容差是 ±0.3 %。
- period_us：这一行的时间段里，相邻两次 DRDY 的最短 ~ 最长间隔。脚本每打一行就把板子上的最短、最长写回初值，重新统计。
  里面除了陀螺仪自己的抖动，主要是中断延迟：DRDY 进来时 SPI1、DMA 中断（同为优先级 5）正在跑，或者哪里关了中断，
  记下的时刻就晚。
- n0 n1 n2：这段时间里 Robot_Loop 开始时，离上一拍开始来了 0 个、1 个、2 个及以上新样本的拍数。
  n0 = 这一拍读到的还是上一拍那个样本（重复）；n2 = 有样本没被读到就被下一个盖掉了（跳过）。
  它们只在相位转到交界处（age_us 从 0 附近跳回 1000 附近）的那一两行出现，平时都是 0。
  每转一圈净多一次：ODR 慢于 1000 Hz 时 n0 比 n2 多 1，快于时 n2 比 n0 多 1，所以“n0 − n2”每秒约 |ODR − 1000|。
  其余的重复和跳过成对出现（一次重复挨着一次跳过）：交界处 DRDY 和 Robot_Loop 读计数这两个时刻挨得很近，
  两边都有几微秒的晃动，而相位每拍只挪约 |ODR − 1000| 个 us（差 0.32 Hz 就是每拍 0.32 us），先后会来回翻很多次。
- age_us：Robot_Loop 最近一次开始时，最新的样本已经出来多久。两边不同步，它在 0~1000 之间一直漂，
  每 1 / |ODR − 1000| 秒转一圈；转到交界处会在两头来回跳一小会儿，就是上面成对的重复和跳过。
  现在的程序在这一拍读这个样本，云台下一拍才用上，所以云台用的样本还要再老约 1 ms。
- ins_us、loop_us：INS_Update、整个 Robot_Loop 最近一拍用了多少 us（期间进来的中断也算在内）；loop_us 斜杠后面是
  这一行时间段里最长的一拍。CLion 的 Debug 是 -O0，比优化后的程序慢。

- 连接：先连 127.0.0.1:6666（gimbal_watch.py 开着时就连它起的 OpenOCD，两个可以同时开）；没有就自己起一个，
  只连不复位、不停核，退出时 shutdown。日志写到 %TEMP%/timing_watch_openocd.log。连接的代码用同目录 gimbal_watch.py 的。
- DAPLink 同时只能给一个 OpenOCD 用：CLion 的调试会话开着时起不来，先停掉。脚本开着时 CLion 也烧不了，先 Ctrl+C。
- 变量地址用 arm-none-eabi-nm 从 ELF 里查，ELF 要和板子里的程序是同一次编译的。
- 会写板子上的 robot_drdy_period_min_us、robot_drdy_period_max_us、robot_loop_max_us（重新统计），别的都只读。
"""

import argparse
import csv
import os
import shutil
import socket
import statistics
import struct
import subprocess
import sys
import tempfile
import time

HERE = os.path.dirname(os.path.abspath(__file__))
sys.path.insert(0, HERE)
import gimbal_watch as gw  # noqa: E402

TIM2_CNT = 0x40000024           # TIM2 基址 0x40000000 + CNT 偏移 0x24
U32 = 0xFFFFFFFF
PID_ACTUAL_OFFSET = 16          # PID_t 里 actual 前面是 Kp、Ki、Kd、target 四个 float
AGE_BAR_WIDTH = 50

# 要读的变量和大小（字节）。robot_* 在 App/robot.c 开头
SYMBOLS = {
    "robot_drdy_count": 4,
    "robot_drdy_us": 4,
    "robot_drdy_period_min_us": 4,
    "robot_drdy_period_max_us": 4,
    "robot_drdy_1000_us": 4,
    "robot_loop_new_samples": 12,
    "robot_sample_age_us": 4,
    "robot_ins_us": 4,
    "robot_loop_us": 4,
    "robot_loop_max_us": 4,
    "ins_bmi088_temp_pid": struct.calcsize(gw.PID_FORMAT),
    "ins_init_status": 1,
    "uwTick": 4,
}
U32_NAMES = ("robot_drdy_count", "robot_drdy_us", "robot_drdy_period_min_us", "robot_drdy_period_max_us",
             "robot_drdy_1000_us", "robot_sample_age_us", "robot_ins_us", "robot_loop_us", "robot_loop_max_us",
             "uwTick")

HEADER = (f"{'t':>6} {'T_C':>5} {'ODR_Hz':>9} {'dev%':>7} {'period_us':>10} "
          f"{'n0':>3} {'n1':>4} {'n2':>3} {'age_us':>6} {'ins_us':>6} {'loop_us':>9}")
CSV_FIELDS = ("pc_time", "tick", "drdy_count", "drdy_us", "drdy_1000_us", "period_min_us", "period_max_us",
              "n0", "n1", "n2", "age_us", "ins_us", "loop_us", "loop_max_us", "temp_c")


def find_symbols(elf):
    nm = shutil.which("arm-none-eabi-nm")
    if nm is None:
        sys.exit("找不到 arm-none-eabi-nm（要在 PATH 里）")
    if not os.path.exists(elf):
        sys.exit(f"找不到 ELF：{elf}\n先在 CLion 里编译，或者用 --elf 指定（CLion 的输出目录可能是 cmake-build-debug）")
    out = subprocess.run([nm, "-S", elf], capture_output=True, text=True, check=True).stdout
    symbols = {}
    for line in out.splitlines():
        parts = line.split()
        if len(parts) == 4 and parts[3] in SYMBOLS:
            if parts[3] in symbols:
                sys.exit(f"ELF 里有两个 {parts[3]}（两个文件各有一个同名的 static 变量），脚本分不清用哪个")
            symbols[parts[3]] = (int(parts[0], 16), int(parts[1], 16))
    missing = [name for name in SYMBOLS if name not in symbols]
    if missing:
        sys.exit(f"ELF 里没有这些变量：{', '.join(missing)}\n"
                 "（robot_* 是 robot.c 里的时序测量量：是不是还没编译加了测量的程序？是不是 Gimbal_test_RS05 的 ELF？）")
    wrong = [f"{name} {symbols[name][1]} 字节（脚本按 {size} 字节读）"
             for name, size in SYMBOLS.items() if symbols[name][1] != size]
    if wrong:
        sys.exit("这些变量的大小和脚本对不上，robot.c 或 PID_t 改过了，脚本开头的 SYMBOLS 要跟着改：\n  " + "\n  ".join(wrong))
    return symbols


def plan_reads(symbols):
    """地址挨着的变量并成一次读：经 DAPLink 每读一次要 1~2 ms，并起来快，同一次读到的也是同一时刻的值。"""
    groups = []
    for address, size, name in sorted((a, s, n) for n, (a, s) in symbols.items()):
        if groups and address - groups[-1]["end"] <= 64:
            groups[-1]["end"] = max(groups[-1]["end"], address + size)
            groups[-1]["names"].append((name, address, size))
        else:
            groups.append({"start": address, "end": address + size, "names": [(name, address, size)]})
    return groups


def connect(args):
    """先连已经在跑的 OpenOCD；没有就自己起一个（日志和 gimbal_watch.py 的分开），返回 (OpenOcd, 自己起的进程或 None)。"""
    try:
        return gw.OpenOcd(socket.create_connection(("127.0.0.1", args.port), timeout=1.0)), None
    except OSError:
        pass
    exe = args.openocd if os.path.exists(args.openocd) else shutil.which("openocd")
    if exe is None:
        sys.exit(f"找不到 OpenOCD：{args.openocd}（用 --openocd 指定）")
    log_path = os.path.join(tempfile.gettempdir(), "timing_watch_openocd.log")
    for attempt in range(2):
        try:
            return gw.spawn_openocd(args, exe, log_path)
        except gw.StartupError as e:
            # 上一个 OpenOCD 被半路打断时，第一次连会报 command mismatch，再连一次一般就好
            if attempt == 1 or "command mismatch" not in str(e):
                sys.exit(f"{e}\n（DAPLink 没插好、板子没电，或者被 CLion 的调试会话占着？）")


def read_snapshot(ocd, groups):
    raw = {}
    for g in groups:
        start = g["start"] & ~3
        data = ocd.read_bytes(start, ((g["end"] - start) + 3) & ~3)
        for name, address, size in g["names"]:
            raw[name] = data[address - start:address - start + size]
    s = {name: struct.unpack("<I", raw[name])[0] for name in U32_NAMES}
    s["hist"] = struct.unpack("<3I", raw["robot_loop_new_samples"])
    s["temp"] = struct.unpack_from("<f", raw["ins_bmi088_temp_pid"], PID_ACTUAL_OFFSET)[0]
    s["ins_status"] = raw["ins_init_status"][0]
    s["pc_time"] = time.time()
    return s


def reset_extremes(ocd, symbols):
    ocd.write(symbols["robot_drdy_period_min_us"][0], 32, [U32])
    ocd.write(symbols["robot_drdy_period_max_us"][0], 32, [0])
    ocd.write(symbols["robot_loop_max_us"][0], 32, [0])


def diff(a, b):
    return (a - b) & U32


def odr_of(s):
    """板子上最近 1000 个 DRDY 周期算出来的 ODR（Hz）；还没数满 1000 个时是 None。"""
    return 1e9 / s["robot_drdy_1000_us"] if s["robot_drdy_1000_us"] else None


def check_running(ocd, symbols, groups):
    """开头确认程序在跑、TIM2 是 1 MHz、DRDY 在来；不对就说该查什么并退出。"""
    tick_address = symbols["uwTick"][0]
    s0 = read_snapshot(ocd, groups)
    tim0, tick0 = ocd.read(TIM2_CNT, 32, 1)[0], ocd.read(tick_address, 32, 1)[0]
    time.sleep(0.3)
    tim1, tick1 = ocd.read(TIM2_CNT, 32, 1)[0], ocd.read(tick_address, 32, 1)[0]
    s1 = read_snapshot(ocd, groups)
    if s1["uwTick"] == s0["uwTick"]:
        sys.exit("uwTick 不涨，程序没在跑：可能卡在 Error_Handler / HardFault，也可能被调试器停住了。")
    if tim1 == tim0:
        sys.exit("TIM2 的计数不动：Robot_Init 里的 HAL_TIM_Base_Start(&htim2) 没跑到？（板子里是加了测量的那次编译吗）")
    rate = diff(tim1, tim0) / max(1, diff(tick1, tick0))
    count = diff(s1["robot_drdy_count"], s0["robot_drdy_count"])
    if count == 0:
        if s1["ins_status"] != 0:
            why = f"BMI088 初始化失败：{gw.BMI088_NAMES.get(s1['ins_status'], s1['ins_status'])}（ins_init_status）"
            if s1["ins_status"] == 5:
                why += "。新加的三个中断寄存器也在回读里，可能是它们没写进去"
        else:
            why = ("BMI088 初始化是好的，但 PC5 上没有中断：板子里是 .ioc 改完、重新生成那次编译的吗？"
                   "gpio.c 里 PC5 应该是 GPIO_MODE_IT_RISING，stm32f4xx_it.c 里要有 EXTI9_5_IRQHandler")
        sys.exit(f"0.3 s 里一个 DRDY 都没来。{why}")
    print(f"程序在跑；TIM2 每毫秒走 {rate:.0f} 个数（应为 1000，读两次之间有 1~2 ms 的误差）；"
          f"0.3 s 里来了 {count} 个 DRDY。", flush=True)
    return s1


def print_line(t, s, prev):
    hist = [diff(s["hist"][i], prev["hist"][i]) for i in range(3)]
    odr = odr_of(s)
    odr_text = f"{odr:9.3f} {(odr / 1000.0 - 1.0) * 100.0:+7.3f}" if odr else f"{'--':>9} {'--':>7}"
    pmin, pmax = s["robot_drdy_period_min_us"], s["robot_drdy_period_max_us"]
    period = f"{pmin}~{pmax}" if pmin != U32 else "--"
    print(f"{t:6.1f} {s['temp']:5.1f} {odr_text} {period:>10} {hist[0]:>3} {hist[1]:>4} {hist[2]:>3} "
          f"{s['robot_sample_age_us']:>6} {s['robot_ins_us']:>6} "
          f"{s['robot_loop_us']:>4}/{s['robot_loop_max_us']:<4}", flush=True)


def csv_row(s):
    return [f"{s['pc_time']:.3f}", s["uwTick"], s["robot_drdy_count"], s["robot_drdy_us"], s["robot_drdy_1000_us"],
            s["robot_drdy_period_min_us"], s["robot_drdy_period_max_us"], *s["hist"], s["robot_sample_age_us"],
            s["robot_ins_us"], s["robot_loop_us"], s["robot_loop_max_us"], f"{s['temp']:.3f}"]


def print_summary(first, last, seen):
    ms = diff(last["uwTick"], first["uwTick"])
    if ms < 1000:
        print("不到 1 s，不打总结。")
        return
    seconds = ms / 1000.0
    count = diff(last["robot_drdy_count"], first["robot_drdy_count"])
    hist = [diff(last["hist"][i], first["hist"][i]) for i in range(3)]
    loops = sum(hist)
    odr = odr_of(last)
    odrs = seen["odr"]
    print()
    print(f"—— 总结：{seconds:.1f} s（按板子的 uwTick 算）——")
    if odr:
        side = "快" if odr > 1000.0 else "慢"
        print(f"陀螺仪实际 ODR：{odr:.3f} Hz（最近 1000 个周期），比 1000 Hz {side} {abs(odr / 10.0 - 100.0):.3f} %"
              "（手册表 5 的容差 ±0.3 %）。")
        if len(odrs) > 1:
            temps = seen["temp"]
            print(f"  这段时间读到的 ODR 在 {min(odrs):.3f} ~ {max(odrs):.3f} Hz 之间，"
                  f"温度 {min(temps):.1f} ~ {max(temps):.1f} ℃。")
    if count:
        # 个数和 uwTick 隔 1~2 ms 读，个数可能差 ±2；另外上面是最近 1000 个周期，这里是整段的平均
        tolerance = 2.0 / count + 50e-6
        cumulative = count / seconds
        print(f"  按 DRDY 个数 / uwTick 算：{count} 个 / {seconds:.3f} s = {cumulative:.2f} Hz"
              f"（个数可能差 ±2，即 ±{2e6 / count:.0f} ppm）。", end="")
        if odr and abs(cumulative / odr - 1.0) <= tolerance:
            print("和上面一致，TIM2 的 1 MHz 也是对的。")
        elif odr:
            print("和上面差得比误差还多：查 .ioc 里 TIM2 的预分频（应为 83）。")
        else:
            print()
    if seen["pmin"] <= seen["pmax"]:
        print(f"DRDY 周期：最短 {seen['pmin']} us，最长 {seen['pmax']} us（含中断延迟的抖动）。")
    # 相位每转一圈净多一次重复（陀螺仪慢）或跳过（快），其余的重复和跳过是交界处先后来回翻出来的，成对
    net = hist[0] - hist[2]
    pairs = min(hist[0], hist[2])
    print(f"Robot_Loop：{loops} 拍，没有新样本（重复）{hist[0]} 次，来了 2 个及以上（跳过）{hist[2]} 次。")
    print(f"  两者之差 {abs(net)}：相位每转一圈，净多一次{'重复' if net >= 0 else '跳过'}", end="")
    if odr and abs(odr - 1000.0) > 0.01:
        print(f"。按最近的 ODR，每 {1.0 / abs(odr - 1000.0):.2f} s 转一圈，这段时间约 "
              f"{seconds * abs(odr - 1000.0):.1f} 圈（ODR {'慢' if odr < 1000.0 else '快'}于 1000 Hz，应该净多"
              f"{'重复' if odr < 1000.0 else '跳过'}）。")
    else:
        print("。")
    if net and pairs:
        print(f"  其余 {pairs} 次重复和 {pairs} 次跳过是成对的，只出现在转到交界处时：DRDY 和 Robot_Loop 读计数"
              f"的先后在那里来回翻。平均每圈 {pairs / abs(net):.1f} 对。")
    print(f"  uwTick 走了 {ms}，Robot_Loop 跑了 {loops} 拍，差 {ms - loops}"
          "（两个数隔 1~2 ms 读的，±2 以内算没漏拍）。")
    ages = seen["age"]
    if ages:
        print(f"样本年龄（Robot_Loop 开始时）：读到 {min(ages)} ~ {max(ages)} us，中位数 {statistics.median(ages):.0f} us"
              f"（{len(ages)} 次）。")
    loops_us, ins_us = seen["loop"], seen["ins"]
    if loops_us:
        print(f"耗时：Robot_Loop 中位数 {statistics.median(loops_us):.0f} us、最长 {seen['loop_max']} us；"
              f"INS_Update 中位数 {statistics.median(ins_us):.0f} us、读到的最长 {max(ins_us)} us（都含中断）。")


def watch(ocd, symbols, groups, args):
    writer = None
    csv_file = None
    if args.csv:
        folder = os.path.join(tempfile.gettempdir(), "timing_watch")
        os.makedirs(folder, exist_ok=True)
        path = os.path.join(folder, time.strftime("%H%M%S") + ".csv")
        csv_file = open(path, "w", newline="", encoding="utf-8")
        writer = csv.writer(csv_file)
        writer.writerow(CSV_FIELDS)
        print(f"csv：{path}")

    first = prev = check_running(ocd, symbols, groups)
    reset_extremes(ocd, symbols)
    seen = {"pmin": U32, "pmax": 0, "loop_max": 0, "odr": [], "temp": [], "age": [], "loop": [], "ins": []}
    start = time.time()
    print("n0 = 这一拍没有新样本（重复），n2 = 来了 2 个及以上（跳过）；period_us 是这一行时间段里 DRDY 周期的最短~最长；"
          "loop_us 是最近一拍 / 这段最长。", flush=True)
    line = 0
    try:
        while True:
            time.sleep(args.period)
            s = read_snapshot(ocd, groups)
            reset_extremes(ocd, symbols)
            if line % 20 == 0:
                print(HEADER)
            print_line(time.time() - start, s, prev)
            line += 1
            if writer:
                writer.writerow(csv_row(s))
            if s["robot_drdy_period_min_us"] != U32:
                seen["pmin"] = min(seen["pmin"], s["robot_drdy_period_min_us"])
                seen["pmax"] = max(seen["pmax"], s["robot_drdy_period_max_us"])
            seen["loop_max"] = max(seen["loop_max"], s["robot_loop_max_us"])
            if odr_of(s):
                seen["odr"].append(odr_of(s))
                seen["temp"].append(s["temp"])
            seen["age"].append(s["robot_sample_age_us"])
            seen["loop"].append(s["robot_loop_us"])
            seen["ins"].append(s["robot_ins_us"])
            prev = s
            if args.seconds and time.time() - start >= args.seconds:
                break
    except KeyboardInterrupt:
        pass
    finally:
        if csv_file:
            csv_file.close()
    print_summary(first, prev, seen)


def watch_age(ocd, symbols, groups, args):
    check_running(ocd, symbols, groups)
    address = symbols["robot_sample_age_us"][0]
    print("样本年龄：Robot_Loop 开始时最新样本出来多久（0~1000 us），# 的位置就是年龄。看它往哪边走、多久转一圈，"
          "转到头时会在两头来回跳一小会儿（20 ms 一行，只抽到其中几拍）：", flush=True)
    start = time.time()
    try:
        while True:
            age = ocd.read(address, 32, 1)[0]
            t = time.time() - start
            pos = min(age, 999) * AGE_BAR_WIDTH // 1000
            print(f"{t:7.3f} s {age:5d} us |{'.' * pos}#{'.' * (AGE_BAR_WIDTH - 1 - pos)}|", flush=True)
            if args.seconds and t >= args.seconds:
                break
            time.sleep(0.02)
    except KeyboardInterrupt:
        pass


def main():
    parser = argparse.ArgumentParser(description="Gimbal_test_RS05：OpenOCD 不停核读 robot.c 的时序测量量")
    parser.add_argument("--elf", default=gw.DEFAULT_ELF, help="烧进板子的那个 ELF（默认 build/Debug/Gimbal_test_RS05.elf）")
    parser.add_argument("--openocd", default=gw.DEFAULT_OPENOCD, help="没有在跑的 OpenOCD 时用来起一个")
    parser.add_argument("--cfg", default=gw.DEFAULT_CFG, help="OpenOCD 的板子配置")
    parser.add_argument("--port", type=int, default=6666, help="OpenOCD 的 TCL 端口")
    parser.add_argument("--period", type=float, default=0.5, help="每行间隔，秒")
    parser.add_argument("--seconds", type=float, default=0.0, help="读这么多秒后打总结退出（0 = 一直读到 Ctrl+C）")
    parser.add_argument("--age", action="store_true", help="只看样本年龄，每 20 ms 一行")
    parser.add_argument("--csv", action="store_true", help="每次读数存一行 csv 到 %%TEMP%%/timing_watch/")
    # 固定 UTF-8：输出走管道（Git Bash 等）时 Python 默认用 GBK，中文会乱码
    sys.stdout.reconfigure(encoding="utf-8", errors="replace")
    sys.stderr.reconfigure(encoding="utf-8", errors="replace")
    args = parser.parse_args()

    symbols = find_symbols(args.elf)
    groups = plan_reads(symbols)
    ocd, proc = connect(args)
    try:
        if args.age:
            watch_age(ocd, symbols, groups, args)
        else:
            watch(ocd, symbols, groups, args)
    except (RuntimeError, ConnectionError, OSError) as e:
        log_hint = f"\nOpenOCD 日志：{proc.log_path}" if proc is not None else ""
        sys.exit(f"读写失败：{e}{log_hint}")
    finally:
        if proc is not None:
            # 自己起的 OpenOCD：shutdown 只退出 OpenOCD，板子上的程序照常跑
            try:
                ocd.command("shutdown")
            except (ConnectionError, OSError):
                pass
            try:
                proc.wait(timeout=3)
            except subprocess.TimeoutExpired:
                proc.terminate()


if __name__ == "__main__":
    main()
