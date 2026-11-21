#!/usr/bin/env python3
"""Gimbal_test_RS05 的 CPU 时间拆分：主循环的各段、各个中断各占多少时间。不改固件，不停核。

原理（统计采样，和电脑上的 perf 一样）：
- 经 OpenOCD 反复读 DWT_PCSR（0xE000101C）：读一次返回内核刚执行过的一条指令的地址，内核照常跑。
  读几万次，落在哪个函数里的比例就是这个函数占 CPU 时间的比例。经 DAPLink 每秒约读 500 次。
- 同一次还读 DWT_CYCCNT（CPU 周期计数，脚本开头打开，结束时恢复原样），用来定这个样本落在一拍里的哪个位置（相位）。
  用相位把一拍里各个位置的样本数拉平（经 USB 读的时刻可能和拍有关联），也用来画时间线（--timeline）。
  - 固件里有 ins_sample_us（第 1 步以后，Robot_Loop 跟着陀螺仪 DRDY 跑）：一拍是一个 DRDY 周期，零点放在 DRDY。
    每采 GRID_EVERY 个样本顺手读一次 TIM2 和 ins_sample_us（固件记的 DRDY 时刻），连成 DRDY 时刻表：数出每两次读
    之间隔了几个 DRDY，拟合间隔，再跟上随温度慢慢漂的那部分。TIM2 和 CYCCNT 出自同一个 PLL，正好 168 个周期 1 us，
    只差一个固定的偏移：读 TIM2 比读 CYCCNT 晚一个 USB 往返，只拿来粗对（准到 10 ms 以内就够，那段时间里 DRDY 的
    相位只漂零点几 ns），剩下的整体平移按 EXTI（DRDY）中断最早开始的地方归零。
  - 没有时（旧固件、旧样本文件、DRDY 没来在按 tick 跑）：一拍是 SysTick 的 LOAD + 1 个周期（1 ms），零点放在 SysTick。
- 归类：nm 查地址落在哪个函数里；objdump 里的 bl / b 是直接调用，从这个函数往上找“根”（中断入口、
  Robot_Loop 和 INS_Update 直接调的各个函数），找到哪个根就算谁的。几个根都调的单列成“共用”。
  只经函数指针调的（HAL 的 DMA 完成回调、USB 类的回调）按名字归到对应的中断。只在上电初始化时跑的不算。
- 空转：main 的 while(1)、Robot_Loop 和 Usb_SendGyro 开头“判断、没到就 return”那几行（按 addr2line 的行号分），
  以及轮询时调的几个小函数（POLL_HELPERS：HAL_GetTick、INS_IsSampleReady 等）。这几个小函数 INS 处理样本、
  读温度查超时时也调：落在 Robot_Loop 正忙的相位里的，算回 Robot_Loop。

用法（板子里是同一次编译的程序、正在跑；CLion 用 Run 烧录，不开调试会话）：
    python C:/Project/RM_Infantry/Test_Module/Gimbal/Gimbal_test_RS05/tools/cpu_profile.py
下面简写成 cpu_profile.py：
    cpu_profile.py --seconds 120    采样时长，默认 60 s。样本越多越准，打出来的表头有误差大概多少
    cpu_profile.py --timeline       另外打一拍里的时间线：每 10 us 一格，这一格里 CPU 主要在干什么
    cpu_profile.py --load 文件      不连板子，重新分析之前存下的样本（每次采样都存到 %TEMP%/cpu_profile/）

表里的数：
- 占比：这一项占 CPU 时间的比例；每拍 us = 占比 × 一拍（DRDY 零点时是一个 DRDY 周期，约 1000.34 us；SysTick 零点时 1 ms）。
- 每秒次数、每次 us：知道次数的才有。SysTick 数 uwTick；Robot_Loop、EXTI 在 DRDY 零点时按每个 DRDY 一次算，
  SysTick 零点时按每 ms 一次算。烧的是带第 0 步测量代码的程序（提交 d92bde5）时，改用 robot_loop_new_samples、
  robot_drdy_count 数。大疆电机的 CAN 帧按 feedback_count 数；RS05 没有计数，按“每拍一问一答、在线的各 1000 帧/s”估。
- DRDY 之后多久轮到（DRDY 零点时才有）：每一项的样本落在 DRDY 之后多少 us，取 1 %、50 %、99 % 分位。
  一个函数每拍从 s 跑到 s + d 的话，它的样本均匀落在这一段里，所以 1 % 分位约等于最早开始的时刻、99 % 约等于最晚结束的时刻。
- 读到 0xFFFFFFFF（内核这时给不出地址）或者不在任何函数里的样本算“无效”，单列。
- CLion 的 Debug 是 -O0，比优化后的程序慢。没接电机时两路 CAN 收不到帧，“CAN 接收”是 0；发送邮箱一直满着，
  各个 SendAll 比接电机时省事。

- 连接：先连 127.0.0.1:6666（别的脚本起的 OpenOCD 也能用）；没有就自己起一个，只连不复位、不停核，退出时 shutdown。
  日志写到 %TEMP%/cpu_profile_openocd.log。
- 会写 DWT_CTRL（打开 CYCCNT，结束时写回原值），别的只读。
"""

import argparse
import bisect
import json
import math
import os
import re
import shutil
import socket
import struct
import subprocess
import sys
import tempfile
import time

HERE = os.path.dirname(os.path.abspath(__file__))
sys.path.insert(0, HERE)
import gimbal_watch as gw  # noqa: E402
import shoot_watch as sw  # noqa: E402

DEMCR = 0xE000EDFC              # bit24 TRCENA：DWT 要它打开才工作（OpenOCD 连上时会打开）
DWT_CTRL = 0xE0001000           # bit0 CYCCNTENA
DWT_CYCCNT = 0xE0001004         # 从这里连读 7 个字，第 7 个是 DWT_PCSR（0xE000101C）
SYST_LOAD = 0xE000E014
TIM2_CNT = 0x40000024           # 固件的 1 MHz 时间戳（ins_sample_us 用的就是它）
CYCLES_PER_US = 168             # CYCCNT 是 168 MHz 的内核时钟；TIM2 是 84 MHz / 84，同一个 PLL
GRID_EVERY = 25                 # 每采这么多个 PC 样本，读一次 TIM2 和 ins_sample_us（DRDY 时刻表），约 45 ms 一次
DRDY_GUESS_US = 1e6 / 999.66    # 上板量过的 DRDY 间隔，只用来数两次读之间隔了几个 DRDY（随后用时刻表自己拟合）
GRID_SMOOTH = 10                # DRDY 相位的慢漂：时刻表残差前后各取这么多个点求中位数（约 ±0.45 s）
GRID_OUTLIER_US = 20.0          # 时刻表里离慢漂超过这么多的点不用（那次 EXTI 中断晚了，第 0 步见过晚 46.5 us 的）
FLASH_BASE = 0x08000000
FLASH_END = 0x08100000
INVALID_PC = 0xFFFFFFFF
PHASE_BINS = 100                # 1 ms 分 100 格（每格 10 us）：拉平、时间线
BUSY_BINS = 500                 # 判断主循环这时在 Robot_Loop 里忙还是在空转，用细一点的 2 us 一格
U32 = 0xFFFFFFFF

IDLE = ("主循环", "空转（等下一拍）")
LOOP = ("主循环", "Robot_Loop")
INS = LOOP + ("INS_Update",)
CAN = ("中断", "CAN 接收")
SPI = ("中断", "SPI1、DMA2（BMI088 读完）")
USB = ("中断", "USB")
EXTI = ("中断", "EXTI（DRDY）")
INIT = ("初始化（上电时跑一次）",)
INVALID = ("无效样本",)
# 轮询时调的小函数：默认算空转，落在 Robot_Loop 正忙的相位里的算回 Robot_Loop（INS 处理样本、读温度查超时也调它们）。
# Robot_NowUs 在 EXTI 中断里也调一次，那几条指令也算成了空转，每拍约 0.1 us，不管
POLL_HELPERS = ("HAL_GetTick", "INS_IsSampleReady", "BMI088_IsSampleReady", "INS_GetSampleUs", "Robot_NowUs")
ROOTS = {
    "CAN1_RX0_IRQHandler": CAN,
    "CAN1_RX1_IRQHandler": CAN,
    "CAN2_RX0_IRQHandler": CAN,
    "RS05_Manager_ProcessRxFrame": CAN + ("RS05 解析",),
    "DJI_MotorManager_ProcessRxFrame": CAN + ("大疆解析",),
    "DMA2_Stream0_IRQHandler": SPI,
    "DMA2_Stream3_IRQHandler": SPI,
    "SPI1_IRQHandler": SPI,
    "EXTI9_5_IRQHandler": EXTI,
    "OTG_FS_IRQHandler": USB,
    "SysTick_Handler": ("中断", "SysTick"),
    "main": IDLE,
    "Robot_Loop": LOOP + ("Robot_Loop 自身",),
    "Usb_SendGyro": ("主循环", "Usb_SendGyro（每 20 ms 发一次）"),
    "Robot_Init": INIT,
    "Reset_Handler": INIT,
}
ROOTS.update({name: IDLE for name in POLL_HELPERS})
INIT_RE = re.compile(r"^(HAL_Init|SystemClock_Config|MX_\w+_Init|SystemInit|__libc_init_array)$")
# 只经函数指针调、objdump 里找不到调用者的，按名字归
POINTER_ROOTS = ((re.compile(r"^SPI_DMA|^HAL_SPI_\w*Callback$"), SPI),
                 (re.compile(r"^(USBD_|CDC_|PCD_|USB_)"), USB))
# 直接调的函数各算一段
SECTION_PARENTS = (("Robot_Loop", LOOP), ("INS_Update", INS))
# 开头“比较 tick、没到时间就 return”的函数：到第一个 return; 为止的几行和最后的 } 算空转
POLL_FUNCS = ("Robot_Loop", "Usb_SendGyro")

FUNC_RE = re.compile(r"^([0-9a-f]{8}) <(.+)>:$")
BRANCH_RE = re.compile(r"^\s*[0-9a-f]+:\s+b\S*\s+([0-9a-f]+) <[^>]+>\s*$")
LOCATION_RE = re.compile(r"^(.*):(\d+)(?: \(discriminator \d+\))?$")   # addr2line 的“文件:行号”

SYMBOLS = ("uwTick", "robot_drdy_count", "robot_loop_new_samples", "ins_feedback", "ins_sample_us",
           "shoot_friction_1", "shoot_friction_2", "shoot_feeder", "gimbal_yaw", "gimbal_pitch")
DJI_MOTORS = ("shoot_friction_1", "shoot_friction_2", "shoot_feeder")
RS05_AXES = ("gimbal_yaw", "gimbal_pitch")
DJI_COUNT_OFFSET = struct.calcsize(sw.MOTOR_FORMAT) - 8   # DJI_Motor_t 最后两个：feedback_count、last_feedback_tick
RS05_TICK_OFFSET = struct.calcsize(gw.MOTOR_FORMAT) - 4   # 云台轴结构体开头是 RS05_Motor_t，最后一个是 feedback_tick


def tool(name):
    path = shutil.which(name)
    if path is None:
        sys.exit(f"找不到 {name}（要在 PATH 里）")
    return path


def run(*cmd):
    return subprocess.run(cmd, capture_output=True, text=True, check=True, errors="replace").stdout


class Elf:
    """ELF 里的函数、直接调用关系、每个函数算哪一组。"""

    def __init__(self, path):
        if not os.path.exists(path):
            sys.exit(f"找不到 ELF：{path}\n先在 CLion 里编译，或者用 --elf 指定")
        self.path = path
        self.funcs = {}         # 起始地址 → (结束地址, 名字)
        self.data = {}          # 变量名 → (地址, 大小)
        for line in run(tool("arm-none-eabi-nm"), "-S", "-n", "--defined-only", path).splitlines():
            parts = line.split()
            if len(parts) != 4:
                continue
            address, size, kind, name = int(parts[0], 16), int(parts[1], 16), parts[2], parts[3]
            if kind in "tTwW" and size and FLASH_BASE <= address < FLASH_END:
                self.funcs[address & ~1] = ((address & ~1) + size, name)
            elif kind in "bBdD" and name in SYMBOLS:
                self.data[name] = (address, size)
        self.starts = sorted(self.funcs)
        self.shared = {}        # “共用：函数名” → 调它的那几组
        self.calls = self._calls()
        self.group = self._groups()
        self.poll_lines = self._poll_lines()

    def name(self, start):
        return self.funcs[start][1]

    def find(self, pc):
        """pc 落在哪个函数里，返回函数起始地址；不在任何函数里返回 None。"""
        i = bisect.bisect_right(self.starts, pc) - 1
        if i >= 0 and pc < self.funcs[self.starts[i]][0]:
            return self.starts[i]
        return None

    def _calls(self):
        calls = {start: [] for start in self.starts}
        current = None
        for line in run(tool("arm-none-eabi-objdump"), "-d", "--no-show-raw-insn", self.path).splitlines():
            m = FUNC_RE.match(line)
            if m:
                current = int(m.group(1), 16)
                current = current if current in calls else None
                continue
            m = BRANCH_RE.match(line) if current is not None else None
            if m:
                target = int(m.group(1), 16)
                # 跳到别的函数开头才算调用（b 跳过去是尾调用）；跳到函数中间是函数内部的分支
                if target in calls and target != current and target not in calls[current]:
                    calls[current].append(target)
        return calls

    def _groups(self):
        by_name = {}
        for start in self.starts:
            by_name.setdefault(self.name(start), []).append(start)
        callers = {start: [] for start in self.starts}
        for caller, targets in self.calls.items():
            for target in targets:
                callers[target].append(caller)

        roots = {}
        for start in self.starts:
            name = self.name(start)
            if name in ROOTS:
                roots[start] = ROOTS[name]
            elif INIT_RE.match(name):
                roots[start] = INIT
            elif not callers[start]:
                for pattern, group in POINTER_ROOTS:
                    if pattern.search(name):
                        roots[start] = group
                        break
        for parent_name, parent_group in SECTION_PARENTS:
            for parent in by_name.get(parent_name, []):
                for callee in self.calls[parent]:
                    if callee not in roots:
                        roots[callee] = parent_group + (self.name(callee),)

        group = {}
        for start in self.starts:
            if start in roots:
                group[start] = roots[start]
                continue
            found, seen, todo = set(), {start}, [start]
            while todo:
                for caller in callers[todo.pop()]:
                    if caller in seen:
                        continue
                    seen.add(caller)
                    if caller in roots:
                        found.add(roots[caller])
                    else:
                        todo.append(caller)
            if len(found) > 1:
                found.discard(INIT)     # 初始化时也调、运行时由别处调的：只算运行时那边
            name = self.name(start)
            if not found:
                group[start] = ("其他（找不到是谁调的）", name)
            elif len(found) == 1:
                group[start] = found.pop()
            else:
                common = os.path.commonprefix(sorted(found))
                label = f"共用：{name}"
                self.shared[label] = sorted(g[-1] for g in found)
                group[start] = tuple(common) + (label,) if common else ("共用（中断和主循环都调）", label)
        return group

    def _poll_lines(self):
        """POLL_FUNCS 里算空转的地址集合。"""
        idle = set()
        for start in self.starts:
            name = self.name(start)
            if name not in POLL_FUNCS:
                continue
            addresses = list(range(start, self.funcs[start][0], 2))
            out = run(tool("arm-none-eabi-addr2line"), "-e", self.path, *[hex(a) for a in addresses]).splitlines()
            locations = {}
            for address, text in zip(addresses, out):
                m = LOCATION_RE.match(text.strip())
                if m and m.group(1) != "??" and int(m.group(2)):
                    locations[address] = (m.group(1), int(m.group(2)))
            if not locations:
                sys.exit(f"addr2line 查不到 {name} 的行号（ELF 要带调试信息，用 Debug 编译）")
            # 只看函数自己那个文件的行：内联进来的 __disable_irq 等，addr2line 报的是 cmsis_gcc.h 的行号
            source = locations[min(locations)][0]
            lines = {a: n for a, (f, n) in locations.items() if f == source}
            first, last = min(lines.values()), max(lines.values())
            with open(source, encoding="utf-8", errors="replace") as f:
                text = f.read().splitlines()
            returns = [n for n in range(first, last + 1) if n <= len(text) and text[n - 1].strip() == "return;"]
            if not returns:
                sys.exit(f"{name} 里找不到开头的 return;，分不出空转和干活（{source}）")
            idle.update(a for a, n in lines.items() if n <= returns[0] or n == last)
        return idle


def check_flash(ocd, elf_path):
    """板子 flash 里的程序和 ELF 是否同一次编译：整片逐块比。"""
    image_path = os.path.join(tempfile.gettempdir(), "cpu_profile_image.bin")
    subprocess.run([tool("arm-none-eabi-objcopy"), "-O", "binary", elf_path, image_path], check=True)
    with open(image_path, "rb") as f:
        image = f.read()
    bad = []
    for offset in range(0, len(image), 4096):
        chunk = image[offset:offset + 4096]
        board = ocd.read_bytes(FLASH_BASE + offset, len(chunk))
        if board != chunk:
            bad.append(FLASH_BASE + offset)
    if bad:
        sys.exit(f"板子里的程序和 ELF 不一样（这些 4 KB 块不同：{', '.join(hex(a) for a in bad)}）。\n"
                 "先在 CLion 里 Run 烧一遍 Gimbal_test_RS05（ELF 要和板子里的是同一次编译）。")


def read_u32(ocd, address):
    return ocd.read(address, 32, 1)[0]


def read_counts(ocd, data):
    c = {"tick": read_u32(ocd, data["uwTick"][0])}
    if "robot_drdy_count" in data and "robot_loop_new_samples" in data:   # 只有第 0 步测量代码那次编译有
        c["drdy"] = read_u32(ocd, data["robot_drdy_count"][0])
        c["loops"] = sum(ocd.read(data["robot_loop_new_samples"][0], 32, 3)) & U32
    if all(name in data for name in DJI_MOTORS + RS05_AXES):
        c["dji"] = sum(read_u32(ocd, data[name][0] + DJI_COUNT_OFFSET) for name in DJI_MOTORS) & U32
        c["rs05_ticks"] = [read_u32(ocd, data[name][0] + RS05_TICK_OFFSET) for name in RS05_AXES]
    return c


def connect(args):
    try:
        return gw.OpenOcd(socket.create_connection(("127.0.0.1", args.port), timeout=1.0)), None
    except OSError:
        pass
    exe = args.openocd if os.path.exists(args.openocd) else shutil.which("openocd")
    if exe is None:
        sys.exit(f"找不到 OpenOCD：{args.openocd}（用 --openocd 指定）")
    log_path = os.path.join(tempfile.gettempdir(), "cpu_profile_openocd.log")
    for attempt in range(2):
        try:
            return gw.spawn_openocd(args, exe, log_path)
        except gw.StartupError as e:
            # 上一个 OpenOCD 被半路打断时，第一次连会报 command mismatch，再连一次一般就好
            if attempt == 1 or "command mismatch" not in str(e):
                sys.exit(f"{e}\n（DAPLink 没插好、板子没电，或者被 CLion 的调试会话占着？）")


def collect(ocd, elf, seconds):
    """在板子上采样，返回 (样本 [(cyccnt, pc)], 附带信息)。"""
    check_flash(ocd, elf.path)
    data = elf.data
    if "uwTick" not in data:
        sys.exit("ELF 里没有 uwTick（是 Gimbal_test_RS05 的 ELF 吗）")
    first = read_counts(ocd, data)
    time.sleep(0.2)
    if read_u32(ocd, data["uwTick"][0]) == first["tick"]:
        sys.exit("uwTick 不涨，程序没在跑：可能卡在 Error_Handler / HardFault，也可能被调试器停住了。")
    if "ins_feedback" in data and ocd.read(data["ins_feedback"][0], 8, 1)[0] == 0:
        print(f"注意：INS 还没开始解算（ins_feedback.valid = 0）。没接 24 V 时要上电 120 s 加热超时以后才标定，"
              f"再约 5 s 才解算；现在量到的 INS_Update 是等温度 / 标定时的。板子已经上电 {first['tick'] / 1000:.0f} s。",
              flush=True)

    demcr = read_u32(ocd, DEMCR)
    if not demcr & (1 << 24):
        ocd.write(DEMCR, 32, [demcr | (1 << 24)])
    ctrl = read_u32(ocd, DWT_CTRL)
    ocd.write(DWT_CTRL, 32, [ctrl | 1])
    grid_address = data["ins_sample_us"][0] if "ins_sample_us" in data else None
    try:
        period = read_u32(ocd, SYST_LOAD) + 1
        start = read_counts(ocd, data)
        print(f"开始采样 {seconds:.0f} s（Ctrl+C 提前结束）……", flush=True)
        samples = []
        grid = []       # [第几个样本之后读的, TIM2, ins_sample_us]
        errors = failed_in_row = 0
        t0 = time.time()
        report = t0 + 10.0
        try:
            while time.time() - t0 < seconds:
                try:
                    words = ocd.read(DWT_CYCCNT, 32, 7)
                except RuntimeError:
                    # SWD 偶尔出错（OpenOCD 日志里会有一行重读 DPIDR），丢掉这一个样本接着采
                    errors += 1
                    failed_in_row += 1
                    if failed_in_row >= 10:
                        print("连续读错 10 次，提前结束（DAPLink 松了、板子断电了？）", flush=True)
                        break
                    time.sleep(0.05)
                    continue
                failed_in_row = 0
                samples.append((words[0], words[6]))
                if grid_address is not None and len(samples) % GRID_EVERY == 0:
                    try:
                        grid.append([len(samples) - 1, read_u32(ocd, TIM2_CNT), read_u32(ocd, grid_address)])
                    except RuntimeError:
                        errors += 1
                if time.time() >= report:
                    print(f"  已采 {len(samples)} 个（{time.time() - t0:.0f} s）", flush=True)
                    report += 10.0
        except KeyboardInterrupt:
            print("提前结束。", flush=True)
        end = read_counts(ocd, data)
    finally:
        try:
            ocd.write(DWT_CTRL, 32, [ctrl])
            if not demcr & (1 << 24):
                ocd.write(DEMCR, 32, [demcr])
        except RuntimeError:
            print("DWT_CTRL 没写回原值（CYCCNT 会一直开着，对程序没影响）。", flush=True)
    ms = (end["tick"] - start["tick"]) & U32
    meta = {"period": period, "ms": ms, "read_errors": errors, "grid": grid,
            "elf": elf.path, "elf_mtime": os.path.getmtime(elf.path), "time": time.strftime("%Y-%m-%d %H:%M:%S")}
    if "loops" in end:
        meta["loops"] = (end["loops"] - start["loops"]) & U32
        meta["drdy"] = (end["drdy"] - start["drdy"]) & U32
    if "dji" in end:
        meta["dji"] = (end["dji"] - start["dji"]) & U32
        meta["rs05_online"] = sum(1 for t in end["rs05_ticks"] if t and ((end["tick"] - t) & U32) < 20)
    return samples, meta


def save(samples, meta):
    folder = os.path.join(tempfile.gettempdir(), "cpu_profile")
    os.makedirs(folder, exist_ok=True)
    path = os.path.join(folder, time.strftime("%H%M%S") + ".json")
    with open(path, "w", encoding="utf-8") as f:
        json.dump({"meta": meta, "samples": samples}, f)
    return path


def unwrap(samples):
    """CYCCNT 是 32 位的，168 MHz 下约 25.6 s 回绕一次，2^32 又不是一拍的整数倍：按先后顺序展开成从第一个样本数起的周期数。
    相邻两个样本只隔一两 ms，差值按 uint32 算就不会错。"""
    total, previous, out = 0, samples[0][0], []
    for cyc, _ in samples:
        total += (cyc - previous) & U32
        previous = cyc
        out.append(total)
    return out


def sd(a, b):
    """a − b 按 32 位有符号算。"""
    return ((a - b + 0x80000000) & U32) - 0x80000000


def base_groups(elf, samples):
    """每个样本按函数归组。轮询用的小函数这时都算空转，有了相位以后由 assign_poll_helpers 再分。"""
    starts = [elf.find(pc) if pc != INVALID_PC else None for _, pc in samples]
    groups = []
    for (_, pc), start in zip(samples, starts):
        if start is None:
            groups.append(INVALID)
        elif pc in elf.poll_lines:
            groups.append(IDLE)
        else:
            groups.append(elf.group[start])
    return starts, groups


def systick_phases(samples, period):
    """每个样本在 SysTick 一拍里的位置（0~1）。零点不定，画时间线时取 SysTick 样本最多的那一格。"""
    return [(c % period) / period for c in unwrap(samples)]


def running_median(values, half):
    out = []
    for i in range(len(values)):
        window = sorted(values[max(0, i - half):i + half + 1])
        out.append(window[len(window) // 2])
    return out


def fit_line(xs, ys):
    """最小二乘 y = a + b·x，返回 (a, b)。"""
    n = len(xs)
    mx, my = sum(xs) / n, sum(ys) / n
    b = sum((x - mx) * (y - my) for x, y in zip(xs, ys)) / sum((x - mx) ** 2 for x in xs)
    return my - b * mx, b


def drdy_phases(samples, groups, meta):
    """按 DRDY 时刻表算每个样本在 DRDY 周期里的位置（0~1），零点是 EXTI（DRDY）中断最早开始的地方。
    返回 (相位, 周期 us, 说明)；用不了时返回 (None, None, {"reason": 原因})。"""
    grid = meta.get("grid") or []
    if len(grid) < 3 * GRID_SMOOTH:
        return None, None, {"reason": "没有 DRDY 时刻表：固件里没有 ins_sample_us，或者是旧的样本文件"}
    cycles = unwrap(samples)
    tim2, drdy, cyc = [], [], []
    total, previous = 0, grid[0][1]
    for index, t, s in grid:
        total += (t - previous) & U32
        previous = t
        tim2.append(total)
        drdy.append(total - sd(t, s))       # 先读 TIM2、后读样本时刻：样本时刻可能比读到的 TIM2 新
        cyc.append(cycles[index])
    if drdy[-1] - drdy[0] < 0.5 * (tim2[-1] - tim2[0]):
        return None, None, {"reason": "样本时刻几乎没变：DRDY 没来，Robot_Loop 在按 tick 跑"}

    # 两次读之间隔了几个 DRDY：先按大概的间隔数，拟合出间隔以后再数一遍（隔得久的两次读，按大概的间隔可能数差一个）
    period = DRDY_GUESS_US
    for _ in range(2):
        k = [0]
        for a, b in zip(drdy, drdy[1:]):
            k.append(k[-1] + round((b - a) / period))
        origin, period = fit_line(k, drdy)
    # 残差 = 相位随温度的慢漂 + 中断延迟。离滑动中位数太远的点（那次中断晚了）去掉，剩下的再取一次滑动中位数当慢漂
    residual = [d - (origin + period * n) for d, n in zip(drdy, k)]
    first = running_median(residual, GRID_SMOOTH)
    kept = [(n, r) for n, r, m in zip(k, residual, first) if abs(r - m) <= GRID_OUTLIER_US]
    kept_k = [n for n, _ in kept]
    drift = running_median([r for _, r in kept], GRID_SMOOTH)

    # CYCCNT 换成 TIM2 的 us：t = (CYCCNT + offset) / 168。offset 用每次读的 TIM2 粗估（取中位数），准到 10 ms 以内就行
    offsets = sorted(CYCLES_PER_US * t - c for t, c in zip(tim2, cyc))
    offset = offsets[len(offsets) // 2]
    tau = []
    for c in cycles:
        t = (c + offset) / CYCLES_PER_US
        x = (t - origin) / period
        j = bisect.bisect_left(kept_k, x)
        if j <= 0:
            r = drift[0]
        elif j >= len(kept_k):
            r = drift[-1]
        else:
            r = drift[j - 1] + (drift[j] - drift[j - 1]) * (x - kept_k[j - 1]) / (kept_k[j] - kept_k[j - 1])
        n = math.floor((t - origin - r) / period)
        tau.append(t - (origin + period * n + r))

    # 零点：EXTI 中断的样本在一拍里聚成一小段（在圆周上求它的中心，免得正好跨过一拍的头尾），取 1 % 分位
    exti = [t for t, g in zip(tau, groups) if g == EXTI]
    if len(exti) < 20:
        return None, None, {"reason": f"EXTI（DRDY）中断的样本只有 {len(exti)} 个，定不了零点"}
    angle = math.atan2(sum(math.sin(2 * math.pi * t / period) for t in exti),
                       sum(math.cos(2 * math.pi * t / period) for t in exti))
    center = angle / (2 * math.pi) * period
    rel = sorted((t - center + period / 2) % period - period / 2 for t in exti)
    zero = center + rel[len(rel) // 100]
    phases = [((t - zero) % period) / period for t in tau]
    scatter = [r - m for (_, r), m in zip(kept, drift)]
    late = sorted(r - m for r, m in zip(residual, first) if r - m > GRID_OUTLIER_US)
    info = {"grid": len(grid), "outliers": len(grid) - len(kept), "late": len(late),
            "late_us": late[len(late) // 2] if late else 0.0,
            "scatter_us": (sum(x * x for x in scatter) / len(scatter)) ** 0.5,
            "drift_us": max(drift) - min(drift),
            "offset_spread_us": (offsets[len(offsets) * 9 // 10] - offsets[len(offsets) // 10]) / CYCLES_PER_US,
            "exti": len(exti), "exti_width_us": rel[len(rel) * 99 // 100] - rel[len(rel) // 100]}
    return phases, period, info


def assign_poll_helpers(elf, starts, groups, phases):
    """轮询用的小函数：所在的相位格里 Robot_Loop 在忙（干活的样本比空转的多）就算回 Robot_Loop，不然是空转。"""
    helpers = {s for s in elf.starts if elf.name(s) in POLL_HELPERS}
    fine = [min(int(p * BUSY_BINS), BUSY_BINS - 1) for p in phases]
    busy = [0] * BUSY_BINS
    for start, group, b in zip(starts, groups, fine):
        if start in helpers:
            continue
        if group[:2] == LOOP:
            busy[b] += 1
        elif group == IDLE:
            busy[b] -= 1
    # 格里只有这几个小函数、判断不了的，往两边相邻的格看
    decided = []
    for b in range(BUSY_BINS):
        score, k = busy[b], 0
        while score == 0 and k < BUSY_BINS // 2:
            k += 1
            score = busy[(b - k) % BUSY_BINS] + busy[(b + k) % BUSY_BINS]
        decided.append(score > 0)
    for i, (start, b) in enumerate(zip(starts, fine)):
        if start in helpers and decided[b]:
            groups[i] = LOOP + (f"{elf.name(start)}（Robot_Loop 里调的）",)


def weights_of(bins):
    """按相位格拉平：每格的样本加起来权重一样（总和 = 样本数）。"""
    per_bin = [0] * PHASE_BINS
    for b in bins:
        per_bin[b] += 1
    filled = sum(1 for n in per_bin if n)
    return [len(bins) / filled / per_bin[b] for b in bins], per_bin


def analyze(elf, samples, meta, timeline):
    starts, groups = base_groups(elf, samples)
    phases, period_us, info = drdy_phases(samples, groups, meta)
    drdy = phases is not None
    if not drdy:
        phases, period_us = systick_phases(samples, meta["period"]), meta["period"] / CYCLES_PER_US
    assign_poll_helpers(elf, starts, groups, phases)
    bins = [min(int(p * PHASE_BINS), PHASE_BINS - 1) for p in phases]
    weights, per_bin = weights_of(bins)
    total = float(len(samples))
    share = {}
    for g, w in zip(groups, weights):
        share[g] = share.get(g, 0.0) + w / total

    seconds = meta["ms"] / 1000.0
    filled = [n for n in per_bin if n]
    print()
    errors = f"，读错跳过 {meta['read_errors']} 次" if meta.get("read_errors") else ""
    print(f"—— CPU 时间拆分（{meta['time']}，采了 {len(samples)} 个样本、{seconds:.1f} s{errors}）——")
    if drdy:
        print(f"一拍 = 一个 DRDY 周期 {period_us:.3f} us（ODR {1e6 / period_us:.3f} Hz），零点 = DRDY（EXTI 中断最早开始的地方）。")
        late = (f"，其中晚到的 {info['late']} 次（{info['late'] * 100 / info['grid']:.1f} %，中位数晚 "
                f"{info['late_us']:.0f} us：那几拍整条链跟着往后推，表里的 99 % 分位含着它们）" if info["late"] else "")
        print(f"DRDY 时刻表读了 {info['grid']} 次：扣掉慢漂以后散布 {info['scatter_us']:.1f} us（RMS），"
              f"离慢漂太远不用的 {info['outliers']} 次{late}；慢漂前后一共 {info['drift_us']:.1f} us。"
              f"CYCCNT 对 TIM2 的偏移每次读散布 {info['offset_spread_us']:.0f} us（10 %~90 %，准到 10 ms 以内就够）；"
              f"按 EXTI 中断的 {info['exti']} 个样本归零，它们在一拍里占 {info['exti_width_us']:.1f} us（1 %~99 %）。")
    else:
        print(f"一拍 = SysTick 的 LOAD + 1 = {meta['period']} 个 CPU 周期，零点 = SysTick（{info['reason']}）。")
    print(f"CYCCNT 量到每 ms 约 {unwrap(samples)[-1] / meta['ms']:.0f} 个周期。"
          f"一拍分 {PHASE_BINS} 格，每格采到 {min(filled)}~{max(filled)} 个，已按格拉平"
          f"{'' if len(filled) == PHASE_BINS else f'（有 {PHASE_BINS - len(filled)} 格一个都没采到！）'}。")
    err = 1.96 * (0.01 * 0.99 / max(1.0, total)) ** 0.5 * period_us
    print(f"统计误差（95 %）：占比 1 % 的项约 ±{err:.2f} us/拍，占比越大误差越大（50 % 的约 "
          f"±{1.96 * (0.25 / max(1.0, total)) ** 0.5 * period_us:.1f} us/拍）。")

    beats = meta["ms"] * 1000.0 / period_us if drdy else meta["ms"]
    rates = {LOOP: meta.get("loops", beats), EXTI: meta.get("drdy", beats),
             ("中断", "SysTick"): meta["ms"], ("主循环", "Usb_SendGyro（每 20 ms 发一次）"): meta["ms"] // 20}
    if "loops" not in meta:
        print(f"Robot_Loop、EXTI 的每秒次数按{'每个 DRDY ' if drdy else '每 ms '}一次算（固件里没有计数）。")
    if "dji" in meta:
        rs05 = meta["rs05_online"] * meta["ms"]
        rates[CAN] = rs05 + meta["dji"]
        rates[CAN + ("RS05 解析",)] = rs05
        rates[CAN + ("大疆解析",)] = meta["dji"]
        if rates[CAN]:
            print(f"CAN 帧：大疆 {meta['dji'] / seconds:.0f} 帧/s（feedback_count），RS05 在线 {meta['rs05_online']} 个、"
                  f"按每拍一问一答估 {rs05 / seconds:.0f} 帧/s。")
        else:
            print("CAN：没收到电机的帧（大疆 feedback_count 没涨，RS05 不在线），CAN 接收中断这次是 0。")

    print()
    print("    占比  每拍us  每秒次数   每次us  项目（同一层按占比从大到小）")
    tree = {}
    for g, s in share.items():
        for depth in range(1, len(g) + 1):
            tree[g[:depth]] = tree.get(g[:depth], 0.0) + s

    def show(prefix):
        children = sorted((p for p in tree if len(p) == len(prefix) + 1 and p[:len(prefix)] == prefix),
                          key=lambda p: -tree[p])
        for p in children:
            s = tree[p]
            counted = f"{rates[p] / seconds:10.0f}{s * seconds * 1e6 / rates[p]:9.2f}" if rates.get(p) else " " * 19
            print(f"{s * 100:7.2f}%{s * period_us:8.1f}{counted}  {'  ' * (len(p) - 1)}{p[-1]}")
            show(p)

    show(())
    shown = [label for label in elf.shared if any(g[-1] == label for g in share)]
    if shown:
        print()
        print("“共用”是几处都调的函数，分不出是谁调的：")
        for label in shown:
            print(f"  {label[3:]} ← {'、'.join(elf.shared[label])}")

    print()
    print("干活的函数里，自身时间最多的（不含它调的函数；空转、无效不列）：")
    own = {}
    for start, w, g in zip(starts, weights, groups):
        if g in (IDLE, INVALID):
            continue
        entry = own.setdefault(elf.name(start), [0.0, {}])
        entry[0] += w / total
        entry[1][g[-1]] = entry[1].get(g[-1], 0.0) + w
    for name, (s, where) in sorted(own.items(), key=lambda kv: -kv[1][0])[:25]:
        main_group = max(where, key=where.get)
        print(f"  {s * 100:6.2f}%  {s * period_us:6.1f} us/拍  {name}（算在：{main_group}）")

    if drdy:
        print_latency(groups, phases, weights, period_us)
    if timeline:
        print_timeline(groups, bins, per_bin, period_us, drdy)


# “DRDY 之后多久轮到”表里的项：名字，哪些组算它
LATENCY_ITEMS = (
    ("EXTI（DRDY）中断：启动 DMA", lambda g: g == EXTI),
    ("SPI1、DMA2 中断：两段 DMA 读完", lambda g: g == SPI),
    ("Robot_Loop（不含开头的轮询）", lambda g: g[:2] == LOOP),
    ("  INS_Update：处理样本、姿态解算（每 100 ms 多一次读温度）", lambda g: g[:3] == INS),
    ("  Gimbal_Update", lambda g: g == LOOP + ("Gimbal_Update",)),
    ("  Shoot_Update", lambda g: g == LOOP + ("Shoot_Update",)),
    ("  RS05_Manager_SendAll", lambda g: g == LOOP + ("RS05_Manager_SendAll",)),
    ("  DJI_MotorManager_SendAll", lambda g: g == LOOP + ("DJI_MotorManager_SendAll",)),
)


def weighted_quantiles(pairs, qs):
    """pairs 是 (值, 权重)，qs 从小到大。"""
    pairs = sorted(pairs)
    total = sum(w for _, w in pairs)
    out, acc, j = [], 0.0, 0
    for q in qs:
        while j < len(pairs) - 1 and acc + pairs[j][1] < q * total:
            acc += pairs[j][1]
            j += 1
        out.append(pairs[j][0])
    return out


def print_latency(groups, phases, weights, period_us):
    print()
    print("DRDY 之后多久轮到（us；1 % 分位约等于最早开始，99 % 约等于最晚结束；拉平的权重照算）：")
    print("    最早    中位    最晚   样本数  项目")
    for label, match in LATENCY_ITEMS:
        pairs = [(p * period_us, w) for p, w, g in zip(phases, weights, groups) if match(g)]
        if len(pairs) < 20:
            print(f"  {'':22}{len(pairs):7d}  {label}（样本太少，不算）")
            continue
        q1, q50, q99 = weighted_quantiles(pairs, (0.01, 0.5, 0.99))
        print(f"  {q1:6.1f}  {q50:6.1f}  {q99:6.1f}  {len(pairs):7d}  {label}")


def print_timeline(groups, bins, per_bin, period_us, drdy):
    """每 10 us 一格，列出这一格里占比最大的几项。DRDY 零点时 0 就是 DRDY；SysTick 零点时取 SysTick 样本最多的那一格。"""
    cells = [{} for _ in range(PHASE_BINS)]
    for g, b in zip(groups, bins):
        cells[b][g[-1]] = cells[b].get(g[-1], 0) + 1
    width = period_us / PHASE_BINS
    if drdy:
        origin = 0
        zero = "DRDY"
    else:
        systick = [cells[b].get("SysTick", 0) / per_bin[b] if per_bin[b] else 0.0 for b in range(PHASE_BINS)]
        origin = max(range(PHASE_BINS), key=lambda b: systick[b]) if max(systick) > 0 else 0
        zero = f"SysTick 中断所在的那 {width:.0f} us"
    print()
    print(f"一拍里的时间线（0 = {zero}；每格列出占比最大的几项，空转 > 90 % 的格并在一起）：")
    idle_name = IDLE[-1]
    run_start = None
    for k in range(PHASE_BINS + 1):
        b = (origin + k) % PHASE_BINS
        n = per_bin[b] if k < PHASE_BINS else 0
        idle = cells[b].get(idle_name, 0) / n if n else 1.0
        if k < PHASE_BINS and idle > 0.9:
            if run_start is None:
                run_start = k
            continue
        if run_start is not None:
            print(f"  {run_start * width:5.0f}~{k * width:4.0f} us  基本空转")
            run_start = None
        if k == PHASE_BINS:
            break
        top = sorted(cells[b].items(), key=lambda kv: -kv[1])[:4]
        print(f"  {k * width:5.0f}~{(k + 1) * width:4.0f} us  " + "  ".join(f"{name} {c * 100 / n:.0f}%" for name, c in top))


def main():
    parser = argparse.ArgumentParser(description="Gimbal_test_RS05：不停核采样 PC，拆分主循环和中断各占多少 CPU 时间")
    parser.add_argument("--elf", default=gw.DEFAULT_ELF, help="烧进板子的那个 ELF（默认 build/Debug/Gimbal_test_RS05.elf）")
    parser.add_argument("--openocd", default=gw.DEFAULT_OPENOCD, help="没有在跑的 OpenOCD 时用来起一个")
    parser.add_argument("--cfg", default=gw.DEFAULT_CFG, help="OpenOCD 的板子配置")
    parser.add_argument("--port", type=int, default=6666, help="OpenOCD 的 TCL 端口")
    parser.add_argument("--seconds", type=float, default=60.0, help="采样时长，秒")
    parser.add_argument("--timeline", action="store_true", help="另外打 1 ms 里的时间线")
    parser.add_argument("--load", help="不连板子，分析之前存下的样本文件")
    # 固定 UTF-8：输出走管道（Git Bash 等）时 Python 默认用 GBK，中文会乱码
    sys.stdout.reconfigure(encoding="utf-8", errors="replace")
    sys.stderr.reconfigure(encoding="utf-8", errors="replace")
    args = parser.parse_args()

    elf = Elf(args.elf)
    if args.load:
        with open(args.load, encoding="utf-8") as f:
            saved = json.load(f)
        samples, meta = [tuple(s) for s in saved["samples"]], saved["meta"]
        if abs(meta.get("elf_mtime", 0) - os.path.getmtime(elf.path)) > 1:
            print("注意：ELF 在采样以后重新编译过，地址可能对不上。", flush=True)
    else:
        ocd, proc = connect(args)
        try:
            samples, meta = collect(ocd, elf, args.seconds)
        except (RuntimeError, ConnectionError, OSError) as e:
            log_hint = f"\nOpenOCD 日志：{proc.log_path}" if proc is not None else ""
            sys.exit(f"读写失败：{e}{log_hint}")
        finally:
            if proc is not None:
                try:
                    ocd.command("shutdown")
                except (ConnectionError, OSError):
                    pass
                try:
                    proc.wait(timeout=3)
                except subprocess.TimeoutExpired:
                    proc.terminate()
        print(f"样本存在：{save(samples, meta)}")
    if len(samples) < 1000:
        sys.exit(f"只采到 {len(samples)} 个样本，太少了。")
    analyze(elf, samples, meta, args.timeline)


if __name__ == "__main__":
    main()
