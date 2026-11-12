#!/usr/bin/env python3
"""用 RS05 自己的 CSP 位置模式把 yaw、pitch 两个电机定住（比云台的串级 PID 硬得多），看结构晃不晃。

做法：OpenOCD 暂停 C 板 CPU（固件在无力时每 1 ms 给两个电机各发一帧停止帧，不停的话位置帧马上被顶掉），
直接写 CAN1 的发送邮箱发 RS05 私有协议的扩展帧，从接收 FIFO0 读电机的反馈；结束时两个电机都停止，再让 CPU 接着跑。
CSP 的参数和使能顺序照果园机器人工程（2026-10 实机用过）：停止 → run_mode 5 → 限流 → 限速 → 目标 → 使能
→ 再写一遍限流 / 限速 / 目标（电机默认限流可能很大，使能前写的不一定收下）；运行中约每 50 ms 重发目标。

用法（程序在跑、云台在无力（模式 0）；没有 OpenOCD 在跑就自己起一个，用完关掉）。路径写全：
    python C:/Project/RM_Infantry/Test_Module/Gimbal/Gimbal_test_RS05/tools/csp_hold.py
        两个电机定在现在的位置，pitch 再以限速慢慢抬到水平（中值），定 60 s；结束时 pitch 放回原位再停止
    csp_hold.py --axes pitch           只定 pitch（yaw 不碰，停着）；--axes yaw 只定 yaw
    csp_hold.py --seconds 120 --no-level --current 2.4 --speed 1.0
- 要人动手：在终端里跑，看到“现在可以动手了”再动，看到“停，别碰”就松手（2026-11-12 第一次没给信号，用户指出）。
- 使能前先读出这几个轴 RS05 自己的 loc_kp / spd_kp / spd_ki（类型 17 读参数）打印出来。
- 每收到一帧电机回话记一行，存 csv（%TEMP%/pid_step/）：抖的话看得出频率和幅度（第一次每 2 s 打一行，抖动看不到）。
- 暂停期间 IMU、加热 PID、和底盘板的通信都停着（加热 PWM 停在最后的占空比，恒温时约 13~25%），底盘板会判板间断线。
- 电机的 canTimeout 写成 1 s：脚本被打断、没来得及发停止时，电机 1 s 后自己进 Reset（松开）。
- 结束后云台留在无力（模式 0，顺便解掉锁）；下次进闭环时固件的使能序列会把 run_mode 写回 3（电流模式）、canTimeout 写回 0.1 s。
- 力矩按反馈帧 ±5.5 N·m 的量程换算（主机模型用的数），只看大小和方向。
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

CAN1 = 0x40006400
CAN_TSR = CAN1 + 0x08
CAN_RF0R = CAN1 + 0x0C
CAN_TX = CAN1 + 0x180           # 发送邮箱 0；邮箱 n 在 +0x10·n：TIR、TDTR、TDLR、TDHR
CAN_RX0 = CAN1 + 0x1B0          # FIFO0 的输出邮箱：RIR、RDTR、RDLR、RDHR
DBGMCU_APB1_FZ = 0xE0042008
DBG_CAN1_STOP = 1 << 25

HOST_ID = 0xFD
TYPE_FEEDBACK, TYPE_ENABLE, TYPE_STOP, TYPE_WRITE = 2, 3, 4, 18
IDX_RUN_MODE, IDX_LOC_REF, IDX_LIMIT_SPD, IDX_LIMIT_CUR, IDX_CAN_TIMEOUT = 0x7005, 0x7016, 0x7017, 0x7018, 0x7028
TYPE_READ = 17
GAIN_INDEXES = (("loc_kp", 0x701E), ("spd_kp", 0x701F), ("spd_ki", 0x7020))
RUN_MODE_CSP = 5
CAN_TIMEOUT_1S = 20000          # 20000 为 1 s（rs05.c）
POS_MAX, VEL_MAX, TORQUE_MAX = 12.57, 50.0, 5.5
ALL_MOTORS = (("yaw", 1), ("pitch", 2))     # gimbal_config.h 的 CAN ID
MOTORS = ALL_MOTORS                         # main 里按 --axes 改


class Bus:
    """C 板暂停时直接操作 bxCAN 寄存器。"""

    def __init__(self, ocd):
        self.ocd = ocd
        self.latest = {}
        self.params = {}
        self.log = []
        self.t0 = time.perf_counter()

    def read32(self, address):
        return self.ocd.read(address, 32, 1)[0]

    def write32(self, address, value):
        self.ocd.write(address, 32, [value & 0xFFFFFFFF])

    def send(self, frame_type, motor_id, data):
        ext_id = (frame_type << 24) | (HOST_ID << 8) | motor_id
        deadline = time.perf_counter() + 0.2
        while True:
            tsr = self.read32(CAN_TSR)
            free = [n for n in range(3) if tsr & (1 << (26 + n))]
            if free:
                break
            if time.perf_counter() > deadline:
                raise RuntimeError(f"CAN1 三个发送邮箱 0.2 s 都没空（总线上没人应答？TSR=0x{tsr:08x}）")
        n = free[0]
        box = CAN_TX + 0x10 * n
        self.write32(box + 4, 8)
        self.write32(box + 8, data[0] | data[1] << 8 | data[2] << 16 | data[3] << 24)
        self.write32(box + 12, data[4] | data[5] << 8 | data[6] << 16 | data[7] << 24)
        self.write32(box, (ext_id << 3) | 0x4 | 0x1)        # 扩展帧（IDE）+ 请求发送（TXRQ）
        while True:
            tsr = self.read32(CAN_TSR)
            if tsr & (1 << (26 + n)):
                if not tsr & (1 << (8 * n + 1)):           # TXOKn
                    raise RuntimeError(f"帧没发成功（类型 {frame_type}、电机 {motor_id}，TSR=0x{tsr:08x}）")
                return
            if time.perf_counter() > deadline:
                raise RuntimeError(f"帧 0.2 s 没发出去（类型 {frame_type}、电机 {motor_id}，没有应答？）")

    def write_param(self, motor_id, index, value_bytes):
        self.send(TYPE_WRITE, motor_id, [index & 0xFF, index >> 8, 0, 0] + list(value_bytes))

    def write_u8(self, motor_id, index, value):
        self.write_param(motor_id, index, [value & 0xFF, 0, 0, 0])

    def write_u32(self, motor_id, index, value):
        self.write_param(motor_id, index, struct.pack("<I", value))

    def write_f32(self, motor_id, index, value):
        self.write_param(motor_id, index, struct.pack("<f", value))

    def drain(self):
        """把 FIFO0 里的帧都读出来，记下每个电机最新的反馈。"""
        for _ in range(8):
            if self.read32(CAN_RF0R) & 0x3 == 0:
                return
            rir, rdtr, rdlr, rdhr = self.ocd.read(CAN_RX0, 32, 4)
            self.write32(CAN_RF0R, 0x20)                   # RFOM0：放掉这一帧
            if not rir & 0x4 or (rdtr & 0xF) != 8:
                continue
            ext_id = rir >> 3
            data = struct.pack("<II", rdlr, rdhr)
            if (ext_id >> 24) & 0x1F == TYPE_READ:          # 读参数的回话：Byte0~1 index、Byte4~7 值
                self.params[((ext_id >> 8) & 0xFF, data[0] | data[1] << 8)] = data[4:8]
                continue
            if (ext_id >> 24) & 0x1F != TYPE_FEEDBACK:
                continue

            def scaled(i, top):
                return ((data[i] << 8) | data[i + 1]) * (2.0 * top / 65535.0) - top

            self.latest[(ext_id >> 8) & 0xFF] = {
                "state": (ext_id >> 22) & 0x3, "fault": (ext_id >> 16) & 0x3F, "pos": scaled(0, POS_MAX),
                "vel": scaled(2, VEL_MAX), "torque": scaled(4, TORQUE_MAX),
                "temp": ((data[6] << 8) | data[7]) / 10.0, "time": time.perf_counter()}
            fb = self.latest[(ext_id >> 8) & 0xFF]
            self.log.append((fb["time"] - self.t0, (ext_id >> 8) & 0xFF, fb["state"], fb["fault"], fb["pos"],
                             fb["vel"], fb["torque"]))

    def read_f32(self, motor_id, index):
        """类型 17 读一个 float 参数；没回话返回 None。"""
        self.drain()
        self.params.pop((motor_id, index), None)
        self.send(TYPE_READ, motor_id, [index & 0xFF, index >> 8, 0, 0, 0, 0, 0, 0])
        for _ in range(5):
            time.sleep(0.005)
            self.drain()
            raw = self.params.get((motor_id, index))
            if raw is not None:
                return struct.unpack("<f", raw)[0]
        return None


def nearest(target, current):
    """target 加减整圈，取离 current 最近的那个（CSP 在多圈坐标上走，不能让它绕一圈）。"""
    return current + math.remainder(target - current, 2.0 * math.pi)


def main():
    parser = argparse.ArgumentParser(description="RS05 CSP 定住 yaw、pitch，看结构晃不晃")
    parser.add_argument("--seconds", type=float, default=60.0)
    parser.add_argument("--axes", default="yaw,pitch", help="定哪几个：yaw、pitch 或 yaw,pitch")
    parser.add_argument("--no-level", action="store_true", help="pitch 不抬到水平，定在原位")
    parser.add_argument("--current", type=float, default=2.4, help="CSP 限流 A（RS05 额定 2.4）")
    parser.add_argument("--speed", type=float, default=1.0, help="CSP 限速 rad/s（输出轴）")
    parser.add_argument("--elf", default=gw.DEFAULT_ELF)
    parser.add_argument("--openocd", default=gw.DEFAULT_OPENOCD)
    parser.add_argument("--cfg", default=gw.DEFAULT_CFG)
    parser.add_argument("--port", type=int, default=6666)
    args = parser.parse_args()
    sys.stdout.reconfigure(encoding="utf-8", errors="replace", line_buffering=True)
    global MOTORS
    wanted = {a.strip() for a in args.axes.split(",") if a.strip()}
    if not wanted or not wanted <= {"yaw", "pitch"}:
        sys.exit("--axes 只能是 yaw、pitch 或 yaw,pitch")
    MOTORS = tuple(m for m in ALL_MOTORS if m[0] in wanted)

    ocd, proc = gw.connect(args)
    try:
        run(ocd, args)
    finally:
        if proc is not None:
            try:
                ocd.command("shutdown")
            except (ConnectionError, OSError):
                pass
            proc.wait(timeout=5)


def run(ocd, args):
    symbols = gw.find_symbols(args.elf)
    s = gw.read_state(ocd, symbols)
    problems = gw.layout_problems(s)
    if problems:
        sys.exit("ELF 和板子里的程序对不上：\n  " + "\n  ".join(problems))
    if s["mode"] != 0:
        sys.exit(f"云台要在无力（实际模式 0）时用（现在 {s['mode']}）：先 gimbal_watch.py set mode 0")
    start = {}
    for name, motor_id in MOTORS:
        a = s if name == "yaw" else s["pitch"]
        age = gw.tick_age(s, a["fb_tick"])
        if a["fb_tick"] == 0 or age > 50 or a["fault"] != 0 or a["motor_id"] != motor_id:
            sys.exit(f"{name} 电机不对劲：反馈 {age} ms 前、fault 0x{a['fault']:02X}、ID {a['motor_id']}（应为 {motor_id}）")
        if abs(a["pos"]) > POS_MAX - 0.5:
            sys.exit(f"{name} 位置 {a['pos']:.3f} rad 太靠近 ±4π，CSP 目标会出范围")
        start[name] = a["pos"]
    level = None if (args.no_level or "pitch" not in start) else nearest(s["pitch"]["zero_rad"], start["pitch"])
    print("开始：" + "、".join(f"{name} {pos:+.4f} rad" for name, pos in start.items())
          + ("" if level is None else f"；pitch 之后抬到水平 {level:+.4f} rad"
                                       f"（{math.degrees(level - start['pitch']):+.1f}°）"))

    ocd.command("halt")
    time.sleep(0.05)
    state = ocd.command("[target current] curstate").strip()
    if state != "halted":
        ocd.command("resume")
        sys.exit(f"CPU 没停住（{state}），什么都没发")
    bus = Bus(ocd)
    targets = dict(start)
    try:
        fz = bus.read32(DBGMCU_APB1_FZ)
        if fz & DBG_CAN1_STOP:
            bus.write32(DBGMCU_APB1_FZ, fz & ~DBG_CAN1_STOP)
            print("DBGMCU 里 CAN1 设成了暂停时冻结，已改成不冻结")
        bus.drain()
        for name, motor_id in MOTORS:
            bus.send(TYPE_STOP, motor_id, [0] * 8)
        for name, motor_id in MOTORS:
            values = [(key, bus.read_f32(motor_id, index)) for key, index in GAIN_INDEXES]
            print(f"{name} 电机自己的增益：" + "，".join(f"{k} " + ("没回话" if v is None else f"{v:g}") for k, v in values))
        for name, motor_id in MOTORS:
            bus.write_u8(motor_id, IDX_RUN_MODE, RUN_MODE_CSP)
            bus.write_u32(motor_id, IDX_CAN_TIMEOUT, CAN_TIMEOUT_1S)
            bus.write_f32(motor_id, IDX_LIMIT_CUR, args.current)
            bus.write_f32(motor_id, IDX_LIMIT_SPD, args.speed)
            bus.write_f32(motor_id, IDX_LOC_REF, targets[name])
        for name, motor_id in MOTORS:
            bus.send(TYPE_ENABLE, motor_id, [0] * 8)
        for name, motor_id in MOTORS:
            bus.write_f32(motor_id, IDX_LIMIT_CUR, args.current)
            bus.write_f32(motor_id, IDX_LIMIT_SPD, args.speed)
            bus.write_f32(motor_id, IDX_LOC_REF, targets[name])
        # FIFO0 只有 3 格，满了以后新帧覆盖最后一格，里面还留着最早那几帧（停止时的回话）。
        # 所以一个一个查：清空 → 补发一帧目标 → 读它这一帧回话
        time.sleep(0.02)
        for name, motor_id in MOTORS:
            bus.drain()
            bus.latest.pop(motor_id, None)
            bus.write_f32(motor_id, IDX_LOC_REF, targets[name])
            time.sleep(0.01)
            bus.drain()
            fb = bus.latest.get(motor_id)
            if fb is None or fb["state"] != 2 or fb["fault"] != 0 or abs(fb["pos"] - targets[name]) > 0.05:
                raise RuntimeError(f"{name} 使能后不对：{fb}（要 state 2、没故障、位置在目标 ±0.05 rad 里）")
        names = "、".join(name for name, _ in MOTORS)
        print(f"{names} 在 CSP 里定住了（限流 {args.current:g} A、限速 {args.speed:g} rad/s）")
        if level is not None:
            targets["pitch"] = level
            print("pitch 正在抬到水平，先别碰……", flush=True)
            hold(bus, targets, abs(level - start["pitch"]) / args.speed + 0.8, quiet=True)
        bar = "=" * 60
        print(f"\n{bar}\n  现在可以动手了：扳 {names}，共 {args.seconds:g} s\n{bar}", flush=True)
        hold(bus, targets, args.seconds)
        back = "pitch 正在放回原位，" if level is not None else ""
        print(f"\n{bar}\n  停，别碰！{back}马上松开电机\n{bar}", flush=True)
        time.sleep(1.5)
        if level is not None:
            targets["pitch"] = start["pitch"]
            hold(bus, targets, abs(level - start["pitch"]) / args.speed + 0.8, quiet=True)
    finally:
        for name, motor_id in MOTORS:
            try:
                bus.send(TYPE_STOP, motor_id, [0] * 8)
            except RuntimeError as error:
                print(f"!! {name} 停止帧没发出去：{error}（canTimeout 1 s 后电机自己会松开）")
        bus.drain()
        bus.write32(CAN_RF0R, 0x18)                         # 清 FULL0、FOVR0（暂停期间 FIFO 一直在溢出）
        ocd.command("resume")
        time.sleep(0.1)
        ocd.write(symbols["gimbal_test"][0] + gw.TEST_MODE_OFFSET, 8, [0])
        print("电机已停止（松开），CPU 继续跑，云台指令设成无力（解锁）。")
        save_log(bus)


def hold(bus, targets, seconds, quiet=False):
    t0 = time.perf_counter()
    next_print = 0.0
    while time.perf_counter() - t0 < seconds:
        for name, motor_id in MOTORS:
            bus.write_f32(motor_id, IDX_LOC_REF, targets[name])
        bus.drain()
        t = time.perf_counter() - t0
        for name, motor_id in MOTORS:
            fb = bus.latest.get(motor_id)
            if fb is not None and (fb["state"] != 2 or fb["fault"] != 0):
                raise RuntimeError(f"{name} 不在运行了：state {fb['state']}、fault 0x{fb['fault']:02X}")
        if not quiet and t >= next_print:
            cells = [f"还剩 {max(0.0, seconds - t):3.0f} s"]
            for name, motor_id in MOTORS:
                fb = bus.latest.get(motor_id)
                if fb is None:
                    cells.append(f"{name} 没反馈")
                    continue
                cells.append(f"{name} 离目标 {math.degrees(fb['pos'] - targets[name]):+6.2f}° 力矩 {fb['torque']:+5.2f} N·m "
                             f"{fb['temp']:.0f}℃")
            print("  " + " | ".join(cells), flush=True)
            next_print += 5.0


def save_log(bus):
    if not bus.log:
        return
    path = os.path.join(tempfile.gettempdir(), "pid_step", time.strftime("%H%M%S") + "_csp_hold.csv")
    os.makedirs(os.path.dirname(path), exist_ok=True)
    with open(path, "w", newline="", encoding="utf-8") as f:
        w = csv.writer(f)
        w.writerow(["t_s", "motor_id", "state", "fault", "pos_rad", "vel_rad_s", "torque_nm"])
        for row in bus.log:
            w.writerow([f"{row[0]:.4f}", row[1], row[2], row[3], f"{row[4]:.5f}", f"{row[5]:.3f}", f"{row[6]:.3f}"])
    print(f"电机回话 {len(bus.log)} 帧，csv：{path}")


if __name__ == "__main__":
    main()
