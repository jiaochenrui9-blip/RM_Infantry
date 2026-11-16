#!/usr/bin/env python3
"""Gimbal_test_RS05 发射机构的读写脚本：通过 OpenOCD（DAPLink）不停核读写 shoot 模块的变量。

只看发射机构（C 板 CAN2：摩擦轮 M3508 ID 1、2，拨弹轮 M2006 ID 3）。云台的 RS05、IMU 不接也行：
云台掉线只锁云台自己，发射模块不看它们。

用法（程序已经烧进板子在跑）。在哪个目录下运行都行，路径写全：
    python C:/Project/RM_Infantry/Test_Module/Gimbal/Gimbal_test_RS05/tools/shoot_watch.py
下面简写成 shoot_watch.py：
    shoot_watch.py                  一直读，每 0.2 s 打一行，Ctrl+C 退出（一行约 140 个字，窗口拉宽一点）
    shoot_watch.py --once           读一次，打全部字段；有问题时给出该查什么
    shoot_watch.py stop             无力（shoot_test.mode = 0），也用来解除故障锁
    shoot_watch.py spin 3000        摩擦轮目标 3000 转子 rpm，进 SPINUP（mode 1）。从无力进来要输入 y 确认
    shoot_watch.py fire 1000        拨弹轮目标 1000 转子 rpm，进 FEED（mode 2）。摩擦轮就绪以后拨弹轮才转
    shoot_watch.py set mode 1       只写 shoot_test.mode：0 RELAX / 1 SPINUP / 2 FEED
    shoot_watch.py set fric 3000    只写 shoot_test.friction_rpm；set feed 1000 只写 feeder_rpm（模式不变）
    shoot_watch.py set fric_kp 5    改两个摩擦轮速度环的 Kp；fric_ki 同理。拨弹轮：feed_kp、feed_ki、feed_pos_kp（位置环）
                                    改 Ki 时先处理积分（原来 Ki 是 0 就清零，不是 0 就按比例缩放），I 项不跳。
                                    板子复位后回到 shoot_config.h 里的值，调好了记得改进去

- 边看边改：一直读的时候，直接在同一个窗口里输入命令再回车（不写前面的 python 和路径），还有 once（打全部字段）。
  也可以另开一个终端运行完整的命令，它会连上一直读的那个起的 OpenOCD（127.0.0.1:6666）。
  在 PowerShell 提示符下直接敲 set mode 1 不行（PowerShell 把 set 当成 Set-Variable），要在一直读的窗口里输入。
- 第一次闭环以前先在 RELAX 下手转核对方向：摩擦轮往外送弹的方向转、拨弹轮往进弹的方向转时，对应的 rpm 要是正的。
  方向错的轮子一闭环就会越转越快，一直顶到电流限幅。错了改 shoot_config.h 的 SHOOT_*_DIRECTION，重新编译烧录。
- 连接 OpenOCD、读写内存用的是同目录 gimbal_watch.py 里的代码；它在跑时这个脚本连它起的 OpenOCD。
  脚本自己起的 OpenOCD 只连不复位、不停核，日志在 %TEMP%\\shoot_watch_openocd.log。
- DAPLink 同时只能给一个 OpenOCD 用：CLion 的调试会话开着时这里起不来，先停掉。脚本开着时 CLion 也烧不了，先 Ctrl+C。
- 变量地址用 arm-none-eabi-nm 从 ELF 里查，ELF 要和板子里的程序是同一次编译的。开头核对变量大小和几个一定成立的数
  （电调 ID 1/2/3、方向 ±1、控制帧 0x200），对不上就报错退出，什么都不写。
- 结构体布局写死在下面（2026-11-16 用 arm-none-eabi-gdb 的 ptype /o 核对过）：dji_motor.h 的 DJI_Motor_t、
  shoot.h 的 ShootCmd_t、PID.h 的 PID_t。改了要跟着改（大小变了脚本会报错）。
"""

import argparse
import math
import os
import queue
import re
import shutil
import socket
import struct
import subprocess
import sys
import tempfile
import threading
import time

HERE = os.path.dirname(os.path.abspath(__file__))
sys.path.insert(0, HERE)
from gimbal_watch import OpenOcd, StartupError, spawn_openocd   # noqa: E402  连接部分和云台脚本共用

PROJECT_DIR = os.path.dirname(HERE)
DEFAULT_ELF = os.path.join(PROJECT_DIR, "build", "Debug", "Gimbal_test_RS05.elf")
DEFAULT_OPENOCD = r"D:\DevEnv\DevEnv\DevEnv\openocd-v0.12.0-i686-w64-mingw32\bin\openocd.exe"
DEFAULT_CFG = r"D:\OpenOCD_Config\DAPLink\stm32f4.cfg"
CAN2_MCR = 0x40006800           # bit4 NART（1 = 不自动重发）、bit6 ABOM（1 = 自动离线恢复）
CAN2_ESR = 0x40006818           # bit2 BOFF、bit6~4 LEC、bit23~16 TEC、bit31~24 REC

# DJI_Motor_t（192 字节）：type, mode, esc_id, feedback_id, tx_id, tx_slot, enable, reduction_ratio, current_limit,
# motor_direction, encoder, speed_rpm, feedback_current, temperature, last_encoder, total_encoder,
# speed{target_rpm, pid}, position{target_deg, pid, zero_encoder, last_update_tick, reference_ready},
# target_current, output_current, feedback_updated, feedback_count, last_feedback_tick
MOTOR_FORMAT = "<BBBxHHBBxxfhbxHhhBxhxxif60sf60siIBxxxhhBxxxII"
MOTOR_SPEED_PID_OFFSET = 40
MOTOR_POS_PID_OFFSET = 104
PID_FORMAT = "<14fB3x"
PID_FIELDS = ("Kp", "Ki", "Kd", "target", "actual", "last_actual", "error0", "error1", "integral", "out",
              "outMax", "outMin", "integralMax", "integralMin", "initialized")
PID_KI_OFFSET = 4
PID_INTEGRAL_OFFSET = 32
# ShootCmd_t（6 字节）：mode（短枚举 1 字节）, friction_rpm, feeder_rpm
CMD_FORMAT = "<Bxhh"
CMD_MODE_OFFSET = 0
CMD_FRIC_OFFSET = 2
CMD_FEED_OFFSET = 4

MOTOR_SIZE = struct.calcsize(MOTOR_FORMAT)
SYMBOLS = {
    "shoot_test": struct.calcsize(CMD_FORMAT),
    "shoot_cmd": struct.calcsize(CMD_FORMAT),
    "shoot_mode": 1,
    "shoot_fault": 1,
    "shoot_friction_1": MOTOR_SIZE,
    "shoot_friction_2": MOTOR_SIZE,
    "shoot_feeder": MOTOR_SIZE,
    "friction_last_not_ready_tick": 4,
    "feeder_jam_last_tick": 4,
    "feeder_jam_count": 1,
    "feeder_jam_state": 1,
    "uwTick": 4,
}
MOTORS = (("F1", "shoot_friction_1"), ("F2", "shoot_friction_2"), ("FD", "shoot_feeder"))

CONSTANTS = {
    "ready_err": "SHOOT_FRICTION_READY_ERR_RPM",
    "ready_hold": "SHOOT_FRICTION_READY_HOLD_MS",
    "offline": "SHOOT_OFFLINE_TIMEOUT_MS",
    "jam_current": "SHOOT_JAM_CURRENT",
    "jam_max": "SHOOT_JAM_MAX_COUNT",
    "fric_limit": "SHOOT_FRICTION_CURRENT_LIMIT",
    "feed_limit": "SHOOT_FEEDER_CURRENT_LIMIT",
}

MODE_NAMES = {0: "RELAX", 1: "SPINUP", 2: "FEED"}
MOTOR_MODE_NAMES = {0: "DISABLE", 1: "CURRENT", 2: "SPEED", 3: "POSITION"}
MOTOR_MODE_SHORT = {0: "D", 1: "C", 2: "S", 3: "P"}
JAM_NAMES = {0: "NORMAL", 1: "DETECT", 2: "REVERSE"}
JAM_SHORT = {0: "N", 1: "D", 2: "R"}
TYPE_NAMES = {0: "M3508", 1: "M2006", 2: "GM6020"}
LEC_NAMES = ["无错误", "填充错误", "格式错误", "没有应答", "隐性位错误", "显性位错误", "CRC 错误", "（软件设置）"]
POSITIVE = {"F1": "往外送弹", "F2": "往外送弹", "FD": "进弹"}

# 命令里的数的范围：M3508 空载转子约 9000 rpm、M2006 约 18000 rpm（手册量级，留余量）
FRIC_RPM_MAX = 9000
FEED_RPM_MAX = 10000


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
        sys.exit(f"ELF 里没有这些变量：{', '.join(missing)}（是不是带发射模块的 Gimbal_test_RS05？是不是旧版本？）")
    wrong = [f"{name} {symbols[name][1]} 字节（脚本按 {size} 字节读）"
             for name, size in SYMBOLS.items() if symbols[name][1] != size]
    if wrong:
        sys.exit("这些变量的大小和脚本里的布局对不上，结构体改过了，脚本开头的 *_FORMAT 要跟着改：\n  "
                 + "\n  ".join(wrong))
    return symbols


def read_constants():
    """从 shoot_config.h 读几个宏（去掉括号和 U / f 后缀），读不到的是 None。"""
    values = {key: None for key in CONSTANTS}
    try:
        with open(os.path.join(PROJECT_DIR, "App", "shoot", "shoot_config.h"), encoding="utf-8", errors="replace") as f:
            text = f.read()
    except OSError:
        return values
    for key, name in CONSTANTS.items():
        match = re.search(rf"^[ \t]*#define[ \t]+{name}[ \t]+\(?([-+]?[0-9.]+)[uUfF]*\)?", text, re.M)
        if match:
            values[key] = float(match.group(1))
    return values


def connect(args):
    """先连已经在跑的 OpenOCD（比如 gimbal_watch.py 起的）；没有就自己起一个。"""
    try:
        return OpenOcd(socket.create_connection(("127.0.0.1", args.port), timeout=1.0)), None
    except OSError:
        pass
    exe = args.openocd if os.path.exists(args.openocd) else shutil.which("openocd")
    if exe is None:
        sys.exit(f"找不到 OpenOCD：{args.openocd}（用 --openocd 指定）")
    log_path = os.path.join(tempfile.gettempdir(), "shoot_watch_openocd.log")
    try:
        return spawn_openocd(args, exe, log_path)
    except StartupError as first:
        # 上一个 OpenOCD 被半路打断时 DAPLink 里会留一个没收的应答，第一次报 command mismatch，再连一次一般就好
        if "command mismatch" not in str(first):
            sys.exit(f"{first}\n（DAPLink 没插好、板子没电，或者被 CLion 占着？）")
    try:
        return spawn_openocd(args, exe, log_path)
    except StartupError as second:
        sys.exit(f"{second}\n（连了两次都不行：拔插一下 DAPLink 再试）")


# ---------- 读 ----------

def unpack_pid(raw):
    return dict(zip(PID_FIELDS, struct.unpack(PID_FORMAT, raw)))


def unpack_motor(raw):
    v = struct.unpack(MOTOR_FORMAT, raw)
    m = {
        "type": v[0], "mode": v[1], "esc_id": v[2], "feedback_id": v[3], "tx_id": v[4], "tx_slot": v[5],
        "enable": v[6], "ratio": v[7], "current_limit": v[8], "direction": v[9], "encoder": v[10],
        "rpm": v[11], "fb_current": v[12], "temp": v[13], "total_encoder": v[15],
        "target_rpm": v[16], "speed_pid": unpack_pid(v[17]),
        "target_deg": v[18], "pos_pid": unpack_pid(v[19]), "zero_encoder": v[20], "ref_ready": v[22],
        "target_current": v[23], "output": v[24], "fb_count": v[26], "fb_tick": v[27],
    }
    # 和 MotorControl_GetDegree 一样：输出轴角度，相对最近一次进位置模式时的零点
    m["deg"] = ((m["total_encoder"] - m["zero_encoder"]) * 360.0 / (8192.0 * m["ratio"])
                if m["ref_ready"] and m["ratio"] > 0 else 0.0)
    return m


def read_state(ocd, symbols):
    tick = ocd.read(symbols["uwTick"][0], 32, 1)[0]
    s = {"tick": tick}
    for short, name in MOTORS:
        s[short] = unpack_motor(ocd.read_bytes(*symbols[name]))
    test = struct.unpack(CMD_FORMAT, ocd.read_bytes(*symbols["shoot_test"]))
    cmd = struct.unpack(CMD_FORMAT, ocd.read_bytes(*symbols["shoot_cmd"]))
    s.update({
        "test_mode": test[0], "test_fric": test[1], "test_feed": test[2],
        "cmd_mode": cmd[0], "cmd_fric": cmd[1], "cmd_feed": cmd[2],
        "mode": ocd.read(symbols["shoot_mode"][0], 8, 1)[0],
        "fault": ocd.read(symbols["shoot_fault"][0], 8, 1)[0],
        "not_ready_tick": ocd.read(symbols["friction_last_not_ready_tick"][0], 32, 1)[0],
        "jam_tick": ocd.read(symbols["feeder_jam_last_tick"][0], 32, 1)[0],
        "jam_count": ocd.read(symbols["feeder_jam_count"][0], 8, 1)[0],
        "jam_state": ocd.read(symbols["feeder_jam_state"][0], 8, 1)[0],
    })
    mcr = ocd.read(CAN2_MCR, 32, 1)[0]
    esr = ocd.read(CAN2_ESR, 32, 1)[0]
    s.update(nart=(mcr >> 4) & 1, abom=(mcr >> 6) & 1,
             tec=(esr >> 16) & 0xFF, rec=(esr >> 24) & 0xFF, lec=(esr >> 4) & 0x7, boff=(esr >> 2) & 1)
    return s


def layout_problems(s):
    """ELF 不是板子里那次编译的时候地址是错的，这几条基本不可能同时成立。"""
    checks = []
    for (short, name), esc_id, kind in zip(MOTORS, (1, 2, 3), (0, 0, 1)):
        m = s[short]
        checks += [
            (m["esc_id"] == esc_id and m["feedback_id"] == 0x200 + esc_id,
             f"{name}.esc_id = {m['esc_id']}、feedback_id = 0x{m['feedback_id']:X}（应为 {esc_id}、0x{0x200 + esc_id:X}）"),
            (m["type"] == kind, f"{name}.type = {m['type']}（应为 {TYPE_NAMES[kind]}）"),
            (m["tx_id"] == 0x200 and m["tx_slot"] == esc_id - 1, f"{name} 控制帧 0x{m['tx_id']:X} 槽位 {m['tx_slot']}"),
            (m["direction"] in (-1, 1), f"{name}.motor_direction = {m['direction']}（应为 ±1）"),
            (m["current_limit"] > 0 and m["ratio"] > 0, f"{name} 限幅 {m['current_limit']}、减速比 {m['ratio']:g}"),
        ]
    checks.append((s["mode"] <= 2 and s["fault"] <= 1 and s["jam_state"] <= 2,
                   f"shoot_mode {s['mode']}、shoot_fault {s['fault']}、feeder_jam_state {s['jam_state']}"))
    return [text for ok, text in checks if not ok]


# ---------- 显示 ----------

def age(s, past):
    return max(0, s["tick"] - past)


def fb_age(s, m):
    if m["fb_count"] == 0:
        return "--"
    a = age(s, m["fb_tick"])
    return str(a) if a < 1000 else ">1s"


def ready_ms(s):
    """摩擦轮已经连续在误差带里多少毫秒（就绪计时）；RELAX 时没意义。"""
    if s["mode"] == 0:
        return "--"
    a = age(s, s["not_ready_tick"])
    return str(a) if a < 100000 else ">99s"


HEADER = (f"{'tick':>8} | cmd {'fric':>5} {'feed':>5} | md lk | "
          f"{'F1rpm':>5} {'cur':>6} {'fb':>3} | {'F2rpm':>5} {'cur':>6} {'fb':>3} | "
          f"m {'FDrpm':>6} {'tgt':>6} {'cur':>6} {'deg':>7} {'fb':>3} j n | {'rdy':>5} | TEC LEC")


def print_line(s, header):
    if header:
        print(HEADER)
    f1, f2, fd = s["F1"], s["F2"], s["FD"]
    print(f"{s['tick']:>8} | {s['test_mode']:>3} {s['test_fric']:>5} {s['test_feed']:>5} | "
          f"{s['mode']:>2} {s['fault']:>2} | "
          f"{f1['rpm']:>5} {f1['output']:>6} {fb_age(s, f1):>3} | {f2['rpm']:>5} {f2['output']:>6} {fb_age(s, f2):>3} | "
          f"{MOTOR_MODE_SHORT.get(fd['mode'], '?')} {fd['rpm']:>6} {fd['target_rpm']:>6.0f} {fd['output']:>6} "
          f"{fd['deg']:>+7.1f} {fb_age(s, fd):>3} {JAM_SHORT.get(s['jam_state'], '?')} {s['jam_count']} | "
          f"{ready_ms(s):>5} | {s['tec']:>3} {s['lec']:>3}" + ("  BUS-OFF" if s["boff"] else ""), flush=True)


def hints(s, const):
    out = []
    offline = const["offline"] or 50
    silent = [short for short, _ in MOTORS
              if s[short]["fb_count"] == 0 or age(s, s[short]["fb_tick"]) > offline]
    if silent:
        who = "、".join(silent)
        if s["tec"] >= 128 or s["boff"] or s["lec"] in (4, 5):
            out.append(f"{who} 没有反馈，CAN2 发不出去（TEC 大 / 位错误）：C 板要接 24 V 或插 USB（只接 DAPLink 时"
                       "收发器没有 5 V）；查 CAN2 的线（4-pin 口：1 5V、2 GND、3 CANH、4 CANL）。")
        elif s["lec"] == 3:
            out.append(f"{who} 没有反馈，发出去的 0x200 没人应答：电调有没有电（24 V）、CANH / CANL 有没有接反、"
                       "是不是接到了 CAN1。")
        else:
            out.append(f"{who} 没有反馈，但总线上有人应答：查电调 ID（摩擦轮 1、2，拨弹轮 3），"
                       "C620 / C610 的灯是不是在闪对应的次数。")
    if s["fault"]:
        max_count = const["jam_max"]
        if max_count is not None and s["jam_count"] >= max_count:
            out.append(f"锁住了：连续卡弹 {s['jam_count']} 次。清掉拨盘里卡住的弹丸，再 stop（mode 0）解锁。")
        else:
            out.append("锁住了：有电机掉过线（任一电机 "
                       f"{offline:.0f} ms 没反馈）。电机都回话以后 stop（mode 0）解锁。")
    elif s["test_mode"] == 2 and s["mode"] == 1:
        err = const["ready_err"]
        band = f"（误差带 ±{err:.0f} rpm）" if err is not None else ""
        out.append(f"指令是 FEED，实际是 SPINUP：摩擦轮还没就绪{band}，或者 friction_rpm ≤ 0。"
                   "看 F1rpm、F2rpm 是不是到了目标、rdy 有没有涨到就绪时间。")
    if s["mode"] == 0 and not s["fault"]:
        out.append("RELAX 下手转核对方向：摩擦轮往外送弹的方向转、拨弹轮往进弹的方向转时，F1rpm / F2rpm / FDrpm 要是正的。"
                   "错了改 shoot_config.h 的 SHOOT_*_DIRECTION 再烧。")
    if s["nart"] or not s["abom"]:
        out.append("CAN2 没开自动重发 / 自动离线恢复（.ioc 里已经开了，can.c 还没重新生成）：CubeMX 里点一次 GENERATE CODE"
                   " 再编译烧录。不影响先试，只是 bus-off 以后要复位 C 板。")
    for short, _ in MOTORS:
        m = s[short]
        if m["temp"] >= 70:
            out.append(f"{short} 电调报的温度 {m['temp']} ℃，偏高。")
    return out


def print_motor(s, short, name):
    m = s[short]
    sp, pp = m["speed_pid"], m["pos_pid"]
    print(f"{short}（{name}，{TYPE_NAMES.get(m['type'], '?')} ID {m['esc_id']}）：电机层模式 {MOTOR_MODE_NAMES.get(m['mode'], '?')}，"
          f"方向 {m['direction']:+d}（逻辑正向 = {POSITIVE[short]}），减速比 {m['ratio']:g}，电流限幅 {m['current_limit']}")
    print(f"  反馈 {fb_age(s, m)} ms 前（共 {m['fb_count']} 帧）：speed_rpm {m['rpm']:+d}（转子，已按方向修正），"
          f"电调电流 {m['fb_current']:+d}，温度 {m['temp']} ℃，编码器 {m['encoder']}")
    print(f"  速度环 Kp {sp['Kp']:g} Ki {sp['Ki']:g} Kd {sp['Kd']:g}：target_rpm {m['target_rpm']:+.0f}  "
          f"I 项 {sp['Ki'] * sp['integral']:+.0f}  输出 output_current {m['output']:+d}")
    if short == "FD":
        print(f"  位置环 Kp {pp['Kp']:g} Ki {pp['Ki']:g} Kd {pp['Kd']:g}（限速 ±{pp['outMax']:g} 转子 rpm）："
              f"target_deg {m['target_deg']:+.1f}，当前 {m['deg']:+.1f}°（输出轴，相对最近一次进位置模式的位置"
              f"{'' if m['ref_ready'] else '，还没进过位置模式'}）")


def print_full(s, const):
    print(f"HAL tick {s['tick']} ms；CAN2 TEC {s['tec']}、REC {s['rec']}、最近一次错误：{LEC_NAMES[s['lec']]}"
          + ("，BUS-OFF" if s["boff"] else "")
          + f"；自动重发 {'关' if s['nart'] else '开'}、自动离线恢复 {'开' if s['abom'] else '关'}")
    print(f"调试指令 shoot_test：mode {s['test_mode']} {MODE_NAMES.get(s['test_mode'], '（不认识）')}，"
          f"friction_rpm {s['test_fric']}，feeder_rpm {s['test_feed']}"
          f"（模块收到的 shoot_cmd：{s['cmd_mode']} / {s['cmd_fric']} / {s['cmd_feed']}）")
    hold = const["ready_hold"]
    print(f"实际模式 {s['mode']} {MODE_NAMES.get(s['mode'], '?')}，"
          + ("锁住了（shoot_fault 1）" if s["fault"] else "没锁")
          + f"；摩擦轮就绪计时 {ready_ms(s)} ms" + (f"（≥ {hold:.0f} 算就绪）" if hold is not None else ""))
    print(f"卡弹：状态 {JAM_NAMES.get(s['jam_state'], '?')}，连续 {s['jam_count']} 次，"
          f"进入当前状态 {age(s, s['jam_tick'])} ms 前")
    for short, name in MOTORS:
        print_motor(s, short, name)
    for text in hints(s, const):
        print(f"提示：{text}")


# ---------- 写 ----------

# set 能写的：名字 → (变量列表, 偏移, 类型)
WRITABLE = {
    "mode": (("shoot_test",), CMD_MODE_OFFSET, "u8"),
    "fric": (("shoot_test",), CMD_FRIC_OFFSET, "s16"),
    "feed": (("shoot_test",), CMD_FEED_OFFSET, "s16"),
    "fric_kp": (("shoot_friction_1", "shoot_friction_2"), MOTOR_SPEED_PID_OFFSET, "f32"),
    "fric_ki": (("shoot_friction_1", "shoot_friction_2"), MOTOR_SPEED_PID_OFFSET + PID_KI_OFFSET, "f32"),
    "feed_kp": (("shoot_feeder",), MOTOR_SPEED_PID_OFFSET, "f32"),
    "feed_ki": (("shoot_feeder",), MOTOR_SPEED_PID_OFFSET + PID_KI_OFFSET, "f32"),
    "feed_pos_kp": (("shoot_feeder",), MOTOR_POS_PID_OFFSET, "f32"),
}
WRITABLE_NAMES = {"mode": "shoot_test.mode", "fric": "shoot_test.friction_rpm", "feed": "shoot_test.feeder_rpm",
                  "fric_kp": "两个摩擦轮 speed.pid.Kp", "fric_ki": "两个摩擦轮 speed.pid.Ki",
                  "feed_kp": "拨弹轮 speed.pid.Kp", "feed_ki": "拨弹轮 speed.pid.Ki",
                  "feed_pos_kp": "拨弹轮 position.pid.Kp"}

USAGE = ("命令：stop、spin <摩擦轮 rpm>、fire <拨弹 rpm>、set mode <0~2>、set fric|feed <rpm>、"
         "set fric_kp|fric_ki|feed_kp|feed_ki|feed_pos_kp <值>、once（一直读时）")


def parse_rpm(name, text):
    try:
        value = int(text)
    except ValueError:
        raise ValueError(f"{name} 要是整数（转子 rpm）：{text}") from None
    limit = FRIC_RPM_MAX if name == "fric" else FEED_RPM_MAX
    if abs(value) > limit:
        raise ValueError(f"{name} 超出 ±{limit} 转子 rpm：{value}")
    return value


def parse_command(words):
    """返回要依次写的 [(名字, 值), ...]；不对时抛 ValueError。"""
    if words == ["stop"]:
        return [("mode", 0)]
    if len(words) == 2 and words[0] == "spin":
        return [("fric", parse_rpm("fric", words[1])), ("mode", 1)]
    if len(words) == 2 and words[0] == "fire":
        return [("feed", parse_rpm("feed", words[1])), ("mode", 2)]
    if len(words) != 3 or words[0] != "set" or words[1] not in WRITABLE:
        raise ValueError(USAGE)
    name, text = words[1], words[2]
    if name == "mode":
        if text not in ("0", "1", "2"):
            raise ValueError("mode 是 0~2：0 RELAX / 1 SPINUP / 2 FEED")
        return [(name, int(text))]
    if name in ("fric", "feed"):
        return [(name, parse_rpm(name, text))]
    try:
        value = float(text)
    except ValueError:
        raise ValueError(f"{name} 要是一个数：{text}") from None
    if not math.isfinite(value) or value < 0:
        raise ValueError(f"{name} 要是一个不小于 0 的有限的数：{text}（方向由 SHOOT_*_DIRECTION 管）")
    return [(name, value)]


def read_stdin_line():
    line = sys.stdin.readline()
    return line if line else None


def needs_confirm(writes, s):
    """从无力（指令 mode 0）切到 1 / 2 时要确认。"""
    return s is not None and s["test_mode"] == 0 and any(name == "mode" and value != 0 for name, value in writes)


def confirm_start(s, const):
    limit = const["fric_limit"]
    print("要让发射机构闭环：摩擦轮开始转（SPINUP），FEED 时拨弹轮也转。\n"
          "  1. 方向核对过了吗？没核对先 stop，RELAX 下手转看 F1rpm / F2rpm / FDrpm 的符号（往外送弹 / 进弹为正）。\n"
          "     方向错的轮子一闭环就越转越快，一直顶到电流限幅"
          + (f"（摩擦轮 {limit:.0f}）" if limit is not None else "") + "。\n"
          "  2. 手、线、工具离开摩擦轮和拨盘；枪口别对着人。\n"
          "  要停：stop（立刻无力，摩擦轮会滑行一会儿才停）。")
    if s["fault"]:
        print("注意：现在锁着（shoot_fault 1），先 stop 解锁，这次切了也还是 RELAX。")
    print("确定？输入 y 回车：", end="", flush=True)
    answer = read_stdin_line()
    return answer is not None and answer.strip().lower() == "y"


def f32_word(value):
    return struct.unpack("<I", struct.pack("<f", value))[0]


def write_one(ocd, symbols, s, name, value):
    names, offset, kind = WRITABLE[name]
    for symbol in names:
        base = symbols[symbol][0]
        if kind == "u8":
            ocd.write(base + offset, 8, [value])
        elif kind == "s16":
            ocd.write(base + offset, 16, [value & 0xFFFF])
        else:
            if name.endswith("_ki"):
                # Ki = 0 时积分照样在攒，直接改 Ki 会让 I 项一下跳到很大：原来是 0（或改成 0）就清零，否则按比例缩放
                pid = s[{"shoot_friction_1": "F1", "shoot_friction_2": "F2", "shoot_feeder": "FD"}[symbol]]["speed_pid"]
                integral = 0.0 if (pid["Ki"] == 0.0 or value == 0.0) else pid["integral"] * pid["Ki"] / value
                ocd.write(base + MOTOR_SPEED_PID_OFFSET + PID_INTEGRAL_OFFSET, 32, [f32_word(integral)])
            ocd.write(base + offset, 32, [f32_word(value)])
    print(f"写进去了：{WRITABLE_NAMES[name]} = {value:g}")


def apply(ocd, symbols, s, const, writes, full=True):
    for name, value in writes:
        write_one(ocd, symbols, s, name, value)
    if any(name.endswith(("_kp", "_ki")) for name, _ in writes):
        print("注意：板子复位后回到 shoot_config.h 里的值，调好了记得改进去。")
    for name, value in writes:
        if name == "mode" and value == 2 and s["test_fric"] <= 0 and not any(n == "fric" for n, _ in writes):
            print("注意：friction_rpm 还是 ≤ 0，摩擦轮不会就绪，拨弹轮不会转：先 spin <rpm>。")
        if name == "mode" and value != 0 and s["fault"]:
            print("注意：现在锁着，先 stop 解锁。")
    if full:
        time.sleep(0.05)
        print()
        print_full(read_state(ocd, symbols), const)


def start_command_reader(commands, last):
    """另开一个线程读键盘，检查好放进队列，由主循环在两次读数之间执行。这个线程只碰键盘，不碰 OpenOCD。"""
    def run():
        while True:
            line = read_stdin_line()
            if line is None:
                return
            words = line.split()
            if not words:
                continue
            if words == ["once"]:
                commands.put("once")
                continue
            try:
                writes = parse_command(words)
            except ValueError as e:
                print(f"  （没执行：{e}）", flush=True)
                continue
            if needs_confirm(writes, last.get("s")) and not confirm_start(last["s"], last["const"]):
                print("  （没改）", flush=True)
                continue
            commands.put(writes)

    threading.Thread(target=run, daemon=True).start()


def main():
    parser = argparse.ArgumentParser(description="Gimbal_test_RS05 发射机构：OpenOCD 不停核读写 shoot 模块的变量")
    parser.add_argument("--elf", default=DEFAULT_ELF, help="烧进板子的那个 ELF（默认 build/Debug/Gimbal_test_RS05.elf）")
    parser.add_argument("--openocd", default=DEFAULT_OPENOCD, help="没有在跑的 OpenOCD 时用来起一个")
    parser.add_argument("--cfg", default=DEFAULT_CFG, help="OpenOCD 的板子配置")
    parser.add_argument("--port", type=int, default=6666, help="OpenOCD 的 TCL 端口")
    parser.add_argument("--period", type=float, default=0.2, help="一直读时每行间隔，秒")
    parser.add_argument("--once", action="store_true", help="读一次，打全部字段")
    parser.add_argument("command", nargs="*", help="stop、spin <rpm>、fire <rpm>、set mode|fric|feed|fric_kp|... <值>")
    sys.stdout.reconfigure(encoding="utf-8", errors="replace")
    sys.stderr.reconfigure(encoding="utf-8", errors="replace")
    args = parser.parse_args()

    writes = None
    if args.command:
        try:
            writes = parse_command(args.command)
        except ValueError as e:
            parser.error(str(e))
    symbols = find_symbols(args.elf)
    const = read_constants()
    ocd, proc = connect(args)
    try:
        state = read_state(ocd, symbols)
        problems = layout_problems(state)
        if problems:
            time.sleep(0.05)
            running = ocd.read(symbols["uwTick"][0], 32, 1)[0] != state["tick"]
            sys.exit("读到的数和 ELF 对不上，什么都没写：\n  " + "\n  ".join(problems) + "\n"
                     + ("ELF 和板子里的程序可能不是同一次编译的：重新烧一遍，或者用 --elf 指定烧进去的那个。" if running else
                        "uwTick 不涨，程序没在跑：可能卡在 Error_Handler 或 HardFault，也可能被调试器停住了。"))
        if writes is not None:
            if needs_confirm(writes, state) and not confirm_start(state, const):
                sys.exit("没改。")
            apply(ocd, symbols, state, const, writes)
        elif args.once:
            print_full(state, const)
        else:
            commands = queue.Queue()
            last = {"s": state, "const": const}
            start_command_reader(commands, last)
            print("一直读，Ctrl+C 退出。直接在这里输入命令再回车：stop、spin 3000、fire 1000、set fric_kp 5、once 等"
                  "（输入的字会和刷新的行混在一起，打完回车就行）", flush=True)
            print("列：cmd = shoot_test 的 mode / fric / feed；md lk = 实际模式、锁；cur = 速度环输出电流（原始值）；"
                  "fb = 反馈几 ms 前；m = 拨弹轮电机层模式 D/S/P；deg = 拨弹轮输出轴角度（位置模式里）；"
                  "j n = 卡弹状态 N/D/R、连续次数；rdy = 摩擦轮就绪计时 ms", flush=True)
            line = 0
            while True:
                state = read_state(ocd, symbols)
                last["s"] = state
                print_line(state, header=(line % 20 == 0))
                line += 1
                while not commands.empty():
                    item = commands.get()
                    if item == "once":
                        print_full(read_state(ocd, symbols), const)
                    else:
                        apply(ocd, symbols, read_state(ocd, symbols), const, item, full=False)
                    sys.stdout.flush()
                    line = 0
                time.sleep(args.period)
    except KeyboardInterrupt:
        pass
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
