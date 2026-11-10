#!/usr/bin/env python3
"""Gimbal_test_RS05 的读数脚本：通过 OpenOCD（DAPLink）不停核读写云台、ins 的变量。

现在是单电机阶段（只有 yaw，C 板拿在手里），只看 yaw；装上 pitch 以后再加 pitch 的列。

用法（程序已经烧进板子在跑）。在哪个目录下运行都行，路径写全：
    python C:/Project/RM_Infantry/Test_Module/Gimbal/Gimbal_test_RS05/tools/gimbal_watch.py
下面简写成 gimbal_watch.py：
    gimbal_watch.py                     一直读，每 0.2 s 打一行，Ctrl+C 退出
    gimbal_watch.py --once              读一次，打全部字段；有问题时给出该查什么
    gimbal_watch.py set mode 3          写 gimbal_test.gimbal_mode：0 无力 / 1 编码器 / 2 IMU / 3 yaw 跟着 C 板转
    gimbal_watch.py set yaw 30          写 gimbal_test.yaw_deg（度）
    gimbal_watch.py add yaw 5           gimbal_test.yaw_deg 加 5°
    gimbal_watch.py set speed_kp 0.016  改 yaw 速度环的 Kp（板子复位后回到 gimbal_config.h 里的值）
    gimbal_watch.py set speed_ki 0.2    改 yaw 速度环的 Ki（同上）。PID 在 Ki = 0 时积分照样在攒，直接改会让 I 项一下顶到
                                        限幅，所以脚本先处理积分：原来 Ki 是 0 就清零，不是 0 就按比例缩放，I 项不跳
    gimbal_watch.py set angle_kp 12     改 yaw 角度环的 Kp（同上）

- 边看边改：一直读的时候，直接在同一个窗口里输入命令再回车，比如 set mode 1、add yaw 5、set speed_kp 0.002
  （和上面一样，只是不写前面的 python 和路径），还有 once（打全部字段）。输入的字会和刷新的行混在一起，打完回车就行。
  也可以另开一个终端运行完整的命令，它会连上一直读的那个起的 OpenOCD（127.0.0.1:6666）。
- 在 PowerShell 提示符下直接敲 set mode 1 不行：PowerShell 把 set 当成它自己的 Set-Variable，不报错、也不碰板子
  （2026-11-10 踩过）。要么在一直读的窗口里输入，要么写全 python 和脚本路径。
- 切模式和改目标要分两次：切模式后的第一个循环里，yaw_deg 会被同步成新模式下的当前角（gimbal_test_cmd.c），
  和切模式同时写进去的目标会被覆盖。
- 变量地址用 arm-none-eabi-nm 从 ELF 里查，ELF 要和板子里的程序是同一次编译的。开头先核对符号的大小，
  再核对几个一定成立的数（方向是 ±1、两个环的输出限幅正负对称、加热 PID 限在 0~1 等），对不上就报错退出，什么都不写。
- 先连 127.0.0.1:6666（OpenOCD 的 TCL 端口）；连不上就自己起一个 OpenOCD，退出时用 shutdown 关掉。
  脚本起的 OpenOCD 只连不复位、不停核。连接这部分和 RS05_test/tools/rs05_watch.py 一样，改了一边另一边也看一下。
- DAPLink 同时只能给一个 OpenOCD 用：CLion 的调试会话开着时这里起不来，先停掉调试。脚本开着时 CLion 也烧不了，先 Ctrl+C。
- 结构体的布局写死在下面，对应 gimbal_test_cmd.h 的 GimbalTest_t、gimbal.c 的 GimbalAxis_t、ins.h 的 InsFeedback_t、
  PID.h 的 PID_t、rs05.h 的 RS05_Motor_t；改了要跟着改（变量大小变了，脚本会报错）。
- 提示里用到的几个常数（标定样本数、到温超时等）从工程源码里读。
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
PROJECT_DIR = os.path.dirname(HERE)
DEFAULT_ELF = os.path.join(PROJECT_DIR, "build", "Debug", "Gimbal_test_RS05.elf")
DEFAULT_OPENOCD = r"D:\DevEnv\DevEnv\DevEnv\openocd-v0.12.0-i686-w64-mingw32\bin\openocd.exe"
DEFAULT_CFG = r"D:\OpenOCD_Config\DAPLink\stm32f4.cfg"
TCL_TERMINATOR = b"\x1a"
CAN1_ESR = 0x40006418           # bxCAN 错误状态寄存器：bit2 BOFF、bit6~4 LEC、bit23~16 TEC、bit31~24 REC

# 结构体布局（2026-11-09 用 arm-none-eabi-gdb 的 ptype /o 核对过）。arm-none-eabi-gcc 默认是短枚举：
# GimbalMode_e、ChassisMode_e 都只占 1 字节，所以 gimbal_mode 要按 8 位写，按 32 位写会把后面 3 个字节一起改掉。
# GimbalTest_t：gimbal_mode, yaw_deg, pitch_deg, chassis_mode, vx_mps, vy_mps, yaw_square_deg, pitch_square_deg, square_period_ms
TEST_FORMAT = "<BxxxffBxxxffffI"
TEST_MODE_OFFSET = 0
TEST_YAW_OFFSET = 4
# GimbalAxis_t：motor, direction, zero_rad, min_deg, max_deg, angle_pid, speed_pid, angle_deg, rate_dps, target_deg
AXIS_FORMAT = "<28sbxxxfff60s60sfff"
AXIS_ANGLE_PID_OFFSET = 44
AXIS_SPEED_PID_OFFSET = 104
# RS05_Motor_t：id, enable, current_a, enable_step, position_rad, velocity_rad_s, state, fault, feedback_tick
MOTOR_FORMAT = "<BBxxfBxxxffBBxxI"
# PID_t：14 个 float，再加 initialized
PID_FORMAT = "<14fB3x"
PID_FIELDS = ("Kp", "Ki", "Kd", "target", "actual", "last_actual", "error0", "error1", "integral", "out",
              "outMax", "outMin", "integralMax", "integralMin", "initialized")
# InsFeedback_t：valid, yaw_deg, pitch_deg, yaw_rate_dps, pitch_rate_dps
INS_FORMAT = "<Bxxxffff"

# 要读的变量和它们的大小。nm -S 查到的大小对不上，说明结构体改过了，上面的布局要跟着改
SYMBOLS = {
    "gimbal_test": struct.calcsize(TEST_FORMAT),
    "gimbal_yaw": struct.calcsize(AXIS_FORMAT),
    "gimbal_mode": 1,
    "gimbal_fault": 1,
    "ins_feedback": struct.calcsize(INS_FORMAT),
    "ins_bmi088_temp_pid": struct.calcsize(PID_FORMAT),
    "ins_calibrate_count": 4,
    "ins_last_tick": 4,
    "ins_init_status": 1,
    "uwTick": 4,
}

# 提示里用的常数：在哪个源文件里、宏叫什么
CONSTANTS = {
    "samples": ("App/ins/ins.c", "INS_CALIBRATE_SAMPLES"),
    "band": ("App/ins/ins.c", "INS_TEMP_BAND_C"),
    "heat_timeout": ("App/ins/ins_config.h", "INS_HEAT_TIMEOUT_MS"),
    "offline": ("App/gimbal/gimbal.c", "GIMBAL_OFFLINE_TIMEOUT_MS"),
}

# set / add 能写的量：名字 → (变量, 偏移, 类型)。PID_t 里 Kp 偏移 0、Ki 偏移 4、integral 偏移 32
PID_KI_OFFSET = 4
PID_INTEGRAL_OFFSET = 32
WRITABLE = {
    "mode": ("gimbal_test", TEST_MODE_OFFSET, "u8"),
    "yaw": ("gimbal_test", TEST_YAW_OFFSET, "f32"),
    "speed_kp": ("gimbal_yaw", AXIS_SPEED_PID_OFFSET, "f32"),
    "speed_ki": ("gimbal_yaw", AXIS_SPEED_PID_OFFSET + PID_KI_OFFSET, "f32"),
    "angle_kp": ("gimbal_yaw", AXIS_ANGLE_PID_OFFSET, "f32"),
}
WRITABLE_NAMES = {"mode": "gimbal_test.gimbal_mode", "yaw": "gimbal_test.yaw_deg",
                  "speed_kp": "gimbal_yaw.speed_pid.Kp", "speed_ki": "gimbal_yaw.speed_pid.Ki",
                  "angle_kp": "gimbal_yaw.angle_pid.Kp"}

MODE_NAMES = {0: "无力", 1: "编码器闭环", 2: "IMU 闭环", 3: "yaw 跟着 C 板转（测试用，发给云台的是编码器闭环）"}
BMI088_NAMES = {0: "正常", 1: "加速度计 ID 不对", 2: "陀螺仪 ID 不对", 3: "空指针", 4: "SPI 出错",
                5: "配置回读不对", 6: "还没初始化", 7: "数据不对", 8: "忙"}
STATE_NAMES = {0: "Reset（停止）", 1: "Cali（标定）", 2: "Motor（运行）"}
FAULT_BITS = ["欠压", "三相电流", "过温", "磁编码", "堵转过载", "未标定"]
LEC_NAMES = ["无错误", "填充错误", "格式错误", "没有应答", "隐性位错误", "显性位错误", "CRC 错误", "（软件设置）"]


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
        sys.exit(f"ELF 里没有这些变量：{', '.join(missing)}（是不是 Gimbal_test_RS05 的 ELF？是不是旧版本？）")
    wrong = [f"{name} {symbols[name][1]} 字节（脚本按 {size} 字节读）"
             for name, size in SYMBOLS.items() if symbols[name][1] != size]
    if wrong:
        sys.exit("这些变量的大小和脚本里的布局对不上，结构体改过了，脚本开头的 *_FORMAT 要跟着改：\n  "
                 + "\n  ".join(wrong))
    return symbols


def read_constants():
    """从工程源码里读提示要用的几个宏（去掉括号和 U / f 后缀），读不到的是 None，提示里就不写那个数。"""
    values = {}
    for key, (path, name) in CONSTANTS.items():
        values[key] = None
        try:
            with open(os.path.join(PROJECT_DIR, path), encoding="utf-8", errors="replace") as f:
                text = f.read()
        except OSError:
            continue
        match = re.search(rf"^[ \t]*#define[ \t]+{name}[ \t]+\(?([-+]?[0-9.]+)[uUfF]*\)?", text, re.M)
        if match:
            values[key] = float(match.group(1))
    return values


class OpenOcd:
    """OpenOCD 的 TCL 端口：发一条命令加 0x1a，回一个结果加 0x1a。"""

    def __init__(self, sock):
        self.sock = sock
        self.sock.settimeout(3.0)

    def command(self, text):
        self.sock.sendall(text.encode() + TCL_TERMINATOR)
        reply = b""
        while not reply.endswith(TCL_TERMINATOR):
            chunk = self.sock.recv(4096)
            if not chunk:
                raise ConnectionError("OpenOCD 断开了")
            reply += chunk
        return reply[:-1].decode(errors="replace").strip()

    def _checked(self, tcl):
        # catch 住错误，失败时结果以 ERROR: 开头，和正常结果区分开。
        # 很多错误 OpenOCD 只写进它自己的日志，这里拿到的原因是空的
        reply = self.command(f"if {{[catch {{{tcl}}} v]}} {{set v \"ERROR: $v\"}} else {{set v}}")
        if reply.startswith("ERROR:"):
            reason = reply[6:].strip() or "OpenOCD 没给原因，看它的日志"
            raise RuntimeError(f"{tcl}：{reason}")
        return reply

    # OpenOCD 0.12.0 只有 target 下的 read_memory / write_memory（stm32f4x.cpu read_memory ...），没有全局的
    def read(self, address, width, count):
        reply = self._checked(f"[target current] read_memory 0x{address:08x} {width} {count}")
        values = [int(v, 0) for v in reply.split()]
        if len(values) != count:
            raise RuntimeError(f"read_memory 0x{address:08x} 只回了 {len(values)} 个值：{reply}")
        return values

    def read_bytes(self, address, size):
        words = self.read(address, 32, (size + 3) // 4)
        return struct.pack(f"<{len(words)}I", *words)[:size]

    def write(self, address, width, values):
        self._checked(f"[target current] write_memory 0x{address:08x} {width} {{{' '.join(hex(v) for v in values)}}}")


class StartupError(Exception):
    pass


def spawn_openocd(args, exe, log_path):
    """起一个 OpenOCD，等它连上板子。失败时抛 StartupError（带日志最后几行）。"""
    log = open(log_path, "w", encoding="utf-8", errors="replace")
    # 只要 TCL 端口；gdb、telnet 端口关掉，免得和别的 OpenOCD 抢 3333 / 4444。
    # 单独一个进程组：控制台里按 Ctrl+C 只到脚本，由脚本发 shutdown 让 OpenOCD 正常退出。
    # 不这样的话 OpenOCD 会跟着被打断，DAPLink 里留下没收的应答，下一次连会报 "CMSIS-DAP command mismatch"
    flags = subprocess.CREATE_NEW_PROCESS_GROUP if os.name == "nt" else 0
    proc = subprocess.Popen([exe, "-f", args.cfg, "-c", f"tcl_port {args.port}",
                             "-c", "gdb_port disabled", "-c", "telnet_port disabled"],
                            stdout=log, stderr=subprocess.STDOUT, creationflags=flags)
    proc.log_path = log_path

    def fail(why):
        if proc.poll() is None:
            proc.terminate()
            proc.wait(timeout=3)
        log.close()
        with open(log_path, encoding="utf-8", errors="replace") as f:
            raise StartupError(f"{why}。日志 {log_path} 最后几行：\n{f.read()[-1500:]}")

    deadline = time.time() + 10.0
    while True:
        if proc.poll() is not None:
            fail("OpenOCD 退出了")
        try:
            ocd = OpenOcd(socket.create_connection(("127.0.0.1", args.port), timeout=1.0))
            break
        except OSError:
            if time.time() > deadline:
                fail("OpenOCD 10 s 内没开 TCL 端口")
            time.sleep(0.2)
    # OpenOCD 先开端口、后连 DAPLink（init），init 做完才处理命令：第一条命令多等一会儿
    try:
        ocd.sock.settimeout(15.0)
        ocd.command("target current")
        ocd.sock.settimeout(3.0)
    except (ConnectionError, OSError):
        fail("OpenOCD 没连上板子")
    return ocd, proc


def connect(args):
    """先连已经在跑的 OpenOCD；没有就自己起一个，返回 (OpenOcd, 自己起的进程或 None)。"""
    try:
        return OpenOcd(socket.create_connection(("127.0.0.1", args.port), timeout=1.0)), None
    except OSError:
        pass

    exe = args.openocd if os.path.exists(args.openocd) else shutil.which("openocd")
    if exe is None:
        sys.exit(f"找不到 OpenOCD：{args.openocd}（用 --openocd 指定）")
    log_path = os.path.join(tempfile.gettempdir(), "gimbal_watch_openocd.log")
    try:
        return spawn_openocd(args, exe, log_path)
    except StartupError as first:
        # 上一个 OpenOCD 被半路打断时，DAPLink 里会留一个没收的应答，第一次连会报 command mismatch，
        # 这一次把它收掉了，再连一次一般就好
        if "command mismatch" not in str(first):
            sys.exit(f"{first}\n（DAPLink 没插好、板子没电，或者被 CLion 占着？）")
    try:
        return spawn_openocd(args, exe, log_path)
    except StartupError as second:
        sys.exit(f"{second}\n（连了两次都不行：拔插一下 DAPLink 再试）")


# ---------- 读 ----------

def unpack_pid(raw):
    return dict(zip(PID_FIELDS, struct.unpack(PID_FORMAT, raw)))


def read_state(ocd, symbols):
    # 先读 tick，紧接着读电机（在 gimbal_yaw 里）和 ins 的最后样本时间。经 DAPLink 每读一次要 1~2 ms，
    # 中间隔的读越多，算出来的“几毫秒前”就越大（2026-11-10 第一版最后才读 tick，正常时也显示 19~20 ms）。
    # 时间戳比 tick 新时按 0 算（tick_age）
    tick = ocd.read(symbols["uwTick"][0], 32, 1)[0]
    axis = struct.unpack(AXIS_FORMAT, ocd.read_bytes(*symbols["gimbal_yaw"]))
    ins_last = ocd.read(symbols["ins_last_tick"][0], 32, 1)[0]
    motor = struct.unpack(MOTOR_FORMAT, axis[0])
    test = struct.unpack(TEST_FORMAT, ocd.read_bytes(*symbols["gimbal_test"]))
    ins = struct.unpack(INS_FORMAT, ocd.read_bytes(*symbols["ins_feedback"]))
    s = {
        "direction": axis[1], "zero_rad": axis[2], "min_deg": axis[3], "max_deg": axis[4],
        "angle_pid": unpack_pid(axis[5]), "speed_pid": unpack_pid(axis[6]),
        "angle": axis[7], "rate": axis[8], "target": axis[9],
        "motor_id": motor[0], "enable": motor[1], "iq": motor[2], "enable_step": motor[3],
        "pos": motor[4], "vel": motor[5], "state": motor[6], "fault": motor[7], "fb_tick": motor[8],
        "cmd_mode": test[0], "yaw_cmd": test[1],
        "valid": ins[0], "imu_yaw": ins[1], "imu_pitch": ins[2], "imu_yaw_rate": ins[3], "imu_pitch_rate": ins[4],
        "temp_pid": unpack_pid(ocd.read_bytes(*symbols["ins_bmi088_temp_pid"])),
        "mode": ocd.read(symbols["gimbal_mode"][0], 8, 1)[0],
        "lock": ocd.read(symbols["gimbal_fault"][0], 8, 1)[0],
        "ins_status": ocd.read(symbols["ins_init_status"][0], 8, 1)[0],
        "cal": ocd.read(symbols["ins_calibrate_count"][0], 32, 1)[0],
        "ins_last": ins_last, "tick": tick,
    }
    esr = ocd.read(CAN1_ESR, 32, 1)[0]
    s.update(tec=(esr >> 16) & 0xFF, rec=(esr >> 24) & 0xFF, lec=(esr >> 4) & 0x7, boff=(esr >> 2) & 1)
    # 发给电机的是 direction × 电流（gimbal.c 的 Gimbal_AxisOutput），再乘一次方向就是按云台正方向（俯视逆时针为正）的电流
    s["current"] = s["direction"] * s["iq"]
    return s


def layout_problems(s):
    """ELF 不是板子里那次编译的时候地址是错的，这几条基本不可能同时成立。返回不成立的那几条。"""
    angle, speed, temp = s["angle_pid"], s["speed_pid"], s["temp_pid"]
    checks = [
        (s["direction"] in (-1, 1), f"gimbal_yaw.direction = {s['direction']}（应为 1 或 -1）"),
        (s["motor_id"] <= 127, f"gimbal_yaw.motor.id = {s['motor_id']}（应在 0~127）"),
        (s["min_deg"] < s["max_deg"], f"yaw 限位 {s['min_deg']:g} ~ {s['max_deg']:g}"),
        (angle["outMax"] > 0 and angle["outMin"] == -angle["outMax"],
         f"角度环输出限幅 {angle['outMin']:g} ~ {angle['outMax']:g}（应正负对称）"),
        (speed["outMax"] > 0 and speed["outMin"] == -speed["outMax"],
         f"速度环输出限幅 {speed['outMin']:g} ~ {speed['outMax']:g}（应正负对称）"),
        ((temp["outMin"], temp["outMax"], temp["integralMin"], temp["integralMax"]) == (0.0, 1.0, 0.0, 1.0),
         f"加热 PID 限幅 {temp['outMin']:g} ~ {temp['outMax']:g}（应为 0 ~ 1）"),
        (s["mode"] <= 2 and s["lock"] <= 1, f"gimbal_mode = {s['mode']}、gimbal_fault = {s['lock']}"),
    ]
    return [text for ok, text in checks if not ok]


# ---------- 显示 ----------

def tick_age(s, past_tick):
    return max(0, s["tick"] - past_tick)


def feedback_age(s):
    if s["fb_tick"] == 0:
        return "--"
    age = tick_age(s, s["fb_tick"])
    return str(age) if age < 1000 else ">1s"


HEADER = (f"{'tick':>9} | {'T_C':>5} {'duty':>4} {'cal':>5} v {'imu_yaw':>8} | cmd md lk {'yaw_deg':>8} | "
          f"{'angle':>8} {'target':>8} {'I_A':>6} | {'fb':>3} st flt | TEC LEC")


def print_line(s, header):
    temp = s["temp_pid"]
    if header:
        print(HEADER)
    print(f"{s['tick']:>9} | {temp['actual']:5.1f} {temp['out'] * 100:3.0f}% {s['cal']:>5} {s['valid']} "
          f"{s['imu_yaw']:+8.2f} | {s['cmd_mode']:>3} {s['mode']:>2} {s['lock']:>2} {s['yaw_cmd']:+8.2f} | "
          f"{s['angle']:+8.2f} {s['target']:+8.2f} {s['current']:+6.3f} | "
          f"{feedback_age(s):>3} {s['state']:>2}  {s['fault']:02X} | {s['tec']:>3} {s['lec']:>3}"
          + ("  BUS-OFF" if s["boff"] else ""), flush=True)


def hints(s, const):
    """按读到的状态给出该查什么、该怎么做。"""
    out = []
    temp = s["temp_pid"]
    samples, band, heat_timeout, offline = const["samples"], const["band"], const["heat_timeout"], const["offline"]

    if s["ins_status"] != 0:
        out.append(f"BMI088 初始化失败（{BMI088_NAMES.get(s['ins_status'], s['ins_status'])}），IMU 用不了："
                   "模式 3 下电机停在原地，模式 2 进不去。复位一次，还不行查 SPI。")
    elif not s["valid"]:
        if samples is not None and s["cal"] >= samples:
            out.append(f"ins 标定完了，但 valid 是 0：{tick_age(s, s['ins_last'])} ms 没有新样本（SPI / DMA 出错？）")
        elif (s["cal"] == 0 and band is not None and abs(temp["actual"] - temp["target"]) > band
              and (heat_timeout is None or s["tick"] < heat_timeout)):
            wait = "" if heat_timeout is None else f"；只插 USB 时加热没电，要等到上电 {heat_timeout / 1000:.0f} s 才不管温度直接标定"
            out.append(f"ins 在等温度：{temp['actual']:.2f} ℃，目标 {temp['target']:.1f} ℃，进到 ±{band:g} ℃ 以内才开始标定"
                       f"（要接 24 V 才加热{wait}）。C 板别动。")
        else:
            total = "" if samples is None else f" / {samples:.0f}"
            out.append(f"ins 在标定：{s['cal']}{total}，C 板别动。")

    limit = 100 if offline is None else offline
    if s["fb_tick"] == 0 or tick_age(s, s["fb_tick"]) > limit:
        if s["tec"] >= 128 or s["boff"] or s["lec"] in (4, 5):
            out.append("yaw 电机没回话，CAN1 发不出去（TEC 大 / 位错误）：查 C 板有没有 24 V 或 USB"
                       "（只接 DAPLink 时收发器没有 5 V）、CAN 线。")
        elif s["lec"] == 3:
            out.append("yaw 电机没回话，发出去的帧没人应答：查电机有没有电、CANH / CANL 有没有接反。")
        else:
            out.append(f"yaw 电机没回话，但帧发出去了、有人应答：查电机 ID（程序里是 {s['motor_id']}，"
                       "gimbal_config.h 的 GIMBAL_YAW_CAN_ID）、是不是在私有协议（RS05_test 的 scan 能查）。"
                       "和 C 板一起上电时，RS05 约 1.4 s 内不回话。")
    if s["fault"]:
        out.append("yaw 电机报故障，算掉线，云台会锁住。这一版不清故障，给电机断电重启。")
    if s["lock"]:
        out.append("云台锁在 RELAX：闭环时 yaw 电机掉过线，或者模式 2 时 IMU 无效。先 set mode 0，再切回去。")
    if s["cmd_mode"] == 2:
        out.append("模式 2 用 C 板的角度做反馈，C 板要装在电机转盘上；C 板拿在手里时电机会一直加速，马上 set mode 0。")
    if s["cmd_mode"] == 3 and not s["valid"]:
        out.append("模式 3：C 板 yaw 还没标定好，电机先停在原地。")
    if s["cmd_mode"] > 3:
        out.append(f"gimbal_mode = {s['cmd_mode']} 不是 0~3，云台当无力处理。")
    return out


def print_full(s, const):
    temp = s["temp_pid"]
    total = "" if const["samples"] is None else f" / {const['samples']:.0f}"
    print(f"HAL tick {s['tick']} ms；CAN1 TEC {s['tec']}、REC {s['rec']}、最近一次错误：{LEC_NAMES[s['lec']]}"
          + ("，BUS-OFF" if s["boff"] else ""))
    print(f"ins：BMI088 初始化 {s['ins_status']}（{BMI088_NAMES.get(s['ins_status'], '?')}）；"
          f"温度 {temp['actual']:.2f} ℃（目标 {temp['target']:.1f}），加热占空比 {temp['out'] * 100:.0f}%；"
          f"标定 {s['cal']}{total}，valid {s['valid']}；最后一个样本 {tick_age(s, s['ins_last'])} ms 前")
    print(f"  C 板 yaw {s['imu_yaw']:+.2f}°  pitch {s['imu_pitch']:+.2f}°  "
          f"yaw 角速度 {s['imu_yaw_rate']:+.2f} °/s  pitch 角速度 {s['imu_pitch_rate']:+.2f} °/s")
    print(f"指令（gimbal_test）：gimbal_mode {s['cmd_mode']} {MODE_NAMES.get(s['cmd_mode'], '（不认识，云台当无力）')}；"
          f"yaw_deg {s['yaw_cmd']:+.2f}" + ("（模式 3 里是 电机角 − C 板 yaw）" if s["cmd_mode"] == 3 else ""))
    print(f"云台：实际模式 {s['mode']} {MODE_NAMES.get(s['mode'], '?')}，"
          + ("锁住了（gimbal_fault 1）" if s["lock"] else "没锁"))
    print(f"yaw 轴：angle_deg {s['angle']:+.2f}  rate_dps {s['rate']:+.2f}  target_deg {s['target']:+.2f}"
          f"（方向 {s['direction']:+d}，中值 {s['zero_rad']:.4f} rad，限位 {s['min_deg']:g} ~ {s['max_deg']:g}）")
    for name, pid, unit in (("角度环", s["angle_pid"], "°/s"), ("速度环（target 含前馈）", s["speed_pid"], "A")):
        print(f"  {name} Kp {pid['Kp']:g} Ki {pid['Ki']:g} Kd {pid['Kd']:g}：target {pid['target']:+.3f}  "
              f"actual {pid['actual']:+.3f}  out {pid['out']:+.4f} {unit}（限幅 ±{pid['outMax']:g}）")
    print(f"  发给电机的电流：{s['current']:+.3f} A（俯视逆时针为正；按电机自己的方向是 {s['iq']:+.3f} A）")
    faults = [name for bit, name in enumerate(FAULT_BITS) if s["fault"] & (1 << bit)]
    print(f"yaw 电机（RS05 ID {s['motor_id']}）：state {s['state']} {STATE_NAMES.get(s['state'], '?')}，"
          f"fault 0x{s['fault']:02X} {'、'.join(faults) if faults else '无'}，反馈 {feedback_age(s)} ms 前"
          f"{'（还没收到过）' if s['fb_tick'] == 0 else ''}，enable {s['enable']}，使能序列下一步 {s['enable_step']}")
    print(f"  position_rad {s['pos']:+.4f}   velocity_rad_s {s['vel']:+.3f}")
    for text in hints(s, const):
        print(f"提示：{text}")


# ---------- 写 ----------

USAGE = "命令：set mode <0~3>、set yaw <度>、add yaw <度>、set speed_kp <值>、set speed_ki <值>、set angle_kp <值>"


def parse_command(command):
    """检查命令（拆好的几个词），返回 (名字, 是不是加, 值)；不对时抛 ValueError，带说明。"""
    if len(command) != 3 or command[0] not in ("set", "add") or command[1] not in WRITABLE:
        raise ValueError(USAGE)
    action, name, text = command
    if action == "add" and name != "yaw":
        raise ValueError("add 只能用在 yaw 上：add yaw <度>")
    if name == "mode":
        if text not in ("0", "1", "2", "3"):
            raise ValueError("mode 是 0~3：0 无力 / 1 编码器闭环 / 2 IMU 闭环 / 3 yaw 跟着 C 板转")
        return name, False, int(text)
    try:
        value = float(text)
    except ValueError:
        raise ValueError(f"{name} 要是一个数：{text}") from None
    if not math.isfinite(value):
        raise ValueError(f"{name} 要是一个有限的数：{text}")
    if name.endswith(("_kp", "_ki")) and value < 0:
        raise ValueError("增益不能是负的（方向由 gimbal_config.h 的 GIMBAL_YAW_DIRECTION 管）")
    return name, action == "add", value


def read_stdin_line():
    """读一行键盘输入；没有输入了（输入被重定向且读完）返回 None。"""
    line = sys.stdin.readline()
    return line if line else None


def confirm_mode2():
    print("模式 2 用 C 板的角度和角速度做反馈，C 板要装在电机转盘上、跟着电机一起转。\n"
          "C 板拿在手里时，电机转了 C 板测不到，误差消不掉，电机会一直加速。")
    print("确定切到模式 2？输入 y 回车：", end="", flush=True)
    answer = read_stdin_line()
    return answer is not None and answer.strip().lower() == "y"


def start_command_reader(commands):
    """一直读时另开一个线程读键盘：每行一条命令，检查好放进 commands 队列，由主循环在两次读数之间执行。
    这个线程只碰键盘，不碰 OpenOCD。"""
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
                command = parse_command(words)
            except ValueError as e:
                print(f"  （没执行：{e}；还能输入 once 打全部字段）", flush=True)
                continue
            if command == ("mode", False, 2) and not confirm_mode2():
                print("  （没改）", flush=True)
                continue
            commands.put(command)

    threading.Thread(target=run, daemon=True).start()


def write_value(ocd, symbols, s, const, name, add, value, full=True):
    symbol, offset, kind = WRITABLE[name]
    address = symbols[symbol][0] + offset
    if name == "speed_ki":
        # PID 在 Ki = 0 时积分照样在攒（I 项乘出来是 0，不限幅），直接改 Ki 会让 I 项一下顶到限幅、电机猛甩。
        # 原来 Ki 是 0（或改成 0）就把积分清零；都不是 0 就按比例缩放，Ki × 积分不变。先写积分、再写 Ki
        old_ki, integral = s["speed_pid"]["Ki"], s["speed_pid"]["integral"]
        integral = 0.0 if (old_ki == 0.0 or value == 0.0) else integral * old_ki / value
        ocd.write(symbols[symbol][0] + AXIS_SPEED_PID_OFFSET + PID_INTEGRAL_OFFSET, 32,
                  [struct.unpack("<I", struct.pack("<f", integral))[0]])
    if kind == "u8":
        ocd.write(address, 8, [value])
    else:
        if add:
            value += s["yaw_cmd"]
        ocd.write(address, 32, [struct.unpack("<I", struct.pack("<f", value))[0]])
    print(f"写进去了：{WRITABLE_NAMES[name]} = {value:g}")
    if name == "yaw" and s["mode"] == 0:
        print("注意：云台现在是无力（实际模式 0），改了 yaw_deg 也不动；切模式时它会被同步成当前角，这次改的会被覆盖。")
    if name == "mode" and value == 3:
        print("注意：模式 3 要 C 板拿在手里；C 板装在电机上时别用，目标会跟着电机跑、一直转下去。")
    if name.endswith(("_kp", "_ki")):
        print("注意：板子复位后回到 gimbal_config.h 里的值，调好了记得改进去。")
    if full:
        # 等程序跑几个周期（切模式时目标同步、RS05 使能序列约 4 ms），再读一遍
        time.sleep(0.05)
        print()
        print_full(read_state(ocd, symbols), const)


def main():
    parser = argparse.ArgumentParser(description="Gimbal_test_RS05：OpenOCD 不停核读写云台、ins 的变量")
    parser.add_argument("--elf", default=DEFAULT_ELF, help="烧进板子的那个 ELF（默认 build/Debug/Gimbal_test_RS05.elf）")
    parser.add_argument("--openocd", default=DEFAULT_OPENOCD, help="没有在跑的 OpenOCD 时用来起一个")
    parser.add_argument("--cfg", default=DEFAULT_CFG, help="OpenOCD 的板子配置")
    parser.add_argument("--port", type=int, default=6666, help="OpenOCD 的 TCL 端口")
    parser.add_argument("--period", type=float, default=0.2, help="一直读时每行间隔，秒")
    parser.add_argument("--once", action="store_true", help="读一次，打全部字段")
    parser.add_argument("command", nargs="*", help="set mode|yaw|speed_kp|angle_kp <值>、add yaw <度>")
    # 固定 UTF-8：输出走管道（Git Bash 等）时 Python 默认用 GBK，中文会乱码。报错（sys.exit）走 stderr
    sys.stdout.reconfigure(encoding="utf-8", errors="replace")
    sys.stderr.reconfigure(encoding="utf-8", errors="replace")
    args = parser.parse_args()

    command = None
    if args.command:
        try:
            command = parse_command(args.command)
        except ValueError as e:
            parser.error(str(e))
    if command == ("mode", False, 2) and not confirm_mode2():
        sys.exit("没改。")
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
                        "uwTick 不涨，程序没在跑：可能卡在 Error_Handler（Robot_Init 里配置出错，比如两个电机 ID 一样）"
                        "或者 HardFault，也可能被调试器停住了。"))
        if command is not None:
            write_value(ocd, symbols, state, const, *command)
        elif args.once:
            print_full(state, const)
        else:
            commands = queue.Queue()
            start_command_reader(commands)
            print("一直读，Ctrl+C 退出。可以直接在这里输入命令再回车，比如 set mode 1、add yaw 5、once"
                  "（输入的字会和刷新的行混在一起，打完回车就行）", flush=True)
            line = 0
            while True:
                print_line(read_state(ocd, symbols), header=(line % 20 == 0))
                line += 1
                while not commands.empty():
                    command = commands.get()
                    if command == "once":
                        print_full(read_state(ocd, symbols), const)
                    else:
                        write_value(ocd, symbols, read_state(ocd, symbols), const, *command, full=False)
                    sys.stdout.flush()                  # 输出接到管道时也马上出来
                    line = 0                            # 下一行重新打表头
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
