#!/usr/bin/env python3
"""RS05_test 的读数脚本：通过 OpenOCD（DAPLink）不停核读写 App/robot.c 里的测试变量。

用法（程序已经烧进板子在跑）。在哪个目录下运行都行，路径写全：
    python C:/Project/RM_Infantry/Test_Module/Gimbal/RS05_test/tools/rs05_watch.py
下面简写成 rs05_watch.py：
    rs05_watch.py                  一直读，每 0.2 s 打一行，Ctrl+C 退出
    rs05_watch.py --once           读一次，打全部字段（参数表、最近收到的帧；没收到反馈时给出该查什么）
    rs05_watch.py set enable 1     写 rs05_test_enable：0 停止（没力），1 电流模式
    rs05_watch.py set current 0.1  写 rs05_test_current_a（A，程序里限到 ±1 A）
    rs05_watch.py set id 127       把程序用的电机 ID（rs05_motor.id）改成 127，不用重新烧
    rs05_watch.py scan             依次试私有协议、MIT、CANopen，找总线上的电机（几秒）
    rs05_watch.py log              一直打印新收到的帧（驱动、读参数用掉的除外），Ctrl+C 退出
    rs05_watch.py send std 0x7F FF FF FF FF FF FF FF FD    发一帧（std / ext，ID，0~8 个字节），再打印 0.2 s 内收到的帧

- 变量地址用 arm-none-eabi-nm 从 ELF 里查，ELF 要和板子里的程序是同一次编译的。
  开头会核对一遍参数表的 index，对不上就报错退出，什么都不写。
- 先连 127.0.0.1:6666（OpenOCD 的 TCL 端口）；连不上就自己起一个 OpenOCD（--openocd、--cfg），
  退出时用 shutdown 关掉。脚本起的 OpenOCD 只连不复位、不停核。
- DAPLink 同时只能给一个 OpenOCD 用：CLion 的调试会话开着时这里起不来，先停掉调试。
  脚本开着时 CLion 也烧不了，先 Ctrl+C。
- 结构体的布局写死在下面（和 rs05.h 的 RS05_Motor_t、robot.c 的 TestParam_t / TestFrame_t 对应），改了要跟着改。
"""

import argparse
import os
import shutil
import socket
import struct
import subprocess
import sys
import tempfile
import time

HERE = os.path.dirname(os.path.abspath(__file__))
PROJECT_DIR = os.path.dirname(HERE)
DEFAULT_ELF = os.path.join(PROJECT_DIR, "build", "Debug", "RS05_test.elf")
DEFAULT_OPENOCD = r"D:\DevEnv\DevEnv\DevEnv\openocd-v0.12.0-i686-w64-mingw32\bin\openocd.exe"
DEFAULT_CFG = r"D:\OpenOCD_Config\DAPLink\stm32f4.cfg"
TCL_TERMINATOR = b"\x1a"

MOTOR_ID_MAX = 127              # 电机 CAN ID 的范围 0~127
HOST_ID = 0xFD                  # robot.c / rs05.c 的本机 ID
CAN1_ESR = 0x40006418           # bxCAN 错误状态寄存器：bit2 BOFF、bit6~4 LEC、bit23~16 TEC、bit31~24 REC

# RS05_Motor_t（rs05.h）：id, enable, current_a, enable_step, position_rad, velocity_rad_s, state, fault, feedback_tick
MOTOR_FORMAT = "<BBxxfBxxxffBBxxI"
# TestParam_t（robot.c）：index, raw, ok_count, fail_count
PARAM_FORMAT = "<HxxIII"
# TestFrame_t（robot.c）：id（bit31 = 扩展帧）, dlc, data[8]，共 16 字节
FRAME_FORMAT = "<IB8s3x"
FRAME_EXT = 0x80000000

# rs05_test_params 里的 index → (名字, 一行里的短名, 类型)。类型按手册：0x70xx 见 4.1 节最后的参数列表，
# 0x20xx / 0x30xx 见 3.3.4 节上位机参数表。robot.c 加了 index 没加到这里的，按原始值显示
PARAMS = {
    0x7019: ("mechPos", "mechPos", "f32"),
    0x701A: ("iqf", "iqf", "f32"),
    0x7029: ("zero_sta", "zs", "u8"),
    0x3004: ("encoderRaw", "enc", "i16"),
    0x3015: ("modPos", "modPos", "f32"),
    0x3014: ("rotation", "rot", "i16"),
    0x2005: ("MechOffset", "offset", "f32"),
}

STATE_NAMES = {0: "Reset（停止）", 1: "Cali（标定）", 2: "Motor（运行）"}
FAULT_BITS = ["欠压", "三相电流", "过温", "磁编码", "堵转过载", "未标定"]
LEC_NAMES = ["无错误", "填充错误", "格式错误", "没有应答", "隐性位错误", "显性位错误", "CRC 错误", "（软件设置）"]
SYMBOLS = ["rs05_motor", "rs05_test_params", "rs05_test_enable", "rs05_test_current_a",
           "rs05_test_rx_log", "rs05_test_rx_count", "rs05_test_tx_frame", "rs05_test_tx_pending", "uwTick"]


def find_symbols(elf):
    nm = shutil.which("arm-none-eabi-nm")
    if nm is None:
        sys.exit("找不到 arm-none-eabi-nm（要在 PATH 里）")
    if not os.path.exists(elf):
        sys.exit(f"找不到 ELF：{elf}\n先编译，或者用 --elf 指定（CLion 的输出目录可能是 cmake-build-debug）")
    out = subprocess.run([nm, "-S", elf], capture_output=True, text=True, check=True).stdout
    symbols = {}
    for line in out.splitlines():
        parts = line.split()
        if len(parts) == 4 and parts[3] in SYMBOLS:
            symbols[parts[3]] = (int(parts[0], 16), int(parts[1], 16))
    missing = [name for name in SYMBOLS if name not in symbols]
    if missing:
        sys.exit(f"ELF 里没有这些符号：{', '.join(missing)}（是不是 RS05_test 的 ELF？是不是旧版本？）")
    return symbols


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
    log_path = os.path.join(tempfile.gettempdir(), "rs05_watch_openocd.log")
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


def decode_param(index, raw):
    name, short, kind = PARAMS.get(index, (f"0x{index:04X}", f"{index:04X}", "raw"))
    if kind == "f32":
        value = struct.unpack("<f", struct.pack("<I", raw))[0]
    elif kind == "i16":
        value = struct.unpack("<h", struct.pack("<H", raw & 0xFFFF))[0]
    elif kind == "u8":
        value = raw & 0xFF
    else:
        value = raw
    return name, short, kind, value


def read_state(ocd, symbols):
    # 先读电机、后读 tick，反馈时间戳不会比 tick 新
    motor = struct.unpack(MOTOR_FORMAT, ocd.read_bytes(*symbols["rs05_motor"]))
    params_addr, params_size = symbols["rs05_test_params"]
    params = list(struct.iter_unpack(PARAM_FORMAT, ocd.read_bytes(params_addr, params_size)))
    tick = ocd.read(symbols["uwTick"][0], 32, 1)[0]
    enable = ocd.read(symbols["rs05_test_enable"][0], 8, 1)[0]
    current_raw = ocd.read(symbols["rs05_test_current_a"][0], 32, 1)[0]
    esr = ocd.read(CAN1_ESR, 32, 1)[0]
    return {
        "motor_id": motor[0], "drv_enable": motor[1], "out_a": motor[2], "enable_step": motor[3],
        "pos": motor[4], "vel": motor[5], "state": motor[6], "fault": motor[7], "fb_tick": motor[8],
        "params": params, "tick": tick, "enable": enable,
        "cmd_a": struct.unpack("<f", struct.pack("<I", current_raw))[0],
        "tec": (esr >> 16) & 0xFF, "rec": (esr >> 24) & 0xFF, "lec": (esr >> 4) & 0x7, "boff": (esr >> 2) & 1,
    }


def check_layout(s):
    """地址对不上时（ELF 不是板子里那次编译的），参数表的 7 个 index 基本不可能刚好对上。"""
    indexes = [p[0] for p in s["params"]]
    if s["motor_id"] > MOTOR_ID_MAX or any(i not in PARAMS for i in indexes):
        sys.exit("读到的数和 ELF 对不上：电机 ID {}，参数 index {}。\n"
                 "ELF 和板子里的程序可能不是同一次编译的：重新烧一遍，或者用 --elf 指定烧进去的那个。"
                 .format(s["motor_id"], " ".join(f"0x{i:04X}" for i in indexes)))


# ---------- 收帧记录、发一帧 ----------

def read_log(ocd, symbols, since):
    """返回 (现在的总数, since 以后记下的帧)。记录只留最近 16 帧，太多时前面的会丢。"""
    count = ocd.read(symbols["rs05_test_rx_count"][0], 32, 1)[0]
    if count <= since:
        return count, []
    addr, size = symbols["rs05_test_rx_log"]
    entries = list(struct.iter_unpack(FRAME_FORMAT, ocd.read_bytes(addr, size)))
    first = max(since, count - len(entries))
    return count, [entries[n % len(entries)] for n in range(first, count)]


def send_frame(ocd, symbols, frame_id, ext, data):
    """填 rs05_test_tx_frame、置 pending，等程序发进邮箱（pending 变 0）。"""
    tx_addr = symbols["rs05_test_tx_frame"][0]
    pending_addr = symbols["rs05_test_tx_pending"][0]

    def wait_not_pending():
        deadline = time.time() + 0.5
        while True:
            pending = ocd.read(pending_addr, 8, 1)[0]
            if pending != 1:
                return pending
            if time.time() > deadline:
                raise RuntimeError("0.5 s 还没发出去：CAN1 是不是发不出去？先 --once 看 TEC / LEC")
            time.sleep(0.002)

    wait_not_pending()
    payload = struct.pack(FRAME_FORMAT, frame_id | (FRAME_EXT if ext else 0), len(data), bytes(data).ljust(8, b"\0"))
    ocd.write(tx_addr, 32, list(struct.unpack("<4I", payload)))
    ocd.write(pending_addr, 8, [1])
    if wait_not_pending() == 2:
        raise RuntimeError("程序说这一帧不合法（标准帧 ID 超过 0x7FF，或者超过 8 个字节）")


def mit_feedback_text(data):
    """MIT 应答指令 1（手册第 6 章）：Byte0 电机 ID；Byte1~2 角度（0~65535 对应 ±12.57 rad）；
    Byte3 + Byte4 高 4 位 速度（12 位，±50 rad/s）；Byte4 低 4 位 + Byte5 力矩（12 位，±5.5 N·m）；
    Byte6 bit7~6 模式、bit5 故障、bit4 预警；Byte6 低 4 位 + Byte7 绕组温度 ×10"""
    pos = ((data[1] << 8) | data[2]) * 25.14 / 65535 - 12.57
    vel = ((data[3] << 4) | (data[4] >> 4)) * 100.0 / 4095 - 50.0
    torque = (((data[4] & 0x0F) << 8) | data[5]) * 11.0 / 4095 - 5.5
    mode = ("Reset", "Cali", "Motor", "?")[data[6] >> 6]
    fault = "有故障" if data[6] & 0x20 else "无故障"
    warn = "，有预警" if data[6] & 0x10 else ""
    temp = (((data[6] & 0x0F) << 8) | data[7]) / 10.0
    return (f"MIT 应答：电机 ID {data[0]}，{mode}，{fault}{warn}，{temp:.1f} ℃，"
            f"角度 {pos:+.3f} rad，速度 {vel:+.2f} rad/s，力矩 {torque:+.2f} N·m")


def frame_guess(frame):
    """猜一帧是什么，只用来给人看（同一个 ID 在不同协议里可能都说得通，scan 判断时不用它）。"""
    raw_id, dlc, data = frame
    if raw_id & FRAME_EXT:
        ext_id = raw_id & 0x1FFFFFFF
        kind = (ext_id >> 24) & 0x1F
        motor = (ext_id >> 8) & 0xFF
        if kind == 17:
            ok = "读取成功" if ((ext_id >> 16) & 0xFF) == 0 else "读取失败"
            return f"私有协议类型 17 应答，电机 ID {motor}，{ok}"
        if kind == 0 and (ext_id & 0xFF) == 0xFE:
            return f"私有协议类型 0（设备 ID）：电机 ID {motor}，数据是 64 位 MCU 唯一标识符"
        text = {2: "类型 2 反馈", 21: "类型 21 故障反馈"}.get(kind, f"类型 {kind}")
        return f"私有协议{text}，电机 ID {motor}"
    std_id = raw_id & 0x7FF
    node = std_id & 0x7F
    if 0x201 <= std_id <= 0x20B:
        return "大疆电调 / GM6020 的反馈：总线上还接着大疆电机"
    if std_id == HOST_ID and dlc == 8:
        # MIT 应答指令 1 发给主机 ID（默认 0xFD），data[0] 是电机 ID；0x0FD 也在 CANopen 紧急报文的范围里
        return f"{mit_feedback_text(data)}（主机 ID 0x0FD；也可能是 CANopen 节点 {node} 的紧急报文）"
    if 1 <= std_id <= MOTOR_ID_MAX and dlc == 8:
        # MIT 应答指令 2：标准帧 ID 是电机 ID，数据是 64 位 MCU 唯一标识符。切协议（指令 8）、改 ID（指令 7）后回这一帧
        return f"像 MIT 应答指令 2：电机 ID {std_id} 回的 64 位 MCU 唯一标识符（切协议、改 ID 以后回这一帧）"
    if (std_id & 0x780) == 0x700 and node:
        return f"CANopen 节点 {node} 的" + ("开机帧" if (dlc == 1 and data[0] == 0) else "心跳")
    if (std_id & 0x780) == 0x580 and node:
        return f"CANopen 节点 {node} 的 SDO 应答"
    if (std_id & 0x780) == 0x080 and node:
        return f"CANopen 节点 {node} 的紧急报文"
    if dlc == 8 and data[0] <= MOTOR_ID_MAX:
        return f"可能是 MIT 协议的应答（主机 ID 0x{std_id:03X}，电机 ID {data[0]}）"
    return "标准帧"


def frame_text(frame):
    """把一帧写成一行：ID、数据，再加上猜的是什么。"""
    raw_id, dlc, data = frame
    hexdata = " ".join(f"{b:02X}" for b in data[:min(dlc, 8)])
    head = f"EXT 0x{raw_id & 0x1FFFFFFF:08X}" if raw_id & FRAME_EXT else f"STD 0x{raw_id & 0x7FF:03X}     "
    return f"{head} [{dlc}] {hexdata:<23}  {frame_guess(frame)}"


def private_reply(frame, probe):
    """私有协议类型 17 的应答（任意电机 ID）→ 电机 ID"""
    raw_id = frame[0]
    if raw_id & FRAME_EXT and ((raw_id >> 24) & 0x1F) == 17:
        return (raw_id >> 8) & 0xFF
    return None


def mit_reply(frame, probe):
    """MIT 应答指令 1：标准帧、8 字节，data[0] 是刚问过的电机 ID（应答可能在下一次读时才看到）→ 电机 ID"""
    raw_id, dlc, data = frame
    if not raw_id & FRAME_EXT and dlc == 8 and not (0x201 <= raw_id <= 0x20B) and data[0] in (probe, probe - 1):
        return data[0]
    return None


def canopen_reply(frame, probe):
    """CANopen SDO 应答 0x580 + 节点号 → 节点号"""
    raw_id = frame[0]
    if not raw_id & FRAME_EXT and (raw_id & 0x780) == 0x580 and (raw_id & 0x7F):
        return raw_id & 0x7F
    return None


# 扫描：每种协议一种只读或无害的帧，ID 从小到大各发一次；哪种协议先回了话就停
SCAN_PASSES = [
    ("私有协议：类型 17 读 mechPos", "private", private_reply, range(0, MOTOR_ID_MAX + 1),
     lambda i: ((17 << 24) | (HOST_ID << 8) | i, True, [0x19, 0x70, 0, 0, 0, 0, 0, 0])),
    ("MIT 协议：指令 2「停止」（电机进 Reset，没力）", "mit", mit_reply, range(1, MOTOR_ID_MAX + 1),
     lambda i: (i, False, [0xFF] * 7 + [0xFD])),
    ("CANopen：SDO 读 0x1000（设备类型，只读）", "canopen", canopen_reply, range(1, MOTOR_ID_MAX + 1),
     lambda i: (0x600 + i, False, [0x40, 0x00, 0x10, 0x00, 0, 0, 0, 0])),
]


def scan(ocd, symbols):
    # 现在用的这个 ID 的类型 2、17 被驱动和读参数直接用掉，不进收帧记录，下面几轮看不见它。
    # 所以先看驱动：它有新反馈，就说明这个 ID 在私有协议、好好的（类型 2 只有私有协议才回）
    s = read_state(ocd, symbols)
    current_ok = s["fb_tick"] != 0 and s["tick"] - s["fb_tick"] < 100
    if current_ok:
        print(f"现在用的 ID {s['motor_id']}：私有协议，驱动 {s['tick'] - s['fb_tick']} ms 前刚收到它的类型 2 反馈，正常。\n"
              f"直接不带参数一直读就行。下面只看看总线上还有没有别的私有协议电机"
              f"（ID {s['motor_id']} 的应答被程序直接用掉，不会列在下面）。")
    passes = SCAN_PASSES[:1] if current_ok else SCAN_PASSES
    since, _ = read_log(ocd, symbols, 0)
    for name, wanted, is_reply, ids, make in passes:
        print(f"{name}，ID {ids[0]}~{ids[-1]} …", flush=True)
        replies, matches, lost = [], [], 0
        for i in list(ids) + [None]:            # 最后多读一次，收最后一个 ID 的应答
            if i is not None:
                frame_id, ext, data = make(i)
                send_frame(ocd, symbols, frame_id, ext, data)
            else:
                time.sleep(0.05)
            probe = i if i is not None else ids[-1] + 1
            count, new = read_log(ocd, symbols, since)
            lost += count - since - len(new)
            since = count
            replies += new
            matches += [m for m in (is_reply(f, probe) for f in new) if m is not None]

        seen = {}
        for frame in replies:
            text = frame_text(frame)
            seen[text] = seen.get(text, 0) + 1
        for text, times in seen.items():
            print(f"    {text}" + (f"  ×{times}" if times > 1 else ""))
        if lost:
            print(f"    （另外漏了 {lost} 帧：来得太快，记录只留 16 帧）")
        if not replies:
            print("    没有回话")
        if not matches:
            continue

        motor = matches[0]
        print()
        if wanted == "private" and current_ok:
            print(f"总线上还有一个私有协议的电机，ID {motor}。要换成测它：rs05_watch.py set id {motor}")
        elif wanted == "private":
            print(f"电机在私有协议，ID {motor}。接着用：rs05_watch.py set id {motor}")
        elif wanted == "mit":
            print(f"电机在 MIT 协议，ID {motor}。\n"
                  f"切回私有协议（手册第 6 章指令 8；会写进电机，给电机断电再上电才生效）：\n"
                  f"    rs05_watch.py send std 0x{motor:02X} FF FF FF FF FF FF 00 FD\n"
                  f"倒数第二个字节 00 是私有协议，别写成 FF（那是「停止」）。发完给电机断电再上电，再 scan 一次。")
        else:
            print(f"电机在 CANopen 协议，节点 {motor}。\n"
                  f"切回私有协议（手册 5.10 节，扩展帧 ID 0xFFF；会写进电机，给电机断电再上电才生效）：\n"
                  f"    rs05_watch.py send ext 0xFFF 01 02 03 04 05 06 00 00\n"
                  f"发完给电机断电再上电，再 scan 一次。")
        return
    if current_ok:
        print(f"\n没有别的电机。ID {s['motor_id']} 就是总线上唯一的私有协议电机。")
        return
    print("\n三种协议都没人回话。总线上有节点在应答（TEC、LEC 都是 0 时），但不是说这三种协议的 RS05。\n"
          "  查：CAN1 上还接了什么（分电板上的电调、USB-CAN、别的板子）；电机自己有没有上电；\n"
          "  再开着 log，给电机断电再上电，看有没有帧冒出来。")


def print_log_since(ocd, symbols, since, title):
    since, new = read_log(ocd, symbols, since)
    print(title)
    for frame in new:
        print(f"  {frame_text(frame)}")
    if not new:
        print("  （没有）")
    return since


# ---------- 显示 ----------

def feedback_age(s):
    return "--" if s["fb_tick"] == 0 else str(max(0, s["tick"] - s["fb_tick"]))


def format_param_cell(index, raw, ok, fail):
    _, short, kind, value = decode_param(index, raw)
    width = max(len(short), 8 if kind in ("f32", "raw") else 5)
    if ok == 0:
        text = "fail" if fail else "--"
    elif kind == "f32":
        text = f"{value:+.4f}"
    elif kind == "raw":
        text = f"{value:08X}"
    else:
        text = str(value)
    return short.rjust(width), text.rjust(width)


def print_line(s, header):
    cells = [format_param_cell(*p) for p in s["params"]]
    if header:
        print(f"{'tick':>9}  id {'fb':>3} st flt | {'pos_rad':>8} {'vel':>7} | en {'cmd_A':>6} {'out_A':>6} | "
              + " ".join(c[0] for c in cells) + " | TEC REC LEC")
    print(f"{s['tick']:>9} {s['motor_id']:>3} {feedback_age(s):>3} {s['state']:>2}  {s['fault']:02X} | "
          f"{s['pos']:+8.4f} {s['vel']:+7.3f} | {s['enable']:>2} {s['cmd_a']:+6.3f} {s['out_a']:+6.3f} | "
          + " ".join(c[1] for c in cells)
          + f" | {s['tec']:>3} {s['rec']:>3} {s['lec']:>3}" + ("  BUS-OFF" if s["boff"] else ""), flush=True)


def print_full(s):
    faults = [name for bit, name in enumerate(FAULT_BITS) if s["fault"] & (1 << bit)]
    print(f"HAL tick {s['tick']} ms；CAN1 TEC {s['tec']}、REC {s['rec']}、最近一次错误：{LEC_NAMES[s['lec']]}"
          + ("，BUS-OFF" if s["boff"] else ""))
    print(f"电机 ID {s['motor_id']}：state {s['state']} {STATE_NAMES.get(s['state'], '?')}，"
          f"fault 0x{s['fault']:02X} {('、'.join(faults)) if faults else '无'}，反馈 {feedback_age(s)} ms 前"
          f"{'（还没收到过）' if s['fb_tick'] == 0 else ''}，使能序列下一步 {s['enable_step']}")
    print(f"  position_rad {s['pos']:+.4f}   velocity_rad_s {s['vel']:+.3f}")
    print(f"指令：rs05_test_enable {s['enable']}，rs05_test_current_a {s['cmd_a']:+.3f} A；"
          f"驱动里 enable {s['drv_enable']}、current_a {s['out_a']:+.3f} A（限流、限速以后实际发的）")
    print("参数（类型 17）：")
    print(f"  {'index':<7}{'名字':<11}{'值':>12}  {'raw':>10}  {'成功':>6}  {'失败':>6}")
    for index, raw, ok, fail in s["params"]:
        name, _, kind, value = decode_param(index, raw)
        if ok == 0:
            text = "失败" if fail else "没回过"
        elif kind == "f32":
            text = f"{value:+.6f}"
        else:
            text = str(value)
        print(f"  0x{index:04X} {name:<12}{text:>12}  0x{raw:08X}  {ok:>7}  {fail:>7}")
    if s["fb_tick"] == 0:
        if s["tec"] >= 128 or s["boff"] or s["lec"] in (4, 5):
            print("提示：CAN1 发不出去（TEC 大 / 位错误）。查 C 板有没有 24 V 或 USB（只接 DAPLink 时收发器没有 5 V）、CAN 线。")
        elif s["lec"] == 3:
            print("提示：发出去的帧没人应答。查电机有没有电、CANH / CANL 有没有接反。")
        else:
            print("提示：帧发出去了、有人应答，但电机不回话。跑一次 scan，看电机在哪种协议、ID 是多少。")


def parse_command(parser, command):
    """检查命令，返回 ("set", 符号名, 位宽, 值) / ("scan",) / ("log",) / ("send", ID, 扩展帧, 数据)。"""
    name = command[0]
    if name == "scan" and len(command) == 1:
        return ("scan",)
    if name == "log" and len(command) == 1:
        return ("log",)
    if name == "send":
        if len(command) < 3 or command[1] not in ("std", "ext"):
            parser.error("send 的写法：send std|ext <ID> [最多 8 个十六进制字节]，比如 send std 0x7F FF FF FF FF FF FF FF FD")
        try:
            frame_id = int(command[2], 0)
            data = [int(b, 16) for b in command[3:]]
        except ValueError:
            parser.error("ID 写成 0x7F 或 127，字节写成十六进制（FF）")
        if len(data) > 8 or any(b > 0xFF for b in data):
            parser.error("最多 8 个字节，每个 00~FF")
        if frame_id > (0x1FFFFFFF if command[1] == "ext" else 0x7FF):
            parser.error("标准帧 ID 最大 0x7FF，扩展帧 0x1FFFFFFF")
        return ("send", frame_id, command[1] == "ext", data)
    if name != "set" or len(command) != 3 or command[1] not in ("enable", "current", "id"):
        parser.error("命令：set enable <0|1>、set current <A>、set id <0~127>、scan、log、send std|ext <ID> <字节…>")
    key, value = command[1], command[2]
    if key == "enable":
        if value not in ("0", "1"):
            parser.error("enable 只能是 0 或 1")
        return ("set", "rs05_test_enable", 8, int(value))
    if key == "id":
        if not value.isdigit() or int(value) > MOTOR_ID_MAX:
            parser.error("id 是 0~127")
        return ("set", "rs05_motor", 8, int(value))        # RS05_Motor_t 的第一个字段 id
    try:
        current = float(value)
    except ValueError:
        parser.error(f"current 要是一个数（A）：{value}")
    return ("set", "rs05_test_current_a", 32, struct.unpack("<I", struct.pack("<f", current))[0])


def main():
    parser = argparse.ArgumentParser(description="RS05_test：OpenOCD 不停核读写测试变量")
    parser.add_argument("--elf", default=DEFAULT_ELF, help="烧进板子的那个 ELF（默认 build/Debug/RS05_test.elf）")
    parser.add_argument("--openocd", default=DEFAULT_OPENOCD, help="没有在跑的 OpenOCD 时用来起一个")
    parser.add_argument("--cfg", default=DEFAULT_CFG, help="OpenOCD 的板子配置")
    parser.add_argument("--port", type=int, default=6666, help="OpenOCD 的 TCL 端口")
    parser.add_argument("--period", type=float, default=0.2, help="一直读时每行间隔，秒")
    parser.add_argument("--once", action="store_true", help="读一次，打全部字段")
    parser.add_argument("command", nargs="*", help="set enable|current|id <值>、scan、log、send std|ext <ID> <字节…>")
    # 固定 UTF-8：输出走管道（Git Bash 等）时 Python 默认用 GBK，中文会乱码。报错（sys.exit）走 stderr
    sys.stdout.reconfigure(encoding="utf-8", errors="replace")
    sys.stderr.reconfigure(encoding="utf-8", errors="replace")
    args = parser.parse_args()

    command = parse_command(parser, args.command) if args.command else None
    symbols = find_symbols(args.elf)
    ocd, proc = connect(args)
    try:
        check_layout(read_state(ocd, symbols))
        if command is None:
            if args.once:
                print_full(read_state(ocd, symbols))
                count, _ = read_log(ocd, symbols, 0)
                print_log_since(ocd, symbols, max(0, count - 16), f"最近收到的帧（驱动、读参数用掉的除外，一共记过 {count} 帧）：")
            else:
                line = 0
                while True:
                    print_line(read_state(ocd, symbols), header=(line % 20 == 0))
                    line += 1
                    time.sleep(args.period)
        elif command[0] == "set":
            _, symbol, width, value = command
            ocd.write(symbols[symbol][0], width, [value])
            time.sleep(0.05)
            print_full(read_state(ocd, symbols))
        elif command[0] == "scan":
            scan(ocd, symbols)
        elif command[0] == "send":
            _, frame_id, ext, data = command
            since, _ = read_log(ocd, symbols, 0)
            send_frame(ocd, symbols, frame_id, ext, data)
            print(f"发了：{'EXT' if ext else 'STD'} 0x{frame_id:X} [{len(data)}] {' '.join(f'{b:02X}' for b in data)}")
            time.sleep(0.2)
            print_log_since(ocd, symbols, since, "0.2 s 内收到的帧：")
        else:
            since, _ = read_log(ocd, symbols, 0)
            print(f"从第 {since} 帧往后打印新收到的帧（驱动、读参数用掉的除外），Ctrl+C 退出", flush=True)
            while True:
                count, new = read_log(ocd, symbols, since)
                if count - since > len(new):
                    print(f"  （漏了 {count - since - len(new)} 帧：来得太快，记录只留 16 帧）")
                for frame in new:
                    print(f"  {frame_text(frame)}", flush=True)
                since = count
                time.sleep(0.05)
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
