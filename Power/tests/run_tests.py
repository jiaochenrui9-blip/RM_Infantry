"""编译真 BSP/App 代码，以内存寄存器模型验证协议、换算和异常路径。"""
from pathlib import Path
import shutil
import subprocess

project = Path(__file__).resolve().parents[1]
compiler = shutil.which("gcc")
if compiler is None:
    raise SystemExit("未找到主机 gcc，请先将 MinGW 的 bin 目录加入 PATH。")
output = project / "build" / "host-tests"
output.mkdir(parents=True, exist_ok=True)
executable = output / "test_power.exe"
subprocess.run([
    compiler, "-std=c11", "-Wall", "-Wextra", "-Werror", "-O0", "-g",
    "-I" + str(project / "tests"),
    "-I" + str(project / "App"),
    "-I" + str(project.parent / "Shared" / "BSP"),
    "-I" + str(project.parent / "Shared" / "Devices" / "DJI_Driver"),
    "-I" + str(project.parent / "Shared" / "Algorithm"),
    str(project / "tests" / "test_power.c"),
    str(project / "App" / "Power.c"),
    str(project / "App" / "robot.c"),
    str(project.parent / "Shared" / "BSP" / "i2c_bus.c"),
    *[str(project.parent / "Shared" / "Devices" / "DJI_Driver" / source)
      for source in ["dji_motor.c", "dji_motor_manager.c", "dji_m3508.c", "motor_control.c"]],
    str(project.parent / "Shared" / "Algorithm" / "PID.c"),
    "-o", str(executable),
], check=True)
subprocess.run([str(executable)], check=True)
