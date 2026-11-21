# RM_Infantry

RoboMaster 步兵机器人 STM32 固件开发工程。当前整车采用 C 板控制 RS05 云台、F405 板控制四轮全向底盘。云台与 F405 底盘工程均已接入 BMI088、IMU 加热和 USB CDC 波形。比赛发射机构卡弹时用位置环反转退弹，连续卡弹会锁定。另有 F103 功率计及单 M3508 测试工程。

## 目录

- `Shared/BSP/`：CAN 等硬件抽象与通信封装。
- `Shared/Devices/`：DJI 电机、BMI088 和 IST8310 驱动。
- `Shared/Algorithm/`：PID、数学工具与姿态解算。
- `F405_Project/`：F405 底盘主控固件。
- `Power/`：STM32F103 + INA228 功率计及单 M3508 速度测试，接线和验证记录见 [Power 说明](Power/README.md)。
- `Test_Module/Gimbal/Gimbal_test_RS05/`：整车使用的 C 板云台固件。
- `Test_Module/Gimbal/Gimbal_test/`：基于 STM32F407IGH6 的云台测试工程，包含 CubeMX 配置、应用代码和 CMake 工程。
- `Test_Module/Chassis/Chassis_test/`：C 板版底盘测试工程。
- `硬件资料/`：开发板、电机、传感器等参考资料。
- `notes.md`：项目约定和开发记录。

## 整车接线

- 云台 C 板 CAN1 接两个 RS05，并通过板间 CAN 接底盘 F405 的 CAN2。云台 C 板 CAN2 接比赛发射机构的两台 M3508（ID 1、2）和一台 M2006（ID 3）。
- 云台发往底盘的 `0x110` 指令为 100 Hz（约每 10 ms 一帧），底盘控制仍以 1 kHz 运行；超过 50 ms 未收到新帧时进入无力模式。
- RS05 云台使用板载 BMI088 的 INT3 → PC5 数据就绪信号启动 DMA 采样，TIM2 以 1 MHz 计时；每个新样本触发一次控制，超过 1.5 ms 没有新样本时按系统 tick 兜底运行。
- 底盘 F405 的 CAN1 接四台 C620 + M3508。底盘插座和调试口脚位见 [F405 底盘说明](F405_Project/README.md)，整车供电见 [电源树](docs/整车电源树.svg)。
- F405 第二版网表中 CAN1 的 2-pin 插座为 1 CANH、2 CANL，与第一版相反。实物接线前须核对，差异见 [第二版板子笔记](F405_Project/notes.md#2026-11-17-第二版板子网表-405板子的网表_v2enet)。

## 编译与烧录

需要 ARM GNU Toolchain、CMake 和 Ninja。保持仓库目录结构，各工程从 `Shared/` 引用公共代码。以 RS05 云台为例：

```powershell
cd Test_Module/Gimbal/Gimbal_test_RS05
cmake --preset Debug
cmake --build --preset Debug --clean-first
```

输出为 `build/Debug/Gimbal_test_RS05.elf`。底盘在 `F405_Project` 下用同一套命令构建，输出 `build/Debug/F405_Project.elf`。将 DAPLink 接到对应板子的 SWD 口，再在 CLion 的 OpenOCD 下载配置中选中对应 ELF，运行下载。

Power 工程也使用 Debug 预设，输出 `Power/build/Debug/Power.elf`，下载配置选择 STM32F1 目标。该测试固件收到 ID1 电机有效反馈后会按目标转子 1000 RPM 控制，接线、参数和下载命令见 [Power 说明](Power/README.md)。

## 当前验证进度

- RS05 云台、F405 底盘、GM6020 云台和 C 板底盘测试工程的 ARM Debug 全量编译通过。复跑了 22 个发射机构主机模型场景，复核了位置反转、掉速退出和连续卡弹锁定。
- 发射机构新增 [`shoot_watch.py`](Test_Module/Gimbal/Gimbal_test_RS05/tools/shoot_watch.py) 调试脚本，使用说明见 [RS05 云台笔记](Test_Module/Gimbal/Gimbal_test_RS05/notes.md)。本版未烧录，发射机构和调试脚本均未在真板上验证。
- RS05 云台 CAN2 的自动重发与离线恢复已通过 CubeMX 生成，当前 `can.c` 两项均为 `ENABLE`。以后修改 `.ioc` 后仍须重新生成并编译。
- 2026-11-19 提交前，六个 ARM 工程均全量编译通过且无编译警告，CarSim、HostTest、HostTest_RS05、Power 九组主机测试通过，九个 Python 脚本语法检查通过。本轮未烧录、未上板验证；Power 笔记中的实板功率计结果仅对应此前版本。
- 2026-11-21 提交前，五个受共用 BMI088 / DJI 驱动改动影响的 ARM 工程全量编译通过，CarSim、HostTest、Power 九组主机测试及 DRDY 时序模拟通过，改动的 Python 脚本语法检查通过，CPU 分析脚本用合成样本复跑正常。RS05 云台已记录部分实板验收，USB 连续运行及带电机验收仍待完成，详见该工程 notes.md；本次提交操作未烧录或操作硬件。
- 同日板间指令降至 100 Hz 后，两版云台、F405 底盘及 C 板底盘测试工程全量编译通过，CarSim、HostTest、HostTest_RS05 通过；降频版本尚未烧录、上板验证。

构建输出、IDE 本地状态和历史备份不纳入版本管理；第三方驱动及硬件资料的权利属于各自权利人。
