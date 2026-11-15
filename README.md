# RM_Infantry

RoboMaster 步兵机器人 STM32 固件开发工程。当前整车采用 C 板控制 RS05 云台、F405 板控制四轮全向底盘。RS05 云台已接入 USB CDC 陀螺仪波形和比赛发射机构的第一版控制。

## 目录

- `Shared/BSP/`：CAN 等硬件抽象与通信封装。
- `Shared/Devices/`：DJI 电机、BMI088 和 IST8310 驱动。
- `Shared/Algorithm/`：PID、数学工具与姿态解算。
- `F405_Project/`：F405 底盘主控固件。
- `Test_Module/Gimbal/Gimbal_test_RS05/`：整车使用的 C 板云台固件。
- `Test_Module/Gimbal/Gimbal_test/`：基于 STM32F407IGH6 的云台测试工程，包含 CubeMX 配置、应用代码和 CMake 工程。
- `Test_Module/Chassis/Chassis_test/`：C 板版底盘测试工程。
- `硬件资料/`：开发板、电机、传感器等参考资料。
- `notes.md`：项目约定和开发记录。

## 整车接线

- 云台 C 板 CAN1 接两个 RS05，并通过板间 CAN 接底盘 F405 的 CAN2。云台 C 板 CAN2 接比赛发射机构的两台 M3508（ID 1、2）和一台 M2006（ID 3）。
- 底盘 F405 的 CAN1 接四台 C620 + M3508。底盘插座和调试口脚位见 [F405 底盘说明](F405_Project/README.md)，整车供电见 [电源树](docs/整车电源树.svg)。

## 编译与烧录

需要 ARM GNU Toolchain、CMake 和 Ninja。保持仓库目录结构，各工程从 `Shared/` 引用公共代码。以 RS05 云台为例：

```powershell
cd Test_Module/Gimbal/Gimbal_test_RS05
cmake --preset Debug
cmake --build --preset Debug --clean-first
```

输出为 `build/Debug/Gimbal_test_RS05.elf`。底盘在 `F405_Project` 下用同一套命令构建，输出 `build/Debug/F405_Project.elf`。将 DAPLink 接到对应板子的 SWD 口，再在 CLion 的 OpenOCD 下载配置中选中对应 ELF，运行下载。

## 当前验证进度

- RS05 云台 ARM Debug 全量编译通过，复查了摩擦轮就绪联锁、掉速退回和掉线锁定的主机模型。该版未烧录，发射机构未上板验证。
- CAN2 的自动重发与离线恢复已在 `Gimbal_test_RS05.ioc` 中打开。请在 CubeMX 点 **GENERATE CODE** 后重新编译；当前生成的 `can.c` 里两项仍为 `DISABLE`。

构建输出、IDE 本地状态和历史备份不纳入版本管理；第三方驱动及硬件资料的权利属于各自权利人。
