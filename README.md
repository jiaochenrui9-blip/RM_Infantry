# RM_Infantry

RoboMaster 步兵机器人 STM32 固件开发工程。目前云台、底盘各有一个测试工程（底盘工程只有框架），整车集成尚在规划中。

## 目录

- `Shared/BSP/`：CAN 等硬件抽象与通信封装。
- `Shared/Devices/`：DJI 电机、BMI088 和 IST8310 驱动。
- `Shared/Algorithm/`：PID、数学工具与姿态解算。
- `Test_Module/Gimbal/Gimbal_test/`：基于 STM32F407IGH6 的云台测试工程，包含 CubeMX 配置、应用代码和 CMake 工程。
- `Test_Module/Chassis/Chassis_test/`：底盘测试工程（同一块 C 板），目前有 CubeMX 配置、CMake 工程和空的装配层，底盘模块待写。
- `硬件资料/`：开发板、电机、传感器等参考资料。
- `notes.md`：项目约定和开发记录。

## 构建测试工程

需要 ARM GNU Toolchain、CMake 和 Ninja。保持本仓库目录结构，因为工程从 `Shared/` 引用公共代码。两个工程的构建方式相同，以云台为例：

```powershell
cd Test_Module/Gimbal/Gimbal_test
cmake --preset Debug
cmake --build --preset Debug --clean-first
```

输出为 `build/Debug/Gimbal_test.elf`。2026-10-28 在本机执行完整清理编译并链接通过（RAM 3136 B，FLASH 24076 B）。底盘工程在 `Test_Module/Chassis/Chassis_test` 下同样构建，输出 `build/Debug/Chassis_test.elf`，2026-11-01 清理编译无警告（RAM 1752 B，FLASH 11676 B）。构建成功不代表传感器、CAN 通信或电机的实物行为已经验证。外设配置以 CubeMX 的 `.ioc` 文件为准，详情见各工程内的笔记（云台工程另有 README）。

构建输出、IDE 本地状态和历史备份不纳入版本管理；第三方驱动及硬件资料的权利属于各自权利人。
