# Gimbal_test

STM32F407IGH6 云台测试工程，使用 STM32CubeMX、CMake 和 ARM GNU Toolchain。

## DJI 电机驱动移植

电机驱动最初来自 `C:\Clion\DJI_Motor_Driver\Core\UserModules`，CAN BSP 与接收分发参考 `C:\Project\Fruit-picking robot\Fruit_Picking\UserModules`。
这些公共代码已移到各工程共用的 `RM_Infantry/Shared/`：电机驱动位于 `Shared/Devices/DJI_Driver/`，PID 位于 `Shared/Algorithm/`，通用 CAN BSP 位于 `Shared/BSP/`：

- `dji_motor.*`、`dji_motor_manager.*`：电机状态、注册、反馈解析和分组发送。
- `dji_m3508.*`、`dji_m2006.*`、`dji_gm6020.*`：型号参数和注册接口。
- `motor_control.*`：电流、速度、位置控制。
- `PID.*`：通用 PID 算法。
- `can_bus.*`：标准帧与扩展帧的过滤、启动和收发。

顶层 CMake 通过 `SHARED_DIR`（本工程目录的 `../../../Shared`）引用公共代码，因此本工程要放在 RM_Infantry 目录结构里才能编译，不能单独拷走。本工程自己的应用层在 `App/` 下，结构和用法见 notes.md。

## 已接入的通信链路

| 总线 | RX | TX | 速率 | 管理器 | 接收中断 |
|---|---|---|---|---|---|
| CAN1 | PD0 | PD1 | 1 Mbps | `robot_can1`（App/robot.c） | CAN1_RX0 |
| CAN2 | PB5 | PB6 | 1 Mbps | `robot_can2`（App/robot.c） | CAN2_RX0 |

两个 manager 由 `App/robot.c` 持有并按顺序初始化：先 Init manager，再让各模块注册电机，最后 Start（配置过滤器、启动 CAN 和 FIFO0 通知）。HAL 接收回调也在 robot.c：先读取一帧，再按 CAN 句柄交给对应 manager 的 `DJI_MotorManager_ProcessRxFrame()`；以后同一总线接入其他设备时把同一帧也交给它们的驱动。CAN1 使用过滤器组 0，CAN2 使用组 14。

云台模块在 CAN1 上注册了 GM6020 ID1（yaw）和 ID2（pitch），烧录后每 1 ms 发一帧 0x1FE；默认 RELAX 模式下电流为 0。电机接在哪路 CAN 在 `Robot_Init()` 里改，ID、方向等参数在 `App/gimbal/gimbal_config.h`。

## 电机注册参数

型号注册参数映射如下，反馈 ID 由注册函数自动计算：

| 型号和电调 ID | 反馈 ID | 注册 tx_id | tx_slot |
|---|---|---|---|
| M3508 / M2006，1～4 | 0x200 + ID | 0x200 | ID - 1 |
| M3508 / M2006，5～8 | 0x200 + ID | 0x1FF | ID - 5 |
| GM6020，1～4 | 0x204 + ID | 0x1FE | ID - 1 |
| GM6020，5～7 | 0x204 + ID | 0x2FE | ID - 5 |

每条总线最多注册 8 个电机；同一总线上反馈 ID 和控制帧槽位不能冲突。一个 manager 应管理该分组帧影响的全部电机，因为未注册槽位会被发送为零。GM6020 沿用源驱动的电流控制协议，实际电机固件须支持并已开启电流环。

`MotorControl_Init()` 的限幅参数为协议命令值，应使用正数且不超过对应型号的驱动上限；方向为 `1` 或 `-1`。离线超过 100 ms 或处于禁用模式时，`MotorControl_Update()` 将输出置零；需持续调用更新及发送才会把零命令发到总线。

## 构建与验证

在工程目录执行（ARM GNU Toolchain、CMake、Ninja 需可用）：

```powershell
cmake --preset Debug
cmake --build --preset Debug --clean-first
```

输出为 `build/Debug/Gimbal_test.elf`。

本次验证（2026-10-27）：BSP 调整后完整清理、编译和链接通过，无编译警告。主机 HAL 模拟的 43 项检查通过，覆盖两个 CAN 的启动、重复启动、过滤器和扩展帧、FIFO 单次读取与分发、DJI 反馈处理、错误返回及邮箱忙。测试使用从 `main.c` 提取的实际接收回调。

公共代码移到 `Shared/` 后再次验证（2026-10-27）：重新配置并完整清理编译，无警告；`text/data/bss` 为 10584/12/1732 字节，与移动前相同；`compile_commands.json` 中 8 个公共源文件都指向 `Shared/`。

App 精简版验证（2026-10-27；带 IMU 的完整版备份在 `../Backup/App_full_2026-10-27`，不参与编译）：完整清理编译无警告（FLASH 24076 B、RAM 3136 B）。主机测试把 gimbal、test_cmd 和真实的 DJI 驱动、PID、can_bus 源码接到一阶电机模型上闭环运行：默认参数 35 项、方向 -1 + 编码器偏移 + yaw 不限位 36 项全部通过，覆盖方向一致性、编码器换算、跨 ±180°（不限位走最短路，有限位不从背面绕）、软限位、掉线锁定与解锁、无扰切换、非法模式和方波。测试代码放在系统临时目录（`gimbal_host_test`），没有放进工程。

速度前馈验证（2026-10-29）：本机没有 ARM 工具链，ARM 工程未重新编译。用桩头文件替代 HAL 和 DJI 驱动，把 `gimbal.c` 原文件与真实的 PID、user_math 用主机 gcc（`-Wall -Wextra`）编译，无警告；接到一阶电机模型（速度环时间常数约 10 ms、rpm 取整量化）上闭环运行，目标以默认斜坡 100°/s 移动 60°～70°：无前馈时斜坡过程中落后目标约 12.5°（= 100 / 角度环 Kp 8），有前馈时最大落后约 0.9°；斜坡结束时有前馈的超调约 0.6°（无前馈约 0.1°），跨 ±180° 的不限位轴结果相同，速度环目标始终在限幅内。模型参数是假设值，实际效果以上板为准。测试代码放在临时目录，没有放进工程。

编译、链接和主机模拟测试只能验证软件链路；实际 CAN 收发、电机方向和运动效果仍需硬件测试。以上验证都没有烧录，也没有让电机动作。
