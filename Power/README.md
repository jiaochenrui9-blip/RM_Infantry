# INA228 直流功率计

当前实现：STM32F103C8T6 经 I2C1 读取 INA228，主循环每 10 ms 更新一次 Power 反馈；CAN1 以 1 Mbps 控制一个 ID=1 的 M3508，SysTick 每 1 ms 执行速度环，目标为转子 1000 RPM。采样电阻由用户确认为 **1 mΩ**。

## 连接与配置

| 功能 | MCU / 网络 | 说明 |
|---|---|---|
| I2C1 SCL | PB6 | 400 kHz，板载 2.2 kΩ 上拉到 3.3 V |
| I2C1 SDA | PB7 | 板载 2.2 kΩ 上拉到 3.3 V |
| INA228 电源 | 3.3 V / GND | 不是 VIN 高压供电脚 |
| INA228 IN+ / IN− | VIN / VOUT | 电流正方向为 VIN 流向 VOUT |
| INA228 VBUS | VIN | 反馈电压为输入侧电压 |
| INA228 A0 / A1 | GND / GND | 七位设备地址 0x40 |
| HSE | PD0 / PD1，8 MHz 陶瓷谐振器 | PLL ×9，SYSCLK 72 MHz，APB1 36 MHz |
| SWD | PA13 / PA14 | CN5：1 SWDIO、2 SWCLK、3 GND、4 3.3 V |
| CAN | PB8 RX / PB9 TX，已重映射 | 1 Mbps；分频 3、BS1=9、BS2=2、SJW=1，采样点约 83.3%；接板载收发器 |

主功率回路为 VIN → 采样电阻 → VOUT，负极共地。XT30/XT60 为同一路的并联接口。输入电容网表为 25 V，不能按 INA228 的 85 V 能力接高压；实板耐压和持续载流尚未验证。

## 代码入口

- `App/motor_config.h`：ID=1、目标转子 1000 RPM、方向 +1、速度 PID 3/10/0、电流指令限幅 ±3000。参数为初值，未在实物负载下整定。按共享驱动的 3591/187 减速比，输出轴约 52.1 RPM。
- `App/robot.c`：直接引用 Shared 的 CAN BSP、DJI 驱动和 PID，不另复制驱动。发送标准帧 0x200，ID1 占前两字节，其他槽位为零；接收 0x201 反馈。未收到反馈或超过 100 ms 未更新时清零输出；恢复反馈后自动继续 1000 RPM 控制。
- `Robot_ControlTick`：由 SysTick 的 USER CODE 区每 1 ms 调用，初始化完成后才执行；不读取 I2C、不等待 CAN 空邮箱，忙时下一拍发最新控制量。CAN RX0 中断优先级 5，SysTick 为 15。1 ms 实际耗时和最坏中断延迟仍待上板测量。

电机连接：将 C620 设置为 ID 1，电调 CANH/CANL 接本板 CANH/CANL，共参考地，按总线拓扑配置两端 120 Ω 终端。PB8/PB9 是 MCU 逻辑引脚，不能直接接电调的 CANH/CANL。电调需主电源供电，调试器不能给电机供电；固件启动并收到有效反馈后会主动转动。控制测试时避免同总线上另一个控制器同时发送 0x200。

调试直接看 `robot.c` 内的 `s_motor.speed.target_rpm`、`s_motor.speed_rpm`、`s_motor.output_current`、`s_motor.feedback_count` 和 `s_motor_ready`，不另存一份调试反馈。修改固件后须按新的 ELF 重新取 VOFA 内存地址。

- `Shared/BSP/i2c_bus.c/.h`：通用阻塞式寄存器读写，调用者传未左移的七位地址，BSP 在调用 HAL 时左移一次。`ReadRegU16/WriteRegU16` 用于高字节在前的两字节寄存器，读取 ID 也用这个接口。
- `App/Power.c/.h`：`Power_Init` 校验厂商 ID、器件型号，写配置并回读；`Power_Update` 检查转换完成后分别读取四个测量寄存器，换算成功才发布整组反馈。
- `App/Power_config.h`：采样电阻、设备地址、电流单位、调度周期。
- `App/robot.c/.h`：持有 I2C1 装配和 10 ms 调度。`main.c` 的 USER CODE 区只调用 `Robot_Init/Robot_Loop`。

参数为 ADCRANGE=0、CURRENT_LSB=312.5 μA/计数、SHUNT_CAL=4096，ADC_CONFIG=0xB6D9。芯片约每 2.24 ms 更新结果，主循环读取频率为 100 Hz。软件量程约 ±163.84 A，不代表整板额定电流。

## 查看反馈

在 CLion 调试器中查看 `Power.c` 的静态变量 `s_feedback`（必要时用 GDB 表达式 `'Power.c'::s_feedback`），不要再建一份全局调试副本。

| 字段 | 单位 / 含义 |
|---|---|
| voltage_v | V，VIN 对地电压 |
| shunt_voltage_v | V，采样电阻压差 |
| current_a | A，有符号电流 |
| power_w | W，芯片无符号功率；方向结合 current_a 判断 |
| sample_tick | 最近整组读成功的 HAL 毫秒时间戳 |
| manufacturer_id | 应为 0x5449 |
| device_id | 高十二位应为 0x228，低四位为版本 |
| status | HAL_OK：读取成功；HAL_BUSY：尚无新样本；HAL_ERROR/HAL_TIMEOUT：失败 |

其他模块通过 `Power_GetFeedback` 获取反馈。数据读取失败时保留上次样本，使用方必须检查状态和时间戳，不能把旧数据当成新数据。初始化失败后应修复连接并重新调用 `Power_Init` 或复位 MCU；当前没有自动重初始化策略。

完整发布只避免软件半组更新，不保证连续转换时四个寄存器来自同一转换周期。Power 与 BSP 由单一主循环串行调用，不适合直接在控制中断内调用。BSP 传给 HAL 的超时为 5 ms，HAL 内部可能还有固定超时等待，不能将其视为函数总阻塞上限。

## 编译与烧录

使用 CLion 打开本目录，工具链选择 ARM GNU，按 `CMakePresets.json` 的 Debug 预设构建。命令行需把 CMake、Ninja、arm-none-eabi-gcc 加入 PATH：

```powershell
cmake --preset Debug
cmake --build --preset Debug
```

**改 .ioc 后必须在 CubeMX 重新生成代码，再编译。** 本次已用 CubeMX 6.15.0 完成生成，包含时钟、I2C、CAN RX0 中断和自动离线恢复；没有手改生成区。生成不覆盖 App/Shared，USER CODE 中的调用应保留。自动重发保持关闭，控制帧按 1 ms 周期更新。

DAPLink 连接 SWDIO、SWCLK、GND、目标电压参考，按探针供电方式接线，避免重复给 3.3 V 供电。需要实际烧录时，在本目录执行：

```powershell
openocd -f interface/cmsis-dap.cfg -c "transport select swd" -f target/stm32f1x.cfg -c "program build/Debug/Power.elf verify reset exit"
```

这是供用户之后执行的烧录命令，本轮没有执行。首次验证用低压限流电源，先看 ID、VIN 与万用表是否一致，再接已知负载检查 current_a 和 power_w。

## 验证记录

- 最新 M3508 版本：`build/Debug/Power.elf` 与 `build/codex-debug/Power.elf` 均构建通过，无编译警告；FLASH 20316 B，RAM 1976 B。九组主机测试通过，新增覆盖真实 DJI 驱动的 ID1 路由、速度误差方向、1000 RPM 目标、电流限幅、离线清零/恢复、忙邮箱及 CAN 启动失败；CAN BSP 边界由主机模型替代。Gimbal_test 和 Chassis_test 共享驱动增量构建通过。**此版本未烧录，电机转动、闭环精度、CAN 实际通信和中断耗时未验证。下方实板记录属于之前只读功率计的固件。**

- 2026-11-17：ARM GNU 13.3.1 Debug 全量编译通过，无编译警告；FLASH 12076 B，RAM 1744 B。独立验证输出位于 `build/codex-debug/Power.elf`。
- `python tests/run_tests.py`：七组测试通过。编译真 BSP、Power 和 robot 代码，仅模拟 HAL/I2C 寄存器，覆盖字节序、地址移位、校准、符号边界、部分读取失败、ID/配置错误及计时回绕。
- 原有 Gimbal_test、Chassis_test 全量 ARM Debug 构建通过。它们尚未使用新增 I2C BSP，此项仅为原有工程构建回归。
- 用户已烧录；OpenOCD 只读检查确认 Flash 与 build/Debug/Power.elf 一致，HSE/PLL 就绪、I2C 400 kHz 配置生效。在仅调试器供电、无主电源/负载的条件下，约 10.4 秒采集的 282 组稳定反馈均为 HAL_OK，ID 正确，观察到 10 ms 更新。实测波形、带电源/负载测量精度、断电重启和温升仍待验证。
- 2026-11-17 后续：CAN 1 Mbps、PB8/PB9 已通过 CubeMX 生成并编译通过；未进行实际 CAN 通信验证。

## 手册

- [INA228 数据手册](https://www.ti.com/lit/ds/symlink/ina228.pdf)：§7.5.1.1 读写，§7.6 寄存器，§8.1.2 换算。
- [STM32F103 数据手册](https://www.st.com/resource/en/datasheet/stm32f103c8.pdf)：引脚、时钟和 I2C 电气限制。
