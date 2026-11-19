# Power 功率计工程笔记

## 当前进度

- 2026-11-17 M3508 接入：Power 直接编译 Shared 的 can_bus、DJI 驱动和 PID；共享 manager 对 CAN2 判断增加条件编译以支持仅 CAN1 的 F103，F4 行为不变。ID=1，0x201 接收，0x200 槽 0 发送；速度模式目标转子 1000 RPM，PID 3/10/0，电流指令限幅 ±3000，配置位于 App/motor_config.h。
- 新版 Robot_ControlTick 在 SysTick USER CODE 区每 1 ms 执行，初始化完成后才控制；Power 仍在主循环每 10 ms 读取，阻塞 I2C 不在控制中断内执行。未收到反馈或超过 100 ms 无反馈时输出零，恢复通信后自动恢复速度目标。CAN RX0 优先级 5；.ioc 开启 ABOM，自动重发保持关闭，已经 CubeMX 生成并导入。下列“CAN 未启动”记录均为旧版历史状态。

- 2026-11-17 上板跟进：用户烧录后首次 OpenOCD 连接失败，用户要求重试后 CMSIS-DAPv2 连接成功。已只读核对 Flash 与 build/Debug/Power.elf 一致；调试器供电、未接 VIN 主电源/负载条件下，初始化与周期读取正常。
- 2026-11-17：基于用户网表新增 Shared/BSP/i2c_bus 与 App/Power.c/.h、Power_config.h、robot.c/.h。用户已确认实物采样电阻为 **1 mΩ**。
- Power_Init 读取并校验 INA228 厂商/器件 ID，写入配置并回读；Power_Update 分别读取压差、电压、电流、功率。主循环每 10 ms 调度一次，无 DMA、无 CAN 收发。
- .ioc 设置 I2C1 PB6/PB7、400 kHz，HSE 8 MHz 经 PLL ×9 到 72 MHz，APB1 36 MHz。已在隔离生成目录运行 CubeMX 6.15.0，并导入生成的 main.c/i2c.c；调用写在 USER CODE 区。
- 2026-11-17 后续：按用户要求将 CAN 配置为 1 Mbps，PB8 RX / PB9 TX 重映射。PCLK1=36 MHz，Prescaler=3、BS1=9、BS2=2、SJW=1，采样点约 83.3%；已通过 CubeMX 生成 can.c。当前 App 仍未启动 CAN 或发送报文。

## 已验证

- 最新 M3508 版本：Power 的 build/Debug 和 build/codex-debug 构建通过，无警告（FLASH 20316 B / RAM 1976 B）；九组主机测试通过，保留原功率计七组并增加真实 DJI 驱动的路由/速度/限幅/离线恢复与忙邮箱/初始化失败测试。主机只模拟 HAL/I2C 与 CAN BSP 边界，不代表真实电机动态。Gimbal_test、Chassis_test 共享驱动增量 ARM 构建通过。
- CubeMX 生成的 can.c 已设置 RX0 优先级 5、使能 USB_LP_CAN1_RX0_IRQn、AutoBusOff=ENABLE；中断入口调用 HAL_CAN_IRQHandler，SysTick 保留 Robot_ControlTick。下面的实板结果只适用于旧版功率计固件。

- 本次实板只读检查：CPU 前后均为 running，WORKAREASIZE=0；Flash 两个加载段（12080 B + 40 B）逐字节匹配用户 build/Debug/Power.elf。没有 halt、reset、烧录或调用目标函数；结束后关闭 OpenOCD，保持 CPU 运行。
- INA228 厂商 ID=0x5449、器件 ID=0x2281，s_i2c 正确指向 hi2c1，HAL I2C ErrorCode=0。约 10.432 秒监测取得 282 组连续两次内存读取相同的反馈，全部 status=HAL_OK；38 组恰逢更新导致两次读取不同而丢弃，不计为通信错误。观察到 sample_tick 以 10 ms 步进，监测读数年龄最大 12 ms。
- 用户确认只接调试器、未接主电源与负载。观察 VIN 2.79297～2.80137 V、电流 −4.375～+5.625 mA、压差 −4.375～+5.625 μV、功率 2～21 mW；仅作为当前无主电源工况记录，不能据此确认测量精度或认定为负载功耗。
- 实读 RCC_CR=0x03036883、RCC_CFGR=0x001D040A：HSE/PLL ready，按 8 MHz HSE 得 SYSCLK 72 MHz、APB1 36 MHz；I2C_CR2=0x24、CCR=0x801E、TRISE=0x0B，对应 400 kHz 配置。CAN_BTR=0x00180002、AFIO_MAPR=0x02004000，位时序为 1 Mbps、PB8/PB9 重映射；CAN 仍处于初始化状态，符合 App 未启动收发的现状。
- 本次原始记录在 build/live-check/report.json、monitor.json（构建目录忽略）；只读采集脚本在 build/read_power_live.py、monitor_power_live.py。使用本次匹配 ELF 的符号地址解释数据，不可把固定地址直接用于别的固件版本。
- 已静态核对网表引脚及 TI INA228 手册的字节序、有效位、校准公式与寄存器地址。
- CubeMX 命令行生成成功，生成代码为 HSE/PLL 72 MHz、I2C 400000 Hz；只导入生成的两份源文件及归一化 .ioc，未改第三方驱动。
- ARM GNU 13.3.1 Debug 全量构建通过，无编译警告；FLASH 12076 B / RAM 1744 B，ELF 中保留 Power_Init / Power_Update / Robot_Loop 及唯一 s_feedback。
- `python tests/run_tests.py` 七组主机测试通过：地址/字节序/参数检查、1 mΩ 校准与回读、分寄存器访问及单位、负电流和二十位边界、逐个读取失败保留旧样本与恢复、ID/配置错误、10 ms 调度及 tick 回绕。
- 按根笔记要求，Gimbal_test、Chassis_test 两个原有 ARM 工程重新全量编译通过；它们未引用新增 I2C BSP，不将此结果宣称为 F4 上 I2C 硬件验证。
- CAN 1 Mbps 更新后 ARM Debug 增量编译通过；已核对生成代码的位时序、GPIOB 引脚与 __HAL_AFIO_REMAP_CAN1_2()。
- CAN 更新后固件 FLASH 12120 B / RAM 1744 B。CubeMX 的 CalculateBaudRate 缓存显示 999999，是浮点计算取整；实际整数分频 36000000 / (3 × (1+9+2)) 精确等于 1000000 bit/s。

## 未验证

- 最新 M3508 版本未烧录；CAN 上板收发、电机转向/1000 RPM 跟踪、PID 整定、实物断线恢复、SysTick 控制耗时及中断延迟均未验证。

- 已核对烧录内容与 ELF 一致，并观察到 HSE/PLL 就绪、400 kHz 配置下应用读取正常；仍未用示波器验证时钟频率、SCL/SDA 上升时间或时序裕量。主电源电压、已知负载电流/功率精度、零点标定和负载温升未验证。
- CAN 已配置 PB8/PB9、1 Mbps，新版 App 已接入启动/接收/控制发送，实际总线收发、终端与对端通信尚未验证。
- ALERT/PA3、LED/PA4 暂未启用，不影响轮询测量；USB 未启用，USB 时钟不作为本阶段配置目标。

## 对外接口（给其他模块用）

- Robot_Init 初始化功率计和 CAN 电机；Robot_Loop 负责功率计主循环调度；Robot_ControlTick 由 SysTick 调用，仅做电机闭环及非等待发送。电机状态只存在 s_motor，目标速度字段 speed.target_rpm、实际速度 speed_rpm 均为转子 RPM。控制初始化失败时 s_motor_ready=0，不发控制帧。

- Shared/BSP/i2c_bus.h：I2C_Bus_ReadReg / WriteReg，七位设备地址、八位寄存器地址、指定数据长度；ReadRegU16 / WriteRegU16 为高字节在前的十六位访问，读取 ID 复用 ReadRegU16。
- App/Power.h：Power_Init(I2C_HandleTypeDef *)、Power_Update(uint32_t now_tick)、Power_GetFeedback(PowerFeedback_t *)。调用应在同一个主循环或任务内串行执行，不保证跨中断/跨任务原子访问。
- PowerFeedback_t 保存 voltage_v（VIN）、shunt_voltage_v（VIN−VOUT）、current_a、power_w、sample_tick、两项 ID 和 status。只有 status=HAL_OK 表示本次完整读取成功；BUSY 表示尚无新样本；错误时保留上次测量数据和时间戳。初始化失败后修复接线，再调用 Power_Init 或复位 MCU。
- 仅保存一份持久测量反馈 s_feedback；供调试器查看。GetFeedback 用于向其他模块传值，不另建全局副本。

## 注意事项和踩过的坑

- 固件收到 ID1 有效反馈后会主动转动；1000 为转子 RPM，按共享驱动减速比输出轴约 52.1 RPM。C620 ID 必须为 1；同总线不要再由其他控制器发送 0x200。反馈恢复后会自动继续运行。
- 本次重新构建了 build/Debug/Power.elf，已不再匹配板上此前烧录的旧固件；旧 VOFA 固定地址和旧只读采集脚本不得直接用于新固件，后续按实际烧录 ELF 重新取地址。

- 本次只读检查：OpenOCD 0.12.0 自动选择 CMSIS-DAPv2，识别 VID:PID=0d28:0202，但报 `CMSIS-DAP command mismatch. Sent 0x0 received 0x5` / `CMD_INFO failed`；改用 HID 后报 `unable to find a matching CMSIS-DAP device`。未证明是固件故障；初步怀疑探针接口兼容/占用/残留状态，未排除。另该版本使用 `gdb_port/tcl_port/telnet_port` 旧命令，不能用带空格的新版语法。连接前设 WORKAREASIZE=0、清空 examine-end 回调，不复位、不烧录；没有进入目标数据读取阶段。
- 网表 R5 的 Value 为 1 mΩ，型号为 2 mΩ；已按用户确认采用 1 mΩ。CURRENT_LSB=312.5 μA/计数，ADCRANGE=0，SHUNT_CAL=4096；理论电流量程约 ±163.84 A **不是整板额定电流**。
- ADC_CONFIG=0xB6D9：电压和压差各 280 μs、平均四次、连续转换，约 2.24 ms 一次。每次先检查转换完成标志，之后逐个读取，不能跨寄存器一次突发读取。
- VSHUNT/CURRENT 为二十位有符号数，位于 23:4；VBUS 右移四位；POWER 使用完整二十四位无符号数。反向功率方向须结合 current_a 判断。
- 完整读取后一次发布保证软件不会暴露半组失败数据，但连续转换下各寄存器读取并不保证来自同一转换周期；不宣称同步瞬时采样。
- BSP 向 HAL 传 5 ms 超时；HAL 内部某些总线状态等待另有固定超时，不能据此宣称整个函数最多阻塞 5 ms。不要在控制中断中调用。
- .ioc 修改必须经 CubeMX 重新生成才生效，本次已生成；以后改引脚、时钟和速度也必须重新生成，不能只改 .ioc 或手改初始化函数。
- CubeMX 启动/生成期间发现原目录 .ioc 被写回旧参数，原因未独立确认；已把隔离目录成功生成后归一化的 .ioc 复制回去，并核对与生成源文件一致。以后外部工具开着工程时注意重新读取磁盘配置。
- VIN 输入电容网表为 25 V，SMAJ36A 不是 25 V 钳位；整板耐压/载流能力不能按芯片最大值认定。

## 下一步

- OpenOCD 重试已成功，当前匹配的 ELF 为 build/Debug/Power.elf（用户构建时间 2026-11-17 20:14）。后续用户重新构建/烧录必须重新匹配 ELF 与符号地址。
- 用低压限流电源验证 ID、3.3 V 和 VIN，再接已知负载核对电流、功率。查看 s_feedback，先确认 status=HAL_OK 且 sample_tick 持续增长。
- 用户烧录新版后，先核对 C620 ID=1 和总线终端，观察 s_motor.feedback_count 递增，再检查 s_motor.speed_rpm 跟踪 1000；上板整定 PID 并量测控制中断耗时。
