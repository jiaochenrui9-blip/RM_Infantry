# RM_Infantry 项目笔记（总）

> 给各个对话看：总进度、跨模块约定、硬件事实。各工程自己的进度写在各自目录的 notes.md。
> 最后更新：2026-10-31（云台对话：v2 第 2 步云台 IMU 模式代码改好，主机模型测试通过）

## 1. 项目组织

当前阶段：底盘、云台各写一个测试工程练手；整车拼好后合并联调。代码自己从 0 写，开源工程只借鉴思路。

```
RM_Infantry/
├── notes.md                    ← 本文件
├── 硬件资料/                    ← C 板、C620、M3508、GM6020 手册（见其 README）
├── Shared/                     ← 各工程共用的 BSP / Devices / Algorithm（用法见第 7 节）
├── Test_Module/
│   ├── Gimbal/Gimbal_test/     ← 云台测试工程，进度见其 notes.md
│   └── Chassis/                ← 底盘测试工程，未开始
└── （以后）Infantry/            ← 合并工程
```

## 2. 架构约定（底盘、云台都按这个写，合并时直接搬模块）

| 层 | 内容 | 位置 |
|---|---|---|
| BSP | 只封装 HAL 外设：can_bus；以后 USB 虚拟串口、SPI、UART、DWT | Shared/BSP |
| Devices | 器件驱动：DJI 电机、BMI088、IST8310；以后 DR16 | Shared/Devices |
| Algorithm | 与硬件无关：PID、user_math（限幅、角度回绕）、Attitude（Mahony 姿态解算）；以后斜坡等 | Shared/Algorithm |
| App 模块 | gimbal（Gimbal_test 已有）、chassis；以后 shoot、ins、robot_cmd | 各工程 App/ |
| 装配层 | robot.c：持有 CAN manager、初始化各模块、1 ms 调度、CAN 接收分发 | 各工程 App/ |

规则：

1. 下层不 include 上层；App 模块之间也互不 include，只由 robot.c（合并后加 robot_cmd）连接。例外：ins 这种没有指令、只往外给数据的传感器模块算下层，别的模块可以 include 它的头文件、用它的 Feedback 类型，但不调它的函数，数据仍由 robot.c 传进来。
2. 每个 App 模块对外只有 `Xxx_Init()`、`Xxx_SetCmd(const XxxCmd_t *)`、`Xxx_Update(uint32_t now_tick)`、`Xxx_GetFeedback(XxxFeedback_t *)`，以及定义在 `xxx.h` 里的 Cmd / Feedback 结构体。命令从哪来（调试变量、遥控、视觉、另一块板经 CAN 转来）模块不关心，所以单板、双板方案都能接。要用传感器模块的数据时（如云台 IMU 模式要 ins 的姿态），再加一个 `Xxx_SetYyy()`，参数直接用那个模块的 Feedback 类型（如 `Gimbal_SetImu(const InsFeedback_t *)`），robot.c 每周期把 GetFeedback 的结果传进去。2026-10-31 改：原来要求本模块再定义一份字段相同的结构体、由 robot.c 逐字段拷，主循环里太乱。没有指令的模块（如 ins）不要 SetCmd。
3. 一路 CAN 只建一个 `DJI_MotorManager_t`，由 robot.c 持有，`Xxx_Init()` 时传给模块，模块把自己的电机注册进去；每个控制周期由 robot.c 对每路 CAN 调一次 `DJI_MotorManager_SendAll()`。原因：manager 负责检查同一路上的 ID / 槽位冲突；SendAll 会把同一控制帧里未注册的槽位发 0，两个 manager 发同一个帧 ID 会互相清零。
4. 模块与调度方式无关：`Xxx_Update()` 固定周期调用，dt 用宏写死。测试工程用裸机 1 kHz 循环；合并工程放进 FreeRTOS 任务，模块代码不改。
5. CubeMX 生成的 main.c 只加两处调用：外设初始化后 `Robot_Init()`，`while(1)` 里一直调 `Robot_Loop()`（内部按 HAL_GetTick 每 1 ms 跑一次）；HAL 回调（如 CAN 接收）放 robot.c。参考实现见 Gimbal_test/App/robot.c。
6. 和具体车有关的参数放模块自己的 `xxx_config.h`（电调 ID、方向、编码器中值、限位、PID、限幅）；全车 CAN 分配登记在第 4 节。
7. 外设一律在 .ioc 里配置后生成代码，不手写初始化代码。
8. 改 Shared 里的代码要在这里记一笔，两个测试工程都要重新编译。

## 3. 统一单位与正方向

- 角度用度（deg），角速度用度/秒（dps）。GM6020 转速反馈 rpm × 6 = dps。
- 云台：yaw 俯视逆时针为正，pitch 抬头为正。
- 底盘（建议）：x 向前、y 向左、wz 俯视逆时针为正（和云台 yaw 同向）。
- 电机装反时用 `motor_direction = -1` 修正，让逻辑正方向符合上面的约定。
- 云台 Feedback 提供 `yaw_rel_deg`（云台相对底盘正前方的角度），底盘跟随云台、运动坐标变换都用它（v1 代码里还叫 `yaw_deg`，v2 改名）。
- ins（惯导）输出也按这套：云台头坐标系 x 朝枪口、y 朝左、z 朝上；yaw 俯视逆时针为正、pitch 抬头为正。没有磁力计时 yaw 以上电朝向为 0，会漂。

## 4. CAN 分配（合并前整体检查）

| 总线 | 设备 | 电调 ID | 反馈 ID | 控制帧 / 槽位 | 状态 |
|---|---|---|---|---|---|
| CAN1（代码默认，待按接线确认） | GM6020 yaw | 1 | 0x205 | 0x1FE / 0 | 云台工程已用 |
| CAN1（代码默认，待按接线确认） | GM6020 pitch | 2 | 0x206 | 0x1FE / 1 | 云台工程已用 |
| 待定 | M3508 ×4（底盘） | 1~4 | 0x201~0x204 | 0x200 / 0~3 | 底盘工程定 |

- 同一路总线上：GM6020 ID 1~4 的反馈 0x205~0x208 和 M3508/M2006 ID 5~8 的反馈相同，不能同时用。
- GM6020 电压控制帧 0x1FF 与 M3508/M2006 ID 5~8 的控制帧相同；现有驱动用电流控制 0x1FE/0x2FE，不冲突。
- 1 Mbps 下一个 8 字节标准帧约 111~135 bit，每毫秒理论最多约 7~9 帧。每个 DJI 电机的反馈就占 1 帧/ms，再加控制帧。合并时按负载把电机分到两路 CAN。

## 5. C 板硬件事实（已对照用户手册 IO 表和原理图）

| 功能 | 引脚 | 备注 |
|---|---|---|
| CAN1（2-pin 口） | PD0 RX / PD1 TX | 不是 PB8/PB9（那是 FPC 口的 I2C1） |
| CAN2（4-pin 口） | PB5 RX / PB6 TX | |
| DBUS 遥控 | USART3_RX PC11 | 板上已反相，手册写 100 kbps |
| BMI088 | SPI1：PB3 SCK / PA7 MOSI / PB4 MISO；CS：加速度计 PA4、陀螺仪 PB0；INT1：加速度计 PC4、陀螺仪 PC5 | SPI 最高 10 MHz |
| IMU 加热 | TIM10_CH1 PF6 | 手册建议比板子正常工作温度高 15~20 ℃ |
| IST8310 | I2C3：PA8 SCL / PC9 SDA；RSTN PG6、DRDY PG3 | 地址 0x0E |
| RGB LED | PH10 蓝 / PH11 绿 / PH12 红 | 高电平亮 |
| 用户按键 | PA0 | |
| 蜂鸣器 | TIM4_CH3 PD14 | 无源，额定 4 kHz |
| 电池电压 | ADC PF10 | |
| micro USB | PA11 DM / PA12 DP | 做 USB 虚拟串口需要 48 MHz：PLLQ 改 7（现在是 4，即 84 MHz） |
| 串口 | 丝印 UART1 = USART6（PG14 TX / PG9 RX）；丝印 UART2 = USART1（PA9 TX / PB7 RX） | 外壳丝印和 STM32 编号不对应 |
| 晶振 | HSE 12 MHz | |

- 原理图上两路 CAN 收发器（TJA1044）后面都有 120 Ω 电阻，应是板载终端电阻 → 总线另一端（最远的电机）打开终端电阻即可。
- CAN 收发器 TJA1044GTK/3：VCC 接 VCC_5V（24 V 经 SY8501 降压，或 USB 的 VBUS 经 DSS34 二极管），VIO 接 3.3 V，STB 接地。只靠调试器 3.3 V 供电时 CAN 发不出去，会进 bus-off（2026-10-30 实测）。两路 CAN 的 Automatic Bus-Off Management 都没开（can.c 里 `AutoBusOff = DISABLE`），bus-off 后要复位才能恢复。
- GM6020：拨码 1~3 位设 ID（1~7），第 4 位是 CAN 终端电阻。反馈 1 kHz，编码器 0~8191，转速单位 rpm。电流控制 ±16384 对应 ±3 A，需要固件 ≥1.0.11.2，并在 RoboMaster Assistant（≥2.7）里打开电流环。

## 6. 各模块进度

| 模块 | 工程 | 状态 |
|---|---|---|
| 云台 | Test_Module/Gimbal/Gimbal_test | App 精简版 v1（robot / gimbal / test_cmd，只有编码器定角度，含速度前馈，499 行）：2026-10-29 本机清理编译无警告，主机闭环测试通过；未上板，参数待标定。v2（ins + IMU 模式）接口已定：第 1 步 ins 已接入 robot.c，2026-10-30 首次上板（只接调试器 3.3 V）程序运行正常，ins 桌面验收通过（手转符号都对）；第 2 步云台 IMU 模式接口 2026-10-30 细化（限位算法已在主机上验证），用户写的第一版（gimbal / robot）2026-10-31 审查后由 Claude 改好：ARM 清理编译无警告，主机模型测试通过，还没上板，见其 notes.md。带 IMU 的旧完整版备份在 Test_Module/Gimbal/Backup |
| 底盘 | Test_Module/Chassis | 未开始；按第 2、3 节约定写，公共代码按第 7 节引用 Shared |

## 7. Shared 的用法与改动记录

新工程（如底盘测试）照 Gimbal_test 顶层 CMakeLists.txt 的写法引用，`../` 的层数按工程所在位置改：

```cmake
get_filename_component(SHARED_DIR "${CMAKE_CURRENT_SOURCE_DIR}/../../../Shared" ABSOLUTE)
target_sources(${CMAKE_PROJECT_NAME} PRIVATE ${SHARED_DIR}/BSP/can_bus.c ...)
target_include_directories(${CMAKE_PROJECT_NAME} PRIVATE ${SHARED_DIR}/BSP ...)
```

- Shared 不在工程目录内，CLion 项目树默认看不到；需要的话用 Tools → CMake → Change Project Root 把根目录设为 RM_Infantry。
- Shared 里的代码 include 各工程自己的 `main.h`（STM32F4 HAL），只适用于 F4 + HAL 的工程（C 板都满足）。

| 日期 | 改动 | 验证 |
|---|---|---|
| 2026-10-27 | Gimbal_test/UserModules 整体移到 Shared，代码内容未改；Gimbal_test 的 CMake 改用 `SHARED_DIR` | Gimbal_test 清理重编无警告，text/data/bss 与移动前相同 |
| 2026-10-27 | 加入 `Shared/Devices/BMI088` 与 `Shared/Algorithm/Mahony`；参考 C 板 SPI1 与独立六轴姿态算法 | Gimbal_test 清理编译通过，主机模拟通过；当前 .ioc 尚无 SPI1/DMA、未接到主程序，未做板上验证；底盘测试工程尚不存在 |
| 2026-10-27 | Gimbal_test 用 CubeMX 配置 BMI088 SPI1、片选、DMA 和数据就绪输入，并生成代码 | 清理编译通过；BMI088 初始化、采样回调及 Mahony 尚未接入，未做板上验证 |
| 2026-10-27 | 按用户要求改为移植 C:\Clion\IMU\Core\Algorithm 的原版 Attitude，并补入依赖的 IST8310；DRDY 改由主循环发起读取 | Attitude 原文件哈希一致，目标单文件编译及主机测试通过；完整构建受 Gimbal_Update 声明不一致阻塞；I2C3 尚未配置，未做硬件验证 |
| 2026-10-27 | 新增 `Shared/Algorithm/user_math.c/.h`（`Math_Clamp`、`Math_WrapDeg180`）；其余 Shared 文件未改 | Gimbal_test 完整清理编译无警告（IST8310 未被调用，链接时被裁掉）；主机测试含回绕边界用例，通过 |
| 2026-10-29 | BMI088 加 `BMI088_SetGyroOffset()`（用户写）：ins 用 DMA 样本自己求零偏后写回驱动，代替阻塞的 `BMI088_CalibrateGyro` | Gimbal_test 清理编译无警告；主机测试零偏估计准确；未上板 |
| 2026-10-29 | Attitude 加 `Attitude_InitFromAcc()`：没有磁力计时用静止时的加速度平均值初始化 roll / pitch（yaw = 0），算法和 `Attitude_InitializeQuaternion` 前半段相同；原有函数未改 | Gimbal_test 清理编译无警告；主机测试（真 Attitude.c）初始化后 pitch 与真值一致；未上板 |
