# RM_Infantry 项目笔记（总）

> 给各个对话看：总进度、跨模块约定、硬件事实。各工程自己的进度写在各自目录的 notes.md。
> 最后更新：2026-11-05（IMU 加热恒温代码改好、待上板，见第 6、8 节）。2026-11-04（IMU 加热恒温开工：查到加热电源只来自 24 V，记在第 5 节。同日早些时候：盘点了还没做的，见第 8 节；第 4 节加了发射机构要和云台电机分路。同日早些时候：云台电机可能从 GM6020 换成灵足 RS05：RS05 驱动在用户的 RS05 测试工程 `C:\Clion\RS05` 里重写成只用电流模式，放进 `Shared/Devices/RS05`；云台工程还是 GM6020，CMake 编了 rs05.c 但没有调用。同日查了 RS05 的零点：编码器量的是转子，上电后的角度可能差整数个 46.45°，中值能不能写死待测。第 2、4、5、6、7 节有改动）

## 1. 项目组织

当前阶段：底盘、云台各写一个测试工程练手；整车拼好后合并联调。代码自己从 0 写，开源工程只借鉴思路。

整车按步兵的逻辑（2026-11-01 用户定）：云台板是主控，底盘接收云台主控发来的速度和模式（`ChassisCmd_t`，见 Chassis_test/notes.md「接口」）。双板时经板间 CAN 帧 0x110 传（`Shared/Link/chassis_link`，两块板编译同一份；云台板这边见 Gimbal_test/notes.md「板间通信」）；单板时 robot_cmd 直接填同一个结构体。

```
RM_Infantry/
├── notes.md                    ← 本文件
├── 硬件资料/                    ← C 板、C620、M3508、GM6020 手册（见其 README）
├── Shared/                     ← 各工程共用的 BSP / Devices / Algorithm / Link（用法见第 7 节）
├── Test_Module/
│   ├── Gimbal/Gimbal_test/     ← 云台测试工程，进度见其 notes.md
│   ├── Gimbal/HostTest/        ← 云台主机测试：电脑上编译真代码 + 电机模型，改了云台代码就跑一遍（见其 README）
│   ├── CarSim/                 ← 整车主机仿真：云台板 + 底盘板真代码 + 整车模型，测各模式的配合（见其 README）
│   └── Chassis/Chassis_test/   ← 底盘测试工程，进度见其 notes.md
└── （以后）Infantry/            ← 合并工程
```

## 2. 架构约定（底盘、云台都按这个写，合并时直接搬模块）

| 层 | 内容 | 位置 |
|---|---|---|
| BSP | 只封装 HAL 外设：can_bus；以后 USB 虚拟串口、SPI、UART、DWT | Shared/BSP |
| Devices | 器件驱动：DJI 电机、BMI088、IST8310、RS05（2026-11-04 加，Gimbal_test 编了但还没调用）；以后 DR16 | Shared/Devices |
| Algorithm | 与硬件无关：PID、user_math（限幅、角度回绕）、Attitude（Mahony 姿态解算）；以后斜坡等 | Shared/Algorithm |
| Link | 板间通信：帧格式、打包 / 解包。chassis_link（云台板发底盘指令、底盘板收），两块板编译同一份 | Shared/Link |
| App 模块 | gimbal（Gimbal_test 已有）、chassis；以后 shoot、ins、robot_cmd | 各工程 App/ |
| 装配层 | robot.c：持有 CAN manager、初始化各模块、1 ms 调度、CAN 接收分发 | 各工程 App/ |

规则：

1. 下层不 include 上层；App 模块之间也互不 include，只由 robot.c（合并后加 robot_cmd）连接。例外：ins 这种没有指令、只往外给数据的传感器模块算下层，别的模块可以 include 它的头文件、用它的 Feedback 类型，但不调它的函数，数据仍由 robot.c 传进来。
2. 每个 App 模块对外只有 `Xxx_Init()`、`Xxx_SetCmd(const XxxCmd_t *)`、`Xxx_Update(uint32_t now_tick)`、`Xxx_GetFeedback(XxxFeedback_t *)`，以及定义在 `xxx.h` 里的 Cmd / Feedback 结构体。命令从哪来（调试变量、遥控、视觉、另一块板经 CAN 转来）模块不关心，所以单板、双板方案都能接。要用传感器模块的数据时（如云台 IMU 模式要 ins 的姿态），再加一个 `Xxx_SetYyy()`，参数直接用那个模块的 Feedback 类型（如 `Gimbal_SetImu(const InsFeedback_t *)`），robot.c 每周期把 GetFeedback 的结果传进去。2026-10-31 改：原来要求本模块再定义一份字段相同的结构体、由 robot.c 逐字段拷，主循环里太乱。没有指令的模块（如 ins）不要 SetCmd。例外（2026-11-03 用户选的）：底盘的 `ChassisMode_e`、`ChassisCmd_t` 就是板间帧的内容，定义在 `Shared/Link/chassis_link.h`，两块板共用一份；底盘模块的头文件 include 它，只用类型。
3. 一路 CAN 只建一个 `DJI_MotorManager_t`，由 robot.c 持有，`Xxx_Init()` 时传给模块，模块把自己的电机注册进去；每个控制周期由 robot.c 对每路 CAN 调一次 `DJI_MotorManager_SendAll()`。原因：manager 负责检查同一路上的 ID / 槽位冲突；SendAll 会把同一控制帧里未注册的槽位发 0，两个 manager 发同一个帧 ID 会互相清零。
4. 模块与调度方式无关：`Xxx_Update()` 固定周期调用，dt 用宏写死。测试工程用裸机 1 kHz 循环；合并工程放进 FreeRTOS 任务，模块代码不改。
5. CubeMX 生成的 main.c 只加两处调用：外设初始化后 `Robot_Init()`，`while(1)` 里一直调 `Robot_Loop()`（内部按 HAL_GetTick 每 1 ms 跑一次）；HAL 回调（如 CAN 接收）放 robot.c。参考实现见 Gimbal_test/App/robot.c。
6. 和具体车有关的参数放模块自己的 `xxx_config.h`（电调 ID、方向、编码器中值、限位、PID、限幅）；全车 CAN 分配登记在第 4 节。
7. 外设一律在 .ioc 里配置后生成代码，不手写初始化代码。CubeMX 也能命令行无界面生成，做法见 Chassis_test/notes.md「工程是怎么建的」。
8. 改 Shared 里的代码要在这里记一笔，两个测试工程都要重新编译。

## 3. 统一单位与正方向

- 角度用度（deg），角速度用度/秒（dps）。GM6020 转速反馈 rpm × 6 = dps。
- 云台：yaw 俯视逆时针为正，pitch 抬头为正。
- 底盘：x 向前、y 向左、wz 俯视逆时针为正（和云台 yaw 同向）；平移速度用 m/s，wz 用度/秒。每个轮子的正方向 = 让底盘前进的方向。
- 底盘指令里的平移速度在云台朝向的坐标系里（只看 yaw：沿枪口为 x、枪口左边为 y），底盘用指令里的 `gimbal_yaw_rel_deg` 转到自己的坐标系；底盘自己的角速度由底盘按模式算，不在指令里。
- 电机装反时用 `motor_direction = -1` 修正，让逻辑正方向符合上面的约定。
- 云台 Feedback 提供 `yaw_rel_deg`（云台相对底盘正前方的角度），底盘跟随云台、运动坐标变换都用它（v1 代码里还叫 `yaw_deg`，v2 改名）。
- ins（惯导）输出也按这套：云台头坐标系 x 朝枪口、y 朝左、z 朝上；yaw 俯视逆时针为正、pitch 抬头为正。没有磁力计时 yaw 以上电朝向为 0，会漂。

## 4. CAN 分配（合并前整体检查）

| 总线 | 设备 | 电调 ID | 反馈 ID | 控制帧 / 槽位 | 状态 |
|---|---|---|---|---|---|
| CAN1（代码默认，待按接线确认） | GM6020 yaw | 1 | 0x205 | 0x1FE / 0 | 云台工程已用 |
| CAN1（代码默认，待按接线确认） | GM6020 pitch | 2 | 0x206 | 0x1FE / 1 | 云台工程已用 |
| CAN2（代码默认，待按接线确认） | M3508 ×4（底盘） | 1~4 | 0x201~0x204 | 0x200 / 0~3 | 底盘工程已用（2026-11-01） |
| 两块板之间那路（两块板都默认 CAN1；云台板上和云台电机同一路） | 板间指令：云台板 → 底盘板 | — | — | 0x110，DLC 8，1 kHz | 代码在 Shared/Link/chassis_link。底盘板接收已写（2026-11-01）；云台板发送 `ChassisLink_SendCmd` 已写（2026-11-03，用户写第一版、Claude 按审查改好），主机测试通过，两板联调待做 |

- 同一路总线上：GM6020 ID 1~4 的反馈 0x205~0x208 和 M3508/M2006 ID 5~8 的反馈相同，不能同时用。
- GM6020 电压控制帧 0x1FF 与 M3508/M2006 ID 5~8 的控制帧相同；现有驱动用电流控制 0x1FE/0x2FE，不冲突。
- 1 Mbps 下一个 8 字节标准帧约 111~135 bit，每毫秒理论最多约 7~9 帧。每个 DJI 电机的反馈就占 1 帧/ms，再加控制帧。合并时按负载把电机分到两路 CAN。
- 底盘默认放 CAN2，和云台分在两路：4 帧反馈 + 1 帧控制 / ms，按上面的帧长算负载约 56%~68%。两个测试工程是分开烧的，现在不冲突；Chassis_test 里改总线只改 robot.c 传给 `Chassis_Init` 的 manager。
- 板间帧 0x110 不在大疆电机的过滤范围（0x200~0x20F）里，要单独配过滤器组：CAN1 用组 2、CAN2 用组 16（组 1 / 15 是 can_bus.h 里给扩展帧设备留的）。0x110 比大疆的控制帧（0x1FE~0x2FE）优先级高。
- 板间那路（双板默认）：两个 GM6020 的反馈 + 0x1FE + 0x110 = 4 帧/ms，按上面的帧长约 44%~54%。底盘板的 CAN1 也挂在这路上，它在这路上什么也不发；它的大疆过滤器会收进云台电机的 0x205 / 0x206，没注册电机，直接忽略。以后发射机构的电机别再往这路加。
- 两个工程的 CAN 都是 `AutoRetransmission = DISABLE`：每帧只发一次，出错或没有应答就丢掉，下一周期照常发。
- RS05（云台可能换成它，还没换；细节见 Gimbal_test/notes.md「换 RS05」）：
  - 私有协议是扩展帧，每个电机每毫秒一帧指令 + 一帧应答，每帧 131~160 bit。两个 RS05 加 0x110 在一路上约 64%~78%。
  - 应答（类型 2）的前 11 位是 0x080~0x0BF，比 0x110 和发给 RS05 的所有指令都优先。`AutoRetransmission = DISABLE` 时，C 板的帧和应答同时起步就会输掉仲裁、直接丢，所以换的那一路要在 .ioc 里打开自动重发。
  - 过滤器用 CAN1 组 1、CAN2 组 15（can_bus.h 预留的），只收类型 2。
  - F4 每路只有 3 个发送邮箱，RS05 每个电机每周期占一个。两个 RS05 加 0x110 正好 3 个，这一路上别再加别的周期帧。
- 发射机构（还没写；按两个 M3508 摩擦轮 + 一个 M2006 拨弹盘算：0x200 + 3 帧反馈 = 4 帧标准帧/ms，约 44%~54%）要和云台电机分在两路：和两个 GM6020 加 0x110 放同一路约 89%~108%，和两个 RS05 放同一路约 97%~118%。云台用 GM6020、或者 RS05 留在 CAN1 时，发射机构放云台板 CAN2；RS05 挪到 CAN2 时，发射机构放 CAN1（加 0x110 约 56%~68%）。2026-11-04 盘点时按上面的帧长算的。

## 5. C 板硬件事实（已对照用户手册 IO 表和原理图）

| 功能 | 引脚 | 备注 |
|---|---|---|
| CAN1（2-pin 口） | PD0 RX / PD1 TX | 不是 PB8/PB9（那是 FPC 口的 I2C1）。插座线序 1 CANL、2 CANH，和 C620 自带的 2-pin CAN 线一样；两个插座 |
| CAN2（4-pin 口） | PB5 RX / PB6 TX | 插座线序 1 5V、2 GND、3 CANH、4 CANL；两个插座 |
| DBUS 遥控 | USART3_RX PC11 | 板上已反相，手册写 100 kbps |
| BMI088 | SPI1：PB3 SCK / PA7 MOSI / PB4 MISO；CS：加速度计 PA4、陀螺仪 PB0；INT1：加速度计 PC4、陀螺仪 PC5 | SPI 最高 10 MHz |
| IMU 加热 | TIM10_CH1 PF6 | 手册建议比板子正常工作温度高 15~20 ℃。高电平加热，满功率 0.58 W。加热电源来自 VCC_5V_M，只由 24 V 降压得到：只接调试器或只插 USB 时不会加热（2026-11-04 查原理图） |
| IST8310 | I2C3：PA8 SCL / PC9 SDA；RSTN PG6、DRDY PG3 | 地址 0x0E |
| RGB LED | PH10 蓝 / PH11 绿 / PH12 红 | 高电平亮 |
| 用户按键 | PA0 | |
| 蜂鸣器 | TIM4_CH3 PD14 | 无源，额定 4 kHz |
| 电池电压 | ADC PF10 | |
| micro USB | PA11 DM / PA12 DP | 做 USB 虚拟串口需要 48 MHz：PLLQ 改 7（现在是 4，即 84 MHz） |
| 串口 | 丝印 UART1 = USART6（PG14 TX / PG9 RX）；丝印 UART2 = USART1（PA9 TX / PB7 RX） | 外壳丝印和 STM32 编号不对应 |
| 晶振 | HSE 12 MHz | |

- 原理图上两路 CAN 收发器（TJA1044）后面都有 120 Ω 电阻，应是板载终端电阻 → 总线另一端（最远的电机）打开终端电阻即可。板间那路两头都是 C 板，两个板载电阻正好是两个终端，这路上的电机终端电阻都关。
- CAN 收发器 TJA1044GTK/3：VCC 接 VCC_5V（24 V 经 SY8501 降压，或 USB 的 VBUS 经 DSS34 二极管），VIO 接 3.3 V，STB 接地。只靠调试器 3.3 V 供电时 CAN 发不出去，会进 bus-off（2026-10-30 实测）。Gimbal_test 两路 CAN 的 Automatic Bus-Off Management 都没开（can.c 里 `AutoBusOff = DISABLE`），bus-off 后要复位才能恢复；Chassis_test 两路都开了（2026-11-01，还没上板验证）。
- GM6020：拨码 1~3 位设 ID（1~7），第 4 位是 CAN 终端电阻。反馈 1 kHz，编码器 0~8191，转速单位 rpm。电流控制 ±16384 对应 ±3 A，需要固件 ≥1.0.11.2，并在 RoboMaster Assistant（≥2.7）里打开电流环。
- C620 + M3508：电调 ID 1~8 用 SET 键设（步骤见 Chassis_test/notes.md）；电流 ±16384 对应 ±20 A；反馈 0x200 + ID、默认 1 kHz，转速是转子 rpm（M3508 减速比 3591/187 ≈ 19.2）；电调上有 120 Ω 终端电阻开关；不能同时插 CAN 线和 PWM 线。
- 灵足 RS05（手册在 `C:\Users\Lenovo\Desktop\Data\01_硬件资料\手册\RS05\RS05使用说明书_260713.pdf`，没放进 硬件资料/）：
  - 电气：额定 48 V，15~60 V 可用；额定 1.6 N·m，对应相电流峰值 2.4 A（约 0.67 N·m/A）；峰值 5.5 N·m，对应 11 A。
  - 机械：7.75:1 减速；编码器 14 bit 单圈绝对值，量的是转子（转子一圈 = 输出轴 46.45°）。
  - 接口：CAN 1 Mbps；插头 XT30(2+2)，电源和 CAN 在一个插头上；手册没提终端电阻。
  - 协议：电机只在收到指令时回反馈；canTimeout 出厂是 0（不启用）；运行中不能切模式。
  - 上电后角度在 0~2π，但不一定是真角度：输出角靠数转子圈数，断电后圈数还在不在手册没写，不在的话可能差整数个 46.45°，中值就不能像 GM6020 那样写死（2026-11-04 查手册的结论）。yaw 的相对角 `yaw_rel_deg` 底盘跟随、运动坐标变换都要用，所以这条跨模块。待上板验证（`C:\Clion\RS05\notes.md` 上板第 7 步），怎么处理见 Gimbal_test/notes.md「换 RS05」的「中值（零点）」。
  - 驱动和测试工程见 `C:\Clion\RS05\notes.md`。

## 6. 各模块进度

| 模块 | 工程 | 状态 |
|---|---|---|
| 云台 | Test_Module/Gimbal/Gimbal_test | App 精简版 v1（robot / gimbal / test_cmd，只有编码器定角度，含速度前馈，499 行）：2026-10-29 本机清理编译无警告，主机闭环测试通过；未上板，参数待标定。v2（ins + IMU 模式）接口已定：第 1 步 ins 已接入 robot.c，2026-10-30 首次上板（只接调试器 3.3 V）程序运行正常，ins 桌面验收通过（手转符号都对）；第 2 步云台 IMU 模式接口 2026-10-30 细化（限位算法已在主机上验证），用户写的第一版（gimbal / robot）2026-10-31 审查后由 Claude 改好：ARM 清理编译无警告，主机模型测试通过，还没上板，见其 notes.md。带 IMU 的旧完整版备份在 Test_Module/Gimbal/Backup。板间通信：2026-11-03 定了云台板发底盘指令的接口（`ChassisLink_SendCmd`，在 Shared/Link），函数体、指令源和 robot.c 用户同日写了第一版，审查发现 vx 高字节写错（1 m/s 收成 0.488）、联锁看的是云台指令模式而不是实际模式，用户让 Claude 改好：ARM 清理编译无警告，主机往返、联锁测试通过，还没上板，见其 notes.md「板间通信」的「第一版审查」。2026-11-04：云台电机可能换成 RS05。驱动已写好，放在 `Shared/Devices/RS05`（在 `C:\Clion\RS05` 里测，还没上板）；云台工程还是 GM6020，换的步骤见其 notes.md「换 RS05」；RS05 编码器量的是转子，中值能不能写死要先上板测，见其「中值（零点）」。v2 第 3 步 IMU 加热恒温：2026-11-04 定接口、.ioc 配 TIM10_CH1（PF6），11-05 用户第一版审查后 Claude 改好，ARM 编译无警告、主机模型里符合预期，还没上板（要接 24 V），见其「第 3 步进度」 |
| 底盘 | Test_Module/Chassis/Chassis_test | 2026-11-01 建好工程：.ioc 由 Gimbal_test.ioc 改来（只留 CAN1 / CAN2，打开自动离线恢复），CubeMX 生成，CMake 接 Shared，App 只有空的 robot.c；清理编译无警告，没上板。接口按步兵逻辑定了（四种模式：RELAX / 不跟随 / 跟随云台 / 小陀螺；指令是云台坐标系的平移速度 + 云台相对角；板间帧 0x110），分 3 步写，见其 notes.md；轮型未定，麦轮 / 全向轮公式都写了。2026-11-01 用户开始写（Infantry.h / Infantry_Chassis.c，速度环用电机层自带的），按用户要求 Claude 写好了初始化、板间指令接收（chassis_link）和 `Chassis_Update`（四种模式、掉线锁定、斜坡、坐标变换、逆运动学；轮型只体现在两个参数上），ARM 编译无警告、主机模型测试通过，没上板。用户还没有电机。参考用户的果园机器人底盘（全向轮，已上板调通，在 `C:\Project\Fruit-picking robot`）。2026-11-02 用户把 Infantry.h 改名为 Infantry_Chassis.h；2026-11-03 chassis_link 搬到 Shared/Link、两个类型移进 chassis_link.h（和云台板共用），代码没改 |

## 7. Shared 的用法与改动记录

新工程照 Gimbal_test 顶层 CMakeLists.txt 的写法引用（Chassis_test 已这样接，2026-11-01；只加了它用到的文件），`../` 的层数按工程所在位置改：

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
| 2026-11-03 | 新增 `Shared/Link/chassis_link.c/.h`：从 Chassis_test/App 原样搬来，`ChassisMode_e`、`ChassisCmd_t` 从 Infantry_Chassis.h 移进 chassis_link.h，加了 `ChassisLink_SendCmd` 的声明（函数体用户写）。两个工程的 CMake 都加了它和 include 目录 `Shared/Link` | 两个工程清理编译无警告，烧进 Flash 的内容和改之前逐字节相同（Gimbal_test FLASH 36568 B / RAM 3376 B，Chassis_test 20876 B / 2952 B）；云台工程还没调用，chassis_link 整个被链接器裁掉 |
| 2026-11-03 | `Shared/Link/chassis_link.c` 加云台板发送：`ChassisLink_SendCmd` + `ChassisLink_BuildFrame`（用户写第一版）、`ChassisLink_WriteS16`；按审查改了 vx 高字节、`(int8_t)` → `(uint8_t)`、限幅后 `roundf`、删两个 `sys/` 头文件。接收部分没动 | 两个工程清理编译无警告（Gimbal_test FLASH 37112 B / RAM 3384 B；Chassis_test 不调发送，固件和改之前逐字节相同）；主机测试：往返 9 组、饱和、NULL 都对；未上板 |
| 2026-11-04 | 新增 `Shared/Devices/RS05/rs05.c/.h`：灵足 RS05 私有协议，只用电流模式。由用户旧 RS05 工程按云台需要重写，和 `C:\Clion\RS05` 里的是同一份；那边的 `can_bus.*` 是本目录 BSP 的拷贝。同日用户想在 CLion 里看到它，Gimbal_test 的 CMake 加了 rs05.c 和 include 目录，还没有代码调用；Chassis_test 没编它 | 两个工程清理编译无警告，FLASH / RAM 和之前相同（Gimbal_test 37112 B / 3384 B，Chassis_test 20876 B / 2952 B）。Gimbal_test 加进 CMake 后，烧进 Flash 的内容和加之前逐字节相同，ELF 里没有 RS05 的符号（整个被链接器裁掉）。主机测试 98 项全过（见 `C:\Clion\RS05\notes.md`）；未上板 |

## 8. 还没做的（2026-11-04 盘点：机械没好，暂时上不了车）

按离车能不能验分组。离车能验的先做：现在除了 ins 在桌上验过，云台控制、底盘、板间通信、RS05 驱动都只在主机上测过，再多写只能主机测的代码，上车时问题会叠在一起。

1. 已经写好、离车就能验（不写新代码）：
   - RS05 单电机上板，重点是第 7 步上电绝对角（`C:\Clion\RS05\notes.md`）。要 RS05、24 V；第 0 步设 ID 要上位机 + USB-CAN。结论决定 yaw 能不能用 RS05（第 5 节）。
   - 两板通信验收（Gimbal_test/notes.md「板间通信」验收 1~3）。要两块 C 板，都要有 5 V（接 24 V 或插 USB）。
   - C 板轴向对应实物的哪条边（ins 第 1 步留下的，Gimbal_test/notes.md 标定记录）。只要 C 板。
   - 云台（或 yaw）留 GM6020、手上有电机的话：桌上先查固件版本、打开电流环（Gimbal_test 已知问题第 4 条）。
2. 要新写、离车能写完也能验完：
   - USB 虚拟串口 + VOFA+ 波形。只要 C 板。.ioc 配 USB_OTG_FS（Device_Only）+ USB_DEVICE 的 CDC，PLLQ 改 7（第 5 节）。上车调 PID 用，下面两项调试也能用。
   - 遥控器 DR16 + robot_cmd。要 DR16 接收机和遥控器。.ioc 配 USART3（RX 在 PC11），串口参数以 DR16 手册为准（硬件资料里还没有）。robot_cmd 取代 gimbal_test_cmd，遥控掉线全车无力；底盘落地跑之前要有它（调试器暂停后 C620 怎么处理手册没写，见 Chassis_test/notes.md 容易错的第 10 条）。
   - IMU 加热恒温（云台 v2 第 3 步，做法见 Gimbal_test/notes.md「第 3 步」）。只要 C 板，但要接 24 V（加热电源只来自 24 V，第 5 节）。2026-11-04 开始：.ioc 已配 TIM10_CH1（PF6）并生成，接口已定。用户第一版 11-05 审查出 12 处要改，用户让 Claude 改好：ARM 清理编译无警告，主机模型里符合预期，**还没上板**（Gimbal_test/notes.md「第 3 步进度」）。第一版在 `Shared/Algorithm/user_math` 加的 `Math_GetAbs` 已撤回（改用 fabsf），Shared 没有净改动。
   - 三项都要改 Gimbal_test 的 .ioc 和 robot.c，一次做一项。
3. 能写、离车只能主机测：发射机构（摩擦轮 + 拨弹盘）。要等机械定电机、拨盘格数；放哪路 CAN 见第 4 节最后一条。
4. 以后：裁判系统（底盘功率控制、枪口热量控制都要它的数据）、底盘 → 云台回传帧、超级电容、视觉通信、云台换 RS05（等第 1 组第一项的结论）、单板 / 双板定了以后的合并工程。

还没定、会卡住后面的：云台电机（GM6020 / RS05，或 pitch、yaw 各用一种）；单板 / 双板；底盘轮型；发射机构用什么电机。
