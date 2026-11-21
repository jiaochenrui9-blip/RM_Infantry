# RM_Infantry 项目笔记（总）

> 给各个对话看：总进度、跨模块约定、硬件事实。各工程自己的进度写在各自目录的 notes.md。
> 最后更新：2026-11-21 晚些板间 0x110 从 1 kHz 降到 100 Hz：C 板 CAN1 负载约 64%~78% → 54%~65%；`Shared/Link/chassis_link.h` 加 `CHASSIS_LINK_PERIOD_MS`（10 ms），两个云台工程的 robot.c 和三个主机仿真按 tick 差每 10 ms 发一帧；底盘板 50 ms 超时不变，F405_Project、Chassis_test 固件逐字节没变、不用重烧；4 个 ARM 工程 0 警告，HostTest、HostTest_RS05、CarSim 过；没上板。见第 4 节、第 7 节表最后一行、最后一节。2026-11-21 云台 RS05 第 1 步代码写好（陀螺仪 DRDY 当 Robot_Loop 的节拍，INS 的 dt 用样本时刻，1.5 ms 没样本按 tick 兜底）；Shared 改了 BMI088 读温度占住 SPI、`DJI_Motor_IsOnline` 改成有符号比；5 个 ARM 工程 0 警告，HostTest、CarSim 过；同日上板查了 ODR 和兜底，都对，USB、电机两条还没做；`cpu_profile.py` 改成以 DRDY 为零点，量出 DRDY 之后约 73~83 µs 云台用上（-O0），另外约 18 % 的 DRDY 固定晚到约 46 µs，原因待查。第 2 节第 4、5 条加了例外，第 7 节表最后一行，细节见 Gimbal_test_RS05/notes.md「第 1 步：DRDY 当节拍」。2026-11-12 晚上第一次在车上跑小陀螺（用户自己跑的，Claude 事后看记录，没改代码）：跟随、小陀螺的整体转向都对；底盘实际 25.9°/s、指令 36，R（0.20）或轮子半径（0.076）不准，待量；云台对地一直在 ±0.8° 左右来回晃；转到第 6 圈 yaw 被硬挡住（缠线或卡齿，还没看车），云台被底盘带着转，用户断电，下次转之前先查机械。见 Gimbal_test_RS05/notes.md「第一次小陀螺」、Chassis_test/notes.md「标定记录」，第 5 节「整车机械」的 yaw 和底盘两条。2026-11-12 白天车上调 yaw / pitch 的 PID、量 pitch 重力、电机定死，只记在 Gimbal_test_RS05/notes.md。2026-11-11 小陀螺准备（模式 2 用脚本调低增益后不抖了，但摩擦让 yaw 停在离目标几度的地方，低增益复位就没了；用户要试小陀螺，底盘 `CHASSIS_SPIN_DPS` 改成 36°/s（每个轮子约 300 转子 rpm），`gimbal_watch.py` 加了 `set chassis`，CarSim 的 `spin_fast` 改成按配置算速度；ARM 0 警告、CarSim 12 个全过、脚本测试 16 项全过；F405 还没烧，底盘还没转过。见 Chassis_test/notes.md、Gimbal_test_RS05/notes.md「当前状态」最后几条）。2026-11-11 云台上车（用户在车上测完方向、中值；Gimbal_test_RS05 的中值按读数改好，PID 和 pitch 重力前馈 Claude 给了初值，ARM 0 警告、HostTest_RS05 0 失败，还没烧。第 5 节「整车机械」滑环那条，细节见 Gimbal_test_RS05/notes.md「当前状态」第一条）。2026-11-11 换全向轮（用户说底盘是全向轮，在整车照片上标了电调 ID：1 前右、2 后右、3 后左、4 前左；底盘逆运动学按用户的全向轮写法改成驱动方向角表，F405_Project、Chassis_test 两份一样，CarSim 的底盘模型也换成全向轮；ARM 0 警告，CarSim 12 个场景全过，没上板。用户还说滑环装好了、yaw 中值还没标，C 板在炮头、F405 在底盘背面。第 3 节轮子正方向、第 4 节表、第 5 节「整车机械」的滑环和底盘两条、第 6 节底盘、第 8 节最后一段）。2026-11-11 最后（用户改了电源：一块电池在底盘，5 根细线并联走 XT 过滑环，滑环里是 XT + CANH、CANL，不再单独走 GND；链路是电池 → 3508 分电板 → F405 → CAN2 / RS05 总线分电板 → 滑环 → 云台。电源树重画、通讯图跟着改；第 4 节表下第一条「滑环、电源」、第 5 节）。2026-11-11 再后来（画了整车电源树和通讯图，放在 `docs/`，设备级；第 1 节、第 4 节表下第一条）。2026-11-11 后来（用户让把底盘逻辑移植到 F405_Project：App 换回 Chassis_test 的底盘代码，robot.c 按整车接线对调 CAN（电机 CAN1、板间 CAN2）；ARM 0 警告，robot.c 主机检查 94 项全过，没上板。第 1、4、6、8 节）。2026-11-11（用户改了接线：底盘加一块分电板，F405 CN8 和 yaw RS05 的 CAN 并在上面，引一路 CAN 过滑环到 C 板 **CAN1** 一号口，二号口接 pitch RS05。Gimbal_test_RS05 改回 CAN1：.ioc、CubeMX 生成的代码、robot.c、主机测试模型都和已提交的逐字一样，gimbal_watch.py 那四行改回。ARM 0 警告，主机测试 0 失败；`%TEMP%` 里两套脚本测试 01:57 被系统清理删了，没重跑。C 板 CAN1 是 2-pin 没有地脚，滑环的 GND 要另接到云台的地上。第 4 节表下第一条）。2026-11-10 晚上（用户让 Claude 把 Gimbal_test_RS05 改到 CAN2：.ioc 的 NART、ABOM 从 CAN1 挪到 CAN2，CubeMX 重新生成只变了 can.c 四行；robot.c 三处 hcan1 → hcan2；gimbal_watch.py 读 CAN2 的 ESR；主机测试模型同样改。ARM 0 警告，主机测试 0 失败，gimbal_watch 测试 182 项、yaw_friction 测试 103 项全过；没上板。第 4、5 节）。2026-11-10 最后（用户把过滑环的那一路从 C 板 CAN1 改到 CAN2：4-pin 有 GND，滑环线两头 3 根对 3 根；CAN1 留给发射机构。代码还是 CAN1，没改。第 4 节表下第一条）。2026-11-10 再后来（滑环很细、过不了电源，调试阶段上下各一块电池；滑环只走 CANH、CANL、GND：一对 CAN 就够，两块电池地不通所以必须走 GND；还没焊。第 4 节表下第一条）。2026-11-10 机械第一版出车（用户发了照片和手转看到的走向）：pitch 是四连杆，曲柄从最下往上炮管先抬头、过一个最高点后反而低头，电机角和 pitch 不是 1:1 也不单调，先量曲线再定先用单调的一段还是等第二版；yaw 是底盘上的 RS05 经打印齿轮带大齿轮（中心是滑环），传动比未知，不是 1:1 时上电中值写不死。用户的接线方案 Claude 核对过可行：F405 CAN1 接四个 C620、CAN2 接板间；C 板 CAN1 = 两个 RS05 + 板间（过滑环），CAN2 留给发射机构；底盘这边代码默认的 CAN1 / CAN2 要对调。没接线、没改代码。第 4、5、8 节。2026-11-10 上车准备（Gimbal_test_RS05：在线判断改回 yaw、pitch 都要在线，删了测试用的模式 3，gimbal_watch.py 加了 pitch；共用的主机测试场景 motor_offline 加了只掉 pitch。ARM 0 警告，两版主机测试都 0 失败；没上车。第 6、8 节，步骤见 Gimbal_test_RS05/notes.md「上车」）。2026-11-10（yaw 摩擦前馈：用 RS05 自己的相对转速补摩擦，不用底盘陀螺仪和回传帧。同日后来用户让 Claude 加上了：Gimbal_test_RS05 的 gimbal.c 一个式子 + gimbal_config.h 三个宏，系数先给 0（用户定第一版保持 0、先不测）；测摩擦脚本 `tools/yaw_friction.py`。主机上验过，没上板。第 6 节、第 8 节第 4 条，细节见 Gimbal_test_RS05/notes.md「yaw 摩擦前馈」）。2026-11-09（Gimbal_test_RS05 改成单电机 + 拿在手里的 C 板来测：yaw 方向改成 −1；在线判断暂时只看 yaw，pitch 没装，装上要改回；test_cmd 加了测试模式 3，转 C 板时 yaw 电机跟着转。主机上验过，没上板。同日后来又加了不停核读写脚本 `Gimbal_test_RS05/tools/gimbal_watch.py`。2026-11-10 上板：模式 3 跟得上；加热恒温第一次在板子上看到工作（稳在 40 ℃）。yaw 闭环原来稳在离目标约 1.4° 处，原因是只有 P、推不动静摩擦；用户让 Claude 在板子上调了 PID，yaw 改成速度环 Kp 0.016、Ki 0.2，角度环 Kp 12，空载电机上稳态误差 < 0.02°。调完以后 C 板的 24 V 断了，CAN1 进了 BUS-OFF；用户让在 .ioc 里打开了 CAN1 的自动离线恢复（ABOM），以后能自己恢复。脚本改成在一直读的窗口里直接输入命令（在 PowerShell 提示符下敲 `set mode 1` 是 PowerShell 自己的 Set-Variable，什么都不会发生）。第 5、6、8 节。同日早些时候：加了 RS05 单电机测试工程 `Test_Module/Gimbal/RS05_test`：C 板 CAN1 上只接一个 RS05，比 `C:\Clion\RS05` 多了类型 17 读参数（mechPos、编码器原始值、转子角、圈数等），用 OpenOCD 不停核读写。上板前两次电机都不回话，第三次用 scan 查出来这台 RS05 在 MIT 协议（ID 1），切回私有协议后正常了。第 1、5、6、8 节。同日早些时候：加了 F405 底盘板工程 `F405_Project`。这块板是队友打的，用作底盘主控；.ioc 按网表配置，App 先从 Chassis_test 复制。两个 CAN 收发器的 STB 要由 MCU 拉低，这一点和 C 板不同。同日后来 App 暂时换成 CAN1 上一个 M2006 的速度环测试（先写在 CAN2，用户让改到 CAN1）。第 1、5、6、7、8 节）。2026-11-08（加了并列工程 `Gimbal_test_RS05`：同一套云台代码，云台电机换成 RS05，Gimbal_test 不动；主机测试 `HostTest_RS05` 和 GM6020 版共用场景。更正：.ioc 里 `CAN1.NART=ENABLE` 才是打开自动重发。第 1、2、4、6、8 节）。2026-11-07（更正 RS05 零点：11-04 写的"上电后的角度可能差整数个 46.45°"是错的，用户的果园机器人工程 2026-10-22 已实测上电位置在输出轴一圈内是绝对的，云台中值能像 GM6020 那样写死；第 5、6、8 节）。2026-11-05（IMU 加热恒温代码改好、待上板，见第 6、8 节）。2026-11-04（IMU 加热恒温开工：查到加热电源只来自 24 V，记在第 5 节。同日早些时候：盘点了还没做的，见第 8 节；第 4 节加了发射机构要和云台电机分路。同日早些时候：云台电机可能从 GM6020 换成灵足 RS05：RS05 驱动在用户的 RS05 测试工程 `C:\Clion\RS05` 里重写成只用电流模式，放进 `Shared/Devices/RS05`；云台工程还是 GM6020，CMake 编了 rs05.c 但没有调用。第 2、4、5、6、7 节有改动）

## 2026-11-14 交接记录

- Codex 已查阅 Claude 的 RM_Infantry 会话和项目记忆，按这份总笔记与相关子工程笔记接续。核心控制先定坐标、单位、接口和验收，由用户按学习进度写第一版，再审查实现。调参、采波、日志和构建等辅助工具可直接实现。硬件结论优先对照实测、当前接线和手册。
- 本机 `C:/Project/RM_Infantry` 的 `main` 在 `56a7c2c`，工作区有原有未提交改动。RS05 云台的 `.ioc`、生成代码和入口已接入 USB CDC，当前工程未编入 `Shared/BSP/vofa.c`，也没有波形发送调用。
- 在临时目录重跑：F405_Project 和 Gimbal_test_RS05 的 ARM Debug 编译通过；HostTest_RS05 15 通过、1 跳过；CarSim 12 通过（云台模型为 GM6020）。本次仅做本机验证，未烧录。USB 枚举与收数待验。
- 实车接续见 Gimbal_test_RS05 笔记「第一次小陀螺」和 Chassis_test 笔记「标定记录」。第 6 圈 yaw 卡住的机械原因待查。底盘 25.9°/s 来自云台日志推算，36°/s 是配置值，轮速跟踪与几何尺寸待测。

- RS05 云台已接入 USB CDC 的 JustFloat Z 轴角速度输出。上板排查发现 USB 连接后曾出现 BMI088 SPI DMA 采样停更；在 .ioc 中将 USB OTG FS 中断优先级从 0 调到 6（SPI/DMA 为 5）并重新生成，用户反馈波形恢复。排查顺序和 OpenOCD 工作区的教训见 [RS05 云台笔记](Test_Module/Gimbal/Gimbal_test_RS05/notes.md#复盘接入-usb-后-bmi088-采样卡住)。

## 1. 项目组织

当前阶段：底盘、云台各写一个测试工程练手；整车拼好后合并联调。代码自己从 0 写，开源工程只借鉴思路。

整车按步兵的逻辑（2026-11-01 用户定）：云台板是主控，底盘接收云台主控发来的速度和模式（`ChassisCmd_t`，见 Chassis_test/notes.md「接口」）。双板时经板间 CAN 帧 0x110 传（`Shared/Link/chassis_link`，两块板编译同一份；云台板这边见 Gimbal_test/notes.md「板间通信」）；单板时 robot_cmd 直接填同一个结构体。

```
RM_Infantry/
├── notes.md                    ← 本文件
├── 硬件资料/                    ← C 板、C620、M3508、GM6020 手册（见其 README）
├── docs/                       ← 整车电源树、通讯图（SVG，设备级，2026-11-11 画的，见第 4 节表下第一条）
├── Shared/                     ← 各工程共用的 BSP / Devices / Algorithm / Link（用法见第 7 节）
├── F405_Project/               ← 底盘主控板（队友打的 F405 板）的工程，跑底盘程序（2026-11-11 从 Chassis_test 移植，CAN 按整车接线对调），见其 notes.md
├── Test_Module/
│   ├── Gimbal/Gimbal_test/     ← 云台测试工程，进度见其 notes.md
│   ├── Gimbal/HostTest/        ← 云台主机测试：电脑上编译真代码 + 电机模型，改了云台代码就跑一遍（见其 README）
│   ├── Gimbal/Gimbal_test_RS05/ ← 云台测试工程的 RS05 版（2026-11-08），和 Gimbal_test 并列，只差电机这一层
│   ├── Gimbal/HostTest_RS05/   ← RS05 版的主机测试：场景和 HostTest 共用，电机模型换成 RS05
│   ├── Gimbal/RS05_test/       ← RS05 单电机测试（2026-11-09）：C 板 CAN1 只接一个 RS05，OpenOCD 读写，见其 notes.md
│   ├── CarSim/                 ← 整车主机仿真：云台板 + 底盘板真代码 + 整车模型，测各模式的配合（见其 README）
│   └── Chassis/Chassis_test/   ← 底盘测试工程，进度见其 notes.md
└── （以后）Infantry/            ← 合并工程
```

## 2. 架构约定（底盘、云台都按这个写，合并时直接搬模块）

| 层 | 内容 | 位置 |
|---|---|---|
| BSP | 只封装 HAL 外设：can_bus；以后 USB 虚拟串口、SPI、UART、DWT | Shared/BSP |
| Devices | 器件驱动：DJI 电机、BMI088、IST8310、RS05（2026-11-04 加；Gimbal_test_RS05 用它，Gimbal_test 编了但没调用）；以后 DR16 | Shared/Devices |
| Algorithm | 与硬件无关：PID、user_math（限幅、角度回绕）、Attitude（Mahony 姿态解算）；以后斜坡等 | Shared/Algorithm |
| Link | 板间通信：帧格式、打包 / 解包。chassis_link（云台板发底盘指令、底盘板收），两块板编译同一份 | Shared/Link |
| App 模块 | gimbal（Gimbal_test 已有）、chassis；以后 shoot、ins、robot_cmd | 各工程 App/ |
| 装配层 | robot.c：持有 CAN manager、初始化各模块、1 ms 调度、CAN 接收分发 | 各工程 App/ |

规则：

1. 下层不 include 上层；App 模块之间也互不 include，只由 robot.c（合并后加 robot_cmd）连接。例外：ins 这种没有指令、只往外给数据的传感器模块算下层，别的模块可以 include 它的头文件、用它的 Feedback 类型，但不调它的函数，数据仍由 robot.c 传进来。
2. 每个 App 模块对外只有 `Xxx_Init()`、`Xxx_SetCmd(const XxxCmd_t *)`、`Xxx_Update(uint32_t now_tick)`、`Xxx_GetFeedback(XxxFeedback_t *)`，以及定义在 `xxx.h` 里的 Cmd / Feedback 结构体。命令从哪来（调试变量、遥控、视觉、另一块板经 CAN 转来）模块不关心，所以单板、双板方案都能接。要用传感器模块的数据时（如云台 IMU 模式要 ins 的姿态），再加一个 `Xxx_SetYyy()`，参数直接用那个模块的 Feedback 类型（如 `Gimbal_SetImu(const InsFeedback_t *)`），robot.c 每周期把 GetFeedback 的结果传进去。2026-10-31 改：原来要求本模块再定义一份字段相同的结构体、由 robot.c 逐字段拷，主循环里太乱。没有指令的模块（如 ins）不要 SetCmd。例外（2026-11-03 用户选的）：底盘的 `ChassisMode_e`、`ChassisCmd_t` 就是板间帧的内容，定义在 `Shared/Link/chassis_link.h`，两块板共用一份；底盘模块的头文件 include 它，只用类型。
3. 一路 CAN 只建一个 `DJI_MotorManager_t`，由 robot.c 持有，`Xxx_Init()` 时传给模块，模块把自己的电机注册进去；每个控制周期由 robot.c 对每路 CAN 调一次 `DJI_MotorManager_SendAll()`。原因：manager 负责检查同一路上的 ID / 槽位冲突；SendAll 会把同一控制帧里未注册的槽位发 0，两个 manager 发同一个帧 ID 会互相清零。
4. 模块与调度方式无关：`Xxx_Update()` 固定周期调用，dt 用宏写死。测试工程用裸机 1 kHz 循环；合并工程放进 FreeRTOS 任务，模块代码不改。例外（2026-11-21 起）：Gimbal_test_RS05 的 ins 用相邻两个陀螺仪样本的时刻差当 dt，因为节拍换成了陀螺仪自己的时钟（实测 999.68 Hz），写死 0.001 s 会把这个差积进角度；云台、发射的 PID 还是写死 0.001 s。
5. CubeMX 生成的 main.c 只加两处调用：外设初始化后 `Robot_Init()`，`while(1)` 里一直调 `Robot_Loop()`（内部按 HAL_GetTick 每 1 ms 跑一次）；HAL 回调（如 CAN 接收）放 robot.c。参考实现见 Gimbal_test/App/robot.c。例外（2026-11-21 起）：Gimbal_test_RS05 的 `Robot_Loop` 陀螺仪每来一个新样本跑一次，离上个样本超过 1.5 ms 才按 tick 每 1 ms 跑；传给模块的 now 还是 tick，所以模块里拿 ISR 打的时间戳和 now 比时要按有符号算（反馈可能比 now 新）。
6. 和具体车有关的参数放模块自己的 `xxx_config.h`（电调 ID、方向、编码器中值、限位、PID、限幅）；全车 CAN 分配登记在第 4 节。
7. 外设一律在 .ioc 里配置后生成代码，不手写初始化代码。CubeMX 也能命令行无界面生成，做法见 Chassis_test/notes.md「工程是怎么建的」。
8. 改 Shared 里的代码要在这里记一笔，两个测试工程都要重新编译。
9. 代码注释只写知识：原理、单位、方向、手册出处、实测结论和怎么测的。日期、谁说的 / 谁确认的、第几版怎么改过来的写进 notes.md，不写进代码。

## 3. 统一单位与正方向

- 角度用度（deg），角速度用度/秒（dps）。GM6020 转速反馈 rpm × 6 = dps。
- 云台：yaw 俯视逆时针为正，pitch 抬头为正。
- 底盘：x 向前、y 向左、wz 俯视逆时针为正（和云台 yaw 同向）；平移速度用 m/s，wz 用度/秒。每个轮子的正方向 = 推底盘俯视逆时针转的方向（和 wz 同向；2026-11-11 换全向轮时按用户的写法改的，原来是“让底盘前进的方向”）。各轮的驱动方向角从车头往左量，见 Chassis_test/notes.md「接口」。
- 底盘指令里的平移速度在云台朝向的坐标系里（只看 yaw：沿枪口为 x、枪口左边为 y），底盘用指令里的 `gimbal_yaw_rel_deg` 转到自己的坐标系；底盘自己的角速度由底盘按模式算，不在指令里。
- 电机装反时用 `motor_direction = -1` 修正，让逻辑正方向符合上面的约定。
- 云台 Feedback 提供 `yaw_rel_deg`（云台相对底盘正前方的角度），底盘跟随云台、运动坐标变换都用它（v1 代码里还叫 `yaw_deg`，v2 改名）。
- ins（惯导）输出也按这套：云台头坐标系 x 朝枪口、y 朝左、z 朝上；yaw 俯视逆时针为正、pitch 抬头为正。没有磁力计时 yaw 以上电朝向为 0，会漂。

## 4. CAN 分配（合并前整体检查）

| 总线 | 设备 | 电调 ID | 反馈 ID | 控制帧 / 槽位 | 状态 |
|---|---|---|---|---|---|
| CAN1（代码默认，待按接线确认） | GM6020 yaw | 1 | 0x205 | 0x1FE / 0 | 云台工程已用 |
| CAN1（代码默认，待按接线确认） | GM6020 pitch | 2 | 0x206 | 0x1FE / 1 | 云台工程已用 |
| CAN2（C 板版 Chassis_test 的默认）；F405_Project 是 CAN1（2026-11-11 按整车接线改好，见表下第一条） | M3508 ×4（底盘） | 1~4（1 前右、2 后右、3 后左、4 前左，2026-11-11 用户标的） | 0x201~0x204 | 0x200 / 0~3 | 底盘工程已用（2026-11-01） |
| 两块板之间那路（两块板都默认 CAN1；云台板上和云台电机同一路。整车接线是 C 板 CAN1 ↔ F405 CAN2（2026-11-11 定），云台板代码就是 CAN1，F405_Project 2026-11-11 已改成 CAN2（C 板版 Chassis_test 还是 CAN1），见表下第一条） | 板间指令：云台板 → 底盘板 | — | — | 0x110，DLC 8，100 Hz（2026-11-21 从 1 kHz 降下来，见最后一节） | 代码在 Shared/Link/chassis_link。底盘板接收已写（2026-11-01）；云台板发送 `ChassisLink_SendCmd` 已写（2026-11-03，用户写第一版、Claude 按审查改好），主机测试通过，两板联调待做 |

- **整车第一版接线（2026-11-11 用户定的；还没接）**：画成图了（设备级，只看链路）：[整车电源树](docs/整车电源树.svg)、[整车通讯图](docs/整车通讯图.svg)。接线、CAN 分配、电源方案改了，两张图要跟着改（SVG 是文本，坐标手写的）。
  - F405 底盘板：CAN1（CN7 / CN15，2-pin，R10 板载终端）接四个 C620，最远那个 C620 打开终端电阻；CAN2（CN8 / CN9，4-pin，R11 板载终端）接板间。和 C 板版 Chassis_test 的默认（底盘电机 CAN2、板间 CAN1）相反；2026-11-11 移植底盘程序时 F405_Project 的 robot.c 已经对调：底盘电机的 manager 用 CAN1（大疆过滤器组 0），0x110 的过滤器配到 CAN2（组 16）。CAN1 用 M2006 实际跑通过；CAN2 重焊后只验过能往总线上发（LEC 3），板间是它第一次真收（这块板在这一路上只收不发）。
  - C 板（云台主控，装在炮管上）：CAN1 = yaw RS05 + pitch RS05 + 0x110，就是下面 RS05 那条算的配置（0x110 100 Hz 时约 54%~65%，1 kHz 时是 64%~78%；发 0x110 的那个周期 3 个发送邮箱正好用满，C 板在这路上再加周期帧要错开那个周期）；CAN2 留给发射机构（在云台上，不过滑环；4-pin 口，大疆电调自带的 2-pin 线要转接）。Gimbal_test_RS05 就是这样写的（2026-11-10 晚上为了 CAN2 的地脚改到过 CAN2，11-11 用户接线换成 CAN1 又改回，见其 notes.md「CAN1 / CAN2 来回」）。
  - yaw RS05 装在底盘这头（第 5 节「整车机械」），pitch RS05、C 板在云台上，所以这路过滑环，一对线同时管 yaw 电机和板间。
  - **走线（2026-11-11 用户定的）**：F405 CN8 → 底盘新加的一块分电板（用户叫它 CAN2 / RS05 总线分电板，yaw RS05 的 CAN 也插在这块上）→ 引出一路 CAN（CANH、CANL；GND 后来改走电源负极，见下面「滑环、电源」）过滑环 → C 板 CAN1 一号口；C 板 CAN1 二号口 → pitch RS05。
    - 这块新分电板只给这一路用，别和 3508 那一路（F405 CAN1）的分电板、CAN 口混用，不然 yaw 和板间就跑到底盘的总线上，云台板够不着。电也走这块板（同日后来改的）：F405 的 XT30 → 这块板 → yaw RS05（XT30 2+2，电源和 CAN 一根线）和过滑环的 XT。
    - **F405 CN8 → 新分电板这根线 1 脚空着**，只接 2 GND、3 CANH、4 CANL：2026-11-09 就是 CN8 接分电板时 1 脚（板子的 5 V）把 F405 的 5 V 拉低、两路 CAN 一起罢工（第 5 节）。
    - GND：电源走滑环以后两边靠电源负极共地，不用再单独走 GND（同日后来改的）。之前上下两块电池时 GND 必须单独过滑环，C 板 CAN1 是 2-pin、没有地脚，要另接到云台的地上。
    - 终端：F405 的 R11（没开关）正好在底盘这头的端点；C 板（按原理图有板载 120 Ω）在中间，pitch RS05 在最末端，C 板 → pitch RS05 那段要短（几十厘米以内）。接好后断电量 CANH–CANL：约 60 Ω 对；约 40 Ω 说明新分电板或 RS05 里还有一个终端（RS05 有没有终端手册没写）。
    - 之前的方案：11-10 定的是 F405 CN9 接 yaw、滑环接 C 板 CAN2；Claude 提过底盘、云台各加一块分电板把两块板挪到最两头，用户嫌浪费没用。现在这样 yaw 是从分电板分出去的短分支，F405 在端点上。
  - 滑环这对线接触不好时 yaw 和板间一起断，现有保护兜得住：RS05 100 ms 收不到指令自己停（驱动写的 canTimeout = 2000，即 100 ms），云台板任一电机掉线整个云台无力并锁住，底盘收不到指令回 RELAX。滑环里的电源断了：C 板、pitch RS05 掉电；yaw RS05 在底盘这头还有电，100 ms 收不到指令自己停；底盘 50 ms 收不到 0x110 回 RELAX。
  - 以后加底盘 → 云台回传帧：走这路的话 1 kHz 约 65%~79%（0x110 已是 100 Hz；0x110 还是 1 kHz 时算的是 75%~92%），要么低频（几十 Hz），要么把板间挪到 C 板 CAN2（和发射机构一起约 56%~68%，要再过一对滑环线；滑环有富余的话现在就让机械留出来）。F405 在这路上发帧时，它的 CAN2 也要打开自动重发（`CAN2.NART=ENABLE`），原因同下面 RS05 那条：RS05 的应答比板间帧优先，不重发会被挤掉。F405_Project 的 .ioc 2026-11-11 已经两路都打开了。
  - **滑环、电源（2026-11-11 用户改了，还没接）：一块电池在底盘，电源也走滑环**。滑环里 5 根细线并联走 XT 电源（+、−），再加 CANH、CANL，没有单独的 GND 线。图：[整车电源树](docs/整车电源树.svg)。
    - 电池链路：电池 → 3508 分电板（四个 C620）→ F405 主控（XT30 一进一出）→ CAN2 / RS05 总线分电板（yaw RS05 的电和 CAN 都从这里出）→ 引一路 XT 过滑环 → 云台（C 板、pitch RS05）。
    - 还不知道的：5 根线几根接 +、几根接 −；云台这头怎么分到 C 板和 pitch RS05（C 板上有 3 个 XT30 输出口，接在 VCC_BAT 上，不经过它自己的防反接；也可以另接分线）。
    - 电流：负极的电流和正极一样大，能过多少按根数少的那一边算；每根的额定看滑环规格书（这种细线滑环常见 1~2 A 一路），并联的几根分流不平均，按总额定打七折左右。现在云台上只有 C 板和 pitch RS05（电流限幅 1 A，多是低速保持），电流不大；上车前可以用带电流显示的电源单独给云台供电，看动起来的最大电流。
    - 给云台和 yaw RS05 的电都穿过 F405 的 XT30（CN1、CN2、CN4 并联）和板上铜皮；F405 上没有保险丝、没有防反接（网表里 XT30 直接接 VCC_BAT / GND）。保险丝加在底盘这头、滑环之前（比如总线分电板引去滑环的那一路），按并联后的电流选：云台那边短路时先断它，不烧滑环和 F405 的铜皮。
    - 共地：两边靠电源负极共地。以前那根单独的 GND 线不要了，别再接到 C 板上（会分走一部分电机回流、走 C 板的地）。
    - 一对 CAN 就够：yaw RS05 和 F405 CAN2 本来就是 C 板 CAN1 这同一路，两个都在底盘这头，在总线分电板上并到一起再过滑环。滑环里 CAN 两路挨着，和电源之间隔一路空的（有富余的话）；出滑环以后 CAN 绞线。
    - 发射机构（两个摩擦轮起转、拨盘）电流大得多，这个细滑环多半带不动，到时候要换能过大电流的滑环。比赛时电源管理模块的云台口、发射机构口本来就要经滑环送上去。
    - DR16 接收机装在云台上、接 C 板 DBUS，不过滑环。
    - 焊好以后把每一路走什么记在这里（还没焊）。
    - 之前的方案（2026-11-10 ~ 11-11）：滑环过不了 XT 电源线，上下各一块电池（底盘电池管 F405、四个 C620、yaw RS05；云台电池管 C 板、pitch RS05），滑环只走 CANH、CANL、GND；GND 必须单独走（两块电池地不通），两边的 24 V 不能连。11-11 用户改成上面这样。
  - 以后接裁判系统时（比赛按规则电机的电要从电源管理模块出，具体以当年规范手册为准；那时要换能过电的滑环）：yaw RS05 虽然装在底盘这头，电要从电源管理模块的云台口取，底盘口的功率算在底盘功率限制里。过滑环的电源：云台、发射机构是电源管理模块的两路输出，分开走，每路按滑环单路的额定电流并几根。测速模块装在枪管上，装甲模块在底盘上，裁判系统模块之间那条总线跨了上下两边的话也要过滑环（线数到时候按规范手册算）。

- 同一路总线上：GM6020 ID 1~4 的反馈 0x205~0x208 和 M3508/M2006 ID 5~8 的反馈相同，不能同时用。
- GM6020 电压控制帧 0x1FF 与 M3508/M2006 ID 5~8 的控制帧相同；现有驱动用电流控制 0x1FE/0x2FE，不冲突。
- 1 Mbps 下一个 8 字节标准帧约 111~135 bit，每毫秒理论最多约 7~9 帧。每个 DJI 电机的反馈就占 1 帧/ms，再加控制帧。合并时按负载把电机分到两路 CAN。
- 底盘默认放 CAN2，和云台分在两路：4 帧反馈 + 1 帧控制 / ms，按上面的帧长算负载约 56%~68%。两个测试工程是分开烧的，现在不冲突；Chassis_test 里改总线只改 robot.c 传给 `Chassis_Init` 的 manager。
- 板间帧 0x110 不在大疆电机的过滤范围（0x200~0x20F）里，要单独配过滤器组：CAN1 用组 2、CAN2 用组 16（组 1 / 15 是 can_bus.h 里给扩展帧设备留的）。0x110 比大疆的控制帧（0x1FE~0x2FE）优先级高。
- 板间那路（双板默认）：两个 GM6020 的反馈 + 0x1FE = 3 帧/ms，加 100 Hz 的 0x110 按上面的帧长约 34%~42%（0x110 1 kHz 时 4 帧/ms，约 44%~54%）。底盘板的 CAN1 也挂在这路上，它在这路上什么也不发；它的大疆过滤器会收进云台电机的 0x205 / 0x206，没注册电机，直接忽略。（整车第一版是 F405 的 CAN2 挂在这路上，同样只收不发；云台是 RS05，扩展帧，F405 的两个过滤器都不收。）以后发射机构的电机别再往这路加。
- Gimbal_test、Chassis_test 的 CAN 都是 `AutoRetransmission = DISABLE`：每帧只发一次，出错或没有应答就丢掉，下一周期照常发。Gimbal_test_RS05 的 CAN1 开了自动重发（见下一条）。F405_Project 两路都开了（2026-11-11 和四个 M3508 测试一起改的，对底盘程序没坏处，见其 notes.md「现在跑的：底盘程序」）。
- RS05（2026-11-08 有了并列工程 Gimbal_test_RS05，Gimbal_test 还是 GM6020；细节见 Gimbal_test/notes.md「换 RS05」）：
  - 私有协议是扩展帧，每个电机每毫秒一帧指令 + 一帧应答，每帧 131~160 bit。两个 RS05 在一路上约 52%~64%，加 100 Hz 的 0x110 约 54%~65%（0x110 1 kHz 时 64%~78%）。
  - 应答（类型 2）的前 11 位是 0x080~0x0BF，比 0x110 和发给 RS05 的所有指令都优先。`AutoRetransmission = DISABLE` 时，C 板的帧和应答同时起步就会输掉仲裁、直接丢，所以换的那一路要在 .ioc 里打开自动重发：`CAN1.NART=ENABLE`（CubeMX 里 `NART` 这一项就叫 Automatic Retransmission，默认 `DISABLE`；2026-11-08 生成代码确认过）。
  - 过滤器用 CAN1 组 1、CAN2 组 15（can_bus.h 预留的），只收类型 2。
  - F4 每路只有 3 个发送邮箱，RS05 每个电机每周期占一个。两个 RS05 加 0x110 正好 3 个；0x110 降到 100 Hz 以后只有发它的那个周期用满，C 板在这一路上再加周期帧要错开那个周期。
- 发射机构（还没写；按两个 M3508 摩擦轮 + 一个 M2006 拨弹盘算：0x200 + 3 帧反馈 = 4 帧标准帧/ms，约 44%~54%）要和云台电机分在两路：和两个 GM6020 加 0x110 放同一路约 89%~108%，和两个 RS05 放同一路约 97%~118%。云台用 GM6020、或者 RS05 留在 CAN1 时，发射机构放云台板 CAN2；RS05 挪到 CAN2 时，发射机构放 CAN1（加 0x110 约 56%~68%）。2026-11-04 盘点时按上面的帧长算的，0x110 按 1 kHz；降到 100 Hz 以后，含 0x110 的两个数（89%~108%、56%~68%）各少约 10%~12%。

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
- CAN 收发器 TJA1044GTK/3：VCC 接 VCC_5V（24 V 经 SY8501 降压，或 USB 的 VBUS 经 DSS34 二极管），VIO 接 3.3 V，STB 接地。只靠调试器 3.3 V 供电时 CAN 发不出去，会进 bus-off（2026-10-30 实测）。Gimbal_test 两路 CAN 的 Automatic Bus-Off Management 都没开（can.c 里 `AutoBusOff = DISABLE`），bus-off 后要复位才能恢复；Chassis_test 两路都开了（2026-11-01，还没上板验证）。Gimbal_test_RS05 的 CAN1 2026-11-10 也打开了：当天 C 板的 24 V 断过，CAN1 进了 bus-off，没开时得复位 C 板。
- GM6020：拨码 1~3 位设 ID（1~7），第 4 位是 CAN 终端电阻。反馈 1 kHz，编码器 0~8191，转速单位 rpm。电流控制 ±16384 对应 ±3 A，需要固件 ≥1.0.11.2，并在 RoboMaster Assistant（≥2.7）里打开电流环。
- C620 + M3508：电调 ID 1~8 用 SET 键设（步骤见 Chassis_test/notes.md）；电流 ±16384 对应 ±20 A；反馈 0x200 + ID、默认 1 kHz，转速是转子 rpm（M3508 减速比 3591/187 ≈ 19.2）；电调上有 120 Ω 终端电阻开关；不能同时插 CAN 线和 PWM 线。
- 灵足 RS05（手册在 `C:\Users\Lenovo\Desktop\Data\01_硬件资料\手册\RS05\RS05使用说明书_260713.pdf`，没放进 硬件资料/）：
  - 电气：额定 48 V，15~60 V 可用；额定 1.6 N·m，对应相电流峰值 2.4 A（约 0.67 N·m/A）；峰值 5.5 N·m，对应 11 A。
  - 机械：7.75:1 减速；编码器 14 bit 单圈绝对值。
  - 接口：CAN 1 Mbps；插头 XT30(2+2)，电源和 CAN 在一个插头上；手册没提终端电阻。
  - 协议：电机只在收到指令时回反馈；canTimeout 出厂是 0（不启用）；运行中不能切模式。
  - 上电位置在输出轴一圈以内是绝对的，圈数不保存（用户的果园机器人工程 2026-10-22 两次断电实测，读的是 mechPos 0x7019，见 `C:\Project\Fruit-picking robot\notes.md` 第 2 节「RS05」）。上电读数 zero_sta = 0 时在 0~2π，= 1 时在 −π~π。所以云台中值能像 GM6020 那样写死，(position − zero) 折到 ±180°，前提是 RS05 直接带轴（整车第一版的 yaw 是齿轮带的、pitch 是连杆带的，见本节最后「整车机械」）。按手册参数表的公式会推断成"上电只能确定到 46.45°"，实测不是。类型 2 的角度和 mechPos：2026-11-09 静止时读到 +6.3252 / +6.3255 rad，差在类型 2 的分辨率（约 0.0004 rad）以内，是同一个角度。转过 ±4π 以后会不会分开（类型 2 在 ±4π 循环，mechPos 是多圈的），还要手转确认（`Test_Module/Gimbal/RS05_test` 上板第 3 步）。
  - 和 C 板一起上电时，约 1.4 s 内不回话（果园工程实测），要等它回过话再使能。
  - **先确认电机在私有协议**（2026-11-09 实测）：用户手上这台原来在 MIT 协议，ID 1，主机 ID 0xFD。在 MIT 协议时，它照样给私有协议的扩展帧应答（应答是 CAN 控制器硬件做的，所以 TEC、LEC 都是 0），但一帧都不回，看起来像 ID 不对。查法：`Test_Module/Gimbal/RS05_test` 的 `scan`。切回私有协议：MIT 下发标准帧指令 8（ID = 电机 ID，`FF FF FF FF FF FF 00 FD`），再给电机断电重启（手册第 6 章）。同日已经切回来了，现在是私有协议、ID 1。
  - 电机自己的正方向（2026-11-09 用户实测）：从输出轴那一面（转盘那面）看，顺时针为正（用户确认是看的这一面）。正电流、正速度、类型 2 角度变大三者同号，所以云台里一个方向系数就够了。推出来装到云台上：yaw −1（Gimbal_test_RS05 2026-11-09 已改）；pitch 电机在左侧 −1、右侧 +1。装好以后以 RELAX 下手转为准，推法见 `Test_Module/Gimbal/RS05_test/notes.md`「第 3、4 步：方向」。类型 2 的角度在 ±12.56 之间循环，云台代码折到 ±180°，不受影响。
  - 反馈（类型 2）是输出端、减速后的：角速度范围 ±50 rad/s 对应空载 480 rpm，力矩 ±5.5 N·m 是峰值输出力矩（M3508 报的是转子）。编码器装在哪手册没写清：只列了一个 14 bit 编码器、叫它"转子（编码器）位置"，但上电时输出端一圈内是绝对的，光靠转子编码器做不到，输出端应该另有位置检测。手册 3.3.4 节上位机参数表里有 encoderRaw（0x3004）、modPos（0x3015，电机未计圈机械角度）、rotation（0x3014，圈数）。2026-11-09 实测：类型 17 读这些 0x30xx（还有 0x2005 MechOffset）都回"读取失败"，只能读 4.1 节列的 0x70xx，所以这几个量只能在上位机里看。
  - 驱动的设计和原来的测试工程见 `C:\Clion\RS05\notes.md`。单电机上板测试 2026-11-09 起用 `Test_Module/Gimbal/RS05_test`（见其 notes.md）。
- 队友打的 F405 底盘主控板（2026-11-09 按网表整理，还没对照实物；完整引脚表在 `F405_Project/notes.md`）：
  - STM32F405RGT6（LQFP64），HSE 8 MHz。
  - CAN1 在 PB8/PB9，插座 2-pin：1 CANL、2 CANH。CAN2 在 PB12/PB13，插座 4-pin：1 5V、2 GND、3 CANH、4 CANL。插座线序和 C 板一样，两路都有板载 120 Ω，没有开关。
  - **和 C 板不一样：** 两个 TJA1044 的 STB 接到 MCU（CAN1 接 PC1、CAN2 接 PC2），不是接地。TJA1044 手册（NXP Rev. 8.0，7.1、7.3.2 节）：STB 高电平是待机，待机时不能发送；STB 内部上拉，悬空也是待机。所以 .ioc 里这两个引脚必须配成输出、初始低电平。
  - **实测（2026-11-09）**：第一块板上两片 TJA1044（3×3 mm 的 HVSON8 封装）都有引脚没焊上。U3 是 8 脚（STB），所以收发器一直在待机，表笔压一下就能通；U4 是 1、4 脚（TXD、RXD）和地。重焊后两路都正常了。查法：用 MCU 测芯片内部的上拉和 RXD 输出，再在空总线上发一帧，报"没有应答"（LEC 3）就说明收发器正常，报位错误（LEC 5）就是发不出去。见 `F405_Project/notes.md`「上板记录」。以后拿到新板，先用这个办法查一遍收发器。
  - **CAN2 口（CN8/CN9）的 1 脚是这块板的 5 V**。2026-11-09 用一根线把 CN8 接到分电板，结果这块板的 5 V 被拉低，两路 CAN 一起罢工，拔掉线就好了。跟别的板子连 CAN 之前先对线序，最好只接 CANH、CANL、GND，不接 5 V。C 板的 CAN2 口也是这个线序，同样要注意。
  - 收发器 VCC 的 5 V 只能由 24 V 降压得到；USB 的 VBUS 没接，只插 USB 板子不上电。只接调试器时 CAN 用不了，这一点和 C 板一样。
  - 板上还有：BMI088（SPI1，引脚和 C 板不同）、DBUS（USART3_RX PB11，硬件反相）、两路 UART、两路 RS485（UART4、USART6）、USB-C。
  - 没有 LED、蜂鸣器、用户按键、IMU 加热。
- **整车机械第一版**（2026-11-10 机械组出车；用户发了照片，pitch 的走向是用户手转看的）：
  - pitch：RS05 装在炮管后面，转盘上的销（曲柄）经一根白色连杆带炮管下面的销，是四连杆，不是直接带轴。用户看到的走向（同日更正后）：曲柄从最下往上，炮管先连续抬头，过一个最高点以后再往上反而低头，最上、最下都是低头。
    - 最高点是曲柄和连杆转到一条直线上的位置（机构学叫极位）：传动比为 0，两边方向相反。所以电机角和 pitch 不是 1:1，整段也不单调。
    - 编码器模式电机照样能停住（环只管电机角），只是炮管不在给的角度；IMU 模式反馈是炮管的真实角度（C 板在炮管上），过了最高点环就是反的，会一直推到机械极限（被电流限幅顶住）。最高点附近电机转很多、炮管只动一点，等效增益接近 0。
    - 能不能先用单调的一段（最下 → 最高点，或最高点 → 最上）要先量“炮管角–电机角”曲线：机械在 CAD 里扫最快；车上量就模式 0 从最下慢慢推到最上，`gimbal_watch.py` 一直读，记 pitch 编码器角和 `imu_p`（C 板在炮管上，`imu_p` 就是炮管角）。要的 pitch 范围放得进一段、用到的部分斜率不低于这段最大斜率的一半左右，就能先用。
    - 先用一段时代码不用改：中值标在炮管水平处，方向按这一段的走向定，`GIMBAL_PITCH_MIN/MAX_DEG`（电机角）限在这一段里、离最高点留余量。代价：编码器模式的 pitch 读数是电机角不是炮管角；增益随位置变，按最陡的那头调。IMU 模式的限位按“IMU 角 − 编码器角”的差换算（gimbal.c 的 `Gimbal_AxisControl`），只有 1:1 时这个差不随位置变；单调时停到限位上还是对的，过程中会差一点，所以余量多留。
    - 给第二版的建议（用户转给机械）：平行四边形，曲柄长 = 炮管那头的摇杆长、连杆长 = 两转轴距离，中位时两臂都和连杆垂直，整个范围内夹角不小于 40° 左右（机械原理的最小传动角）。这样 pitch = 电机角，现在的中值、方向、限位直接用，IMU 模式的限位换算也是准的。铰点上轴承或衬套、连杆够刚：间隙和弹性都在环里。
  - yaw：RS05 装在底盘底面，轴上一个打印的小齿轮带大齿轮，大齿轮中心是滑环，带整个云台转。用户说转着不舒服。
    - **传动比还不知道（要问机械要齿数）。不是 1:1 时上面 RS05 那条“中值写死”不成立**：RS05 上电只在自己输出轴一圈内是绝对的，传动比 N = p/q（最简分数）时上电有 p 个可能的 yaw、相差 360°/p（N = 2 就是差 180°）。yaw_rel_deg 错了，底盘跟随、小陀螺的坐标变换跟着错；RS05 中途断电重上也一样。非 1:1 只能在 yaw 轴上另加绝对位置检测（或规定上电时云台大致朝前），代码要在 `Gimbal_AxisRead` 加传动比和多圈展开。建议机械做成 1:1（齿轮只为了把电机挪开、中心留给滑环）。1:1 时力矩就是 RS05 本身的（额定 1.6、峰值 5.5 N·m），够不够按 CAD 里 yaw 的转动惯量估。
    - 打印齿轮有齿隙，在 RS05 编码器和云台之间：编码器模式看不出来（电机停住，云台在间隙里晃）；IMU 模式在环里，推断会在目标附近来回找。桌上调的 PID、测的摩擦都不含它。建议机加工齿轮 + 电机座可调中心距，或 1:1 同步带加张紧（没有齿隙，有一点弹性）。
    - 转着不舒服可能的原因（没查）：照片上小齿轮齿面毛糙；中心距太紧或偏心（每圈同一位置发紧）；RS05 自己反拖就不顺（桌上静摩擦 0.03~0.15 A，随位置、方向变）。查法：松开电机座让齿轮脱开，单转云台、单转 RS05，再合上慢转一整圈。
    - 2026-11-12 晚上第一次小陀螺：云台相对底盘往一个方向连续转到第 6 圈（5.97 圈）时被硬挡住，1 A（约 0.67 N·m）推不动，反方向一推就走；之前每圈经过同一位置都正常。缠线还是卡齿还没查，见 Gimbal_test_RS05/notes.md「第一次小陀螺」。
  - C 板装在炮管上（跟着 pitch 转）。用户说 X 朝前、Y 朝左、Z 朝右；Y 朝左时按右手系 Z 只能朝上（Y、Z 不可能一左一右），“朝右”应是笔误，待用户确认。是 Z 朝上的话安装矩阵就是现在的单位矩阵，不用改；上车第 2 步用数据确认：炮管水平静止时 `BMI088_DATA.acc_g` ≈ (0, 0, +1)，抬炮口时 `acc_g[0]` 变正。
  - 滑环：很细。2026-11-10 先定的是上下各一块电池、滑环只走 CAN + GND；11-11 用户改成一块电池在底盘、5 根细线并联走 XT 电源，见第 4 节表下第一条「滑环、电源」。
    - 2026-11-11 用户说滑环装好了，yaw 中值还没标。标之前云台发给底盘的 `gimbal_yaw_rel_deg` 不是相对车头的角，底盘的坐标变换、跟随都跟着偏；底盘单独调的时候这个角填 0。
    - 同日晚上上车标了方向、中值（Gimbal_test_RS05/notes.md「当前状态」第一条）：方向参数是对的；中值改好了（yaw 是读数“−91 左右”换算的）。读数里编码器走 30°、`imu_y` 走 30.3°，yaw 传动比约 1:1，`gimbal_yaw_rel_deg` 就是云台相对车头的角。模式 2 抖得厉害、电流打到 1 A，抖了约 9 s 后 C 板和两个 RS05 一起掉电，分析见 Gimbal_test_RS05/notes.md「上板记录」最后一条。
  - yaw RS05 中间穿不了滑环，所以机械把它挪到底盘底面用齿轮带；用户 2026-11-10 走线走了一晚上，觉得这版结构不好接线。
  - **第二版要跟机械提的（电控这边，2026-11-11 汇总）**：
    1. 滑环要能过电：比赛时电源管理模块的云台口、发射机构口都要经滑环送上去，按两个摩擦轮起转时的最大电流选型；路数够放电源若干路 + CAN 一对 + 备用一对，以后还有裁判系统的线。
    2. yaw 1:1（上电中值才能写死），机加工齿轮或同步带，电机座能调中心距。滑环必须在 yaw 轴上（放旁边的话云台一转，滑环到云台那段线照样会缠）；想把电机放中间，就得换中空轴的电机，让滑环或线从中间穿过去（用户 2026-11-11 问过）。或者用大孔径的过孔滑环套在电机外面（和 yaw 同轴、电机在它的孔里）：RS05 居中直接带云台，1:1、没有齿轮齿隙，上电中值能写死；孔径要大于 RS05 外径 Ø46 mm（STEP 模型里最大的圆柱面 R23，单位 mm）再留余量。这种滑环比现在的大、贵；云台的重量还是靠单独的大轴承扛。
    3. pitch 做成平行四边形连杆，铰点上轴承或衬套。
    4. 走线在 CAD 里就定：线从哪走、分电板和接插件放哪；电机装好以后插头还能插拔（第一版 yaw RS05 在底面不好接）；跨 pitch 轴的线留够余量。
    5. 电机到炮头的连接要硬，给一个能量的指标（2026-11-12 晚上加）：第一版小陀螺时量到炮头在软连接上相对电机摆动的频率约 5.6 Hz（阻尼比约 0.15），而 yaw 角速度环的穿越频率约 4 Hz，两者挨着，所以一转就晃、增益加不上去。第二版把这个频率做到 30 Hz 以上、最好 50 Hz 以上（经验上要比角速度环带宽高 3 ~ 5 倍），yaw、pitch 都按这个验。验收办法：电机定住，敲一下炮头，看 IMU 角速度上振荡的频率。做到了，现在的“IMU 角度环 + 角速度环”就是最终的结构，不用为软连接另加东西。见 Gimbal_test_RS05/notes.md「第一次小陀螺」。
  - 底盘：四个全向轮装在四角、轮轴指向车中心，带悬挂。2026-11-11 用户确认是全向轮，在整车照片上标了：车头是云台那头，车尾是电池（电池位置不能动）；电调 ID 1 前右、2 后右、3 后左、4 前左（俯视顺时针，和果园机器人的排法一样）。轮子半径、轮心到车中心的距离还没量（代码里先用 0.076、0.20）。2026-11-12 落地小陀螺实际 25.9°/s、指令 36（0.72 倍）：轮子都跟上、不打滑的话，轮子半径 0.076 不变时 R ≈ 0.278 m，见 Chassis_test/notes.md「标定记录」。
  - 板子位置（2026-11-11 用户说的）：C 板在炮头，F405 底盘主控在底盘背面。底盘现在不用 F405 上的 BMI088，以后要用时按实际朝向定安装矩阵。

## 6. 各模块进度

| 模块 | 工程 | 状态 |
|---|---|---|
| 云台 | Test_Module/Gimbal/Gimbal_test | App 精简版 v1（robot / gimbal / test_cmd，只有编码器定角度，含速度前馈，499 行）：2026-10-29 本机清理编译无警告，主机闭环测试通过；未上板，参数待标定。v2（ins + IMU 模式）接口已定：第 1 步 ins 已接入 robot.c，2026-10-30 首次上板（只接调试器 3.3 V）程序运行正常，ins 桌面验收通过（手转符号都对）；第 2 步云台 IMU 模式接口 2026-10-30 细化（限位算法已在主机上验证），用户写的第一版（gimbal / robot）2026-10-31 审查后由 Claude 改好：ARM 清理编译无警告，主机模型测试通过，还没上板，见其 notes.md。带 IMU 的旧完整版备份在 Test_Module/Gimbal/Backup。板间通信：2026-11-03 定了云台板发底盘指令的接口（`ChassisLink_SendCmd`，在 Shared/Link），函数体、指令源和 robot.c 用户同日写了第一版，审查发现 vx 高字节写错（1 m/s 收成 0.488）、联锁看的是云台指令模式而不是实际模式，用户让 Claude 改好：ARM 清理编译无警告，主机往返、联锁测试通过，还没上板，见其 notes.md「板间通信」的「第一版审查」。2026-11-04：云台电机可能换成 RS05。驱动已写好，放在 `Shared/Devices/RS05`（在 `C:\Clion\RS05` 里测，还没上板）；云台工程还是 GM6020，换的步骤见其 notes.md「换 RS05」；RS05 上电位置在一圈内是绝对的（果园工程实测），中值照 GM6020 写死，见其「中值（零点）」。v2 第 3 步 IMU 加热恒温：2026-11-04 定接口、.ioc 配 TIM10_CH1（PF6），11-05 用户第一版审查后 Claude 改好，ARM 编译无警告、主机模型里符合预期，还没上板（要接 24 V），见其「第 3 步进度」。2026-11-08：按「换 RS05」做了并列工程 `Test_Module/Gimbal/Gimbal_test_RS05`（从已提交的 Gimbal_test 复制，只差电机这一层；ARM 0 警告，FLASH 38584 B / RAM 3144 B；`HostTest_RS05` 同一套 16 个场景 0 失败；未上板），Gimbal_test 没动。以后 Gimbal_test 的 ins、test_cmd、robot.c、.ioc 改了要手动同步过去，对比方法见其 notes.md。2026-11-09：建了 RS05 单电机测试工程 `Test_Module/Gimbal/RS05_test`（C 板，只开 CAN1；OpenOCD 读写脚本 `tools/rs05_watch.py`）。同日上板两次，电机都不回话。总线是好的，C 板的帧有人应答（TEC、LEC 都是 0），软件配置用 OpenOCD 查过都对；私有协议给 ID 0~127 都发过，没有一个回话。固件改成什么帧都收、能发任意一帧，脚本的 scan 三种协议都试（ARM 0 警告，FLASH 12896 B / RAM 2072 B；主机检查 486 项全过）。第三次上板 scan 查出来：这台 RS05 在 MIT 协议、ID 1，没有故障。用户切回私有协议后正常了：类型 2 1 kHz，mechPos 和类型 2 的角度一致；类型 17 读不了 0x20xx / 0x30xx。下一步手转、看电流方向，见其 notes.md「上板记录」。同日后来手转、电流方向测完（从输出轴那面看顺时针为正），用户手上只有一个 RS05、没有 pitch 电机，要“C 板标定好以后，转 C 板，yaw 电机跟着转”，在 Gimbal_test_RS05 里改了三处：`GIMBAL_YAW_DIRECTION` 改成 −1；`Gimbal_Update` 的在线判断暂时只看 yaw（装上 pitch 要改回）；test_cmd 加了测试模式 3（发给云台的是编码器模式，yaw 目标 = C 板 yaw + 进模式时算好的差，所以进模式时电机不动；C 板装到云台以后别用，会一直转）。ARM 0 警告，FLASH 38616 B / RAM 3144 B；HostTest_RS05 0 失败，模式 3 另在临时目录检查 19 项全过；没上板。用法、注意事项见其 notes.md「单电机：yaw 跟着 C 板转」。2026-11-10：Gimbal_test_RS05 的 yaw 加了摩擦前馈（系数是 0 = 关，用户定第一版保持 0、先不测；Gimbal_test 没有）和测摩擦脚本 `tools/yaw_friction.py`；ARM 0 警告，HostTest_RS05 0 失败，脚本在假板子上、前馈在加了库仑摩擦的主机模型里验过，没上板，见其 notes.md「yaw 摩擦前馈」。同日后来改成上车的配置：在线判断改回 yaw、pitch 都要在线，删了测试用的模式 3，gimbal_watch.py 加了 pitch；没上车，步骤见其 notes.md「上车」 |
| 底盘 | Test_Module/Chassis/Chassis_test（C 板）、F405_Project（F405 板） | 2026-11-01 建好工程：.ioc 由 Gimbal_test.ioc 改来（只留 CAN1 / CAN2，打开自动离线恢复），CubeMX 生成，CMake 接 Shared，App 只有空的 robot.c；清理编译无警告，没上板。接口按步兵逻辑定了（四种模式：RELAX / 不跟随 / 跟随云台 / 小陀螺；指令是云台坐标系的平移速度 + 云台相对角；板间帧 0x110），分 3 步写，见其 notes.md；轮型未定，麦轮 / 全向轮公式都写了。2026-11-01 用户开始写（Infantry.h / Infantry_Chassis.c，速度环用电机层自带的），按用户要求 Claude 写好了初始化、板间指令接收（chassis_link）和 `Chassis_Update`（四种模式、掉线锁定、斜坡、坐标变换、逆运动学；轮型只体现在两个参数上），ARM 编译无警告、主机模型测试通过，没上板。用户还没有电机。参考用户的果园机器人底盘（全向轮，已上板调通，在 `C:\Project\Fruit-picking robot`）。2026-11-02 用户把 Infantry.h 改名为 Infantry_Chassis.h；2026-11-03 chassis_link 搬到 Shared/Link、两个类型移进 chassis_link.h（和云台板共用），代码没改。2026-11-09：底盘主控用队友打的 F405 板，建了并列工程 `F405_Project`（在仓库根目录下）。App 从 Chassis_test 原样复制；.ioc 按网表配置（CAN1 PB8/PB9、CAN2 PB12/PB13、两个收发器的 STB 拉低），CubeMX 命令行生成。ARM 0 警告，FLASH 21108 B / RAM 2952 B，没上板。同日后来用户让先去掉底盘逻辑：App 换成单个 M2006（C610 ID 1）、速度环转到 500 转子 rpm 的上板测试（PID 3 / 20 / 0 没调过，限幅 2 A）。总线先是 CAN2，后改到 CAN1，最后电机经分电板接到 CAN2，又改回 CAN2。ARM 0 警告，FLASH 14772 B / RAM 1904 B，主机检查通过。同日第一次上板 M2006 不转：用 OpenOCD 读状态，软件和配置都对；两路 CAN 收发器都没往总线上驱动（CAN1 一直 bus-off，CAN2 测试帧读不回显性），静默模式下也解不出帧。用户确认 24 V 和 TJA1044 的 5 V 都正常。后来用户拿电压表量 U3 的 STB 时电机转了（收到 6071 帧 0x201），拿开就停。再用 MCU 逐个测芯片引脚（借芯片内部的上拉和 RXD 输出），查出 U3 的 8 脚（STB）没焊上，U4 的 1 脚（TXD）、4 脚（RXD）没焊上。两片 TJA1044 都要重焊，软件不用改。用户重焊后 CAN1 正常了：1 kHz 反馈，TEC/REC 都是 0，M2006 平均稳在 500 转子 rpm（标准差约 50，PID 3/20/0 还没调）。U4 再焊以后，CAN2 测试帧报"没有应答"（LEC 3），说明 U4 也正常了，两路 CAN 都好了。CAN2 还没接电调实际收发过。见其 notes.md「上板记录」。2026-11-11：工作区里有一版没提交的四个 M3508 测试（CAN1、ID 1~4、500 转子 rpm，.ioc 两路打开自动重发；notes 里之前没记，有没有上板不知道）。同日用户让把底盘逻辑移植过来：App 换回 Chassis_test 的底盘代码（模块三个文件一样），robot.c 按整车接线对调 CAN（电机 CAN1、板间 CAN2），.ioc 没动。ARM 0 警告，FLASH 21108 B / RAM 2952 B；robot.c 主机检查 94 项全过（换成没对调的 C 板版 robot.c 有 35 项失败）；没上板。四个 M3508 测试存在 git stash 里（commit `1dc61b8`）。以后底盘模块两边一起改（CarSim 编的是 Chassis_test 那份），robot.c 各按各的板子分 CAN，见 F405_Project/notes.md。2026-11-11 换全向轮：用户说车是全向轮、标了电调 ID（1 前右、2 后右、3 后左、4 前左），逆运动学按用户 ELRS 工程 `Omni_Remote.c` 的写法改成驱动方向角表（坐标换成这边的 x 前、y 左、逆时针），斜坡和坐标变换没动；`CHASSIS_XY_FACTOR` 删了，方向先填推出来的 4 个 −1（待核对），R 先填 0.20（待量）。两份底盘模块一样；CarSim 的底盘模型换成全向轮，spin_fast 改成 4 m/s、link_loss 看 1.5 s 后，12 个场景全过，故意改错 4 处都能抓到。ARM 0 警告，F405 FLASH 21064 B / RAM 2952 B；没上板，见 Chassis_test/notes.md「当前状态」最后一条 |

## 7. Shared 的用法与改动记录

新工程照 Gimbal_test 顶层 CMakeLists.txt 的写法引用（Chassis_test 已这样接，2026-11-01；只加了它用到的文件。F405_Project 在仓库根目录下一层，写 `../Shared`，2026-11-09），`../` 的层数按工程所在位置改：

```cmake
get_filename_component(SHARED_DIR "${CMAKE_CURRENT_SOURCE_DIR}/../../../Shared" ABSOLUTE)
target_sources(${CMAKE_PROJECT_NAME} PRIVATE ${SHARED_DIR}/BSP/can_bus.c ...)
target_include_directories(${CMAKE_PROJECT_NAME} PRIVATE ${SHARED_DIR}/BSP ...)
```

- Shared 不在工程目录内，CLion 项目树默认看不到；需要的话用 Tools → CMake → Change Project Root 把根目录设为 RM_Infantry。
- Shared 里的代码 include 各工程自己的 `main.h`（STM32F4 HAL），只适用于 F4 + HAL 的工程（C 板、F405 板都满足；F405 和 F407 的 bxCAN 一样，也是 28 个过滤器组，can_bus.h 的分组照用）。

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
| 2026-11-16 | `Shared/Devices/DJI_Driver/motor_control.c/.h`（发射机构卡弹反转用位置模式，用户让 Claude 直接加）：`MotorControl_SetMode` 切到 `MOTOR_MODE_POSITION` 时把当前 `total_encoder` 记成零点（原来只在第一次进位置环时记一次，之后再也不变）；新增 `MotorControl_GetDegree`（当前输出轴角度，相对这个零点）、`MotorControl_SetPositionSpeedLimit`（位置环输出限幅，原来写死 300 转子 rpm，默认值没变）。改之前全仓库没人用位置模式 | Gimbal_test_RS05、F405_Project、Gimbal_test、Chassis_test 清理编译 0 警告；CarSim、HostTest 通过；发射主机检查 22 个场景（模型补了编码器）按预期；未上板 |
| 2026-11-16 | `motor_control.c`：位置环周期 5 ms → 1 ms（`MOTOR_CONTROL_POSITION_PERIOD_MS` 1、`_PERIOD_S` 0.001），和速度环一样；用户的理由是大疆反馈 1 kHz，不用就浪费。PID 用传入的 dt，KP 不用改。只有发射机构拨弹轮用位置模式 | 4 个 ARM 工程 0 警告；CarSim、HostTest 通过；发射检查 22 个场景按预期（反转 160 ms、到 −26.9°，和 5 ms 时差不多，限速 2000 才是瓶颈）；未上板 |
| 2026-11-19 | 只改注释：`can_bus.h` 的过滤器分组补上 RS05（CAN1 组 1、CAN2 组 15）和板间 0x110（组 2、16），删了“本工程尚未配置”；`motor_control.c` 删日期；`Attitude.c/.h`、`dji_gm6020.c/.h` 删 CLion 生成的 `Created by` 文件头 | 6 个 ARM 工程清理编译 0 警告，`.bin` 和改前逐字节相同 |
| 2026-11-21 | `BMI088.c/.h`：`BMI088_ReadTemperature` 阻塞读的时候把状态置成新加的 `BMI088_STATE_READING_TEMP`（关中断看到 IDLE 才置，读完放回 IDLE），这期间 `BMI088_StartSample` 返回忙、不碰片选。Gimbal_test_RS05 在 EXTI 里启动采样，要靠它才不会把温度读截断；按 tick 调的工程（Gimbal_test、F405_Project）行为不变。`dji_motor.c`：`DJI_Motor_IsOnline` 改成按有符号比，反馈时间戳比 now 新（调用者取 tick 之后才收到）时算刚收到，和 `RS05_IsOnline` 一样；Gimbal_test_RS05 跟 DRDY 跑以后才会碰到 | Gimbal_test_RS05、Gimbal_test、F405_Project、Chassis_test、Power 清理编译 0 警告；HostTest 15 过 1 跳过、CarSim 14 过；Gimbal_test_RS05 的主机时序模拟通过（读温度时进 DRDY 中断不截断）；未上板 |
| 2026-11-21 | `Link/chassis_link.h`：加 `CHASSIS_LINK_PERIOD_MS`（10 ms，0x110 从 1 kHz 改成 100 Hz），帧格式说明和 `ChassisLink_SendCmd` 的注释跟着改；`CHASSIS_LINK_TIMEOUT_MS` 还是 50，chassis_link.c 没改。什么时候发由各云台工程的 robot.c 按 tick 差判 | F405_Project、Chassis_test 清理编译 0 警告，`.bin` 和改前逐字节相同；Gimbal_test、Gimbal_test_RS05 0 警告；HostTest、HostTest_RS05 15 过 1 跳过，CarSim 14 过；未上板 |

## 8. 还没做的（2026-11-04 盘点：机械没好，暂时上不了车）

按离车能不能验分组。离车能验的先做：现在除了 ins 在桌上验过，云台控制、底盘、板间通信、RS05 驱动都只在主机上测过，再多写只能主机测的代码，上车时问题会叠在一起。

1. 已经写好、离车就能验（不写新代码）：
   - RS05 单电机上板：2026-11-09 起用 `Test_Module/Gimbal/RS05_test`，步骤见其 notes.md（原来的 `C:\Clion\RS05` 也还能用，但它不读类型 17 参数）。要 RS05、24 V；第 0 步设 ID 要上位机 + USB-CAN（2026-11-11：也可以用 RS05_test 的 `send` 发类型 7 改，见其 notes「改电机的 CAN ID」，还没试）。上电绝对角果园工程已经测过（第 5 节），不再卡 yaw 能不能用 RS05；第 7 步只确认类型 2 的角度也一样（2026-11-07 改）。
   - 单电机 + 拿在手里的 C 板：转 C 板，yaw 电机跟着转（Gimbal_test_RS05 模式 3，2026-11-09）。要 RS05、C 板、24 V。这也是第一次在真电机上闭环，C 板接 24 V 的话还是加热恒温第一次上板。步骤见 Gimbal_test_RS05/notes.md「单电机：yaw 跟着 C 板转」。2026-11-10 做完了（用户说差不多了）；之后模式 3 删了，工程改成上车的配置，上车步骤见 Gimbal_test_RS05/notes.md「上车」。
   - 两板通信验收（Gimbal_test/notes.md「板间通信」验收 1~3）。要两块 C 板，都要有 5 V（接 24 V 或插 USB）。底盘板也可以用 F405 板，但它只能接 24 V。
   - F405 底盘板上板验收（`F405_Project/notes.md`「上板验收」）：第 1、2 步时钟、STB 电平 11-09 用 OpenOCD 查过。现在是底盘程序（2026-11-11）：第 3 步接四个 C620（ID 1~4）看反馈，第 4 步和云台板通 0x110（F405 这头是 CAN2），之后按 Chassis_test 的上板步骤调底盘。要这块板、DAPLink、24 V、四个 M3508 + C620。
   - C 板轴向对应实物的哪条边（ins 第 1 步留下的，Gimbal_test/notes.md 标定记录）。只要 C 板。
   - 云台（或 yaw）留 GM6020、手上有电机的话：桌上先查固件版本、打开电流环（Gimbal_test 已知问题第 4 条）。
2. 要新写、离车能写完也能验完：
   - USB 虚拟串口 + VOFA+ 波形。只要 C 板。.ioc 配 USB_OTG_FS（Device_Only）+ USB_DEVICE 的 CDC，PLLQ 改 7（第 5 节）。上车调 PID 用，下面两项调试也能用。
   - 遥控器 DR16 + robot_cmd。要 DR16 接收机和遥控器。.ioc 配 USART3（RX 在 PC11），串口参数以 DR16 手册为准（硬件资料里还没有）。robot_cmd 取代 gimbal_test_cmd，遥控掉线全车无力；底盘落地跑之前要有它（调试器暂停后 C620 怎么处理手册没写，见 Chassis_test/notes.md 容易错的第 10 条）。
   - IMU 加热恒温（云台 v2 第 3 步，做法见 Gimbal_test/notes.md「第 3 步」）。只要 C 板，但要接 24 V（加热电源只来自 24 V，第 5 节）。2026-11-04 开始：.ioc 已配 TIM10_CH1（PF6）并生成，接口已定。用户第一版 11-05 审查出 12 处要改，用户让 Claude 改好：ARM 清理编译无警告，主机模型里符合预期，**还没上板**（Gimbal_test/notes.md「第 3 步进度」）。第一版在 `Shared/Algorithm/user_math` 加的 `Math_GetAbs` 已撤回（改用 fabsf），Shared 没有净改动。
   - 三项都要改 Gimbal_test 的 .ioc 和 robot.c，一次做一项。
3. 能写、离车只能主机测：发射机构（摩擦轮 + 拨弹盘）。要等机械定电机、拨盘格数；放哪路 CAN 见第 4 节最后一条。
4. 以后：裁判系统（底盘功率控制、枪口热量控制都要它的数据）、底盘 → 云台回传帧（2026-11-10：yaw 的底盘角速度前馈不用它，云台板用 RS05 自己的相对转速就行，见 Gimbal_test_RS05/notes.md「yaw 摩擦前馈」）、超级电容、视觉通信、单板 / 双板定了以后的合并工程。云台换 RS05：并列工程 Gimbal_test_RS05 已写好（2026-11-08，ARM 0 警告、主机测试全过），上板前先做第 1 组第一项。

还没定、会卡住后面的：yaw 齿轮传动比（要 1:1，第 5 节「整车机械」）；pitch 连杆（第一版有极位：先量曲线，定先用单调的一段还是等第二版）；发射机构用什么电机。2026-11-10 定了的：整车第一版云台两个都是 RS05；双板，F405 底盘 + C 板云台（接线见第 4 节表下第一条）。2026-11-11 定了的：底盘是四角装的全向轮，电调 ID 排法见第 5 节「整车机械」。

## 2026-11-15 C 板 CAN2 发射机构规划

- 当前进度：用户确定 CAN2 为 M3508 摩擦轮 ID 1、2，M2006 拨弹轮 ID 3；已基于现有 RS05 云台工程做架构规划，尚未实现。
- 已验证：静态检查 CAN1 为 RS05 与板间链路；CAN2 应用未启动。现有 DJI manager/速度环可作为接入基础。
- 未验证：本轮未编译、烧录、上板；实际电调 ID、方向、传动结构、控制参数待确认。
- 对外接口：拟增 App/shoot 的 Init / SetCmd / Update / GetFeedback，由 robot.c 持有唯一 CAN2 DJI manager，统一收发 0x200 与 0x201~0x203。
- 注意事项和踩过的坑：速度使用转子 rpm，电流限幅为协议原始值；CAN2 自动重发/离线恢复当前关闭；第一阶段建议速度控制和就绪联锁，单发等尚未确定。
- 下一步：细化接口与机械参数后逐步实现，详见 Test_Module/Gimbal/Gimbal_test_RS05/notes.md「2026-11-15 CAN2 发射机构架构规划」。

## 2026-11-15 发射模块骨架交接

- 当前进度：Gimbal_test_RS05 新增 shoot 和 shoot_test_cmd；C 板 CAN2 的两个 M3508（ID 1、2）及一个 M2006（ID 3）注册/初始化、收发、1 ms 调度已接入。控制逻辑由用户写第一版。
- 已验证：ARM Debug 全量编译零警告；主机模拟检查注册、反馈分发及 FEED 指令下仍为零输出通过。
- 未验证：未烧录、未上板；方向、PID、传动和电流限幅未实测。此次未改 .ioc 和 Shared；CAN2 自动重发、离线恢复仍关闭。
- 对外接口：Shoot_Init / Shoot_SetCmd / Shoot_Update / Shoot_GetFeedback；ShootCmd_t 包含模式、摩擦轮和拨弹轮转子 rpm。
- 注意事项和踩过的坑：初始化后均失能，PID 为零；修改调试变量不会转动。就绪联锁和故障锁定未实现；MotorControl_SetMode 不能每周期调用。
- 下一步：用户从 Shoot_Update 注释处写第一版，细节见子工程 notes.md「发射机构准备工作完成」。

## 2026-11-15 下午～傍晚 发射机构接口重定、用户第一版

- 当前进度：下午按用户的半个 `Shoot_UpdateMode` 定了接口（反馈 mode 是实际模式；SPINUP 时拨弹轮速度环目标 0；电机模式只在进出 RELAX 时切），config 加了掉线超时、就绪门槛、限幅和 PID 初值，`.ioc` 打开了 CAN2 的 ABOM、NART。傍晚用户写完第一版。
- 踩过的坑：17:11 发射模块 4 个文件和两份 notes 被 CLion 以外的程序写回旧版（原因没查出来），从 CLion 内容缓存 `caches\content.dat` 找回了代码，notes 照原文补回。改完及时提交。
- 已验证：ARM 0 警告；主机检查 check2 11 个场景，掉线锁对。
- 问题：模式切换里先 return 再判就绪，指令 FEED 时每周期清 PID、发不了弹；进了 FEED 掉速也不退。用户改。
- 未验证：没烧、没上板；`.ioc` 改了但 can.c 没生成（ssh 里 CubeMX 命令行卡住，要用户在 GUI 点 GENERATE CODE）。
- 细节见 Gimbal_test_RS05/notes.md 最后两节。
- 晚上（补）：用户让 Claude 把第一版的问题改了（先算完整模式再比、就绪每周期算、只在进出 RELAX 时切电机模式），ARM 0 警告，check2 里 RELAX 直接 FEED 约 1.1 s 进 FEED、拨弹中掉速退回 SPINUP。没上板、没提交。

## 2026-11-15 RS05 云台版本提交

- 当前进度：USB CDC 陀螺仪波形、发射模块及晚上的控制修正随本版提交。根目录 README 已补充两块板的接线和编译烧录步骤。
- 已验证：本机重跑 RS05 云台 ARM Debug 全量编译，零警告。复跑 check2 共 11 个主机模型场景，复核了就绪后进入 FEED、掉速退回 SPINUP 和掉线锁定。
- 未验证：本版未烧录，发射机构未上板验证。
- 对外接口：`Shoot_Init / Shoot_SetCmd / Shoot_Update / Shoot_GetFeedback`，板间 CAN 仍使用 `0x110`。
- 注意事项和踩过的坑：CAN2 自动重发和离线恢复已写入 `.ioc`，当前生成的 `can.c` 两项仍关闭。
- 下一步：在 CubeMX 重新生成代码，确认 CAN2 两项配置更新后再编译并上板。
- 晚上（补）：发射机构第二版接口定了：只加卡弹检测 + 反转退弹（单发等拨盘格数，没做）。对外接口不变，卡弹在 FEED 内部处理，连续卡满次数走 `shoot_fault` 锁。config 加了 7 个 `SHOOT_JAM_*` 宏，shoot.c 加了桩 `Shoot_FeederTarget`（用户写），check2 加了卡弹场景 12~19。ARM 0 警告，行为和第一版一样。没提交。
- 夜里（补）：卡弹第一版完成（用户写状态机，Claude 补了 DETECT 返回值、RELAX 清卡弹次数以便解锁、CLEAR_MS 清零、离开 FEED 复位）。ARM 0 警告，check2 19 个场景按预期。上板要看起转会不会被误判成卡弹。没提交。
- 深夜（补）：发射机构下一版只做“卡弹反转改成位置模式、转固定角度”，单发 / 连发不做。还没开始；Claude 先改了代码被叫停，已全部撤回（Shared 无改动）。开工前要定 Shared 位置环的零点和限速怎么改，见 Gimbal_test_RS05/notes.md 最后一节。
- 本次提交收录卡弹检测、定时反转和连续卡弹锁定。重新全量编译 RS05 云台 ARM Debug，零警告。复跑 check2 的 19 个模型场景，复核了反转、持续卡住锁定和解锁恢复。未烧录、未上板。
- 2026-11-16：发射机构第三版完成（用户让 Claude 直接加）：卡弹反转改成位置模式往回转 30°（M2006 输出轴），转到或 300 ms 超时回正转。Shared 电机层加了位置模式零点、读角度、设限速（见第 7 节）。所有用这个驱动的工程编译 0 警告，CarSim、HostTest 通过，发射检查 22 个场景按预期。没上板、没提交。
- 2026-11-16：Gimbal_test_RS05 加了发射机构读写脚本 `tools/shoot_watch.py`（stop / spin / fire / set / 调参 / once），只接发射机构就能用；假 OpenOCD 测 27 项全过，没上真板。见 Gimbal_test_RS05/notes.md 最后一节。
- 2026-11-16 提交前复核：四个受共用电机层改动影响的 ARM 工程全量编译通过，CarSim、HostTest、HostTest_RS05 通过。复跑了 22 个发射机构模型场景，并用新编译的 RS05 ELF 完成 `shoot_watch.py` 的 27 项模拟检查。未烧录、未上板。
- 2026-11-17：F405 底盘板第二版网表（`F405_Project/405板子的网表_v2.enet`）检查完，**CAN1 2-pin 插座线序和第一版、C 板反了**（第二版 1 CANH、2 CANL），C620 原装线会 H、L 反接；另有加热电阻功率偏紧。照 C 板移植 BMI088 / ins、加热（TIM3_CH3）、USB CDC 波形：.ioc 和 App 改完，用户用 CubeMX GUI 生成，ARM 清理编译 0 警告（FLASH 63768 B / RAM 11128 B），没上板、没提交。见 F405_Project/notes.md 最后一节。

## 2026-11-17 Power 功率计与 Shared I2C BSP

- 当前进度：新增 Shared/BSP/i2c_bus.c/.h，提供七位设备地址、八位寄存器指针的阻塞读写和高字节在前的十六位访问；Power/App 新增 INA228 初始化、测量反馈与 10 ms 调度。新 BSP 通过工程 main.h 使用 HAL，Power 已验证为 F1；原有其他 Shared 模块的 F4 限制不因此改变。
- 已验证：用户确认采样电阻为 1 mΩ；CubeMX 6.15.0 已从 .ioc 生成 HSE 8 MHz→PLL 72 MHz、I2C1 400 kHz；Power ARM Debug 全量编译无警告（FLASH 12076 B / RAM 1744 B），七组主机模拟测试通过。原有 Gimbal_test 和 Chassis_test ARM 全量构建通过。
- 未验证：后续已烧录并通过 OpenOCD 完成无主电源/负载条件下的运行检查，详见下方记录。I2C 实际波形、带电源/负载测量精度及额定载流仍未验证。CAN 已配置为 PB8/PB9、1 Mbps，但尚未启动或验证通信。
- 对外接口：Power_Init / Power_Update / Power_GetFeedback；反馈包含输入电压、压差、电流、功率、两项 ID、最近成功时间和状态。详见 [Power 笔记](Power/notes.md) 与 [使用说明](Power/README.md)。
- 注意事项和踩过的坑：R5 网表 Value 与型号冲突，按用户确认的 1 mΩ；采样范围不是整板载流额定值。寄存器逐个读取，负电流须符号扩展，读取失败不覆盖上次样本；初始化失败需重新初始化或复位。CubeMX 生成期间原 .ioc 曾回退，已恢复生成后的配置并核对。
- 下一步：限流低压上电检查 ID 与 VIN，再接已知负载核对电流、功率；CAN 报文协议及收发后续接入。现有 F405 工作区改动未由本任务修改。
- 2026-11-17 后续：用户要求 CAN 1 Mbps，已将 Power 的 .ioc 设置为 PB8/PB9 重映射、PCLK1 36 MHz / 分频 3 / BS1 9 / BS2 2 / SJW 1，通过 CubeMX 生成 can.c 并完成 ARM Debug 编译。未烧录、未上板通信。
- 2026-11-17 上板跟进：用户反馈 Power 已烧录。OpenOCD 只读检查在探针阶段失败：CMSIS-DAPv2 返回 CMD_INFO 响应不匹配，HID 后备接口未找到设备；两次探针连接后停止。未取得目标数据、未复位或重烧；不能据此判定 Power 代码是否正常。待结束其他调试会话、重新插拔 DAPLink 后再读，细节见 Power/notes.md。
- 2026-11-17 随后用户要求重试：CMSIS-DAPv2 连接成功，Flash 与 Power/build/Debug/Power.elf 全部加载段匹配；INA228 ID=0x5449/0x2281，约 10.4 秒内 282 组稳定反馈均 HAL_OK，观察到 10 ms 更新。用户确认只接调试器、无主电源/负载，VIN 约 2.793～2.801 V、电流 −4.375～+5.625 mA，不据此确认精度。时钟、I2C 400 kHz、CAN 1 Mbps/PB8-PB9 参数已在寄存器核对；CAN 未启动。全程不停核、不复位、不烧录，OpenOCD 已关闭，CPU 保持运行。

## 2026-11-17 Power 单 M3508 速度测试

- 当前进度：Power 引用 Shared/BSP/can_bus、Devices/DJI_Driver 和 Algorithm/PID，不复制驱动；共享 manager 的 CAN2 分支增加条件编译兼容 F103。ID1、CAN1 PB8/PB9 1 Mbps、目标转子 1000 RPM（输出轴约 52.1 RPM），PID 3/10/0，指令电流限幅 ±3000。
- 已验证：CubeMX 已从 .ioc 生成 CAN RX0 中断优先级 5 与 ABOM；Power 的 Debug/codex-debug ARM 构建通过，FLASH 20316 B / RAM 1976 B；九组主机测试通过；Gimbal_test、Chassis_test 共享驱动增量构建通过。
- 未验证：最新电机版本未烧录，实际 CAN 通信、速度跟踪、PID 和 SysTick 耗时未验证。之前 Power 实板功率计结论不覆盖本版本。
- 对外接口：Robot_ControlTick 在 SysTick USER CODE 内每 1 ms 执行控制；Robot_Loop 主循环每 10 ms 读取功率计。参数在 Power/App/motor_config.h；电机只保存 s_motor 一份状态。
- 注意事项和踩过的坑：无反馈/超时 100 ms 清零，反馈恢复后自动继续 1000 RPM；上电有效反馈后会转。I2C 阻塞读取不进入控制中断；CAN 邮箱忙下一周期发新值，自动重发关闭。重新构建的 Power/build/Debug/Power.elf 不再匹配板上旧固件，VOFA 地址须重新取。未改 F405 的其他工作区改动。
- 下一步：用户烧录新版后查看 feedback_count 与 speed_rpm，验证 1000 RPM 跟踪和断线恢复；测控制耗时。详见 Power/notes.md 与 README.md。

## 2026-11-19 精简注释（全仓库手写代码）

- 范围：Shared、各工程 App、主机测试（HostTest、HostTest_RS05、CarSim）、Power、tools/*.py、各工程 CMakeLists.txt 里手写的注释。CubeMX 生成的文件、HAL / Middlewares、README 和各 notes.md 没动。
- 做了什么：只改注释，代码没动。删了注释里的日期、“用户说 / 用户确认 / 用户让 Claude 加”“你选”、第几版怎么改过来的；CLion 生成的 `// Created by game on ...` 文件头全删（17 个文件）。知识点（原理、单位、方向、手册出处、实测结论和测法）都留着。约定写进第 2 节第 9 条。
- 顺手改的过时说法：`can_bus.h` 的过滤器分组（见第 7 节）；Gimbal_test_RS05 的 `shoot.c` 开头约定按现在的实现合成「控制逻辑」1~4 条，`robot.c`、`shoot_test_cmd.h` 删了“控制骨架零输出”“控制逻辑写完之前不会转”；Gimbal_test 的 CMake 里 RS05 那行改成“本工程不调用”；F405 的 `ins_config.h` 里用户写的 “11.17 实测 0.2f, 0.02f 比较稳定” 并进加热 PI 的注释，删了原来的“占位值”说明。细节见 Gimbal_test_RS05/notes.md 最后一节。
- 验证：F405_Project、Power、Chassis_test、Gimbal_test、Gimbal_test_RS05、RS05_test 在 `%TEMP%` 清理编译，0 警告，导出的 `.bin` 和改前逐字节相同；HostTest、HostTest_RS05 各 15 过 1 跳过，CarSim 12 过；改过的 Python 脚本语法检查通过。没烧录。
- 注意：这次是在原有未提交改动（F405 的 USB / ins / I2C / TIM、Power 工程、Shared/BSP/i2c_bus）上直接改的，没有提交；F405_Project/CMakeLists.txt、F405 的 ins 文件里两种改动混在一起。

## 2026-11-19 版本提交前复核

- 当前进度：本版一并收录 F405 的 BMI088 / ins、加热、USB CDC、第二版网表，Power 功率计及单 M3508 测试工程，共用 I2C BSP 与注释整理。README 已补充目录、接线差异及编译烧录说明。
- 已验证：本轮在临时目录全量编译 F405_Project、Power、Chassis_test、Gimbal_test、Gimbal_test_RS05、RS05_test，均通过且无编译警告；CarSim、HostTest、HostTest_RS05 及 Power 九组主机测试通过，九个 Python 脚本语法检查通过。
- 未验证：本轮未烧录、未上板；此前 Power 功率计实板结果仅适用于对应旧固件，最新电机控制版本仍待实测。
- 对外接口：Power_Init / Power_Update / Power_GetFeedback、Robot_ControlTick，以及 Shared/BSP/i2c_bus.h 中的寄存器读写接口，详见 Power/notes.md。
- 注意事项和踩过的坑：F405 第二版网表 CAN1 插座线序有变化；RS05 云台的 CAN2 自动重发与离线恢复仍需用 CubeMX 生成。AGENTS.txt 和 tools.zip 为本地附件。
- 下一步：按各工程笔记完成硬件核对、烧录及对应上板验收。

## 2026-11-19 板间相对角跨 ±180° 的检查（只查了，代码没改）

- 问题：0x110 里的 `gimbal_yaw_rel` 在 -180~180 内，云台相对底盘转过 ±180° 时数值跳 360°，底盘拿它做坐标变换、跟随还对不对。
- 结论：对，底盘不用改。底盘只在两处用它（`Infantry_Chassis.c` 的第 1、3 步）：坐标变换只取 cos / sin，差 360° 结果一样；跟随是纯 P，折到 ±180° 的值就是最短路径的误差，跨过去只是“往左转 180°”换成“往右转 180°”，wz 目标换号由 720 °/s² 的斜坡接住。底盘没有对这个角求差、滤波、插值。int16 × 0.01° 装得下 ±327.67°，±180° 不溢出。
- 怎么验的：`%TEMP%\wrap_check` 里另编一份 CarSim（真代码，只把 test_car.c 换成临时场景，没放进仓库；云台是 GM6020 版，底盘代码和 F405_Project 逐字相同）。① 小陀螺 36 °/s + 按 W 1.0 m/s 跑 25 s，跨 ±180° 3 次（底盘收到 -180.00 → 179.96）；跨的前后 50 ms 内对地速度方向最大偏 0.22°、轮电流每 ms 最大变化 10（原始值），和全程（0.23°、11）一样。② 小陀螺中相对角走到 -170° 时切跟随：底盘反过来顺时针转 170°。走到 -179.0 / -179.5 / -179.9° 时切：底盘只减到 29.7 / 34.5 / 36.0 °/s，29 / 14 / 3 ms 后被惯性带过 -180°，目标换回正，接着逆时针转约 180°；只跨 1 次、不来回抖，最后都对齐（差 0.1° 内）。③ 不跟随时云台转到 ±179.5° 再切跟随：各往近的一边转。
- 原来 CarSim 的 12 个场景都没跨过 ±180°（相对角最多到 -142.75°，小陀螺场景只转 4~5 s）。同日按用户的意思加了两个场景（只改 `Test_Module/CarSim/test_car.c` 和它的 README，底盘、云台代码没动）：`spin_wrap`（小陀螺 + 平移转 1.5 圈，比较 |相对角| > 170° 时和别处的对地方向、轮电流每 ms 变化）、`follow_wrap`（静止时 ±179.5° 各往近的一边转；小陀螺中离 ±180° 差半个刹车距离时切跟随，被惯性带过去、角速度不跳、最后对齐）。圈数、切换点都按 `chassis_config.h` 算，改小陀螺转速不用改场景。CMake 编到 `%TEMP%`：0 警告，14 个全过，全套约 0.9 s；yaw 改成 ±90° 有限位时 8 过 6 跳过 0 失败。往底盘代码副本里注入两个 bug，都只有新场景失败：对相对角做低通 → `spin_wrap`（轮电流每 ms 跳 3762，正常 10；对地方向只多偏 0.7°，惯性抹平了）；跟随用 0~360° 的角 → `follow_wrap` 的 ①（-179.5° 逆时针绕了 180.5°）。没提交。
- 以后要注意：会被跳变坑的是把几个时刻的角放在一起算——求微分（PID.c 的 D 直接对反馈求差，跨的那一拍是 360° / 1 ms，所以跟随只用 P）、低通 / 求平均（179.9 和 -179.9 平均成 0，错 180°）、两帧之间插值、对角度本身做斜坡。要这么做就先 `Math_WrapDeg180(新 − 旧)` 再算，或对 cos、sin 分别滤波。θ + wz × 延时这种外推后再取 cos / sin 不用折。小陀螺转得快时切跟随要减速一段（v² / 2a，360 °/s、720 °/s² 约 90°），中途过了 ±180° 底盘会顺着原方向转过去，最后照样对齐。

## 2026-11-20 云台板 INS 和云台的时序（只分析、定方案，代码没改）

- 用户想用 BMI088 的 DRDY 让每个 IMU 样本对应一次云台计算。细节见 [Gimbal_test_RS05 笔记](Test_Module/Gimbal/Gimbal_test_RS05/notes.md) 最后一节。
- 现在 IMU 样本到云台用上 1~2 ms，而且随相位漂（陀螺仪 ODR 手册 ±0.3 %，和 SysTick 不锁相）；RS05 是问答式，已经跟着控制周期走。现在的车上看不出影响（机械是瓶颈），第二版机械以后有用。
- 推荐方案（用户没定）：陀螺仪 DRDY（BMI088 INT3 → PC5，已对原理图）当整个控制的节拍，2 ms 没样本退回按 tick 跑；INS 的 dt 改用时间戳。要动的约定：第 2 节第 4 条（dt 用宏写死）对 INS 不再成立，第 5 条（Robot_Loop 按 HAL_GetTick 每 1 ms 跑）改成按新样本跑。Shared 的 BMI088 驱动要加 DRDY 寄存器配置和温度读的 SPI 归属，改了以后 F405_Project 跟着重编。
- 同日后来做了第 0 步（只量、不改控制）：Gimbal_test_RS05 的 `.ioc` 里 PC5 改成 EXTI5（上升沿、下拉、优先级 5），TIM2 配成 1 MHz 自由计数当时间戳；Shared 的 BMI088 驱动在 `BMI088_Init` 里打开陀螺仪 DRDY（INT3 推挽高有效）并加进回读校验，Gimbal_test、F405_Project 清理重编 0 警告（它们接 INT3 的引脚都是普通输入，没影响）；robot.c 加了计数、计时，`tools/timing_watch.py` 实时读。顺带生成了 11-15 改了 .ioc 没生成的 CAN2 自动重发 / 离线恢复（`can.c` 两行），11-15「下一步」那条做完了。CubeMX 命令行在 Claude 桌面版的 shell 里能跑（ssh 里会卡在 UserAuth）。还没烧、没上板。第 2 节第 4、5 条这一步还没变。
- 同日上板量了第 0 步：陀螺仪 ODR 999.68 Hz（慢 0.032 %，随温度约 −23 ppm/℃，加热稳在 40 ℃ 以后在 999.67~999.69）；Robot_Loop 的重复 / 跳过只在相位交界（约每 3.1 s 一次）成串出现，每次净 1 次，其余成对、每次约 29 对。“每秒约 |ODR − 1000| 次”说的是重复和跳过的差，之前写成合计是错的。Robot_Loop 102 µs、INS_Update 44 µs（-O0）。细节见 Gimbal_test_RS05/notes.md「第 0 步上板实测」。
- 同日夜里量了 CPU 时间拆分（Gimbal_test_RS05 的 `tools/cpu_profile.py`，不改固件、不停核采样 PC）：没接电机时干活约 12 %，每拍 SysTick 之后约 123 µs 做完。大头是 BMI088 的 DMA 读取（约 30 µs/拍，-O0 的 HAL 开销比线上时间还多）和每拍阻塞读温度（约 27 µs，温度寄存器 1.28 s 才更新一次）。CAN 接收中断要接上电机再量。细节见 Gimbal_test_RS05/notes.md「CPU 时间拆分」。
- 同日深夜：第 0 步原样提交并推送（`d92bde5`），之后删了 Gimbal_test_RS05 robot.c 里的测量代码，只留启动 TIM2（第 1 步当时间戳用）；`cpu_profile.py` 改成不依赖它。以后量时序、耗时先用不改固件的办法（`cpu_profile.py` 采 PC，OpenOCD 读控制代码自己的量和寄存器）；只有精确次数、最坏一次这种绕不开的，才临时往固件里加，量完删，原样留在提交里。
- 第 1 步要改的四处定了：驱动读温度时占住 SPI、温度 100 ms 读一次、EXTI 启动采样且 INS 的 dt 用时间戳、Robot_Loop 有新样本才跑并带 2 ms 兜底。代码还没写，细节见 Gimbal_test_RS05/notes.md「第 1 步要改什么」。
- 2026-11-21 第 1 步写好，还没烧。和定的不一样的两处：兜底门限改成 1.5 ms（2 ms 正好是丢一个样本时的间隔，主机模拟里兜底那一次正好落在下一个样本的 DMA 读取里）；`DJI_Motor_IsOnline` 改成有符号比（跟 DRDY 跑以后 CAN 中断打的时间戳会比 Robot_Loop 开头取的 tick 新 1）。第 2 节第 4、5 条加了例外，Shared 改动记在第 7 节。上板验收和 `cpu_profile.py`、验收脚本还没做。细节见 Gimbal_test_RS05/notes.md「第 1 步：DRDY 当节拍」。
- 同日上板查了验收 1~3（只读，加一次 OpenOCD 屏蔽 EXTI5）：陀螺仪 ODR 999.662 Hz（和第 0 步一致）；屏蔽 EXTI5 时按 tick 每 1 ms 跑、valid 0、温度照读，放开马上跟回 DRDY；INS 标定、valid 1、温度 40 ℃。USB + VOFA 10 min、电机两条和 DRDY → Robot_Loop 的延迟还没做。细节见 Gimbal_test_RS05/notes.md「第 1 步上板」。
- 同日 `cpu_profile.py` 改成以 DRDY 为零点（每 25 个 PC 样本读一次 `ins_sample_us` 连成 DRDY 时刻表，按 EXTI 中断样本归零；假样本验过），上板量了两次 120 s（-O0）：DRDY 之后约 50 µs 两段 DMA 读完、Robot_Loop 开始，约 73~83 µs 云台用上，约 105 µs 发完 CAN；DMA 那 50 µs 里线上只占约 23 µs，其余是 HAL。Robot_Loop 每拍 58.7 µs，空转 90.8 %。顺带发现约 18 % 的 DRDY 固定晚到约 46 µs（升温中、稳态都一样，和 tick 无关，CPU 没挡中断），第 0 步时很少，原因没查清，对积分和控制没影响。细节见 Gimbal_test_RS05/notes.md「cpu_profile.py 改成以 DRDY 为零点」。

## 2026-11-21 DRDY 第 1 步本地提交复核

- 当前进度：收录 RS05 云台按样本调度、1.5 ms 超时兜底、按时间戳积分、100 ms 温度读取、SPI 互斥、DJI 在线时间判断及 CPU 分析脚本更新。README 已更新接线、调度方式和 CAN2 生成状态。
- 已验证：本轮全量编译 Gimbal_test_RS05、F405_Project、Gimbal_test、Chassis_test、Power，均通过且无编译警告；CarSim、HostTest、Power 九组主机测试通过。重新编译运行既有 DRDY 模拟，所有检查通过；CPU 分析脚本使用本次 ELF 和 120 s 合成样本复跑正常，两个改动的 Python 脚本语法检查通过。
- 未验证：本轮未烧录或操作硬件。此前部分上板结果见子工程笔记；USB 连续运行、带电机验收及 DRDY 晚到原因仍待验证。
- 对外接口：INS_GyroDrdy / INS_IsSampleReady / INS_GetSampleUs，Robot_Loop 消费新样本并保留 tick 超时兜底。
- 注意事项和踩过的坑：timing_watch.py 对应第 0 步测量固件 d92bde5；临时 DRDY 模拟与 CPU 合成样本不进入仓库。AGENTS.txt、tools.zip 为本地附件。
- 下一步：按子工程笔记继续 USB 长时间运行及带电机验收。

## 2026-11-21 板间 0x110 降到 100 Hz

- 为什么：C 板 CAN1（两个 RS05 + 0x110）负载偏高，以后这路上还可能加超级电容之类的帧。按帧长算：两个 RS05 每 1 ms 一问一答，4 个扩展帧 52%~64%；0x110 一帧 111~135 bit，1 kHz 时 11%~13.5%，100 Hz 只要 1.1%~1.35%。合计 64%~78% → 54%~65%，空出约 10%~12%，差不多是一个 1 kHz 的 8 字节标准帧。超电要是也 1 kHz 发到这路上，又回到 65%~79%；100 Hz 的话只加 1% 多。
- 改了什么：
  - `Shared/Link/chassis_link.h`：加 `CHASSIS_LINK_PERIOD_MS 10U`，帧格式说明改成每 `CHASSIS_LINK_PERIOD_MS` 一帧。`CHASSIS_LINK_TIMEOUT_MS` 还是 50（100 Hz 下连丢 5 帧才 RELAX；C 板 CAN1 开了自动重发，正常不会丢）。chassis_link.c 没改，发送这边还是没有状态。
  - Gimbal_test_RS05、Gimbal_test 的 robot.c：`(now - robot_link_tick) >= CHASSIS_LINK_PERIOD_MS` 才发，发了记下 now。按 tick 差判、不按 tick 取余：RS05 版跟着陀螺仪跑，两次之间 tick 可能走 2，取余会正好跳过要发的那个 tick；平时 10 ms 一帧，tick 走 2 的时候偶尔 11 ms。节拍放在 robot.c（调度归装配层）。GM6020 版没上车，为了两个云台工程的协议一致一起改了。
  - HostTest、HostTest_RS05、CarSim 的仿真照 robot.c 改；CarSim 断线时云台板照样到点发，只是底盘板收不到。
  - HostTest 的 `chassis_cmd` 场景（HostTest_RS05 共用）：改完指令后的等待从 10 ms 改成 `CHASSIS_LINK_PERIOD_MS + 10`。原来的 10 ms 在 10 个发送相位里有 1 个会失败（IMU 失效那一步：云台锁住要下一个周期才反映到指令上，锁住的那个周期正好发帧的话，下一帧在等待之外），是在临时副本里挪相位试出来的。
- 底盘板：解包、超时都没变，F405_Project、Chassis_test 的固件和改前逐字节相同，不用重烧。云台板要重烧才生效。
- 影响（CarSim 1 kHz → 100 Hz 对比，云台是 GM6020 模型，底盘 36°/s）：
  - 小陀螺 + 按 W 1 m/s：对地方向平均偏 0.37° → 0.53°，最大 0.39° → 0.55°（多出的约 0.16° ≈ 36°/s × 平均多旧的 4.5 ms）。
  - 小陀螺时轮电流每 ms 最大变化 11 → 81（原始值，约 0.1 A）：相对角每 10 ms 跳一次 0.36°，平移分到各轮的速度目标跟着跳。`spin_wrap` 比的是跨 ±180° 附近和别处（65、81），两边差不多，照过。
  - 跟随边走边转：行进方向最多落后枪口 5.42° → 5.59°。断线到无力 42~51 ms（看最后一帧离断线多久，1 kHz 时是 51 ms）。
  - 以后小陀螺转快了要注意：底盘每转 100°/s，相对角比 1 kHz 时最多多落后约 1°，而且每 10 ms 跳一次。现在 36°/s 不要紧；转到几百 °/s 时平移方向会差几度，那时在底盘这头用自己的 wz 外推（rel − wz × 离收到这帧的时间，云台对地不动时成立），或者提高 0x110 的频率。
- 验证：F405_Project、Chassis_test、Gimbal_test、Gimbal_test_RS05 在 `%TEMP%` 清理编译 0 警告；Gimbal_test_RS05 text +20 B、Gimbal_test +24 B，反汇编里是 `cmp r3, #9`（tick 差 ≥ 10 才发）。HostTest、HostTest_RS05 各 15 过 1 跳过，CarSim 14 过。临时副本里把发送相位 0~9 挨个试（每个场景开头挪 `sim_link_tick` / `car_link_tick`，副本没进仓库）：CarSim 每个相位 14 个全过，HostTest、HostTest_RS05 每个相位都是 15 过 1 跳过。脚本（gimbal_watch.py 等）用 nm 从 ELF 查地址，多一个静态变量不影响。没烧、没上板。
- 上板看：云台板烧了以后，底盘板的 `chassis_link_tick` 每 10 ms 跳一次、离 HAL tick 不超过 10 ms 左右；跟随、小陀螺照常。
- 本次推送前复核：Gimbal_test_RS05、Gimbal_test、F405_Project、Chassis_test 在新临时目录全量编译通过且无编译警告，CarSim、HostTest、HostTest_RS05 全部通过。README 已同步 100 Hz 发送周期和 50 ms 接收超时；本轮未烧录、未上板。
