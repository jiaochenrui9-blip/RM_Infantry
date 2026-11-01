# Gimbal_test 笔记（云台测试工程）

> 本工程的进度与注意事项。跨模块约定（分层、接口、单位、正方向、CAN 分配、C 板引脚）见 `RM_Infantry/notes.md`；驱动移植细节和验证记录见本目录 `README.md`；BMI088 / Attitude 的接入说明见 `Shared/Devices/BMI088/README.md`。
> 最后更新：2026-10-31（v2 第 2 步云台 IMU 模式：用户写第一版，Claude 按审查意见改好）

## 当前状态

- .ioc：CAN1 已改到 PD0/PD1；SPI1 + DMA、BMI088 片选和数据就绪输入已配置。IST8310 的 I2C3、IMU 加热的 TIM10 还没配。
- App：编码器定角度（v1，2026-10-29 加了速度前馈）+ ins（v2 第 1 步）+ 云台 IMU 模式（v2 第 2 步），共 790 行。v1 主机闭环模型测试通过（见 README）。**云台还没接电机上板**：方向、中值、限位、PID 都是占位值。
- v2（IMU 定角度）接口 2026-10-29 已定，见「v2 接口」一节；用户自己写第一版。第 1 步 ins 已改好并接入 robot.c。2026-10-30 首次上板（只接调试器 3.3 V、没接电机）：程序正常跑，ins 桌面验收全部通过（单位矩阵下符号都符合约定），C 板轴向对应到实物的哪条边待用户确认，见「第 1 步进度」。第 2 步（云台 IMU 模式）2026-10-31 用户写了第一版，Claude 按审查意见改好：ARM 清理编译无警告（FLASH 36568 B、RAM 3376 B），主机模型测试通过；**还没上板**，见「第 2 步进度」。
- 之前带 IMU 模式的完整版（935 行）备份在 `Test_Module/Gimbal/Backup/App_full_2026-10-27`，不参与编译。用户觉得变量太多、太乱，所以重写成现在这版；反思写进了全局 CLAUDE.md。v2 以新接口为准，备份只作参考。

## 已知问题 / 坑

1. CAN1 引脚：已改为 PD0/PD1（原来 PB8/PB9 是错的）。
2. 驱动里 `speed_rpm`、`total_encoder` 和输出电流都乘了 `motor_direction`，但 `encoder`（0~8191 原始值）没乘 → `Gimbal_AxisRead()` 里自己乘了方向。
3. `DJI_Motor_GetFeedback()` 返回 0 表示没有新帧、不填结构体 → `Gimbal_AxisRead()` 直接返回，沿用上一次的值。
4. GM6020 用电流控制（0x1FE/0x2FE）：固件 ≥1.0.11.2，并在 RoboMaster Assistant（≥2.7）里打开电流环。手册没写未打开时收到 0x1FE 会怎样；“有反馈但不动”先查这个。
5. `motor_control` 的位置模式以第一次进入位置模式时的位置为零点，只能用编码器和 rpm 做反馈，不适合云台 → 云台电机设成 `MOTOR_MODE_CURRENT`，串级 PID 在 gimbal.c 里算。
6. 调试器暂停时控制帧停发，GM6020 会不会保持最后的电流手册没写，第一次上电用小电流限幅试一下。
7. 锁定：闭环时电机掉线，或 IMU 模式下 `ins_feedback.valid` 变 0 → 锁在 RELAX（`gimbal_fault = 1`），要先把 `gimbal_test.mode` 改成 0 再改回去。上电时电机还没反馈就要求闭环、上电约 1 s 内（ins 还在标定）就切 IMU 模式，同样会锁住。
8. `Robot_Init()` 里配置类错误（ID 冲突、注册失败）会进 `Error_Handler()`（关中断死循环），现象是板子像“死了”。
9. 斜坡：不限位的轴（限位写成 ±180）按最短方向走；有限位的轴不跨 ±180°，避免从背面绕（旧版有这个问题，已修）。v2 起目标先在编码器系里拉回限位内再走斜坡，所以从限位外进闭环时目标直接拉回限位，不再按斜坡走回来（主机模型：pitch 靠在 -20°、软限位 -15° 时进闭环，最多到 -14.8°，v1 是 -14.4°）。
10. IMU 模式下底盘带着云台顶在限位上转时没有前馈（diff = 0），云台会超出软限位约“底盘角速度 / 角度环 Kp”（主机模型：Kp 8、底盘 30°/s 时 3.8°，底盘停下后回到限位）。软限位余量按这个留，或调大角度环 Kp；yaw 有滑环（±180）就没有这个问题。
11. 只接调试器 3.3 V 时 CAN 用不了：CAN 收发器的 VCC 接 VCC_5V（24 V 或 USB 才有），发出的显性位读回来是隐性，CAN1 几毫秒内进入 bus-off（2026-10-30 实测：TEC 248、最近错误“显性位错误”）。.ioc 里两路 CAN 的 Automatic Bus-Off Management 都没开，bus-off 后不会自己恢复：先接调试器、后上 24 V 时要复位一次板子，或者以后在 .ioc 里打开它。

## 代码结构

```
Gimbal_test/App/
├── robot.c / robot.h            装配层：robot_can1/2、初始化顺序、1 ms 调度、CAN 接收回调
├── gimbal/
│   ├── gimbal.c / gimbal.h      云台模块（合并时整个文件夹搬走）
│   └── gimbal_config.h          ID、方向、中值、限位、限幅、斜坡、速度前馈、重力前馈、PID
├── ins/
│   ├── ins.c / ins.h            惯导：BMI088 采样、标定、换坐标、姿态解算（v2 第 1 步）
│   └── ins_config.h             安装矩阵
└── test_cmd/
    └── gimbal_test_cmd.c / .h   调试器指令源（合并后由 robot_cmd 取代）
Shared/Algorithm/user_math.*     Math_Clamp、Math_WrapDeg180
```

`Robot_Loop()` 每 1 ms：`INS_Update` → `INS_GetFeedback` → `Gimbal_SetImu` → `Gimbal_GetFeedback` → `GimbalTestCmd_Update` → `Gimbal_SetCmd` → `Gimbal_Update` → 每路 CAN `SendAll` 一次。HAL 的 SPI 完成 / 出错回调也在 robot.c，转给 BMI088 驱动。

主机测试在 `Test_Module/Gimbal/HostTest`（2026-11-01 起放进仓库）：电脑上编译 gimbal.c、gimbal_test_cmd.c、DJI 驱动、PID、user_math 的真代码，接两个刚体轴 + 底盘 + 理想 IMU 的模型，13 个场景（RELAX、阶跃、斜坡前馈、限位、扶住松手、IMU 自稳、跨 ±180°、模式切换、各种锁定）。**改了 gimbal / test_cmd / DJI 驱动就跑一遍**，用法见其 README。以前 notes 里记的“主机模型测试”是临时目录里的版本，以这个为准。

gimbal.h 对外：`GimbalMode_e`（RELAX / ENCODER / IMU）、`GimbalCmd_t`（模式 + 两个目标角，坐标系跟着模式走）、`GimbalFeedback_t`（模式 + 编码器角、IMU 角各两个），以及 Init / SetImu / SetCmd / Update / GetFeedback 五个函数。gimbal.h include ins.h 只为用 `InsFeedback_t` 类型。

gimbal.c 内部每个轴一个 `GimbalAxis_t`（9 个字段）：电机、中值、上下限位、角度环和速度环 PID、编码器角、电机角速度、斜坡后的目标（存在本模式的坐标系里，IMU 模式下是对地的角）。IMU 数据存一份 `gimbal_imu`。`Gimbal_Update` 按模式分两支，各显式调两次 `Gimbal_AxisControl`，只是传的反馈不同；RELAX 不调，电流为 0。方向存在 `motor.motor_direction`，环的输出就看 `angle_pid.out` / `speed_pid.out`，不另存。速度环目标 = 角度环输出 + 速度前馈（斜坡每毫秒走的角度换算成 dps），再限幅到 `GIMBAL_SPEED_LIMIT_DPS`，看 `speed_pid.target`。

## 怎么用（CLion 调试器）

| 变量 | 内容 |
|---|---|
| `gimbal_test` | 指令：`mode`（0 无力 / 1 编码器闭环 / 2 IMU 闭环）、`yaw_deg`、`pitch_deg`（模式 1 相对底盘，模式 2 相对地面）；方波：`yaw_square_deg`、`pitch_square_deg`（幅值，0 为不加）、`square_period_ms` |
| `gimbal_yaw`、`gimbal_pitch` | 每个轴：`angle_deg`、`rate_dps`、`target_deg`、`speed_pid.target`（目标角速度，含前馈）、`angle_pid.out`（其中角度环的部分）、`speed_pid.out`（电流）、`motor.encoder`（原始编码器） |
| `gimbal_mode`、`gimbal_fault` | 实际模式、是否被锁定（电机掉线，或 IMU 模式下 IMU 失效） |
| `gimbal_imu` | 云台收到的 IMU 数据（`Gimbal_SetImu` 拷进来的 `InsFeedback_t`），应和 `ins_feedback` 一样 |
| `ins_feedback` | IMU 输出：`valid`、`yaw_deg`、`pitch_deg`、`yaw_rate_dps`、`pitch_rate_dps`（已按全车约定） |
| `ins_init_status`、`ins_calibrate_count`、`ins_last_tick` | BMI088 初始化结果（0 正常，见 `BMI088_Status_t`）；标定进度（到 1000 为止）；最后一次样本的 tick（正常时跟着 HAL tick 走） |
| `BMI088_DATA` | 驱动里的原始数据（C 板坐标系）：`acc_g`、`gyro_dps`（已减零偏）、`gyro_offset`。标定 C 板轴向时看它 |

流程：上电 → RELAX 下手转标定中值和方向 → `gimbal_test.mode` 改成 1（IMU 模式改成 2，上电 1 s 后再切）→ 再改 `yaw_deg` / `pitch_deg`。改 mode 时目标会自动同步成新模式下的当前角，所以要先切模式、再改目标。

## 版本规划

- **v1 编码器定角度**：代码已完成，待上板标定和调参。
- **v2 IMU 定角度**：接口已定（见下一节），分三步：ins（已完成，桌面验收通过）→ 云台 IMU 模式（代码 10-31 改好、主机模型测试通过；上板验收要等 v1 调通、C 板装到云台）→ 加热恒温。
- **v3 遥控器**：DR16 接 USART3（PC11），手册写 100 kbps；其余串口参数本目录没有 DR16 手册，用前核对。
- **合并准备**：定单板 / 双板；按根目录 notes.md 的 CAN 表分配负载。

## v2 接口（2026-10-29 定，用户自己写第一版）

分三步，每步单独验收：

1. **ins 模块**：BMI088 + 姿态解算。C 板单独放桌上就能测，不依赖云台，现在就能写。
2. **云台 IMU 模式**：代码可以先写；上板验收等 v1 调通、C 板装到云台后再做。两个模式用同一套串级 PID，PID 没调好时两边的问题会叠在一起。
3. **加热恒温**：以后（见本节最后）。

按 **C 板装在 pitch 上、随枪管俯仰** 设计。如果装在 yaw 上，IMU 测的是 yaw 平台的姿态，pitch 的 IMU 角要改成 IMU 角 + 编码器角，到时重定。

数据流：BMI088（DMA）→ ins（乘安装矩阵 → `Attitude_Update` → 换成全车约定）→ robot.c 把 `INS_GetFeedback` 的结果原样传给 `Gimbal_SetImu` → gimbal。gimbal.h include ins.h 只为用 `InsFeedback_t` 这个类型，不调 ins 的函数（根目录 notes 第 2 节规则 1 的例外）。

`Robot_Loop()` 每 1 ms（第 2 步之后）：`INS_Update` → `INS_GetFeedback` → `Gimbal_SetImu` → `Gimbal_GetFeedback` → `GimbalTestCmd_Update` → `Gimbal_SetCmd` → `Gimbal_Update` → 每路 CAN `SendAll`。

### 第 1 步：ins（App/ins/ins.h、ins.c、ins_config.h）

```c
/* ins.h */
typedef struct
{
    uint8_t valid;              /* 1：初始化成功、标定完、最近 10 ms 内有新样本 */
    float yaw_deg;              /* 俯视逆时针为正，-180~180。没有磁力计：以上电朝向为 0，会漂 */
    float pitch_deg;            /* 抬头为正 */
    float yaw_rate_dps;         /* 绕竖直轴的角速度 */
    float pitch_rate_dps;       /* 绕 pitch 轴（云台头 y 轴）的角速度，抬头为正 */
} InsFeedback_t;

/* 只做 BMI088_Init（阻塞约 85 ms）。返回 BMI088_Status_t，0 为成功；失败时 valid 一直为 0，编码器模式照常用。 */
uint8_t INS_Init(SPI_HandleTypeDef *hspi);
/* 每 1 ms 调一次：先处理上个周期 DMA 读回的样本，再 BMI088_StartSample() 启动下一次。
   前 1000 个样本（约 1 s，整车静止）求陀螺仪零偏，之后才解算。 */
void INS_Update(uint32_t now_tick);
void INS_GetFeedback(InsFeedback_t *feedback);
```

```c
/* ins_config.h */
/* 安装矩阵：v_head = M · v_board。云台头坐标系：x 朝枪口、y 朝左、z 朝上（云台在中值、水平时）。
   每行只有一个 ±1：第 i 行 = 云台头第 i 轴是 C 板的哪个轴、什么符号。待标定 */
#define INS_INSTALL_MATRIX  {{1.0f, 0.0f, 0.0f}, {0.0f, 1.0f, 0.0f}, {0.0f, 0.0f, 1.0f}}
```

- 安装矩阵代替旧版的 `ROBOT_IMU_PITCH_USE_ROLL` 开关和几个符号宏，C 板怎么装都只改这一处。
- 零偏标定（用户第一版的做法，比原定的阻塞 `BMI088_CalibrateGyro` 好）：在 `INS_Update` 里用前 1000 个 DMA 样本求平均，`BMI088_SetGyroOffset()` 写回驱动。Robot_Init 不卡住，标定期间云台、CAN 照常跑；以后加热只是在标定前多一个阶段。
- ins.c 内部 static：`INS_Init` 的返回值、标定计数（阶段由它决定：前 1000 个样本标定，之后解算）、最后一次样本的 tick、标定用的角速度和加速度累加和、`InsFeedback_t`。dt 用宏写死 0.001f。
- 初始姿态：标定时顺便累加加速度，标定结束用平均值（转到云台头坐标系后）调 `Attitude_InitFromAcc`，roll / pitch 由重力算、yaw = 0，所以标定完姿态就是对的，不用预热。平均加速度不在 0.8~1.2 g 时清零重新标定。
- `valid` = 初始化成功 && 标定完 && 最近 10 ms 内有新样本，初值 0，每次 `INS_Update` 都重算。
- `INS_Update`：不管有没有新样本，最后都要 `BMI088_StartSample()`，不能提前 return。有新样本 → `BMI088_ProcessSample` + `BMI088_GetData`（C 板坐标系）→ 标定阶段累加；之后：局部副本里的 `gyro_dps`、`acc_g` 原地乘安装矩阵 → `Attitude_Update(&副本, NULL, dt)` → 换成全车约定写进 feedback。
- 故障恢复：`BMI088_Init` 失败就一直无效，不在 1 ms 循环里重试（它里面的 HAL_Delay 共约 85 ms，会卡住云台控制）；单次 SPI / DMA 出错由驱动处理（错误回调置 ERROR，下一次 `BMI088_StartSample` 自动复位重试）；一直没样本就靠 10 ms 超时让 valid 变 0，恢复后自动回到 1。ins 不锁定，锁定由云台的 `gimbal_fault` 做。
- 换成全车约定（gx、gy、gz 为云台头坐标系的陀螺仪）：`yaw = Attitude 的 yaw`；`pitch = −Attitude 的 pitch`，`pitch_rate = −gy`；`yaw_rate = cos(pitch)·gz + sin(pitch)·gx`。
- robot.c：`Robot_Init()` 里在 `DJI_MotorManager_Start()` 之前 `(void)INS_Init(&hspi1)`；`Robot_Loop()` 第一步 `INS_Update(now)`；`HAL_SPI_TxRxCpltCallback` / `HAL_SPI_ErrorCallback` 转给 `BMI088_SPI_TxRxCpltCallback` / `BMI088_SPI_ErrorCallback`。
- CMake 加 `App/ins/ins.c` 和 include 目录 `App/ins`（BMI088.c、Attitude.c 已经在里面）。.ioc 不用改。

容易错的地方：

1. 安装矩阵必须是旋转（右手系，行列式 +1）。只把一个轴取反会变成镜像：转动时陀螺仪往一个方向积分，停下后加速度计又把角度慢慢拉到反方向。做法：先定 z（云台水平静止时读 +1 g 的轴）和 x（朝枪口的轴），再按 y = z × x 定 y。
2. C 板上 BMI088 的轴向在用户手册的 IMU 章节里没写，要实测：看 `BMI088_DATA.acc_g` 哪个轴朝上时读 ≈ +1；按右手定则转板子，看 `BMI088_DATA.gyro_dps` 哪个轴为正。
3. 矩阵乘在 Mahony 之前：gyro、acc 两个向量先转到云台头坐标系再喂给 `Attitude_Update`，不要解算完再换欧拉角（C 板竖着装时，C 板系的欧拉角在 pitch ±90° 附近奇异）。
4. pitch 取负的原因：`Attitude` 是 ZYX 欧拉角，在 x 前、y 左、z 上的坐标系里绕 +y 正转是低头。yaw 绕 +z 正转本来就是俯视逆时针，不用改。
5. yaw 角速度不能只用 gz：云台抬头 θ 时，竖直轴在云台头坐标系里是 (sin θ, 0, cos θ)，只用 gz 会小 cos θ 倍（20° 时小 6%）。这里用 IMU 自己的 pitch，没算 roll，平地上够用。
6. 初始姿态：`Attitude_InitializeQuaternion` 要求磁场在 30~70 µT，没有磁力计用不了。从单位四元数起步时 roll / pitch 靠加速度计拉回来，时间常数约 1/Kp = 0.5 s（主机测试：低头 15° 起步，0.5 s 到 63%，约 2.5 s 到位）。陀螺仪的平均值只是零偏，不含姿态信息，初始化四元数要用加速度的平均值。
7. 零偏标定时整车不能动：被碰了零偏就算错，yaw 会明显漂，重新上电即可。

桌面验收（C 板单独放桌上，先用单位矩阵）：

1. `INS_Init` 返回 0（1 / 2 是 ID 不对，4 是 SPI 出错，5 是配置回读不对）。
2. 静止：`ins_last_tick` 跟着 HAL tick 走（说明每毫秒都有新样本）；`ins_calibrate_count` 约 1 s 数到 1000，同时 `valid` 变 1，之前一直是 0；pitch 稳定。
3. 记下 C 板轴向（见上面第 2 条），按“C 板平放在桌上”写安装矩阵，让 x 前、y 左、z 上。
4. 手转板子：俯视逆时针 → `yaw_deg` 增大、`yaw_rate_dps` 为正；x 轴那头抬起 → `pitch_deg` 增大、`pitch_rate_dps` 为正；转 90° 再转回来，角度也跟着变 90° 再回来。
5. 静止 1 分钟，记 yaw 漂了多少（没有恒温时漂得明显是正常的，见第 3 步）。

#### 第 1 步进度

- 2026-10-29 用户写了第一版（原样备份在 `Test_Module/Gimbal/Backup/ins_v0_2026-10-29`）。审查发现：没有启动 DMA 采样；valid 一上来就是 1 且不再更新；函数名和头文件不一致（robot.c 一调用就链接失败）；没用安装矩阵，pitch 和 pitch_rate 符号反，yaw_rate 只用 gz；`ins_Error` 置的 INITING 状态没有代码处理；没接入 robot.c。零偏标定的计数和求平均是对的。
- 同日用户让 Claude 按审查意见改好：重写 ins；`Shared/Algorithm/Attitude` 加 `Attitude_InitFromAcc`；robot.c 接入 `INS_Init` / `INS_Update` 和两个 SPI 回调；CMake 把 ins.c 放进 `target_sources`，加 include 目录 `App/ins`。
- 验证：ARM 清理编译无警告（FLASH 36124 B、RAM 3352 B）。主机测试用 ins.c 原文件 + 真 Attitude.c + 模拟 BMI088（测试代码在临时目录，不在工程里），单位矩阵和“C 板竖装”矩阵各 21 项全部通过：标定中 valid = 0；第 1000 个样本处理完 valid = 1，此刻 pitch 已是真值；零偏估计准确；抬头时 pitch_rate 为正；抬头 20° 时 yaw_rate = 30.000；最后一个样本之后 11 ms valid 变 0，恢复后自动回 1；初始化失败、加速度全 0 时一直无效。故意用错安装矩阵时测试会失败。
- 2026-10-30 首次上板（只接调试器 3.3 V，板子静止）：用 OpenOCD 读内存、不停核，板子 Flash 与 CLion 编出的 ELF 逐字节一致。主循环每 1 ms 正常跑（`robot_last_tick` 跟着 HAL tick），没有 HardFault 记录；时钟 168 MHz，PD0 / PD1 是 AF9。`ins_init_status` = 0（加速度计 ID 0x1E、陀螺仪 ID 0x0F），标定完成，`valid` = 1，`ins_last_tick` 与 HAL tick 相同。桌面验收第 1、2 条通过；数据见标定记录。
- 2026-10-30 手转测试（单位矩阵，OpenOCD 不停核约 32 Hz 连续读 `ins_feedback`、`ATTITUDE_DATA`、`BMI088_DATA`）：
  - 俯视逆时针转约 90°：yaw +93.4°，yaw_rate 峰值 +242 dps，陀螺仪积分绕 +z +92.7°；转回：yaw −93.9°。
  - 抬起 +x 那一侧约 29°：pitch +29.0°，抬的过程中 pitch_rate 为正；抬起 −x 那一侧约 32°：pitch −31.9°，和用加速度算出的倾角 31.9° 一致；放平后 pitch 回到约 0.5°~0.9°（板子本身的倾斜）。
  - 抬起 +y 那一侧到接近竖直：roll +88.9°，加速度 y 读 +0.98 g，pitch 基本不变。
  - 结论：BMI088 坐标系是右手系，单位矩阵下 yaw、pitch 和两个角速度的符号都符合约定（俯视逆时针为正、抬头为正）；valid 全程为 1。桌面验收第 1~5 条通过。
- 下一步：用户确认抬的几条边对应实物上的哪条边，把 C 板轴向记进标定记录（装到云台后定安装矩阵要用）。第 2 步已开始写。

### 第 2 步：云台 IMU 模式（2026-10-30 细化，用户自己写第一版）

代码现在就能写。上板验收要 C 板装到 pitch 上、接 24 V 和电机，并且先按「v1 调试步骤」调通编码器模式（两个模式共用一套 PID）。只接调试器 3.3 V 时电机离线，ENCODER / IMU 一进就被掉线锁定，只能验编译、看 RELAX 下 `gimbal_imu` 跟着 `ins_feedback` 变。

```c
/* gimbal.h 的改动 */
#include "ins.h"                /* 新增：只用 InsFeedback_t 这个类型，不调 ins 的函数 */

typedef enum
{
    GIMBAL_MODE_RELAX = 0,
    GIMBAL_MODE_ENCODER,        /* 目标角相对底盘正前方 */
    GIMBAL_MODE_IMU             /* 目标角相对地面：pitch 以水平为 0，yaw 以上电时的朝向为 0 */
} GimbalMode_e;

/* GimbalCmd_t 结构不变，只改注释：yaw_deg / pitch_deg 的坐标系跟着 mode 走 */

typedef struct
{
    GimbalMode_e mode;          /* 实际模式，可能和指令不同 */
    float yaw_rel_deg;          /* 原 yaw_deg 改名（根目录 notes 第 3 节）：任何模式下都是编码器角，底盘跟随用它 */
    float pitch_rel_deg;
    float yaw_imu_deg;          /* 新增：IMU 角，指令源切到 IMU 模式时用它同步目标 */
    float pitch_imu_deg;
} GimbalFeedback_t;

/* 新增：每周期在 Gimbal_GetFeedback、Gimbal_Update 之前调，robot.c 把 INS_GetFeedback 的结果直接传进来。
   valid 为 0 时不许进 IMU 模式；IMU 模式中变 0 → 无力并锁住 */
void Gimbal_SetImu(const InsFeedback_t *imu);
```

gimbal.c：

- 新增 static `InsFeedback_t gimbal_imu`，`Gimbal_SetImu` 整个拷进来（和 `gimbal_cmd` 一样存一份）；初值全 0，即 valid = 0。`GimbalAxis_t` 不加字段。
- `Gimbal_SetCmd`：ENCODER、IMU 原样收下，其他值当 RELAX。
- `Gimbal_AxisControl(axis, cmd_deg, fb_deg, fb_dps)`：多两个参数，传本模式的反馈角和反馈角速度。编码器模式传 `axis->angle_deg`、`axis->rate_dps`；IMU 模式传 `gimbal_imu` 里这个轴的角度和角速度。`target_deg` 存在本模式的坐标系里（IMU 模式下是对地的角）。按这个顺序算（wrap = `Math_WrapDeg180`，clamp = `Math_Clamp`）：
  1. `offset = wrap(fb_deg − axis->angle_deg)`：本模式坐标系比编码器系多出来的角，编码器模式下正好是 0。
  2. `cur = clamp(wrap(target_deg − offset), min, max)`：现在的目标换到编码器系，并拉回限位内。
  3. `goal = clamp(wrap(cmd_deg − offset), min, max)`：指令换到编码器系再限位。
  4. `diff = goal − cur`；不限位的轴（限位差 ≥ 360°）折到 ±180°；再限到 ±`GIMBAL_RAMP_DEG_PER_MS`。
  5. `target_deg = wrap(cur + diff + offset)`：换回本模式坐标系。前馈仍是 `GIMBAL_SPEED_FF_RATIO × diff / dt`。
  6. 后面和 v1 一样，只是角度环的反馈换成 `fb_deg`，速度环的反馈换成 `fb_dps`。
- 编码器模式下 offset = 0，结果和 v1 只有一处不同：目标在限位外时（比如刚进闭环时 pitch 靠在机械限位上），第 2 步直接把它拉回限位内，不再按斜坡走回来（角速度仍受 `GIMBAL_SPEED_LIMIT_DPS` 限制）。
- `Gimbal_UpdateMode`：
  - 锁定多一个条件：指令是 IMU 而 `gimbal_imu.valid == 0` → `gimbal_fault = 1`；解除方法不变（指令给一次 RELAX）。
  - 模式变了：两个角度环都清；两个速度环只在从 RELAX 进闭环时清，ENCODER ↔ IMU 之间不清。
  - 目标同步成新模式的反馈角：IMU 模式用 `gimbal_imu` 的角，其他用编码器角。
- `Gimbal_Update`：两种闭环模式各按自己的反馈调 `Gimbal_AxisControl`；pitch 重力前馈两种模式都加，继续用编码器 pitch。
- `Gimbal_GetFeedback`：rel 填编码器角，imu 填 `gimbal_imu` 的角。

为什么这样写：

- 限位按编码器角，因为机械限位是相对底盘的。不要在 IMU 坐标系里直接比大小：底盘朝向 170° 时，±90° 的限位在 IMU 坐标系里是一段跨过 ±180° 的区间。
- 斜坡步长在编码器系里算、加到本模式的目标上：底盘转、指令不变时，`goal − cur` 里的 offset 抵消，diff = 0，云台对地不动，前馈也是 0。
- 第 2 步先把目标拉回限位内：底盘转得比斜坡（100°/s）快时，目标也不会被带到限位外；到了限位，目标每周期被拉回限位，云台跟着底盘走。2026-10-29 的草稿没有这一步，目标会暂时落在限位外。
- 速度环在 ENCODER ↔ IMU 之间不清：它的积分抵消的是重力、摩擦，和用哪个坐标系无关；清掉的话，pitch 靠积分扛重力时一切模式就往下掉。角度环要清：编码器角和 IMU 角可能差上百度，PID 里存的上一次反馈不清掉，微分项会冲一下。
- IMU 模式的速度环用 IMU 角速度，不用电机转速：电机转速是相对底盘的，底盘一转就和对地角速度不一样。
- 重力前馈继续用编码器 pitch：代码不用改；编码器中值按水平标定，平地上和 IMU pitch 一样。严格说重力矩跟着对水平面的角度走，上坡时 IMU pitch 更准，以后需要再改。

gimbal_test：`mode` 多一个 2（IMU）；切模式时按新模式同步目标：2 用 `yaw_imu_deg` / `pitch_imu_deg`，其他用 `yaw_rel_deg` / `pitch_rel_deg`。

robot.c：`Robot_Loop()` 第 0 步改成 `INS_Update(now)`、`INS_GetFeedback(&ins)`、`Gimbal_SetImu(&ins)` 三行，往下和现在一样。CMake、.ioc 不用改（没有新文件，`App/ins` 已经在 include 目录里）。

容易错的地方：

1. IMU 角和编码器角方向相反时，IMU 模式是正反馈，一闭环就甩到限位。所以上板先在 RELAX 下手转核对方向（验收第 1 条）；不一致改安装矩阵，不改电机方向（电机方向在 v1 已按编码器标好）。
2. 上电后约 1 s 内 ins 还在标定，valid = 0，这时切 IMU 模式会锁住，给一次 RELAX 再切。标定期间别碰云台，零偏会算错。
3. 没有磁力计、没有恒温，yaw 会漂（2026-10-30 实测静止约 0.3°/分钟）：IMU 模式下底盘不动，云台也会相对底盘慢慢转。这是 ins 的问题，不用在控制里找；pitch 有重力校正，不漂。
4. 调试器里看 IMU 模式：`gimbal_yaw.target_deg` 是对地的角；`angle_deg`、`rate_dps` 还是编码器角和电机转速；PID 实际用的反馈看 `angle_pid.actual`、`speed_pid.actual`。

验收（C 板装在 pitch 上、接 24 V 和电机、v1 已调通）：

0. 定安装矩阵：云台在中值、水平时静止，`BMI088_DATA.acc_g` 读 +1 g 的轴是 z（矩阵第 3 行），C 板朝枪口的轴是 x（第 1 行），y = z × x（第 2 行），见第 1 步容易错的第 1 条。
1. RELAX 下手转云台：`gimbal_imu` 的角度、角速度和编码器角（`gimbal_yaw.angle_deg` 等）同方向变；水平时 IMU pitch 和编码器 pitch 应接近。
2. 编码器模式和 v1 一样（回归）。
3. IMU 模式：手慢慢转底盘，云台对地朝向不变；转到超过 yaw 限位后，云台被限位带着走；底盘前后倾（不超过 pitch 限位），枪管仰角不变。
4. 0 / 1 / 2 来回切，云台不跳；1 ↔ 2 切换时 pitch 不往下掉。
5. IMU 失效：暂停，把 `BMI088_DATA.initialized` 改成 0 再继续（暂停期间控制帧停发，见已知问题第 6 条，电流限幅保持小值）→ 驱动不再启动采样，10 ms 内 `ins_feedback.valid` 变 0，云台无力、`gimbal_fault` = 1。改回 1 → valid 回到 1，云台仍锁着，直到 `gimbal_test.mode` 给一次 0。2026-10-29 草稿写的是改 `ins_init_status`，不行：`INS_Update` 看到它非 0 直接 return，valid 不重算，一直停在 1。

#### 第 2 步进度

- 2026-10-30 细化接口（相对 10-29 草稿改了三处：限位前先把目标拉回限位内；ENCODER ↔ IMU 切换不清速度环；验收第 5 条换成改 `BMI088_DATA.initialized`）。
- 同日在主机上只验证了上面 `Gimbal_AxisControl` 第 1~5 步的算法（真 user_math.c，测试代码在临时目录，不是用户的代码）：编码器模式、目标在限位内起步时，200 组随机指令各 2 万周期和 v1 逐周期一致；目标在限位外起步时直接拉回限位（v1 是按斜坡走回）；IMU 模式指令不变、底盘慢转（峰值 50°/s）时对地目标不动、diff = 0；限位 ±90° 时底盘 360°/s 一直转，编码器系里的目标最大 90.0000°，不出限位；不限位的 yaw 底盘 720°/s 转，对地目标不变；底盘朝 170°、指令跨 ±180° 时不从背面绕。
- 2026-10-31 用户觉得在主循环里逐字段拷太乱：删掉 `GimbalImu_t`（字段和 `InsFeedback_t` 完全一样），`Gimbal_SetImu` 直接收 `const InsFeedback_t *`，gimbal.h include ins.h 只用类型；根目录 notes 第 2 节规则 1、2 同步改。
- 2026-10-31 用户写了第一版的 gimbal.c / gimbal.h / robot.c（gimbal_test_cmd 还没改，整个工程还编不过）。审查方法：单文件 ARM 编译；主机模型（两个刚体轴，J = 0.005 kg·m²，无重力，编码器和 rpm 按实际量化）跑用户原文件，只在测试用的包装文件里补了一句 `Gimbal_SetImu`，和 v1（git HEAD）对比。
  - 对的：枚举、`GimbalFeedback_t`、`Gimbal_SetCmd` 收 IMU、`#include "ins.h"` + `InsFeedback_t gimbal_imu`、按模式选反馈再传进 `Gimbal_AxisControl`、`Gimbal_GetFeedback`。编码器模式阶跃（0→20°）峰值 21.1°、293 ms 进 ±0.5°；IMU 模式在限位内时底盘转动、对地朝向保持住。
  - 要改：① RELAX 也在算 PID、输出电流（切 RELAX 后电流打到 5000，yaw 转遍 ±180°；v1 为 0）；② 斜坡步长写成了 `goal − fb_deg`（v1 是 `goal − target_deg`）：云台跟不上时目标一直往前跑（推到 +10° 扶 1 s 松手，冲到 −48.4°；v1 −0.2°），前馈变成增益 1000/s 的位置 P 项（停在 20° 时电流 −891~1005；v1 −6~7）；③ IMU 模式没按编码器系限位（底盘 30°/s 转到 150°，编码器角到 −150°，限位 −90° 没起作用）；④ 切 IMU 模式时目标同步成编码器角（IMU yaw 比编码器大 40° 时对地甩 12°）；⑤ 没检查 valid（运动中 IMU 冻结、valid = 0，1 s 内又转了至少 180°，电流 3773）；⑥ 没写 `Gimbal_SetImu` 的定义（robot.c 一调用就链接失败）；⑦ robot.c 删了下指令的三行（云台收不到指令，`feedback`、`cmd` 未使用警告）。另外：重力前馈、速度目标限幅（±`GIMBAL_SPEED_LIMIT_DPS`）丢了；ENCODER ↔ IMU 仍清速度环；两个轴用 for 循环 + 三目 + 指针比较选数据，不如两次显式调用清楚（循环里也不好单独给 pitch 加重力前馈）。`Gimbal_Select_Parameter` 本身按模式、按轴取的反馈是对的；RELAX 输出电流的根子在 `Gimbal_Update` 不分模式都调控制。
  - 同日用户让 Claude 按审查意见改（用户第一版的 gimbal.c / gimbal.h / robot.c 原样备份在 `Test_Module/Gimbal/Backup/gimbal_imu_v0_2026-10-31`）：`Gimbal_AxisControl` 前半段按上面第 1~5 步写，速度目标限幅加回；`Gimbal_Update` 按模式分两支显式调用，RELAX 不调，重力前馈两个闭环模式都加；`Gimbal_UpdateMode` 加 IMU 失效锁定、切 IMU 时目标同步成 IMU 角、速度环只在从 RELAX 进闭环时清；删掉 `Gimbal_Select_Parameter`；补 `Gimbal_SetImu`；robot.c 加回下指令的三行、`Gimbal_SetImu` 挪到第 0 步；gimbal_test 加模式 2、按新模式同步目标；gimbal_config.h 的限位注释写明两个模式都按编码器角限。
  - 验证：ARM 清理编译无警告（FLASH 36568 B、RAM 3376 B）。主机模型（同上）：编码器模式 T1 RELAX 电流 0、0→20° 阶跃峰值 21.76° / 430 ms、指令超限位、手推 10° 扶 1 s 松手（-0.17°~10°）、停住时电流 -6~7，和 v1 逐项相同；IMU 模式：IMU 与编码器差 40° 时切模式对地 0.000° 不动；0/1/2 来回切 7 次，切换后 300 ms 内 yaw、pitch 最多动 0.000°；对地 0→20° 阶跃峰值 21.70° / 432 ms；转动中 IMU 冻结、valid = 0 → 下一周期锁住、电流 0，valid 恢复后仍锁着，给一次 RELAX 再切才回 IMU；上电标定中（valid = 0）切 IMU → 锁住；IMU 模式电机掉线 → 锁住；底盘 30°/s 转 150°：对地朝向保持（2 s 时 0.12°），到限位后被带着走，最多超出 3.8°（见已知问题第 10 条）。pitch 加 0.2 N·m 重力、速度环 Ki 200 时编码器 → IMU 切换只掉 0.085°（四个 PID 全清时掉 3.69°）；yaw 限位改成 ±180、底盘 360°/s 小陀螺 5 s，对地朝向偏 1.45°（速度环只有 P，抵消摩擦的稳态误差）。
  - 下一步：上板。先按「v1 调试步骤」调通编码器模式，再做上面的验收 0~5。

### 第 3 步（以后）：加热恒温

- 原因：规格书里陀螺仪零偏温漂典型值 ±0.015 °/s/K。标定后板子如果升温 10 ℃，零偏可能变 0.15 °/s，yaw 每分钟漂约 9°。
- 要做的：.ioc 配 TIM10_CH1（PF6）PWM；用 `BMI088_ReadTemperature` 读温度（阻塞读，要在 DMA 空闲时调，即处理完样本、启动下一次之前；温度寄存器 1.28 s 才更新一次）；PID 控占空比，目标比板子正常工作温度高 15~20 ℃（用户手册）。
- 对接口的影响：要先升到温度再标定零偏。标定已经在 `INS_Update` 里了，加热只是在标定前加一个“等温度到”的阶段；它的长短不是固定样本数，到时候用状态枚举比用样本计数清楚。

## v1 调试步骤与验收

1. 看反馈：上电默认 RELAX，只发 0 电流。手转时 `gimbal_yaw.motor.encoder` 跟着变。
2. 标定：扶到正前方 / 水平，记下 `motor.encoder` 填进 `GIMBAL_*_ECD_OFFSET`；pitch 推到上下极限记 `angle_deg`，填软限位（留几度余量）；按约定正方向手转，`angle_deg` 和 `rate_dps` 都应变大，否则方向改 -1。
3. 首次闭环：电流限幅保持小值，`mode` 改 1，把目标改动 5° 左右，云台应往目标方向走。
4. 速度环：看 `speed_pid.target`（目标角速度）和 `rate_dps` 的跟随，先调 P 再加 I（pitch 有重力，P 不够时会下垂，靠 I 或 `GIMBAL_PITCH_GRAVITY_FF`）。
5. 角度环：手推看回位；把 `yaw_square_deg` 设成 10 左右用方波看超调和调节时间（斜坡会把阶跃变缓，看阶跃时可临时把 `GIMBAL_RAMP_DEG_PER_MS` 调大）。斜坡运动中 `target_deg - angle_deg` 应接近 0；把 `GIMBAL_SPEED_FF_RATIO` 改成 0 可对比无前馈时的落后量（约为斜坡速度 / 角度环 Kp）。前馈在斜坡停下时会突然归零，若停止时有超调，可把系数调到 1 以下或提高速度环带宽。
6. 保护：拔掉一个电机的 CAN 线，整个云台应无力，`gimbal_fault` 变 1。

验收：两轴稳在设定角，手推回位不振荡，阶跃无明显超调，掉线自动无力。电流限幅调好再放开。

调参看波形：USB 虚拟串口（CubeMX 开 USB_OTG_FS Device_Only + USB_DEVICE 的 CDC 类，时钟 PLLQ 改 7 得到 48 MHz）+ VOFA+ 等上位机。VOFA+ 的 JustFloat 格式：若干个小端 float，后接帧尾 `0x00 0x00 0x80 0x7F`。

## 标定记录（测完填）

| 项 | 值 |
|---|---|
| yaw / pitch 所在总线、电调 ID | 代码默认 CAN1，ID1 / ID2，待按实际接线确认 |
| yaw 中值（encoder） | 待测 |
| pitch 中值（encoder） | 待测 |
| pitch 上限 / 下限（deg） | 待测（代码占位 20 / -15） |
| yaw 限位 | 待定（代码占位 ±90；有滑环就写 ±180） |
| yaw / pitch 的 motor_direction | 待测 |
| 速度环 / 角度环 PID | 待调（代码占位：角度 Kp 8，速度 Kp 20） |
| C 板轴向（静止时读 +1 g 的轴朝哪、各轴右手定则正转方向） | 2026-10-30 实测：板子平放时 +z 朝上（读 +0.99 g，当时哪面朝上待确认）；手转测试第一段第 4 步抬起的边在 +y 方向，第二段先抬的边在 −x 方向、后抬的在 +x 方向。面对 +y 那条边时 +x 在右手边。对应实物的哪条边待用户确认 |
| 陀螺仪零偏、噪声（未恒温） | 2026-10-30：零偏 (0.226, −0.027, 0.090) dps，约 9 分钟后复测均值差 ≤ 0.012 dps；单个样本噪声 0.20~0.25 dps；加速度噪声约 3.5 mg，静止时模长约 0.991 g |
| 安装矩阵 `INS_INSTALL_MATRIX` | 待定（C 板装到云台后定；代码占位为单位矩阵） |
| 静止 yaw 漂移（未恒温） | 2026-10-30：上电后约 9 分钟内平均约 −0.3°/分钟（前 5 分钟约 −0.4，之后约 −0.24），前提是期间板子没动 |
