# Gimbal_test 笔记（云台测试工程）

> 本工程的进度与注意事项。跨模块约定（分层、接口、单位、正方向、CAN 分配、C 板引脚）见 `RM_Infantry/notes.md`；驱动移植细节和验证记录见本目录 `README.md`；BMI088 / Attitude 的接入说明见 `Shared/Devices/BMI088/README.md`。
> 最后更新：2026-10-29（v2 第 1 步：ins 改好并接入 robot.c）

## 当前状态

- .ioc：CAN1 已改到 PD0/PD1；SPI1 + DMA、BMI088 片选和数据就绪输入已配置。IST8310 的 I2C3、IMU 加热的 TIM10 还没配。
- App 是精简版的 v1（只有编码器定角度），共 499 行：主机闭环模型测试通过（见 README）。2026-10-29 加了速度前馈；同日在本机用 ARM 工具链完整清理编译通过，无警告（FLASH 24128 B、RAM 3136 B）。**还没上板**：方向、中值、限位、PID 都是占位值。
- v2（IMU 定角度）接口 2026-10-29 已定，见「v2 接口」一节；用户自己写第一版。第 1 步 ins 已改好并接入 robot.c：ARM 编译无警告、主机测试通过，**还没烧录做桌面验收**，见「第 1 步进度」。
- 之前带 IMU 模式的完整版（935 行）备份在 `Test_Module/Gimbal/Backup/App_full_2026-10-27`，不参与编译。用户觉得变量太多、太乱，所以重写成现在这版；反思写进了全局 CLAUDE.md。v2 以新接口为准，备份只作参考。

## 已知问题 / 坑

1. CAN1 引脚：已改为 PD0/PD1（原来 PB8/PB9 是错的）。
2. 驱动里 `speed_rpm`、`total_encoder` 和输出电流都乘了 `motor_direction`，但 `encoder`（0~8191 原始值）没乘 → `Gimbal_AxisRead()` 里自己乘了方向。
3. `DJI_Motor_GetFeedback()` 返回 0 表示没有新帧、不填结构体 → `Gimbal_AxisRead()` 直接返回，沿用上一次的值。
4. GM6020 用电流控制（0x1FE/0x2FE）：固件 ≥1.0.11.2，并在 RoboMaster Assistant（≥2.7）里打开电流环。手册没写未打开时收到 0x1FE 会怎样；“有反馈但不动”先查这个。
5. `motor_control` 的位置模式以第一次进入位置模式时的位置为零点，只能用编码器和 rpm 做反馈，不适合云台 → 云台电机设成 `MOTOR_MODE_CURRENT`，串级 PID 在 gimbal.c 里算。
6. 调试器暂停时控制帧停发，GM6020 会不会保持最后的电流手册没写，第一次上电用小电流限幅试一下。
7. 掉线锁定：闭环时电机掉线 → 锁在 RELAX（`gimbal_fault = 1`），要先把 `gimbal_test.mode` 改成 0 再改回 1。上电时电机还没反馈就要求闭环，同样会锁住。
8. `Robot_Init()` 里配置类错误（ID 冲突、注册失败）会进 `Error_Handler()`（关中断死循环），现象是板子像“死了”。
9. 斜坡：不限位的轴（限位写成 ±180）按最短方向走；有限位的轴从限位外回来时不跨 ±180°，避免从背面绕（旧版有这个问题，已修）。

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

`Robot_Loop()` 每 1 ms：`INS_Update` → `Gimbal_GetFeedback` → `GimbalTestCmd_Update` → `Gimbal_SetCmd` → `Gimbal_Update` → 每路 CAN `SendAll` 一次。ins 的结果现在还没喂给云台（v2 第 2 步）。HAL 的 SPI 完成 / 出错回调也在 robot.c，转给 BMI088 驱动。

gimbal.h 对外：`GimbalMode_e`（RELAX / ENCODER）、`GimbalCmd_t`（模式 + 两个目标角）、`GimbalFeedback_t`（模式 + 两个当前角），以及 Init / SetCmd / Update / GetFeedback 四个函数。

gimbal.c 内部每个轴一个 `GimbalAxis_t`（9 个字段）：电机、中值、上下限位、角度环和速度环 PID、当前角、角速度、斜坡后的目标。方向存在 `motor.motor_direction`，环的输出就看 `angle_pid.out` / `speed_pid.out`，不另存。速度环目标 = 角度环输出 + 速度前馈（斜坡每毫秒走的角度换算成 dps），再限幅到 `GIMBAL_SPEED_LIMIT_DPS`，看 `speed_pid.target`。

## 怎么用（CLion 调试器）

| 变量 | 内容 |
|---|---|
| `gimbal_test` | 指令：`mode`（0 无力 / 1 闭环）、`yaw_deg`、`pitch_deg`；方波：`yaw_square_deg`、`pitch_square_deg`（幅值，0 为不加）、`square_period_ms` |
| `gimbal_yaw`、`gimbal_pitch` | 每个轴：`angle_deg`、`rate_dps`、`target_deg`、`speed_pid.target`（目标角速度，含前馈）、`angle_pid.out`（其中角度环的部分）、`speed_pid.out`（电流）、`motor.encoder`（原始编码器） |
| `gimbal_mode`、`gimbal_fault` | 实际模式、是否被掉线锁定 |
| `ins_feedback` | IMU 输出：`valid`、`yaw_deg`、`pitch_deg`、`yaw_rate_dps`、`pitch_rate_dps`（已按全车约定） |
| `ins_init_status`、`ins_calibrate_count`、`ins_last_tick` | BMI088 初始化结果（0 正常，见 `BMI088_Status_t`）；标定进度（到 1000 为止）；最后一次样本的 tick（正常时跟着 HAL tick 走） |
| `BMI088_DATA` | 驱动里的原始数据（C 板坐标系）：`acc_g`、`gyro_dps`（已减零偏）、`gyro_offset`。标定 C 板轴向时看它 |

流程：上电 → RELAX 下手转标定中值和方向 → `gimbal_test.mode` 改成 1 → 再改 `yaw_deg` / `pitch_deg`。改 mode 时目标会自动同步成当前角，所以要先切模式、再改目标。

## 版本规划

- **v1 编码器定角度**：代码已完成，待上板标定和调参。
- **v2 IMU 定角度**：接口已定（见下一节），分三步：ins（现在就能写，桌上测）→ 云台 IMU 模式（v1 上板调通、C 板装到云台后）→ 加热恒温。
- **v3 遥控器**：DR16 接 USART3（PC11），手册写 100 kbps；其余串口参数本目录没有 DR16 手册，用前核对。
- **合并准备**：定单板 / 双板；按根目录 notes.md 的 CAN 表分配负载。

## v2 接口（2026-10-29 定，用户自己写第一版）

分三步，每步单独验收：

1. **ins 模块**：BMI088 + 姿态解算。C 板单独放桌上就能测，不依赖云台，现在就能写。
2. **云台 IMU 模式**：等 v1 上板调通、C 板装到云台后再接。两个模式用同一套串级 PID，PID 没调好时两边的问题会叠在一起。
3. **加热恒温**：以后（见本节最后）。

按 **C 板装在 pitch 上、随枪管俯仰** 设计。如果装在 yaw 上，IMU 测的是 yaw 平台的姿态，pitch 的 IMU 角要改成 IMU 角 + 编码器角，到时重定。

数据流：BMI088（DMA）→ ins（乘安装矩阵 → `Attitude_Update` → 换成全车约定）→ robot.c 逐字段拷 → `Gimbal_SetImu` → gimbal。gimbal 不 include ins.h（根目录 notes 第 2 节规则 1）。

`Robot_Loop()` 每 1 ms（第 2 步之后）：`INS_Update` → `INS_GetFeedback` → 拷成 `GimbalImu_t` → `Gimbal_SetImu` → `Gimbal_GetFeedback` → `GimbalTestCmd_Update` → `Gimbal_SetCmd` → `Gimbal_Update` → 每路 CAN `SendAll`。

### 第 1 步：ins（App/ins/ins.h、ins.c、ins_config.h）

```c
/* ins.h */
typedef struct
{
    uint8_t valid;              /* 1：初始化成功、标定完、姿态已收敛、最近 10 ms 内有新样本 */
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
- 下一步：烧录做上面的桌面验收，把 C 板轴向记进标定记录。

### 第 2 步：云台 IMU 模式

```c
/* gimbal.h 的改动 */
typedef enum
{
    GIMBAL_MODE_RELAX = 0,
    GIMBAL_MODE_ENCODER,        /* 目标角相对底盘 */
    GIMBAL_MODE_IMU             /* 目标角相对地面 */
} GimbalMode_e;

/* GimbalCmd_t 不变：目标角的坐标系跟着 mode 走 */

/* 新增：robot.c 从 InsFeedback_t 逐字段拷过来 */
typedef struct
{
    uint8_t valid;              /* 0 时不许进 IMU 模式；IMU 模式中变 0 → 无力并锁住 */
    float yaw_deg;
    float pitch_deg;
    float yaw_rate_dps;
    float pitch_rate_dps;
} GimbalImu_t;

typedef struct
{
    GimbalMode_e mode;
    float yaw_rel_deg;          /* 原来的 yaw_deg 改名（和根目录 notes 第 3 节一致）：任何模式下都是编码器角 */
    float pitch_rel_deg;
    float yaw_imu_deg;          /* 新增：IMU 角，切到 IMU 模式时指令源用它同步目标 */
    float pitch_imu_deg;
} GimbalFeedback_t;

/* 新增：每周期在 Gimbal_GetFeedback / Gimbal_Update 之前调 */
void Gimbal_SetImu(const GimbalImu_t *imu);
```

gimbal.c：

- 存一份 `GimbalImu_t`，IMU 模式下角度环、速度环的反馈直接读它；`GimbalAxis_t` 不加字段。
- `Gimbal_SetCmd` 接受 ENCODER 和 IMU，其他值当 RELAX。
- 故障锁定多一个条件：要求 IMU 模式而 `valid == 0`，同样置 `gimbal_fault`（上电约 4 s 内切 IMU 模式会锁住）。
- 切模式时目标同步成新模式坐标系下的当前角。
- 限位仍按编码器角。设 `offset = 反馈角 − 编码器角`（编码器模式下为 0，v1 现在的逻辑就是它的特例）：目标先减 offset 换到编码器系里，在编码器系里限位、算斜坡步长，再把步长加到目标上。不要在世界系里直接比大小：底盘朝向 170° 时，±90° 的限位在世界系里是跨过 ±180° 的一段。
- 有限位的轴，底盘转得比斜坡（100°/s）快时，目标会暂时落在限位外。yaw 有滑环（限位写 ±180）就没这个问题；测 pitch 时别猛掰底盘。
- 重力前馈继续用编码器 pitch；PID 两个模式先共用一套。

gimbal_test：`mode` 多一个 2（IMU）；切模式时按新模式用 `yaw_imu_deg` / `pitch_imu_deg` 或 `yaw_rel_deg` / `pitch_rel_deg` 同步目标。

验收：

1. RELAX 下手转云台：IMU 角、IMU 角速度的方向和编码器角一致（不一致改安装矩阵，不改电机方向）；水平时 `pitch_imu_deg` 和 `pitch_rel_deg` 应接近。
2. 编码器模式和 v1 一样（回归）。
3. IMU 模式：手慢慢转底盘，云台相对地面的朝向不变；超过 yaw 限位后被限位带着走。
4. 几个模式来回切，云台不跳。
5. 让 ins 失效（比如在调试器里把它保存的初始化返回值改成非 0），10 ms 内 `valid` 变 0，云台无力、`gimbal_fault` = 1。

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
| C 板轴向（静止时读 +1 g 的轴朝哪、各轴右手定则正转方向） | 待测（桌面验收第 3 条） |
| 安装矩阵 `INS_INSTALL_MATRIX` | 待定（C 板装到云台后定；代码占位为单位矩阵） |
| 静止 1 分钟 yaw 漂移（未恒温） | 待测 |
