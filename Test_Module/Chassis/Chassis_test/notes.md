# Chassis_test 笔记（底盘测试工程）

> 本工程的进度与注意事项。跨模块约定（分层、接口、单位、正方向、CAN 分配、C 板引脚）见 `RM_Infantry/notes.md`；云台那边踩过的坑见 `Test_Module/Gimbal/Gimbal_test/notes.md` 的「已知问题」，大部分底盘也适用。
> 最后更新：2026-11-01（建工程；按步兵逻辑定接口：底盘接收云台主控的指令，四种模式，板间 CAN 帧）

## 当前状态

- 2026-11-01 建好工程，还没有底盘模块：App 里只有空的 robot.c（两路 CAN 的 manager、1 ms 调度、每周期发送、CAN 接收分发），没注册电机，SendAll 什么也不发。清理编译无警告（FLASH 11676 B、RAM 1752 B），没上板。
- 2026-11-01 用户定：按步兵的逻辑做，底盘接收云台主控发来的速度和模式。接口按这个定，见「接口」（同一天早些时候那版只有底盘坐标系速度的草案作废）。遥控接收、测试用的指令源用户写过很多遍，这里不展开。
- 用户手上还没有电机。写完先做主机模型测试（和云台一样），电机到了再上板。
- 轮型未定（2026-11-01 问过，用户没选）。接口和轮型无关，麦轮、全向轮的逆运动学都写在「接口」里，定了再写对应那组。
- 参考：用户自己的果园机器人底盘（全向轮，已上板调通），见「参考：果园机器人底盘」。
- 下一步：用户按「接口」第 1 步自己写第一版，写完 Claude 审查 + 主机模型测试。

## 工程是怎么建的（以后改外设照这个做）

- `Chassis_test.ioc` 从 `Gimbal_test.ioc` 改来，和它的区别只有：
  - 去掉 SPI1、DMA 和 BMI088 的 4 个 GPIO（底盘不用 IMU）；
  - 去掉 CAN1_RX1 中断（过滤器都进 FIFO0，见 `Shared/BSP/can_bus.h`，这个中断用不上）；
  - 两路 CAN 打开 Automatic Bus-Off Management：Gimbal_test 已知问题第 11 条——只接调试器 3.3 V 时 CAN 会 bus-off，没开这项就得复位才能恢复；开了以后上 24 V 应能自己恢复（还没上板验证；果园机器人工程也开着）；
  - Project Manager → Code Generator 改成只拷用到的库文件：Drivers 6.4 MB / 76 个文件（云台工程拷了全部，57 MB）。
  - 其余（HSE 12 MHz → 168 MHz、CAN 1 Mbps：预分频 3 / BS1 9 / BS2 4、NVIC 优先级分组 4、CAN 接收中断优先级 0）和云台相同。
- 代码由 STM32CubeMX 6.15.0 生成。这次用的是命令行无界面模式：脚本里写 `config load <ioc>`、`project generate`、`exit`，在 CubeMX 安装目录下运行 `jre\bin\java -jar STM32CubeMX.exe -q <脚本>`（UM1718 第 3.3.2 节）。平时用 GUI 打开 `.ioc` 改完点 GENERATE CODE 即可。
- 用户代码只在 USER CODE 段里：main.c 的 `#include "robot.h"`、`Robot_Init()`、`Robot_Loop()`，重新生成不会丢。其他全在 App/。
- 顶层 CMakeLists.txt（CubeMX 只生成一次）：加了 `SHARED_DIR`、`App/robot.c` 和底盘要用的 Shared 代码（dji_motor、dji_motor_manager、dji_m3508、motor_control、PID、user_math、can_bus）。GM6020、M2006、BMI088、IST8310、Attitude 没加。
- 编译：`cmake --preset Debug`、`cmake --build --preset Debug`，输出 `build/Debug/Chassis_test.elf`。CLion 打开 `Test_Module/Chassis/Chassis_test`，照云台工程建「OpenOCD 下载并运行」配置，board config 用 `D:\OpenOCD_Config\DAPLink\stm32f4.cfg`。Reset 选 Halt 最稳；这个 cfg 已经把 reset-init 覆盖成空（果园机器人 notes 第 3 节），选 Init 也不会再卡在 SystemClock_Config。

## 硬件要点（C620 / M3508 / C 板手册，2026-11-01 读过）

| 项 | 内容 |
|---|---|
| C620 控制帧 | 1 Mbps；0x200 控制 ID 1~4，0x1FF 控制 ID 5~8；每个电调 2 字节，高字节在前；−16384~16384 对应转矩电流 −20~20 A |
| C620 反馈 | 帧 ID 0x200 + 电调 ID，默认 1 kHz；DATA[0..1] 转子机械角度 0~8191，DATA[2..3] **转子**转速 rpm（不是减速后的输出轴），DATA[4..5] 实际转矩电流（单位手册没写），DATA[6] 电机温度 ℃ |
| 设电调 ID | 独立设置：正常工作时短按 SET 一次（指示灯灭），再短按 N 次就是 ID N（1~8），3 秒不按自动保存，重新上电生效。快速设置：短按一次后再长按，总线上所有电调橙灯常亮，按想要的顺序依次用手把各电机转子转半圈以上，从 1 开始编号，重新上电生效 |
| 指示灯 | 绿灯每秒闪 N 次 = ID 为 N（正常）；橙灯每秒闪 2 次 = 总线上 ID 重复（关输出）；红灯常亮 = 7-pin 线读不到电机或电机不匹配；红灯闪 2 次 = 三相线没接；闪 3 次 = 位置传感器数据丢（查 7-pin 线）。异常时电调关输出 |
| 终端电阻 | C620 上有 120 Ω 终端电阻拨码开关。C 板两路 CAN 的收发器后面都有 120 Ω，应是板载终端电阻（根目录 notes 第 5 节；C 板上没有能拨的开关），所以只把总线另一端（离 C 板最远的电调）打开，其余关 |
| 其他 | 不能同时插 CAN 线和 PWM 线（手册：电机控制失常），换信号方式要断电。换电机或电调后建议校准一次：长按 SET 到绿灯快闪松开，电机会自己转，空载做、别碰。电机 ≥125 ℃ 橙灯每秒闪 1 次（温度保护默认开，开着时同时关输出，可在 RoboMaster Assistant 里关），≥180 ℃ 红灯闪 4 次 |
| M3508 | 减速比 3591/187 ≈ 19.2（驱动里的 `DJI_M3508_REDUCTION_RATIO`）；24 V 空载输出轴 482 rpm（转子约 9256 rpm），额定 469 rpm、3 N·m（最大持续转矩），堵转 4.5 N·m；转矩常数 0.3 N·m/A；绕组最高 125 ℃ |
| C 板 CAN 口 | CAN1 是 2-pin（1 CANL、2 CANH），和 C620 自带的 2-pin CAN 线（黑 CAN_L、红 CAN_H）线序一样；CAN2 是 4-pin（1 5V、2 GND、3 CANH、4 CANL）。每路两个插座 |
| 手册没写的 | CAN 模式下正电流朝哪转（PWM 模式默认正方向是输出轴逆时针；果园机器人的实测推出来 CAN 模式也是这样，见下一节）；控制帧停发后电调是保持还是停 |

## 参考：果园机器人底盘（用户自己写、上板调通过）

代码在 `C:\Project\Fruit-picking robot\Fruit_Picking\UserModules\APP\Chassis`（Chassis_Control.c、Omni_Remote.c、Mecanum_Remote.c），记录在那个工程 notes.md 的第 4 节。它用的 PID.c、M3508 驱动和 Shared 里的是同一份，所以下面的参数单位和这边相同。

- 硬件：全向轮，4 个 M3508 + C620 接 CAN1，ID 1~4 = 右前、右后、左后、左前（俯视顺时针），轮子在四角、轮轴指向车中心。4 个方向都是 1（2026-10-21 实测，那边的正方向定义是“推车体顺时针转”）。
- 速度环：用电机层（motor_control）的速度模式，Kp 5（ID2 是 8）、Ki 20、Kd 0，单位是“转子 rpm → 电流、dt 1 ms”。这边可以从这组起调（车重、轮子不同，要重调）。
- 上坡的教训（2026-10-25）：电流限幅 3000（≈ 3.7 A，约 1.1 N·m）上不去坡，放到 8000（≈ 9.8 A）；速度环积分项被 motor_control 里写死的 1000（≈ 1.2 A）限住，给它加了 `MotorControl_SetSpeedIntegralLimit` 放到 6000——RM 的 Shared 里没有这个函数。电流放大后不加指令斜坡，起步 / 刹车电流约翻倍，所以又加了 vx / vy / wz 同比例的斜坡（`Chassis_Control_Ramp`）。
- 轮速超过 `RPM_MAX`（8000 转子 rpm）时 4 个轮等比例缩小，合成方向不变（在 Omni_Remote.c / Mecanum_Remote.c 里）。
- 坐标系和这边不同：那边 vx 向右、vy 向前、wz 顺时针为正，指令是 −1~1 的归一化值；RM 约定 x 前、y 左、wz 俯视逆时针为正，单位 m/s、度/秒。借公式时换算：vx_RM = vy_果园，vy_RM = −vx_果园，wz_RM = −wz_果园。换完后它的麦轮公式（“从上往下看辊子连成 X 形”的装法）和下面的麦轮公式一致；全向轮公式也一致，只是每个轮子正方向的定义不同（2026-11-01 用 10 万组随机输入对过）。
- 由此推出的电机方向：那边电机都是轴朝外装、方向全是 1、正 = 推车体顺时针，说明 C620 正电流时输出轴从轴端看逆时针转（和 C620 手册 PWM 模式的默认正方向一致）。放到这边“正 = 往前滚”的约定下：左侧两个轮正转往前、右侧两个往后，也就是左侧方向 1、右侧 −1。这是推出来的，上板照「上板步骤」第 1 步第 3 条核对。
- 要借这个底盘测第 1、2 步：底盘在 CAN1，robot.c 里改传 `&robot_can1`；ID 不用重设，config 里填左前 4、左后 3、右后 2、右前 1；用全向轮公式，R 和轮子半径按那台车量。

## 版本规划（按步兵逻辑，分 3 步，每步单独验收）

1. **第 1 步：不跟随模式**——坐标变换、逆运动学（含超转速等比例缩小）、4 个速度环、RELAX 和掉线锁定。底盘单独就能测，云台角在调试器里手填。
2. **第 2 步：跟随云台、小陀螺 + 加速度斜坡**。符号底盘单独能测（云台角填固定值）；真正的跟随效果要云台在 IMU 模式、第 3 步通了以后看。
3. **第 3 步：板间通信 chassis_link**。两块板都在时做。单板方案不要这一步，robot_cmd 直接填 `ChassisCmd_t`。
- **以后**：功率控制（裁判系统给功率上限和缓冲能量，在速度环和输出之间按比例缩小 4 个电流）、超级电容、小陀螺时平移方向的延迟补偿、变速小陀螺、底盘→云台的回传帧（裁判系统接在底盘板时，把热量 / 弹速上限传给云台板）。

## 接口（2026-11-01 定，按步兵逻辑；用户自己写第一版）

数据流：云台板（键鼠 / 遥控 → robot_cmd；云台角来自 `Gimbal_GetFeedback` 的 `yaw_rel_deg`）→ 板间 CAN 0x110 → 底盘板 `ChassisLink_GetCmd` → robot.c → `Chassis_SetCmd` → `Chassis_Update` → 4 个 M3508。

```c
/* chassis.h */
#include "dji_motor_manager.h"

typedef enum
{
    CHASSIS_MODE_RELAX = 0,     /* 无力：电流 0。上电默认；板间通信超时、电机掉线时也是这个 */
    CHASSIS_MODE_NO_FOLLOW,     /* 不跟随：底盘不转，只按云台朝向平移 */
    CHASSIS_MODE_FOLLOW,        /* 跟随云台：底盘转到和云台同向，平移按云台朝向 */
    CHASSIS_MODE_SPIN           /* 小陀螺：底盘按固定角速度一直转，平移仍按云台朝向 */
} ChassisMode_e;                /* 数值也是板间帧的内容，两块板要一致，别随便调顺序 */

typedef struct
{
    ChassisMode_e mode;
    float vx_mps;               /* 平移速度，在云台朝向的坐标系里（只看 yaw）：沿枪口为正，m/s */
    float vy_mps;               /* 枪口左边为正，m/s */
    float gimbal_yaw_rel_deg;   /* 云台相对底盘正前方的角度（云台的 yaw_rel_deg），俯视逆时针为正，-180~180 */
} ChassisCmd_t;

/* 把 4 个 M3508 注册到这路 CAN 的 manager，必须在 DJI_MotorManager_Start() 之前调用。 */
HAL_StatusTypeDef Chassis_Init(DJI_MotorManager_t *bus);
void Chassis_SetCmd(const ChassisCmd_t *cmd);
/* 每 1 ms 调一次：定模式 → 算 wz → 斜坡 → 坐标变换 → 逆运动学 → 4 个速度环 → 设置电流。发 CAN 帧由 robot.c 统一做。 */
void Chassis_Update(uint32_t now_tick);
```

- 指令里没有 wz：底盘自己的角速度在每种模式下都由底盘算，云台板只管“往哪走”和“云台朝哪”。跟随环、小陀螺转速、斜坡、以后的功率控制都在底盘板一处调；指令也正好装进一帧 8 字节。
- 没有 `Chassis_GetFeedback`：云台板现在不需要底盘的数据，以后要回传时再加。

各模式：

| 模式 | 底盘角速度 wz | 平移 |
|---|---|---|
| RELAX | — | 4 个电流 0，PID 不算 |
| NO_FOLLOW | 0 | (vx, vy) 按云台角转到底盘坐标系 |
| FOLLOW | 跟随环：wz = Kp × gimbal_yaw_rel（云台在左 → 底盘往左转），限 ±`CHASSIS_FOLLOW_MAX_DPS` | 同上 |
| SPIN | `CHASSIS_SPIN_DPS`（正负决定方向） | 同上 |

`Chassis_Update` 的计算顺序：

1. 定模式：指令 RELAX → 清 fault；指令是运动模式但有一个轮子掉线（`DJI_Motor_IsOnline`，100 ms）→ fault = 1；fault 时强制 RELAX，要给一次 RELAX 才解除。从 RELAX 进运动模式时清 4 个速度环和跟随环，斜坡从 0 开始；运动模式之间切换不清速度环，斜坡接着走；切进 FOLLOW 时清跟随环。
2. 按模式算目标 wz（上表）。跟随环用 PID.c：目标 0、反馈 −gimbal_yaw_rel（底盘相对云台的角度），输出就是 wz，输出限幅就是 `CHASSIS_FOLLOW_MAX_DPS`。
3. 斜坡（第 2 步加）：(vx, vy) 在**云台坐标系**里往指令走一步，按向量长度每毫秒最多变 `CHASSIS_ACCEL_MPS2` × 0.001，方向不变；wz 每毫秒最多变 `CHASSIS_WZ_ACCEL_DPS2` × 0.001。
4. 坐标变换（θ = gimbal_yaw_rel，换成 rad）：vx_底 = vx·cosθ − vy·sinθ，vy_底 = vx·sinθ + vy·cosθ。
5. 逆运动学（公式见下）→ 4 个目标转子转速；任一个超过 `CHASSIS_WHEEL_MAX_RPM` 时 4 个一起等比例缩小（合成方向不变）。
6. 4 个速度环 → 电流 → `Math_Clamp` 到 ±电流限幅 → `DJI_Motor_SetCurrent` → `MotorControl_Update`（电机层的掉线保护，同云台的 `Gimbal_AxisOutput`）。以后功率控制插在速度环和 SetCurrent 之间。

chassis.c 内部（建议）：

- 每个轮子一个 `ChassisWheel_t`，只有两个字段：`DJI_Motor_t motor`（方向在 `motor.motor_direction`，转速反馈看 `motor.speed_rpm`，驱动里已经乘过方向）和 `PID_t speed_pid`（目标转速看 `speed_pid.target`，输出电流看 `speed_pid.out`）。4 个放数组 `chassis_wheel[4]`，下标 0 左前、1 左后、2 右后、3 右前（俯视逆时针，正好是 x 前 y 左坐标系里的第 1~4 象限）。
- `PID_t chassis_follow_pid`：跟随环。
- 斜坡后的速度 `chassis_vx_mps`、`chassis_vy_mps`（云台坐标系）、`chassis_wz_dps`：3 个 float，不用结构体。
- 另外和云台一样：`chassis_cmd`（SetCmd 拷进来的）、`chassis_mode`（实际模式）、`chassis_fault`（掉线锁定）。
- `Chassis_Init`：4 次 `DJI_M3508_Register`（ID 1~4 用 0x200、槽位 ID−1）；电机层 `MotorControl_Init`（电流限幅、方向，PID 参数全给 0）+ `MotorControl_SetMode(MOTOR_MODE_CURRENT)`；`PID_Init` 4 个速度环（输出和积分项都限在 ±电流限幅）和跟随环。
- `Chassis_SetCmd`：三种运动模式原样收下，其他值当 RELAX。

逆运动学：v 是轮子线速度，正 = 这个轮子往让底盘前进的方向滚；wz 先换成 rad/s（× π/180）。

```
麦轮（标准装法：从上往下看辊子连成 X 形；向左平移时左前、右后轮往后转，左后、右前轮往前转）
v_左前 = vx − vy − (a + b)·wz        a = 左右轮中心距 / 2，b = 前后轮中心距 / 2
v_左后 = vx + vy − (a + b)·wz
v_右后 = vx − vy + (a + b)·wz
v_右前 = vx + vy + (a + b)·wz

全向轮（4 个装在四角、轮轴指向车中心）
v_左前 = (vx − vy)·√2/2 − R·wz        R = 轮心到底盘中心的距离
v_左后 = (vx + vy)·√2/2 − R·wz
v_右后 = (vx − vy)·√2/2 + R·wz
v_右前 = (vx + vy)·√2/2 + R·wz
```

- 这里的 vx、vy 是坐标变换后底盘坐标系里的。目标转子转速 rpm = v / 轮子半径 × 60 / (2π) × 减速比（`motor.reduction_ratio`）。
- 两种轮子符号规律一样，左右两侧的电机方向也一样（见「参考」最后两条）。

chassis_config.h（建议）：

| 宏 | 内容 |
|---|---|
| 4 个电调 ID、4 个方向 | 新车建议 ID 1~4 按下标顺序；方向预计左侧 1、右侧 −1，待核对 |
| 轮子半径、a、b（全向轮是 R） | 待测 |
| 电流限幅 | C620 ±16384 对应 ±20 A；先给小（比如 3000），果园机器人最后用 8000 |
| 速度环 Kp / Ki / Kd | 误差是转子 rpm、输出是电流、dt 1 ms；从 5 / 20 / 0 起调 |
| `CHASSIS_WHEEL_MAX_RPM` | 转子转速上限，果园机器人用 8000（空载约 9256） |
| 跟随环 Kp / Kd、`CHASSIS_FOLLOW_MAX_DPS` | Kp 单位是“度/秒 每 度”；先只用 P |
| `CHASSIS_SPIN_DPS` | 小陀螺转速，先给小（比如 180） |
| `CHASSIS_ACCEL_MPS2`、`CHASSIS_WZ_ACCEL_DPS2` | 平移、旋转的加速度限制，待调 |

板间通信（第 3 步，`App/chassis/chassis_link.c/.h`，和 chassis.h 放在一起，帧格式只写在这一处；云台板的工程也编这个文件，只用发送）：

| 字节 | 内容 |
|---|---|
| 0 | mode（`ChassisMode_e`） |
| 1 | 保留，发 0 |
| 2~3 | vx，int16，mm/s |
| 4~5 | vy，int16，mm/s |
| 6~7 | gimbal_yaw_rel，int16，0.01°（±180° 是 ±18000） |

标准帧 0x110，DLC 8，云台板每 1 ms 发一次，高字节在前（和大疆电机帧一样）。走两块板之间那路 CAN：底盘板默认 CAN1（电机在 CAN2）。

```c
/* chassis_link.h */
#include "chassis.h"
#include "can_bus.h"

#define CHASSIS_LINK_CMD_ID         0x110U
#define CHASSIS_LINK_TIMEOUT_MS     50U

/* 云台板：每 1 ms 调一次，打包后发出去。 */
HAL_StatusTypeDef ChassisLink_SendCmd(CAN_HandleTypeDef *hcan, const ChassisCmd_t *cmd);
/* 底盘板：给 0x110 配一个过滤器组（CAN1 用组 2、CAN2 用组 16，can_bus.h 的分配说明里补一行）。 */
HAL_StatusTypeDef ChassisLink_Init(CAN_HandleTypeDef *hcan);
/* 底盘板：robot.c 的 CAN 接收回调里，把这路上读到的每一帧都交给它，它只认 0x110。 */
void ChassisLink_ProcessRxFrame(const CAN_RxHeaderTypeDef *header, const uint8_t data[8]);
/* 底盘板：每 1 ms 取最新指令；超过 CHASSIS_LINK_TIMEOUT_MS 没收到就给 RELAX。 */
void ChassisLink_GetCmd(uint32_t now_tick, ChassisCmd_t *cmd);
```

robot.c（底盘板）：`Robot_Init` 里在 `DJI_MotorManager_Start` 之前 `Chassis_Init(&robot_can2)`，第 3 步再加 `ChassisLink_Init(&hcan1)`，失败都进 `Error_Handler()`；`Robot_Loop` 里在发送之前 `ChassisLink_GetCmd(now, &cmd)` → `Chassis_SetCmd(&cmd)` → `Chassis_Update(now)`；CAN 接收回调里 CAN1 的帧也交给 `ChassisLink_ProcessRxFrame`。第 1、2 步还没有云台板时，指令先用调试器改 robot.c 里一个 `volatile ChassisCmd_t`。CMake 加 `App/chassis/chassis.c`（第 3 步再加 `chassis_link.c`）和 include 目录 `App/chassis`。.ioc 不用改。

为什么这样写：

- 模式和跟随环放底盘板，云台板只发意图和云台角：底盘怎么转（跟随环、小陀螺、斜坡、功率）都在一处调；单板方案也是同一个 `ChassisCmd_t`，只是不经过 chassis_link。
- 平移用云台坐标系：操作手按 W 永远朝枪口走；小陀螺时底盘在下面转，平移方向在地面上不跟着转（2026-11-01 平面模型：360°/s 小陀螺、云台对地不动时，地面上的平移方向误差不到 1e-13°）。
- 斜坡加在坐标变换**之前**：小陀螺时底盘坐标系里的速度方向一直在转，2 m/s、360°/s 时分量的变化率约 12.6 m/s²。同一个模型里把 3 m/s² 的斜坡放在变换之后，2 m/s 的指令只剩 0.38~0.55 m/s，方向偏 66~83°。
- 速度环在 chassis.c 里算、电机层只用电流模式（和云台一样）：motor_control 自带速度模式的积分限幅写死 1000（约 1.2 A），果园机器人上坡时就是被它限住、改了驱动才放到 6000；功率控制要在速度环和输出之间按比例缩放 4 个电流，PID 在本模块里算才好插这一步。
- 一个轮子掉线 4 个都停：少一个轮子出力，合力方向就不对，底盘会乱跑。

容易错的地方：

1. 跟随要云台在 IMU 模式：编码器模式下云台跟着底盘转，yaw_rel 不会变小，底盘会一直转下去。云台板的 robot_cmd 别让这两个组合同时出现。
2. gimbal_yaw_rel 在 ±180° 处跳变：跟随环的 P 没问题（误差本来就在 ±180° 内）；PID.c 的微分是对反馈直接求差，过 ±180° 那一拍会冲一下，所以先只用 P。
3. 角度要换 rad：坐标变换的 cos / sin、wz 进逆运动学都要 × π/180。
4. 小陀螺 + 平移很容易超转速：等比例缩小会把转速和平移一起降。想保转速、少给平移是另一种做法，以后再说。
5. 板间：`ProcessRxFrame` 在中断里写、`GetCmd` 在主循环读，拷贝时关中断（参考 `DJI_Motor_GetFeedback`），不然会读到半新半旧的一帧；两块板字节序要一致。
6. `speed_rpm` 是转子转速，轮子转速要再除 19.2。
7. 方向：标定按「手转轮子、让它往前滚时 `speed_rpm` 为正」来定；驱动里反馈和输出电流都乘了方向，反馈方向标对了，输出方向也就对了（前提是电调正电流对应正转速，果园机器人上是这样）。首次闭环用小电流限幅；如果一闭环就转到最快，就是正反馈，先停下查。
8. 借果园机器人的公式要换坐标系（见「参考」），直接抄符号会全反。
9. 麦轮平移方向不对、或平移时底盘自己转：先查电调 ID 和位置是否对应、轮子的辊子朝向（装法），这不是改符号能解决的。
10. 调试器暂停时控制帧停发，C620 停发后怎么处理手册没写：架空测试问题不大，落地前先确认。

## 上板步骤与验收（电机到了再做）

第 1 步（底盘单独，调试器改指令）：

1. 接线：4 个 C620 接 CAN2（接 CAN1 就改 robot.c 里传的 manager），设好 ID（绿灯闪 1~4 次），离 C 板最远的电调打开终端电阻。要上 24 V，只接调试器时 CAN 用不了。
2. 架空底盘（轮子离地），上电默认 RELAX。手转轮子：4 个 `motor.feedback_count` 在涨，`speed_rpm` 跟着变。
3. 方向：按「容易错的地方」第 7 条逐个标定。
4. 首次闭环：电流限幅保持小值，mode 改成 NO_FOLLOW，`gimbal_yaw_rel_deg` 填 0，`vx_mps` 给 0.2，4 个轮子都应往前转、转速接近目标。
5. 速度环：看 `speed_pid.target` 和 `motor.speed_rpm` 的跟随，先调 P 再加 I。
6. 方向核对（两种轮子一样）：`vy_mps` 给 0.2 → 左前、右后往后转，左后、右前往前转。
7. 坐标变换：`gimbal_yaw_rel_deg` 填 90，`vx_mps` 给 0.2 → 轮子转向和上一条一样（云台朝左时沿枪口走 = 底盘往左平移）。
8. 落地低速：看底盘实际走向符合约定（x 前、y 左）。
9. 保护：拔掉一个电调的 CAN 线，4 个轮子都无力，`chassis_fault` 变 1。

第 2 步：

1. FOLLOW 的符号：`gimbal_yaw_rel_deg` 填 +10（固定），底盘应逆时针转、wz = 跟随 Kp × 10；填 −10 → 顺时针。底盘单独时这个角不会变，所以会一直转，只用来看符号。
2. SPIN：按 `CHASSIS_SPIN_DPS` 转；切回 NO_FOLLOW 时按斜坡慢慢停下。
3. 几个模式来回切，底盘不冲、不打滑。
4. 真正的跟随（第 3 步通了、云台在 IMU 模式）：手把底盘推转一个角度，底盘应转回和云台同向、不振荡。

第 3 步：

1. 云台板发 0x110，底盘板收到（看 chassis_link 里最后一帧的 tick 跟着走）。
2. 拔掉板间 CAN 线，50 ms 内底盘无力；插回后按当前指令恢复。
3. 全车：键鼠切四种模式，平移方向始终跟枪口走；小陀螺时边转边平移，走向不歪。

验收：四个轮子跟得上目标转速、不振荡；四种模式行为符合上表；掉线、断通信自动无力。电流限幅调好再放开。

## 标定记录（测完填）

| 项 | 值 |
|---|---|
| 轮型 | 待定 |
| 底盘电机所在总线 | 代码默认 CAN2，待按实际接线确认 |
| 板间通信总线 | 底盘板默认 CAN1，待按实际接线确认 |
| 电调 ID ↔ 位置 | 建议 1 左前、2 左后、3 右后、4 右前 |
| 各轮 motor_direction | 待测（按果园机器人的实测推出来是左 1、右 −1） |
| 轮子半径、a / b（全向轮 R） | 待测 |
| 速度环 PID | 待调（起点 5 / 20 / 0） |
| 跟随环 Kp、最大 wz | 待调 |
| 小陀螺转速、两个加速度限制 | 待调 |
