# Chassis_test 笔记（底盘测试工程）

> 本工程的进度与注意事项。跨模块约定（分层、接口、单位、正方向、CAN 分配、C 板引脚）见 `RM_Infantry/notes.md`；云台那边踩过的坑见 `Test_Module/Gimbal/Gimbal_test/notes.md` 的「已知问题」，大部分底盘也适用。
> 最后更新：2026-11-01（建工程：.ioc → CubeMX 生成 → CMake 接 Shared → 空装配层，清理编译无警告；写 v1 接口草案；对照用户的果园机器人底盘整理参考）

## 当前状态

- 2026-11-01 建好工程，还没有底盘模块：App 里只有空的 robot.c（两路 CAN 的 manager、1 ms 调度、每周期发送、CAN 接收分发），没注册电机，SendAll 什么也不发。清理编译无警告（FLASH 11676 B、RAM 1752 B），没上板。
- 用户手上还没有电机。v1 写完先做主机模型测试（和云台一样），电机到了再上板。
- 轮型未定（2026-11-01 问过，用户没选）。v1 接口和轮型无关，麦轮、全向轮的逆运动学都写在「v1 接口」里，定了再写对应那组。
- 参考：用户自己的果园机器人底盘（全向轮，已上板调通），见「参考：果园机器人底盘」。
- 下一步：用户按「v1 接口」自己写第一版（chassis + chassis_test_cmd，robot.c 加三处调用，CMake 加文件），写完 Claude 审查 + 主机模型测试。

## 工程是怎么建的（以后改外设照这个做）

- `Chassis_test.ioc` 从 `Gimbal_test.ioc` 改来，和它的区别只有：
  - 去掉 SPI1、DMA 和 BMI088 的 4 个 GPIO（底盘 v1 不用 IMU）；
  - 去掉 CAN1_RX1 中断（过滤器都进 FIFO0，见 `Shared/BSP/can_bus.h`，这个中断用不上）；
  - 两路 CAN 打开 Automatic Bus-Off Management：Gimbal_test 已知问题第 11 条——只接调试器 3.3 V 时 CAN 会 bus-off，没开这项就得复位才能恢复；开了以后上 24 V 应能自己恢复（还没上板验证；果园机器人工程也开着）；
  - Project Manager → Code Generator 改成只拷用到的库文件：Drivers 6.4 MB / 76 个文件（云台工程拷了全部，57 MB）。
  - 其余（HSE 12 MHz → 168 MHz、CAN 1 Mbps：预分频 3 / BS1 9 / BS2 4、NVIC 优先级分组 4、CAN 接收中断优先级 0）和云台相同。
- 代码由 STM32CubeMX 6.15.0 生成。这次用的是命令行无界面模式：脚本里写 `config load <ioc>`、`project generate`、`exit`，在 CubeMX 安装目录下运行 `jre\bin\java -jar STM32CubeMX.exe -q <脚本>`（UM1718 第 3.3.2 节）。平时用 GUI 打开 `.ioc` 改完点 GENERATE CODE 即可。
- 用户代码只在 USER CODE 段里：main.c 的 `#include "robot.h"`、`Robot_Init()`、`Robot_Loop()`，重新生成不会丢。其他全在 App/。
- 顶层 CMakeLists.txt（CubeMX 只生成一次）：加了 `SHARED_DIR`、`App/robot.c` 和底盘 v1 要用的 Shared 代码（dji_motor、dji_motor_manager、dji_m3508、motor_control、PID、user_math、can_bus）。GM6020、M2006、BMI088、IST8310、Attitude 没加。
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
- 坐标系和这边不同：那边 vx 向右、vy 向前、wz 顺时针为正，指令是 −1~1 的归一化值；RM 约定 x 前、y 左、wz 俯视逆时针为正，单位 m/s、度/秒。借公式时换算：vx_RM = vy_果园，vy_RM = −vx_果园，wz_RM = −wz_果园。换完后它的麦轮公式（“从上往下看辊子连成 X 形”的装法）和下面 v1 的麦轮公式一致；全向轮公式也一致，只是每个轮子正方向的定义不同。
- 由此推出的电机方向：那边电机都是轴朝外装、方向全是 1、正 = 推车体顺时针，说明 C620 正电流时输出轴从轴端看逆时针转（和 C620 手册 PWM 模式的默认正方向一致）。放到这边“正 = 往前滚”的约定下：左侧两个轮正转往前、右侧两个往后，也就是左侧方向 1、右侧 −1。这是推出来的，上板照「v1 上板步骤」第 3 步核对。
- 要借这个底盘测 v1：底盘在 CAN1，robot.c 里改传 `&robot_can1`；ID 不用重设，config 里填左前 4、左后 3、右后 2、右前 1；用全向轮公式，R 和轮子半径按那台车量。

## 版本规划

- **v1 速度闭环 + 逆运动学**：调试器给底盘坐标系下的 vx、vy、wz → 四个轮子的目标转速 → 速度环 → 电流。接口见下一节，用户自己写第一版。
- **v2 跟随云台 / 小陀螺**：指令里的平移速度改成在云台坐标系里，加跟随 PID 和坐标旋转，要用云台的 `yaw_rel_deg`。App 模块之间不 include，这个角由 robot.c（合并后 robot_cmd）放进底盘指令。接口到时再定，v1 的结构体只需要加字段、加模式。
- **v3 遥控器**：DR16（和云台 v3 一起）。加速度斜坡、轮速超限时 4 个轮子等比例缩小放这时候（果园机器人都有现成做法，见上一节）：手推摇杆才需要平滑，v1 用调试器给阶跃正好看速度环。
- **以后**：功率控制（裁判系统给功率上限和缓冲能量，按比例缩放 4 个电流）、超级电容。
- **合并准备**：单板 / 双板、CAN 负载分配（根目录 notes 第 4 节）。

## v1 接口（2026-11-01 草案，用户自己写第一版）

```c
/* chassis.h */
#include "dji_motor_manager.h"

typedef enum
{
    CHASSIS_MODE_RELAX = 0,     /* 无力：输出 0。上电默认，电机掉线时被强制进入 */
    CHASSIS_MODE_SPEED          /* 速度闭环：vx、vy、wz 都在底盘坐标系里 */
} ChassisMode_e;

typedef struct
{
    ChassisMode_e mode;
    float vx_mps;               /* 前进为正，m/s */
    float vy_mps;               /* 向左为正，m/s */
    float wz_dps;               /* 俯视逆时针为正，度/秒（和云台 yaw 同向） */
} ChassisCmd_t;

/* 把 4 个 M3508 注册到这路 CAN 的 manager，必须在 DJI_MotorManager_Start() 之前调用。 */
HAL_StatusTypeDef Chassis_Init(DJI_MotorManager_t *bus);
void Chassis_SetCmd(const ChassisCmd_t *cmd);
/* 每 1 ms 调一次：定模式 → 逆运动学 → 4 个速度环 → 设置电流。发 CAN 帧由 robot.c 统一做。 */
void Chassis_Update(uint32_t now_tick);
```

- v1 没有 `Chassis_GetFeedback`：现在没有别的模块要底盘的数据，调试时直接看 chassis.c 里的变量。以后谁要用再加。

chassis.c 内部（建议）：

- 每个轮子一个 `ChassisWheel_t`，只有两个字段：`DJI_Motor_t motor`（方向在 `motor.motor_direction`，转速反馈看 `motor.speed_rpm`，驱动里已经乘过方向）和 `PID_t speed_pid`（目标转速看 `speed_pid.target`，输出电流看 `speed_pid.out`）。目标转速、电流都不另存。
- 4 个轮子放数组 `chassis_wheel[4]`，下标 0 左前、1 左后、2 右后、3 右前（俯视逆时针，正好是 x 前 y 左坐标系里的第 1~4 象限；和果园机器人麦轮函数的顺序相同，和它全向轮的顺序不同）。电调 ID 用宏单独配，新车建议按下标顺序设成 1~4。
- 另外几个 static，和云台一样：`chassis_cmd`（SetCmd 拷进来的）、`chassis_mode`（实际模式）、`chassis_fault`（掉线锁定）。
- `Chassis_Init`：4 次 `DJI_M3508_Register`（ID 1~4 用 0x200、槽位 ID−1）；电机层 `MotorControl_Init`（电流限幅、方向，PID 参数全给 0）+ `MotorControl_SetMode(MOTOR_MODE_CURRENT)`；`PID_Init` 4 个速度环，输出和积分项都限在 ±电流限幅（和云台一样，积分项最多用满输出）。
- `Chassis_SetCmd`：SPEED 原样收下，其他值当 RELAX。
- `Chassis_Update`：
  1. 定模式：指令 RELAX → 清 fault；指令闭环但有一个轮子掉线（`DJI_Motor_IsOnline`，100 ms）→ fault = 1；fault 时强制 RELAX，要给一次 RELAX 才解除。从 RELAX 进 SPEED 时清 4 个速度环。
  2. RELAX：4 个电流都是 0，PID 不算。
  3. SPEED：逆运动学 → 4 个目标转子转速 → `PID_SetTarget` → `PID_Calculate(反馈 motor.speed_rpm)`。
  4. 输出：`Math_Clamp` 到 ±电流限幅 → `DJI_Motor_SetCurrent` → `MotorControl_Update`（电机层的掉线保护，同云台的 `Gimbal_AxisOutput`）。

逆运动学：v 是轮子线速度，正 = 这个轮子往让底盘前进的方向滚。wz 先换成 rad/s（× π/180）。

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

- 目标转子转速 rpm = v / 轮子半径 × 60 / (2π) × 减速比（`motor.reduction_ratio`）。
- 两种轮子符号规律一样，左右两侧的电机方向也一样（见上一节最后两条）。

chassis_config.h（建议）：4 个电调 ID；4 个方向（预计左侧 1、右侧 −1，待核对）；轮子半径和 a、b（全向轮是 R），待测；电流限幅（C620 ±16384 对应 ±20 A，先给小，比如 3000；果园机器人最后用 8000）；速度环 Kp / Ki / Kd（误差是转子 rpm、输出是电流、dt 1 ms，可从 5 / 20 / 0 起调）。

chassis_test（`App/test_cmd/chassis_test_cmd.c/.h`）：`volatile` 的调试变量 `chassis_test`（mode、vx_mps、vy_mps、wz_dps），`ChassisTestCmd_Update(now_tick, &cmd)` 拷成 `ChassisCmd_t`。建议改 mode 时把三个速度清零（云台是改 mode 时同步目标），免得一进闭环就按旧速度跑；看阶跃响应时可以像云台一样加方波幅值。

robot.c：`Robot_Init` 里在 `DJI_MotorManager_Start` 之前 `Chassis_Init(&robot_can2)`，失败进 `Error_Handler()`；`Robot_Loop` 里在发送之前 `ChassisTestCmd_Update` → `Chassis_SetCmd` → `Chassis_Update`。CMake 加 `App/chassis/chassis.c`、`App/test_cmd/chassis_test_cmd.c` 和 include 目录 `App/chassis`、`App/test_cmd`。.ioc 不用改。

为什么这样写：

- 速度环在 chassis.c 里算、电机层只用电流模式（和云台一样）：motor_control 自带速度模式的积分限幅写死 1000（约 1.2 A），果园机器人上坡时就是被它限住、改了驱动才放到 6000；以后功率控制要在速度环和输出之间按比例缩放 4 个电流，PID 在本模块里算才好插这一步。也可以照果园机器人那样用电机层速度模式，那就要先把 `MotorControl_SetSpeedIntegralLimit` 加进 Shared（改 Shared 要在根目录 notes 第 7 节记一笔、两个工程都重编）。
- 速度环单位用转子 rpm：反馈本来就是转子 rpm，不用换算；轮速、m/s 只在逆运动学里出现一次；果园机器人的参数也能直接拿来起调。
- 一个轮子掉线 4 个都停：少一个轮子出力，合力方向就不对，底盘会乱跑。

容易错的地方：

1. wz 是度/秒，代进公式前要乘 π/180，否则旋转分量大 57 倍。
2. `speed_rpm` 是转子转速，轮子转速要再除 19.2。
3. 方向：标定按「手转轮子、让它往前滚时 `speed_rpm` 为正」来定；驱动里反馈和输出电流都乘了方向，反馈方向标对了，输出方向也就对了（前提是电调正电流对应正转速，果园机器人上是这样）。首次闭环用小电流限幅；如果一闭环就转到最快，就是正反馈，先停下查。
4. 借果园机器人的公式要换坐标系（见上一节），直接抄符号会全反。
5. 麦轮平移方向不对、或平移时底盘自己转：先查电调 ID 和位置是否对应、轮子的辊子朝向（装法），这不是改符号能解决的。
6. 调试器暂停时控制帧停发，C620 停发后怎么处理手册没写：架空测试问题不大，落地前先确认。

## v1 上板步骤与验收（电机到了再做）

1. 接线：4 个 C620 接 CAN2（接 CAN1 就改 robot.c 里传的 manager），设好 ID（绿灯闪 1~4 次），离 C 板最远的电调打开终端电阻。要上 24 V，只接调试器时 CAN 用不了。
2. 架空底盘（轮子离地），上电默认 RELAX。手转轮子：4 个 `motor.feedback_count` 在涨，`speed_rpm` 跟着变。
3. 方向：按上面第 3 条逐个标定。
4. 首次闭环：电流限幅保持小值，`chassis_test.mode` 改 1，`vx_mps` 给 0.2，4 个轮子都应往前转、转速接近目标。
5. 速度环：看 `speed_pid.target` 和 `motor.speed_rpm` 的跟随，先调 P 再加 I。
6. 方向核对（两种轮子一样）：`vy_mps` 给 0.2 → 左前、右后往后转，左后、右前往前转；`wz_dps` 给 30 → 左侧两个往后、右侧两个往前。
7. 落地低速：vx / vy / wz 分别给小值，看底盘实际走向和转向符合约定（x 前、y 左、wz 俯视逆时针）。
8. 保护：拔掉一个电调的 CAN 线，4 个轮子都无力，`chassis_fault` 变 1。

验收：四个轮子跟得上目标转速、不振荡；底盘按指令方向走；掉线自动无力。电流限幅调好再放开。

## 标定记录（测完填）

| 项 | 值 |
|---|---|
| 轮型 | 待定 |
| 底盘电机所在总线 | 代码默认 CAN2，待按实际接线确认 |
| 电调 ID ↔ 位置 | 建议 1 左前、2 左后、3 右后、4 右前 |
| 各轮 motor_direction | 待测（按果园机器人的实测推出来是左 1、右 −1） |
| 轮子半径、a / b（全向轮 R） | 待测 |
| 速度环 PID | 待调（起点 5 / 20 / 0） |
