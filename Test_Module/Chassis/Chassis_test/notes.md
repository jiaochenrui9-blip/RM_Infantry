# Chassis_test 笔记（底盘测试工程）

> 本工程的进度与注意事项。跨模块约定（分层、接口、单位、正方向、CAN 分配、C 板引脚）见 `RM_Infantry/notes.md`；云台那边踩过的坑见 `Test_Module/Gimbal/Gimbal_test/notes.md` 的「已知问题」，大部分底盘也适用。
> 最后更新：2026-11-03（chassis_link 搬到 Shared/Link，`ChassisMode_e`、`ChassisCmd_t` 移进 chassis_link.h，和云台板共用一份；代码没改，固件逐字节相同。云台板的发送同日写好、主机测试通过，见 Gimbal_test/notes.md「板间通信」）

## 当前状态

- 2026-11-01 建好工程，还没有底盘模块：App 里只有空的 robot.c（两路 CAN 的 manager、1 ms 调度、每周期发送、CAN 接收分发），没注册电机，SendAll 什么也不发。清理编译无警告（FLASH 11676 B、RAM 1752 B），没上板。
- 2026-11-01 用户定：按步兵的逻辑做，底盘接收云台主控发来的速度和模式。接口按这个定，见「接口」（同一天早些时候那版只有底盘坐标系速度的草案作废）。遥控接收、测试用的指令源用户写过很多遍，这里不展开。
- 用户手上还没有电机。写完先做主机模型测试（和云台一样），电机到了再上板。
- 轮型未定（2026-11-01 问过，用户没选）。接口和轮型无关，麦轮、全向轮的逆运动学都写在「接口」里，定了再写对应那组。
- 参考：用户自己的果园机器人底盘（全向轮，已上板调通），见「参考：果园机器人底盘」。
- 2026-11-01 用户开始写（`App/Infantry.h`、`App/Infantry_Chassis.c`，速度环用电机层自带的、轮子顺时针编号），并让 Claude 写好初始化和接收指令：`Infantry_Chassis_Init`、`Chassis_SetCmd`、`chassis_config.h`、`chassis_link`（板间帧接收），robot.c 接好。ARM 清理编译无警告（FLASH 13764 B、RAM 2544 B），主机测试通过（见「初始化和接收的验证」），没上板。
- 2026-11-01 用户写了 `Chassis_Update` 的开头（先切模式、最后统一 `MotorControl_Update`），说后面不太会，让 Claude 写完：四种模式、掉线锁定、斜坡、坐标变换、逆运动学（轮型只体现在两个参数上）、超转速等比例缩小；robot.c 每周期调用。ARM 清理编译无警告（FLASH 20876 B、RAM 2952 B）；主机模型测试 53 项通过（见「Chassis_Update 的验证」），没上板。
- 2026-11-02 用户把 `App/Infantry.h` 改名为 `App/Infantry_Chassis.h`（include 保护宏还是 `CHASSIS_TEST_INFANTRY_H`，不影响），Infantry_Chassis.c 里加了几处注释。
- 2026-11-03 为了让云台板用同一份帧格式和类型（用户选的）：`chassis_link.c/.h` 从 App 原样搬到 `Shared/Link`，`ChassisMode_e`、`ChassisCmd_t` 从 Infantry_Chassis.h 移进 chassis_link.h（Infantry_Chassis.h 改成 include 它），CMake 改路径。代码没改：清理编译无警告，FLASH 20876 B、RAM 2952 B，烧进 Flash 的内容和搬之前逐字节相同。云台板的发送同日写好（用户写第一版、Claude 按审查改好），主机上和这边的解包对过，两板联调待做，见 Gimbal_test/notes.md「板间通信」。
- 2026-11-03 整车主机仿真放进仓库：`Test_Module/CarSim`，云台板 + 底盘板的真代码（含本工程的 Infantry_Chassis.c、Shared/Link）编在一起，接 4 个 M3508 麦轮刚体底盘 + 云台模型，12 个场景（RELAX、不跟随直走 / 横移、跟随、边走边转、小陀螺、超速等比例缩小、编码器模式下跟随降级、板间断线、轮子掉线锁定、模式切换）全部通过。**改了底盘代码就跑一遍**，用法见其 README。以前 notes 里记的主机测试是临时目录里的，以它为准。
- 第 1、2 步的代码都有了（第 3 步只差云台板的发送）。下一步：电机到了按「上板步骤」标方向、调速度环；轮型定了改 `CHASSIS_XY_FACTOR`、`CHASSIS_ROTATE_RADIUS_M`。

## 工程是怎么建的（以后改外设照这个做）

- `Chassis_test.ioc` 从 `Gimbal_test.ioc` 改来，和它的区别只有：
  - 去掉 SPI1、DMA 和 BMI088 的 4 个 GPIO（底盘不用 IMU）；
  - 去掉 CAN1_RX1 中断（过滤器都进 FIFO0，见 `Shared/BSP/can_bus.h`，这个中断用不上）；
  - 两路 CAN 打开 Automatic Bus-Off Management：Gimbal_test 已知问题第 11 条——只接调试器 3.3 V 时 CAN 会 bus-off，没开这项就得复位才能恢复；开了以后上 24 V 应能自己恢复（还没上板验证；果园机器人工程也开着）；
  - Project Manager → Code Generator 改成只拷用到的库文件：Drivers 6.4 MB / 76 个文件（云台工程拷了全部，57 MB）。
  - 其余（HSE 12 MHz → 168 MHz、CAN 1 Mbps：预分频 3 / BS1 9 / BS2 4、NVIC 优先级分组 4、CAN 接收中断优先级 0）和云台相同。
- 代码由 STM32CubeMX 6.15.0 生成。这次用的是命令行无界面模式：脚本里写 `config load <ioc>`、`project generate`、`exit`，在 CubeMX 安装目录下运行 `jre\bin\java -jar STM32CubeMX.exe -q <脚本>`（UM1718 第 3.3.2 节）。平时用 GUI 打开 `.ioc` 改完点 GENERATE CODE 即可。
- 用户代码只在 USER CODE 段里：main.c 的 `#include "robot.h"`、`Robot_Init()`、`Robot_Loop()`，重新生成不会丢。其他全在 App/。
- 顶层 CMakeLists.txt（CubeMX 只生成一次）：加了 `SHARED_DIR`、`App/robot.c` 和底盘要用的 Shared 代码（Link/chassis_link、dji_motor、dji_motor_manager、dji_m3508、motor_control、PID、user_math、can_bus）。GM6020、M2006、BMI088、IST8310、Attitude 没加。
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
- 要借这个底盘测第 1、2 步：底盘在 CAN1，robot.c 里 `Infantry_Chassis_Init` 改传 `&robot_can1`；ID 不用重设，config 里填前左 4、前右 1、后右 2、后左 3；用全向轮公式，R 和轮子半径按那台车量。

## 版本规划（按步兵逻辑，分 3 步，每步单独验收）

1. **第 1 步：不跟随模式**——坐标变换、逆运动学（含超转速等比例缩小）、4 个速度环、RELAX 和掉线锁定。底盘单独就能测，云台角在调试器里手填。
2. **第 2 步：跟随云台、小陀螺 + 加速度斜坡**。符号底盘单独能测（云台角填固定值）；真正的跟随效果要云台在 IMU 模式、第 3 步通了以后看。
3. **第 3 步：板间通信 chassis_link**。底盘板的接收 2026-11-01 已写（用户要求提前做）；2026-11-03 chassis_link 搬到 Shared/Link、云台板发送写好（主机测试通过），两板联调两块 C 板就能做。单板方案不要这一步，robot_cmd 直接填 `ChassisCmd_t`。
- **以后**：功率控制（裁判系统给功率上限和缓冲能量，在速度环和输出之间按比例缩小 4 个电流）、超级电容、小陀螺时平移方向的延迟补偿、变速小陀螺、底盘→云台的回传帧（裁判系统接在底盘板时，把热量 / 弹速上限传给云台板）。

## 接口（2026-11-01 定，按步兵逻辑；初始化、接收、Chassis_Update 都已写）

文件（用户定的名字）：`App/Infantry_Chassis.h`（底盘的函数；2026-11-02 用户从 Infantry.h 改名）、`App/Infantry_Chassis.c`（底盘模块）、`App/chassis_config.h`（参数）、`Shared/Link/chassis_link.c/.h`（板间通信，和云台板共用一份；`ChassisMode_e`、`ChassisCmd_t` 也在这里，2026-11-03 从 Infantry_Chassis.h 移过来）。

数据流：云台板（键鼠 / 遥控 → robot_cmd；云台角来自 `Gimbal_GetFeedback` 的 `yaw_rel_deg`）→ 板间 CAN 0x110 → 底盘板 `ChassisLink_GetCmd` → robot.c → `Chassis_SetCmd` → `Chassis_Update` → 4 个 M3508。

```c
/* Shared/Link/chassis_link.h（模式和指令结构体是用户写的，2026-11-03 从 Infantry_Chassis.h 移过来，和这里一样） */
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

/* App/Infantry_Chassis.h：#include "dji_motor_manager.h"、"chassis_link.h"（只用上面两个类型） */
/* 已写：把 4 个 M3508 注册到这路 CAN 的 manager，必须在 DJI_MotorManager_Start() 之前调用；注册失败返回 HAL_ERROR。 */
HAL_StatusTypeDef Infantry_Chassis_Init(DJI_MotorManager_t *manager);
/* 已写：每 1 ms 在 Chassis_Update 之前调，非法模式当 RELAX。 */
void Chassis_SetCmd(const ChassisCmd_t *cmd);
/* 已写：每 1 ms 调一次：定模式 → 算 wz → 斜坡 → 坐标变换 → 逆运动学 → 4 个电机的目标转速 → MotorControl_Update。
   发 CAN 帧由 robot.c 统一做。运动模式下任一电机掉线（100 ms）→ 无力并锁住，指令给一次 RELAX 才解除。 */
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

1. 定模式：指令 RELAX → 清 fault；指令是运动模式但有一个轮子掉线（`DJI_Motor_IsOnline`，100 ms）→ fault = 1；fault 时强制 RELAX，要给一次 RELAX 才解除。从 RELAX 进运动模式时电机层切到速度模式（`MotorControl_SetMode` 会清速度环），斜坡从 0 开始；运动模式之间切换电机层一直是速度模式、不清速度环，斜坡接着走；切进 FOLLOW 时清跟随环。
2. 按模式算目标 wz（上表）。跟随只用 P：wz = `Math_Clamp(CHASSIS_FOLLOW_KP × gimbal_yaw_rel, ±CHASSIS_FOLLOW_MAX_DPS)`，不用 PID_t。原因：微分在云台角过 ±180° 时会冲一下；不需要积分（轮子的速度环有积分，云台不动时 P 就能收敛到 0）。
3. 斜坡（第 2 步加）：(vx, vy) 在**云台坐标系**里往指令走一步，按向量长度每毫秒最多变 `CHASSIS_ACCEL_MPS2` × 0.001，方向不变；wz 每毫秒最多变 `CHASSIS_WZ_ACCEL_DPS2` × 0.001。
4. 坐标变换（θ = gimbal_yaw_rel，换成 rad）：vx_底 = vx·cosθ − vy·sinθ，vy_底 = vx·sinθ + vy·cosθ。
5. 逆运动学（公式见下）→ 4 个目标转子转速；任一个超过 `CHASSIS_WHEEL_MAX_RPM` 时 4 个一起等比例缩小（合成方向不变）。代码里两种轮子写成一套：前左 = k(vx − vy) − r·wz、前右 = k(vx + vy) + r·wz、后右 = k(vx − vy) + r·wz、后左 = k(vx + vy) − r·wz；k = `CHASSIS_XY_FACTOR`（麦轮 1、全向轮 √2/2），r = `CHASSIS_ROTATE_RADIUS_M`（麦轮 a + b、全向轮 R）。轮型用参数值表达，不加开关。
6. 4 个目标转速交给电机层：`MotorControl_SetTargetSpeed`（转子 rpm，int16）→ 每个电机 `MotorControl_Update(motor, now)`（速度环、电流限幅、100 ms 掉线保护都在里面）。以后功率控制放在 MotorControl_Update 之后、SendAll 之前，按比例缩 4 个 `output_current`。

Infantry_Chassis.c 内部：

- 4 个电机 `static DJI_Motor_t chassis_motors[4]`，下标 0 前左、1 前右、2 后右、3 后左（俯视顺时针，用户定的；文件里有 `CHASSIS_FL` 等下标）。方向在 `motor.motor_direction`，转速反馈看 `motor.speed_rpm`（驱动里已经乘过方向）。
- 速度环用电机层自带的（用户的草稿这样写，和果园机器人一样）：速度环参数在初始化时传给 `MotorControl_Init`，PID 就是 `motor.speed.pid`，目标转速看 `motor.speed.target_rpm`。注册完电机层是 `MOTOR_MODE_DISABLE`（输出 0）。切模式用 `MotorControl_SetMode`，而且只在电机层模式真的变了才调：每周期都调会把速度环积分清掉（果园机器人 Chassis_Control.c 里的注释）。
- 已写（2026-11-01，Claude）：`Infantry_Chassis_Init`：4 次 `DJI_M3508_Register`（ID 1~4 用 0x200、槽位 ID−1）+ `MotorControl_Init`（电流限幅、方向、速度环参数，位置环给 0），任一个失败就返回 HAL_ERROR；`Chassis_SetCmd`：三种运动模式原样收下，其他值当 RELAX。static `chassis_cmd`（SetCmd 拷进来的）。
- `Chassis_Update`（2026-11-01，用户写了开头，Claude 补完）：`Chassis_ModeUpdate`（掉线锁定，电机层模式只在变了时切：从 RELAX 进运动模式 SetMode(SPEED)、进 RELAX 时 SetMode(DISABLE) 并清斜坡）→ 按模式算 wz → `Chassis_Ramp` → 坐标变换 → `Chassis_WheelRpm`（逆运动学 + 超转速缩小）→ `MotorControl_SetTargetSpeed` → 最后 4 个 `MotorControl_Update`（RELAX 时也调，输出 0）。static 多了 `chassis_mode`、`chassis_fault`、斜坡后的 `chassis_vx_mps`、`chassis_vy_mps`（云台坐标系）、`chassis_wz_dps`，调试时看这几个。

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

chassis_config.h：

| 宏 | 内容 |
|---|---|
| `CHASSIS_FL/FR/BR/BL_ESC_ID` | 已写：1~4，按下标顺序；只能 1~4（都走 0x200） |
| `CHASSIS_FL/FR/BR/BL_DIRECTION` | 已写：前左 1、前右 −1、后右 −1、后左 1（按果园机器人推出来的左 1 右 −1），待核对 |
| `CHASSIS_CURRENT_LIMIT` | 已写：3000（C620 ±16384 对应 ±20 A；先给小，果园机器人最后用 8000） |
| `CHASSIS_SPEED_KP/KI/KD` | 已写：5 / 20 / 0（误差是转子 rpm、输出是电流、dt 1 ms；积分项被 motor_control 限在 1000） |
| `CHASSIS_WHEEL_RADIUS_M`、`CHASSIS_XY_FACTOR`、`CHASSIS_ROTATE_RADIUS_M` | 已写占位：0.076、1（麦轮）、0.40；待测。全向轮改成 0.7071 和轮心到中心距离 |
| `CHASSIS_WHEEL_MAX_RPM` | 已写：8000（转子转速上限，空载约 9256，果园机器人也用 8000） |
| `CHASSIS_FOLLOW_KP`、`CHASSIS_FOLLOW_MAX_DPS` | 已写：3（度/秒 每 度）、180；只用 P。主机模型里 Kp 5 以内不超调，8 开始超调约 1.7° |
| `CHASSIS_SPIN_DPS` | 已写：180（先给小） |
| `CHASSIS_ACCEL_MPS2`、`CHASSIS_WZ_ACCEL_DPS2` | 已写：3 m/s²（按速度向量长度限）、720 度/s²；待调 |

板间通信（`Shared/Link/chassis_link.c/.h`，和云台板共用一份，2026-11-03 前在本工程 App/ 里；底盘板这一侧 2026-11-01 已写；帧格式写在 chassis_link.h 里，只这一处）：

| 字节 | 内容 |
|---|---|
| 0 | mode（`ChassisMode_e`） |
| 1 | 保留，发 0 |
| 2~3 | vx，int16，mm/s |
| 4~5 | vy，int16，mm/s |
| 6~7 | gimbal_yaw_rel，int16，0.01°（±180° 是 ±18000） |

标准帧 0x110，DLC 8，云台板每 1 ms 发一次，高字节在前（和大疆电机帧一样）。走两块板之间那路 CAN：底盘板默认 CAN1（电机在 CAN2）。

```c
/* chassis_link.h：上面的 ChassisMode_e、ChassisCmd_t 也定义在这里；下面三个是底盘板调的 */
#define CHASSIS_LINK_CMD_ID         0x110U
#define CHASSIS_LINK_TIMEOUT_MS     50U

/* 给 0x110 配一个过滤器组：CAN1 用组 2、CAN2 用组 16。CAN 启动前后调都行。 */
HAL_StatusTypeDef ChassisLink_Init(CAN_HandleTypeDef *hcan);
/* CAN 接收回调（中断）里调：这路上读到的每一帧都交给它，它只认 0x110，只存原始数据和时间。 */
void ChassisLink_ProcessRxFrame(const CAN_RxHeaderTypeDef *header, const uint8_t data[8]);
/* 主循环每 1 ms 调：关中断拷一份再解码；超时（包括上电后还没收到过）给 RELAX、速度 0。 */
void ChassisLink_GetCmd(uint32_t now_tick, ChassisCmd_t *cmd);
```

- 云台板的发送 `ChassisLink_SendCmd`（按上表打包、`CAN_Bus_SendStdData`）：2026-11-03 已写（同一个 chassis_link.c 里），见 Gimbal_test/notes.md「板间通信」。本工程不调它，链接时被裁掉。
- 超时按有符号数比：robot.c 读完 now_tick 之后才收到的帧，记下的 tick 会比 now_tick 大，按无符号比会被当成超时。

robot.c（底盘板，2026-11-01 已接好）：`Robot_Init` 里在 `DJI_MotorManager_Start` 之前 `Infantry_Chassis_Init(&robot_can2)`、`ChassisLink_Init(&hcan1)`，失败都进 `Error_Handler()`；`Robot_Loop` 里在发送之前 `ChassisLink_GetCmd(now, &cmd)` → `Chassis_SetCmd(&cmd)`，`Chassis_Update` 写好后加在“2. 控制”那里；CAN 接收回调里 CAN1 的每一帧交给 `ChassisLink_ProcessRxFrame` 和 robot_can1 的 manager（各自按 ID 只认自己的），CAN2 的交给 robot_can2。CMake 里加了 `${SHARED_DIR}/Link/chassis_link.c` 和 include 目录 `${SHARED_DIR}/Link`（2026-11-03 前是 `App/chassis_link.c`；用户的两个文件 CLion 加在了 add_executable 里，也能编）。.ioc 不用改。

- 还没有云台板时测底盘：robot.c 里临时把 `ChassisLink_GetCmd` 换成调试器改的一个 `volatile ChassisCmd_t`，或者用另一块板 / USB-CAN 按上面的格式发 0x110（50 ms 内不发就会 RELAX）。

为什么这样写：

- 模式和跟随环放底盘板，云台板只发意图和云台角：底盘怎么转（跟随环、小陀螺、斜坡、功率）都在一处调；单板方案也是同一个 `ChassisCmd_t`，只是不经过 chassis_link。
- 平移用云台坐标系：操作手按 W 永远朝枪口走；小陀螺时底盘在下面转，平移方向在地面上不跟着转（2026-11-01 平面模型：360°/s 小陀螺、云台对地不动时，地面上的平移方向误差不到 1e-13°）。
- 斜坡加在坐标变换**之前**：小陀螺时底盘坐标系里的速度方向一直在转，2 m/s、360°/s 时分量的变化率约 12.6 m/s²。同一个模型里把 3 m/s² 的斜坡放在变换之后，2 m/s 的指令只剩 0.38~0.55 m/s，方向偏 66~83°。
- 速度环用电机层自带的（用户的草稿这样写，和果园机器人一样；之前建议过在模块里自己算）：模块代码少。代价：motor_control 的速度环积分项写死在 1000（约 1.2 A），果园机器人上坡时就是被它限住——到时把果园机器人的 `MotorControl_SetSpeedIntegralLimit`（9 行）移进 Shared（改 Shared 要在根目录 notes 第 7 节记一笔、两个工程都重编）；功率控制只能在 MotorControl_Update 之后改 `output_current`。
- 板间接收拆成 chassis_link，不放进底盘模块：中断里只拷 8 字节，解码、超时在主循环；底盘模块只认 `ChassisCmd_t`，没有云台板时也能换个指令源测。
- chassis_link 放 Shared、两个类型放 chassis_link.h（2026-11-03 用户选的）：两块板编译同一份，帧格式和模式编号不会两边对不上。代价是底盘的 Cmd 类型不在底盘模块自己的头文件里（根目录 notes 第 2 节规则 2 的例外）。
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
11. `CAN_RxHeaderTypeDef` 里 `IDE` 是帧类型（`CAN_ID_STD` = 0、`CAN_ID_EXT` = 4），帧 ID 在 `StdId` / `ExtId`。用户 2026-11-01 的草稿写成了 `header.IDE == 0x110`，永远不成立。

## 初始化和接收的验证（2026-11-01）

- ARM：清理编译无警告，FLASH 13764 B、RAM 2544 B（那时 `Chassis_Update` 还没被调用，链接时被裁掉）。Chassis_Update 写完后重跑这个测试（robot.c 现在每周期调它），仍是 59 项通过。
- 主机测试（CLion 自带的 MinGW gcc 13.1，`-Wall -Wextra` 无警告；robot.c、Infantry_Chassis.c、chassis_link.c 原文件 + Shared 的驱动、PID 原文件，HAL 是假的；测试代码在临时目录，不在工程里），59 项全部通过：
  - 初始化：0x110 的过滤器在 CAN1 组 2、16 位列表、FIFO0；大疆的过滤器 CAN1 组 0、CAN2 组 14；两路 CAN 都启动；CAN2 上注册 4 个电机，`chassis_motors[0~3]` 的电调 ID 1~4、反馈 0x201~0x204、0x200 槽位 0~3、方向 1 / −1 / −1 / 1、电流限幅 3000、速度环 5 / 20 / 0（积分上限 1000）、失能、输出 0；CAN1 上没有电机。
  - 上电还没收到指令：RELAX；每周期 CAN2 发一帧全 0 的 0x200，CAN1 什么也不发（板间那路上还有云台电机，不能乱发）。
  - 解码：FOLLOW、vx 1.234、vy −0.567、云台角 −179.99；极端值 ±32.767 m/s、180°；非法模式（9）→ RELAX。
  - 不收别人的帧：0x111、扩展帧、远程帧、DLC 6、从 CAN2 来的 0x110、CAN1 上云台电机的 0x205 都不改数据。
  - 超时：最后一帧后 50 ms 还有效，51 ms 起 RELAX、速度清零；帧回来后恢复。比 now_tick 新 1 ms 的帧不算超时。
  - 关中断拷贝后恢复原来的中断状态；电机反馈同一个回调里照常更新（前右方向 −1 时 1000 rpm 读成 −1000）；重复注册、NULL、不认识的 CAN 句柄都返回 HAL_ERROR，CAN2 上的过滤器在组 16。
- 反向检查：把超时改成无符号比较、恢复中断改成直接开中断、字节序反过来、去掉帧 ID 检查，四种改法分别有 1、1、3、2 项失败，都被抓到。

## Chassis_Update 的验证（2026-11-01）

- ARM：清理编译无警告，FLASH 20876 B、RAM 2952 B。
- 主机模型测试（同上，Infantry_Chassis.c 原文件 + Shared 驱动 / PID / user_math 原文件 + 假 HAL；测试代码在临时目录）：电机用惯性为主的模型（原始转速' = 2.5 rpm/s 每单位电流 × 电流 − 转速 / 2 s），电流从 0x200 帧里取（已乘方向），转速按原始方向做成反馈帧送回驱动；底盘实际运动由 4 个轮速反算；云台按 IMU 模式保持对地朝向。53 项全部通过：
  - 上电时电机还没反馈就要求运动 → 锁住、电流 0；有反馈后仍锁着，给一次 RELAX 才解除（和云台一样，所以云台主控上电要先发 RELAX）。
  - 不跟随、vx 0 → 1 m/s：斜坡 100 ms 时 0.300；4 个目标都是 2413 转子 rpm（1 m/s，轮半径 0.076）；底盘实际 1.000 m/s 往前，不横移、不转。稳定后速度环积分保持住（没有每周期被清）。
  - 坐标变换：云台角 90°、沿枪口 0.5 m/s → 前左、后右 −1206，前右、后左 +1206，底盘实际往左 0.5 m/s。
  - 跟随符号：云台角固定 +10° → wz 30°/s 逆时针（左侧轮负、右侧正）；−10° → −30°/s。
  - 运动模式之间切换（不跟随 → 小陀螺）不清速度环积分。
  - 真正的跟随：云台对地 60°、底盘从 0° 起 → 不超调，1.78 s 进 1° 以内，最后差 0.011°。
  - 小陀螺 180°/s + 沿枪口 1 m/s：给电机的目标反算出来，地面走向误差 ≤ 0.012°、速度 1.000 ± 0.0003 m/s、转速 180 ± 0.03°/s。模型里的实际运动受速度环带宽限制：电流限幅 3000 时电流顶满，走向偏 17.8°、只转到 158°/s；限幅 8000、Kp 5 时峰值电流 4555，走向偏 11.2°；限幅 8000、Kp 15 时偏 4.3°。原因：小陀螺加平移时每个轮子的目标转速按小陀螺的频率正弦变化（180°/s 是 0.5 Hz，平移分量幅值 √2 × 1 m/s），要几 m/s² 的来回加速度，速度环跟不上就有相位滞后。实车上看这一项要先把电流限幅放开、速度环调硬；这个数和电机模型有关，只说明趋势。
  - 超转速：vx 3、vy 3 → 前右、后左缩到 8000，前左、后右 0；小陀螺 + 3 m/s 时最大 8000。
  - 运动中一个电机停发反馈：100 ms 时还没锁，102 ms 锁住，4 个都失能、电流 0、斜坡清零；反馈恢复后仍锁着，给一次 RELAX 再给运动才恢复，恢复后 1.000 m/s。非法模式（7）当 RELAX，电流 0。
- 反向检查：跟随符号反、坐标变换符号反、去掉超转速缩小、每次换模式都 SetMode(SPEED)、去掉掉线锁定、进 RELAX 不清斜坡，六种改法分别有 4、8、2、1、13、1 项失败，都被抓到。

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

1. 云台板发 0x110，底盘板收到（看 chassis_link 里最后一帧的 tick 跟着走）。不接电机、两块 C 板就能做的部分见 Gimbal_test/notes.md「板间通信」的验收。
2. 拔掉板间 CAN 线，50 ms 内底盘无力；插回后按当前指令恢复。
3. 全车：键鼠切四种模式，平移方向始终跟枪口走；小陀螺时边转边平移，走向不歪。

验收：四个轮子跟得上目标转速、不振荡；四种模式行为符合上表；掉线、断通信自动无力。电流限幅调好再放开。

## 标定记录（测完填）

| 项 | 值 |
|---|---|
| 轮型 | 待定 |
| 底盘电机所在总线 | 代码默认 CAN2，待按实际接线确认 |
| 板间通信总线 | 底盘板默认 CAN1，待按实际接线确认 |
| 电调 ID ↔ 位置 | 代码默认 1 前左、2 前右、3 后右、4 后左（俯视顺时针，用户定的编号） |
| 各轮 motor_direction | 待测（按果园机器人的实测推出来是左 1、右 −1） |
| 轮子半径、a / b（全向轮 R） | 待测（代码占位 0.076 m、a + b = 0.40 m） |
| 速度环 PID | 待调（起点 5 / 20 / 0） |
| 跟随环 Kp、最大 wz | 待调 |
| 小陀螺转速、两个加速度限制 | 待调 |
