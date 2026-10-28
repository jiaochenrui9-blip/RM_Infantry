# Gimbal_test 笔记（云台测试工程）

> 本工程的进度与注意事项。跨模块约定（分层、接口、单位、正方向、CAN 分配、C 板引脚）见 `RM_Infantry/notes.md`；驱动移植细节和验证记录见本目录 `README.md`；BMI088 / Attitude 的接入说明见 `Shared/Devices/BMI088/README.md`。
> 最后更新：2026-10-27

## 当前状态

- .ioc：CAN1 已改到 PD0/PD1；SPI1 + DMA、BMI088 片选和数据就绪输入已配置。IST8310 的 I2C3、IMU 加热的 TIM10 还没配。
- App 是精简版的 v1（只有编码器定角度），共 488 行：编译无警告，主机闭环模型测试通过（见 README）。**还没上板**：方向、中值、限位、PID 都是占位值。
- 之前带 IMU 模式的完整版（935 行）备份在 `Test_Module/Gimbal/Backup/App_full_2026-10-27`，不参与编译，v2 写 IMU 时参考。用户觉得变量太多、太乱，所以重写成现在这版；反思写进了全局 CLAUDE.md。

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
│   └── gimbal_config.h          ID、方向、中值、限位、限幅、斜坡、重力前馈、PID
└── test_cmd/
    └── gimbal_test_cmd.c / .h   调试器指令源（合并后由 robot_cmd 取代）
Shared/Algorithm/user_math.*     Math_Clamp、Math_WrapDeg180
```

`Robot_Loop()` 每 1 ms：`Gimbal_GetFeedback` → `GimbalTestCmd_Update` → `Gimbal_SetCmd` → `Gimbal_Update` → 每路 CAN `SendAll` 一次。

gimbal.h 对外：`GimbalMode_e`（RELAX / ENCODER）、`GimbalCmd_t`（模式 + 两个目标角）、`GimbalFeedback_t`（模式 + 两个当前角），以及 Init / SetCmd / Update / GetFeedback 四个函数。

gimbal.c 内部每个轴一个 `GimbalAxis_t`（9 个字段）：电机、中值、上下限位、角度环和速度环 PID、当前角、角速度、斜坡后的目标。方向存在 `motor.motor_direction`，环的输出就看 `angle_pid.out` / `speed_pid.out`，不另存。

## 怎么用（CLion 调试器）

| 变量 | 内容 |
|---|---|
| `gimbal_test` | 指令：`mode`（0 无力 / 1 闭环）、`yaw_deg`、`pitch_deg`；方波：`yaw_square_deg`、`pitch_square_deg`（幅值，0 为不加）、`square_period_ms` |
| `gimbal_yaw`、`gimbal_pitch` | 每个轴：`angle_deg`、`rate_dps`、`target_deg`、`angle_pid.out`（目标角速度）、`speed_pid.out`（电流）、`motor.encoder`（原始编码器） |
| `gimbal_mode`、`gimbal_fault` | 实际模式、是否被掉线锁定 |

流程：上电 → RELAX 下手转标定中值和方向 → `gimbal_test.mode` 改成 1 → 再改 `yaw_deg` / `pitch_deg`。改 mode 时目标会自动同步成当前角，所以要先切模式、再改目标。

## 版本规划

- **v1 编码器定角度**：代码已完成，待上板标定和调参。
- **v2 IMU 定角度**：等 C 板装到云台上再写。需要：验证 C 板轴映射、配 TIM10 加热恒温（手册建议比板子正常工作温度高 15~20 ℃）、接 ins。完整版备份里有一份参考实现（ins 模块、IMU 模式、轴映射），没上过板。
- **v3 遥控器**：DR16 接 USART3（PC11），手册写 100 kbps；其余串口参数本目录没有 DR16 手册，用前核对。
- **合并准备**：定单板 / 双板；按根目录 notes.md 的 CAN 表分配负载。

## v1 调试步骤与验收

1. 看反馈：上电默认 RELAX，只发 0 电流。手转时 `gimbal_yaw.motor.encoder` 跟着变。
2. 标定：扶到正前方 / 水平，记下 `motor.encoder` 填进 `GIMBAL_*_ECD_OFFSET`；pitch 推到上下极限记 `angle_deg`，填软限位（留几度余量）；按约定正方向手转，`angle_deg` 和 `rate_dps` 都应变大，否则方向改 -1。
3. 首次闭环：电流限幅保持小值，`mode` 改 1，把目标改动 5° 左右，云台应往目标方向走。
4. 速度环：看 `angle_pid.out`（目标角速度）和 `rate_dps` 的跟随，先调 P 再加 I（pitch 有重力，P 不够时会下垂，靠 I 或 `GIMBAL_PITCH_GRAVITY_FF`）。
5. 角度环：手推看回位；把 `yaw_square_deg` 设成 10 左右用方波看超调和调节时间（斜坡会把阶跃变缓，看阶跃时可临时把 `GIMBAL_RAMP_DEG_PER_MS` 调大）。
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
