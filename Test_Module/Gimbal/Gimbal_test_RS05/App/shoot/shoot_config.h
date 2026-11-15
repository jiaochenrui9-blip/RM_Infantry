#ifndef SHOOT_CONFIG_H
#define SHOOT_CONFIG_H

/* 三台电机都接 C 板 CAN2，按电调 ID 命名；哪个是左、哪个是右还没确认。 */
#define SHOOT_FRICTION_1_ESC_ID       1U
#define SHOOT_FRICTION_2_ESC_ID       2U
#define SHOOT_FEEDER_ESC_ID           3U

/* 方向待实测（RELAX 下手转看 speed_rpm 的符号）：摩擦轮逻辑正向 = 往外送弹，拨弹轮逻辑正向 = 进弹。
   两个摩擦轮面对面装，往外送弹时电机转向相反，所以两个里多半有一个是 -1。 */
#define SHOOT_FRICTION_1_DIRECTION    1
#define SHOOT_FRICTION_2_DIRECTION    1
#define SHOOT_FEEDER_DIRECTION        1

/* 任一电机这么久没反馈就锁进 RELAX，要先给一次 RELAX 指令才解除。电机层自己是 100 ms 清输出。 */
#define SHOOT_OFFLINE_TIMEOUT_MS      50U

/* 摩擦轮就绪：两个摩擦轮的 |目标 − speed_rpm|（speed_rpm 已按方向修正）都小于 ERR，
   并且连续 HOLD_MS 都这样，才允许拨弹；掉出去一次就重新计时。建议的初值，没实测。 */
#define SHOOT_FRICTION_READY_ERR_RPM  200
#define SHOOT_FRICTION_READY_HOLD_MS  200U

/* 电流限幅是协议原始值：C620 ±16384 = ±20 A，C610 ±10000 = ±10 A。
   第一次上电用小的：摩擦轮 6000（约 7.3 A，只影响起转快慢），拨弹 3000（3 A，卡弹时不至于打坏拨盘）。 */
#define SHOOT_FRICTION_CURRENT_LIMIT  6000.0f
#define SHOOT_FEEDER_CURRENT_LIMIT    3000.0f

/* 速度环，目标和反馈都是转子 rpm。初值：摩擦轮按底盘 M3508 那组（5 / 20），拨弹按 F405 上 M2006 实测转得起来的那组（3 / 20），
   都没带负载调过。注意电机层把速度环积分项限在 ±1000（motor_control.c 的 SPEED_PID_I_OUTPUT_LIMIT）。 */
#define SHOOT_FRICTION_SPEED_KP       5.0f
#define SHOOT_FRICTION_SPEED_KI       20.0f
#define SHOOT_FRICTION_SPEED_KD       0.0f
#define SHOOT_FEEDER_SPEED_KP         3.0f
#define SHOOT_FEEDER_SPEED_KI         20.0f
#define SHOOT_FEEDER_SPEED_KD         0.0f

#endif
