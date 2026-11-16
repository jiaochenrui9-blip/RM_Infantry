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

/* 卡弹（只在实际 FEED、feeder_rpm > 0 时判）：拨弹轮 output_current ≥ JAM_CURRENT 且 speed_rpm < JAM_SPEED_RPM，
   连续 JAM_DETECT_MS 算卡一次 → 切位置模式往回转 JAM_REVERSE_DEG，转到（差 < DONE_DEG）或超时（TIMEOUT_MS）再正转。
   正转顺利跑满 JAM_CLEAR_MS 次数清零；连续卡到 JAM_MAX_COUNT 次就锁住（shoot_fault，给一次 RELAX 才解除）。
   JAM_CURRENT 必须小于 SHOOT_FEEDER_CURRENT_LIMIT，不然永远判不出来。全是建议值，转速都是转子 rpm（M2006 减速比 36）。
   角度是 M2006 输出轴（减速后、逻辑方向）的度数；拨盘和输出轴之间如果还有传动，按输出轴算，拨盘格数知道以后再换算。 */
#define SHOOT_JAM_CURRENT             2700
#define SHOOT_JAM_SPEED_RPM           300
#define SHOOT_JAM_DETECT_MS           100U
#define SHOOT_JAM_REVERSE_DEG         30.0f   /* 8 格拨盘一格 45°，先给 30° */
#define SHOOT_JAM_REVERSE_DONE_DEG    3.0f
#define SHOOT_JAM_REVERSE_TIMEOUT_MS  300U    /* 反转本身也被挡住时最多等这么久，回去再卡会再判一次、次数照常加 */
#define SHOOT_JAM_CLEAR_MS            500U
#define SHOOT_JAM_MAX_COUNT           3U

/* 拨弹轮位置环（电机层 1 ms 一次，和速度环一样）：输入输出轴角度（度），输出给速度环的目标（转子 rpm）。
   KP 100：差 20° 以上就顶到限速。POS_SPEED_LIMIT 2000 转子 rpm ≈ 输出轴 55 rpm，30° 约 0.1 s；电机层默认限速只有 300。 */
#define SHOOT_FEEDER_POS_KP           100.0f
#define SHOOT_FEEDER_POS_KI           0.0f
#define SHOOT_FEEDER_POS_KD           0.0f
#define SHOOT_FEEDER_POS_SPEED_LIMIT  2000.0f

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
