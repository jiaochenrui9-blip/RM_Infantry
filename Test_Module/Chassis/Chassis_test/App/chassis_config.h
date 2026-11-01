#ifndef CHASSIS_CONFIG_H
#define CHASSIS_CONFIG_H

/*
 * 底盘参数，只有 Infantry_Chassis.c 用。轮子编号：俯视顺时针从前左开始，0 前左、1 前右、2 后右、3 后左。
 * 标“待标定 / 待调”的值上板实测，记到 notes.md 的标定记录里。
 */

/* C620 电调 ID（用 SET 键设）。只能是 1~4：都走 0x200 控制帧。 */
#define CHASSIS_FL_ESC_ID          1U
#define CHASSIS_FR_ESC_ID          2U
#define CHASSIS_BR_ESC_ID          3U
#define CHASSIS_BL_ESC_ID          4U

/* 方向（1 或 -1）：手转轮子、让它往前滚时 speed_rpm 应为正，否则改 -1。
   按果园机器人的实测推出来是左侧 1、右侧 -1，待核对 */
#define CHASSIS_FL_DIRECTION       1
#define CHASSIS_FR_DIRECTION       (-1)
#define CHASSIS_BR_DIRECTION       (-1)
#define CHASSIS_BL_DIRECTION       1

/* 电流限幅：C620 ±16384 对应 ±20 A。先给小，调好再放开（果园机器人最后用 8000）。 */
#define CHASSIS_CURRENT_LIMIT      3000.0f

/* 速度环用电机层自带的：误差是转子 rpm，输出是电流，dt 1 ms。
   积分项被 motor_control 限在 1000（约 1.2 A），上坡不够时见 notes。起点是果园机器人的实测值，待调 */
#define CHASSIS_SPEED_KP           5.0f
#define CHASSIS_SPEED_KI           20.0f
#define CHASSIS_SPEED_KD           0.0f

/* 尺寸（米）。轮型只体现在下面两个系数上，代码不用改，待测：
   麦轮（从上往下看辊子连成 X 形）：CHASSIS_XY_FACTOR = 1，CHASSIS_ROTATE_RADIUS_M = (左右轮中心距 + 前后轮中心距) / 2；
   全向轮（4 个装在四角、轮轴指向车中心）：CHASSIS_XY_FACTOR = 0.7071（√2/2），CHASSIS_ROTATE_RADIUS_M = 轮心到底盘中心的距离。 */
#define CHASSIS_WHEEL_RADIUS_M     0.076f
#define CHASSIS_XY_FACTOR          1.0f
#define CHASSIS_ROTATE_RADIUS_M    0.40f

/* 转子转速上限：任一个轮子超过就 4 个一起等比例缩小（M3508 24 V 空载转子约 9256 rpm，果园机器人用 8000） */
#define CHASSIS_WHEEL_MAX_RPM      8000.0f

/* 跟随云台：wz = Kp × 云台相对角（度/秒 每 度），再限幅。只用 P。
   主机模型（加上 wz 斜坡）里 Kp 5 以内不超调，8 开始超调约 1.7°。待调 */
#define CHASSIS_FOLLOW_KP          3.0f
#define CHASSIS_FOLLOW_MAX_DPS     180.0f

/* 小陀螺转速，度/秒，正为俯视逆时针。先给小，待调 */
#define CHASSIS_SPIN_DPS           180.0f

/* 加速度限制：平移 m/s²（按速度向量的长度限），旋转 度/s²。待调 */
#define CHASSIS_ACCEL_MPS2         3.0f
#define CHASSIS_WZ_ACCEL_DPS2      720.0f

#endif /* CHASSIS_CONFIG_H */
