#ifndef CHASSIS_CONFIG_H
#define CHASSIS_CONFIG_H

/*
 * 底盘参数，只有 Infantry_Chassis.c 用。轮子编号：俯视顺时针从前左开始，0 前左、1 前右、2 后右、3 后左。
 * 标“待标定 / 待调”的值上板实测，记到 notes.md 的标定记录里。
 */

/* C620 电调 ID（用 SET 键设）。只能是 1~4：都走 0x200 控制帧。
   用户 2026-11-11 在整车照片上标的：1 前右、2 后右、3 后左、4 前左（俯视顺时针，从前右开始） */
#define CHASSIS_FL_ESC_ID          4U
#define CHASSIS_FR_ESC_ID          1U
#define CHASSIS_BR_ESC_ID          2U
#define CHASSIS_BL_ESC_ID          3U

/* 方向（1 或 -1）：轮子逻辑正转 = 推车体俯视逆时针转（Infantry_Chassis.c 的驱动方向角）。
   手转轮子、让它往“推车体逆时针转”的方向滚（架空时就是轮子顶上往俯视逆时针走），speed_rpm 应为正，否则改符号。
   果园机器人实测：同样四角装、电机轴朝外，正 = 推车体顺时针时 4 个都是 1。这台车也是轴朝外的话就是 4 个 -1，待核对 */
#define CHASSIS_FL_DIRECTION       (-1)
#define CHASSIS_FR_DIRECTION       (-1)
#define CHASSIS_BR_DIRECTION       (-1)
#define CHASSIS_BL_DIRECTION       (-1)

/* 电流限幅：C620 ±16384 对应 ±20 A。先给小，调好再放开（果园机器人最后用 8000）。 */
#define CHASSIS_CURRENT_LIMIT      3000.0f

/* 速度环用电机层自带的：误差是转子 rpm，输出是电流，dt 1 ms。
   积分项被 motor_control 限在 1000（约 1.2 A），上坡不够时见 notes。起点是果园机器人的实测值，待调 */
#define CHASSIS_SPEED_KP           5.0f
#define CHASSIS_SPEED_KI           20.0f
#define CHASSIS_SPEED_KD           0.0f

/* 尺寸（米），全向轮装在四角、轮轴指向车中心。这台车都待量：
   轮子半径和用户 ELRS 工程 Omni_Remote.c 一样是 0.076；旋转半径 = 轮心到底盘中心的距离，先用那份代码里的 0.20 */
#define CHASSIS_WHEEL_RADIUS_M     0.076f
#define CHASSIS_ROTATE_RADIUS_M    0.20f

/* 转子转速上限：任一个轮子超过就 4 个一起等比例缩小（M3508 24 V 空载转子约 9256 rpm，果园机器人用 8000） */
#define CHASSIS_WHEEL_MAX_RPM      8000.0f

/* 跟随云台：wz = Kp × 云台相对角（度/秒 每 度），再限幅。只用 P。
   主机模型（加上 wz 斜坡）里 Kp 5 以内不超调，8 开始超调约 1.7°。待调 */
#define CHASSIS_FOLLOW_KP          3.0f
#define CHASSIS_FOLLOW_MAX_DPS     180.0f

/* 小陀螺转速，度/秒，正为俯视逆时针。待调。
   2026-11-11 第一次上车按每个轮子约 300 转子 rpm 给（用户说先给 300 RPM）：
   转子 rpm = R × ω × CHASSIS_MPS_TO_RPM，R 0.20 时 36°/s ≈ 303 rpm，10 s 转一圈 */
#define CHASSIS_SPIN_DPS           36.0f

/* 加速度限制：平移 m/s²（按速度向量的长度限），旋转 度/s²。待调 */
#define CHASSIS_ACCEL_MPS2         3.0f
#define CHASSIS_WZ_ACCEL_DPS2      720.0f

#endif /* CHASSIS_CONFIG_H */
