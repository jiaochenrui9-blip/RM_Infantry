#ifndef GIMBAL_CONFIG_H
#define GIMBAL_CONFIG_H

/*
 * 云台参数，只有 gimbal.c 用。正方向：yaw 俯视逆时针为正，pitch 抬头为正。
 * 标“待标定 / 待调”的值装好云台后实测，记到 notes.md 的标定记录里。
 */

/* GM6020 拨码设的电调 ID（1~7）。电机接在哪路 CAN 在 robot.c 里选。 */
#define GIMBAL_YAW_ESC_ID              1U
#define GIMBAL_PITCH_ESC_ID            2U

/* 方向（1 或 -1）：手转云台往正方向转时，角度和角速度都应变大，否则改成 -1。待标定 */
#define GIMBAL_YAW_DIRECTION           1
#define GIMBAL_PITCH_DIRECTION         1

/* 中值：云台朝正前方 / 水平时的编码器读数（0~8191）。待标定 */
#define GIMBAL_YAW_ECD_OFFSET          0U
#define GIMBAL_PITCH_ECD_OFFSET        0U

/* 软限位（度，编码器角，编码器 / IMU 两个模式都按它限），在机械极限基础上往里留几度。
   yaw 能 360° 转时写 -180 / 180 就是不限位。IMU 模式下底盘带着云台顶在限位上转时，
   云台会超出软限位约“底盘角速度 / 角度环 Kp”度（Kp 8、30°/s 时约 4°），余量按这个留。待标定 */
#define GIMBAL_YAW_MIN_DEG             (-90.0f)
#define GIMBAL_YAW_MAX_DEG             (90.0f)
#define GIMBAL_PITCH_MIN_DEG           (-15.0f)
#define GIMBAL_PITCH_MAX_DEG           (20.0f)

/* 电流限幅：GM6020 电流控制 ±16384 对应 ±3 A。先给小，调好再放开。 */
#define GIMBAL_CURRENT_LIMIT           5000.0f
/* 角度环输出（目标角速度）限幅，度/秒 */
#define GIMBAL_SPEED_LIMIT_DPS         200.0f
/* 目标斜坡：每毫秒最多变化多少度，0.1 即 100°/s。看阶跃响应时可以临时调大。 */
#define GIMBAL_RAMP_DEG_PER_MS         0.1f
/* 速度前馈系数：斜坡速度乘它加到速度环目标上（再按 GIMBAL_SPEED_LIMIT_DPS 限幅）。
   1 为完整前馈，0 为关闭；对比有无前馈时改这里。 */
#define GIMBAL_SPEED_FF_RATIO          1.0f
/* pitch 重力前馈：pitch 水平时抵消重力需要的电流，按 cos(pitch) 缩放。先给 0。 */
#define GIMBAL_PITCH_GRAVITY_FF        0.0f

/* PID，全部待调。角度环：误差 deg → 目标角速度 dps；速度环：误差 dps → 电流。 */
#define GIMBAL_YAW_ANGLE_KP            8.0f
#define GIMBAL_YAW_ANGLE_KI            0.0f
#define GIMBAL_YAW_ANGLE_KD            0.0f
#define GIMBAL_YAW_SPEED_KP            20.0f
#define GIMBAL_YAW_SPEED_KI            0.0f
#define GIMBAL_YAW_SPEED_KD            0.0f

#define GIMBAL_PITCH_ANGLE_KP          8.0f
#define GIMBAL_PITCH_ANGLE_KI          0.0f
#define GIMBAL_PITCH_ANGLE_KD          0.0f
#define GIMBAL_PITCH_SPEED_KP          20.0f
#define GIMBAL_PITCH_SPEED_KI          0.0f
#define GIMBAL_PITCH_SPEED_KD          0.0f

#endif /* GIMBAL_CONFIG_H */
