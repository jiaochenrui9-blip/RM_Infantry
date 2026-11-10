#ifndef GIMBAL_CONFIG_H
#define GIMBAL_CONFIG_H

/*
 * 云台参数，只有 gimbal.c 用。正方向：yaw 俯视逆时针为正，pitch 抬头为正。
 * 标“待标定 / 待调”的值装好云台后实测，记到 notes.md 的标定记录里。
 */

/* RS05 的 CAN ID（上位机里设，1~127，两个电机要不同）。电机接在哪路 CAN 在 robot.c 里选。 */
#define GIMBAL_YAW_CAN_ID              1U
#define GIMBAL_PITCH_CAN_ID            2U

/* 方向（1 或 -1）：手转云台往正方向转时，角度和角速度都应变大，否则改成 -1。
   yaw：RS05 正转是从输出轴那面看顺时针（2026-11-09 RS05_test 实测），输出轴朝上装时俯视逆时针为正要取 -1。
   pitch 还没装，待标定（电机在左侧 -1、右侧 1，推法见 RS05_test 的 notes） */
#define GIMBAL_YAW_DIRECTION           (-1)
#define GIMBAL_PITCH_DIRECTION         1

/* 中值：云台朝正前方 / 水平时 RS05 的角度读数（motor.position_rad，rad）。待标定
   RS05 上电后在输出轴一圈内是绝对的（果园机器人工程实测），和 GM6020 一样写死。读数在 0~2π 还是 −π~π、
   通电时数了几圈都没关系：差的是整圈，gimbal.c 把差值折到 ±180° 时就没了。 */
#define GIMBAL_YAW_ZERO_RAD            0.0f
#define GIMBAL_PITCH_ZERO_RAD          0.0f

/* 软限位（度，编码器角，编码器 / IMU 两个模式都按它限），在机械极限基础上往里留几度。
   yaw 能 360° 转时写 -180 / 180 就是不限位。IMU 模式下底盘带着云台顶在限位上转时，
   云台会超出软限位约“底盘角速度 / 角度环 Kp”度（Kp 8、30°/s 时约 4°），余量按这个留。待标定 */
#define GIMBAL_YAW_MIN_DEG             (-180.0f)
#define GIMBAL_YAW_MAX_DEG             (180.0f)  /* 360° 转时写 -180 / 180 就是不限位 */
#define GIMBAL_PITCH_MIN_DEG           (-15.0f)
#define GIMBAL_PITCH_MAX_DEG           (20.0f)

/* 电流限幅（A，RS05 的 iq）。约 0.67 N·m/A，1 A 和 GM6020 那版的 5000（约 0.68 N·m）差不多。
   先给小，调好再放开（RS05 额定 2.4 A、峰值 11 A）。 */
#define GIMBAL_CURRENT_LIMIT           1.0f
/* 角度环输出（目标角速度）限幅，度/秒 */
#define GIMBAL_SPEED_LIMIT_DPS         200.0f
/* 目标斜坡：每毫秒最多变化多少度，0.1 即 100°/s。看阶跃响应时可以临时调大。 */
#define GIMBAL_RAMP_DEG_PER_MS         0.1f
/* 速度前馈系数：斜坡速度乘它加到速度环目标上（再按 GIMBAL_SPEED_LIMIT_DPS 限幅）。
   1 为完整前馈，0 为关闭；对比有无前馈时改这里。 */
#define GIMBAL_SPEED_FF_RATIO          1.0f
/* pitch 重力前馈：pitch 水平时抵消重力需要的电流（A），按 cos(pitch) 缩放。先给 0。 */
#define GIMBAL_PITCH_GRAVITY_FF        0.0f
/* yaw 摩擦前馈：抵消 yaw 电机减速箱、轴承（上车后还有滑环）的动摩擦。摩擦跟着电机相对底盘的转速走；
   底盘转、云台对地不动时，这个转速就是 −底盘角速度，所以底盘带云台的那份摩擦也是它补。
   电流（A）= GIMBAL_YAW_FRICTION_FF × clamp(转速 / GIMBAL_YAW_FRICTION_BAND_DPS, −1, 1)
            + GIMBAL_YAW_VISCOUS_FF × 转速（°/s）。
   用 tools/yaw_friction.py 测（模式 1 匀速转，匀速段的电流就是摩擦），按测出来的 80% 左右给：
   补过头等于往运动方向推，低速时会来回找。粘住（相对转速为 0）时是静摩擦，补不了，靠速度环。先给 0（关）。 */
#define GIMBAL_YAW_FRICTION_FF         0.0f    /* 库仑项，A */
#define GIMBAL_YAW_VISCOUS_FF          0.0f    /* 粘滞项，A/(°/s) */
/* 库仑项在 ±这个转速以内线性过渡：静止时 RS05 转速读数有约 ±2.5°/s 的噪声，直接用 sgn 电流会来回跳。
   越窄，起转时补得越早，静止时电流也抖得越厉害。 */
#define GIMBAL_YAW_FRICTION_BAND_DPS   10.0f

/* PID。角度环：误差 deg → 目标角速度 dps；速度环：误差 dps → 电流（A）。
   yaw：2026-11-10 在桌上调的，RS05 空载（转盘上没东西）、电流限幅 1 A，装上云台负载后要重调。
   只有 P 时，减速箱的静摩擦让 yaw 停在离目标 0.5~1° 的地方，速度环加 I 才消掉；阶跃停下时冲过头 1~1.5°
   是电流限幅刹不住，不是 PID 的问题。记录见 notes.md「单电机：yaw 跟着 C 板转」的「调 PID」。
   pitch 还是占位值：速度环从 GM6020 那版（Kp 20）按同样的力矩换算来，× 3/16384 × 0.741 / 0.67 ≈ 0.0002。 */
#define GIMBAL_YAW_ANGLE_KP            12.0f
#define GIMBAL_YAW_ANGLE_KI            0.0f
#define GIMBAL_YAW_ANGLE_KD            0.0f
#define GIMBAL_YAW_SPEED_KP            0.016f
#define GIMBAL_YAW_SPEED_KI            0.2f
#define GIMBAL_YAW_SPEED_KD            0.0f

#define GIMBAL_PITCH_ANGLE_KP          8.0f
#define GIMBAL_PITCH_ANGLE_KI          0.0f
#define GIMBAL_PITCH_ANGLE_KD          0.0f
#define GIMBAL_PITCH_SPEED_KP          0.004f
#define GIMBAL_PITCH_SPEED_KI          0.0f
#define GIMBAL_PITCH_SPEED_KD          0.0f

#endif /* GIMBAL_CONFIG_H */
