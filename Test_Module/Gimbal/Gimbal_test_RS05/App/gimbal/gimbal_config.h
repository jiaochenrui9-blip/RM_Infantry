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
   yaw：RS05 正转是从输出轴那面看顺时针（2026-11-09 RS05_test 实测）。中间有齿轮也只看一件事：云台俯视逆时针转时，
   从输出轴那面看电机是逆时针（电机反转）就取 -1，顺时针就取 1。2026-11-11 用户确认：从输出轴那面看电机逆时针转时，
   云台俯视逆时针，所以取 -1。上车再在 RELAX 下手转确认：云台俯视逆时针转，yaw 角要变大。
   pitch：从输出轴那面看，曲柄顺时针转是抬头（2026-11-11 用户确认，四连杆从最下到最高点那一段），取 1；
   过了最高点方向反过来，只用单调的一段。两个都要装好以后在 RELAX 下手转确认。
   2026-11-12 pitch 机械改了结构以后，最下 → 最上编码器（−16.4 → 50°）和 IMU（−29.5 → 30°）一起变大，还是 1 */
#define GIMBAL_YAW_DIRECTION           (-1)
#define GIMBAL_PITCH_DIRECTION         1

/* 中值：云台朝正前方 / 水平时 RS05 的角度读数（motor.position_rad，rad）。
   RS05 上电后在输出轴一圈内是绝对的（果园机器人工程实测），和 GM6020 一样写死。读数在 0~2π 还是 −π~π、
   通电时数了几圈都没关系：差的是整圈，gimbal.c 把差值折到 ±180° 时就没了。
   2026-11-11 上车标的：中值还是 0 时，枪口朝车头 yaw 读 −91° 左右、炮管水平 pitch 读 −86°（用户读的角度列）。
   yaw 方向是 −1，所以电机读数是 +91°；pitch 方向 1，就是 −86°。要更准就用 gimbal_watch.py 的 once 读 position_rad 填。
   2026-11-12 pitch 机械改了结构，重标：中值还是 −1.5010（−86°）时，最下 / 水平 / 最上 编码器读 −16.4 / 18.5 / 50°，
   C 板 IMU 读 −29.5 / 0 / 30°（用户读的）。水平那个加上去：−86 + 18.5 = −67.5° */
#define GIMBAL_YAW_ZERO_RAD            1.5882f      /* +91° */
#define GIMBAL_PITCH_ZERO_RAD          (-1.1781f)   /* −67.5° */

/* 软限位（度，编码器角，编码器 / IMU 两个模式都按它限），在机械极限基础上往里留几度。
   yaw 能 360° 转时写 -180 / 180 就是不限位。IMU 模式下底盘带着云台顶在限位上转时，
   云台会超出软限位约“底盘角速度 / 角度环 Kp”度（Kp 8、30°/s 时约 4°），余量按这个留。
   pitch：2026-11-12 新结构，按上面的新中值最下 −34.9°、最上 +31.5°，各往里留 3° 多 */
#define GIMBAL_YAW_MIN_DEG             (-180.0f)
#define GIMBAL_YAW_MAX_DEG             (180.0f)  /* 360° 转时写 -180 / 180 就是不限位 */
#define GIMBAL_PITCH_MIN_DEG           (-31.0f)
#define GIMBAL_PITCH_MAX_DEG           (28.0f)

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
/* pitch 重力前馈：pitch 水平时抵消重力需要的电流（A），按 cos(pitch) 缩放。
   2026-11-12 新结构上量的（tools/pitch_gravity.py：模式 1 以 5°/s 来回扫，上下两趟电流的平均是重力）：
   按 FF × cos 拟合是 0.40。实测不是 cos 形：−10° 以下约 0.27~0.33、−6° 以上约 0.43~0.48，−8° ~ −5° 之间有个台阶
   （像机械上有东西被拉住或蹭到，还没查），填 0.40 时两边差的都在摩擦（约 ±0.15~0.25 A）以内，剩下的交给速度环积分。
   结构或配重改了要重测 */
#define GIMBAL_PITCH_GRAVITY_FF        0.40f
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
   2026-11-10 桌上 RS05 空载（转盘上没东西）、1 A 限幅调的 yaw 是 速度环 0.016 / 0.2、角度环 12，
   记录见 notes.md「单电机：yaw 跟着 C 板转」的「调 PID」。
   yaw：2026-11-12 车上（模式 2 为主）调的，速度环 0.013 / 0.12、角度环 8，记录见 notes.md「车上调 yaw」。
   模式 2 下速度环 Kp 被炮头和电机之间那段软连接的共振（约 6~7 Hz）限住；角度环 12 起步快、超调大（10° 时 5°），用户定 8。
   pitch：2026-11-12 车上调的，速度环 0.005 / 0.05、角度环 4，记录见 notes.md「车上调 pitch、量重力」。这是模式 2 下稳得住的那组：
   模式 2 的反馈在炮管上，四连杆的间隙（动起来炮管相对电机晃 6° 多）让速度环到 0.013 就振；
   模式 1 下 0.02 / 0.2 / 12 好得多（±3° 0.3~0.4 s 停稳），两个模式共用一组，只能按模式 2 取 */
#define GIMBAL_YAW_ANGLE_KP            8.0f
#define GIMBAL_YAW_ANGLE_KI            0.0f
#define GIMBAL_YAW_ANGLE_KD            0.0f
#define GIMBAL_YAW_SPEED_KP            0.013f
#define GIMBAL_YAW_SPEED_KI            0.12f
#define GIMBAL_YAW_SPEED_KD            0.0f

#define GIMBAL_PITCH_ANGLE_KP          4.0f
#define GIMBAL_PITCH_ANGLE_KI          0.0f
#define GIMBAL_PITCH_ANGLE_KD          0.0f
#define GIMBAL_PITCH_SPEED_KP          0.005f
#define GIMBAL_PITCH_SPEED_KI          0.05f
#define GIMBAL_PITCH_SPEED_KD          0.0f

#endif /* GIMBAL_CONFIG_H */
