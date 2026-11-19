#ifndef POWER_MOTOR_CONFIG_H
#define POWER_MOTOR_CONFIG_H

/* C620 电调 ID；速度单位为电机转子 RPM，不是减速箱输出轴 RPM。 */
#define POWER_MOTOR_ID                  1U
#define POWER_MOTOR_TARGET_RPM       1000
#define POWER_MOTOR_DIRECTION           1
/* C620 指令计数，16384 对应 20 A；3000 约为 3.66 A。 */
#define POWER_MOTOR_CURRENT_LIMIT    3000.0f
/* 初始速度环参数，实际负载下仍需整定。 */
#define POWER_MOTOR_SPEED_KP            3.0f
#define POWER_MOTOR_SPEED_KI           10.0f
#define POWER_MOTOR_SPEED_KD            0.0f

#endif
