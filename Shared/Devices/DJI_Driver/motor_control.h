#ifndef MOTOR_CONTROL_H
#define MOTOR_CONTROL_H

#include "dji_motor.h"

HAL_StatusTypeDef MotorControl_Init(DJI_Motor_t *motor, float current_limit,
                                    int8_t motor_direction,
                                    float speed_kp, float speed_ki, float speed_kd,
                                    float pos_kp,float pos_ki,float pos_kd);

/* 位置模式：目标是输出轴角度（度，逻辑方向），相对切进位置模式那一刻的位置（SetMode 时记零点）。 */
void MotorControl_SetTargetDegree(DJI_Motor_t *motor, float target_degree);
/* 当前输出轴角度，相对同一个零点；还没进过位置模式时返回 0。 */
float MotorControl_GetDegree(const DJI_Motor_t *motor);
/* 位置环输出（给速度环的目标，转子 rpm）的限幅，默认 300（MOTOR_CONTROL_POSITION_SPEED_LIMIT）。在 Init 之后调。 */
void MotorControl_SetPositionSpeedLimit(DJI_Motor_t *motor, float limit_rpm);

void MotorControl_SetTargetSpeed(DJI_Motor_t *motor, int16_t target_speed);

void MotorControl_Update(DJI_Motor_t *motor, uint32_t now_tick);
void MotorControl_SetMode(DJI_Motor_t *motor, MotorControlMode_e mode);

#endif /* MOTOR_CONTROL_H */
