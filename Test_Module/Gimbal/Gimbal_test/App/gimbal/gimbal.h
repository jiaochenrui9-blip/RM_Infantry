#ifndef GIMBAL_H
#define GIMBAL_H

#include "dji_motor_manager.h"

/*
 * 云台模块（v1：编码器定角度）。合并工程时整个 gimbal/ 文件夹直接搬走。
 * 模块不关心指令从哪来，只认下面两个结构体。
 * 角度单位：度。正方向：yaw 俯视逆时针为正，pitch 抬头为正。
 */

typedef enum
{
    GIMBAL_MODE_RELAX = 0,      /* 无力：输出 0。上电默认，电机掉线时被强制进入 */
    GIMBAL_MODE_ENCODER         /* 编码器角度闭环：角度相对底盘 */
} GimbalMode_e;

typedef struct
{
    GimbalMode_e mode;
    float yaw_deg;              /* 目标角，绝对值 */
    float pitch_deg;
} GimbalCmd_t;

typedef struct
{
    GimbalMode_e mode;          /* 实际模式，可能和指令不同 */
    float yaw_deg;              /* 当前角（相对底盘），底盘跟随云台要用 */
    float pitch_deg;
} GimbalFeedback_t;

/* 把两个 GM6020 注册到对应总线的 manager，必须在 DJI_MotorManager_Start() 之前调用。 */
HAL_StatusTypeDef Gimbal_Init(DJI_MotorManager_t *yaw_bus, DJI_MotorManager_t *pitch_bus);
void Gimbal_SetCmd(const GimbalCmd_t *cmd);
/* 每 1 ms 调一次：读反馈 → 定模式 → 串级 PID → 设置电流。发 CAN 帧由 robot.c 统一做。 */
void Gimbal_Update(uint32_t now_tick);
void Gimbal_GetFeedback(GimbalFeedback_t *feedback);

#endif /* GIMBAL_H */
