#ifndef GIMBAL_H
#define GIMBAL_H

#include "rs05.h"
#include "ins.h"                /* 只用 InsFeedback_t 这个类型，不调 ins 的函数 */

/*
 * 云台模块（v2：编码器 / IMU 定角度）。合并工程时整个 gimbal/ 文件夹直接搬走。
 * RS05 版：和 Gimbal_test 的 gimbal 只差电机这一层（GM6020 → 灵足 RS05 电流模式），模式、控制、接口都一样。
 * 模块不关心指令从哪来，只认下面的结构体；IMU 数据由 robot.c 每周期传进来。
 * 角度单位：度。正方向：yaw 俯视逆时针为正，pitch 抬头为正。
 */

typedef enum
{
    GIMBAL_MODE_RELAX = 0,      /* 无力：输出 0。上电默认，电机掉线、IMU 失效时被强制进入 */
    GIMBAL_MODE_ENCODER,        /* 编码器角度闭环：目标角相对底盘正前方 */
    GIMBAL_MODE_IMU             /* IMU 角度闭环：目标角相对地面，pitch 以水平为 0，yaw 以上电时的朝向为 0 */
} GimbalMode_e;

typedef struct
{
    GimbalMode_e mode;
    float yaw_deg;              /* 目标角，坐标系跟着 mode 走 */
    float pitch_deg;
} GimbalCmd_t;

typedef struct
{
    GimbalMode_e mode;          /* 实际模式，可能和指令不同 */
    float yaw_rel_deg;          /* 编码器角（相对底盘），任何模式下都有，底盘跟随用它 */
    float pitch_rel_deg;
    float yaw_imu_deg;          /* IMU 角（相对地面），指令源切到 IMU 模式时用它同步目标 */
    float pitch_imu_deg;
} GimbalFeedback_t;

/* 把两个 RS05 注册到对应总线的 manager，必须在 RS05_Manager_Start() 之前调用。 */
HAL_StatusTypeDef Gimbal_Init(RS05_Manager_t *yaw_bus, RS05_Manager_t *pitch_bus);
/* 每 1 ms 在 Gimbal_GetFeedback、Gimbal_Update 之前调，传 INS_GetFeedback 的结果。
   valid 为 0 时不许进 IMU 模式；IMU 模式中变 0 → 无力并锁住，指令给一次 RELAX 才解除。 */
void Gimbal_SetImu(const InsFeedback_t *imu);
void Gimbal_SetCmd(const GimbalCmd_t *cmd);
/* 每 1 ms 调一次：读反馈 → 定模式 → 串级 PID → 设置电流。发 CAN 帧由 robot.c 统一做。 */
void Gimbal_Update(uint32_t now_tick);
void Gimbal_GetFeedback(GimbalFeedback_t *feedback);

#endif /* GIMBAL_H */
