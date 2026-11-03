#ifndef GIMBAL_SIM_H
#define GIMBAL_SIM_H

#include <stdint.h>
#include <stdio.h>

#include "chassis_link.h"
#include "gimbal.h"
#include "gimbal_config.h"

/* yaw 软限位写满 360°（-180 / 180）表示装了滑环、不限位：仿真里也就没有 yaw 机械极限 */
#define SIM_YAW_UNLIMITED   ((GIMBAL_YAW_MAX_DEG - GIMBAL_YAW_MIN_DEG) >= 360.0f)

/*
 * 云台主机仿真：yaw、pitch 两个刚体轴 + 能转、能倾斜的底盘 + 理想 IMU。
 * 被测的是真代码：gimbal.c、gimbal_test_cmd.c、DJI 驱动（帧打包、方向、反馈解析、掉线保护）、PID、user_math，
 * 以及板间通信 chassis_link.c（云台板发 0x110，用底盘板那一侧的接收代码解出来，见 Sim_ChassisCmd）。
 * 只有 HAL 和 CAN 收发是假的（见 stub/），电机反馈按真实 CAN 帧格式送进 DJI_MotorManager_ProcessRxFrame。
 *
 * 角度约定和云台模块一样：度；yaw 俯视逆时针为正，pitch 抬头为正。
 * 模型参数（转动惯量、摩擦等）是假设值，只用来验证逻辑和符号，不代表真云台的手感，PID 参数不能照搬。
 */

typedef struct
{
    float rel_deg;          /* 云台相对底盘的角度（真值），编码器测的就是它 */
    float world_dps;        /* 云台对地角速度（真值），IMU 测的就是它 */
    float base_deg;         /* 底盘：yaw 轴是车头朝向，pitch 轴是底盘在 pitch 方向的倾角 */
    float base_dps;         /* 底盘角速度，由测试场景直接给定（底盘不受云台反作用） */
    float mech_min_deg;     /* 机械极限（相对底盘），撞上就停 */
    float mech_max_deg;
    int8_t phys_dir;        /* 电机实际方向：1 表示“编码器变大 / 正电流”对应约定的正方向。默认等于 config 里的方向，即方向已标对 */
    uint8_t online;         /* 0：不发反馈帧，模拟掉线 */
    uint8_t held;           /* 1：被手扶住，相对底盘不动（电机怎么出力都转不动） */
    int16_t cmd_current;    /* 最近一次控制帧里这个电机的电流（帧里的原始值，已乘过驱动里的方向） */
} SimAxis_t;

typedef struct
{
    SimAxis_t yaw;
    SimAxis_t pitch;
    float pitch_gravity_nm; /* pitch 水平时的重力矩（N·m），抬头为正，默认 0 */
    uint8_t imu_valid;      /* 传给云台的 valid */
    uint8_t imu_frozen;     /* 1：IMU 数据停在上一次的值（模拟采样卡住） */
    FILE *trace;            /* 非 NULL 时每个周期写一行 CSV，见 Sim_TraceHeader */
} Sim_t;

extern Sim_t sim;

/* 重新初始化：模型回到默认状态，重新注册电机、Gimbal_Init，gimbal_test 清零（RELAX）。
   之后先改 sim 里的初始状态，再 Sim_Run，让云台读到第一帧反馈。 */
void Sim_Reset(void);
/* 跑 ms 个控制周期，每个周期和 robot.c 的 Robot_Loop 顺序一样，然后模型往前走 1 ms */
void Sim_Run(uint32_t ms);

uint32_t Sim_Tick(void);
float Sim_YawWorld(void);       /* 云台对地 yaw，-180~180 */
float Sim_PitchWorld(void);
GimbalMode_e Sim_Mode(void);    /* 云台的实际模式（Gimbal_GetFeedback） */
/* 底盘板这一周期会收到的指令：云台板发出的 0x110 经底盘板的 ChassisLink_GetCmd 解出来（含 50 ms 超时） */
void Sim_ChassisCmd(ChassisCmd_t *cmd);
void Sim_TraceHeader(FILE *file);

#endif /* GIMBAL_SIM_H */
