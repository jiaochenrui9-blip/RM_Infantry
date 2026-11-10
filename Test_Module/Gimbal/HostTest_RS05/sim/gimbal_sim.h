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
 * RS05 版云台（Gimbal_test_RS05）的主机仿真。接口和 ../HostTest（GM6020 版）的 gimbal_sim.h 一样，
 * 场景用的是同一份 ../HostTest/test_gimbal.c；只有电机模型换成按 RS05 手册行为写的（见 gimbal_sim.c 开头）。
 * 被测的是真代码：gimbal.c、gimbal_test_cmd.c、RS05 驱动（使能序列、停止帧、帧打包解析、掉线判断）、PID、user_math，
 * 以及板间通信 chassis_link.c。只有 HAL 和 CAN 收发是假的（stub/ 和 ../HostTest/stub/main.h）。
 *
 * 角度约定和云台模块一样：度；yaw 俯视逆时针为正，pitch 抬头为正。
 * 模型参数（转动惯量、摩擦等）是假设值，只用来验证逻辑和符号，不代表真云台的手感，PID 参数不能照搬。
 */

typedef struct
{
    float rel_deg;          /* 云台相对底盘的角度（真值），RS05 的角度测的就是它 */
    float world_dps;        /* 云台对地角速度（真值），IMU 测的就是它 */
    float base_deg;         /* 底盘：yaw 轴是车头朝向，pitch 轴是底盘在 pitch 方向的倾角 */
    float base_dps;         /* 底盘角速度，由测试场景直接给定（底盘不受云台反作用） */
    float mech_min_deg;     /* 机械极限（相对底盘），撞上就停 */
    float mech_max_deg;
    int8_t phys_dir;        /* 电机实际方向：1 表示“RS05 角度变大 / 正 iq”对应约定的正方向。默认等于 config 里的方向 */
    uint8_t online;         /* 0：CAN 断了，电机收不到指令、也不回反馈（电机还通着电，canTimeout 到了自己进 Reset） */
    uint8_t held;           /* 1：被手扶住，相对底盘不动（电机怎么出力都转不动） */
    int16_t cmd_current;    /* 电机这 1 ms 实际输出的 iq，mA（没在电流模式运行时为 0；方向是电机自己的） */
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

/* 重新初始化：模型回到默认状态（电机下一个周期才上电），重新注册电机、Gimbal_Init，gimbal_test 清零（RELAX）。
   之后先改 sim 里的初始状态，再 Sim_Run：电机按那时的位置上电，云台读到第一帧反馈。 */
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
