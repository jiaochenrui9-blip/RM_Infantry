#ifndef CAR_SIM_H
#define CAR_SIM_H

#include <stdint.h>
#include <stdio.h>

#include "chassis_link.h"
#include "gimbal.h"
#include "gimbal_config.h"

/*
 * 整车主机仿真：两块板的真代码编在一起，接一个整车模型。
 *   云台板：gimbal.c + gimbal_test_cmd.c，每 1 ms 的顺序同 Gimbal_test/App/robot.c，最后 ChassisLink_SendCmd 发 0x110
 *   底盘板：Infantry_Chassis.c，顺序同 Chassis_test/App/robot.c，ChassisLink_GetCmd 收 0x110（比云台板晚 1 ms 用上）
 *   模型：两个 GM6020 云台轴（yaw 的底座就是底盘，跟着底盘转）+ 4 个 M3508 全向轮（四角装）推动的刚体底盘 + 理想 IMU
 * 只有 HAL 和 CAN 收发是假的（stub/ 和 HostTest 的 stub/main.h），电机反馈按真实帧格式送进 DJI 驱动。
 *
 * 底盘模型：轮子不打滑，轮速 = J·[vx, vy, wz]，4 个轮子的力经 Jᵀ 合成到同一个 3 自由度刚体上。
 * 4 个速度环各管各的，要是互相顶（内耗）在这里能看出来。质量、惯量、阻力是假设值，见 README。
 *
 * 约定和代码一致：度、m/s；yaw / 车头俯视逆时针为正；地面坐标 x 是上电时车头方向、y 是左。
 */

/* yaw 软限位写满 360°（-180 / 180）表示装了滑环、不限位：模型里也就没有 yaw 机械极限，小陀螺场景才有意义 */
#define CAR_YAW_UNLIMITED   ((GIMBAL_YAW_MAX_DEG - GIMBAL_YAW_MIN_DEG) >= 360.0f)

typedef struct
{
    float rel_deg;          /* 相对底盘的角度（真值），编码器测的就是它；yaw 不回绕 */
    float world_dps;        /* 对地角速度（真值），IMU 测的就是它 */
    float mech_min_deg;     /* 机械极限（相对底盘），撞上就停 */
    float mech_max_deg;
    int16_t cmd_current;    /* 最近一次控制帧里这个电机的电流原始值 */
    uint8_t online;         /* 0：不发反馈帧，模拟掉线 */
} CarGimbalAxis_t;

typedef struct
{
    float vx_mps;           /* 底盘系速度：沿车头为 x、车头左边为 y */
    float vy_mps;
    float wz_dps;           /* 俯视逆时针为正 */
    float heading_deg;      /* 车头对地朝向，不回绕 */
    float x_m;              /* 对地位置 */
    float y_m;
    int16_t wheel_current[4];   /* 0x200 帧里 4 个轮子的电流原始值，下标同 Infantry_Chassis.c：0 前左、1 前右、2 后右、3 后左 */
    uint8_t wheel_online[4];    /* 0：这个轮子不发反馈帧，模拟掉线 */
} CarChassis_t;

typedef struct
{
    CarGimbalAxis_t yaw;
    CarGimbalAxis_t pitch;
    CarChassis_t chassis;
    uint8_t imu_valid;      /* 传给云台的 valid，默认 1 */
    uint8_t link_on;        /* 0：云台板不发 0x110，模拟板间断线；默认 1 */
    FILE *trace;            /* 非 NULL 时每个周期写一行 CSV，见 Car_TraceHeader */
} Car_t;

extern Car_t car;

/* 重新初始化：模型回到原点、静止，gimbal_test 清零（云台、底盘都 RELAX），两块板重新 Init，
   然后在 RELAX 下跑 100 ms（电机上线、清掉上一个场景留下的锁定）。 */
void Car_Reset(void);
/* 跑 ms 个 1 ms 周期：电机反馈 → 底盘板一周期 → 云台板一周期 → 模型往前走 1 ms */
void Car_Run(uint32_t ms);

uint32_t Car_Tick(void);
float Car_GimbalWorldYaw(void);     /* 云台对地 yaw，-180~180 */
float Car_Heading(void);            /* 车头对地朝向，-180~180 */
float Car_Speed(void);              /* 对地速度大小 m/s */
float Car_VelocityDir(void);        /* 对地速度方向，-180~180；速度为 0 时没有意义 */
GimbalMode_e Car_GimbalMode(void);  /* 云台实际模式 */
void Car_ChassisRx(ChassisCmd_t *cmd);  /* 底盘板最近一个周期从 ChassisLink_GetCmd 拿到的指令 */
int Car_MaxWheelCurrent(void);      /* 4 个轮子电流原始值的最大绝对值 */
/* 内耗：转着的轮子里，有的出力往前推、有的出力往后拽，返回 1 */
uint8_t Car_WheelsFighting(void);
void Car_TraceHeader(FILE *file);

#endif /* CAR_SIM_H */
