//
// Created by game on 2026/11/1.
//

#ifndef CHASSIS_TEST_INFANTRY_H
#define CHASSIS_TEST_INFANTRY_H

#include "dji_motor_manager.h"

typedef enum
{
    CHASSIS_MODE_RELAX = 0,     /* 无力：上电默认；板间通信超时、电机掉线时也是这个 */
    CHASSIS_MODE_NO_FOLLOW,     /* 不跟随：底盘不转，只按云台朝向平移 */
    CHASSIS_MODE_FOLLOW,        /* 跟随云台：底盘转到和云台同向 */
    CHASSIS_MODE_SPIN           /* 小陀螺：底盘按固定角速度一直转，平移仍按云台朝向 */
} ChassisMode_e;                /* 数值也是板间帧的内容，两块板要一致，别随便调顺序 */

typedef struct
{
    ChassisMode_e mode;         /*云台主控配置的底盘模式*/
    float vx_mps;               /* 平移速度，在云台朝向的坐标系里：沿枪口为正 */
    float vy_mps;               /* 枪口左边为正 */
    float gimbal_yaw_rel_deg;   /* 云台相对底盘正前方的角度，来自云台的 yaw_rel_deg */
} ChassisCmd_t;

/* 把 4 个 M3508 注册到这路 CAN 的 manager，必须在 DJI_MotorManager_Start() 之前调用。
   注册失败（ID 重复、ID 不在 1~4）返回 HAL_ERROR。 */
HAL_StatusTypeDef Infantry_Chassis_Init(DJI_MotorManager_t *manager);
/* 每 1 ms 在 Chassis_Update 之前调，传这一周期的指令（双板时来自 ChassisLink_GetCmd）。非法模式当 RELAX。 */
void Chassis_SetCmd(const ChassisCmd_t *cmd);
/* 每 1 ms 调一次：定模式 → 算 wz → 斜坡 → 坐标变换 → 逆运动学 → 电机层速度环。发 CAN 帧由 robot.c 统一做。
   运动模式下任一电机掉线（100 ms）→ 无力并锁住，指令给一次 RELAX 才解除。 */
void Chassis_Update(uint32_t now_tick);

#endif //CHASSIS_TEST_INFANTRY_H
