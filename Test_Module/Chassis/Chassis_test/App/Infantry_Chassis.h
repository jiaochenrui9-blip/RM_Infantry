//
// Created by game on 2026/11/1.
//

#ifndef CHASSIS_TEST_INFANTRY_H
#define CHASSIS_TEST_INFANTRY_H

#include "dji_motor_manager.h"
#include "chassis_link.h"       /* 只用 ChassisMode_e、ChassisCmd_t：两块板共用，所以定义在板间通信里；不调它的函数 */

/* 把 4 个 M3508 注册到这路 CAN 的 manager，必须在 DJI_MotorManager_Start() 之前调用。
   注册失败（ID 重复、ID 不在 1~4）返回 HAL_ERROR。 */
HAL_StatusTypeDef Infantry_Chassis_Init(DJI_MotorManager_t *manager);
/* 每 1 ms 在 Chassis_Update 之前调，传这一周期的指令（双板时来自 ChassisLink_GetCmd）。非法模式当 RELAX。 */
void Chassis_SetCmd(const ChassisCmd_t *cmd);
/* 每 1 ms 调一次：定模式 → 算 wz → 斜坡 → 坐标变换 → 逆运动学 → 电机层速度环。发 CAN 帧由 robot.c 统一做。
   运动模式下任一电机掉线（100 ms）→ 无力并锁住，指令给一次 RELAX 才解除。 */
void Chassis_Update(uint32_t now_tick);

#endif //CHASSIS_TEST_INFANTRY_H
