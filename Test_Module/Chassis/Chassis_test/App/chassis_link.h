#ifndef CHASSIS_LINK_H
#define CHASSIS_LINK_H

#include "Infantry.h"

/*
 * 板间通信，底盘板这一侧：接收云台主控发来的底盘指令。
 * 帧格式（云台板发送时按这个打包）：标准数据帧 0x110，DLC 8，每 1 ms 一帧，高字节在前（和大疆电机帧一样）
 *   DATA[0]     mode（ChassisMode_e）
 *   DATA[1]     保留，发 0
 *   DATA[2..3]  vx，int16，mm/s
 *   DATA[4..5]  vy，int16，mm/s
 *   DATA[6..7]  gimbal_yaw_rel，int16，0.01°
 */
#define CHASSIS_LINK_CMD_ID         0x110U
#define CHASSIS_LINK_TIMEOUT_MS     50U     /* 超过这么久没收到新帧就当 RELAX */

/* 给 0x110 配一个过滤器组：CAN1 用组 2、CAN2 用组 16（组的分配见 can_bus.h）。CAN 启动前后调都行。 */
HAL_StatusTypeDef ChassisLink_Init(CAN_HandleTypeDef *hcan);
/* 在 CAN 接收回调（中断）里调：这路上读到的每一帧都交给它，它只认 0x110，只存原始数据和时间。 */
void ChassisLink_ProcessRxFrame(const CAN_RxHeaderTypeDef *header, const uint8_t data[8]);
/* 主循环每 1 ms 调：解出最新一帧；超时（包括上电后还没收到过）给 RELAX、速度 0。 */
void ChassisLink_GetCmd(uint32_t now_tick, ChassisCmd_t *cmd);

#endif /* CHASSIS_LINK_H */
