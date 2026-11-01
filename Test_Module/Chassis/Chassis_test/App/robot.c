#include "robot.h"

#include "main.h"
#include "can.h"

#include "can_bus.h"
#include "dji_motor_manager.h"

/* 每路 CAN 只能有一个 manager，这路上的所有电机都注册进来（见根目录 notes.md 第 2 节）。 */
static DJI_MotorManager_t robot_can1;
static DJI_MotorManager_t robot_can2;

static uint32_t robot_last_tick;

void Robot_Init(void)
{
    if ((DJI_MotorManager_Init(&robot_can1, &hcan1) != HAL_OK) ||
        (DJI_MotorManager_Init(&robot_can2, &hcan2) != HAL_OK))
    {
        Error_Handler();
    }

    /* 电机要在 CAN 启动前注册：底盘模块的 Init 加在这里。
       底盘电机默认接 CAN2（根目录 notes.md 第 4 节），按实际接线改传进去的 manager。 */

    if ((DJI_MotorManager_Start(&robot_can1) != HAL_OK) ||
        (DJI_MotorManager_Start(&robot_can2) != HAL_OK))
    {
        Error_Handler();
    }

    robot_last_tick = HAL_GetTick();
}

void Robot_Loop(void)
{
    uint32_t now = HAL_GetTick();

    if (now == robot_last_tick)
    {
        return;
    }
    robot_last_tick = now;

    /* 1. 指令、2. 控制：底盘模块写好后加在这里（测试工程用调试变量下指令） */

    /* 3. 发送：所有模块算完后，每路 CAN 每周期只发一次 */
    (void)DJI_MotorManager_SendAll(&robot_can1);
    (void)DJI_MotorManager_SendAll(&robot_can2);
}

/* HAL 的 CAN 接收回调：每次读一帧，按总线交给对应的 manager。
   同一路上以后有别的设备（超级电容、板间通信），把这一帧也交给它们的驱动。 */
void HAL_CAN_RxFifo0MsgPendingCallback(CAN_HandleTypeDef *hcan)
{
    CAN_RxHeaderTypeDef header;
    uint8_t data[8];

    if (CAN_Bus_ReadFifo0(hcan, &header, data) != HAL_OK)
    {
        return;
    }

    if (hcan == &hcan1)
    {
        DJI_MotorManager_ProcessRxFrame(&robot_can1, &header, data);
    }
    else if (hcan == &hcan2)
    {
        DJI_MotorManager_ProcessRxFrame(&robot_can2, &header, data);
    }
}
