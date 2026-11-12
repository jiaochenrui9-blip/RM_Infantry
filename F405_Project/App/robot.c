#include "robot.h"

#include "main.h"
#include "can.h"

#include "can_bus.h"
#include "dji_motor_manager.h"
#include "Infantry_Chassis.h"
#include "chassis_link.h"

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

    /* 电机要在 CAN 启动前注册。这块板的底盘电机接 CAN1（CN7 / CN15），和 C 板的 Chassis_test 相反（根目录 notes.md 第 4 节）。 */
    if (Infantry_Chassis_Init(&robot_can1) != HAL_OK)
    {
        Error_Handler();
    }

    /* 云台主控的指令从两块板之间那路 CAN 来，这块板是 CAN2（CN8 / CN9）。 */
    if (ChassisLink_Init(&hcan2) != HAL_OK)
    {
        Error_Handler();
    }

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
    ChassisCmd_t cmd;

    if (now == robot_last_tick)
    {
        return;
    }
    robot_last_tick = now;

    /* 1. 指令：云台主控经板间 CAN 发来，超时给 RELAX */
    ChassisLink_GetCmd(now, &cmd);
    Chassis_SetCmd(&cmd);

    /* 2. 控制 */
    Chassis_Update(now);

    /* 3. 发送：所有模块算完后，每路 CAN 每周期只发一次 */
    (void)DJI_MotorManager_SendAll(&robot_can1);
    (void)DJI_MotorManager_SendAll(&robot_can2);
}

/* HAL 的 CAN 接收回调：每次读一帧，交给这路上的每个驱动，各自按 ID 只处理自己的帧。
   CAN1 上是底盘电机，CAN2 上是板间指令（这路上还有云台的两个 RS05，是扩展帧，过滤器不收）。 */
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
        ChassisLink_ProcessRxFrame(&header, data);
        DJI_MotorManager_ProcessRxFrame(&robot_can2, &header, data);
    }
}
