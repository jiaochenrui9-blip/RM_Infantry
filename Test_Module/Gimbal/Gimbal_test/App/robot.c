#include "robot.h"

#include "main.h"
#include "can.h"
#include "spi.h"

#include "can_bus.h"
#include "dji_motor_manager.h"
#include "BMI088.h"
#include "gimbal.h"
#include "gimbal_test_cmd.h"
#include "ins.h"

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

    /* 电机要在 CAN 启动前注册。云台电机接在哪路 CAN 按实际接线改这里。 */
    if (Gimbal_Init(&robot_can1, &robot_can1) != HAL_OK)
    {
        Error_Handler();
    }

    /* IMU：这里只初始化 BMI088（约 85 ms），零偏和初始姿态在上电后约 1 s 内标定，期间整车别动。
       失败只影响 IMU 模式，原因看 ins.c 里的 ins_init_status（BMI088_Status_t）。 */
    (void)INS_Init(&hspi1);

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
    GimbalFeedback_t feedback;
    GimbalCmd_t cmd;

    if (now == robot_last_tick)
    {
        return;
    }
    robot_last_tick = now;

    /* 0. 传感器：IMU 采样和姿态解算。结果现在只在调试器里看 ins_feedback，v2 第 2 步再喂给云台 */
    INS_Update(now);

    /* 1. 指令：测试工程用调试变量；合并后换成 robot_cmd */
    Gimbal_GetFeedback(&feedback);
    GimbalTestCmd_Update(now, &feedback, &cmd);
    Gimbal_SetCmd(&cmd);

    /* 2. 控制：以后发射、底盘的 Update 也加在这里 */
    Gimbal_Update(now);

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

/* HAL 的 SPI 回调：BMI088 的 DMA 读取靠它往下走（读完加速度计接着读陀螺仪，读完才置 ready），不转就永远没有样本。 */
void HAL_SPI_TxRxCpltCallback(SPI_HandleTypeDef *hspi)
{
    BMI088_SPI_TxRxCpltCallback(hspi);
}

void HAL_SPI_ErrorCallback(SPI_HandleTypeDef *hspi)
{
    BMI088_SPI_ErrorCallback(hspi);
}
