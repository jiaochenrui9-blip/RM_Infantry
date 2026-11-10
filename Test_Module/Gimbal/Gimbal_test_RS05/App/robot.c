#include "robot.h"

#include "main.h"
#include "can.h"
#include "spi.h"

#include "can_bus.h"
#include "chassis_link.h"
#include "rs05.h"
#include "BMI088.h"
#include "gimbal.h"
#include "gimbal_test_cmd.h"
#include "ins.h"
#include "tim.h"

/* 每路 CAN 只能有一个 manager，这路上的所有电机都注册进来（见根目录 notes.md 第 2 节）。
   两个 RS05 都在 CAN1，和板间通信同一路；这一版云台板上没有大疆电机，CAN2 不用、不启动
   （以后发射机构接 CAN2 时，在这里加它的 manager 并 Start）。 */
static RS05_Manager_t robot_can1;

static uint32_t robot_last_tick;

void Robot_Init(void)
{
    if (RS05_Manager_Init(&robot_can1, &hcan1) != HAL_OK)
    {
        Error_Handler();
    }

    /* 电机要在 CAN 启动前注册。云台电机接在哪路 CAN 按实际接线改这里。 */
    if (Gimbal_Init(&robot_can1, &robot_can1) != HAL_OK)
    {
        Error_Handler();
    }

    /* IMU：这里初始化 BMI088（约 85 ms）、启动加热 PWM。上电后先升温（要接 24 V），到温后约 5 s 标定零偏和初始姿态，
       标定期间整车别动。失败只影响 IMU 模式，原因看 ins.c 里的 ins_init_status（BMI088_Status_t）。 */
    (void)INS_Init(&hspi1, &htim10);

    /* 配 RS05 的过滤器（CAN1 组 1，只收类型 2 反馈）并启动 CAN1。0x110 也走 CAN1，只发不收，不用另外启动。 */
    if (RS05_Manager_Start(&robot_can1) != HAL_OK)
    {
        Error_Handler();
    }

    robot_last_tick = HAL_GetTick();
}

void Robot_Loop(void)
{
    uint32_t now = HAL_GetTick();
    InsFeedback_t imu;
    GimbalFeedback_t feedback;
    GimbalCmd_t gimbal_cmd;
    ChassisCmd_t chassis_cmd;

    if (now == robot_last_tick)
    {
        return;
    }
    robot_last_tick = now;

    /* 0. 传感器：IMU 采样和姿态解算，结果交给要用它的模块（现在只有云台的 IMU 模式） */
    INS_Update(now);
    INS_GetFeedback(&imu);
    Gimbal_SetImu(&imu);

    /* 1. 指令：测试工程用调试变量；合并后换成 robot_cmd */
    Gimbal_GetFeedback(&feedback);
    GimbalTestCmd_Update(now, &feedback, &gimbal_cmd, &chassis_cmd);
    Gimbal_SetCmd(&gimbal_cmd);

    /* 2. 控制：以后发射、底盘的 Update 也加在这里 */
    Gimbal_Update(now);

    /* 3. 发送：所有模块算完后，每路 CAN 每周期只发一次。RS05 的反馈只靠应答，
          所以每个电机每周期都要发一帧（RELAX 时是停止帧）。 */
    (void)RS05_Manager_SendAll(&robot_can1);

    /* 板间指令 0x110 发给底盘板（两块板之间那路，云台板默认 CAN1）。放在 SendAll 后面：
       邮箱不够时先让电机帧进邮箱。两个 RS05 加 0x110 正好占满 3 个发送邮箱，这一路别再加周期帧。
       CAN1 在 .ioc 里开了自动重发：RS05 的应答比这三帧都优先，仲裁输了要靠重发（根目录 notes.md 第 4 节）。 */
    (void)ChassisLink_SendCmd(&hcan1, &chassis_cmd);
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
        RS05_Manager_ProcessRxFrame(&robot_can1, &header, data);
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
