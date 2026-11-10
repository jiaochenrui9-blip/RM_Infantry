#include "robot.h"

#include "main.h"
#include "can.h"

#include "can_bus.h"
#include "dji_motor_manager.h"
#include "dji_m2006.h"
#include "motor_control.h"

/*
 * 上板测试（2026-11-09）：CAN2 上一个 M2006（C610 电调 ID 1），速度环转到 500 转子 rpm。
 * 底盘代码暂时去掉了，原版在 Test_Module/Chassis/Chassis_test/App，恢复办法见本工程 notes.md。
 */
#define M2006_TEST_ESC_ID           1U
#define M2006_TEST_TARGET_RPM       5000     /* 转子 rpm；减速比 36，输出轴约 13.9 rpm */
#define M2006_TEST_CURRENT_LIMIT    2000.0f /* C610：±10000 对应 ±10 A，先给 2 A */
/* 误差是转子 rpm、输出是电流、dt 1 ms。没调过：DJI 驱动工程 M3508 测试用的起点 */
#define M2006_TEST_SPEED_KP         3.0f
#define M2006_TEST_SPEED_KI         20.0f
#define M2006_TEST_SPEED_KD         0.0f

/* 每路 CAN 只能有一个 manager（根目录 notes.md 第 2 节）。现在只用 CAN2。 */
static DJI_MotorManager_t robot_can2;
static DJI_Motor_t robot_m2006;

static uint32_t robot_last_tick;

void Robot_Init(void)
{
    if (DJI_MotorManager_Init(&robot_can2, &hcan2) != HAL_OK)
    {
        Error_Handler();
    }

    /* 电机要在 CAN 启动前注册。ID 1~4 走 0x200，槽位 = ID - 1；位置环不用，参数给 0。 */
    if ((DJI_M2006_Register(&robot_can2, &robot_m2006, M2006_TEST_ESC_ID,
                            DJI_MOTOR_CONTROL_ID_1_4, M2006_TEST_ESC_ID - 1U) != HAL_OK) ||
        (MotorControl_Init(&robot_m2006, M2006_TEST_CURRENT_LIMIT, 1,
                           M2006_TEST_SPEED_KP, M2006_TEST_SPEED_KI, M2006_TEST_SPEED_KD,
                           0.0f, 0.0f, 0.0f) != HAL_OK))
    {
        Error_Handler();
    }

    /* SetMode 会把目标清零，所以先切模式再给目标。
       目标就存在 robot_m2006.speed.target_rpm 里，调试器里可以直接改。 */
    MotorControl_SetMode(&robot_m2006, MOTOR_MODE_SPEED);
    MotorControl_SetTargetSpeed(&robot_m2006, M2006_TEST_TARGET_RPM);

    if (DJI_MotorManager_Start(&robot_can2) != HAL_OK)
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

    /* 速度环（100 ms 没收到反馈就输出 0），然后发 0x200 */
    MotorControl_Update(&robot_m2006, now);
    (void)DJI_MotorManager_SendAll(&robot_can2);
}

/* HAL 的 CAN 接收回调：每次读一帧。现在只启动了 CAN2，它的帧交给 CAN2 的 manager。 */
void HAL_CAN_RxFifo0MsgPendingCallback(CAN_HandleTypeDef *hcan)
{
    CAN_RxHeaderTypeDef header;
    uint8_t data[8];

    if (CAN_Bus_ReadFifo0(hcan, &header, data) != HAL_OK)
    {
        return;
    }

    if (hcan == &hcan2)
    {
        DJI_MotorManager_ProcessRxFrame(&robot_can2, &header, data);
    }
}
