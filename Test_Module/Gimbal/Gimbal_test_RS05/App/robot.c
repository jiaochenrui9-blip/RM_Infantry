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
#include "shoot.h"
#include "shoot_test_cmd.h"
#include "ins.h"
#include "tim.h"

/* 每路 CAN 只能有一个 manager，这路上的所有电机都注册进来（见根目录 notes.md 第 2 节）。
   两个 RS05 和板间通信使用 CAN1；发射机构的三个大疆电机共用 CAN2 的一个 manager。 */
static RS05_Manager_t robot_can1;
static DJI_MotorManager_t robot_can2;

/* 离上个 IMU 样本超过这么久（us），Robot_Loop 改成按 tick 跑。取 1.5 个样本间隔，离正常的间隔（1 ms 左右，加上中断延迟）
   和丢了一个样本的间隔（2 ms 左右）都远。取 2 ms 的话，丢一个样本时会正好在下一个样本 DMA 读的那十几 us 里按 tick 跑一次，
   SPI 正忙，那一次要是轮到读温度就会失败 */
#define ROBOT_SAMPLE_TIMEOUT_US  1500U

static uint32_t robot_last_tick;

/* TIM2 的计数：1 MHz，单位 us；32 位约 71.6 min 回绕一次，两个时刻按 uint32_t 相减不受回绕影响 */
static uint32_t Robot_NowUs(void)
{
    return __HAL_TIM_GET_COUNTER(&htim2);
}

void Robot_Init(void)
{
    /* TIM2 只当时间戳用（不开中断），要在 INS_Init 打开 BMI088 的 DRDY 之前启动。 */
    if (HAL_TIM_Base_Start(&htim2) != HAL_OK)
    {
        Error_Handler();
    }

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

    /* 发射电机先注册和初始化，再启动 CAN2。 */
    if ((DJI_MotorManager_Init(&robot_can2, &hcan2) != HAL_OK) ||
        (Shoot_Init(&robot_can2) != HAL_OK))
    {
        Error_Handler();
    }

    /* 配 RS05 的过滤器（CAN1 组 1，只收类型 2 反馈）并启动 CAN1。0x110 也走 CAN1，只发不收，不用另外启动。 */
    if (RS05_Manager_Start(&robot_can1) != HAL_OK)
    {
        Error_Handler();
    }

    if (DJI_MotorManager_Start(&robot_can2) != HAL_OK)
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
    ShootCmd_t shoot_cmd;

    /* 跟着陀螺仪跑：DRDY 中断启动 DMA，读完一个样本就跑一次，样本出来到云台用上只隔 DMA 读取和 INS 解算。
       离上个样本超过 ROBOT_SAMPLE_TIMEOUT_US（BMI088 初始化失败、DMA 卡住、DRDY 不来）就按 tick 跑，每个 tick 一次，编码器模式照常。
       超时用 us 判：样本按陀螺仪自己的时钟出，和 tick 不同步，两个样本之间 tick 可能走 2，
       按 tick 差判（差 2 就算超时）会在样本到之前误判、多跑一次。
       传给模块的还是 tick：模块拿它判超时、计时，没有拿两次的 tick 差当 dt，两次之间 tick 差 0 或 2 都没事 */
    if ((INS_IsSampleReady() == 0U) &&
        ((now == robot_last_tick) || ((Robot_NowUs() - INS_GetSampleUs()) <= ROBOT_SAMPLE_TIMEOUT_US)))
    {
        return;
    }
    robot_last_tick = now;

    /* 0. 传感器：处理 IMU 样本、姿态解算，结果交给要用它的模块（现在只有云台的 IMU 模式） */
    INS_Update(now);
    INS_GetFeedback(&imu);
    Gimbal_SetImu(&imu);

    /* 1. 指令：测试工程用调试变量；合并后换成 robot_cmd */
    Gimbal_GetFeedback(&feedback);
    GimbalTestCmd_Update(now, &feedback, &gimbal_cmd, &chassis_cmd);
    Gimbal_SetCmd(&gimbal_cmd);
    ShootTestCmd_Update(&shoot_cmd);
    Shoot_SetCmd(&shoot_cmd);

    /* 2. 控制：模块分别更新，不在模块内部发送 CAN 帧 */
    Gimbal_Update(now);
    Shoot_Update(now);

    /* 3. 发送：所有模块算完后，每路 CAN 每周期只发一次。RS05 的反馈只靠应答，
          所以每个电机每周期都要发一帧（RELAX 时是停止帧）。 */
    (void)RS05_Manager_SendAll(&robot_can1);

    /* 板间指令 0x110 发给底盘板（两块板之间那路，云台板默认 CAN1）。放在 SendAll 后面：
       邮箱不够时先让电机帧进邮箱。两个 RS05 加 0x110 正好占满 3 个发送邮箱，这一路别再加周期帧。
       CAN1 在 .ioc 里开了自动重发：RS05 的应答比这三帧都优先，仲裁输了要靠重发（根目录 notes.md 第 4 节）。 */
    (void)ChassisLink_SendCmd(&hcan1, &chassis_cmd);

    /* 三个大疆电机合成一帧 0x200；第四槽为零。CAN2 邮箱与 CAN1 独立。 */
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
        RS05_Manager_ProcessRxFrame(&robot_can1, &header, data);
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

/* HAL 的 EXTI 回调：PC5 上升沿是陀螺仪存好了一个新样本（BMI088_Init 里打开的 DRDY），在这里启动 DMA 读它。
   记下的时刻比真正的上升沿晚一个中断延迟：同为优先级 5 的 SPI1、DMA 中断正在跑，CAN 接收中断（优先级 0）插进来，
   或者有地方关了中断，都会往后推。 */
void HAL_GPIO_EXTI_Callback(uint16_t GPIO_Pin)
{
    if (GPIO_Pin == BMI088_GYRO_DRDY_Pin)
    {
        INS_GyroDrdy(Robot_NowUs());
    }
}
