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

static uint32_t robot_last_tick;

/* 时序测量：只量，不参与控制。tools/timing_watch.py 读这些量，*_min / *_max 由脚本写回初值重新统计。
   时间戳用 TIM2 的计数：1 MHz，单位 us；32 位约 71.6 min 回绕一次，两个时间戳直接按 uint32_t 相减不受回绕影响。
   volatile：DRDY 的个数和时刻在中断里写、主循环里读；其余的程序里不用、只给脚本看，加上免得优化时被删掉。 */
static volatile uint32_t robot_drdy_count;                       /* 陀螺仪 DRDY（PC5 上升沿）的个数 */
static volatile uint32_t robot_drdy_us;                          /* 最近一次 DRDY 的时刻 */
static volatile uint32_t robot_drdy_period_min_us = UINT32_MAX;  /* 相邻两次 DRDY 的间隔 */
static volatile uint32_t robot_drdy_period_max_us;
static volatile uint32_t robot_drdy_1000_us;    /* 最近 1000 个 DRDY 周期一共多少 us：陀螺仪实际的 ODR = 1e9 / 它（Hz） */
static uint32_t robot_drdy_1000_start_us;
static volatile uint32_t robot_loop_new_samples[3];  /* Robot_Loop 开始时，离上一次开始来了 0 / 1 / 2 个及以上新样本，各多少次 */
static uint32_t robot_loop_drdy_count;
static volatile uint32_t robot_sample_age_us;   /* Robot_Loop 开始时，最新的样本已经出来多久 */
static volatile uint32_t robot_ins_us;          /* INS_Update 的耗时（含每拍阻塞读一次温度） */
static volatile uint32_t robot_loop_us;         /* Robot_Loop 的耗时，期间进来的中断也算在内 */
static volatile uint32_t robot_loop_max_us;

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
    uint32_t start_us;
    uint32_t drdy_count;
    uint32_t drdy_us;
    uint32_t new_samples;
    InsFeedback_t imu;
    GimbalFeedback_t feedback;
    GimbalCmd_t gimbal_cmd;
    ChassisCmd_t chassis_cmd;
    ShootCmd_t shoot_cmd;

    if (now == robot_last_tick)
    {
        return;
    }
    robot_last_tick = now;

    /* 测量：离上一拍开始来了几个新样本，最新的那个出来多久了。个数和时刻要是同一次 DRDY 的：
       读到一半进了 DRDY 中断（前后两次读到的个数不一样）就重读。 */
    do
    {
        drdy_count = robot_drdy_count;
        drdy_us = robot_drdy_us;
    } while (drdy_count != robot_drdy_count);
    start_us = Robot_NowUs();
    new_samples = drdy_count - robot_loop_drdy_count;
    robot_loop_drdy_count = drdy_count;
    robot_loop_new_samples[(new_samples < 2u) ? new_samples : 2u]++;
    robot_sample_age_us = start_us - drdy_us;

    /* 0. 传感器：IMU 采样和姿态解算，结果交给要用它的模块（现在只有云台的 IMU 模式） */
    INS_Update(now);
    robot_ins_us = Robot_NowUs() - start_us;
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

    robot_loop_us = Robot_NowUs() - start_us;
    if (robot_loop_us > robot_loop_max_us)
    {
        robot_loop_max_us = robot_loop_us;
    }
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

/* HAL 的 EXTI 回调：PC5 上升沿是陀螺仪存好了一个新样本（BMI088_Init 里打开的 DRDY）。现在只记个数和时刻。
   记下的时刻比真正的上升沿晚一个中断延迟：同为优先级 5 的 SPI1、DMA 中断正在跑，或者有地方关了中断，都会往后推。 */
void HAL_GPIO_EXTI_Callback(uint16_t GPIO_Pin)
{
    uint32_t now_us;
    uint32_t period_us;

    if (GPIO_Pin != BMI088_GYRO_DRDY_Pin)
    {
        return;
    }

    now_us = Robot_NowUs();
    if (robot_drdy_count == 0u)
    {
        robot_drdy_1000_start_us = now_us;
    }
    else
    {
        period_us = now_us - robot_drdy_us;
        if (period_us < robot_drdy_period_min_us)
        {
            robot_drdy_period_min_us = period_us;
        }
        if (period_us > robot_drdy_period_max_us)
        {
            robot_drdy_period_max_us = period_us;
        }
        if ((robot_drdy_count % 1000u) == 0u)
        {
            robot_drdy_1000_us = now_us - robot_drdy_1000_start_us;
            robot_drdy_1000_start_us = now_us;
        }
    }
    robot_drdy_us = now_us;
    robot_drdy_count++;
}
