//
// Created by game on 2026/10/29.
//

#include "ins.h"

#include <math.h>
#include <string.h>

#include "ins_config.h"
#include "BMI088.h"
#include "Attitude.h"
#include "PID.h"

#define INS_DT_S                 0.001f    /* INS_Update 的调用周期 */
#define INS_CALIBRATE_SAMPLES    5000U     /* 到温后连续这么多个样本（约 5 s）用来标定，期间整车静止 */
#define INS_SAMPLE_TIMEOUT_MS    10U       /* 超过这么久没有新样本，valid 置 0 */
#define INS_TEMP_BAND_C          0.25f     /* 标定期间温度要一直在目标 ±这么多以内（2 个 LSB），出去就从头标定 */
#define INS_RAD_PER_DEG          0.0174532925f

static const float ins_install[3][3] = INS_INSTALL_MATRIX;

/* 调试器里 watch 这几个变量 */
static uint8_t ins_init_status = BMI088_ERR_NOT_INIT;   /* INS_Init 的返回值（BMI088_Status_t），0 为成功 */
static uint32_t ins_calibrate_count;    /* 已用于标定的样本数，到 INS_CALIBRATE_SAMPLES 为止；升温时停在 0 */
static uint32_t ins_last_tick;          /* 最后一次处理样本的 tick，正常时每毫秒跟着变 */
static float ins_gyro_sum[3];           /* 标定用的累加和，都在 C 板坐标系 */
static float ins_acc_sum[3];
static InsFeedback_t ins_feedback;

static TIM_HandleTypeDef *ins_heat_tim; /* 加热电阻的 PWM（C 板 TIM10_CH1），INS_Init 传进来 */
static PID_t ins_bmi088_temp_pid;       /* 温度环：target 目标温度，actual 最近一次读到的温度，out 占空比 0~1 */

/* 乘安装矩阵：C 板坐标系 → 云台头坐标系，原地改 */
static void INS_Rotate(float v[3])
{
    const float board[3] = {v[0], v[1], v[2]};
    uint8_t i;

    for (i = 0U; i < 3U; i++)
    {
        v[i] = ins_install[i][0] * board[0] + ins_install[i][1] * board[1] + ins_install[i][2] * board[2];
    }
}

/* 把 Attitude 的结果换成全车约定。gyro_dps 是云台头坐标系的角速度。 */
static void INS_UpdateFeedback(const float gyro_dps[3])
{
    Attitude_Data_t attitude;
    float pitch_rad;

    Attitude_GetData(&attitude);
    /* Attitude 是 ZYX 欧拉角。x 前 y 左 z 上的系里，绕 +z 正转是俯视逆时针，yaw 直接用；
       绕 +y 正转是低头，pitch 和 pitch 角速度都要取负才是“抬头为正” */
    ins_feedback.yaw_deg = attitude.yaw_deg;
    ins_feedback.pitch_deg = -attitude.pitch_deg;
    ins_feedback.pitch_rate_dps = -gyro_dps[1];
    /* 抬头 θ 时竖直轴在云台头坐标系里是 (sin θ, 0, cos θ)，只用 gz 会小 cos θ 倍 */
    pitch_rad = ins_feedback.pitch_deg * INS_RAD_PER_DEG;
    ins_feedback.yaw_rate_dps = cosf(pitch_rad) * gyro_dps[2] + sinf(pitch_rad) * gyro_dps[0];
}

/* 从头标定：计数和两个累加和一起清。升温时、标定中温度出了范围、平均加速度不对时都走这里 */
static void INS_RestartCalibration(void)
{
    ins_calibrate_count = 0U;
    memset(ins_gyro_sum, 0, sizeof(ins_gyro_sum));
    memset(ins_acc_sum, 0, sizeof(ins_acc_sum));
}

/* 标定：静止时角速度的平均就是零偏，加速度的平均就是重力方向。
   零偏在 C 板坐标系里求、写回驱动（BMI088_Init 已把零偏清 0，所以这里累加的是原始角速度）。
   所以只能标定一次：标定完再标，累加的是减过零偏的残差（≈ 0），SetGyroOffset 是覆盖，零偏就丢了；
   以后要运行中重新标定，先 BMI088_SetGyroOffset(0, 0, 0)；
   重力方向转到云台头坐标系后初始化姿态，标定完姿态就是对的，不用等 Mahony 从单位四元数慢慢收敛。 */
static void INS_Calibrate(const BMI088_Data_t *sample)
{
    const float n = (float)INS_CALIBRATE_SAMPLES;
    const float zero_rate[3] = {0.0f, 0.0f, 0.0f};
    float acc_avg[3];
    uint8_t i;

    for (i = 0U; i < 3U; i++)
    {
        ins_gyro_sum[i] += sample->gyro_dps[i];
        ins_acc_sum[i] += sample->acc_g[i];
    }
    ins_calibrate_count++;
    if (ins_calibrate_count < INS_CALIBRATE_SAMPLES)
    {
        return;
    }

    for (i = 0U; i < 3U; i++)
    {
        acc_avg[i] = ins_acc_sum[i] / n;
    }
    INS_Rotate(acc_avg);
    /* 平均加速度不在 0.8~1.2 g 说明数据不对，清零重新标定 */
    if (Attitude_InitFromAcc(acc_avg) == 0U)
    {
        INS_RestartCalibration();
        return;
    }
    BMI088_SetGyroOffset(ins_gyro_sum[0] / n, ins_gyro_sum[1] / n, ins_gyro_sum[2] / n);
    /* 标定时是静止的，角速度按 0 填；这样 valid 变 1 的这一刻，feedback 里已经是初始姿态 */
    INS_UpdateFeedback(zero_rate);
}

uint8_t INS_Init(SPI_HandleTypeDef *hspi, TIM_HandleTypeDef *htim)
{
    INS_RestartCalibration();
    ins_last_tick = 0U;
    memset(&ins_feedback, 0, sizeof(ins_feedback));

    /*不用D项，因为温度有差分噪声， 只用P项和I项，P项是为了快速响应，I项是为了消除稳态误差*/
    /* 输出和积分项都限在 0~1：加热器只能加热、不能制冷，PID 要知道输出最小就是 0，
       不然输出在 -1~0 时它以为没饱和，会把积分往负的积（这个 PID 的积分限幅限的是 Ki × 积分，单位和输出一样） */
    ins_heat_tim = htim;
    PID_Init(&ins_bmi088_temp_pid, INS_TEMP_PID_KP, INS_TEMP_PID_KI,
             0.0f, 0.0f, 1.0f, 0.0f, 1.0f);
    PID_SetTarget(&ins_bmi088_temp_pid, INS_TEMP_TARGET_C);

    ins_init_status = BMI088_Init(hspi);
    /* 读不到温度就不能加热，所以初始化成功才启动 PWM（CCR 初值 0，不加热）。PWM_Start 会顺带启动计数器 */
    if (ins_init_status == BMI088_OK)
    {
        (void)HAL_TIM_PWM_Start(ins_heat_tim, TIM_CHANNEL_1);
    }
    return ins_init_status;
}

/* 读温度、算加热占空比（0~1，1 为满功率 0.58 W）。读温度是阻塞读，要在 DMA 空闲时调（BMI088_StartSample 之前），
   DMA 在跑时驱动返回 BMI088_ERR_BUSY。读失败返回 0，这一周期不加热 */
static float INS_Temp_Update(void)
{
    float temp_c;

    if (BMI088_ReadTemperature(&temp_c) != BMI088_OK)
    {
        return 0.0f;
    }

    return PID_Calculate(&ins_bmi088_temp_pid, temp_c, INS_DT_S);
}

/* 占空比换成 CCR：乘 ARR + 1。占空比为 1 时 CCR = ARR + 1，PWM1 模式下一直是高电平，就是满功率 */
static void INS_SetPWM(float PWM)
{
    __HAL_TIM_SET_COMPARE(ins_heat_tim, TIM_CHANNEL_1,
                          (uint32_t)(PWM * (float)(__HAL_TIM_GET_AUTORELOAD(ins_heat_tim) + 1U)));
}

void INS_Update(uint32_t now_tick)
{
    BMI088_Data_t sample;

    /* 初始化失败：valid 一直为 0。不在这里重试，BMI088_Init 里的延时共约 85 ms，会卡住控制 */
    if (ins_init_status != BMI088_OK)
    {
        return;
    }

    /* 0. 先读取BMI088的温度值，然后设置占空比。上一周期启动的 DMA 早就读完了，这时 SPI 空闲 */
    INS_SetPWM(INS_Temp_Update());

    /* 1. 上一周期启动的 DMA 读完了：取样本（C 板坐标系，已减零偏） */
    if (BMI088_IsSampleReady() != 0U)
    {
        BMI088_ProcessSample();
        BMI088_GetData(&sample);
        ins_last_tick = now_tick;

        if (ins_calibrate_count < INS_CALIBRATE_SAMPLES)
        {
            /* 标定要在恒温下做：温度不在目标 ±INS_TEMP_BAND_C 内就从头来，升温时计数一直停在 0。
               上电 INS_HEAT_TIMEOUT_MS 后还没到温（没接 24 V 等）就不管温度了。标定完不再看温度 */
            if ((fabsf(ins_bmi088_temp_pid.actual - ins_bmi088_temp_pid.target) > INS_TEMP_BAND_C) &&
                (now_tick < INS_HEAT_TIMEOUT_MS))
            {
                INS_RestartCalibration();
            }
            else
            {
                INS_Calibrate(&sample);
            }
        }
        else
        {
            /* 先转到云台头坐标系再解算，不要解算完再去换欧拉角 */
            INS_Rotate(sample.gyro_dps);
            INS_Rotate(sample.acc_g);
            Attitude_Update(&sample, NULL, INS_DT_S);
            INS_UpdateFeedback(sample.gyro_dps);
        }
    }

    /* 2. 不管这周期有没有样本都启动下一次。SPI / DMA 出过错时，驱动会在这里自动复位重试 */
    (void)BMI088_StartSample();

    /* 3. 每周期重算：标定完、且最近 10 ms 内有新样本。采样恢复后自动回到 1，锁定交给云台 */
    ins_feedback.valid = ((ins_calibrate_count >= INS_CALIBRATE_SAMPLES) &&
                          ((now_tick - ins_last_tick) <= INS_SAMPLE_TIMEOUT_MS)) ? 1U : 0U;
}

void INS_GetFeedback(InsFeedback_t *feedback)
{
    if (feedback != NULL)
    {
        *feedback = ins_feedback;
    }
}
