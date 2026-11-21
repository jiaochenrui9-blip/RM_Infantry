#ifndef INS_H
#define INS_H

#include <stdint.h>
#include "main.h"

/*
 * 惯导模块：BMI088 采样 → 乘安装矩阵转到云台头坐标系 → Mahony 姿态解算。
 * 输出按全车约定：yaw 俯视逆时针为正，pitch 抬头为正；单位 deg、dps。
 */
typedef struct
{
    uint8_t valid;              /* 1：初始化成功、到温后标定完、最近 10 ms 内有新样本 */
    float yaw_deg;              /* -180~180。没有磁力计：以上电朝向为 0，会漂 */
    float pitch_deg;            /* 抬头为正 */
    float yaw_rate_dps;         /* 绕竖直轴的角速度，俯视逆时针为正 */
    float pitch_rate_dps;       /* 绕 pitch 轴（云台头 y 轴）的角速度，抬头为正 */
} InsFeedback_t;

/* 初始化 BMI088（阻塞约 85 ms），成功后启动加热 PWM（占空比 0）。返回 BMI088_Status_t，0 为成功；
   失败时不加热、valid 一直为 0，编码器模式照常用。htim 传加热电阻那路 PWM（C 板 TIM10，用通道 1）。 */
uint8_t INS_Init(SPI_HandleTypeDef *hspi, TIM_HandleTypeDef *htim);
/* 陀螺仪 DRDY 中断里调：启动 DMA 读这个样本，now_us 记成它的时刻（32 位 us 计数，按 uint32_t 相减，回绕不影响）。 */
void INS_GyroDrdy(uint32_t now_us);
/* 有没有读完、还没处理的样本。有就该调 INS_Update 了。 */
uint8_t INS_IsSampleReady(void);
/* 最后一次处理的样本的时刻（INS_GyroDrdy 记的 us）。 */
uint32_t INS_GetSampleUs(void);
/* 有新样本就调（每个样本一次）：处理样本、姿态解算，dt 用相邻两个样本的时刻；每 100 ms 读一次温度、更新加热占空比。
   采样停了也要每 1 ms 调一次，温度才会接着读、valid 才会变 0。
   上电后先升温（要接 24 V）；温度在目标 ±0.25 ℃ 内连续 5000 个样本（约 5 s，期间整车静止）才标定好零偏和初始姿态。
   上电 INS_HEAT_TIMEOUT_MS 后还没到温（比如没接 24 V），就不管温度直接标定。标定完不再看温度。 */
void INS_Update(uint32_t now_tick);
void INS_GetFeedback(InsFeedback_t *feedback);

#endif /* INS_H */
