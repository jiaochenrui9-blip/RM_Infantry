//
// Created by game on 2026/10/29.
//

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
    uint8_t valid;              /* 1：初始化成功、标定完、最近 10 ms 内有新样本 */
    float yaw_deg;              /* -180~180。没有磁力计：以上电朝向为 0，会漂 */
    float pitch_deg;            /* 抬头为正 */
    float yaw_rate_dps;         /* 绕竖直轴的角速度，俯视逆时针为正 */
    float pitch_rate_dps;       /* 绕 pitch 轴（云台头 y 轴）的角速度，抬头为正 */
} InsFeedback_t;

/* 只初始化 BMI088（阻塞约 85 ms）。返回 BMI088_Status_t，0 为成功；失败时 valid 一直为 0，编码器模式照常用。 */
uint8_t INS_Init(SPI_HandleTypeDef *hspi);
/* 每 1 ms 调一次：处理上个周期 DMA 读回的样本，再启动下一次。
   上电后前 1000 个样本（约 1 s）用来标定零偏和初始姿态，期间整车静止。 */
void INS_Update(uint32_t now_tick);
void INS_GetFeedback(InsFeedback_t *feedback);

#endif /* INS_H */
