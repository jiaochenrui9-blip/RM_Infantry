//
// Created by game on 2026/8/17.
//

#ifndef BMI088_ATTITUDE_H
#define BMI088_ATTITUDE_H

#include "BMI088.h"
#include "IST8310.h"

typedef struct
{
    float q0;
    float q1;
    float q2;
    float q3;

    float roll_deg;
    float pitch_deg;
    float yaw_deg;

    float gyro_corrected_dps[3];
    float integral_error[3];
} Attitude_Data_t;

typedef struct
{
    float acc_g[3];
    float mag_uT[3];
} Attitude_StartupAverage_t;

/* The BMI088 and IST8310 axes must both use the same body coordinate system. */
void Attitude_Init(void);
uint8_t Attitude_CollectStartupAverage(Attitude_StartupAverage_t *average);
uint8_t Attitude_InitializeQuaternion(const Attitude_StartupAverage_t *average);
/* 没有磁力计时用：只用加速度（静止时的平均值）初始化，roll / pitch 由重力方向算，yaw 置 0。
   加速度的模不在 0.8~1.2 g 时返回 0，姿态不变。 */
uint8_t Attitude_InitFromAcc(const float acc_g[3]);
void Attitude_Update(const BMI088_Data_t *imu, const IST8310_Data_t *mag, float dt_s);
void Attitude_GetData(Attitude_Data_t *data);

#endif //BMI088_ATTITUDE_H
