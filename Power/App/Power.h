#ifndef POWER_H
#define POWER_H

#include "main.h"

/* 对外反馈只存换算后的物理量，原始寄存器数据仅作为函数内临时量。 */
typedef struct
{
    float voltage_v;            /* VIN 对地电压。 */
    float shunt_voltage_v;      /* VIN 减 VOUT，正值表示输入流向负载。 */
    float current_a;
    float power_w;              /* INA228 的无符号功率结果，方向看电流。 */
    uint32_t sample_tick;       /* 最近完整读取成功的毫秒时间戳。 */
    uint16_t manufacturer_id;
    uint16_t device_id;         /* 包含低四位版本号。 */
    HAL_StatusTypeDef status;   /* OK：成功；BUSY：尚无新样本；其余：失败。 */
} PowerFeedback_t;

/* 读 ID、配置并回读核验；上电或 INA228 复位后调用。 */
HAL_StatusTypeDef Power_Init(I2C_HandleTypeDef *hi2c);
/* 由主循环周期调用，不在函数内死循环；每次按寄存器分别读取。 */
HAL_StatusTypeDef Power_Update(uint32_t now_tick);
void Power_GetFeedback(PowerFeedback_t *feedback);

#endif
