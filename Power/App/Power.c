#include "Power.h"
#include "Power_config.h"
#include "i2c_bus.h"
#include <stddef.h>

#define INA228_REG_CONFIG          0x00U
#define INA228_REG_ADC_CONFIG      0x01U
#define INA228_REG_SHUNT_CAL       0x02U
#define INA228_REG_VSHUNT          0x04U
#define INA228_REG_VBUS            0x05U
#define INA228_REG_CURRENT         0x07U
#define INA228_REG_POWER           0x08U
#define INA228_REG_DIAG_ALRT       0x0BU
#define INA228_REG_MANUFACTURER_ID 0x3EU
#define INA228_REG_DEVICE_ID       0x3FU
#define INA228_CONVERSION_READY    (1U << 1)

/* 大压差档 ±163.84 mV；连续测量电压、压差，各 280 μs，平均四次。 */
#define INA228_CONFIG_VALUE        0x0000U
#define INA228_ADC_CONFIG_VALUE    0xB6D9U
#define INA228_SHUNT_CAL_VALUE \
    ((uint16_t)(13107200000.0 * POWER_CURRENT_LSB_A * POWER_SHUNT_OHM + 0.5))

static I2C_HandleTypeDef *s_i2c;
static PowerFeedback_t s_feedback = {.status = HAL_ERROR};

static HAL_StatusTypeDef Power_WriteChecked(uint8_t reg, uint16_t value)
{
    uint16_t actual;
    HAL_StatusTypeDef status = I2C_Bus_WriteRegU16(s_i2c, POWER_I2C_ADDRESS, reg, value);
    if (status != HAL_OK) {
        return status;
    }
    status = I2C_Bus_ReadRegU16(s_i2c, POWER_I2C_ADDRESS, reg, &actual);
    if (status != HAL_OK) {
        return status;
    }
    return actual == value ? HAL_OK : HAL_ERROR;
}

static HAL_StatusTypeDef Power_ReadU24(uint8_t reg, uint32_t *value)
{
    uint8_t data[3];
    HAL_StatusTypeDef status = I2C_Bus_ReadReg(s_i2c, POWER_I2C_ADDRESS,
                                              reg, data, sizeof(data));
    if (status == HAL_OK) {
        *value = ((uint32_t)data[0] << 16) | ((uint32_t)data[1] << 8) | data[2];
    }
    return status;
}

static int32_t Power_DecodeSigned20(uint32_t raw)
{
    /* 有效位在 23:4；减去 2^20 完成符号扩展，不依赖有符号右移。 */
    int32_t value = (int32_t)(raw >> 4);
    return (value & 0x80000) != 0 ? value - 0x100000 : value;
}

HAL_StatusTypeDef Power_Init(I2C_HandleTypeDef *hi2c)
{
    s_i2c = hi2c;
    s_feedback = (PowerFeedback_t){.status = HAL_ERROR};
    if (hi2c == NULL)
    {
        return HAL_ERROR;
    }

    HAL_StatusTypeDef status = I2C_Bus_ReadRegU16(s_i2c, POWER_I2C_ADDRESS,
                                INA228_REG_MANUFACTURER_ID, &s_feedback.manufacturer_id);
    if (status != HAL_OK)
    {
        goto fail;
    }
    status = I2C_Bus_ReadRegU16(s_i2c, POWER_I2C_ADDRESS,
                                INA228_REG_DEVICE_ID, &s_feedback.device_id);
    if (status != HAL_OK) {
        goto fail;
    }
    /* 只匹配器件型号，允许芯片低四位修订号不同。 */
    if (s_feedback.manufacturer_id != 0x5449U ||
        (s_feedback.device_id >> 4) != 0x228U) {
        status = HAL_ERROR;
        goto fail;
    }

    status = Power_WriteChecked(INA228_REG_ADC_CONFIG, 0x0000U);
    if (status != HAL_OK) {
        goto fail;
    }
    status = Power_WriteChecked(INA228_REG_CONFIG, INA228_CONFIG_VALUE);
    if (status != HAL_OK) {
        goto fail;
    }
    status = Power_WriteChecked(INA228_REG_SHUNT_CAL, INA228_SHUNT_CAL_VALUE);
    if (status != HAL_OK) {
        goto fail;
    }
    status = Power_WriteChecked(INA228_REG_ADC_CONFIG, INA228_ADC_CONFIG_VALUE);
    if (status != HAL_OK) {
        goto fail;
    }
    /* 初始化成功还没有测量结果，不能把清零的数据标为有效。 */
    s_feedback.status = HAL_BUSY;
    return HAL_OK;

fail:
    s_i2c = NULL;
    s_feedback.status = status;
    return status;
}

HAL_StatusTypeDef Power_Update(uint32_t now_tick)
{
    if (s_i2c == NULL)
    {
        return s_feedback.status;
    }

    uint16_t diagnostic;
    HAL_StatusTypeDef status = I2C_Bus_ReadRegU16(s_i2c, POWER_I2C_ADDRESS,
                                                INA228_REG_DIAG_ALRT, &diagnostic);
    if (status != HAL_OK)
    {
        goto finish;
    }
    if ((diagnostic & INA228_CONVERSION_READY) == 0U)
    {
        status = HAL_BUSY;
        goto finish;
    }

    /* 临时组装，全部读取成功才发布，避免某次失败留下半组新数据。 */
    PowerFeedback_t next = s_feedback;
    uint32_t raw;
    status = Power_ReadU24(INA228_REG_VSHUNT, &raw);
    if (status != HAL_OK) {
        goto finish;
    }
    next.shunt_voltage_v = (float)Power_DecodeSigned20(raw) * 0.0000003125f;

    status = Power_ReadU24(INA228_REG_VBUS, &raw);
    if (status != HAL_OK) {
        goto finish;
    }
    next.voltage_v = (float)(raw >> 4) * 0.0001953125f;

    status = Power_ReadU24(INA228_REG_CURRENT, &raw);
    if (status != HAL_OK)
    {
        goto finish;
    }
    next.current_a = (float)Power_DecodeSigned20(raw) * POWER_CURRENT_LSB_A;

    status = Power_ReadU24(INA228_REG_POWER, &raw);
    if (status != HAL_OK) {
        goto finish;
    }
    /* 功率使用完整二十四位；只有压差、电压、电流需要去掉低四位。 */
    next.power_w = (float)raw * (3.2f * POWER_CURRENT_LSB_A);
    next.sample_tick = now_tick;
    s_feedback = next;

finish:
    s_feedback.status = status;
    return status;
}

void Power_GetFeedback(PowerFeedback_t *feedback)
{
    if (feedback != NULL)
    {
        *feedback = s_feedback;
    }
}
