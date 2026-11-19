#include "i2c_bus.h"
#include <stddef.h>

/* 仅在主循环或任务内调用，不能在控制中断内等待 I2C。 */
#define I2C_BUS_TIMEOUT_MS 5U

HAL_StatusTypeDef I2C_Bus_ReadReg(I2C_HandleTypeDef *hi2c, uint8_t address,
                                 uint8_t reg, uint8_t *data, uint16_t length)
{
    if (hi2c == NULL || address > 0x7FU || data == NULL || length == 0U) {
        return HAL_ERROR;
    }
    /* HAL 接收左移一位后的设备地址，读写方向由 HAL 添加。 */
    return HAL_I2C_Mem_Read(hi2c, (uint16_t)address << 1, reg,
                            I2C_MEMADD_SIZE_8BIT, data, length, I2C_BUS_TIMEOUT_MS);
}

HAL_StatusTypeDef I2C_Bus_WriteReg(I2C_HandleTypeDef *hi2c, uint8_t address,
                                  uint8_t reg, const uint8_t *data, uint16_t length)
{
    if (hi2c == NULL || address > 0x7FU || data == NULL || length == 0U) {
        return HAL_ERROR;
    }
    /* HAL 的发送参数没有 const，阻塞发送不会修改这段数据。 */
    return HAL_I2C_Mem_Write(hi2c, (uint16_t)address << 1, reg,
                             I2C_MEMADD_SIZE_8BIT, (uint8_t *)data, length,
                             I2C_BUS_TIMEOUT_MS);
}

HAL_StatusTypeDef I2C_Bus_ReadRegU16(I2C_HandleTypeDef *hi2c, uint8_t address,
                                    uint8_t reg, uint16_t *value)
{
    uint8_t data[2];
    if (value == NULL)
    {
        return HAL_ERROR;
    }

    HAL_StatusTypeDef status = I2C_Bus_ReadReg(hi2c, address, reg, data, sizeof(data));

    if (status == HAL_OK)
    {
        *value = ((uint16_t)data[0] << 8) | data[1];
    }
    return status;
}

HAL_StatusTypeDef I2C_Bus_WriteRegU16(I2C_HandleTypeDef *hi2c, uint8_t address,
                                     uint8_t reg, uint16_t value)
{
    const uint8_t data[2] = {(uint8_t)(value >> 8), (uint8_t)value};
    return I2C_Bus_WriteReg(hi2c, address, reg, data, sizeof(data));
}
