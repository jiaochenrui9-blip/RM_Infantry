#ifndef I2C_BUS_H
#define I2C_BUS_H

#include "main.h"

/* 阻塞访问：设备地址传未左移的七位地址，寄存器地址为一个字节。 */
HAL_StatusTypeDef I2C_Bus_ReadReg(I2C_HandleTypeDef *hi2c, uint8_t address,
                                 uint8_t reg, uint8_t *data, uint16_t length);
HAL_StatusTypeDef I2C_Bus_WriteReg(I2C_HandleTypeDef *hi2c, uint8_t address,
                                  uint8_t reg, const uint8_t *data, uint16_t length);

/* 高字节在前的十六位寄存器，可用于读取器件 ID；失败时不修改输出值。 */
HAL_StatusTypeDef I2C_Bus_ReadRegU16(I2C_HandleTypeDef *hi2c, uint8_t address,
                                    uint8_t reg, uint16_t *value);
HAL_StatusTypeDef I2C_Bus_WriteRegU16(I2C_HandleTypeDef *hi2c, uint8_t address,
                                     uint8_t reg, uint16_t value);

#endif
