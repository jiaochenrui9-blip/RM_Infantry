/*
 * 假的 can_bus：代替 Shared/BSP/can_bus.c。
 * 过滤器、启动直接返回成功；发出去的帧交给仿真（模型从里面取电机的电流指令）。
 * 只实现 DJI 驱动用到的三个函数，以后驱动调了别的 CAN_Bus_xxx，链接时会报“未定义”，照着补就行。
 */

#include "can_bus.h"

CAN_TypeDef sim_can1_regs;
CAN_TypeDef sim_can2_regs;
CAN_HandleTypeDef hcan1 = {&sim_can1_regs};
CAN_HandleTypeDef hcan2 = {&sim_can2_regs};

/* 在 sim/gimbal_sim.c 里 */
void Sim_OnCanTx(const CAN_HandleTypeDef *hcan, uint16_t std_id, const uint8_t *data, uint8_t length);

HAL_StatusTypeDef CAN_Bus_ConfigStdIdMask(CAN_HandleTypeDef *hcan,
                                          uint32_t filter_bank,
                                          uint32_t slave_start_bank,
                                          uint16_t std_id,
                                          uint16_t std_id_mask)
{
    (void)filter_bank;
    (void)slave_start_bank;
    (void)std_id;
    (void)std_id_mask;
    return (hcan != NULL) ? HAL_OK : HAL_ERROR;
}

HAL_StatusTypeDef CAN_Bus_StartRxFifo0(CAN_HandleTypeDef *hcan)
{
    return (hcan != NULL) ? HAL_OK : HAL_ERROR;
}

HAL_StatusTypeDef CAN_Bus_SendStdData(CAN_HandleTypeDef *hcan,
                                      uint16_t std_id,
                                      const uint8_t *data,
                                      uint8_t length)
{
    if ((hcan == NULL) || (data == NULL))
    {
        return HAL_ERROR;
    }

    Sim_OnCanTx(hcan, std_id, data, length);
    return HAL_OK;
}
