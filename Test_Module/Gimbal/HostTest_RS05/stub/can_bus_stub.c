/*
 * 假的 can_bus：代替 Shared/BSP/can_bus.c。
 * 过滤器、启动直接返回成功；发出去的帧交给仿真：扩展帧给 RS05 模型（它马上回一帧反馈），标准帧里的 0x110 给底盘板一侧。
 * 只实现 RS05 驱动和板间通信（chassis_link）用到的函数，以后调了别的 CAN_Bus_xxx，链接时会报“未定义”，照着补就行。
 * 发送邮箱没有模拟：每帧都直接“发出去”。
 */

#include "can_bus.h"

CAN_TypeDef sim_can1_regs;
CAN_TypeDef sim_can2_regs;
CAN_HandleTypeDef hcan1 = {&sim_can1_regs};
CAN_HandleTypeDef hcan2 = {&sim_can2_regs};

/* 在 sim/gimbal_sim.c 里 */
void Sim_OnCanTxStd(const CAN_HandleTypeDef *hcan, uint16_t std_id, const uint8_t *data, uint8_t length);
void Sim_OnCanTxExt(const CAN_HandleTypeDef *hcan, uint32_t ext_id, const uint8_t *data, uint8_t length);

HAL_StatusTypeDef CAN_Bus_ConfigStdIdList(CAN_HandleTypeDef *hcan,
                                          uint32_t filter_bank,
                                          uint32_t slave_start_bank,
                                          const uint16_t std_ids[4])
{
    (void)filter_bank;
    (void)slave_start_bank;
    (void)std_ids;
    return (hcan != NULL) ? HAL_OK : HAL_ERROR;
}

HAL_StatusTypeDef CAN_Bus_ConfigExtIdMask(CAN_HandleTypeDef *hcan,
                                          uint32_t filter_bank,
                                          uint32_t slave_start_bank,
                                          uint32_t ext_id,
                                          uint32_t ext_id_mask)
{
    (void)filter_bank;
    (void)slave_start_bank;
    (void)ext_id;
    (void)ext_id_mask;
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

    Sim_OnCanTxStd(hcan, std_id, data, length);
    return HAL_OK;
}

HAL_StatusTypeDef CAN_Bus_SendExtData(CAN_HandleTypeDef *hcan,
                                      uint32_t ext_id,
                                      const uint8_t *data,
                                      uint8_t length)
{
    if ((hcan == NULL) || (data == NULL))
    {
        return HAL_ERROR;
    }

    Sim_OnCanTxExt(hcan, ext_id, data, length);
    return HAL_OK;
}
