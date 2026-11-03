/*
 * 整车仿真用的假 can_bus：代替 Shared/BSP/can_bus.c。
 * 两块板各有两路 CAN，各用一个句柄（Instance 照真的填 CAN1 / CAN2，驱动按它选过滤器组）。
 * 过滤器、启动直接成功；发出去的帧交给 sim/car_sim.c 的 Car_OnCanTx，按“哪块板、哪一路”路由。
 */

#include "can_bus.h"

CAN_TypeDef sim_can1_regs;
CAN_TypeDef sim_can2_regs;

/* main.h 里声明了这两个（真工程的 CubeMX 句柄），被测代码不用，定义上免得链接报错 */
CAN_HandleTypeDef hcan1 = {&sim_can1_regs};
CAN_HandleTypeDef hcan2 = {&sim_can2_regs};

CAN_HandleTypeDef car_gimbal_hcan1 = {&sim_can1_regs};     /* 云台板 CAN1：云台电机 + 板间 */
CAN_HandleTypeDef car_gimbal_hcan2 = {&sim_can2_regs};     /* 云台板 CAN2：现在没东西 */
CAN_HandleTypeDef car_chassis_hcan1 = {&sim_can1_regs};    /* 底盘板 CAN1：板间 */
CAN_HandleTypeDef car_chassis_hcan2 = {&sim_can2_regs};    /* 底盘板 CAN2：4 个 M3508 */

/* 在 sim/car_sim.c 里 */
void Car_OnCanTx(const CAN_HandleTypeDef *hcan, uint16_t std_id, const uint8_t *data, uint8_t length);

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

    Car_OnCanTx(hcan, std_id, data, length);
    return HAL_OK;
}
