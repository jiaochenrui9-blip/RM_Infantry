#include "chassis_link.h"

#include <math.h>
#include <string.h>

#include "can_bus.h"
#include "user_math.h"

/* 中断里写、主循环里读：主循环关中断整体拷一份，免得读到半新半旧的一帧。调试时看 chassis_link_tick 跟不跟着走。 */
static uint8_t chassis_link_data[8];        /* 最新一帧的原始数据 */
static uint32_t chassis_link_tick;          /* 收到最新一帧时的 HAL tick */

static int16_t ChassisLink_ReadS16(const uint8_t *data)
{
    return (int16_t)(((uint16_t)data[0] << 8) | (uint16_t)data[1]);
}

/* 和 ReadS16 对应：高字节在前。先转成 uint16 再拆，负数也是按模 65536，不靠有符号右移 */
static void ChassisLink_WriteS16(uint8_t *data, int16_t value)
{
    data[0] = (uint8_t)((uint16_t)value >> 8);
    data[1] = (uint8_t)value;
}

HAL_StatusTypeDef ChassisLink_Init(CAN_HandleTypeDef *hcan)
{
    /* 16 位列表模式一组能放 4 个 ID，这里只要一个，4 个位置都填 0x110；列表模式只收标准数据帧 */
    const uint16_t ids[4] = {CHASSIS_LINK_CMD_ID, CHASSIS_LINK_CMD_ID, CHASSIS_LINK_CMD_ID, CHASSIS_LINK_CMD_ID};

    if (hcan == NULL)
    {
        return HAL_ERROR;
    }
    if (hcan->Instance == CAN1)
    {
        return CAN_Bus_ConfigStdIdList(hcan, 2U, 14U, ids);
    }
    if (hcan->Instance == CAN2)
    {
        return CAN_Bus_ConfigStdIdList(hcan, 16U, 14U, ids);
    }
    return HAL_ERROR;
}

void ChassisLink_ProcessRxFrame(const CAN_RxHeaderTypeDef *header, const uint8_t data[8])
{
    /* 同一路上还有大疆电机的反馈帧，不是 0x110 的直接忽略 */
    if ((header == NULL) || (data == NULL) ||
        (header->IDE != CAN_ID_STD) || (header->RTR != CAN_RTR_DATA) ||
        (header->StdId != CHASSIS_LINK_CMD_ID) || (header->DLC != 8U))
    {
        return;
    }

    memcpy(chassis_link_data, data, sizeof(chassis_link_data));
    chassis_link_tick = HAL_GetTick();
}

void ChassisLink_GetCmd(uint32_t now_tick, ChassisCmd_t *cmd)
{
    uint8_t data[8];
    uint32_t tick;
    uint32_t primask;

    if (cmd == NULL)
    {
        return;
    }

    primask = __get_PRIMASK();
    __disable_irq();
    memcpy(data, chassis_link_data, sizeof(data));
    tick = chassis_link_tick;
    __set_PRIMASK(primask);

    /* 按有符号数比：robot.c 读完 now_tick 之后才收到的帧，tick 会比 now_tick 大，不能算成超时 */
    if ((int32_t)(now_tick - tick) > (int32_t)CHASSIS_LINK_TIMEOUT_MS)
    {
        *cmd = (ChassisCmd_t){.mode = CHASSIS_MODE_RELAX};
        return;
    }

    cmd->mode = (ChassisMode_e)data[0];     /* 非法值由 Chassis_SetCmd 当 RELAX */
    cmd->vx_mps = (float)ChassisLink_ReadS16(&data[2]) * 0.001f;
    cmd->vy_mps = (float)ChassisLink_ReadS16(&data[4]) * 0.001f;
    cmd->gimbal_yaw_rel_deg = (float)ChassisLink_ReadS16(&data[6]) * 0.01f;
}

/* 按文件开头的帧格式打包，8 个字节都在这里写。 */
static void ChassisLink_BuildFrame(uint8_t data[8], const ChassisCmd_t *cmd)
{
    /* 先限幅再转：超出 int16 的 float 直接转是未定义行为。roundf 四舍五入：float 存不准小数，直接截断有时会少 1 */
    int16_t vx_int16 = (int16_t)roundf(Math_Clamp(cmd->vx_mps * 1000.0f, -32768.0f, 32767.0f));
    int16_t vy_int16 = (int16_t)roundf(Math_Clamp(cmd->vy_mps * 1000.0f, -32768.0f, 32767.0f));
    int16_t yaw_int16 = (int16_t)roundf(Math_Clamp(cmd->gimbal_yaw_rel_deg * 100.0f, -32768.0f, 32767.0f));

    data[0] = (uint8_t)cmd->mode;
    data[1] = 0U;                               /* 保留 */
    ChassisLink_WriteS16(&data[2], vx_int16);   /* mm/s */
    ChassisLink_WriteS16(&data[4], vy_int16);
    ChassisLink_WriteS16(&data[6], yaw_int16);  /* 0.01° */
}

HAL_StatusTypeDef ChassisLink_SendCmd(CAN_HandleTypeDef *hcan, const ChassisCmd_t *cmd)
{
    uint8_t data[8];

    if ((hcan == NULL) || (cmd == NULL))
    {
        return HAL_ERROR;
    }

    ChassisLink_BuildFrame(data, cmd);
    return CAN_Bus_SendStdData(hcan, CHASSIS_LINK_CMD_ID, data, sizeof(data));
}
