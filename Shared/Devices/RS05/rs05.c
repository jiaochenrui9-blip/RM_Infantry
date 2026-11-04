#include "rs05.h"

#include <string.h>

#include "can_bus.h"

#define RS05_HOST_ID                0xFDU   /* 本机（主机）ID，填在指令 ID 的 bit15~8，手册示例都用 0xFD */

/* 通信类型（ID 的 bit28~24） */
#define RS05_TYPE_FEEDBACK          2U
#define RS05_TYPE_ENABLE            3U
#define RS05_TYPE_STOP              4U
#define RS05_TYPE_WRITE             18U     /* 单个参数写入，掉电丢失 */

/* 参数 index（手册「可读写单个参数列表」） */
#define RS05_INDEX_RUN_MODE         0x7005U /* uint8，3 为电流模式 */
#define RS05_INDEX_IQ_REF           0x7006U /* float，A */
#define RS05_INDEX_CAN_TIMEOUT      0x7028U /* uint32，20000 为 1 s，0 不启用（出厂为 0） */

#define RS05_RUN_MODE_CURRENT       3U
/* 超过这么久没收到 CAN 指令，电机自己进 Reset（没有力）。不设的话停发（比如调试器暂停）后电机会怎样手册没写，
   一般是保持最后一次的电流。这里 2000 即 100 ms */
#define RS05_CAN_TIMEOUT            2000U
#define RS05_STATE_RUN              2U
#define RS05_ENABLE_STEP_COUNT      4U
/* 使能序列里，电机超过这么久没回反馈就从第一步重来。应答一般在发出后 1 ms 内就到（手册没写延迟，上板看），留 1 ms 余量 */
#define RS05_REPLY_TIMEOUT_MS       2U

/* 反馈的换算范围：0~65535 对应 -max~max */
#define RS05_POSITION_MAX_RAD       12.57f
#define RS05_VELOCITY_MAX_RAD_S     50.0f

static RS05_Motor_t *RS05_FindMotor(const RS05_Manager_t *manager, uint8_t id)
{
    for (uint8_t i = 0U; i < manager->count; i++)
    {
        if (manager->motors[i]->id == id)
        {
            return manager->motors[i];
        }
    }
    return NULL;
}

/* 主机发的指令：数据区 2 只放主机 ID */
static HAL_StatusTypeDef RS05_Send(CAN_HandleTypeDef *hcan, uint8_t type, uint8_t id, const uint8_t data[8])
{
    uint32_t ext_id = ((uint32_t)type << 24) | ((uint32_t)RS05_HOST_ID << 8) | id;

    return CAN_Bus_SendExtData(hcan, ext_id, data, 8U);
}

/* 类型 18：Byte0~1 是 index，Byte4~7 是数值，都是低字节在前；uint8 的值在 Byte4，float 按 IEEE 754 的 4 个字节 */
static HAL_StatusTypeDef RS05_WriteParameter(CAN_HandleTypeDef *hcan, uint8_t id, uint16_t index, uint32_t value)
{
    uint8_t data[8] = {0};

    data[0] = (uint8_t)index;
    data[1] = (uint8_t)(index >> 8);
    data[4] = (uint8_t)value;
    data[5] = (uint8_t)(value >> 8);
    data[6] = (uint8_t)(value >> 16);
    data[7] = (uint8_t)(value >> 24);
    return RS05_Send(hcan, RS05_TYPE_WRITE, id, data);
}

static HAL_StatusTypeDef RS05_WriteFloat(CAN_HandleTypeDef *hcan, uint8_t id, uint16_t index, float value)
{
    uint32_t raw;

    memcpy(&raw, &value, sizeof(raw));
    return RS05_WriteParameter(hcan, id, index, raw);
}

/* 反馈数据：高字节在前，0~65535 线性对应 -max~max */
static float RS05_ReadScaled(const uint8_t *data, float max)
{
    uint16_t raw = (uint16_t)(((uint16_t)data[0] << 8) | data[1]);

    return (float)raw * (2.0f * max / 65535.0f) - max;
}

/* 给一个电机发这一周期的那一帧（见 rs05.h 开头） */
static HAL_StatusTypeDef RS05_SendMotor(CAN_HandleTypeDef *hcan, RS05_Motor_t *motor, uint32_t now_tick)
{
    static const uint8_t zero[8] = {0};    /* 使能、停止的数据区全 0（停止帧 Byte0 = 1 是清故障，这里不用） */
    HAL_StatusTypeDef status;

    if (motor->enable == 0U)
    {
        motor->enable_step = 0U;
        return RS05_Send(hcan, RS05_TYPE_STOP, motor->id, zero);
    }
    if (motor->state == RS05_STATE_RUN)
    {
        motor->enable_step = 0U;
        return RS05_WriteFloat(hcan, motor->id, RS05_INDEX_IQ_REF, motor->current_a);
    }

    /* 没在运行：按手册 4.3.2 先设模式再使能（运行中不能切模式，所以只在没运行时写 run_mode）。
       canTimeout、run_mode 用类型 18 写，掉电丢失，所以每次使能都重写。
       电机里可能还留着上一次的 iq_ref（停止会不会清手册没写），使能前先写 0。
       电机最近没回过反馈（还没上电、正在重启）就一直发第一步：不然它可能半路才上电、先收到使能，
       停在上电默认的运控模式里，之后写 iq_ref 都不起作用，反馈里却显示在运行。 */
    if ((motor->feedback_tick == 0U) ||
        ((int32_t)(now_tick - motor->feedback_tick) > (int32_t)RS05_REPLY_TIMEOUT_MS))
    {
        motor->enable_step = 0U;
    }
    switch (motor->enable_step)
    {
        case 0U:
            status = RS05_WriteParameter(hcan, motor->id, RS05_INDEX_RUN_MODE, RS05_RUN_MODE_CURRENT);
            break;
        case 1U:
            status = RS05_WriteParameter(hcan, motor->id, RS05_INDEX_CAN_TIMEOUT, RS05_CAN_TIMEOUT);
            break;
        case 2U:
            status = RS05_WriteFloat(hcan, motor->id, RS05_INDEX_IQ_REF, 0.0f);
            break;
        default:
            status = RS05_Send(hcan, RS05_TYPE_ENABLE, motor->id, zero);
            break;
    }
    /* 这一步进了邮箱才走下一步；使能之后还没在运行（比如使能帧丢了），下周期从头再来 */
    if (status == HAL_OK)
    {
        motor->enable_step = (uint8_t)((motor->enable_step + 1U) % RS05_ENABLE_STEP_COUNT);
    }
    return status;
}

HAL_StatusTypeDef RS05_Manager_Init(RS05_Manager_t *manager, CAN_HandleTypeDef *hcan)
{
    if ((manager == NULL) || (hcan == NULL))
    {
        return HAL_ERROR;
    }

    *manager = (RS05_Manager_t){0};
    manager->hcan = hcan;
    return HAL_OK;
}

HAL_StatusTypeDef RS05_Register(RS05_Manager_t *manager, RS05_Motor_t *motor, uint8_t id)
{
    if ((manager == NULL) || (manager->hcan == NULL) || (motor == NULL) || (id > 127U) ||
        (manager->count >= RS05_MOTOR_MAX) || (RS05_FindMotor(manager, id) != NULL))
    {
        return HAL_ERROR;
    }

    *motor = (RS05_Motor_t){0};
    motor->id = id;
    manager->motors[manager->count] = motor;
    manager->count++;
    return HAL_OK;
}

HAL_StatusTypeDef RS05_Manager_Start(RS05_Manager_t *manager)
{
    HAL_StatusTypeDef status;
    uint32_t filter_bank;

    if ((manager == NULL) || (manager->hcan == NULL))
    {
        return HAL_ERROR;
    }

    /* 过滤器组分配见 can_bus.h：扩展帧设备用 CAN1 的组 1、CAN2 的组 15 */
    if (manager->hcan->Instance == CAN1)
    {
        filter_bank = 1U;
    }
    else if (manager->hcan->Instance == CAN2)
    {
        filter_bank = 15U;
    }
    else
    {
        return HAL_ERROR;
    }

    /* 只比较通信类型（bit28~24）：反馈帧 ID 里的状态、故障位、电机 ID 每帧都可能不同 */
    status = CAN_Bus_ConfigExtIdMask(manager->hcan, filter_bank, 14U,
                                     (uint32_t)RS05_TYPE_FEEDBACK << 24, 0x1FUL << 24);
    if (status != HAL_OK)
    {
        return status;
    }

    return CAN_Bus_StartRxFifo0(manager->hcan);
}

HAL_StatusTypeDef RS05_Manager_SendAll(RS05_Manager_t *manager)
{
    HAL_StatusTypeDef result = HAL_OK;
    uint32_t now_tick = HAL_GetTick();      /* 和接收中断里给反馈打的时间戳是同一个钟 */

    if ((manager == NULL) || (manager->hcan == NULL))
    {
        return HAL_ERROR;
    }

    for (uint8_t i = 0U; i < manager->count; i++)
    {
        HAL_StatusTypeDef status = RS05_SendMotor(manager->hcan, manager->motors[i], now_tick);

        if ((status != HAL_OK) && (result == HAL_OK))
        {
            result = status;
        }
    }
    return result;
}

void RS05_Manager_ProcessRxFrame(RS05_Manager_t *manager, const CAN_RxHeaderTypeDef *header,
                                 const uint8_t data[8])
{
    RS05_Motor_t *motor;
    uint32_t ext_id;

    if ((manager == NULL) || (header == NULL) || (data == NULL))
    {
        return;
    }
    /* 同一路上可能还有大疆电机、板间通信的标准帧 */
    if ((header->IDE != CAN_ID_EXT) || (header->RTR != CAN_RTR_DATA) || (header->DLC != 8U))
    {
        return;
    }

    /* 类型 2：bit23~22 模式状态、bit21~16 故障、bit15~8 电机 ID、bit7~0 主机 ID */
    ext_id = header->ExtId;
    if (((ext_id >> 24) & 0x1FU) != RS05_TYPE_FEEDBACK)
    {
        return;
    }
    motor = RS05_FindMotor(manager, (uint8_t)(ext_id >> 8));
    if (motor == NULL)
    {
        return;
    }

    /* Byte4~5 力矩、Byte6~7 温度现在用不到，不存 */
    motor->position_rad = RS05_ReadScaled(&data[0], RS05_POSITION_MAX_RAD);
    motor->velocity_rad_s = RS05_ReadScaled(&data[2], RS05_VELOCITY_MAX_RAD_S);
    motor->state = (uint8_t)((ext_id >> 22) & 0x03U);
    motor->fault = (uint8_t)((ext_id >> 16) & 0x3FU);
    motor->feedback_tick = HAL_GetTick();
}

void RS05_SetCurrent(RS05_Motor_t *motor, uint8_t enable, float current_a)
{
    if (motor == NULL)
    {
        return;
    }

    motor->enable = (enable != 0U) ? 1U : 0U;
    motor->current_a = current_a;
}

uint8_t RS05_IsOnline(const RS05_Motor_t *motor, uint32_t now_tick, uint32_t timeout_ms)
{
    if ((motor == NULL) || (motor->feedback_tick == 0U) || (motor->fault != 0U))
    {
        return 0U;
    }

    /* 反馈可能在调用者取 now_tick 之后才到，时间戳比 now_tick 还大：按有符号算，算作刚收到 */
    return ((int32_t)(now_tick - motor->feedback_tick) <= (int32_t)timeout_ms) ? 1U : 0U;
}
