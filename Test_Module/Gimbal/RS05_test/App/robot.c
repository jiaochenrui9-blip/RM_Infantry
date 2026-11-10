#include "robot.h"

#include <math.h>

#include "main.h"
#include "can.h"

#include "can_bus.h"
#include "rs05.h"
#include "user_math.h"

/*
 * RS05 单电机测试：C 板 CAN1 上只接一个 RS05。下面的 rs05_* 变量用 OpenOCD 不停核读写
 * （tools/rs05_watch.py），也可以在调试器里暂停改。步骤见 notes.md「上板步骤」：
 *   1. 手转：rs05_test_enable = 0，电机没力，看 rs05_motor.position_rad、velocity_rad_s 怎么变
 *   2. 电流方向：rs05_test_enable = 1，rs05_test_current_a 给小电流，看往哪边转、iqf 的正负
 *   3. 编码器：程序一直用类型 17 轮流读 rs05_test_params 里的参数（mechPos、编码器原始值、转子角、圈数……），
 *      和类型 2 的 position_rad 对照
 * 电机一直不回话时（ID 不对、协议不对）：CAN1 什么帧都收，驱动和读参数用不到的帧记在 rs05_test_rx_log 里；
 * 外面还能经 rs05_test_tx_frame 发任意一帧。脚本的 scan 就是用这两样试私有协议、MIT、CANopen。
 * 电机 ID 只存在 rs05_motor.id 里，运行中改了，电机帧和读参数一起换过去，不用重新烧。
 */

#define RS05_TEST_MOTOR_ID          1U      /* 上电时用的电机 CAN ID（上位机里设的）。之后以 rs05_motor.id 为准 */
#define RS05_TEST_CURRENT_LIMIT_A   1.0f    /* rs05_test_current_a 超过就限到这里（约 0.67 N·m），防手误 */
/* 电流模式没有速度环，空载给电流会一直加速到约 50 rad/s。输出轴超过这个速度就给 0（不管哪个方向），
   降下来再接着出力，所以最多转到这个速度附近。约 1 圈/秒 */
#define RS05_TEST_SPEED_LIMIT_RAD_S 6.0f
#define RS05_TEST_READ_PERIOD_MS    10U     /* 每隔这么久用类型 17 读一个参数，按 rs05_test_params 的顺序轮流读 */

/* 类型 17（单个参数读取，手册 4.1 节）只有这个测试用，驱动 rs05.c 里没有，帧格式写在这里 */
#define RS05_TEST_HOST_ID           0xFDU   /* 本机 ID，和 rs05.c 的 RS05_HOST_ID 一样 */
#define RS05_TEST_TYPE_FEEDBACK     2U
#define RS05_TEST_TYPE_READ         17U

/* 什么帧都收：组 3 收全部标准数据帧，组 4 收全部扩展数据帧（类型 17 的应答也从这里进来）。
   CAN1 的组 1 是 rs05.c 收类型 2 用的；组 0、2 在别的工程里给了大疆电机、板间帧 0x110 */
#define RS05_TEST_STD_FILTER_BANK   3U
#define RS05_TEST_EXT_FILTER_BANK   4U

#define RS05_TEST_FRAME_EXT         0x80000000UL    /* TestFrame_t.id 的 bit31：1 扩展帧，0 标准帧 */
#define RS05_TEST_RX_LOG_SIZE       16U

typedef struct
{
    uint16_t index;         /* 参数 index */
    uint32_t raw;           /* 读取成功时应答的 Byte4~7（低字节在前）原样存：float 是 IEEE 754 的 4 个字节，
                               int16、uint8 在低位。按类型换算在 tools/rs05_watch.py 里做 */
    uint32_t ok_count;      /* 应答里 bit23~16 为 00（读取成功）的次数，一直在涨说明 raw 是新的 */
    uint32_t fail_count;    /* 为 01（读取失败）的次数。两个都不涨就是电机没回这个 index */
} TestParam_t;

typedef struct
{
    uint32_t id;            /* bit31 为 1 是扩展帧（低 29 位是 ID），为 0 是标准帧（低 11 位是 ID） */
    uint8_t dlc;
    uint8_t data[8];
} TestFrame_t;

/* 手册 4.1 节最后的「可读写单个参数列表」（类型 17 / 18 用的）只有 0x70xx。0x20xx、0x30xx 是 3.3.4 节
   上位机参数表里的，类型 17 能不能读手册没写，看 ok_count / fail_count。
   volatile：固件只写不读这些字段，不加的话 Release 优化可能把写删掉，OpenOCD 就读不到 */
static volatile TestParam_t rs05_test_params[] = {
    {.index = 0x7019U},     /* mechPos：负载端计圈机械角度，float，rad。和 position_rad 对照 */
    {.index = 0x701AU},     /* iqf：iq 滤波值，float，A。和给的电流比正负 */
    {.index = 0x7029U},     /* zero_sta：uint8，0 时上电读数在 0~2π，1 时在 −π~π */
    {.index = 0x3004U},     /* encoderRaw：磁编码器采样值，int16（手册示例 11396） */
    {.index = 0x3015U},     /* modPos：电机未计圈机械角度，float，rad（手册示例 4.363409） */
    {.index = 0x3014U},     /* rotation：圈数，int16 */
    {.index = 0x2005U},     /* MechOffset：磁编码器角度偏置，float（手册示例 4.619583） */
};
#define RS05_TEST_PARAM_COUNT       (sizeof(rs05_test_params) / sizeof(rs05_test_params[0]))

static RS05_Manager_t robot_can1;
/* 驱动的电机。id 也是这个测试读参数用的 ID，外面改了 id，电机帧和读参数一起换过去 */
static RS05_Motor_t rs05_motor;
/* 外面改：rs05_test_enable 为 1 时进电流模式、输出 rs05_test_current_a（A）；为 0 时停止，电机没力 */
static volatile uint8_t rs05_test_enable;
static volatile float rs05_test_current_a;
/* 收到的帧，驱动和读参数已经用掉的（现在这个电机的类型 2、17）不记：环形记录，rs05_test_rx_count 是记过的总数，
   最新一帧在 rs05_test_rx_log[(rs05_test_rx_count - 1) % 16]。接收中断里写 */
static volatile TestFrame_t rs05_test_rx_log[RS05_TEST_RX_LOG_SIZE];
static volatile uint32_t rs05_test_rx_count;
/* 外面要发的一帧：先填 rs05_test_tx_frame，再把 rs05_test_tx_pending 置 1。程序发进邮箱后清 0；
   ID 或 DLC 不合法发不了时改成 2 */
static volatile TestFrame_t rs05_test_tx_frame;
static volatile uint8_t rs05_test_tx_pending;
static uint8_t rs05_test_param_next;        /* 下一个要读的参数在 rs05_test_params 里的下标 */
static uint32_t robot_last_tick;

/* 类型 17：ID = 17 | 本机 ID(bit15~8) | 电机 ID；Byte0~1 是 index（低字节在前），其余为 0 */
static HAL_StatusTypeDef TestParam_SendRead(uint8_t motor_id, uint16_t index)
{
    uint8_t data[8] = {0};
    uint32_t ext_id = ((uint32_t)RS05_TEST_TYPE_READ << 24) | ((uint32_t)RS05_TEST_HOST_ID << 8) | motor_id;

    data[0] = (uint8_t)index;
    data[1] = (uint8_t)(index >> 8);
    return CAN_Bus_SendExtData(&hcan1, ext_id, data, 8U);
}

/* 类型 17 应答：bit23~16 为 00 读取成功、01 读取失败，bit15~8 是电机 ID；Byte0~1 是 index，Byte4~7 是数值 */
static void TestParam_ProcessReply(const CAN_RxHeaderTypeDef *header, const uint8_t data[8])
{
    uint16_t index;

    if ((header->IDE != CAN_ID_EXT) || (header->RTR != CAN_RTR_DATA) || (header->DLC != 8U) ||
        (((header->ExtId >> 24) & 0x1FU) != RS05_TEST_TYPE_READ) ||
        ((uint8_t)(header->ExtId >> 8) != rs05_motor.id))
    {
        return;
    }

    index = (uint16_t)(data[0] | ((uint16_t)data[1] << 8));
    for (uint8_t i = 0U; i < RS05_TEST_PARAM_COUNT; i++)
    {
        volatile TestParam_t *param = &rs05_test_params[i];

        if (param->index != index)
        {
            continue;
        }
        if (((header->ExtId >> 16) & 0xFFU) == 0U)
        {
            param->raw = (uint32_t)data[4] | ((uint32_t)data[5] << 8) | ((uint32_t)data[6] << 16) |
                         ((uint32_t)data[7] << 24);
            param->ok_count++;
        }
        else
        {
            param->fail_count++;
        }
    }
}

/* 驱动和读参数用不到的帧都记下来。现在这个电机的类型 2、17 已经用掉了，不记，免得 1 kHz 的反馈把别的帧冲掉 */
static void TestLog_Add(const CAN_RxHeaderTypeDef *header, const uint8_t data[8])
{
    volatile TestFrame_t *entry;

    if (header->IDE == CAN_ID_EXT)
    {
        uint32_t type = (header->ExtId >> 24) & 0x1FU;

        if (((type == RS05_TEST_TYPE_FEEDBACK) || (type == RS05_TEST_TYPE_READ)) &&
            ((uint8_t)(header->ExtId >> 8) == rs05_motor.id))
        {
            return;
        }
    }

    entry = &rs05_test_rx_log[rs05_test_rx_count % RS05_TEST_RX_LOG_SIZE];
    entry->id = (header->IDE == CAN_ID_EXT) ? (header->ExtId | RS05_TEST_FRAME_EXT) : header->StdId;
    entry->dlc = (uint8_t)header->DLC;
    for (uint8_t i = 0U; i < 8U; i++)
    {
        entry->data[i] = data[i];
    }
    rs05_test_rx_count++;
}

/* 发外面填的那一帧 */
static void TestFrame_SendPending(void)
{
    uint8_t data[8];
    uint32_t id = rs05_test_tx_frame.id;
    uint8_t dlc = rs05_test_tx_frame.dlc;
    HAL_StatusTypeDef status;

    for (uint8_t i = 0U; i < 8U; i++)
    {
        data[i] = rs05_test_tx_frame.data[i];
    }
    if ((id & RS05_TEST_FRAME_EXT) != 0U)
    {
        status = CAN_Bus_SendExtData(&hcan1, id & 0x1FFFFFFFUL, data, dlc);
    }
    else
    {
        status = (id <= 0x7FFU) ? CAN_Bus_SendStdData(&hcan1, (uint16_t)id, data, dlc) : HAL_ERROR;
    }

    if (status == HAL_OK)
    {
        rs05_test_tx_pending = 0U;
    }
    else if (status == HAL_ERROR)
    {
        rs05_test_tx_pending = 2U;      /* ID 或 DLC 不合法；邮箱满（HAL_BUSY）就下次再发 */
    }
}

void Robot_Init(void)
{
    if ((RS05_Manager_Init(&robot_can1, &hcan1) != HAL_OK) ||
        (RS05_Register(&robot_can1, &rs05_motor, RS05_TEST_MOTOR_ID) != HAL_OK))
    {
        Error_Handler();
    }

    /* 什么帧都收（ID 掩码全 0，只比较 IDE、RTR）。然后 RS05_Manager_Start 配类型 2 的过滤器（组 1），
       启动 CAN1 和接收中断 */
    if ((CAN_Bus_ConfigStdIdMask(&hcan1, RS05_TEST_STD_FILTER_BANK, 14U, 0U, 0U) != HAL_OK) ||
        (CAN_Bus_ConfigExtIdMask(&hcan1, RS05_TEST_EXT_FILTER_BANK, 14U, 0U, 0U) != HAL_OK) ||
        (RS05_Manager_Start(&robot_can1) != HAL_OK))
    {
        Error_Handler();
    }

    robot_last_tick = HAL_GetTick();
}

void Robot_Loop(void)
{
    uint32_t now = HAL_GetTick();
    float current;

    if (now == robot_last_tick)
    {
        return;
    }
    robot_last_tick = now;

    /* 1. 电机：每 1 ms 一帧（驱动按 enable 发停止帧、使能序列或 iq_ref），反馈全靠这一帧的应答。
          实际发出去的电流就是 rs05_motor.current_a */
    current = Math_Clamp(rs05_test_current_a, -RS05_TEST_CURRENT_LIMIT_A, RS05_TEST_CURRENT_LIMIT_A);
    if (fabsf(rs05_motor.velocity_rad_s) > RS05_TEST_SPEED_LIMIT_RAD_S)
    {
        current = 0.0f;
    }
    RS05_SetCurrent(&rs05_motor, rs05_test_enable, current);
    (void)RS05_Manager_SendAll(&robot_can1);

    /* 2. 读参数：放在电机帧后面，邮箱不够时先让电机帧进去；这一个没进邮箱就下次再读它 */
    if ((now % RS05_TEST_READ_PERIOD_MS) == 0U)
    {
        if (TestParam_SendRead(rs05_motor.id, rs05_test_params[rs05_test_param_next].index) == HAL_OK)
        {
            rs05_test_param_next = (uint8_t)((rs05_test_param_next + 1U) % RS05_TEST_PARAM_COUNT);
        }
    }

    /* 3. 外面填的那一帧 */
    if (rs05_test_tx_pending == 1U)
    {
        TestFrame_SendPending();
    }
}

/* HAL 的 CAN 接收回调（本工程只有 CAN1）：每次读一帧，类型 2 交给驱动，类型 17 交给读参数（各自按通信类型
   挑自己的帧），剩下的记进 rs05_test_rx_log */
void HAL_CAN_RxFifo0MsgPendingCallback(CAN_HandleTypeDef *hcan)
{
    CAN_RxHeaderTypeDef header;
    uint8_t data[8];

    if (CAN_Bus_ReadFifo0(hcan, &header, data) != HAL_OK)
    {
        return;
    }

    RS05_Manager_ProcessRxFrame(&robot_can1, &header, data);
    TestParam_ProcessReply(&header, data);
    TestLog_Add(&header, data);
}
