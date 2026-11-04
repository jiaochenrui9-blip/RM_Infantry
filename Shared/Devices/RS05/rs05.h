#ifndef RS05_H
#define RS05_H

#include "main.h"

/*
 * 灵足 RS05 电机驱动：私有协议，只用电流模式（run_mode = 3，写 iq_ref）。手册：RS05 使用说明书第 4 章。
 * 帧格式：扩展数据帧，1 Mbps，DLC 8；29 位 ID = 通信类型(bit28~24) | 数据区 2(bit23~8) | 目标地址(bit7~0)。
 *
 * 电机只在收到指令时回一帧类型 2 反馈（主动上报最快 10 ms，不用），所以每个电机每 1 ms 都要发一帧：
 *   不出力    → 停止（类型 4），电机进 Reset、没有力，照样回反馈
 *   出力、电机在运行 → 写 iq_ref
 *   出力、电机没在运行（刚上电、被 canTimeout 复位） → 每周期一步：写 run_mode = 3 → 写 canTimeout
 *                     → 写 iq_ref = 0 → 使能，4 ms 进电流模式；没进邮箱的那步下周期重发，不跳步；
 *                     电机 2 ms 内没回反馈就从第一步重来（电机半路才上电时，先收到的一定是 run_mode）
 *
 * 这一路 CAN 要开自动重发（.ioc 里 Automatic Retransmission = Enable）：电机的应答比主机发的帧都优先，
 * 关着时主机的帧和应答同时起步就会输掉仲裁、直接丢，使能序列里丢了帧驱动也发现不了。
 *
 * 用法和 DJI manager 一样：一路 CAN 一个 manager，由装配层持有；模块把电机注册进去；
 * 装配层每周期调一次 RS05_Manager_SendAll，CAN 接收回调里把读到的每一帧交给 RS05_Manager_ProcessRxFrame。
 * 驱动只给手册的单位（rad、rad/s、A）和电机自己的正方向；方向、中值由用它的模块处理。
 */

/* 一路最多几个：SendAll 每周期给每个电机往发送邮箱放一帧，F4 每路 CAN 只有 3 个邮箱；
   同一路还要发别的帧（如板间 0x110）时更少 */
#define RS05_MOTOR_MAX  3U

typedef struct
{
    uint8_t id;                 /* 电机 CAN ID（上位机或通信类型 7 设置），注册时写入 */
    uint8_t enable;             /* 1：电流模式出力；0：停止 */
    float current_a;            /* iq 指令，A（手册范围 ±16，最大相电流 11 A），限幅由调用者做 */
    uint8_t enable_step;        /* 使能序列下一步：0 写 run_mode、1 写 canTimeout、2 写 iq_ref = 0、3 使能 */

    /* 反馈，接收中断里写、主循环直接读：每个字段是一次 32 位以内的写，读不到半个值，
       角度和角速度最多差一帧，控制上没影响 */
    float position_rad;         /* 输出轴角度，-4π~4π 循环（手册 4.2.3），上电时在 0~2π */
    float velocity_rad_s;       /* 输出轴角速度 */
    uint8_t state;              /* 0 Reset（停止）、1 标定、2 运行 */
    uint8_t fault;              /* bit0 欠压、1 三相电流、2 过温、3 磁编码、4 堵转过载、5 未标定 */
    uint32_t feedback_tick;     /* 最后一次反馈的 HAL tick，0 表示还没收到过 */
} RS05_Motor_t;

typedef struct
{
    CAN_HandleTypeDef *hcan;
    RS05_Motor_t *motors[RS05_MOTOR_MAX];
    uint8_t count;
} RS05_Manager_t;

HAL_StatusTypeDef RS05_Manager_Init(RS05_Manager_t *manager, CAN_HandleTypeDef *hcan);
/* 在 RS05_Manager_Start() 之前注册。会清空 motor；id 超过 127、和已注册的重复、满了都返回 HAL_ERROR。 */
HAL_StatusTypeDef RS05_Register(RS05_Manager_t *manager, RS05_Motor_t *motor, uint8_t id);
/* 配一个只收类型 2（反馈）的扩展帧过滤器：CAN1 用组 1、CAN2 用组 15（见 can_bus.h），再启动 CAN 和接收中断。 */
HAL_StatusTypeDef RS05_Manager_Start(RS05_Manager_t *manager);
/* 每 1 ms 调一次：每个电机发一帧（见文件开头）。一个电机没发出去不影响后面的，返回第一个出错的状态；
   3 个发送邮箱都满时是 HAL_BUSY。 */
HAL_StatusTypeDef RS05_Manager_SendAll(RS05_Manager_t *manager);
/* 在 CAN 接收回调（中断）里调：只处理已注册电机的类型 2 反馈，标准帧、别的类型直接忽略。 */
void RS05_Manager_ProcessRxFrame(RS05_Manager_t *manager, const CAN_RxHeaderTypeDef *header,
                                 const uint8_t data[8]);

/* enable 为 0：停止，current_a 不用；为 1：进电流模式并输出 current_a（A）。下一次 SendAll 生效。 */
void RS05_SetCurrent(RS05_Motor_t *motor, uint8_t enable, float current_a);
/* timeout_ms 内收到过反馈，并且没有故障位，返回 1。 */
uint8_t RS05_IsOnline(const RS05_Motor_t *motor, uint32_t now_tick, uint32_t timeout_ms);

#endif /* RS05_H */
