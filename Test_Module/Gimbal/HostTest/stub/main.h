#ifndef HOST_STUB_MAIN_H
#define HOST_STUB_MAIN_H

/*
 * 主机测试用的假 main.h：只提供被测代码用到的 HAL 类型和函数，不是真的 HAL。
 * 真工程的 main.h 在 Gimbal_test/Core/Inc，会拉进整个 STM32 HAL，电脑上编译不了。
 * 驱动以后用到新的 HAL 符号时，在这里补上对应的类型 / 宏，编译报错会告诉你缺什么。
 */

#include <stddef.h>
#include <stdint.h>

typedef enum
{
    HAL_OK = 0x00U,
    HAL_ERROR = 0x01U,
    HAL_BUSY = 0x02U,
    HAL_TIMEOUT = 0x03U
} HAL_StatusTypeDef;

/* CAN：驱动只用到句柄里的 Instance（区分 CAN1 / CAN2）和接收帧头的几个字段 */
typedef struct
{
    uint32_t dummy;
} CAN_TypeDef;

extern CAN_TypeDef sim_can1_regs;
extern CAN_TypeDef sim_can2_regs;
#define CAN1 (&sim_can1_regs)
#define CAN2 (&sim_can2_regs)

typedef struct
{
    CAN_TypeDef *Instance;
} CAN_HandleTypeDef;

typedef struct
{
    uint32_t StdId;
    uint32_t ExtId;
    uint32_t IDE;
    uint32_t RTR;
    uint32_t DLC;
    uint32_t Timestamp;
    uint32_t FilterMatchIndex;
} CAN_RxHeaderTypeDef;

#define CAN_ID_STD      0x00000000U
#define CAN_ID_EXT      0x00000004U
#define CAN_RTR_DATA    0x00000000U
#define CAN_RTR_REMOTE  0x00000002U

extern CAN_HandleTypeDef hcan1;
extern CAN_HandleTypeDef hcan2;

/* ins.h 的函数声明里用到（BMI088 的 SPI、加热 PWM 的定时器），主机测试不编译 ins.c */
typedef struct
{
    uint32_t dummy;
} SPI_HandleTypeDef;

typedef struct
{
    uint32_t dummy;
} TIM_HandleTypeDef;

/* 仿真时间，由 sim/gimbal_sim.c 提供：每跑一个控制周期加 1 */
uint32_t HAL_GetTick(void);

/* 电脑上没有中断，开关中断什么都不做 */
static inline uint32_t __get_PRIMASK(void)
{
    return 0U;
}

static inline void __disable_irq(void)
{
}

static inline void __enable_irq(void)
{
}

static inline void __set_PRIMASK(uint32_t primask)
{
    (void)primask;
}

#endif /* HOST_STUB_MAIN_H */
