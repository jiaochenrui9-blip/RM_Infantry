#ifndef TEST_MAIN_H
#define TEST_MAIN_H

#include <stdint.h>
#include <stddef.h>

/* 主机测试只替换 HAL 边界，编译的 BSP 和 Power 实现仍是工程真代码。 */
typedef enum { HAL_OK, HAL_ERROR, HAL_BUSY, HAL_TIMEOUT } HAL_StatusTypeDef;
typedef struct { uint32_t instance; } I2C_HandleTypeDef;
#define I2C_MEMADD_SIZE_8BIT 1U

HAL_StatusTypeDef HAL_I2C_Mem_Read(I2C_HandleTypeDef *, uint16_t, uint16_t,
                                  uint16_t, uint8_t *, uint16_t, uint32_t);
HAL_StatusTypeDef HAL_I2C_Mem_Write(I2C_HandleTypeDef *, uint16_t, uint16_t,
                                   uint16_t, uint8_t *, uint16_t, uint32_t);
uint32_t HAL_GetTick(void);

/* 模拟只有 CAN1 的 F103，故意不定义 CAN2。 */
#define CAN1 ((void *)1)
#define CAN_ID_STD 0U
#define CAN_ID_EXT 4U
#define CAN_RTR_DATA 0U
#define CAN_RTR_REMOTE 2U
#define CAN_RX_FIFO0 0U
typedef struct { void *Instance; } CAN_HandleTypeDef;
typedef struct {
    uint32_t StdId, ExtId, IDE, RTR, DLC;
} CAN_RxHeaderTypeDef;
uint32_t HAL_CAN_GetRxFifoFillLevel(CAN_HandleTypeDef *, uint32_t);
void HAL_CAN_RxFifo0MsgPendingCallback(CAN_HandleTypeDef *);
static inline uint32_t __get_PRIMASK(void) { return 0U; }
static inline void __disable_irq(void) {}
static inline void __enable_irq(void) {}

#endif
