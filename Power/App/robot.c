#include "robot.h"
#include "Power.h"
#include "Power_config.h"
#include "i2c.h"
#include "can.h"
#include "dji_m3508.h"
#include "motor_control.h"
#include "motor_config.h"

static uint32_t s_last_update;
static DJI_MotorManager_t s_motor_manager;
static DJI_Motor_t s_motor;
static volatile uint8_t s_motor_ready;

void Robot_Init(void)
{
    s_motor_ready = 0U;
    /* 初始化结果保存在 Power 反馈中，失败时修复连接后重新初始化。 */
    (void)Power_Init(&hi2c1);
    s_last_update = HAL_GetTick();

    if ((DJI_MotorManager_Init(&s_motor_manager, &hcan) != HAL_OK) ||
        (DJI_M3508_Register(&s_motor_manager, &s_motor, POWER_MOTOR_ID,
                            DJI_MOTOR_CONTROL_ID_1_4, 0U) != HAL_OK) ||
        (MotorControl_Init(&s_motor, POWER_MOTOR_CURRENT_LIMIT,
                           POWER_MOTOR_DIRECTION, POWER_MOTOR_SPEED_KP,
                           POWER_MOTOR_SPEED_KI, POWER_MOTOR_SPEED_KD,
                           0.0f, 0.0f, 0.0f) != HAL_OK)) {
        return;
    }
    MotorControl_SetMode(&s_motor, MOTOR_MODE_SPEED);
    MotorControl_SetTargetSpeed(&s_motor, POWER_MOTOR_TARGET_RPM);
    if (DJI_MotorManager_Start(&s_motor_manager) == HAL_OK) {
        s_motor_ready = 1U;
    }
}

void Robot_ControlTick(void)
{
    if (s_motor_ready == 0U) {
        return;
    }
    /* 未收到反馈或超过 100 ms 未更新时，共享控制器将电流输出清零。 */
    MotorControl_Update(&s_motor, HAL_GetTick());
    /* 邮箱忙时不等待，下个 1 ms 周期发送最新控制量。 */
    (void)DJI_MotorManager_SendAll(&s_motor_manager);
}

void HAL_CAN_RxFifo0MsgPendingCallback(CAN_HandleTypeDef *can)
{
    CAN_RxHeaderTypeDef header;
    uint8_t data[8];
    if (can != &hcan) {
        return;
    }
    while (HAL_CAN_GetRxFifoFillLevel(can, CAN_RX_FIFO0) > 0U) {
        if (CAN_Bus_ReadFifo0(can, &header, data) != HAL_OK) {
            break;
        }
        DJI_MotorManager_ProcessRxFrame(&s_motor_manager, &header, data);
    }
}

void Robot_Loop(void)
{
    uint32_t now = HAL_GetTick();
    if ((uint32_t)(now - s_last_update) < POWER_UPDATE_PERIOD_MS) {
        return;
    }
    s_last_update = now;
    (void)Power_Update(now);
}
