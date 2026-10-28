#include "gimbal_test_cmd.h"

/* 没写到的字段为 0：无力、目标 0°、不加方波 */
volatile GimbalTest_t gimbal_test = {.square_period_ms = 2000U};
static GimbalMode_e gimbal_test_last_mode;

void GimbalTestCmd_Update(uint32_t now_tick, const GimbalFeedback_t *feedback, GimbalCmd_t *cmd)
{
    GimbalMode_e mode = gimbal_test.mode;
    uint8_t square_high = 0U;

    /* 刚切模式：目标同步成当前角，免得一进闭环云台就转去旧目标 */
    if (mode != gimbal_test_last_mode)
    {
        gimbal_test.yaw_deg = feedback->yaw_deg;
        gimbal_test.pitch_deg = feedback->pitch_deg;
        gimbal_test_last_mode = mode;
    }

    /* 方波：后半个周期加上幅值 */
    if (gimbal_test.square_period_ms >= 2U)
    {
        square_high = (uint8_t)((now_tick / (gimbal_test.square_period_ms / 2U)) % 2U);
    }

    cmd->mode = mode;
    cmd->yaw_deg = gimbal_test.yaw_deg + ((square_high != 0U) ? gimbal_test.yaw_square_deg : 0.0f);
    cmd->pitch_deg = gimbal_test.pitch_deg + ((square_high != 0U) ? gimbal_test.pitch_square_deg : 0.0f);
}
