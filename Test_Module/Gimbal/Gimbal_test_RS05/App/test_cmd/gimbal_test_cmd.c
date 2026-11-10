#include "gimbal_test_cmd.h"

/* 没写到的字段为 0：无力、目标 0°、不加方波 */
volatile GimbalTest_t gimbal_test = {.square_period_ms = 2000U};
static GimbalMode_e gimbal_test_last_mode;

void GimbalTestCmd_Update(uint32_t now_tick, const GimbalFeedback_t *feedback,
                          GimbalCmd_t *gimbal_cmd, ChassisCmd_t *chassis_cmd)
{
    GimbalMode_e mode = gimbal_test.gimbal_mode;
    ChassisMode_e chassis_mode = gimbal_test.chassis_mode;
    uint8_t square_high = 0U;

    /* 刚切模式：目标同步成新模式下的当前角，免得一进闭环云台就转去旧目标 */
    if (mode != gimbal_test_last_mode)
    {
        if (mode == GIMBAL_MODE_IMU)
        {
            gimbal_test.yaw_deg = feedback->yaw_imu_deg;
            gimbal_test.pitch_deg = feedback->pitch_imu_deg;
        }
        else
        {
            gimbal_test.yaw_deg = feedback->yaw_rel_deg;
            gimbal_test.pitch_deg = feedback->pitch_rel_deg;
        }
        /* 模式 3 的 yaw 目标是 yaw_deg + C 板 yaw：先减掉现在的 C 板 yaw，进模式 3 时电机不动 */
        if (mode == GIMBAL_TEST_MODE_FOLLOW_BOARD)
        {
            gimbal_test.yaw_deg -= feedback->yaw_imu_deg;
        }
        gimbal_test_last_mode = mode;
    }

    /* 方波：后半个周期加上幅值 */
    if (gimbal_test.square_period_ms >= 2U)
    {
        square_high = (uint8_t)((now_tick / (gimbal_test.square_period_ms / 2U)) % 2U);
    }

    gimbal_cmd->mode = mode;
    gimbal_cmd->yaw_deg = gimbal_test.yaw_deg + ((square_high != 0U) ? gimbal_test.yaw_square_deg : 0.0f);
    gimbal_cmd->pitch_deg = gimbal_test.pitch_deg + ((square_high != 0U) ? gimbal_test.pitch_square_deg : 0.0f);

    /* 模式 3：云台按编码器闭环跑，yaw 目标再加上 C 板的 yaw。ins 标定完以前 C 板 yaw 一直是 0（标定完从 0 开始），
       电机停在原地。斜坡还是限 100°/s：C 板转得更快时电机落在后面，C 板停下后追上；
       落后超过 180° 时电机会掉头走近路（yaw 不限位，按最短方向走），最后朝向一样 */
    if (mode == GIMBAL_TEST_MODE_FOLLOW_BOARD)
    {
        gimbal_cmd->mode = GIMBAL_MODE_ENCODER;
        gimbal_cmd->yaw_deg += feedback->yaw_imu_deg;
    }

    /* 跟随、小陀螺要云台实际在 IMU 模式（对地稳住），不然底盘会一直转、平移方向跟着转。
       看 feedback->mode 不看指令：云台被锁在 RELAX 时指令还是 IMU。只降级发出去的指令，
       不改 gimbal_test 里的输入，云台进了 IMU 自动恢复。 */
    if ((feedback->mode != GIMBAL_MODE_IMU) &&
        ((chassis_mode == CHASSIS_MODE_FOLLOW) || (chassis_mode == CHASSIS_MODE_SPIN)))
    {
        chassis_mode = CHASSIS_MODE_NO_FOLLOW;
    }

    chassis_cmd->mode = chassis_mode;
    chassis_cmd->vx_mps = gimbal_test.vx_mps;
    chassis_cmd->vy_mps = gimbal_test.vy_mps;
    chassis_cmd->gimbal_yaw_rel_deg = feedback->yaw_rel_deg;    /* 底盘板用它把平移转到底盘坐标系，跟随也用它 */
}
