#ifndef GIMBAL_TEST_CMD_H
#define GIMBAL_TEST_CMD_H

#include "gimbal.h"

/*
 * 测试工程专用的指令源：在调试器里改 gimbal_test 给云台下指令。
 * 合并工程里换成 robot_cmd（遥控器 / 键鼠 / 视觉），云台模块不用改。
 */
typedef struct
{
    GimbalMode_e mode;          /* 0 无力 / 1 编码器闭环 */
    float yaw_deg;              /* 目标角 */
    float pitch_deg;
    float yaw_square_deg;       /* 方波幅值：非 0 时目标在“目标角”和“目标角 + 幅值”之间来回切，看阶跃响应用 */
    float pitch_square_deg;
    uint32_t square_period_ms;
} GimbalTest_t;

/* 改 mode 时目标会自动同步成当前角，所以先切模式、再改目标。 */
extern volatile GimbalTest_t gimbal_test;

void GimbalTestCmd_Update(uint32_t now_tick, const GimbalFeedback_t *feedback, GimbalCmd_t *cmd);

#endif /* GIMBAL_TEST_CMD_H */
