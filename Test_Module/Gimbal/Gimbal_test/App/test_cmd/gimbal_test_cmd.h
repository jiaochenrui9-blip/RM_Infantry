#ifndef GIMBAL_TEST_CMD_H
#define GIMBAL_TEST_CMD_H

#include "gimbal.h"
#include "chassis_link.h"

/*
 * 测试工程专用的指令源：在调试器里改 gimbal_test 给云台和底盘下指令（底盘指令经板间 CAN 发给底盘板）。
 * 合并工程里换成 robot_cmd（遥控器 / 键鼠 / 视觉），云台模块不用改。
 */
typedef struct
{
    GimbalMode_e gimbal_mode;   /* 0 无力 / 1 编码器闭环 / 2 IMU 闭环 */
    float yaw_deg;              /* 目标角：模式 1 相对底盘，模式 2 相对地面 */
    float pitch_deg;

    ChassisMode_e chassis_mode; /* 底盘模式：0 无力 / 1 不跟随 / 2 跟随 / 3 小陀螺；2、3 要云台实际在 IMU 模式，否则发出去的是 1 */
    float vx_mps;               /* 底盘平移速度，在云台朝向的坐标系里：沿枪口为正，m/s */
    float vy_mps;               /* 枪口左边为正，m/s */

    float yaw_square_deg;       /* 方波幅值：非 0 时目标在“目标角”和“目标角 + 幅值”之间来回切，看阶跃响应用 */
    float pitch_square_deg;
    uint32_t square_period_ms;
} GimbalTest_t;

/* 改 gimbal_mode 时目标会自动同步成新模式下的当前角，所以先切模式、再改目标。 */
extern volatile GimbalTest_t gimbal_test;

void GimbalTestCmd_Update(uint32_t now_tick, const GimbalFeedback_t *feedback,
                          GimbalCmd_t *gimbal_cmd, ChassisCmd_t *chassis_cmd);

#endif /* GIMBAL_TEST_CMD_H */
