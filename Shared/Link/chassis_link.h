#ifndef CHASSIS_LINK_H
#define CHASSIS_LINK_H

#include "main.h"

/*
 * 板间通信（双板方案）：云台主控把底盘指令发给底盘板。
 * 两块板编译同一份文件，模式编号和帧格式只在这里定义；改了两个工程都要重编、两块板都要重烧。
 * 帧格式：标准数据帧 0x110，DLC 8，每 1 ms 一帧，高字节在前（和大疆电机帧一样）
 *   DATA[0]     mode（ChassisMode_e）
 *   DATA[1]     保留，发 0
 *   DATA[2..3]  vx，int16，mm/s
 *   DATA[4..5]  vy，int16，mm/s
 *   DATA[6..7]  gimbal_yaw_rel，int16，0.01°
 */
#define CHASSIS_LINK_CMD_ID         0x110U
#define CHASSIS_LINK_TIMEOUT_MS     50U     /* 超过这么久没收到新帧就当 RELAX */

/*底盘模式，由云台主控配置，板间通信（CAN）传过来*/
typedef enum
{
    CHASSIS_MODE_RELAX = 0,     /* 无力：上电默认；板间通信超时、电机掉线时也是这个 */
    CHASSIS_MODE_NO_FOLLOW,     /* 不跟随：底盘不转，只按云台朝向平移 */
    CHASSIS_MODE_FOLLOW,        /* 跟随云台：底盘转到和云台同向 */
    CHASSIS_MODE_SPIN           /* 小陀螺：底盘按固定角速度一直转，平移仍按云台朝向 */
} ChassisMode_e;                /* 数值也是板间帧的内容，两块板要一致，别随便调顺序 */

typedef struct
{
    ChassisMode_e mode;         /*云台主控配置的底盘模式*/
    float vx_mps;               /* 平移速度，在云台朝向的坐标系里：沿枪口为正 */
    float vy_mps;               /* 枪口左边为正 */
    float gimbal_yaw_rel_deg;   /* 云台相对底盘正前方的角度，来自云台的 yaw_rel_deg */
} ChassisCmd_t;

/* ---- 底盘板调 ---- */

/* 给 0x110 配一个过滤器组：CAN1 用组 2、CAN2 用组 16（组的分配见 can_bus.h）。CAN 启动前后调都行。 */
HAL_StatusTypeDef ChassisLink_Init(CAN_HandleTypeDef *hcan);
/* 在 CAN 接收回调（中断）里调：这路上读到的每一帧都交给它，它只认 0x110，只存原始数据和时间。 */
void ChassisLink_ProcessRxFrame(const CAN_RxHeaderTypeDef *header, const uint8_t data[8]);
/* 主循环每 1 ms 调：解出最新一帧；超时（包括上电后还没收到过）给 RELAX、速度 0。 */
void ChassisLink_GetCmd(uint32_t now_tick, ChassisCmd_t *cmd);

/* ---- 云台板调 ---- */

/* 主循环每 1 ms 调一次，放在各路 SendAll 之后；hcan 传两块板之间那路（云台板默认 CAN1）。按上面的帧格式打包发出。
   只发不收，不用 Init：CAN 已由 DJI_MotorManager_Start 启动。速度、角度先限到 int16 装得下的范围再转换。
   cmd 为 NULL 返回 HAL_ERROR；其余返回 CAN_Bus_SendStdData 的结果，3 个发送邮箱都满时是 HAL_BUSY（这一帧丢掉，下一毫秒照常发）。 */
HAL_StatusTypeDef ChassisLink_SendCmd(CAN_HandleTypeDef *hcan, const ChassisCmd_t *cmd);

#endif /* CHASSIS_LINK_H */
