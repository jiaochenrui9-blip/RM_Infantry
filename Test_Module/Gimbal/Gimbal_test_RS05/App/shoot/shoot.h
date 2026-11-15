#ifndef SHOOT_H
#define SHOOT_H

#include "dji_motor_manager.h"

/* 指令和反馈共用这一套模式。
   - 指令里是“想要什么”；反馈里是“实际在什么”，由 shoot.c 根据在线、故障锁、摩擦轮就绪决定。
   - 指令 FEED 时，摩擦轮没就绪（或刚掉出就绪）反馈是 SPINUP，拨弹轮不转；就绪以后才是 FEED。 */
typedef enum
{
    SHOOT_MODE_RELAX = 0,    /* 三台电机无力（电机层 DISABLE，输出 0）；故障锁住时也是它 */
    SHOOT_MODE_SPINUP,       /* 摩擦轮速度环转到 friction_rpm；拨弹轮速度环、目标 0（顶住不让弹丸被推进去） */
    SHOOT_MODE_FEED          /* 摩擦轮照常；拨弹轮速度环转到 feeder_rpm */
} ShootMode_e;

typedef struct
{
    ShootMode_e mode;        /* 请求的模式 */
    int16_t friction_rpm;    /* 两个摩擦轮共用的目标，转子 rpm，逻辑正向 = 往外送弹；≤ 0 时不算就绪，不会拨弹 */
    int16_t feeder_rpm;      /* 拨弹轮目标，转子 rpm，逻辑正向 = 进弹；只在实际模式是 FEED 时用 */
} ShootCmd_t;

typedef struct
{
    ShootMode_e mode;        /* 实际模式，见上面 ShootMode_e 的说明 */
} ShootFeedback_t;

/* 注册并初始化三台电机（都在 DISABLE），必须在这路 manager Start 之前调用一次。 */
HAL_StatusTypeDef Shoot_Init(DJI_MotorManager_t *bus);
/* 每周期复制一次指令；空指针按 RELAX 处理。 */
void Shoot_SetCmd(const ShootCmd_t *cmd);
/* 每 1 ms 调用：定实际模式 → 给目标 → 三台电机的速度环。不发 CAN，发送由 robot.c 的 SendAll 统一做。 */
void Shoot_Update(uint32_t now_tick);
void Shoot_GetFeedback(ShootFeedback_t *feedback);

#endif
