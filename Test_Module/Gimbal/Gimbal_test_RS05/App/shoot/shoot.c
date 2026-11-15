#include "shoot.h"

#include <stdlib.h>

#include "shoot_config.h"
#include "dji_m3508.h"
#include "dji_m2006.h"
#include "motor_control.h"

/* 电机已有方向、目标、反馈和 PID，不在应用层重复保存。 */
static DJI_Motor_t shoot_friction_1;
static DJI_Motor_t shoot_friction_2;
static DJI_Motor_t shoot_feeder;
static ShootCmd_t shoot_cmd;
static ShootMode_e shoot_mode;
static uint8_t shoot_fault;    /* 电机掉线或故障 → 锁在 RELAX，指令给一次 RELAX 才解除 */
static uint32_t friction_last_not_ready_tick;    /* 摩擦轮不就绪的最后一次 tick，计时用 */
HAL_StatusTypeDef Shoot_Init(DJI_MotorManager_t *bus)
{
    shoot_cmd = (ShootCmd_t){0};
    shoot_mode = SHOOT_MODE_RELAX;

    if ((DJI_M3508_Register(bus, &shoot_friction_1, SHOOT_FRICTION_1_ESC_ID,
                           DJI_MOTOR_CONTROL_ID_1_4, SHOOT_FRICTION_1_ESC_ID - 1U) != HAL_OK) ||
        (DJI_M3508_Register(bus, &shoot_friction_2, SHOOT_FRICTION_2_ESC_ID,
                           DJI_MOTOR_CONTROL_ID_1_4, SHOOT_FRICTION_2_ESC_ID - 1U) != HAL_OK) ||
        (DJI_M2006_Register(bus, &shoot_feeder, SHOOT_FEEDER_ESC_ID,
                           DJI_MOTOR_CONTROL_ID_1_4, SHOOT_FEEDER_ESC_ID - 1U) != HAL_OK))
    {
        return HAL_ERROR;
    }

    /* 注册后默认失能。只初始化速度环，位置环参数为零；减速比沿用驱动默认值，尚未核对机械。 */
    if ((MotorControl_Init(&shoot_friction_1, SHOOT_FRICTION_CURRENT_LIMIT,
                           SHOOT_FRICTION_1_DIRECTION,
                           SHOOT_FRICTION_SPEED_KP, SHOOT_FRICTION_SPEED_KI, SHOOT_FRICTION_SPEED_KD,
                           0.0f, 0.0f, 0.0f) != HAL_OK) ||
        (MotorControl_Init(&shoot_friction_2, SHOOT_FRICTION_CURRENT_LIMIT,
                           SHOOT_FRICTION_2_DIRECTION,
                           SHOOT_FRICTION_SPEED_KP, SHOOT_FRICTION_SPEED_KI, SHOOT_FRICTION_SPEED_KD,
                           0.0f, 0.0f, 0.0f) != HAL_OK) ||
        (MotorControl_Init(&shoot_feeder, SHOOT_FEEDER_CURRENT_LIMIT,
                           SHOOT_FEEDER_DIRECTION,
                           SHOOT_FEEDER_SPEED_KP, SHOOT_FEEDER_SPEED_KI, SHOOT_FEEDER_SPEED_KD,
                           0.0f, 0.0f, 0.0f) != HAL_OK))
    {
        return HAL_ERROR;
    }

    return HAL_OK;
}

void Shoot_SetCmd(const ShootCmd_t *cmd)
{
    shoot_cmd = (cmd != NULL) ? *cmd : (ShootCmd_t){0};
}

/* ---- 控制逻辑的约定（2026-11-15 定）----
 * Shoot_Update 每 1 ms：Shoot_UpdateMode → Shoot_SetTarget → 三个 MotorControl_Update。
 *
 * 1. Shoot_UpdateMode(now_tick)：定实际模式 shoot_mode（反馈给出去的就是它）。
 *    - 任一电机 SHOOT_OFFLINE_TIMEOUT_MS 没反馈 → shoot_fault = 1；指令 RELAX 时清掉；有故障就是 RELAX。
 *    - 指令 FEED，但摩擦轮还没就绪（见 2）→ 实际是 SPINUP；拨弹中掉出就绪也退回 SPINUP。
 *    - 电机模式只在 RELAX ↔ 非 RELAX 切换时改：进 SPINUP / FEED 三台都 MotorControl_SetMode(..., MOTOR_MODE_SPEED)，
 *      回 RELAX 三台都 MOTOR_MODE_DISABLE。SetMode 自己会清两个 PID 和目标。
 *      SPINUP ↔ FEED 不动电机模式，也不要清摩擦轮的 PID（清了每次开拨、停拨摩擦轮转速都会掉一下）。
 * 2. 摩擦轮就绪 Shoot_FrictionReady(now_tick)：
 *    - 条件：shoot_cmd.friction_rpm > 0，两个摩擦轮 |friction_rpm − speed_rpm| < SHOOT_FRICTION_READY_ERR_RPM，
 *      并且已经连续 SHOOT_FRICTION_READY_HOLD_MS。speed_rpm 驱动里已经按方向修正过，直接和逻辑目标比。
 *    - 计时只要一个量：记最后一次“不满足”的 tick，now − 它 ≥ HOLD_MS 就是就绪。
 *    - 不在 SPINUP / FEED（比如 RELAX）时把它当作不满足，下次起转重新计时。
 * 3. Shoot_SetTarget()：按实际模式给目标（MotorControl_SetTargetSpeed，转子 rpm）。
 *    - SPINUP：两个摩擦轮 friction_rpm，拨弹 0；FEED：摩擦轮同上，拨弹 feeder_rpm；RELAX：不用管（DISABLE 会清目标）。
 *    - 方向不用管，MotorControl 里乘 motor_direction。
 */

/*判断摩擦轮是否就绪；非 RELAX 时每周期都要调，计时才连续*/
static uint8_t Shoot_FrictionReady(uint32_t now_tick)
{
    int16_t target_rpm = shoot_cmd.friction_rpm;

    if (target_rpm <= 0 || abs(target_rpm - shoot_friction_1.speed_rpm) >= SHOOT_FRICTION_READY_ERR_RPM ||
        abs(target_rpm - shoot_friction_2.speed_rpm) >= SHOOT_FRICTION_READY_ERR_RPM)
    {
        friction_last_not_ready_tick = now_tick;
        return 0;
    }

    if ((now_tick - friction_last_not_ready_tick) < SHOOT_FRICTION_READY_HOLD_MS)
    {
        return 0;
    }
    return 1;
}
/*更新模式，如果cmd变化则模式变化*/
static void Shoot_UpdateMode(uint32_t now_tick)
{
    uint8_t online = 1;
    ShootMode_e mode = shoot_cmd.mode;

    if ((DJI_Motor_IsOnline(&shoot_friction_1, now_tick, SHOOT_OFFLINE_TIMEOUT_MS) == 0U) ||
        (DJI_Motor_IsOnline(&shoot_friction_2,now_tick, SHOOT_OFFLINE_TIMEOUT_MS) == 0U) ||
        (DJI_Motor_IsOnline(&shoot_feeder, now_tick, SHOOT_OFFLINE_TIMEOUT_MS) == 0U))
    {
        online = 0;
    }
    /*如果当前模式为无力模式则清除故障标志*/
    if (mode == SHOOT_MODE_RELAX)
    {
        shoot_fault = 0;
    }
    /*如果电机掉线就锁住，进入无力模式*/
    if (online == 0)
    {
        shoot_fault = 1;
    }

    if (shoot_fault != 0)
    {
        mode = SHOOT_MODE_RELAX;
    }

    /*先把实际模式算完整再和上一周期比：就绪在非 RELAX 时每周期都算，
      没就绪（包括拨弹中掉出就绪）FEED 退回 SPINUP；RELAX 时当作不满足，下次起转重新计时*/
    if (mode == SHOOT_MODE_RELAX)
    {
        friction_last_not_ready_tick = now_tick;
    }
    else if ((Shoot_FrictionReady(now_tick) == 0U) && (mode == SHOOT_MODE_FEED))
    {
        mode = SHOOT_MODE_SPINUP;
    }

    if (shoot_mode == mode)
    {
        return;
    }

    /*电机模式只在进出 RELAX 时切；SetMode 会清 PID 和目标，SPINUP ↔ FEED 不切，摩擦轮不掉速*/
    if (mode == SHOOT_MODE_RELAX)
    {
        MotorControl_SetMode(&shoot_friction_1,MOTOR_MODE_DISABLE);
        MotorControl_SetMode(&shoot_friction_2,MOTOR_MODE_DISABLE);
        MotorControl_SetMode(&shoot_feeder,MOTOR_MODE_DISABLE);
    }
    else if (shoot_mode == SHOOT_MODE_RELAX)
    {
        MotorControl_SetMode(&shoot_friction_1,MOTOR_MODE_SPEED);
        MotorControl_SetMode(&shoot_friction_2,MOTOR_MODE_SPEED);
        MotorControl_SetMode(&shoot_feeder,MOTOR_MODE_SPEED);
    }

    shoot_mode = mode;
}

/* 按实际模式 shoot_mode 给三台电机的速度目标，见上面约定第 3 条。 */
static void Shoot_SetTarget(void)
{
    /* RELAX 不用给：DISABLE 每周期都清目标 */
    if (shoot_mode == SHOOT_MODE_SPINUP)
    {
        MotorControl_SetTargetSpeed(&shoot_friction_1, shoot_cmd.friction_rpm);
        MotorControl_SetTargetSpeed(&shoot_friction_2, shoot_cmd.friction_rpm);
        MotorControl_SetTargetSpeed(&shoot_feeder, 0);
    }
    else if (shoot_mode == SHOOT_MODE_FEED)
    {
        MotorControl_SetTargetSpeed(&shoot_friction_1, shoot_cmd.friction_rpm);
        MotorControl_SetTargetSpeed(&shoot_friction_2, shoot_cmd.friction_rpm);
        MotorControl_SetTargetSpeed(&shoot_feeder, shoot_cmd.feeder_rpm);
    }
}

void Shoot_Update(uint32_t now_tick)
{
    Shoot_UpdateMode(now_tick);
    Shoot_SetTarget();

    MotorControl_Update(&shoot_friction_1,now_tick);
    MotorControl_Update(&shoot_friction_2,now_tick);
    MotorControl_Update(&shoot_feeder,now_tick);
}

void Shoot_GetFeedback(ShootFeedback_t *feedback)
{
    if (feedback != NULL)
    {
        feedback->mode = shoot_mode;
    }
}
