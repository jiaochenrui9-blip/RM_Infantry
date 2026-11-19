#include "shoot.h"

#include <stdlib.h>
#include <math.h>

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
/* 卡弹状态机只在本文件用，不放 shoot.h */
typedef enum
{
    SHOOT_JAM_NORMAL,     /* 正转 */
    SHOOT_JAM_DETECT,     /* 满足卡弹条件，计时中，还在正转 */
    SHOOT_JAM_REVERSE,    /* 判定卡弹，拨弹轮在位置模式里往回转 SHOOT_JAM_REVERSE_DEG */
} Shoot_JAM_e;

static uint32_t feeder_jam_last_tick;    /* 进入当前卡弹状态的 tick：DETECT 计时、REVERSE 超时、NORMAL 里判 CLEAR_MS 都用它 */
static uint8_t feeder_jam_count;    /* 连续卡弹次数，到 SHOOT_JAM_MAX_COUNT 锁住；正转顺利 CLEAR_MS 或指令 RELAX 清零 */
static Shoot_JAM_e feeder_jam_state;    /* 卡弹状态机 */

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

    /* 注册后默认失能。摩擦轮只用速度环；拨弹轮另有位置环（卡弹反转用）。减速比沿用驱动默认值（M2006 36），尚未核对机械。 */
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
                           SHOOT_FEEDER_POS_KP, SHOOT_FEEDER_POS_KI, SHOOT_FEEDER_POS_KD) != HAL_OK))
    {
        return HAL_ERROR;
    }
    MotorControl_SetPositionSpeedLimit(&shoot_feeder, SHOOT_FEEDER_POS_SPEED_LIMIT);

    return HAL_OK;
}

void Shoot_SetCmd(const ShootCmd_t *cmd)
{
    shoot_cmd = (cmd != NULL) ? *cmd : (ShootCmd_t){0};
}

/* ---- 控制逻辑 ----
 * Shoot_Update 每 1 ms：Shoot_UpdateMode → Shoot_SetTarget → 三个 MotorControl_Update。
 *
 * 1. Shoot_UpdateMode：定实际模式 shoot_mode（反馈给出去的就是它）。
 *    - 任一电机 SHOOT_OFFLINE_TIMEOUT_MS 没反馈、或连续卡弹到 SHOOT_JAM_MAX_COUNT 次 → shoot_fault = 1，有故障就是 RELAX；
 *      指令 RELAX 时清故障和卡弹次数。
 *    - 指令 FEED，但摩擦轮还没就绪（见 2）→ 实际是 SPINUP；拨弹中掉出就绪也退回 SPINUP。
 *    - 电机模式只在 RELAX ↔ 非 RELAX 时切：进 SPINUP / FEED 三台都切 SPEED，回 RELAX 三台都 DISABLE。
 *      SetMode 会清 PID 和目标，所以 SPINUP ↔ FEED 不切（清了每次开拨、停拨摩擦轮转速都会掉一下）。
 * 2. 摩擦轮就绪 Shoot_FrictionReady：friction_rpm > 0，两个摩擦轮 |friction_rpm − speed_rpm| < SHOOT_FRICTION_READY_ERR_RPM，
 *    并且已经连续 SHOOT_FRICTION_READY_HOLD_MS。speed_rpm 驱动里已经按方向修正过，直接和逻辑目标比。
 *    计时只记最后一次“不满足”的 tick，now − 它 ≥ HOLD_MS 就是就绪；RELAX 时当作不满足，下次起转重新计时。
 * 3. Shoot_SetTarget：按实际模式给目标（转子 rpm，方向由 MotorControl 乘 motor_direction）。
 *    SPINUP：两个摩擦轮 friction_rpm，拨弹 0；FEED：摩擦轮同上，拨弹轮交给 Shoot_FeederUpdate；RELAX：DISABLE 会清目标，不用给。
 * 4. Shoot_FeederUpdate（只在实际 FEED 时调）：拨弹轮的卡弹检测和反转退弹，状态见 Shoot_JAM_e。
 *    - 卡弹条件：feeder_rpm > 0，output_current ≥ SHOOT_JAM_CURRENT，speed_rpm < SHOOT_JAM_SPEED_RPM，连续 SHOOT_JAM_DETECT_MS。
 *      output_current、speed_rpm 都是逻辑方向（驱动里已按方向修正）。用 output_current（速度环算出来的指令）
 *      而不是 feedback_current：卡住时速度环顶到限幅，指令最稳。
 *    - 判到卡弹：次数 +1，拨弹轮切位置模式（电机层把当前位置记成零点、清 PID），目标 −SHOOT_JAM_REVERSE_DEG，
 *      位置环自己转。转到（离目标 < SHOOT_JAM_REVERSE_DONE_DEG）或超时（SHOOT_JAM_REVERSE_TIMEOUT_MS，反转也被挡住时）
 *      → 切回速度环正转。反转期间不判卡弹。
 *    - 正转顺利（没满足卡弹条件）连续 SHOOT_JAM_CLEAR_MS，次数清零。
 *    - 离开 FEED：反转停掉、计时重来（下次进 FEED 别拿旧 tick 立刻判卡）；退回 SPINUP 时拨弹轮切回速度环，
 *      进 RELAX 时 UpdateMode 已经切成 DISABLE。次数在指令 RELAX 时清，SPINUP ↔ FEED 来回时保留（摩擦轮掉速和卡弹常一起出现）。
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
    /*如果当前模式为无力模式则清除故障标志；卡弹次数也要一起清，不然下面马上又锁上*/
    if (mode == SHOOT_MODE_RELAX)
    {
        shoot_fault = 0;
        feeder_jam_count = 0;
    }
    /*如果电机掉线或者堵转次数过多就锁住，进入无力模式*/
    if (online == 0 || feeder_jam_count >= SHOOT_JAM_MAX_COUNT)
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

/* 实际 FEED 时给拨弹轮设目标，含卡弹检测和反转，见上面控制逻辑第 4 条。 */
static void Shoot_FeederUpdate(uint32_t now_tick)
{
    uint8_t jamming = (shoot_cmd.feeder_rpm > 0) &&
                      (shoot_feeder.output_current >= SHOOT_JAM_CURRENT) &&
                      (shoot_feeder.speed_rpm < SHOOT_JAM_SPEED_RPM);

    if (feeder_jam_state == SHOOT_JAM_NORMAL)
    {
        if (jamming)
        {
            feeder_jam_state = SHOOT_JAM_DETECT;
            feeder_jam_last_tick = now_tick;
        }
        else if ((feeder_jam_count != 0U) && (now_tick - feeder_jam_last_tick >= SHOOT_JAM_CLEAR_MS))
        {
            /*正转顺利跑满 CLEAR_MS，连续卡弹次数清零*/
            feeder_jam_count = 0;
        }
        MotorControl_SetTargetSpeed(&shoot_feeder, shoot_cmd.feeder_rpm);
        return;
    }

    if (feeder_jam_state == SHOOT_JAM_DETECT)
    {
        if (!jamming)
        {
            /*误判，退回NORMAL模式*/
            feeder_jam_state = SHOOT_JAM_NORMAL;
            feeder_jam_last_tick = now_tick;
        }
        else if (now_tick - feeder_jam_last_tick >= SHOOT_JAM_DETECT_MS)
        {
            /*判定为卡弹：切位置模式（当前位置为零点）往回转固定角度；次数到上限时下一周期 UpdateMode 锁住*/
            feeder_jam_state = SHOOT_JAM_REVERSE;
            feeder_jam_last_tick = now_tick;
            feeder_jam_count++;
            MotorControl_SetMode(&shoot_feeder, MOTOR_MODE_POSITION);
            MotorControl_SetTargetDegree(&shoot_feeder, -SHOOT_JAM_REVERSE_DEG);
            return;
        }
        MotorControl_SetTargetSpeed(&shoot_feeder, shoot_cmd.feeder_rpm);    /*计时期间还在正转*/
        return;
    }

    /* SHOOT_JAM_REVERSE：位置环自己转，这里只判转到没有、超没超时 */
    if ((fabsf(MotorControl_GetDegree(&shoot_feeder) + SHOOT_JAM_REVERSE_DEG) >= SHOOT_JAM_REVERSE_DONE_DEG) &&
        (now_tick - feeder_jam_last_tick < SHOOT_JAM_REVERSE_TIMEOUT_MS))
    {
        return;
    }
    /*转到了（或超时），切回速度环正转*/
    feeder_jam_state = SHOOT_JAM_NORMAL;
    feeder_jam_last_tick = now_tick;
    MotorControl_SetMode(&shoot_feeder, MOTOR_MODE_SPEED);
    MotorControl_SetTargetSpeed(&shoot_feeder, shoot_cmd.feeder_rpm);
}

/* 按实际模式 shoot_mode 给三台电机的速度目标，见上面控制逻辑第 3 条。 */
static void Shoot_SetTarget(uint32_t now_tick)
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
        Shoot_FeederUpdate(now_tick);
    }

    /* 不在 FEED：反转停掉、计时重来，下次进 FEED 从正转开始判（次数在 UpdateMode 里随 RELAX 清）。
       反转到一半退回 SPINUP 时拨弹轮还在位置模式，切回速度环（目标 0）；RELAX 时已经是 DISABLE，不动。 */
    if (shoot_mode != SHOOT_MODE_FEED)
    {
        if ((shoot_mode == SHOOT_MODE_SPINUP) && (shoot_feeder.mode == MOTOR_MODE_POSITION))
        {
            MotorControl_SetMode(&shoot_feeder, MOTOR_MODE_SPEED);
        }
        feeder_jam_state = SHOOT_JAM_NORMAL;
        feeder_jam_last_tick = now_tick;
    }
}

void Shoot_Update(uint32_t now_tick)
{
    Shoot_UpdateMode(now_tick);
    Shoot_SetTarget(now_tick);

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
