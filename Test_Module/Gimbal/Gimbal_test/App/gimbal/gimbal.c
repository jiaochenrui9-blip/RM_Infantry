#include "gimbal.h"

#include <math.h>
#include <string.h>

#include "gimbal_config.h"
#include "dji_gm6020.h"
#include "motor_control.h"
#include "user_math.h"

#define GIMBAL_DT_S                  0.001f    /* Gimbal_Update 的调用周期 */
#define GIMBAL_OFFLINE_TIMEOUT_MS    100U
#define GIMBAL_DEG_PER_ECD           (360.0f / 8192.0f)
#define GIMBAL_DPS_PER_RPM           6.0f      /* 1 rpm = 360° / 60 s */
#define GIMBAL_RAD_PER_DEG           0.0174532925f

/* 一个轴。yaw、pitch 数据形状相同，用同一套函数处理。 */
typedef struct
{
    DJI_Motor_t motor;          /* 方向存在 motor.motor_direction 里 */
    uint16_t ecd_offset;        /* 中值 */
    float min_deg;              /* 软限位（编码器角，两个模式都按它限） */
    float max_deg;
    PID_t angle_pid;            /* 角度环：deg → 角速度，输出 angle_pid.out，加上前馈才是目标角速度 */
    PID_t speed_pid;            /* 速度环：目标角速度看 speed_pid.target，输出电流 speed_pid.out */
    float angle_deg;            /* 编码器角，相对中值 */
    float rate_dps;             /* 电机转速换算的角速度（相对底盘） */
    float target_deg;           /* 经过限位和斜坡的目标，在本模式的坐标系里（IMU 模式下是对地的角） */
} GimbalAxis_t;

/* 调试器里 watch 这几个变量 */
static GimbalAxis_t gimbal_yaw;
static GimbalAxis_t gimbal_pitch;
static GimbalCmd_t gimbal_cmd;
static InsFeedback_t gimbal_imu;    /* Gimbal_SetImu 拷进来的 IMU 数据；初值全 0，即 valid = 0 */
static GimbalMode_e gimbal_mode;
static uint8_t gimbal_fault;        /* 闭环时电机掉线或 IMU 失效 → 锁在 RELAX，指令给一次 RELAX 才解除 */

static HAL_StatusTypeDef Gimbal_AxisInit(GimbalAxis_t *axis, DJI_MotorManager_t *bus,
                                         uint8_t esc_id, int8_t direction, uint16_t ecd_offset,
                                         float min_deg, float max_deg)
{
    HAL_StatusTypeDef status;

    memset(axis, 0, sizeof(*axis));
    axis->ecd_offset = ecd_offset;
    axis->min_deg = min_deg;
    axis->max_deg = max_deg;

    /* GM6020 电流控制帧：ID 1~4 用 0x1FE，ID 5~7 用 0x2FE，槽位从 0 数。ID 非法时注册函数会报错。 */
    if (esc_id <= 4U)
    {
        status = DJI_GM6020_Register(bus, &axis->motor, esc_id, 0x1FEU, (uint8_t)(esc_id - 1U));
    }
    else
    {
        status = DJI_GM6020_Register(bus, &axis->motor, esc_id, 0x2FEU, (uint8_t)(esc_id - 5U));
    }
    if (status != HAL_OK)
    {
        return status;
    }

    /* 电机层只用电流模式，它自带的 PID 不用（参数给 0），串级 PID 在本文件里算。 */
    status = MotorControl_Init(&axis->motor, GIMBAL_CURRENT_LIMIT, direction,
                               0.0f, 0.0f, 0.0f, 0.0f, 0.0f, 0.0f);
    if (status == HAL_OK)
    {
        MotorControl_SetMode(&axis->motor, MOTOR_MODE_CURRENT);
    }
    return status;
}

/* 读电机反馈。DJI_Motor_GetFeedback 返回 0 表示这 1 ms 没有新帧，沿用上一次的值。 */
static void Gimbal_AxisRead(GimbalAxis_t *axis)
{
    DJI_MotorFeedback_t feedback;
    int32_t delta;

    if (DJI_Motor_GetFeedback(&axis->motor, &feedback) == 0U)
    {
        return;
    }

    /* 编码器 → 相对中值的角度（-180~180）。驱动里的 encoder 是原始值、没乘方向，这里补上。 */
    delta = (int32_t)feedback.angle - (int32_t)axis->ecd_offset;
    if (delta >= 4096)
    {
        delta -= 8192;
    }
    else if (delta < -4096)
    {
        delta += 8192;
    }
    axis->angle_deg = (float)(delta * axis->motor.motor_direction) * GIMBAL_DEG_PER_ECD;

    /* speed_rpm 在驱动里已经乘过方向 */
    axis->rate_dps = (float)feedback.speed_rpm * GIMBAL_DPS_PER_RPM;
}

/* 一个轴的控制：目标限位 → 斜坡 → 串级 PID（带速度前馈），返回电流。
   fb_deg、fb_dps 是本模式的反馈：编码器模式传编码器角和电机转速，IMU 模式传 IMU 的对地角和角速度。
   机械限位是相对底盘的，所以目标先换到编码器系里限位、走斜坡，再换回本模式的坐标系。 */
static float Gimbal_AxisControl(GimbalAxis_t *axis, float cmd_deg, float fb_deg, float fb_dps)
{
    /* 本模式坐标系比编码器系多出来的角，编码器模式下为 0 */
    float offset = Math_WrapDeg180(fb_deg - axis->angle_deg);
    /* 现在的目标先拉回限位内：底盘转得比斜坡快时，目标也不会被带到限位外 */
    float cur = Math_Clamp(Math_WrapDeg180(axis->target_deg - offset), axis->min_deg, axis->max_deg);
    float goal = Math_Clamp(Math_WrapDeg180(cmd_deg - offset), axis->min_deg, axis->max_deg);
    float diff = goal - cur;
    float error;
    float ff_dps;
    float speed_target_dps;

    /* 不限位的轴按最短方向走；有限位的轴不能跨过 ±180° 绕到背面去 */
    if ((axis->max_deg - axis->min_deg) >= 360.0f)
    {
        diff = Math_WrapDeg180(diff);
    }
    diff = Math_Clamp(diff, -GIMBAL_RAMP_DEG_PER_MS, GIMBAL_RAMP_DEG_PER_MS);
    /* 底盘转、指令不变时，goal − cur 里的 offset 抵消，diff = 0，云台对地不动 */
    axis->target_deg = Math_WrapDeg180(cur + diff + offset);

    /* 速度前馈：目标这 1 ms 走了 diff 度，换算成角速度直接给速度环。
       没有它，角度环只能靠误差产生这个速度，斜坡时会一直落后“斜坡速度 / 角度环 Kp”度。 */
    ff_dps = GIMBAL_SPEED_FF_RATIO * diff / GIMBAL_DT_S;

    /* 误差先折到 ±180°，再把 PID 目标写成“反馈 + 误差”，跨 ±180° 时误差不会算成 350° */
    error = Math_WrapDeg180(axis->target_deg - fb_deg);
    PID_SetTarget(&axis->angle_pid, fb_deg + error);
    speed_target_dps = Math_Clamp(PID_Calculate(&axis->angle_pid, fb_deg, GIMBAL_DT_S) + ff_dps,
                                  -GIMBAL_SPEED_LIMIT_DPS, GIMBAL_SPEED_LIMIT_DPS);
    PID_SetTarget(&axis->speed_pid, speed_target_dps);
    return PID_Calculate(&axis->speed_pid, fb_dps, GIMBAL_DT_S);
}

static void Gimbal_AxisOutput(GimbalAxis_t *axis, float current, uint32_t now_tick)
{
    current = Math_Clamp(current, -GIMBAL_CURRENT_LIMIT, GIMBAL_CURRENT_LIMIT);
    (void)DJI_Motor_SetCurrent(&axis->motor, (int16_t)current);
    /* 电机层自带离线保护：超过 100 ms 没有反馈会把输出清零 */
    MotorControl_Update(&axis->motor, now_tick);
}

/* 根据指令、电机在线和 IMU 有效决定实际模式。模式变了，目标从新模式下的当前角开始，避免一切换就猛甩。 */
static void Gimbal_UpdateMode(uint8_t online)
{
    GimbalMode_e mode = gimbal_cmd.mode;

    if (mode == GIMBAL_MODE_RELAX)
    {
        gimbal_fault = 0U;
    }
    else if ((online == 0U) || ((mode == GIMBAL_MODE_IMU) && (gimbal_imu.valid == 0U)))
    {
        gimbal_fault = 1U;
    }
    if (gimbal_fault != 0U)
    {
        mode = GIMBAL_MODE_RELAX;
    }

    if (mode != gimbal_mode)
    {
        /* 角度环的反馈换了坐标系（编码器角和 IMU 角可能差上百度），要清，不然微分项会冲一下。
           速度环只在从 RELAX 进闭环时清：它的积分抵消的是重力、摩擦，和坐标系无关，
           ENCODER ↔ IMU 时清掉的话，pitch 靠积分扛重力时会往下掉。 */
        PID_Reset(&gimbal_yaw.angle_pid);
        PID_Reset(&gimbal_pitch.angle_pid);
        if (gimbal_mode == GIMBAL_MODE_RELAX)
        {
            PID_Reset(&gimbal_yaw.speed_pid);
            PID_Reset(&gimbal_pitch.speed_pid);
        }

        if (mode == GIMBAL_MODE_IMU)
        {
            gimbal_yaw.target_deg = gimbal_imu.yaw_deg;
            gimbal_pitch.target_deg = gimbal_imu.pitch_deg;
        }
        else
        {
            gimbal_yaw.target_deg = gimbal_yaw.angle_deg;
            gimbal_pitch.target_deg = gimbal_pitch.angle_deg;
        }
        gimbal_mode = mode;
    }
}

HAL_StatusTypeDef Gimbal_Init(DJI_MotorManager_t *yaw_bus, DJI_MotorManager_t *pitch_bus)
{
    HAL_StatusTypeDef status;

    if ((yaw_bus == NULL) || (pitch_bus == NULL))
    {
        return HAL_ERROR;
    }

    status = Gimbal_AxisInit(&gimbal_yaw, yaw_bus, GIMBAL_YAW_ESC_ID, GIMBAL_YAW_DIRECTION,
                             GIMBAL_YAW_ECD_OFFSET, GIMBAL_YAW_MIN_DEG, GIMBAL_YAW_MAX_DEG);
    if (status != HAL_OK)
    {
        return status;
    }
    status = Gimbal_AxisInit(&gimbal_pitch, pitch_bus, GIMBAL_PITCH_ESC_ID, GIMBAL_PITCH_DIRECTION,
                             GIMBAL_PITCH_ECD_OFFSET, GIMBAL_PITCH_MIN_DEG, GIMBAL_PITCH_MAX_DEG);
    if (status != HAL_OK)
    {
        return status;
    }

    /* 输出限幅：角度环限最大角速度，速度环限电流；积分项最多可以用满输出。 */
    PID_Init(&gimbal_yaw.angle_pid, GIMBAL_YAW_ANGLE_KP, GIMBAL_YAW_ANGLE_KI, GIMBAL_YAW_ANGLE_KD,
             -GIMBAL_SPEED_LIMIT_DPS, GIMBAL_SPEED_LIMIT_DPS, -GIMBAL_SPEED_LIMIT_DPS, GIMBAL_SPEED_LIMIT_DPS);
    PID_Init(&gimbal_yaw.speed_pid, GIMBAL_YAW_SPEED_KP, GIMBAL_YAW_SPEED_KI, GIMBAL_YAW_SPEED_KD,
             -GIMBAL_CURRENT_LIMIT, GIMBAL_CURRENT_LIMIT, -GIMBAL_CURRENT_LIMIT, GIMBAL_CURRENT_LIMIT);
    PID_Init(&gimbal_pitch.angle_pid, GIMBAL_PITCH_ANGLE_KP, GIMBAL_PITCH_ANGLE_KI, GIMBAL_PITCH_ANGLE_KD,
             -GIMBAL_SPEED_LIMIT_DPS, GIMBAL_SPEED_LIMIT_DPS, -GIMBAL_SPEED_LIMIT_DPS, GIMBAL_SPEED_LIMIT_DPS);
    PID_Init(&gimbal_pitch.speed_pid, GIMBAL_PITCH_SPEED_KP, GIMBAL_PITCH_SPEED_KI, GIMBAL_PITCH_SPEED_KD,
             -GIMBAL_CURRENT_LIMIT, GIMBAL_CURRENT_LIMIT, -GIMBAL_CURRENT_LIMIT, GIMBAL_CURRENT_LIMIT);

    gimbal_cmd.mode = GIMBAL_MODE_RELAX;
    gimbal_mode = GIMBAL_MODE_RELAX;
    gimbal_fault = 0U;
    return HAL_OK;
}

void Gimbal_SetImu(const InsFeedback_t *imu)
{
    if (imu == NULL)
    {
        return;
    }

    gimbal_imu = *imu;
}

void Gimbal_SetCmd(const GimbalCmd_t *cmd)
{
    if (cmd == NULL)
    {
        return;
    }

    gimbal_cmd = *cmd;
    if ((gimbal_cmd.mode != GIMBAL_MODE_ENCODER) && (gimbal_cmd.mode != GIMBAL_MODE_IMU))
    {
        gimbal_cmd.mode = GIMBAL_MODE_RELAX;    /* 非法值一律当无力 */
    }
}

void Gimbal_Update(uint32_t now_tick)
{
    float yaw_current = 0.0f;
    float pitch_current = 0.0f;

    Gimbal_AxisRead(&gimbal_yaw);
    Gimbal_AxisRead(&gimbal_pitch);
    Gimbal_UpdateMode((DJI_Motor_IsOnline(&gimbal_yaw.motor, now_tick, GIMBAL_OFFLINE_TIMEOUT_MS) != 0U) &&
                      (DJI_Motor_IsOnline(&gimbal_pitch.motor, now_tick, GIMBAL_OFFLINE_TIMEOUT_MS) != 0U));

    /* 两个闭环模式只是反馈不同：编码器模式用编码器角和电机转速，IMU 模式用 IMU 的对地角和角速度 */
    if (gimbal_mode == GIMBAL_MODE_ENCODER)
    {
        yaw_current = Gimbal_AxisControl(&gimbal_yaw, gimbal_cmd.yaw_deg,
                                         gimbal_yaw.angle_deg, gimbal_yaw.rate_dps);
        pitch_current = Gimbal_AxisControl(&gimbal_pitch, gimbal_cmd.pitch_deg,
                                           gimbal_pitch.angle_deg, gimbal_pitch.rate_dps);
    }
    else if (gimbal_mode == GIMBAL_MODE_IMU)
    {
        yaw_current = Gimbal_AxisControl(&gimbal_yaw, gimbal_cmd.yaw_deg,
                                         gimbal_imu.yaw_deg, gimbal_imu.yaw_rate_dps);
        pitch_current = Gimbal_AxisControl(&gimbal_pitch, gimbal_cmd.pitch_deg,
                                           gimbal_imu.pitch_deg, gimbal_imu.pitch_rate_dps);
    }

    /* pitch 重力前馈：重力矩随 cos(pitch) 变化，两个闭环模式都用编码器 pitch */
    if (gimbal_mode != GIMBAL_MODE_RELAX)
    {
        pitch_current += GIMBAL_PITCH_GRAVITY_FF * cosf(gimbal_pitch.angle_deg * GIMBAL_RAD_PER_DEG);
    }

    /* RELAX 时两个电流都是 0 */
    Gimbal_AxisOutput(&gimbal_yaw, yaw_current, now_tick);
    Gimbal_AxisOutput(&gimbal_pitch, pitch_current, now_tick);
}

void Gimbal_GetFeedback(GimbalFeedback_t *feedback)
{
    if (feedback == NULL)
    {
        return;
    }

    feedback->mode = gimbal_mode;
    feedback->yaw_rel_deg = gimbal_yaw.angle_deg;
    feedback->pitch_rel_deg = gimbal_pitch.angle_deg;
    feedback->yaw_imu_deg = gimbal_imu.yaw_deg;
    feedback->pitch_imu_deg = gimbal_imu.pitch_deg;
}
