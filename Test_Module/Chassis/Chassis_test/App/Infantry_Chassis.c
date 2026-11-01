//
// Created by game on 2026/11/1.
//

#include "Infantry.h"

#include <math.h>

#include "chassis_config.h"
#include "dji_m3508.h"
#include "motor_control.h"
#include "user_math.h"

#define CHASSIS_DT_S                  0.001f    /* Chassis_Update 的调用周期 */
#define CHASSIS_OFFLINE_TIMEOUT_MS    100U
#define CHASSIS_RAD_PER_DEG           0.0174532925f
/* 轮子线速度 m/s → 转子转速 rpm：除以轮子周长得每秒圈数，× 60 是轮子 rpm，再 × 减速比是转子 rpm */
#define CHASSIS_MPS_TO_RPM            (60.0f / (2.0f * 3.14159265f * CHASSIS_WHEEL_RADIUS_M) * DJI_M3508_REDUCTION_RATIO)

/* 轮子下标：俯视顺时针，从前左开始 */
enum
{
    CHASSIS_FL = 0,
    CHASSIS_FR,
    CHASSIS_BR,
    CHASSIS_BL,
    CHASSIS_MOTOR_NUM
};

static ChassisCmd_t chassis_cmd;                        /* Chassis_SetCmd 拷进来的指令 */
static DJI_Motor_t chassis_motors[CHASSIS_MOTOR_NUM];   /* 4 个底盘电机，按顺时针从前左开始编号：0~3。 */

/* 调试器里 watch 这几个变量 */
static ChassisMode_e chassis_mode;      /* 实际模式，可能和指令不同 */
static uint8_t chassis_fault;           /* 运动模式下电机掉线 → 锁在 RELAX，指令给一次 RELAX 才解除 */
static float chassis_vx_mps;            /* 斜坡后的平移速度，在云台坐标系里 */
static float chassis_vy_mps;
static float chassis_wz_dps;            /* 斜坡后的底盘角速度，俯视逆时针为正 */

/* 注册一个 M3508，配好电机层：速度环用电机层自带的，位置环不用（参数给 0）。 */
static HAL_StatusTypeDef Chassis_MotorInit(DJI_Motor_t *motor, DJI_MotorManager_t *manager,
                                           uint8_t esc_id, int8_t direction)
{
    HAL_StatusTypeDef status;

    /* ID 1~4 用 0x200 控制帧，槽位 = ID - 1；ID 不在 1~4、manager 为 NULL 时注册函数会报错 */
    status = DJI_M3508_Register(manager, motor, esc_id, DJI_MOTOR_CONTROL_ID_1_4, (uint8_t)(esc_id - 1U));
    if (status != HAL_OK)
    {
        return status;
    }

    /* 注册完电机层是失能的（MOTOR_MODE_DISABLE），输出 0；Chassis_Update 按模式切到速度模式 */
    return MotorControl_Init(motor, CHASSIS_CURRENT_LIMIT, direction,
                             CHASSIS_SPEED_KP, CHASSIS_SPEED_KI, CHASSIS_SPEED_KD,
                             0.0f, 0.0f, 0.0f);
}

/*初始化各种电机*/
HAL_StatusTypeDef Infantry_Chassis_Init(DJI_MotorManager_t *manager)
{
    /* 任一个注册失败就停下返回，robot.c 会进 Error_Handler */
    if ((Chassis_MotorInit(&chassis_motors[CHASSIS_FL], manager, CHASSIS_FL_ESC_ID, CHASSIS_FL_DIRECTION) != HAL_OK) ||
        (Chassis_MotorInit(&chassis_motors[CHASSIS_FR], manager, CHASSIS_FR_ESC_ID, CHASSIS_FR_DIRECTION) != HAL_OK) ||
        (Chassis_MotorInit(&chassis_motors[CHASSIS_BR], manager, CHASSIS_BR_ESC_ID, CHASSIS_BR_DIRECTION) != HAL_OK) ||
        (Chassis_MotorInit(&chassis_motors[CHASSIS_BL], manager, CHASSIS_BL_ESC_ID, CHASSIS_BL_DIRECTION) != HAL_OK))
    {
        return HAL_ERROR;
    }

    return HAL_OK;
}

/*每1ms调用一次，给底盘设置速度命令*/
void Chassis_SetCmd(const ChassisCmd_t *cmd)
{
    if (cmd == NULL)
    {
        return;
    }

    chassis_cmd = *cmd;
    if ((chassis_cmd.mode != CHASSIS_MODE_NO_FOLLOW) &&
        (chassis_cmd.mode != CHASSIS_MODE_FOLLOW) &&
        (chassis_cmd.mode != CHASSIS_MODE_SPIN))
    {
        chassis_cmd.mode = CHASSIS_MODE_RELAX;    /* 非法值一律当无力 */
    }
}

/* 根据指令和电机在线定实际模式。电机层的模式只在真的变了时切：每周期都 SetMode 会把速度环积分清掉。 */
static void Chassis_ModeUpdate(uint32_t now_tick)
{
    ChassisMode_e mode = chassis_cmd.mode;
    uint8_t online = 1U;

    for (uint8_t i = 0U; i < CHASSIS_MOTOR_NUM; i++)
    {
        if (DJI_Motor_IsOnline(&chassis_motors[i], now_tick, CHASSIS_OFFLINE_TIMEOUT_MS) == 0U)
        {
            online = 0U;
        }
    }

    if (mode == CHASSIS_MODE_RELAX)
    {
        chassis_fault = 0U;
    }
    else if (online == 0U)
    {
        chassis_fault = 1U;     /* 少一个轮子出力，合力方向就不对，4 个都停 */
    }
    if (chassis_fault != 0U)
    {
        mode = CHASSIS_MODE_RELAX;
    }

    if (mode == chassis_mode)
    {
        return;
    }

    /* 进 RELAX：电机层失能（输出 0），斜坡清零，下次从 0 开始加速。
       从 RELAX 进运动模式：电机层切到速度模式（会清速度环）。运动模式之间切换电机层不动，斜坡接着走。 */
    if (mode == CHASSIS_MODE_RELAX)
    {
        for (uint8_t i = 0U; i < CHASSIS_MOTOR_NUM; i++)
        {
            MotorControl_SetMode(&chassis_motors[i], MOTOR_MODE_DISABLE);
        }
        chassis_vx_mps = 0.0f;
        chassis_vy_mps = 0.0f;
        chassis_wz_dps = 0.0f;
    }
    else if (chassis_mode == CHASSIS_MODE_RELAX)
    {
        for (uint8_t i = 0U; i < CHASSIS_MOTOR_NUM; i++)
        {
            MotorControl_SetMode(&chassis_motors[i], MOTOR_MODE_SPEED);
        }
    }
    chassis_mode = mode;
}

/* 斜坡：平移按向量长度限加速度（vx、vy 同比例走，方向不变），wz 单独限。
   必须在坐标变换之前、在云台坐标系里做：小陀螺时底盘坐标系里的速度方向一直在转，变换后再限会把平移拖慢、拖歪。 */
static void Chassis_Ramp(float wz_target_dps)
{
    const float step = CHASSIS_ACCEL_MPS2 * CHASSIS_DT_S;
    const float wz_step = CHASSIS_WZ_ACCEL_DPS2 * CHASSIS_DT_S;
    float dvx = chassis_cmd.vx_mps - chassis_vx_mps;
    float dvy = chassis_cmd.vy_mps - chassis_vy_mps;
    float dv = sqrtf(dvx * dvx + dvy * dvy);

    if (dv > step)
    {
        dvx *= step / dv;
        dvy *= step / dv;
    }
    chassis_vx_mps += dvx;
    chassis_vy_mps += dvy;
    chassis_wz_dps += Math_Clamp(wz_target_dps - chassis_wz_dps, -wz_step, wz_step);
}

/* 逆运动学：底盘坐标系的 vx、vy（m/s）和 wz（度/秒）→ 4 个目标转子转速。
   轮速正 = 这个轮子往让底盘前进的方向滚。麦轮和全向轮符号规律一样，只差 chassis_config.h 里的两个系数。 */
static void Chassis_WheelRpm(float vx, float vy, float wz_dps, float rpm[CHASSIS_MOTOR_NUM])
{
    float xy_fl_br = CHASSIS_XY_FACTOR * (vx - vy);     /* 前左、后右两个轮的平移分量 */
    float xy_fr_bl = CHASSIS_XY_FACTOR * (vx + vy);     /* 前右、后左两个轮的平移分量 */
    float w = CHASSIS_ROTATE_RADIUS_M * wz_dps * CHASSIS_RAD_PER_DEG;  /* 旋转分量：逆时针转时左侧往后、右侧往前 */
    float max_rpm = 0.0f;

    rpm[CHASSIS_FL] = (xy_fl_br - w) * CHASSIS_MPS_TO_RPM;
    rpm[CHASSIS_FR] = (xy_fr_bl + w) * CHASSIS_MPS_TO_RPM;
    rpm[CHASSIS_BR] = (xy_fl_br + w) * CHASSIS_MPS_TO_RPM;
    rpm[CHASSIS_BL] = (xy_fr_bl - w) * CHASSIS_MPS_TO_RPM;

    /* 任一个超过上限就 4 个一起等比例缩小，合成运动方向不变（小陀螺加平移时很容易超） */
    for (uint8_t i = 0U; i < CHASSIS_MOTOR_NUM; i++)
    {
        if (fabsf(rpm[i]) > max_rpm)
        {
            max_rpm = fabsf(rpm[i]);
        }
    }
    if (max_rpm > CHASSIS_WHEEL_MAX_RPM)
    {
        for (uint8_t i = 0U; i < CHASSIS_MOTOR_NUM; i++)
        {
            rpm[i] *= CHASSIS_WHEEL_MAX_RPM / max_rpm;
        }
    }
}

/*电机的控制逻辑，由主循环调用*/
void Chassis_Update(uint32_t now_tick)
{
    float wz_target_dps = 0.0f;
    float rpm[CHASSIS_MOTOR_NUM];

    /*模式切换*/
    Chassis_ModeUpdate(now_tick);

    if (chassis_mode != CHASSIS_MODE_RELAX)
    {
        /* 1. 底盘自己的目标角速度：不跟随为 0；跟随 = Kp × 云台角（云台在左 → 底盘往左转）；小陀螺定速。
              跟随只用 P：云台角过 ±180° 会跳变，PID.c 的微分对反馈直接求差，会冲一下 */
        if (chassis_mode == CHASSIS_MODE_FOLLOW)
        {
            wz_target_dps = Math_Clamp(CHASSIS_FOLLOW_KP * chassis_cmd.gimbal_yaw_rel_deg,
                                       -CHASSIS_FOLLOW_MAX_DPS, CHASSIS_FOLLOW_MAX_DPS);
        }
        else if (chassis_mode == CHASSIS_MODE_SPIN)
        {
            wz_target_dps = CHASSIS_SPIN_DPS;
        }

        /* 2. 斜坡（在云台坐标系里） */
        Chassis_Ramp(wz_target_dps);

        /* 3. 平移从云台坐标系转到底盘坐标系：云台相对底盘转了 theta（逆时针为正），把速度向量也转 theta */
        float theta = chassis_cmd.gimbal_yaw_rel_deg * CHASSIS_RAD_PER_DEG;
        float c = cosf(theta);
        float s = sinf(theta);

        /* 4. 逆运动学 → 目标转速，交给电机层的速度环 */
        Chassis_WheelRpm(chassis_vx_mps * c - chassis_vy_mps * s,
                         chassis_vx_mps * s + chassis_vy_mps * c,
                         chassis_wz_dps, rpm);
        for (uint8_t i = 0U; i < CHASSIS_MOTOR_NUM; i++)
        {
            MotorControl_SetTargetSpeed(&chassis_motors[i], (int16_t)rpm[i]);
        }
    }

    /* 5. 电机层：速度环、电流限幅、掉线保护（失能或掉线时输出 0） */
    for (uint8_t i = 0U; i < CHASSIS_MOTOR_NUM; i++)
    {
        MotorControl_Update(&chassis_motors[i], now_tick);
    }
}
