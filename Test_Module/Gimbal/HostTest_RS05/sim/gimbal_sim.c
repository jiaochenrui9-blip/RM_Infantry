#include "gimbal_sim.h"

#include <math.h>
#include <string.h>

#include "main.h"
#include "chassis_link.h"
#include "rs05.h"
#include "gimbal_config.h"
#include "gimbal_test_cmd.h"
#include "ins.h"
#include "user_math.h"

/*
 * RS05 电机模型，按手册第 4 章写（和 C:\Clion\RS05\notes.md 里驱动的主机测试是同一套假设，以上板为准）：
 * - 只在收到指令时回一帧类型 2：ID = 类型 2 | 模式状态 bit23~22 | 故障 bit21~16 | 电机 ID bit15~8 | 主机 ID；
 *   数据是角度（±4π 循环）、角速度（±50 rad/s）、力矩（±5.5 N·m）、温度 ×10，都是 16 位、高字节在前。
 *   这些都是减速后输出轴的量。CAN 断了（online = 0）时不收也不回。
 * - 类型 3 使能 → 运行；类型 4 停止 → Reset；类型 18 写参数：run_mode（运行中写不进）、iq_ref、canTimeout。
 * - 运行并且 run_mode = 3（电流模式）时按 iq_ref 出力。上电默认 run_mode = 0（运控），这时 iq_ref 不起作用。
 * - canTimeout（20000 = 1 s）不为 0 时，超过这么久没收到指令就自己进 Reset。上电默认 0。
 * - 上电（Sim_Reset 后第一个周期）时角度落在 0~2π（zero_sta = 0），之后通电期间连续数圈：
 *   测云台代码对"上电读数差整圈""±4π 循环"的处理。
 * - 不模拟：齿隙、上电约 1.4 s 不回话、故障、发送邮箱满。
 */

#define SIM_SUBSTEPS        10                          /* 每个 1 ms 周期里模型积分的步数 */
#define SIM_DT_S            (0.001f / SIM_SUBSTEPS)
#define SIM_KT_NM_PER_A     0.67f                       /* RS05：额定 1.6 N·m 对应相电流峰值 2.4 A */
#define SIM_IQ_MAX_A        11.0f                       /* 最大相电流（峰值） */
#define SIM_J_KGM2          0.005f                      /* 每个轴的转动惯量（假设值，和 GM6020 版一样） */
#define SIM_FRICTION        0.005f                      /* 粘滞摩擦 N·m·s/rad，作用在相对底盘的转速上（轴承、滑环） */
#define SIM_RAD_PER_DEG     0.0174532925f
#define SIM_TWO_PI          6.28318531f
#define SIM_POSITION_MAX    12.57f                      /* 手册类型 2 的角度范围，和驱动里的一样 */
#define SIM_VELOCITY_MAX    50.0f
#define SIM_TORQUE_MAX      5.5f
#define SIM_HOST_ID         0xFDU                       /* 驱动用的主机 ID，模型回帧时带回去 */
#define SIM_RUN_MODE_CURRENT 3U

/* 一个 RS05 电机（电机这一侧的状态，云台代码看不到） */
typedef struct
{
    uint8_t id;
    uint8_t powered;        /* 0：还没上电，Sim_Reset 后第一个周期上电 */
    uint8_t running;        /* 1：运行（Motor 模式），0：Reset */
    uint8_t run_mode;
    uint32_t can_timeout;   /* 0x7028 的原始值，20000 = 1 s */
    float iq_ref;           /* 0x7006，A */
    uint32_t last_rx_tick;  /* 最近一次收到指令的时间，canTimeout 用 */
    float zero_rad;         /* 云台在中值时电机的读数：取 gimbal_config.h 里的值，相当于已经标定好了 */
    float turn_offset_rad;  /* 上电时定的整圈偏移，让上电读数落在 0~2π */
} SimMotor_t;

Sim_t sim;

static RS05_Manager_t sim_can1;     /* 和 robot.c 一样，两个云台电机都在 CAN1 */
static SimMotor_t sim_yaw_motor;
static SimMotor_t sim_pitch_motor;
static uint32_t sim_tick;
static InsFeedback_t sim_imu;

uint32_t HAL_GetTick(void)
{
    return sim_tick;
}

uint32_t Sim_Tick(void)
{
    return sim_tick;
}

static SimAxis_t *Sim_AxisOf(const SimMotor_t *motor)
{
    return (motor == &sim_yaw_motor) ? &sim.yaw : &sim.pitch;
}

/* 电机自己的角度（rad，还没按 ±4π 折）：按电机的方向数，通电期间连续 */
static float Sim_MotorAngle(const SimMotor_t *motor)
{
    const SimAxis_t *axis = Sim_AxisOf(motor);

    return motor->zero_rad + (float)axis->phys_dir * axis->rel_deg * SIM_RAD_PER_DEG + motor->turn_offset_rad;
}

/* 电机这 1 ms 实际输出的 iq：只有运行在电流模式时才按 iq_ref 出力 */
static float Sim_MotorIq(const SimMotor_t *motor)
{
    if ((motor->running == 0U) || (motor->run_mode != SIM_RUN_MODE_CURRENT))
    {
        return 0.0f;
    }
    return Math_Clamp(motor->iq_ref, -SIM_IQ_MAX_A, SIM_IQ_MAX_A);
}

/* 上电：参数都回到默认（类型 18 写的掉电丢失），角度落在 0~2π */
static void Sim_MotorPowerUp(SimMotor_t *motor)
{
    motor->powered = 1U;
    motor->running = 0U;
    motor->run_mode = 0U;
    motor->can_timeout = 0U;
    motor->iq_ref = 0.0f;
    motor->turn_offset_rad = 0.0f;
    motor->turn_offset_rad = -SIM_TWO_PI * floorf(Sim_MotorAngle(motor) / SIM_TWO_PI);
}

/* 每个周期开头：Sim_Reset 后第一次就上电；运行中超过 canTimeout 没收到指令就进 Reset */
static void Sim_MotorTick(SimMotor_t *motor)
{
    if (motor->powered == 0U)
    {
        Sim_MotorPowerUp(motor);
    }
    if ((motor->running != 0U) && (motor->can_timeout != 0U) &&
        ((sim_tick - motor->last_rx_tick) * 20U > motor->can_timeout))
    {
        motor->running = 0U;
    }
}

/* 手册的 float_to_uint：(x − min) × 65535 / (max − min)，截断 */
static uint16_t Sim_ToU16(float x, float max)
{
    x = Math_Clamp(x, -max, max);
    return (uint16_t)((x + max) * 65535.0f / (2.0f * max));
}

/* 角度按 ±4π 循环（周期 8π） */
static float Sim_Wrap4Pi(float rad)
{
    const float half = 2.0f * SIM_TWO_PI;

    rad = fmodf(rad + half, 2.0f * half);
    if (rad < 0.0f)
    {
        rad += 2.0f * half;
    }
    return rad - half;
}

/* 回一帧类型 2，交给真驱动解析（真车上是随后的 CAN 接收中断） */
static void Sim_MotorReply(const SimMotor_t *motor)
{
    const SimAxis_t *axis = Sim_AxisOf(motor);
    CAN_RxHeaderTypeDef header = {0};
    uint8_t data[8];
    uint16_t position = Sim_ToU16(Sim_Wrap4Pi(Sim_MotorAngle(motor)), SIM_POSITION_MAX);
    uint16_t velocity = Sim_ToU16((float)axis->phys_dir * (axis->world_dps - axis->base_dps) * SIM_RAD_PER_DEG,
                                  SIM_VELOCITY_MAX);
    uint16_t torque = Sim_ToU16(SIM_KT_NM_PER_A * Sim_MotorIq(motor), SIM_TORQUE_MAX);
    uint16_t temperature = 300U;    /* 30.0 ℃ × 10 */

    data[0] = (uint8_t)(position >> 8);
    data[1] = (uint8_t)position;
    data[2] = (uint8_t)(velocity >> 8);
    data[3] = (uint8_t)velocity;
    data[4] = (uint8_t)(torque >> 8);
    data[5] = (uint8_t)torque;
    data[6] = (uint8_t)(temperature >> 8);
    data[7] = (uint8_t)temperature;

    header.ExtId = (2UL << 24) | ((uint32_t)((motor->running != 0U) ? 2U : 0U) << 22) |
                   ((uint32_t)motor->id << 8) | SIM_HOST_ID;
    header.IDE = CAN_ID_EXT;
    header.RTR = CAN_RTR_DATA;
    header.DLC = 8U;
    RS05_Manager_ProcessRxFrame(&sim_can1, &header, data);
}

/* can_bus_stub.c 发扩展帧时调用：交给目标电机处理，电机马上回一帧类型 2 */
void Sim_OnCanTxExt(const CAN_HandleTypeDef *hcan, uint32_t ext_id, const uint8_t *data, uint8_t length)
{
    uint8_t target = (uint8_t)(ext_id & 0xFFU);
    uint8_t type = (uint8_t)((ext_id >> 24) & 0x1FU);
    SimMotor_t *motor;
    uint16_t index;
    uint32_t value;

    if ((hcan != &hcan1) || (length != 8U))
    {
        return;
    }
    if (target == sim_yaw_motor.id)
    {
        motor = &sim_yaw_motor;
    }
    else if (target == sim_pitch_motor.id)
    {
        motor = &sim_pitch_motor;
    }
    else
    {
        return;
    }
    /* CAN 断了、还没上电：收不到 */
    if ((Sim_AxisOf(motor)->online == 0U) || (motor->powered == 0U))
    {
        return;
    }

    motor->last_rx_tick = sim_tick;
    switch (type)
    {
        case 3U:    /* 使能 */
            motor->running = 1U;
            break;
        case 4U:    /* 停止 */
            motor->running = 0U;
            break;
        case 18U:   /* 写参数：index 在 Byte0~1，值在 Byte4~7，都是低字节在前 */
            index = (uint16_t)(data[0] | ((uint16_t)data[1] << 8));
            value = (uint32_t)data[4] | ((uint32_t)data[5] << 8) | ((uint32_t)data[6] << 16) | ((uint32_t)data[7] << 24);
            if ((index == 0x7005U) && (motor->running == 0U))   /* 运行中不能切模式 */
            {
                motor->run_mode = (uint8_t)value;
            }
            else if (index == 0x7006U)
            {
                memcpy(&motor->iq_ref, &value, sizeof(motor->iq_ref));
            }
            else if (index == 0x7028U)
            {
                motor->can_timeout = value;
            }
            break;
        default:    /* 这一版用不到的类型，不回 */
            return;
    }
    Sim_MotorReply(motor);
}

/* can_bus_stub.c 发标准帧时调用：板间帧 0x110 交给底盘板那一侧的接收代码 */
void Sim_OnCanTxStd(const CAN_HandleTypeDef *hcan, uint16_t std_id, const uint8_t *data, uint8_t length)
{
    CAN_RxHeaderTypeDef header = {0};

    /* 两块板编译的是同一份 chassis_link.c：云台板这边发出去的帧，直接用底盘板的 ProcessRxFrame 收，
       Sim_ChassisCmd 再用 GetCmd 解出来，等于把打包、解包、超时都测了一遍 */
    if ((hcan != &hcan1) || (std_id != CHASSIS_LINK_CMD_ID))
    {
        return;
    }
    header.StdId = std_id;
    header.IDE = CAN_ID_STD;
    header.RTR = CAN_RTR_DATA;
    header.DLC = length;
    ChassisLink_ProcessRxFrame(&header, data);
}

/* 理想 IMU：直接给出云台对地的角度和角速度（相当于安装矩阵、零偏都标好了，也不漂） */
static void Sim_UpdateImu(void)
{
    if (sim.imu_frozen == 0U)
    {
        sim_imu.yaw_deg = Sim_YawWorld();
        sim_imu.pitch_deg = Sim_PitchWorld();
        sim_imu.yaw_rate_dps = sim.yaw.world_dps;
        sim_imu.pitch_rate_dps = sim.pitch.world_dps;
    }
    sim_imu.valid = sim.imu_valid;
}

/* 一个轴往前走 dt：J·dω/dt = 电机力矩 − 摩擦 + 重力；撞到机械极限就停 */
static void Sim_AxisStep(SimAxis_t *axis, const SimMotor_t *motor, float gravity_nm, float dt)
{
    float world_rad = (axis->base_deg + axis->rel_deg) * SIM_RAD_PER_DEG;
    float torque = (float)axis->phys_dir * SIM_KT_NM_PER_A * Sim_MotorIq(motor)
                 - SIM_FRICTION * (axis->world_dps - axis->base_dps) * SIM_RAD_PER_DEG
                 - gravity_nm * cosf(world_rad);

    axis->world_dps += torque / SIM_J_KGM2 / SIM_RAD_PER_DEG * dt;
    if (axis->held != 0U)
    {
        axis->world_dps = axis->base_dps;
    }
    axis->base_deg += axis->base_dps * dt;
    axis->rel_deg += (axis->world_dps - axis->base_dps) * dt;

    if (axis->rel_deg > axis->mech_max_deg)
    {
        axis->rel_deg = axis->mech_max_deg;
        axis->world_dps = axis->base_dps;
    }
    else if (axis->rel_deg < axis->mech_min_deg)
    {
        axis->rel_deg = axis->mech_min_deg;
        axis->world_dps = axis->base_dps;
    }
}

static void Sim_Trace(void)
{
    if (sim.trace == NULL)
    {
        return;
    }
    fprintf(sim.trace, "%lu,%d,%.3f,%.3f,%d,%.3f,%.3f,%d,%u\n",
            (unsigned long)sim_tick, (int)Sim_Mode(),
            sim.yaw.rel_deg, Sim_YawWorld(), sim.yaw.cmd_current,
            sim.pitch.rel_deg, Sim_PitchWorld(), sim.pitch.cmd_current,
            (unsigned)sim.imu_valid);
}

void Sim_TraceHeader(FILE *file)
{
    fprintf(file, "tick,mode,yaw_rel,yaw_world,yaw_iq_ma,pitch_rel,pitch_world,pitch_iq_ma,imu_valid\n");
}

static void Sim_Step(void)
{
    GimbalFeedback_t feedback;
    GimbalCmd_t cmd;
    ChassisCmd_t chassis_cmd;
    uint8_t i;

    sim_tick++;

    /* 1. 电机这一侧：Sim_Reset 后第一个周期上电；canTimeout 到了自己进 Reset */
    Sim_MotorTick(&sim_yaw_motor);
    Sim_MotorTick(&sim_pitch_motor);

    /* 2. 下面和 robot.c 的 Robot_Loop 顺序一样：IMU → 指令 → 云台 → 发帧。
          RS05 的反馈是应答：SendAll 发出去的每一帧，模型马上回一帧类型 2 交给驱动，云台下个周期读到。 */
    Sim_UpdateImu();
    Gimbal_SetImu(&sim_imu);

    Gimbal_GetFeedback(&feedback);
    GimbalTestCmd_Update(sim_tick, &feedback, &cmd, &chassis_cmd);
    Gimbal_SetCmd(&cmd);

    Gimbal_Update(sim_tick);
    (void)RS05_Manager_SendAll(&sim_can1);
    (void)ChassisLink_SendCmd(&hcan1, &chassis_cmd);

    /* 3. 模型按电机实际输出的 iq 往前走 1 ms */
    sim.yaw.cmd_current = (int16_t)lroundf(Sim_MotorIq(&sim_yaw_motor) * 1000.0f);
    sim.pitch.cmd_current = (int16_t)lroundf(Sim_MotorIq(&sim_pitch_motor) * 1000.0f);
    for (i = 0U; i < SIM_SUBSTEPS; i++)
    {
        Sim_AxisStep(&sim.yaw, &sim_yaw_motor, 0.0f, SIM_DT_S);
        Sim_AxisStep(&sim.pitch, &sim_pitch_motor, sim.pitch_gravity_nm, SIM_DT_S);
    }

    Sim_Trace();
}

void Sim_Run(uint32_t ms)
{
    uint32_t i;

    for (i = 0U; i < ms; i++)
    {
        Sim_Step();
    }
}

static void Sim_AxisDefault(SimAxis_t *axis, int8_t direction, float mech_min_deg, float mech_max_deg)
{
    memset(axis, 0, sizeof(*axis));
    axis->phys_dir = direction;
    axis->mech_min_deg = mech_min_deg;
    axis->mech_max_deg = mech_max_deg;
    axis->online = 1U;
}

static void Sim_MotorDefault(SimMotor_t *motor, uint8_t id, float zero_rad)
{
    memset(motor, 0, sizeof(*motor));
    motor->id = id;
    motor->zero_rad = zero_rad;
}

void Sim_Reset(void)
{
    FILE *trace = sim.trace;

    memset(&sim, 0, sizeof(sim));
    sim.trace = trace;
    /* 机械极限比 gimbal_config.h 的软限位各往外 5°。yaw 限位写满 360°（滑环）时没有机械极限 */
    if (SIM_YAW_UNLIMITED)
    {
        Sim_AxisDefault(&sim.yaw, GIMBAL_YAW_DIRECTION, -1.0e9f, 1.0e9f);
    }
    else
    {
        Sim_AxisDefault(&sim.yaw, GIMBAL_YAW_DIRECTION, GIMBAL_YAW_MIN_DEG - 5.0f, GIMBAL_YAW_MAX_DEG + 5.0f);
    }
    Sim_AxisDefault(&sim.pitch, GIMBAL_PITCH_DIRECTION, GIMBAL_PITCH_MIN_DEG - 5.0f, GIMBAL_PITCH_MAX_DEG + 5.0f);
    sim.imu_valid = 1U;

    /* 电机下一个周期才上电：场景在这之后改的初始位置，就是电机上电时的位置 */
    Sim_MotorDefault(&sim_yaw_motor, GIMBAL_YAW_CAN_ID, GIMBAL_YAW_ZERO_RAD);
    Sim_MotorDefault(&sim_pitch_motor, GIMBAL_PITCH_CAN_ID, GIMBAL_PITCH_ZERO_RAD);

    /* 时间只往前走、不回到 0：chassis_link 里存着上一个场景收到的帧和时间，
       时间倒回去的话，旧帧会被当成“比现在还新”，不算超时 */
    sim_tick += 1000U;
    memset(&sim_imu, 0, sizeof(sim_imu));

    gimbal_test.gimbal_mode = GIMBAL_MODE_RELAX;
    gimbal_test.yaw_deg = 0.0f;
    gimbal_test.pitch_deg = 0.0f;
    gimbal_test.chassis_mode = CHASSIS_MODE_RELAX;
    gimbal_test.vx_mps = 0.0f;
    gimbal_test.vy_mps = 0.0f;
    gimbal_test.yaw_square_deg = 0.0f;
    gimbal_test.pitch_square_deg = 0.0f;
    gimbal_test.square_period_ms = 2000U;

    /* 和 robot.c 的 Robot_Init 同样的顺序：manager 初始化 → 模块注册电机 → 启动 */
    (void)RS05_Manager_Init(&sim_can1, &hcan1);
    (void)Gimbal_Init(&sim_can1, &sim_can1);
    (void)RS05_Manager_Start(&sim_can1);
}

float Sim_YawWorld(void)
{
    return Math_WrapDeg180(sim.yaw.base_deg + sim.yaw.rel_deg);
}

float Sim_PitchWorld(void)
{
    return sim.pitch.base_deg + sim.pitch.rel_deg;
}

void Sim_ChassisCmd(ChassisCmd_t *cmd)
{
    ChassisLink_GetCmd(sim_tick, cmd);
}

GimbalMode_e Sim_Mode(void)
{
    GimbalFeedback_t feedback;

    Gimbal_GetFeedback(&feedback);
    return feedback.mode;
}
