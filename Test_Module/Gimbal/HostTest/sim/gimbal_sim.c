#include "gimbal_sim.h"

#include <math.h>
#include <string.h>

#include "main.h"
#include "chassis_link.h"
#include "dji_motor_manager.h"
#include "gimbal_config.h"
#include "gimbal_test_cmd.h"
#include "ins.h"
#include "user_math.h"

#define SIM_SUBSTEPS        10                          /* 每个 1 ms 周期里模型积分的步数 */
#define SIM_DT_S            (0.001f / SIM_SUBSTEPS)
#define SIM_KT_NM_PER_A     0.741f                      /* GM6020 转矩常数，手册约 741 mN·m/A */
#define SIM_A_PER_CMD       (3.0f / 16384.0f)           /* 电流控制：±16384 对应 ±3 A */
#define SIM_J_KGM2          0.005f                      /* 每个轴的转动惯量（假设值） */
#define SIM_FRICTION        0.005f                      /* 粘滞摩擦 N·m·s/rad，作用在相对底盘的转速上（轴承、滑环） */
#define SIM_RAD_PER_DEG     0.0174532925f
#define SIM_DEG_PER_RAD     57.2957795f

Sim_t sim;

static DJI_MotorManager_t sim_can1;     /* 和 robot.c 一样，两个云台电机都在 CAN1 */
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

/* 电调 ID → 控制帧 ID 和槽位（和 gimbal.c 的 Gimbal_AxisInit 一样的规则） */
static uint16_t Sim_TxId(uint8_t esc_id)
{
    return (esc_id <= 4U) ? 0x1FEU : 0x2FEU;
}

static uint8_t Sim_TxSlot(uint8_t esc_id)
{
    return (esc_id <= 4U) ? (uint8_t)(esc_id - 1U) : (uint8_t)(esc_id - 5U);
}

/* can_bus_stub.c 发帧时调用：从控制帧里取出两个电机的电流；板间帧 0x110 交给底盘板那一侧的接收代码 */
void Sim_OnCanTx(const CAN_HandleTypeDef *hcan, uint16_t std_id, const uint8_t *data, uint8_t length)
{
    uint8_t slot;

    if ((hcan != &hcan1) || (length != 8U))
    {
        return;
    }

    /* 两块板编译的是同一份 chassis_link.c：云台板这边发出去的帧，直接用底盘板的 ProcessRxFrame 收，
       Sim_ChassisCmd 再用 GetCmd 解出来，等于把打包、解包、超时都测了一遍 */
    if (std_id == CHASSIS_LINK_CMD_ID)
    {
        CAN_RxHeaderTypeDef header = {0};

        header.StdId = std_id;
        header.IDE = CAN_ID_STD;
        header.RTR = CAN_RTR_DATA;
        header.DLC = length;
        ChassisLink_ProcessRxFrame(&header, data);
        return;
    }

    if (std_id == Sim_TxId(GIMBAL_YAW_ESC_ID))
    {
        slot = Sim_TxSlot(GIMBAL_YAW_ESC_ID);
        sim.yaw.cmd_current = (int16_t)(((uint16_t)data[slot * 2U] << 8) | data[slot * 2U + 1U]);
    }
    if (std_id == Sim_TxId(GIMBAL_PITCH_ESC_ID))
    {
        slot = Sim_TxSlot(GIMBAL_PITCH_ESC_ID);
        sim.pitch.cmd_current = (int16_t)(((uint16_t)data[slot * 2U] << 8) | data[slot * 2U + 1U]);
    }
}

/* 按 GM6020 反馈帧格式把模型状态打包（大端：角度、转速 rpm、电流、温度），交给真驱动解析 */
static void Sim_SendFeedback(const SimAxis_t *axis, uint8_t esc_id, uint16_t ecd_offset)
{
    CAN_RxHeaderTypeDef header = {0};
    uint8_t data[8] = {0};
    int32_t ecd;
    int16_t rpm;

    if (axis->online == 0U)
    {
        return;
    }

    /* 编码器：中值 + 相对角换算的计数，只有 8192 档；方向不对时这里和约定相反 */
    ecd = (int32_t)lroundf((float)axis->phys_dir * axis->rel_deg * (8192.0f / 360.0f)) + (int32_t)ecd_offset;
    ecd %= 8192;
    if (ecd < 0)
    {
        ecd += 8192;
    }
    /* 转速：电机测的是相对底盘的转速，整数 rpm（1 rpm = 6°/s） */
    rpm = (int16_t)lroundf((float)axis->phys_dir * (axis->world_dps - axis->base_dps) / 6.0f);

    data[0] = (uint8_t)((uint32_t)ecd >> 8);
    data[1] = (uint8_t)ecd;
    data[2] = (uint8_t)((uint16_t)rpm >> 8);
    data[3] = (uint8_t)rpm;
    data[6] = 30U;

    header.StdId = 0x204U + esc_id;
    header.IDE = CAN_ID_STD;
    header.RTR = CAN_RTR_DATA;
    header.DLC = 8U;
    DJI_MotorManager_ProcessRxFrame(&sim_can1, &header, data);
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
static void Sim_AxisStep(SimAxis_t *axis, float gravity_nm, float dt)
{
    float world_rad = (axis->base_deg + axis->rel_deg) * SIM_RAD_PER_DEG;
    float torque = (float)axis->phys_dir * SIM_KT_NM_PER_A * (float)axis->cmd_current * SIM_A_PER_CMD
                 - SIM_FRICTION * (axis->world_dps - axis->base_dps) * SIM_RAD_PER_DEG
                 - gravity_nm * cosf(world_rad);

    axis->world_dps += torque / SIM_J_KGM2 * SIM_DEG_PER_RAD * dt;
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
    fprintf(file, "tick,mode,yaw_rel,yaw_world,yaw_current,pitch_rel,pitch_world,pitch_current,imu_valid\n");
}

static void Sim_Step(void)
{
    GimbalFeedback_t feedback;
    GimbalCmd_t cmd;
    ChassisCmd_t chassis_cmd;
    uint8_t i;

    sim_tick++;

    /* 1. 电机反馈帧：真车上是 CAN 中断里收到的，这里在控制之前送进驱动 */
    Sim_SendFeedback(&sim.yaw, GIMBAL_YAW_ESC_ID, GIMBAL_YAW_ECD_OFFSET);
    Sim_SendFeedback(&sim.pitch, GIMBAL_PITCH_ESC_ID, GIMBAL_PITCH_ECD_OFFSET);

    /* 2. 下面和 robot.c 的 Robot_Loop 顺序一样：IMU → 指令 → 云台 → 发帧 */
    Sim_UpdateImu();
    Gimbal_SetImu(&sim_imu);

    Gimbal_GetFeedback(&feedback);
    GimbalTestCmd_Update(sim_tick, &feedback, &cmd, &chassis_cmd);
    Gimbal_SetCmd(&cmd);

    Gimbal_Update(sim_tick);
    (void)DJI_MotorManager_SendAll(&sim_can1);
    (void)ChassisLink_SendCmd(&hcan1, &chassis_cmd);

    /* 3. 模型按收到的电流往前走 1 ms */
    for (i = 0U; i < SIM_SUBSTEPS; i++)
    {
        Sim_AxisStep(&sim.yaw, 0.0f, SIM_DT_S);
        Sim_AxisStep(&sim.pitch, sim.pitch_gravity_nm, SIM_DT_S);
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
    (void)DJI_MotorManager_Init(&sim_can1, &hcan1);
    (void)Gimbal_Init(&sim_can1, &sim_can1);
    (void)DJI_MotorManager_Start(&sim_can1);
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
