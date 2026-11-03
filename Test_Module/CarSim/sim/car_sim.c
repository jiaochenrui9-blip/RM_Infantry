#include "car_sim.h"

#include <math.h>
#include <stdlib.h>
#include <string.h>

#include "main.h"
#include "chassis_config.h"
#include "dji_m3508.h"
#include "dji_motor_manager.h"
#include "gimbal_test_cmd.h"
#include "Infantry_Chassis.h"
#include "ins.h"
#include "user_math.h"

#define CAR_SUBSTEPS        10                          /* 每个 1 ms 周期里模型积分的步数 */
#define CAR_DT_S            (0.001f / CAR_SUBSTEPS)
#define CAR_RAD_PER_DEG     0.0174532925f
#define CAR_DEG_PER_RAD     57.2957795f
#define CAR_TWO_PI          6.28318531f

/* 云台轴（和 Gimbal/HostTest 的模型一样） */
#define CAR_GM_KT           0.741f                      /* GM6020 转矩常数 N·m/A */
#define CAR_GM_A_PER_CMD    (3.0f / 16384.0f)           /* 电流控制：±16384 对应 ±3 A */
#define CAR_GM_J            0.005f                      /* 转动惯量（假设值） */
#define CAR_GM_FRICTION     0.005f                      /* 粘滞摩擦 N·m·s/rad，作用在相对底盘的转速上（轴承、滑环） */

/* 底盘（假设值，见 README） */
#define CAR_MASS_KG         20.0f
#define CAR_IZ_KGM2         0.6f
#define CAR_M3508_KT_OUT    0.3f                        /* M3508 减速后输出轴 N·m/A（手册约 0.3） */
#define CAR_C620_A_PER_CMD  (20.0f / 16384.0f)          /* C620：±16384 对应 ±20 A */
#define CAR_WHEEL_DAMPING   1.0f                        /* 每个轮子的滚动阻力 N·s/m */

/* 内耗判定：轮子转速超过这个才算“在转”，电流超过这个才算“在出力” */
#define CAR_FIGHT_MIN_SPEED_MPS     0.2f
#define CAR_FIGHT_MIN_CURRENT       200

extern CAN_HandleTypeDef car_gimbal_hcan1;
extern CAN_HandleTypeDef car_gimbal_hcan2;
extern CAN_HandleTypeDef car_chassis_hcan1;
extern CAN_HandleTypeDef car_chassis_hcan2;

Car_t car;

static uint32_t car_tick;
static DJI_MotorManager_t car_gimbal_can1;     /* 和两块板的 robot.c 一样：每路 CAN 一个 manager */
static DJI_MotorManager_t car_gimbal_can2;
static DJI_MotorManager_t car_chassis_can1;
static DJI_MotorManager_t car_chassis_can2;
static ChassisCmd_t car_chassis_rx;            /* 底盘板最近一周期拿到的指令 */
static float car_rotor_rev[4];                 /* 4 个 M3508 转子累计圈数，算编码器用 */

static const uint8_t car_wheel_esc_id[4] = {CHASSIS_FL_ESC_ID, CHASSIS_FR_ESC_ID, CHASSIS_BR_ESC_ID, CHASSIS_BL_ESC_ID};
static const int8_t car_wheel_dir[4] = {CHASSIS_FL_DIRECTION, CHASSIS_FR_DIRECTION, CHASSIS_BR_DIRECTION, CHASSIS_BL_DIRECTION};
/* 轮速 = vx + J_VY·vy + J_W·R·wz（底盘系；麦轮再乘 XY_FACTOR）：和 Infantry_Chassis.c 的逆运动学同一组符号 */
static const float car_j_vy[4] = {-1.0f, 1.0f, -1.0f, 1.0f};
static const float car_j_w[4] = {-1.0f, 1.0f, 1.0f, -1.0f};

uint32_t HAL_GetTick(void)
{
    return car_tick;
}

uint32_t Car_Tick(void)
{
    return car_tick;
}

/* 4 个轮子的线速度（往让车前进的方向滚为正） */
static void Car_WheelSpeeds(float v[4])
{
    const CarChassis_t *c = &car.chassis;
    float wz_rad = c->wz_dps * CAR_RAD_PER_DEG;
    uint8_t i;

    for (i = 0U; i < 4U; i++)
    {
        v[i] = CHASSIS_XY_FACTOR * (c->vx_mps + car_j_vy[i] * c->vy_mps) + car_j_w[i] * CHASSIS_ROTATE_RADIUS_M * wz_rad;
    }
}

static uint8_t Car_TxSlot(uint8_t esc_id)
{
    return (esc_id <= 4U) ? (uint8_t)(esc_id - 1U) : (uint8_t)(esc_id - 5U);
}

static int16_t Car_ReadS16(const uint8_t *data)
{
    return (int16_t)(((uint16_t)data[0] << 8) | data[1]);
}

/* can_bus_stub.c 发帧时调用：按“哪块板、哪一路、哪个 ID”交给模型或另一块板 */
void Car_OnCanTx(const CAN_HandleTypeDef *hcan, uint16_t std_id, const uint8_t *data, uint8_t length)
{
    uint8_t i;

    if (length != 8U)
    {
        return;
    }

    if (hcan == &car_gimbal_hcan1)
    {
        /* 板间帧：底盘板 CAN1 收到，进中断里调的 ProcessRxFrame */
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
        if (std_id == ((GIMBAL_YAW_ESC_ID <= 4U) ? 0x1FEU : 0x2FEU))
        {
            car.yaw.cmd_current = Car_ReadS16(&data[Car_TxSlot(GIMBAL_YAW_ESC_ID) * 2U]);
        }
        if (std_id == ((GIMBAL_PITCH_ESC_ID <= 4U) ? 0x1FEU : 0x2FEU))
        {
            car.pitch.cmd_current = Car_ReadS16(&data[Car_TxSlot(GIMBAL_PITCH_ESC_ID) * 2U]);
        }
    }
    else if ((hcan == &car_chassis_hcan2) && (std_id == 0x200U))
    {
        for (i = 0U; i < 4U; i++)
        {
            car.chassis.wheel_current[i] = Car_ReadS16(&data[Car_TxSlot(car_wheel_esc_id[i]) * 2U]);
        }
    }
}

/* 按大疆电机反馈帧格式打包（大端：角度、转速 rpm、电流、温度），交给真驱动解析 */
static void Car_SendMotorFrame(DJI_MotorManager_t *manager, uint16_t feedback_id, int32_t ecd, float rpm, int16_t current)
{
    CAN_RxHeaderTypeDef header = {0};
    uint8_t data[8] = {0};
    int16_t rpm_int = (int16_t)lroundf(rpm);

    ecd %= 8192;
    if (ecd < 0)
    {
        ecd += 8192;
    }
    data[0] = (uint8_t)((uint32_t)ecd >> 8);
    data[1] = (uint8_t)ecd;
    data[2] = (uint8_t)((uint16_t)rpm_int >> 8);
    data[3] = (uint8_t)rpm_int;
    data[4] = (uint8_t)((uint16_t)current >> 8);
    data[5] = (uint8_t)current;
    data[6] = 30U;

    header.StdId = feedback_id;
    header.IDE = CAN_ID_STD;
    header.RTR = CAN_RTR_DATA;
    header.DLC = 8U;
    DJI_MotorManager_ProcessRxFrame(manager, &header, data);
}

/* 电机反馈：真车上是 CAN 中断里收到的，这里在两块板的控制之前送进驱动。方向都当作已经标对了 */
static void Car_SendFeedback(void)
{
    float v[4];
    float rotor_rpm;
    uint8_t i;

    if (car.yaw.online != 0U)
    {
        Car_SendMotorFrame(&car_gimbal_can1, (uint16_t)(0x204U + GIMBAL_YAW_ESC_ID),
                           (int32_t)lroundf((float)GIMBAL_YAW_DIRECTION * car.yaw.rel_deg * (8192.0f / 360.0f)) + (int32_t)GIMBAL_YAW_ECD_OFFSET,
                           (float)GIMBAL_YAW_DIRECTION * (car.yaw.world_dps - car.chassis.wz_dps) / 6.0f, 0);
    }
    if (car.pitch.online != 0U)
    {
        Car_SendMotorFrame(&car_gimbal_can1, (uint16_t)(0x204U + GIMBAL_PITCH_ESC_ID),
                           (int32_t)lroundf((float)GIMBAL_PITCH_DIRECTION * car.pitch.rel_deg * (8192.0f / 360.0f)) + (int32_t)GIMBAL_PITCH_ECD_OFFSET,
                           (float)GIMBAL_PITCH_DIRECTION * car.pitch.world_dps / 6.0f, 0);
    }

    Car_WheelSpeeds(v);
    for (i = 0U; i < 4U; i++)
    {
        if (car.chassis.wheel_online[i] == 0U)
        {
            continue;
        }
        /* 转子 rpm = 方向 × 轮子 rpm × 减速比 */
        rotor_rpm = (float)car_wheel_dir[i] * v[i] / (CAR_TWO_PI * CHASSIS_WHEEL_RADIUS_M) * 60.0f * DJI_M3508_REDUCTION_RATIO;
        Car_SendMotorFrame(&car_chassis_can2, (uint16_t)(0x200U + car_wheel_esc_id[i]),
                           (int32_t)lroundf((car_rotor_rev[i] - floorf(car_rotor_rev[i])) * 8192.0f),
                           rotor_rpm, car.chassis.wheel_current[i]);
    }
}

/* 理想 IMU：直接给云台对地的角度和角速度（安装矩阵、零偏都标好了，不漂、没噪声） */
static void Car_UpdateImu(InsFeedback_t *imu)
{
    memset(imu, 0, sizeof(*imu));
    imu->yaw_deg = Car_GimbalWorldYaw();
    imu->pitch_deg = car.pitch.rel_deg;
    imu->yaw_rate_dps = car.yaw.world_dps;
    imu->pitch_rate_dps = car.pitch.world_dps;
    imu->valid = car.imu_valid;
}

/* 一个云台轴走 dt：J·dω/dt = 电机力矩 − 摩擦（作用在相对底座的转速上）；撞到机械极限就停 */
static void Car_GimbalAxisStep(CarGimbalAxis_t *axis, int8_t direction, float base_dps, float dt)
{
    float torque = (float)direction * CAR_GM_KT * (float)axis->cmd_current * CAR_GM_A_PER_CMD
                 - CAR_GM_FRICTION * (axis->world_dps - base_dps) * CAR_RAD_PER_DEG;

    axis->world_dps += torque / CAR_GM_J * CAR_DEG_PER_RAD * dt;
    axis->rel_deg += (axis->world_dps - base_dps) * dt;
    if (axis->rel_deg > axis->mech_max_deg)
    {
        axis->rel_deg = axis->mech_max_deg;
        axis->world_dps = base_dps;
    }
    else if (axis->rel_deg < axis->mech_min_deg)
    {
        axis->rel_deg = axis->mech_min_deg;
        axis->world_dps = base_dps;
    }
}

/* 底盘刚体走 dt：轮子力 → Jᵀ 合成广义力 → 底盘系里的动力学（转动坐标系要加 wz × v 那一项） */
static void Car_ChassisStep(float dt)
{
    CarChassis_t *c = &car.chassis;
    float v[4];
    float force;
    float fx = 0.0f;
    float fy = 0.0f;
    float mz = 0.0f;
    float wz_rad = c->wz_dps * CAR_RAD_PER_DEG;
    float heading_rad = c->heading_deg * CAR_RAD_PER_DEG;
    float vx = c->vx_mps;
    float vy = c->vy_mps;
    uint8_t i;

    Car_WheelSpeeds(v);
    for (i = 0U; i < 4U; i++)
    {
        /* 掉线的电调收不到控制帧也照样会停；这里当作掉线就不出力 */
        float amps = (c->wheel_online[i] != 0U) ? (float)c->wheel_current[i] * CAR_C620_A_PER_CMD : 0.0f;

        force = (float)car_wheel_dir[i] * amps * CAR_M3508_KT_OUT / CHASSIS_WHEEL_RADIUS_M - CAR_WHEEL_DAMPING * v[i];
        fx += CHASSIS_XY_FACTOR * force;
        fy += CHASSIS_XY_FACTOR * car_j_vy[i] * force;
        mz += car_j_w[i] * CHASSIS_ROTATE_RADIUS_M * force;
        car_rotor_rev[i] += (float)car_wheel_dir[i] * v[i] / (CAR_TWO_PI * CHASSIS_WHEEL_RADIUS_M) * DJI_M3508_REDUCTION_RATIO * dt;
    }

    c->vx_mps += (fx / CAR_MASS_KG + wz_rad * vy) * dt;
    c->vy_mps += (fy / CAR_MASS_KG - wz_rad * vx) * dt;
    c->wz_dps += mz / CAR_IZ_KGM2 * CAR_DEG_PER_RAD * dt;
    c->x_m += (vx * cosf(heading_rad) - vy * sinf(heading_rad)) * dt;
    c->y_m += (vx * sinf(heading_rad) + vy * cosf(heading_rad)) * dt;
    c->heading_deg += c->wz_dps * dt;
}

static void Car_Trace(void)
{
    if (car.trace == NULL)
    {
        return;
    }
    fprintf(car.trace, "%lu,%d,%d,%.3f,%.3f,%.3f,%.4f,%.4f,%.3f,%.4f,%.2f,%d,%d,%d,%d\n",
            (unsigned long)car_tick, (int)Car_GimbalMode(), (int)car_chassis_rx.mode,
            Car_Heading(), Car_GimbalWorldYaw(), car.yaw.rel_deg, car.chassis.x_m, car.chassis.y_m,
            Car_VelocityDir(), Car_Speed(), car.chassis.wz_dps,
            car.chassis.wheel_current[0], car.chassis.wheel_current[1],
            car.chassis.wheel_current[2], car.chassis.wheel_current[3]);
}

void Car_TraceHeader(FILE *file)
{
    fprintf(file, "tick,gimbal_mode,chassis_mode_rx,heading,gimbal_world_yaw,yaw_rel,x,y,vel_dir,speed,wz_dps,"
                  "i_fl,i_fr,i_br,i_bl\n");
}

static void Car_Step(void)
{
    InsFeedback_t imu;
    GimbalFeedback_t feedback;
    GimbalCmd_t gimbal_cmd;
    ChassisCmd_t chassis_cmd;
    uint8_t i;

    car_tick++;

    /* 1. 两块板的 CAN 中断：电机反馈帧 */
    Car_SendFeedback();

    /* 2. 底盘板 Robot_Loop：用的是上一周期云台板发来的 0x110（两块板各跑各的，差 1 ms 左右） */
    ChassisLink_GetCmd(car_tick, &car_chassis_rx);
    Chassis_SetCmd(&car_chassis_rx);
    Chassis_Update(car_tick);
    (void)DJI_MotorManager_SendAll(&car_chassis_can1);
    (void)DJI_MotorManager_SendAll(&car_chassis_can2);

    /* 3. 云台板 Robot_Loop：IMU → 指令 → 云台 → 发电机帧 → 发 0x110 */
    Car_UpdateImu(&imu);
    Gimbal_SetImu(&imu);
    Gimbal_GetFeedback(&feedback);
    GimbalTestCmd_Update(car_tick, &feedback, &gimbal_cmd, &chassis_cmd);
    Gimbal_SetCmd(&gimbal_cmd);
    Gimbal_Update(car_tick);
    (void)DJI_MotorManager_SendAll(&car_gimbal_can1);
    (void)DJI_MotorManager_SendAll(&car_gimbal_can2);
    if (car.link_on != 0U)
    {
        (void)ChassisLink_SendCmd(&car_gimbal_hcan1, &chassis_cmd);
    }

    /* 4. 模型按收到的电流往前走 1 ms */
    for (i = 0U; i < CAR_SUBSTEPS; i++)
    {
        float base_dps = car.chassis.wz_dps;

        Car_ChassisStep(CAR_DT_S);
        Car_GimbalAxisStep(&car.yaw, GIMBAL_YAW_DIRECTION, base_dps, CAR_DT_S);
        Car_GimbalAxisStep(&car.pitch, GIMBAL_PITCH_DIRECTION, 0.0f, CAR_DT_S);
    }

    Car_Trace();
}

void Car_Run(uint32_t ms)
{
    uint32_t i;

    for (i = 0U; i < ms; i++)
    {
        Car_Step();
    }
}

void Car_Reset(void)
{
    FILE *trace = car.trace;
    uint8_t i;

    memset(&car, 0, sizeof(car));
    car.trace = trace;
    car.yaw.online = 1U;
    car.pitch.online = 1U;
    if (CAR_YAW_UNLIMITED)
    {
        car.yaw.mech_min_deg = -1.0e9f;
        car.yaw.mech_max_deg = 1.0e9f;
    }
    else
    {
        car.yaw.mech_min_deg = GIMBAL_YAW_MIN_DEG - 5.0f;
        car.yaw.mech_max_deg = GIMBAL_YAW_MAX_DEG + 5.0f;
    }
    car.pitch.mech_min_deg = GIMBAL_PITCH_MIN_DEG - 5.0f;
    car.pitch.mech_max_deg = GIMBAL_PITCH_MAX_DEG + 5.0f;
    for (i = 0U; i < 4U; i++)
    {
        car.chassis.wheel_online[i] = 1U;
    }
    car.imu_valid = 1U;
    car.link_on = 1U;
    memset(car_rotor_rev, 0, sizeof(car_rotor_rev));
    memset(&car_chassis_rx, 0, sizeof(car_chassis_rx));

    /* 时间只往前走：chassis_link 里存着上一个场景的帧和时间，时间倒回去的话旧帧会被当成“比现在还新” */
    car_tick += 1000U;

    gimbal_test.gimbal_mode = GIMBAL_MODE_RELAX;
    gimbal_test.yaw_deg = 0.0f;
    gimbal_test.pitch_deg = 0.0f;
    gimbal_test.chassis_mode = CHASSIS_MODE_RELAX;
    gimbal_test.vx_mps = 0.0f;
    gimbal_test.vy_mps = 0.0f;
    gimbal_test.yaw_square_deg = 0.0f;
    gimbal_test.pitch_square_deg = 0.0f;
    gimbal_test.square_period_ms = 2000U;

    /* 两块板的 Robot_Init：manager 初始化 → 模块注册电机 → 启动 */
    (void)DJI_MotorManager_Init(&car_gimbal_can1, &car_gimbal_hcan1);
    (void)DJI_MotorManager_Init(&car_gimbal_can2, &car_gimbal_hcan2);
    (void)Gimbal_Init(&car_gimbal_can1, &car_gimbal_can1);
    (void)DJI_MotorManager_Start(&car_gimbal_can1);
    (void)DJI_MotorManager_Start(&car_gimbal_can2);

    (void)DJI_MotorManager_Init(&car_chassis_can1, &car_chassis_hcan1);
    (void)DJI_MotorManager_Init(&car_chassis_can2, &car_chassis_hcan2);
    (void)Infantry_Chassis_Init(&car_chassis_can2);
    (void)ChassisLink_Init(&car_chassis_hcan1);
    (void)DJI_MotorManager_Start(&car_chassis_can1);
    (void)DJI_MotorManager_Start(&car_chassis_can2);

    Car_Run(100U);
}

float Car_GimbalWorldYaw(void)
{
    return Math_WrapDeg180(car.chassis.heading_deg + car.yaw.rel_deg);
}

float Car_Heading(void)
{
    return Math_WrapDeg180(car.chassis.heading_deg);
}

float Car_Speed(void)
{
    return sqrtf(car.chassis.vx_mps * car.chassis.vx_mps + car.chassis.vy_mps * car.chassis.vy_mps);
}

float Car_VelocityDir(void)
{
    float heading_rad = car.chassis.heading_deg * CAR_RAD_PER_DEG;
    float vx = car.chassis.vx_mps;
    float vy = car.chassis.vy_mps;

    return atan2f(vx * sinf(heading_rad) + vy * cosf(heading_rad),
                  vx * cosf(heading_rad) - vy * sinf(heading_rad)) * CAR_DEG_PER_RAD;
}

GimbalMode_e Car_GimbalMode(void)
{
    GimbalFeedback_t feedback;

    Gimbal_GetFeedback(&feedback);
    return feedback.mode;
}

void Car_ChassisRx(ChassisCmd_t *cmd)
{
    *cmd = car_chassis_rx;
}

int Car_MaxWheelCurrent(void)
{
    int max_current = 0;
    uint8_t i;

    for (i = 0U; i < 4U; i++)
    {
        if (abs(car.chassis.wheel_current[i]) > max_current)
        {
            max_current = abs(car.chassis.wheel_current[i]);
        }
    }
    return max_current;
}

uint8_t Car_WheelsFighting(void)
{
    float v[4];
    uint8_t pushing = 0U;
    uint8_t dragging = 0U;
    uint8_t i;

    Car_WheelSpeeds(v);
    for (i = 0U; i < 4U; i++)
    {
        /* 换算成“让车前进为正”的电流，和轮子转动方向比 */
        int current = car_wheel_dir[i] * car.chassis.wheel_current[i];

        if ((fabsf(v[i]) < CAR_FIGHT_MIN_SPEED_MPS) || (abs(current) < CAR_FIGHT_MIN_CURRENT))
        {
            continue;
        }
        if ((current > 0) == (v[i] > 0.0f))
        {
            pushing = 1U;
        }
        else
        {
            dragging = 1U;
        }
    }
    return (uint8_t)(pushing && dragging);
}
