/*
 * 云台主机测试：每个场景从 Sim_Reset 开始，用 gimbal_test 下指令（和调试器里改变量一样），
 * 跑够时间后检查结果。阈值按“逻辑对不对”定，留了余量；调参后某项不过时，
 * 先看是逻辑改坏了，还是新参数在这个假设的模型上本来就不合适（见 README）。
 *
 * 用法：gimbal_host_test                 跑全部场景
 *       gimbal_host_test 场景名           只跑一个
 *       gimbal_host_test 场景名 out.csv   只跑一个，并把每毫秒的状态写进 CSV（可以用 Excel / VOFA+ 画图）
 */

#include <math.h>
#include <stdio.h>
#include <stdlib.h>
#include <string.h>

#include "gimbal_config.h"
#include "gimbal_sim.h"
#include "gimbal_test_cmd.h"
#include "user_math.h"

static int test_failed;     /* 当前场景没通过的检查数 */

#define CHECK(cond, ...)                \
    do                                  \
    {                                   \
        if (!(cond))                    \
        {                               \
            printf("    [FAIL] ");      \
            printf(__VA_ARGS__);        \
            printf("\n");               \
            test_failed++;              \
        }                               \
    } while (0)

#define INFO(...)                       \
    do                                  \
    {                                   \
        printf("    ");                 \
        printf(__VA_ARGS__);            \
        printf("\n");                   \
    } while (0)

/* 改模式后先跑 1 个周期：gimbal_test_cmd 会把目标同步成新模式下的当前角，之后再改目标 */
static void SetMode(GimbalMode_e mode)
{
    gimbal_test.mode = mode;
    Sim_Run(1U);
}

/* 先在 RELAX 下跑 50 ms：电机收到反馈“上线”，test_cmd 记住当前模式 */
static void Start(void)
{
    Sim_Run(50U);
}

static float MaxF(float a, float b)
{
    return (a > b) ? a : b;
}

static float MinF(float a, float b)
{
    return (a < b) ? a : b;
}

/* ---------------------------------------------------------------- 场景 */

/* RELAX 时不管指令是什么，两个电机都发 0 电流 */
static void Test_RelaxOutputsZero(void)
{
    int max_current = 0;
    uint32_t i;

    Sim_Reset();
    Start();
    gimbal_test.yaw_deg = 30.0f;
    gimbal_test.pitch_deg = 10.0f;
    for (i = 0U; i < 300U; i++)
    {
        Sim_Run(1U);
        max_current = (abs(sim.yaw.cmd_current) > max_current) ? abs(sim.yaw.cmd_current) : max_current;
        max_current = (abs(sim.pitch.cmd_current) > max_current) ? abs(sim.pitch.cmd_current) : max_current;
    }
    INFO("RELAX 300 ms 内最大电流 %d", max_current);
    CHECK(Sim_Mode() == GIMBAL_MODE_RELAX, "模式应为 RELAX，实际 %d", (int)Sim_Mode());
    CHECK(max_current == 0, "RELAX 时电流应为 0，实际最大 %d", max_current);
}

/* 编码器模式阶跃：yaw 0→20°、pitch 0→10°，停得住、超调不大 */
static void Test_EncoderStep(void)
{
    float yaw_peak = 0.0f;
    float pitch_peak = 0.0f;
    uint32_t i;

    Sim_Reset();
    Start();
    SetMode(GIMBAL_MODE_ENCODER);
    Sim_Run(300U);
    gimbal_test.yaw_deg = 20.0f;
    gimbal_test.pitch_deg = 10.0f;
    for (i = 0U; i < 2000U; i++)
    {
        Sim_Run(1U);
        yaw_peak = MaxF(yaw_peak, sim.yaw.rel_deg);
        pitch_peak = MaxF(pitch_peak, sim.pitch.rel_deg);
    }
    INFO("yaw 终值 %.2f°（峰值 %.2f°），pitch 终值 %.2f°（峰值 %.2f°）",
         sim.yaw.rel_deg, yaw_peak, sim.pitch.rel_deg, pitch_peak);
    CHECK(fabsf(sim.yaw.rel_deg - 20.0f) < 0.5f, "yaw 应停在 20° 附近");
    CHECK(fabsf(sim.pitch.rel_deg - 10.0f) < 0.5f, "pitch 应停在 10° 附近");
    CHECK(yaw_peak < 23.0f, "yaw 超调过大");
    CHECK(pitch_peak < 13.0f, "pitch 超调过大");
}

/* 斜坡 + 速度前馈：目标按 100°/s 走时，云台紧跟着走（没有前馈会一直落后约 斜坡速度 / 角度环 Kp = 12.5°）。
   斜坡刚开始时速度环要先把云台加速起来，会落后一两度，所以只检查后段（400 ms 以后）的稳定落后。 */
static void Test_RampFeedforward(void)
{
    float start_lag = 0.0f;
    float max_lag = 0.0f;
    float expected;
    uint32_t i;

    Sim_Reset();
    Start();
    SetMode(GIMBAL_MODE_ENCODER);
    Sim_Run(200U);
    gimbal_test.yaw_deg = 60.0f;
    for (i = 1U; i <= 600U; i++)
    {
        Sim_Run(1U);
        expected = MinF(60.0f, GIMBAL_RAMP_DEG_PER_MS * (float)i);
        if (i < 400U)
        {
            start_lag = MaxF(start_lag, expected - sim.yaw.rel_deg);
        }
        else
        {
            max_lag = MaxF(max_lag, expected - sim.yaw.rel_deg);
        }
    }
    Sim_Run(1500U);
    INFO("斜坡起步时最大落后 %.2f°，后段最大落后 %.2f°，终值 %.2f°", start_lag, max_lag, sim.yaw.rel_deg);
    CHECK(max_lag < 1.0f, "斜坡后段落后 %.2f°，速度前馈可能没起作用", max_lag);
    CHECK(fabsf(sim.yaw.rel_deg - 60.0f) < 0.5f, "yaw 应停在 60° 附近");
}

/* 软限位：指令超出限位，云台停在限位上。
   软限位限的是目标，云台带着斜坡的速度过去，会像普通阶跃一样冲过一点（和 encoder_step 的超调差不多），
   所以软限位要比机械极限往里留余量。这里检查冲过的量小于仿真里留的 5° 余量的一半。 */
static void Test_SoftLimit(void)
{
    float yaw_max = -1000.0f;
    float pitch_max = -1000.0f;
    uint32_t i;

    Sim_Reset();
    Start();
    SetMode(GIMBAL_MODE_ENCODER);
    gimbal_test.yaw_deg = GIMBAL_YAW_MAX_DEG + 60.0f;
    gimbal_test.pitch_deg = GIMBAL_PITCH_MAX_DEG + 30.0f;
    for (i = 0U; i < 3000U; i++)
    {
        Sim_Run(1U);
        yaw_max = MaxF(yaw_max, sim.yaw.rel_deg);
        pitch_max = MaxF(pitch_max, sim.pitch.rel_deg);
    }
    INFO("yaw 最大 %.2f°（软限位 %.0f°），pitch 最大 %.2f°（软限位 %.0f°）",
         yaw_max, GIMBAL_YAW_MAX_DEG, pitch_max, GIMBAL_PITCH_MAX_DEG);
    CHECK(yaw_max < GIMBAL_YAW_MAX_DEG + 2.5f, "yaw 冲过软限位太多");
    CHECK(pitch_max < GIMBAL_PITCH_MAX_DEG + 2.5f, "pitch 冲过软限位太多");
    CHECK(fabsf(sim.yaw.rel_deg - GIMBAL_YAW_MAX_DEG) < 0.5f, "yaw 应停在限位上");
    CHECK(fabsf(sim.pitch.rel_deg - GIMBAL_PITCH_MAX_DEG) < 0.5f, "pitch 应停在限位上");
}

/* pitch 断电后靠在机械下限（软限位外）时进闭环：回到软限位内，不往上冲 */
static void Test_EnterFromOutsideLimit(void)
{
    float pitch_max = -1000.0f;
    uint32_t i;

    Sim_Reset();
    sim.pitch.rel_deg = sim.pitch.mech_min_deg;
    Start();
    SetMode(GIMBAL_MODE_ENCODER);
    for (i = 0U; i < 1500U; i++)
    {
        Sim_Run(1U);
        pitch_max = MaxF(pitch_max, sim.pitch.rel_deg);
    }
    INFO("起点 %.1f°，终值 %.2f°，最高 %.2f°（软下限 %.0f°）",
         sim.pitch.mech_min_deg, sim.pitch.rel_deg, pitch_max, GIMBAL_PITCH_MIN_DEG);
    CHECK(fabsf(sim.pitch.rel_deg - GIMBAL_PITCH_MIN_DEG) < 0.5f, "pitch 应停在软下限上");
    CHECK(pitch_max < GIMBAL_PITCH_MIN_DEG + 1.0f, "pitch 回软限位时冲过头");
}

/* 手把云台推到 +10° 扶住 1 s 再松手：回到目标 0°，不冲到别处。
   扶住期间云台跟不上，目标不能一直往前跑（第一版用反馈算斜坡步长时，松手后冲到了 -48°）；
   以后速度环加了积分，这个场景也能看出积分饱和松手后冲多少。 */
static void Test_HoldAndRelease(void)
{
    float yaw_min = 1000.0f;
    float yaw_max = -1000.0f;
    uint32_t i;

    Sim_Reset();
    Start();
    SetMode(GIMBAL_MODE_ENCODER);
    Sim_Run(300U);
    sim.yaw.rel_deg = 10.0f;
    sim.yaw.held = 1U;
    Sim_Run(1000U);
    sim.yaw.held = 0U;
    for (i = 0U; i < 1500U; i++)
    {
        Sim_Run(1U);
        yaw_min = MinF(yaw_min, sim.yaw.rel_deg);
        yaw_max = MaxF(yaw_max, sim.yaw.rel_deg);
    }
    INFO("松手后 yaw 范围 %.2f° ~ %.2f°，终值 %.2f°", yaw_min, yaw_max, sim.yaw.rel_deg);
    CHECK(fabsf(sim.yaw.rel_deg) < 0.5f, "松手后应回到 0°");
    CHECK(yaw_min > -2.0f, "松手后往反方向冲过头");
    CHECK(yaw_max < 10.5f, "松手后先往外跑了");
}

/* IMU 模式自稳：底盘 yaw 以 30°/s 转、pitch 方向 ±5° 晃，云台对地指向不变 */
static void Test_ImuHoldWhileChassisMoves(void)
{
    float yaw0;
    float pitch0;
    float yaw_dev = 0.0f;
    float pitch_dev = 0.0f;
    uint32_t i;

    Sim_Reset();
    Start();
    SetMode(GIMBAL_MODE_IMU);
    Sim_Run(300U);
    yaw0 = Sim_YawWorld();
    pitch0 = Sim_PitchWorld();

    sim.yaw.base_dps = 30.0f;
    for (i = 0U; i < 2000U; i++)
    {
        /* 底盘 pitch 倾角 5·sin(2πt)，1 Hz */
        sim.pitch.base_dps = 5.0f * 6.2831853f * cosf(6.2831853f * (float)i / 1000.0f);
        Sim_Run(1U);
        yaw_dev = MaxF(yaw_dev, fabsf(Math_WrapDeg180(Sim_YawWorld() - yaw0)));
        pitch_dev = MaxF(pitch_dev, fabsf(Sim_PitchWorld() - pitch0));
    }
    INFO("底盘转了 %.1f°，云台相对底盘 %.1f°；对地 yaw 最大偏 %.2f°，pitch 最大偏 %.2f°",
         sim.yaw.base_deg, sim.yaw.rel_deg, yaw_dev, pitch_dev);
    CHECK(sim.yaw.rel_deg < -55.0f, "底盘转了 60°，云台相对底盘应反向转约 60°");
    CHECK(yaw_dev < 1.0f, "底盘转动时云台对地 yaw 没稳住");
    CHECK(pitch_dev < 1.0f, "底盘晃动时云台对地 pitch 没稳住");
}

/* IMU 模式下底盘一直转，云台顶到 yaw 软限位后被带着走，不会冲出太多（notes 已知问题第 10 条，约 4°） */
static void Test_ImuDraggedAtLimit(void)
{
    float yaw_min = 1000.0f;
    uint32_t i;

    Sim_Reset();
    Start();
    SetMode(GIMBAL_MODE_IMU);
    Sim_Run(300U);
    sim.yaw.base_dps = 30.0f;
    for (i = 0U; i < 5000U; i++)
    {
        Sim_Run(1U);
        yaw_min = MinF(yaw_min, sim.yaw.rel_deg);
    }
    sim.yaw.base_dps = 0.0f;
    Sim_Run(1000U);
    INFO("底盘转了 %.0f°，云台相对底盘最小 %.2f°（软限位 %.0f°），底盘停下后 %.2f°",
         sim.yaw.base_deg, yaw_min, GIMBAL_YAW_MIN_DEG, sim.yaw.rel_deg);
    CHECK(yaw_min > GIMBAL_YAW_MIN_DEG - 5.0f, "顶在限位上时超出软限位太多");
    CHECK(fabsf(sim.yaw.rel_deg - GIMBAL_YAW_MIN_DEG) < 0.5f, "底盘停下后应回到软限位上");
}

/* IMU 模式跨 ±180°：车头朝 170°，对地目标改成 -170°（往左再转 20°），走近路、不从背面绕 */
static void Test_ImuWrap180(void)
{
    float yaw_min = 1000.0f;
    uint32_t i;

    Sim_Reset();
    sim.yaw.base_deg = 170.0f;
    Start();
    SetMode(GIMBAL_MODE_IMU);
    Sim_Run(300U);
    gimbal_test.yaw_deg = -170.0f;
    for (i = 0U; i < 2000U; i++)
    {
        Sim_Run(1U);
        yaw_min = MinF(yaw_min, sim.yaw.rel_deg);
    }
    INFO("对地 yaw 终值 %.2f°，相对底盘 %.2f°（应约 20°），过程中最小 %.2f°",
         Sim_YawWorld(), sim.yaw.rel_deg, yaw_min);
    CHECK(fabsf(Math_WrapDeg180(Sim_YawWorld() + 170.0f)) < 0.5f, "对地 yaw 应停在 -170°");
    CHECK(fabsf(sim.yaw.rel_deg - 20.0f) < 0.5f, "应往左转 20°，而不是绕 340°");
    CHECK(yaw_min > -1.0f, "转动方向反了（先往右走了）");
}

/* 模式来回切不跳：车头朝 40°、底盘 pitch 倾 3°（IMU 角和编码器角都不一样），ENCODER ↔ IMU 来回切 */
static void Test_ModeSwitchNoJump(void)
{
    static const GimbalMode_e sequence[] = {GIMBAL_MODE_IMU, GIMBAL_MODE_ENCODER, GIMBAL_MODE_IMU,
                                            GIMBAL_MODE_ENCODER, GIMBAL_MODE_IMU};
    float yaw0;
    float pitch0;
    float yaw_dev = 0.0f;
    float pitch_dev = 0.0f;
    uint32_t i;
    uint32_t k;

    Sim_Reset();
    sim.yaw.base_deg = 40.0f;
    sim.pitch.base_deg = 3.0f;
    Start();
    SetMode(GIMBAL_MODE_ENCODER);
    Sim_Run(300U);
    yaw0 = Sim_YawWorld();
    pitch0 = Sim_PitchWorld();
    for (k = 0U; k < sizeof(sequence) / sizeof(sequence[0]); k++)
    {
        SetMode(sequence[k]);
        for (i = 0U; i < 300U; i++)
        {
            Sim_Run(1U);
            yaw_dev = MaxF(yaw_dev, fabsf(Math_WrapDeg180(Sim_YawWorld() - yaw0)));
            pitch_dev = MaxF(pitch_dev, fabsf(Sim_PitchWorld() - pitch0));
        }
        CHECK(Sim_Mode() == sequence[k], "第 %u 次切换后模式应为 %d，实际 %d",
              (unsigned)(k + 1U), (int)sequence[k], (int)Sim_Mode());
    }
    INFO("来回切 %u 次，对地 yaw 最大动 %.3f°，pitch 最大动 %.3f°",
         (unsigned)(sizeof(sequence) / sizeof(sequence[0])), yaw_dev, pitch_dev);
    CHECK(yaw_dev < 0.5f, "切换模式时 yaw 跳了");
    CHECK(pitch_dev < 0.5f, "切换模式时 pitch 跳了");
}

/* IMU 模式中 IMU 失效：下一周期就无力并锁住；IMU 恢复也不自己解锁，要给一次 RELAX */
static void Test_ImuInvalidLocks(void)
{
    Sim_Reset();
    Start();
    SetMode(GIMBAL_MODE_IMU);
    Sim_Run(300U);
    sim.yaw.base_dps = 30.0f;       /* 底盘在转的时候 IMU 卡住，最危险 */
    sim.imu_frozen = 1U;
    sim.imu_valid = 0U;
    Sim_Run(1U);
    CHECK(Sim_Mode() == GIMBAL_MODE_RELAX, "IMU 失效后应立即进入 RELAX");
    CHECK((sim.yaw.cmd_current == 0) && (sim.pitch.cmd_current == 0), "IMU 失效后电流应为 0");

    sim.imu_frozen = 0U;
    sim.imu_valid = 1U;
    Sim_Run(200U);
    CHECK(Sim_Mode() == GIMBAL_MODE_RELAX, "IMU 恢复后应保持锁定，不能自己回到闭环");

    SetMode(GIMBAL_MODE_RELAX);
    SetMode(GIMBAL_MODE_IMU);
    Sim_Run(50U);
    CHECK(Sim_Mode() == GIMBAL_MODE_IMU, "给一次 RELAX 后应能重新进入 IMU 模式");
    INFO("失效 → RELAX 锁定 → 恢复后仍锁定 → 给 RELAX 后解锁");
}

/* 上电标定中（valid = 0）就切 IMU 模式：锁住，不会用无效数据闭环 */
static void Test_EnterImuWhileInvalid(void)
{
    Sim_Reset();
    sim.imu_valid = 0U;
    Start();
    SetMode(GIMBAL_MODE_IMU);
    Sim_Run(100U);
    CHECK(Sim_Mode() == GIMBAL_MODE_RELAX, "IMU 无效时不应进入 IMU 模式");

    sim.imu_valid = 1U;
    Sim_Run(100U);
    CHECK(Sim_Mode() == GIMBAL_MODE_RELAX, "valid 变 1 后也应保持锁定");

    SetMode(GIMBAL_MODE_RELAX);
    SetMode(GIMBAL_MODE_IMU);
    Sim_Run(50U);
    CHECK(Sim_Mode() == GIMBAL_MODE_IMU, "给一次 RELAX 后应能进入 IMU 模式");
    INFO("标定中切 IMU → 锁定 → 给 RELAX 后解锁");
}

/* 闭环中电机掉线（拔 CAN 线）：超过 100 ms 后无力并锁住，重新上线也要给一次 RELAX */
static void Test_MotorOfflineLocks(void)
{
    Sim_Reset();
    Start();
    SetMode(GIMBAL_MODE_ENCODER);
    Sim_Run(300U);
    sim.yaw.online = 0U;
    Sim_Run(150U);
    CHECK(Sim_Mode() == GIMBAL_MODE_RELAX, "yaw 掉线后应进入 RELAX");
    CHECK((sim.yaw.cmd_current == 0) && (sim.pitch.cmd_current == 0), "掉线后两个电机电流都应为 0");

    sim.yaw.online = 1U;
    Sim_Run(200U);
    CHECK(Sim_Mode() == GIMBAL_MODE_RELAX, "重新上线后应保持锁定");

    SetMode(GIMBAL_MODE_RELAX);
    SetMode(GIMBAL_MODE_ENCODER);
    Sim_Run(50U);
    CHECK(Sim_Mode() == GIMBAL_MODE_ENCODER, "给一次 RELAX 后应能重新闭环");
    INFO("掉线 → RELAX 锁定 → 上线后仍锁定 → 给 RELAX 后解锁");
}

/* ---------------------------------------------------------------- 入口 */

typedef struct
{
    const char *name;
    void (*run)(void);
    const char *desc;
} TestCase_t;

static const TestCase_t test_cases[] = {
    {"relax_zero",       Test_RelaxOutputsZero,         "RELAX 时电流为 0"},
    {"encoder_step",     Test_EncoderStep,              "编码器模式阶跃"},
    {"ramp_ff",          Test_RampFeedforward,          "斜坡 + 速度前馈不落后"},
    {"soft_limit",       Test_SoftLimit,                "软限位"},
    {"outside_limit",    Test_EnterFromOutsideLimit,    "从软限位外进闭环"},
    {"hold_release",     Test_HoldAndRelease,           "手扶住再松手"},
    {"imu_hold",         Test_ImuHoldWhileChassisMoves, "IMU 模式自稳"},
    {"imu_limit_drag",   Test_ImuDraggedAtLimit,        "IMU 模式顶限位被底盘带着走"},
    {"imu_wrap180",      Test_ImuWrap180,               "IMU 模式跨 ±180° 走近路"},
    {"mode_switch",      Test_ModeSwitchNoJump,         "ENCODER ↔ IMU 切换不跳"},
    {"imu_invalid",      Test_ImuInvalidLocks,          "IMU 失效锁定"},
    {"imu_enter_invalid", Test_EnterImuWhileInvalid,    "IMU 无效时不许进 IMU 模式"},
    {"motor_offline",    Test_MotorOfflineLocks,        "电机掉线锁定"},
};

int main(int argc, char **argv)
{
    const char *only = (argc >= 2) ? argv[1] : NULL;
    const char *trace_path = (argc >= 3) ? argv[2] : NULL;
    size_t count = sizeof(test_cases) / sizeof(test_cases[0]);
    size_t i;
    int run = 0;
    int failed = 0;

    if (trace_path != NULL)
    {
        sim.trace = fopen(trace_path, "w");
        if (sim.trace == NULL)
        {
            printf("打不开 %s\n", trace_path);
            return 2;
        }
        Sim_TraceHeader(sim.trace);
    }

    for (i = 0U; i < count; i++)
    {
        if ((only != NULL) && (strcmp(only, test_cases[i].name) != 0))
        {
            continue;
        }
        printf("[%s] %s\n", test_cases[i].name, test_cases[i].desc);
        test_failed = 0;
        test_cases[i].run();
        printf("    %s\n", (test_failed == 0) ? "PASS" : "FAIL");
        run++;
        failed += (test_failed != 0) ? 1 : 0;
    }

    if (sim.trace != NULL)
    {
        fclose(sim.trace);
    }

    if (run == 0)
    {
        printf("没有叫 %s 的场景，可用的有：\n", only);
        for (i = 0U; i < count; i++)
        {
            printf("  %s\n", test_cases[i].name);
        }
        return 2;
    }

    printf("\n%d 个场景，%d 个通过，%d 个失败\n", run, run - failed, failed);
    return (failed == 0) ? 0 : 1;
}
