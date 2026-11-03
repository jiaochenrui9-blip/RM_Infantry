/*
 * 整车主机仿真的场景：每个场景从 Car_Reset 开始，用 gimbal_test 下指令（和调试器里改变量一样），
 * 跑够时间后检查。阈值按“逻辑对不对”定，留了余量；模型参数是假设的，调参后某项不过，
 * 先分清是逻辑改坏了，还是新参数在这个模型上本来就不合适（见 README）。
 *
 * 用法：car_sim                 跑全部场景
 *       car_sim 场景名           只跑一个
 *       car_sim 场景名 out.csv   只跑一个，并把每毫秒的状态写进 CSV
 */

#include <math.h>
#include <stdio.h>
#include <string.h>

#include "car_sim.h"
#include "chassis_config.h"
#include "gimbal_test_cmd.h"
#include "user_math.h"

static int test_failed;
static int test_skipped;

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

#define SKIP(...)                       \
    do                                  \
    {                                   \
        printf("    跳过：");           \
        printf(__VA_ARGS__);            \
        printf("\n");                   \
        test_skipped = 1;               \
    } while (0)

/* 改云台模式后先跑 1 个周期：gimbal_test_cmd 会把目标同步成新模式下的当前角，之后再改目标 */
static void SetGimbalMode(GimbalMode_e mode)
{
    gimbal_test.gimbal_mode = mode;
    Car_Run(1U);
}

static ChassisMode_e RxMode(void)
{
    ChassisCmd_t rx;

    Car_ChassisRx(&rx);
    return rx.mode;
}

/* ---------------------------------------------------------------- 场景 */

/* 底盘 RELAX：给了速度也不动 */
static void Test_Relax(void)
{
    int max_current = 0;
    uint32_t i;

    Car_Reset();
    gimbal_test.vx_mps = 1.0f;
    for (i = 0U; i < 500U; i++)
    {
        Car_Run(1U);
        max_current = (Car_MaxWheelCurrent() > max_current) ? Car_MaxWheelCurrent() : max_current;
    }
    INFO("RELAX 500 ms：最大轮电流 %d，位移 %.3f m", max_current,
         sqrtf(car.chassis.x_m * car.chassis.x_m + car.chassis.y_m * car.chassis.y_m));
    CHECK(max_current == 0, "RELAX 时轮子电流应为 0");
    CHECK(RxMode() == CHASSIS_MODE_RELAX, "底盘收到的应是 RELAX，实际 %d", (int)RxMode());
}

/* 不跟随、直走：速度、方向对，车头不转，4 个轮子不互相顶 */
static void Test_NoFollowStraight(void)
{
    float speed_err = 0.0f;
    float dir_err = 0.0f;
    int fighting = 0;
    uint32_t i;

    Car_Reset();
    SetGimbalMode(GIMBAL_MODE_IMU);
    gimbal_test.chassis_mode = CHASSIS_MODE_NO_FOLLOW;
    gimbal_test.vx_mps = 1.5f;
    Car_Run(1500U);
    for (i = 0U; i < 1000U; i++)
    {
        Car_Run(1U);
        speed_err = fmaxf(speed_err, fabsf(Car_Speed() - 1.5f));
        dir_err = fmaxf(dir_err, fabsf(Car_VelocityDir()));
        fighting += Car_WheelsFighting();
    }
    INFO("不跟随直走 1.5 m/s：速度误差最大 %.3f m/s，方向最大偏 %.2f°，车头 %.2f°，内耗 %d/1000 个周期",
         speed_err, dir_err, Car_Heading(), fighting);
    CHECK(speed_err < 0.05f, "直走速度不对");
    CHECK(dir_err < 1.0f, "直走方向偏了");
    CHECK(fabsf(Car_Heading()) < 1.0f, "不跟随时车头不该转");
    CHECK(fighting == 0, "直走时有轮子在拽（内耗）");
}

/* 不跟随：云台转到车的左边 90°，按 W，车往枪口方向（车的左边）横移，车头不转 */
static void Test_NoFollowStrafe(void)
{
    float dir_err = 0.0f;
    uint32_t i;

    Car_Reset();
    SetGimbalMode(GIMBAL_MODE_IMU);
    gimbal_test.yaw_deg = 90.0f;
    Car_Run(1500U);
    gimbal_test.chassis_mode = CHASSIS_MODE_NO_FOLLOW;
    gimbal_test.vx_mps = 1.0f;
    Car_Run(1500U);
    for (i = 0U; i < 1000U; i++)
    {
        Car_Run(1U);
        dir_err = fmaxf(dir_err, fabsf(Math_WrapDeg180(Car_VelocityDir() - 90.0f)));
    }
    INFO("云台在左边 90° 按 W：速度方向 %.2f°（最大偏 %.2f°），速度 %.3f m/s，车头 %.2f°",
         Car_VelocityDir(), dir_err, Car_Speed(), Car_Heading());
    CHECK(dir_err < 2.0f, "应沿枪口方向横移");
    CHECK(fabsf(Car_Heading()) < 1.0f, "不跟随时车头不该转");
}

/* 跟随：云台甩到 60°，车头转过去对准，云台对地不动 */
static void Test_FollowTurn(void)
{
    float peak = 0.0f;
    float gimbal_dev = 0.0f;
    uint32_t settle_ms = 0U;
    uint32_t i;

    Car_Reset();
    SetGimbalMode(GIMBAL_MODE_IMU);
    gimbal_test.chassis_mode = CHASSIS_MODE_FOLLOW;
    Car_Run(200U);
    gimbal_test.yaw_deg = 60.0f;
    for (i = 1U; i <= 4000U; i++)
    {
        Car_Run(1U);
        peak = fmaxf(peak, Car_Heading());
        if (i > 1000U)  /* 前 1 s 云台自己在转 */
        {
            gimbal_dev = fmaxf(gimbal_dev, fabsf(Car_GimbalWorldYaw() - 60.0f));
        }
        if ((settle_ms == 0U) && (fabsf(Car_Heading() - 60.0f) < 1.0f))
        {
            settle_ms = i;
        }
    }
    INFO("云台甩到 60°：车头 %.2f°（峰值 %.2f°，%u ms 进入 ±1°），yaw_rel %.2f°；底盘转的时候云台对地最大偏 %.2f°",
         Car_Heading(), peak, (unsigned)settle_ms, car.yaw.rel_deg, gimbal_dev);
    CHECK(fabsf(Car_Heading() - 60.0f) < 1.0f, "车头应转到和云台同向");
    CHECK(peak < 65.0f, "跟随超调过大");
    CHECK(gimbal_dev < 1.0f, "底盘转的时候云台对地没稳住");
}

/* 跟随、边走边转：云台以 45°/s 转到 90°，行进方向一直沿枪口，最后车头对准。
   转弯要靠轮子速度环的误差产生向心力，行进方向会落后枪口几度，这是参数问题不是逻辑问题 */
static void Test_FollowDrive(void)
{
    float dir_err = 0.0f;
    uint32_t i;

    Car_Reset();
    SetGimbalMode(GIMBAL_MODE_IMU);
    gimbal_test.chassis_mode = CHASSIS_MODE_FOLLOW;
    gimbal_test.vx_mps = 1.5f;
    Car_Run(1000U);
    for (i = 1U; i <= 2000U; i++)
    {
        gimbal_test.yaw_deg = fminf(90.0f, 0.045f * (float)i);
        Car_Run(1U);
        dir_err = fmaxf(dir_err, fabsf(Math_WrapDeg180(Car_VelocityDir() - Car_GimbalWorldYaw())));
    }
    Car_Run(2000U);
    INFO("跟随边走边转：行进方向最多落后枪口 %.2f°；最后车头 %.2f°、行进方向 %.2f°、速度 %.3f m/s",
         dir_err, Car_Heading(), Car_VelocityDir(), Car_Speed());
    CHECK(dir_err < 6.0f, "行进方向应一直沿枪口");
    CHECK(fabsf(Car_Heading() - 90.0f) < 1.0f, "最后车头应对准云台");
    CHECK(fabsf(Car_VelocityDir() - 90.0f) < 1.0f, "最后应沿枪口走");
}

/* 小陀螺：v_mps 为 0 时原地转、不漂；不为 0 时对地沿枪口走。云台对地不动 */
static void RunSpin(float v_mps)
{
    float dir_sum = 0.0f;
    float dir_max = 0.0f;
    float speed_min = 1.0e9f;
    float wz_min = 1.0e9f;
    float wz_max = -1.0e9f;
    float gimbal_dev = 0.0f;
    float x0;
    float y0;
    float forward;
    float lateral;
    uint32_t i;

    if (!CAR_YAW_UNLIMITED)
    {
        SKIP("yaw 有限位（没滑环），云台跟不住一直转的底盘");
        return;
    }

    Car_Reset();
    SetGimbalMode(GIMBAL_MODE_IMU);
    gimbal_test.chassis_mode = CHASSIS_MODE_SPIN;
    gimbal_test.vx_mps = v_mps;
    Car_Run(2000U);
    x0 = car.chassis.x_m;
    y0 = car.chassis.y_m;
    for (i = 0U; i < 2000U; i++)
    {
        Car_Run(1U);
        wz_min = fminf(wz_min, car.chassis.wz_dps);
        wz_max = fmaxf(wz_max, car.chassis.wz_dps);
        gimbal_dev = fmaxf(gimbal_dev, fabsf(Car_GimbalWorldYaw()));
        if (v_mps > 0.0f)
        {
            dir_sum += Car_VelocityDir();
            dir_max = fmaxf(dir_max, fabsf(Car_VelocityDir()));
            speed_min = fminf(speed_min, Car_Speed());
        }
    }
    forward = car.chassis.x_m - x0;
    lateral = car.chassis.y_m - y0;
    INFO("小陀螺 %.0f~%.0f °/s，云台对地最大偏 %.2f°（纯 P 的稳态落后，见 Gimbal/HostTest 的 imu_spin）",
         wz_min, wz_max, gimbal_dev);
    CHECK(fabsf(wz_min - CHASSIS_SPIN_DPS) < 5.0f && fabsf(wz_max - CHASSIS_SPIN_DPS) < 5.0f, "小陀螺转速不对");
    CHECK(gimbal_dev < 3.0f, "小陀螺时云台对地没稳住");
    CHECK(Car_GimbalMode() == GIMBAL_MODE_IMU, "云台应保持 IMU 模式");
    if (v_mps > 0.0f)
    {
        INFO("按 W %.1f m/s：对地速度最小 %.2f m/s，方向平均偏 %.2f°、最大偏 %.2f°；2 s 往前 %.2f m、横向 %.2f m",
             v_mps, speed_min, dir_sum / 2000.0f, dir_max, forward, lateral);
        CHECK(speed_min > 0.9f * v_mps, "小陀螺时平移速度掉太多");
        CHECK(dir_max < 10.0f, "小陀螺时平移方向偏太多");
    }
    else
    {
        INFO("原地转 2 s 漂了 %.3f m", sqrtf(forward * forward + lateral * lateral));
        CHECK(sqrtf(forward * forward + lateral * lateral) < 0.02f, "原地小陀螺时车不该漂走");
    }
}

static void Test_SpinStill(void)
{
    RunSpin(0.0f);
}

static void Test_SpinDrive(void)
{
    RunSpin(1.0f);
}

/* 小陀螺 + 高速平移：轮速超过 CHASSIS_WHEEL_MAX_RPM，4 个轮子要等比例缩小。
   等比例缩小保证的是“平移 : 旋转”的比例和方向都不变，只是一起变慢；
   只削超的那个轮子的话，主要是旋转被砍得多（比例变了），方向也会歪一点 */
static void Test_SpinFastSaturated(void)
{
    float dir_max = 0.0f;
    float speed_min = 1.0e9f;
    float speed_max = 0.0f;
    float wz_min = 1.0e9f;
    float wz_max = -1.0e9f;
    float ratio_err = 0.0f;
    const float cmd_ratio = 3.0f / CHASSIS_SPIN_DPS;   /* 指令的 平移速度 / 旋转速度 */
    uint32_t i;

    if (!CAR_YAW_UNLIMITED)
    {
        SKIP("yaw 有限位（没滑环），不能开小陀螺");
        return;
    }

    Car_Reset();
    SetGimbalMode(GIMBAL_MODE_IMU);
    gimbal_test.chassis_mode = CHASSIS_MODE_SPIN;
    gimbal_test.vx_mps = 3.0f;
    Car_Run(3000U);
    for (i = 0U; i < 2000U; i++)
    {
        Car_Run(1U);
        dir_max = fmaxf(dir_max, fabsf(Car_VelocityDir()));
        speed_min = fminf(speed_min, Car_Speed());
        speed_max = fmaxf(speed_max, Car_Speed());
        wz_min = fminf(wz_min, car.chassis.wz_dps);
        wz_max = fmaxf(wz_max, car.chassis.wz_dps);
        if (car.chassis.wz_dps > 10.0f)
        {
            ratio_err = fmaxf(ratio_err, fabsf(Car_Speed() / car.chassis.wz_dps / cmd_ratio - 1.0f));
        }
    }
    INFO("小陀螺 + 按 W 3 m/s（轮速超 %.0f rpm 上限）：速度 %.2f~%.2f m/s，转速 %.0f~%.0f °/s，"
         "平移 / 旋转 和指令的比例最多差 %.1f%%，方向最大偏 %.2f°",
         (double)CHASSIS_WHEEL_MAX_RPM, speed_min, speed_max, wz_min, wz_max, ratio_err * 100.0f, dir_max);
    CHECK(ratio_err < 0.25f, "平移和旋转没有按同一比例缩小（4 个轮子没有等比例缩小？）");
    CHECK(dir_max < 10.0f, "超速缩放后平移方向偏太多");
    CHECK(speed_max < 3.0f, "应该真的超了上限、被缩小（场景本身没测到想测的）");
}

/* 云台在编码器模式下要跟随：云台板降成不跟随，底盘不会一直转（不降级的话会以 跟随Kp × 相对角 一直转） */
static void Test_FollowNeedsImu(void)
{
    float wz_max = 0.0f;
    uint32_t i;

    Car_Reset();
    SetGimbalMode(GIMBAL_MODE_ENCODER);
    gimbal_test.yaw_deg = 30.0f;
    Car_Run(1000U);
    gimbal_test.chassis_mode = CHASSIS_MODE_FOLLOW;
    for (i = 0U; i < 3000U; i++)
    {
        Car_Run(1U);
        wz_max = fmaxf(wz_max, fabsf(car.chassis.wz_dps));
    }
    INFO("编码器模式要跟随：底盘收到 mode %d，最大角速度 %.2f °/s，车头 %.2f°", (int)RxMode(), wz_max, Car_Heading());
    CHECK(RxMode() == CHASSIS_MODE_NO_FOLLOW, "应降成不跟随");
    CHECK(wz_max < 1.0f, "底盘不该一直转");
}

/* 板间断线：约 50 ms 后底盘无力（滑行）；接回后自动恢复（断线不锁定），加速度受斜坡限制 */
static void Test_LinkLoss(void)
{
    uint32_t relax_ms = 0U;
    float accel_max = 0.0f;
    float speed_prev;
    float speed_reconnect;
    uint32_t i;

    Car_Reset();
    SetGimbalMode(GIMBAL_MODE_IMU);
    gimbal_test.chassis_mode = CHASSIS_MODE_NO_FOLLOW;
    gimbal_test.vx_mps = 1.5f;
    Car_Run(2000U);
    car.link_on = 0U;
    for (i = 1U; i <= 200U; i++)
    {
        Car_Run(1U);
        if ((relax_ms == 0U) && (Car_MaxWheelCurrent() == 0))
        {
            relax_ms = i;
        }
    }
    INFO("断线：%u ms 后轮电流归零，底盘收到 mode %d；断线 200 ms 后还在以 %.2f m/s 滑行",
         (unsigned)relax_ms, (int)RxMode(), Car_Speed());
    CHECK((relax_ms > 0U) && (relax_ms <= CHASSIS_LINK_TIMEOUT_MS + 5U), "断线后应在约 50 ms 内无力");

    car.link_on = 1U;
    Car_Run(5U);
    speed_reconnect = Car_Speed();
    speed_prev = speed_reconnect;
    for (i = 0U; i < 1000U; i++)
    {
        Car_Run(1U);
        accel_max = fmaxf(accel_max, fabsf(Car_Speed() - speed_prev) / 0.001f);
        speed_prev = Car_Speed();
    }
    INFO("接回：mode %d，1 s 后 %.2f m/s（接回时 %.2f），最大加速度 %.2f m/s²（斜坡 %.1f）",
         (int)RxMode(), Car_Speed(), speed_reconnect, accel_max, (double)CHASSIS_ACCEL_MPS2);
    CHECK(RxMode() == CHASSIS_MODE_NO_FOLLOW, "接回后应自动回到运动模式");
    CHECK(fabsf(Car_Speed() - 1.5f) < 0.05f, "接回后应恢复到指令速度");
    CHECK(accel_max < CHASSIS_ACCEL_MPS2 * 1.2f, "恢复时加速度应受斜坡限制");
}

/* 一个轮子掉线：4 个都无力并锁住；重新上线仍锁住；给一次 RELAX 才恢复 */
static void Test_WheelOffline(void)
{
    Car_Reset();
    SetGimbalMode(GIMBAL_MODE_IMU);
    gimbal_test.chassis_mode = CHASSIS_MODE_NO_FOLLOW;
    gimbal_test.vx_mps = 1.0f;
    Car_Run(1500U);
    car.chassis.wheel_online[2] = 0U;
    Car_Run(150U);
    CHECK(Car_MaxWheelCurrent() == 0, "后右轮掉线后 4 个轮子都应无力，实际最大 %d", Car_MaxWheelCurrent());

    car.chassis.wheel_online[2] = 1U;
    Car_Run(300U);
    CHECK(Car_MaxWheelCurrent() == 0, "重新上线后应保持锁定");

    gimbal_test.chassis_mode = CHASSIS_MODE_RELAX;
    Car_Run(20U);
    gimbal_test.chassis_mode = CHASSIS_MODE_NO_FOLLOW;
    Car_Run(1500U);
    INFO("掉线 → 锁定 → 上线仍锁定 → 给一次 RELAX 后恢复到 %.2f m/s", Car_Speed());
    CHECK(fabsf(Car_Speed() - 1.0f) < 0.05f, "给一次 RELAX 后应恢复");
}

/* 小陀螺 → 跟随 → 不跟随：角速度平滑过渡，跟随后车头对准云台，最后停转 */
static void Test_ModeSwitch(void)
{
    float accel_max = 0.0f;
    float wz_prev;
    float follow_err;
    uint32_t i;

    if (!CAR_YAW_UNLIMITED)
    {
        SKIP("yaw 有限位（没滑环），不能开小陀螺");
        return;
    }

    Car_Reset();
    SetGimbalMode(GIMBAL_MODE_IMU);
    gimbal_test.chassis_mode = CHASSIS_MODE_SPIN;
    Car_Run(2000U);
    wz_prev = car.chassis.wz_dps;
    gimbal_test.chassis_mode = CHASSIS_MODE_FOLLOW;
    for (i = 0U; i < 3000U; i++)
    {
        Car_Run(1U);
        accel_max = fmaxf(accel_max, fabsf(car.chassis.wz_dps - wz_prev) / 0.001f);
        wz_prev = car.chassis.wz_dps;
    }
    follow_err = Math_WrapDeg180(Car_Heading() - Car_GimbalWorldYaw());
    gimbal_test.chassis_mode = CHASSIS_MODE_NO_FOLLOW;
    for (i = 0U; i < 1000U; i++)
    {
        Car_Run(1U);
        accel_max = fmaxf(accel_max, fabsf(car.chassis.wz_dps - wz_prev) / 0.001f);
        wz_prev = car.chassis.wz_dps;
    }
    INFO("小陀螺 → 跟随 → 不跟随：最大角加速度 %.0f °/s²（斜坡 %.0f），跟随后车头和云台差 %.2f°，最后 %.2f °/s",
         accel_max, (double)CHASSIS_WZ_ACCEL_DPS2, follow_err, car.chassis.wz_dps);
    CHECK(accel_max < CHASSIS_WZ_ACCEL_DPS2 * 1.3f, "切模式时角速度跳变");
    CHECK(fabsf(follow_err) < 1.0f, "从小陀螺切跟随后车头应对准云台");
    CHECK(fabsf(car.chassis.wz_dps) < 0.5f, "切到不跟随后底盘应停转");
}

/* ---------------------------------------------------------------- 入口 */

typedef struct
{
    const char *name;
    void (*run)(void);
    const char *desc;
} TestCase_t;

static const TestCase_t test_cases[] = {
    {"relax",              Test_Relax,            "底盘 RELAX 不动"},
    {"no_follow_straight", Test_NoFollowStraight, "不跟随直走，没有内耗"},
    {"no_follow_strafe",   Test_NoFollowStrafe,   "云台转 90° 按 W，沿枪口横移"},
    {"follow_turn",        Test_FollowTurn,       "跟随：车头转向云台"},
    {"follow_drive",       Test_FollowDrive,      "跟随边走边转"},
    {"spin_still",         Test_SpinStill,        "原地小陀螺"},
    {"spin_drive",         Test_SpinDrive,        "小陀螺 + 平移"},
    {"spin_fast",          Test_SpinFastSaturated, "小陀螺 + 高速平移（轮速超限等比例缩小）"},
    {"follow_needs_imu",   Test_FollowNeedsImu,   "编码器模式下跟随降成不跟随"},
    {"link_loss",          Test_LinkLoss,         "板间断线 → 无力，接回自动恢复"},
    {"wheel_offline",      Test_WheelOffline,     "轮子掉线锁定"},
    {"mode_switch",        Test_ModeSwitch,       "小陀螺 → 跟随 → 不跟随"},
};

int main(int argc, char **argv)
{
    const char *only = (argc >= 2) ? argv[1] : NULL;
    const char *trace_path = (argc >= 3) ? argv[2] : NULL;
    size_t count = sizeof(test_cases) / sizeof(test_cases[0]);
    size_t i;
    int run = 0;
    int failed = 0;
    int skipped = 0;

    if (trace_path != NULL)
    {
        car.trace = fopen(trace_path, "w");
        if (car.trace == NULL)
        {
            printf("打不开 %s\n", trace_path);
            return 2;
        }
        Car_TraceHeader(car.trace);
    }

    for (i = 0U; i < count; i++)
    {
        if ((only != NULL) && (strcmp(only, test_cases[i].name) != 0))
        {
            continue;
        }
        printf("[%s] %s\n", test_cases[i].name, test_cases[i].desc);
        test_failed = 0;
        test_skipped = 0;
        test_cases[i].run();
        printf("    %s\n", (test_failed != 0) ? "FAIL" : ((test_skipped != 0) ? "SKIP" : "PASS"));
        run++;
        failed += (test_failed != 0) ? 1 : 0;
        skipped += ((test_failed == 0) && (test_skipped != 0)) ? 1 : 0;
    }

    if (car.trace != NULL)
    {
        fclose(car.trace);
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

    printf("\n%d 个场景，%d 个通过，%d 个跳过（不适用于当前配置），%d 个失败\n",
           run, run - failed - skipped, skipped, failed);
    return (failed == 0) ? 0 : 1;
}
