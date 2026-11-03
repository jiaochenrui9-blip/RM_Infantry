# 云台主机测试（HostTest）

在**电脑上**编译运行云台的真代码，用一个简单的物理模型代替电机和底盘，验证控制逻辑、符号和状态机。
不需要 STM32、电机，几秒钟跑完；改了云台代码就跑一遍，确认没把原来对的东西改坏（回归测试）。

> 这是电脑上的独立 CMake 工程，**不要**加进 `Gimbal_test` 的 ARM 工程里。

## 1. 怎么跑

### 命令行（需要电脑上的 gcc + CMake）

```powershell
cd Test_Module/Gimbal/HostTest
cmake -S . -B build
cmake --build build
./build/gimbal_host_test            # Windows：build\gimbal_host_test.exe
```

全部通过时最后一行类似 `16 个场景，15 个通过，1 个跳过（不适用于当前配置），0 个失败`，返回值 0；有失败返回 1。也可以 `ctest --test-dir build`。

“跳过”是场景和现在的 `gimbal_config.h` 不适用：yaw 限位写 ±180（滑环、不限位）时跳过 `imu_limit_drag`，写成有限位时跳过 `imu_spin`、`encoder_wrap180`。两种配置都应该 0 失败。

Windows 上 gcc 可以用 MinGW（CLion 自带的 MinGW 工具链就行）或 WSL。输出里的中文如果乱码，在 cmd 里先执行 `chcp 65001`。

### CLion

`File → Open` 直接打开 `Test_Module/Gimbal/HostTest` 这个文件夹（当成一个新项目），工具链选电脑上的（MinGW / WSL），**不要选 ARM 工具链**。选中 `gimbal_host_test` 目标点运行即可，也能打断点单步看云台代码在模型里怎么走。

### 只跑一个场景、导出曲线

```powershell
gimbal_host_test ramp_ff                # 只跑 ramp_ff
gimbal_host_test ramp_ff ramp.csv       # 只跑 ramp_ff，并把每毫秒的状态写进 ramp.csv
```

CSV 列：`tick, mode, yaw_rel, yaw_world, yaw_current, pitch_rel, pitch_world, pitch_current, imu_valid`（角度是度，电流是 CAN 帧里的原始值；`yaw_rel` 是模型里的真值，滑环一直转时不回绕）。用 Excel 画图，和上板调参看波形是一回事。

## 2. 原理

```
test_gimbal.c（场景：给指令、改底盘 / IMU / 掉线状态、检查结果）
      │  每 1 ms 一个周期，顺序和 robot.c 的 Robot_Loop 一样
      ▼
真代码：gimbal.c、gimbal_test_cmd.c、DJI 驱动、PID、user_math、chassis_link
      │  控制帧（0x1FE）、板间帧 0x110  ▲ 反馈帧（0x205 / 0x206，按 GM6020 格式）
      ▼                               │
stub/：假 main.h（HAL 类型）、假 can_bus（发帧交给模型）
      │                               │
      ▼                               │
sim/：两个刚体轴 + 底盘 + 理想 IMU ─────┘
```

| 目录 / 文件 | 作用 |
|---|---|
| `stub/main.h` | 假的 HAL：`HAL_StatusTypeDef`、CAN 句柄和帧头、`HAL_GetTick`、开关中断。驱动以后用到新的 HAL 符号，编译报错时在这里补 |
| `stub/can_bus_stub.c` | 代替 `Shared/BSP/can_bus.c`：过滤器和启动直接成功，发出去的帧交给模型 |
| `Shared/Link/chassis_link.c`（真代码） | 云台板每周期发 0x110；模型把这一帧交给同一个文件里底盘板用的 `ChassisLink_ProcessRxFrame`，`Sim_ChassisCmd` 再用 `ChassisLink_GetCmd` 解出来，等于两块板之间的打包、解包、超时一起测 |
| `sim/gimbal_sim.c` | 模型和 1 ms 循环。每个周期：按真实帧格式把编码器（8192 档）、转速（整数 rpm）送进 `DJI_MotorManager_ProcessRxFrame` → IMU → 指令 → `Gimbal_Update` → `SendAll` → 从控制帧取电流 → 模型积分 1 ms |
| `test_gimbal.c` | 测试场景和 `main` |

因为 DJI 驱动用的是真代码，CAN 帧打包、`motor_direction` 取反、反馈解析、100 ms 掉线保护这一整条链路也一起被测到了。顺序和 `robot.c` 的 `Robot_Loop` 一样，最后一步是 `ChassisLink_SendCmd`。

### 模型

每个轴：$J\,\dot\omega = K_t I - b\,(\omega - \omega_{\text{底盘}}) - \tau_g\cos\theta$，撞到机械极限就停。

| 参数 | 值 | 说明 |
|---|---|---|
| $J$ | 0.005 kg·m² | 假设值 |
| $K_t$ | 0.741 N·m/A | GM6020 手册 |
| 电流换算 | ±16384 ↔ ±3 A | 电流控制模式 |
| $b$ | 0.005 N·m·s/rad | 轴承、滑环的粘滞摩擦（假设值），作用在相对底盘的转速上 |
| $\tau_g$ | 默认 0 | `sim.pitch_gravity_nm` |
| 机械极限 | 软限位往外 5°；yaw 限位写满 360°（±180，滑环）时 yaw 没有机械极限 | `Sim_Reset` 里，`SIM_YAW_UNLIMITED` 见 `gimbal_sim.h` |
| IMU | 理想：直接给对地真值 | 相当于安装矩阵、零偏都标好了，不漂、没噪声 |

`sim` 里能改的状态：底盘转速 / 倾斜（`base_dps`）、电机掉线（`online`）、手扶住（`held`）、IMU 失效 / 卡住（`imu_valid`、`imu_frozen`）、电机实际方向（`phys_dir`）。

## 3. 现有场景

| 名字 | 测什么 | 对应的 bug（注入后会失败） |
|---|---|---|
| `relax_zero` | RELAX 时电流为 0 | RELAX 也算 PID 输出电流 |
| `encoder_step` | 编码器模式阶跃停得住、超调不大 | |
| `ramp_ff` | 斜坡后段落后 < 1°（没有前馈约 12.5°） | 去掉速度前馈 |
| `soft_limit` | 指令超限位时停在限位上，冲过的量 < 2.5°（yaw 不限位时只测 pitch） | |
| `outside_limit` | pitch 靠在机械下限时进闭环，回到软限位内不冲过头 | |
| `hold_release` | 推到 +10° 扶 1 s 松手，回到 0° 不冲 | 斜坡步长用反馈算（第一版的 bug，松手冲到约 -39°） |
| `imu_hold` | IMU 模式下底盘转 60°、pitch 晃 ±5°，对地指向偏差 < 1° | |
| `imu_limit_drag` | 底盘一直转、顶到限位被带着走，超出软限位 < 5°（yaw 不限位时跳过） | 不在编码器系里限位 |
| `imu_wrap180` | 车头 170°、对地目标 -170°：往左转 20°，不绕 340° | 指令换坐标系时不回绕；切 IMU 时目标同步成编码器角 |
| `mode_switch` | ENCODER ↔ IMU 来回切 5 次，对地不动 | 切 IMU 时目标同步成编码器角 |
| `imu_invalid` | IMU 模式中失效 → 下一周期无力锁定，恢复后仍锁定 | 去掉 IMU 失效检查 |
| `imu_enter_invalid` | 标定中（valid = 0）切 IMU → 锁定 | 去掉 IMU 失效检查 |
| `motor_offline` | 电机掉线 → 无力锁定，上线后仍锁定 | 掉线不锁定 |
| `imu_spin` | yaw 不限位，底盘 360°/s 转 3 s：IMU 模式保持，相对角每圈跨 ±180° 时对地 yaw 不跳（稳定后变化 < 0.2°）。落后量会打印出来，见第 5 节（yaw 有限位时跳过） | |
| `encoder_wrap180` | yaw 不限位，相对角 170° 时指令给 -170°：往前转 20° 跨过 180°，不绕 340°（yaw 有限位时跳过） | 限位写满 360° 时斜坡步长不回绕 |
| `chassis_cmd` | 板间指令：vx、vy、yaw_rel 打包解包后一致；云台不在 IMU 模式（编码器模式、被锁在 RELAX）时跟随 / 小陀螺降成不跟随，IMU 模式原样发 | 去掉降级；降级看指令模式而不是实际模式；yaw_rel 发指令角而不是实际角 |

“对应的 bug”是故意往真代码里注入这些错误跑出来的（前 13 个场景 2026-11-01，后 3 个 2026-11-03）：每个都至少让一个场景失败，说明这些场景真的能抓到问题。

## 4. 加一个新场景

照着 `test_gimbal.c` 里现有的写：

```c
static void Test_MyCase(void)
{
    Sim_Reset();                        /* 1. 重新开始 */
    sim.yaw.base_deg = 30.0f;           /* 2. 改初始状态（可选） */
    Start();                            /* 3. RELAX 下跑 50 ms，电机上线 */
    SetMode(GIMBAL_MODE_IMU);           /* 4. 切模式（会同步目标），再改目标 */
    gimbal_test.yaw_deg = 10.0f;
    Sim_Run(1000U);                     /* 5. 跑 1 s */
    CHECK(fabsf(Sim_YawWorld() - 10.0f) < 0.5f, "应停在 10°");   /* 6. 检查 */
}
```

最后在 `test_cases[]` 里加一行 `{"my_case", Test_MyCase, "说明"}`。

## 5. 注意：模型能证明什么、不能证明什么

- **能**：符号和方向、坐标换算、限位、跨 ±180°、模式切换、锁定逻辑、改代码前后行为是否一致。
- **不能**：真云台的 PID 参数、齿隙 / 振动 / 结构变形、真实摩擦和重力、CAN 干扰和供电问题。模型参数是假设的，上车后的表现以实测为准。
- **测试用的是真的 `gimbal_config.h`**。上车调参后如果某个场景不过，先看失败的是哪类检查：
  - 锁定、切换不跳、跨 ±180° 这类**逻辑**检查不过 → 多半是代码改坏了；
  - 超调、落后这类**性能**检查不过 → 可能是新参数在这个假设的模型（$J$、摩擦）上不合适，不一定是 bug。可以按真云台的估算改 `sim/gimbal_sim.c` 里的 `SIM_J_KGM2` 等参数，再判断。
- **小陀螺时的稳态落后**（`imu_spin` 打印的那个数）：滑环摩擦随转速变大，两个环都只有 P 时云台会落后一个固定角度，约 摩擦力矩 /（力矩常数 × 电流换算 × 速度环 Kp × 角度环 Kp）。模型里 360°/s 时约 1.5°、30°/s 时约 0.1°；真车摩擦多大要实测。要消掉就给速度环加积分，或者按相对底盘的转速加摩擦前馈。
- 没覆盖的：ins（姿态解算）本身、重力和速度环积分、板间通信的中断时序（模型里没有中断，`chassis_link` 关中断拷贝、时间戳比 now 新这类问题测不到）。需要时照第 4 节加。
