# 整车主机仿真（CarSim）

在**电脑上**把云台板和底盘板的真代码编在一起，接一个整车模型，验证各个模式能不能正常配合：不跟随、跟随、小陀螺，以及板间断线、轮子掉线这些异常。
不需要 STM32、电机，一秒内跑完。改了云台、底盘、板间通信的代码就跑一遍。

> 这是电脑上的独立 CMake 工程，**不要**加进任何板子的 ARM 工程里。
> 云台单独的细节（限位、IMU 失效锁定、斜坡前馈……）在 `Test_Module/Gimbal/HostTest`，这里不重复，两个都跑。

## 1. 怎么跑

```powershell
cd Test_Module/CarSim
cmake -S . -B build
cmake --build build
./build/car_sim            # Windows：build\car_sim.exe
```

全部通过时最后一行类似 `12 个场景，12 个通过，0 个跳过（不适用于当前配置），0 个失败`，返回值 0；有失败返回 1。也可以 `ctest --test-dir build`。

CLion：`File → Open` 打开 `Test_Module/CarSim` 这个文件夹，工具链选电脑上的（MinGW / WSL），**不要选 ARM 工具链**。中文乱码时 cmd 先 `chcp 65001`。

只跑一个场景、导出曲线：

```powershell
car_sim spin_drive                 # 只跑 spin_drive
car_sim spin_drive spin.csv        # 并把每毫秒的状态写进 spin.csv
```

CSV 列：`tick, gimbal_mode, chassis_mode_rx（底盘板收到的模式）, heading（车头）, gimbal_world_yaw, yaw_rel, x, y（对地位置 m）, vel_dir（对地速度方向）, speed, wz_dps, i_fl, i_fr, i_br, i_bl（4 个轮子的电流原始值）`。用 Excel 画 x-y 散点图就是车的轨迹。

## 2. 原理

```
test_car.c（场景：改 gimbal_test、断线、掉线，检查结果）
   │ 每 1 ms：
   ├─ 电机反馈帧（GM6020 ×2、M3508 ×4，按真实帧格式）→ 两块板的 DJI 驱动
   ├─ 底盘板一周期：ChassisLink_GetCmd → Chassis_SetCmd → Chassis_Update → SendAll   （同 Chassis_test/App/robot.c）
   ├─ 云台板一周期：IMU → GimbalTestCmd_Update → Gimbal_Update → SendAll → ChassisLink_SendCmd（同 Gimbal_test/App/robot.c）
   │        └─ 0x110 交给底盘板的 ChassisLink_ProcessRxFrame，下一周期底盘板才用上（两块板差 1 ms）
   └─ 模型走 1 ms：0x200 帧的电流推底盘，0x1FE 帧的电流推云台；底盘转，云台 yaw 的底座跟着转
```

| 目录 / 文件 | 作用 |
|---|---|
| `sim/car_sim.c/.h` | 整车模型、两块板的循环、CAN 路由、CSV |
| `stub/can_bus_stub.c` | 代替 `Shared/BSP/can_bus.c`：两块板各两路 CAN 各一个句柄，发出去的帧按“哪块板、哪一路”交给模型或另一块板 |
| `test_car.c` | 场景和 `main` |
| 假的 `main.h` | 和云台主机测试共用 `Test_Module/Gimbal/HostTest/stub/main.h`，不另存一份 |

被测的真代码：`gimbal.c`、`gimbal_test_cmd.c`、`Infantry_Chassis.c`、`chassis_link.c`、DJI 驱动（GM6020、M3508、manager、motor_control）、PID、user_math。参数用的都是真的 `gimbal_config.h`、`chassis_config.h`。

### 模型

**底盘**：刚体，轮子不打滑。4 个全向轮装在四角、轮轴指向车中心（2026-11-11 从麦轮换过来）。每个轮子沿驱动方向的速度 = J·[vx, vy, wz]，第 i 行是 (cos θᵢ, sin θᵢ, R)，正 = 推车体逆时针转，和 `Infantry_Chassis.c` 的逆运动学同一套约定；辊子方向不受力。每个轮子的力 = 电机力矩 / 轮半径 − 滚动阻力，经 Jᵀ 合成底盘的力和力矩：

$$m(\dot v_x - \omega_z v_y) = F_x,\quad m(\dot v_y + \omega_z v_x) = F_y,\quad I_z\dot\omega_z = M_z$$

4 个速度环作用在同一个 3 自由度刚体上，轮子互相顶（内耗）在这里能看出来（`Car_WheelsFighting`）。

| 参数 | 值 | 说明 |
|---|---|---|
| 质量 $m$ | 20 kg | 假设值 |
| 转动惯量 $I_z$ | 0.6 kg·m² | 假设值 |
| M3508 输出轴转矩常数 | 0.3 N·m/A | 手册 |
| C620 电流换算 | ±16384 ↔ ±20 A | |
| 每个轮子滚动阻力 | 1 N·s/m | 假设值 |
| 轮半径、旋转半径、方向、ID | 读 `chassis_config.h` | 方向当作已经标对 |
| 驱动方向角 θ | 前左 135°、前右 45°、后右 315°、后左 225°（从车头往左量） | 模型按车的装法自己存一份（`car_wheel_theta_deg`），不用代码里那张表：表写错了才查得出来 |

**云台**：和 `Gimbal/HostTest` 一样的两个刚体轴，yaw 的底座就是底盘（底盘转多少、云台相对底盘反着转多少）。yaw 限位写满 360°（滑环）时没有 yaw 机械极限，否则在软限位外 5°。IMU 理想。

能改的状态：`car.link_on`（0 = 云台板不发 0x110）、`car.chassis.wheel_online[i]`（轮子掉线）、`car.yaw/pitch.online`、`car.imu_valid`。

## 3. 现有场景

| 名字 | 测什么 | 注入后会让它失败的 bug |
|---|---|---|
| `relax` | 底盘 RELAX 时给了速度也不动，轮电流 0 | |
| `no_follow_straight` | 不跟随直走 1.5 m/s：速度、方向对，车头不转，4 个轮子不内耗 | |
| `no_follow_strafe` | 云台转到车的左边 90° 按 W：沿枪口横移，车头不转 | 坐标变换旋转方向反了 |
| `follow_turn` | 跟随：云台甩到 60°，车头转过去、不超调，云台对地不动 | 跟随符号反了 |
| `follow_drive` | 跟随边走边转：行进方向一直沿枪口（落后 < 6°），最后车头对准 | 坐标变换方向反了；跟随符号反了 |
| `spin_still` | 原地小陀螺：转速对、车不漂、云台对地稳住 | |
| `spin_drive` | 小陀螺 + 按 W 1 m/s：对地沿枪口走（方向偏 < 10°），云台对地稳住 | 坐标变换方向反了；斜坡放到坐标变换之后（偏约 17°） |
| `spin_fast` | 小陀螺 + 高速平移：轮速超 `CHASSIS_WHEEL_MAX_RPM`，平移和旋转按同一比例缩小，转速不能被削没 | 超速时只削超的那个轮子。平移速度按配置算：车转到哪个角度都超、再多给 20%（小陀螺 36°/s 时 5.4 m/s，180°/s 时 4.6 m/s）。180°/s 时这个 bug 的比例差 223%（正确的 7%）；36°/s 时它把旋转整个削没（转速掉到 0），靠“转速不低于小陀螺的 30%”那条抓。2026-11-11 前写死过 3 m/s（麦轮）、4 m/s（全向轮、180°/s），小陀螺改慢以后都测不到 |
| `follow_needs_imu` | 云台在编码器模式下要跟随：降成不跟随，底盘不转 | 去掉降级（底盘会以 跟随 Kp × 相对角 = 90°/s 一直转） |
| `link_loss` | 板间断线约 50 ms 后无力；接回自动恢复（1.5 s 后回到指令速度），加速度受斜坡限制 | |
| `wheel_offline` | 一个轮子掉线 → 4 个都无力并锁住；上线仍锁住；给一次 RELAX 恢复 | 上线自动解锁 |
| `mode_switch` | 小陀螺 → 跟随 → 不跟随：角速度平滑，跟随后车头对准云台，最后停转 | 跟随符号反了 |

“注入后会让它失败”是 2026-11-03 故意往真代码里改错跑出来的，说明场景真能抓到问题。2026-11-11 换全向轮以后又改错了 4 处逆运动学，都能抓到：只削超的那个轮子（`spin_fast`）；把 ELRS 那份 `Omni_Remote.c` 的角度按位置照抄（x 右、顺时针的约定，5 个场景失败）；旋转项符号反（5 个）；前左、前右的角度对调（5 个）。yaw 有限位（没滑环）时 `spin_still`、`spin_drive`、`spin_fast`、`mode_switch` 自动跳过，两种配置都应 0 失败。

加新场景照 `test_car.c` 里现有的写：`Car_Reset()` → 改 `gimbal_test` / `car` → `Car_Run(ms)` → `CHECK(...)`，最后在 `test_cases[]` 里加一行。

## 4. 模型能证明什么、不能证明什么

- **能**：两块板之间的指令、模式配合（降级、跟随、小陀螺）、坐标变换和运动学的符号、斜坡位置、超速缩放、断线 / 掉线的处理，以及改代码前后行为是否一致。
- **不能**：真车的 PID 参数、打滑、悬挂和地面不平、真实的质量和摩擦、功率限制、CAN 负载和干扰。质量、惯量、阻力都是假设的。
- 上车调参后某个场景不过：锁定、降级、方向这类**逻辑**检查不过，多半是代码改坏了；落后几度、超调这类**性能**检查不过，可能只是新参数在这个模型上不合适，按真车估算改 `sim/car_sim.c` 开头的参数再看。

模型里看到的几个现象（参数问题，不是 bug，上车时留意）：

- 跟随转 60° 要 1.8 s 左右（`CHASSIS_FOLLOW_KP` = 3，不超调），想快可以往 5 调；
- 跟随边走边转、小陀螺时行进方向会落后：跟随边走边转最多 5.4°（检查限 6°），小陀螺 + 1 m/s 约 1.6°。轮子速度环要靠误差产生向心力 / 跟上变化的轮速；
- 小陀螺 180°/s 时云台对地落后约 0.7°：两个环都只有 P，滑环摩擦造成的稳态误差（见 `Gimbal/HostTest` 的 `imu_spin`）；
- 板间断线后电机直接失能，车靠阻力自己停（模型里 200 ms 后还有 1.47 m/s）。要不要先主动刹车再失能，上车看实际滑多远再定；
- 全向轮在电流限幅 3000 时出力不够（2026-11-11 换全向轮后看到的）：平移时每个轮子的力只有 √2/2 用在走的方向上，模型（20 kg）里最大加速度约 2 m/s²，跟不上 3 m/s² 的斜坡，速度环积分攒起来，起步、断线接回以后会冲到约 1.55 m/s（指令 1.5）再花半秒多回落；R 0.20 时转起来的力矩也只有麦轮模型（0.40）的一半。和麦轮模型比，跟随边走边转的落后从 3.1° 变成 5.4°，跟随转 60° 从 1.67 s 变成 1.77 s。上车把电流限幅放开（果园机器人最后用 8000）或者把斜坡调小再看。
