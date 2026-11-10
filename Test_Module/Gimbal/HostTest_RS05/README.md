# 云台主机测试（RS05 版，HostTest_RS05）

在**电脑上**编译运行 `Gimbal_test_RS05` 的真云台代码和真 RS05 驱动，用模型代替 RS05 电机、云台和底盘。
和 `../HostTest`（GM6020 版）是同一套场景：`test_gimbal.c`、假 `main.h` 直接用那边的，这里只有电机模型（`sim/`）和假 can_bus（`stub/`）。

> 这是电脑上的独立 CMake 工程，**不要**加进 `Gimbal_test_RS05` 的 ARM 工程里。

## 怎么跑

和 `../HostTest/README.md` 第 1 节一样，换成这个目录：

```powershell
cd Test_Module/Gimbal/HostTest_RS05
cmake -S . -B build
cmake --build build
./build/gimbal_rs05_host_test            # Windows：build\gimbal_rs05_host_test.exe
```

全部通过时最后一行是 `16 个场景，15 个通过，1 个跳过（不适用于当前配置），0 个失败`。只跑一个场景、导出 CSV 的用法也一样；CSV 里的电流列是电机实际输出的 iq（mA）。

## 和 GM6020 版的区别

| | GM6020 版（HostTest） | RS05 版（这里） |
|---|---|---|
| 被测的电机驱动 | DJI 驱动（0x1FE 控制帧、0x205 / 0x206 反馈） | `Shared/Devices/RS05/rs05.c`（扩展帧） |
| 反馈 | 每个周期开头模型主动发一帧 | 只有应答：驱动每发一帧，模型马上回一帧类型 2，云台下个周期读到 |
| 电流 | 控制帧里的原始值（±16384 = ±3 A），0.741 N·m/A | iq_ref（A），只有运行在电流模式时才出力，0.67 N·m/A |
| 其他 | | 使能 / 停止、run_mode（运行中写不进，上电默认 0 = 运控）、canTimeout（到了自己进 Reset）、上电读数落在 0~2π、通电时 ±4π 循环 |

`sim.yaw.online = 0` 表示 CAN 断了：电机收不到指令、也不回反馈，但还通着电（位置照常数圈，canTimeout 100 ms 后自己进 Reset）。`Sim_Reset` 后电机在第一个周期才上电，所以场景在 `Sim_Reset` 和 `Start` 之间摆的位置，就是电机上电时的位置。

模型参数（转动惯量、摩擦）和 GM6020 版一样，是假设值。**不模拟**：齿隙、上电约 1.4 s 不回话、故障位、发送邮箱满。

## 注意

- 改 `../HostTest/test_gimbal.c` 时，两个版本都会用到：场景里只用 `gimbal_sim.h` 的接口和 `gimbal_config.h` 里两版都有的宏（模式、限位、斜坡），别直接用 GM6020 专有的（`*_ESC_ID`、`*_ECD_OFFSET`、16384）。改完两个都跑一遍。
- 2026-11-08 的验证（改配置、故意注入错误）记在 `Gimbal_test_RS05/notes.md` 最后一节。速度环增益没换算这类错，这个理想模型抓不到。
