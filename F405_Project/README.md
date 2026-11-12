# F405_Project：底盘主控（F405 板）

队友打的 F405 板当步兵的底盘主控：接收云台主控（C 板）经板间 CAN 发来的模式和速度，控制四个 M3508。底盘代码和 `Test_Module/Chassis/Chassis_test`（C 板版）是同一份，只有两路 CAN 的分配按整车接线对调了。进度和注意事项见 `notes.md`。

## CAN 分配

| 总线 | 接什么 | 帧 |
|---|---|---|
| CAN1：PB8 RX / PB9 TX，STB 接 PC1；插座 CN7、CN15（2-pin：1 CANL、2 CANH） | 四个 C620 + M3508（全向轮，装在四角），电调 ID 1 前右、2 后右、3 后左、4 前左（俯视顺时针） | 每 1 ms 发一帧 0x200，收反馈 0x201~0x204 |
| CAN2：PB12 RX / PB13 TX，STB 接 PC2；插座 CN8、CN9（4-pin：1 5V、2 GND、3 CANH、4 CANL） | 板间：经底盘的分电板、滑环到云台 C 板的 CAN1，这一路上还有云台的两个 RS05 | 只收 0x110（云台板每 1 ms 发一帧），不发 |

- 两路都是 1 Mbps，开了自动离线恢复和自动重发；板上各有一个 120 Ω 终端（R10、R11，没有开关）。
- 两个 CAN 收发器的 STB 由 MCU 拉低才工作，改 .ioc 时别删 PC1、PC2。
- CN8 / CN9 的 1 脚是这块板的 5 V，接分电板时空着，只接 GND、CANH、CANL。

## 运行起来是什么样

- 上电是 RELAX（四个电机电流 0），CAN1 照样每 1 ms 发一帧全 0 的 0x200。
- 50 ms 内收到过 CAN2 上的 0x110，才按帧里的模式动：RELAX、不跟随、跟随云台、小陀螺。平移速度在云台朝向的坐标系里，底盘用帧里的云台相对角转到自己的坐标系。帧格式见 `../Shared/Link/chassis_link.h`。
- 运动模式下任一个电机 100 ms 没反馈：四个都无力并锁住，要收到一次 RELAX 才解除。所以云台主控上电要先发一段 RELAX。
- 没有云台板、单独测底盘时：临时在 `App/robot.c` 里把 `ChassisLink_GetCmd` 换成调试器里改的指令，或者用 USB-CAN 往 CN8 / CN9 发 0x110。

## 文件

- `App/robot.c`：两路 CAN 的 manager、1 ms 调度、CAN 接收分发。和 Chassis_test 只差 CAN 分配那三处。
- `App/Infantry_Chassis.c/.h`：底盘模块（模式、斜坡、坐标变换、全向轮逆运动学、掉线锁定）。各轮驱动方向角在 `chassis_wheel_theta`：从车头往左量，轮子正转推车体俯视逆时针转。
- `App/chassis_config.h`：电调 ID、方向、电流限幅、速度环 PID、轮子尺寸、跟随和小陀螺参数。大多还没标定：方向是推出来的 4 个 −1，轮子半径 0.076、轮心到车中心 0.20 都待量。
- 公共代码（DJI 驱动、PID、chassis_link、can_bus）在同级的 `../Shared`。
- 外设都在 `F405_Project.ioc` 里配，用 CubeMX 生成，不手写初始化代码。

## 编译、下载

- CLion 打开本目录，用 CMake 的 Debug 预设编译，输出 `build/Debug/F405_Project.elf`。
- 下载用 OpenOCD + DAPLink，SWD 接 CN5（1 SWDIO、2 SWCLK、3 GND、4 3V3）。
- 板子要接 24 V：CAN 收发器的 5 V 只能由 24 V 降压得到，只接调试器时 CAN 不能用；USB 的 VBUS 没接，只插 USB 板子不上电。
- 整车上电池先到 3508 分电板，再进这块板的一个 XT30，从另一个 XT30 出去给 CAN2 / RS05 总线分电板（yaw RS05 和过滑环到云台的电）：三个 XT30 并联，给云台、yaw 的电都从这块板穿过去。板上没有保险丝、没有防反接，插电池前看清正负。图见 `../docs/整车电源树.svg`。

## 调试器里看

- `chassis_mode`、`chassis_fault`、收到的指令 `chassis_cmd`，斜坡后的 `chassis_vx_mps`、`chassis_vy_mps`、`chassis_wz_dps`（都在 `Infantry_Chassis.c`）。
- `chassis_motors[0..3]` 的 `feedback_count`（涨就说明收到反馈）、`speed_rpm`（转子 rpm）、`speed.target_rpm`、`output_current`。下标是前左、前右、后右、后左，对应电调 ID 4、1、2、3。
- 板间最后一帧的时间 `chassis_link_tick`（在 `chassis_link.c`），它跟着 HAL tick 走就说明在收 0x110。

第一次接电机：先架空底盘，电流限幅保持小值（现在 3000，约 3.7 A），按 `Test_Module/Chassis/Chassis_test/notes.md`「上板步骤与验收」一步步来；那边写的“4 个 C620 接 CAN2”，在这块板上是 CAN1。
