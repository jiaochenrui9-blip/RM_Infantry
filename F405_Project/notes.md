# F405_Project 笔记（底盘主控，队友打的 F405 板）

> 和 `Test_Module/Chassis/Chassis_test`（C 板版）并列：CubeMX 这一层（.ioc、Core、启动文件、链接脚本）按这块板改，App 用 Chassis_test 的底盘代码。**现在 App 暂时换成了 M2006 上板测试**（2026-11-09），底盘代码不在这个工程里，恢复办法见「现在跑的：M2006 上板测试」。
> 底盘本身的进度、接口、上板步骤、容易错的地方都以 `Chassis_test/notes.md` 为准，这里只记这块板的硬件和与 C 板版不一样的地方。跨模块约定见 `RM_Infantry/notes.md`。
> 最后更新：2026-11-09（同日后来去掉底盘逻辑，改成一个 M2006 的速度环测试。总线先是 CAN2，后改到 CAN1，最后电机经分电板接到 CAN2，又改回 CAN2）

## 当前状态

- 2026-11-09 用户拿来队友打的 F405 板（用作底盘主控）：这个目录原来只有 CubeMX 新建的空工程（只开了 SWD，时钟是 16 MHz HSI）和网表 `405板子的网表.enet`（嘉立创 EDA 导出的 JSON）。用户要求照 C 板工程的格式做成差不多的工程。做法：.ioc 按网表配置，用 CubeMX 命令行生成；`App/` 从 Chassis_test 原样复制，底盘模块也带过来了。
- ARM Debug 清理编译 0 警告：FLASH 21108 B，RAM 2952 B。同一天用同样方法编 Chassis_test，结果是 20876 B / 2952 B。多出的 232 B 来自三处：STB 两个引脚的初始化（`HAL_GPIO_WritePin` +50 B、`MX_GPIO_Init` +52 B）；`HSE_VALUE` 不同，`HAL_RCC_GetSysClockFreq` 编出来多 128 B；F405 没有 ETH/DCMI，少了 3 个默认中断函数（−6 B）。
- 在副本上用 CubeMX 原地重新生成过一次：除 CubeMX 自己的记录文件 `.mxproject`，所有文件都没变，USER CODE 段也保住了。
- 同日后来，用户让先去掉底盘逻辑，改成一个 M2006（ID 1）转到 500 的上板测试，见下一节。先写在 CAN2 上，用户随后让改到 CAN1；后来电机经分电板接到 CAN2，又改回 CAN2。用户自己改了 `DJI_MotorManager_Init` 那一行，Claude 补上了接收回调的 `&hcan2` 和 manager 的改名。ARM Debug 清理编译 0 警告：FLASH 14772 B，RAM 1904 B。
- 2026-11-09 第一次上板：M2006 不转。OpenOCD 读下来软件和配置都对，两路 CAN 收发器既发不出去、也解不出帧。用户确认 24 V 和 TJA1044 的 5 V 都正常。后来用户拿电压表量 U3 的 STB 时电机转了，拿开就停。再用 MCU 逐个测芯片引脚：U3 的 8 脚（STB）没焊上，U4 的 1 脚（TXD）、4 脚（RXD）没焊上，软件不用改。用户重焊以后，CAN1 正常了，M2006 平均稳在 500 rpm（波动约 ±50，PID 还没调）；U4 还不行，像是地（2 脚和底部 EP）没接上，要再焊。U4 再焊后又测了一次：CAN2 测试帧报"没有应答"（LEC 3），U4 也正常了。中间有一次两路都不行，是 24 V 还没接上时测的。之后两路都正常，M2006 在 CAN1 上平均稳在 500 rpm（PID 还没调）。后来用户把 CAN2 接到分电板（上面接了电调），两路又都发不出去了。查下来是分电板那根线把这块板的 5 V 拉低了，拔掉线就恢复，两个收发器都正常；要先对好线序（最好不接 5 V 那根），再接分电板测 CAN2。详细记录见「上板记录」。
- 原来的空工程备份在 `%TEMP%\f405_backup_original`，不在仓库里，临时目录清掉就没了。那只是 CubeMX 的空工程，随时能重新建。

## 现在跑的：M2006 上板测试（2026-11-09）

用户要求：去掉 main 之前的逻辑，注册一个 M2006，ID 为 1，速度控制到 500。总线先说 CAN2，随后改成 CAN1；电机经分电板接到 CAN2 以后，又改回 CAN2。

- **代码**：`App/robot.c` 整个换成了测试（`robot.h`、main.c 没动）。
  - `Robot_Init`：CAN2 的 manager `robot_can2` → `DJI_M2006_Register`（ID 1，0x200 槽位 0）→ `MotorControl_Init` → 切速度模式 → 目标 500 → 启动 CAN2。CAN2 的大疆过滤器是组 14，manager 按 `Instance` 自己选。先切模式再给目标，因为 `MotorControl_SetMode` 会把目标清零。
  - `Robot_Loop`：每 1 ms 调一次 `MotorControl_Update`，再 `SendAll` 发 0x200。电机 100 ms 没反馈时输出 0，这是电机层自带的。
  - 接收回调只把 `hcan == &hcan2` 的帧交给 manager。换总线时这里也要改：只改 `DJI_MotorManager_Init` 那一行的话，反馈根本进不来，电机会一直被判掉线。
  - CAN1 没有建 manager，也没启动。`MX_CAN1_Init` 照常跑，因为 .ioc 没改。
  - 换总线只改 robot.c 里的四处：`hcan1` / `hcan2`、manager 变量名、接收回调的判断、注释。
- **参数**（robot.c 开头的宏）：

  | 宏 | 值 | 说明 |
  |---|---|---|
  | `M2006_TEST_ESC_ID` | 1 | C610 电调 ID |
  | `M2006_TEST_TARGET_RPM` | 500 | 转子 rpm，和电机层的单位一样；减速比 36，输出轴约 13.9 rpm |
  | `M2006_TEST_CURRENT_LIMIT` | 2000 | C610 的 ±10000 对应 ±10 A，这里是 2 A |
  | `M2006_TEST_SPEED_KP/KI/KD` | 3 / 20 / 0 | **没调过** |

- **速度环参数的来历**：用的是 DJI 驱动工程（`C:\Clion\DJI_Motor_Driver`）测 M3508 时的起点。
  - PID.c 的积分是 Ki × ∫e·dt（按秒算），所以 Ki 20 就相当于 `motor_control.c` 里那个没被用到的 `MOTOR_CONTROL_SPEED_KI 0.02` 按每拍累加的写法（0.02 × 1000 Hz）。
  - `C:\Clion\F405_M2006_CAN1`（今天建的手写工程，没有 .ioc）写的是 Ki 0.02，配上这个 PID.c，积分几乎为零。
  - 积分项被电机层限在 ±1000（1 A）。
- **调试器里看**：
  - 反馈：`robot_m2006.feedback_count`（涨就说明收到反馈）、`robot_m2006.speed_rpm`。
  - 输出：`robot_m2006.output_current`。
  - 目标：`robot_m2006.speed.target_rpm`，可以直接改。
- **接线**：
  - 板子接 24 V，CAN 收发器的 5 V 只能由 24 V 降压得到；C610 也要 24 V。
  - 现在 C610 经分电板接 CN8 或 CN9（CAN2，GH 4P）：3 CANH、4 CANL、2 GND，**1 脚是这块板的 5 V，不要接到分电板上**（接了会把 5 V 拉低，两路 CAN 都罢工，见「上板记录」）。板子这头有 120 Ω（R11）。
  - 改回 CAN1 的话，C610 接 CN7 或 CN15（GH 1.25 2P）：1 CANL、2 CANH。线序和 C 板 CAN1、C620 自带的 2-pin CAN 线一样；C610 的线没核对过。板子这头有 120 Ω（R10）。
  - 电调 ID 设成 1。设置方法以 C610 手册为准，`硬件资料/` 里还没有 C610 手册。
- **注意**：上电后只要收到反馈电机就会自己转，所以先固定好、保持空载。如果一闭环就抖或啸叫，先把 Kp 降到 1~2。

主机检查（2026-11-09，测试代码在临时目录，没进仓库）：编译了 robot.c 原文件，加上 Shared 的驱动、motor_control、PID 原文件，CAN 用桩函数。

- 结果：
  - 只启动了 CAN1，大疆过滤器配在 CAN1 的组 0。
  - M2006 的反馈 ID 是 0x201，控制帧 0x200、槽位 0。
  - 切模式后目标 500 还在。
  - 没收到反馈时每 1 ms 发一帧全 0 的 0x200，CAN2 上不发。
  - CAN2 来的帧不处理。
  - 第一拍电流 1510（3 × 500 + 积分 10），其他槽位为 0。
  - 停止反馈 100 ms 时还在出力，第 101 ms 输出归 0。
- 闭环用的是粗模型（转子 rpm' = A × 电流 − 2 × rpm），A 是估出来的，没有实测：
  - A = 95 rpm/s 每单位（按估计的转子惯量）：稳在 500，峰值 508。
  - A = 477（惯量只有估计值的 1/5）：先冲到 720 再回到 500。
  - 只说明方向和收敛没问题，参数要上板调。
- 改到 CAN1 以后重跑了一遍，结果一样，0 个失败。
- 改回 CAN2 以后又重跑了一遍：只启动了 CAN2，过滤器在 CAN2 的组 14，0x200 只从 CAN2 发，CAN1 的帧不处理。其余结果一样，0 个失败。

恢复成底盘程序：

1. 把 `Test_Module/Chassis/Chassis_test/App` 的五个文件（robot.c、robot.h、Infantry_Chassis.c/.h、chassis_config.h）拷回 `App/`。
2. 顶层 CMakeLists.txt 用 Chassis_test 的那份，改两处：工程名改成 `F405_Project`；`SHARED_DIR` 改成 `../Shared`。
3. 用下面「两个工程怎么保持一致」的命令对一遍。.ioc 不用动。

## 和 Chassis_test 的区别（底盘版时只有这些）

| 文件 | 区别 |
|---|---|
| `F405_Project.ioc` | 芯片 STM32F405RGTx（LQFP64）。<br>HSE 8 MHz、PLLM 4（C 板 12 MHz、PLLM 6）；其余时钟一样：SYSCLK 168 MHz、APB1 42 MHz、APB2 84 MHz、PLLQ 4。<br>CAN 配置都一样：分频 3 / BS1 9 / BS2 4 = 1 Mbps，开自动离线恢复，不自动重发，RX0 中断优先级 0。<br>CAN1 引脚 PB8 RX / PB9 TX（C 板 PD0/PD1），CAN2 引脚 PB12 RX / PB13 TX（C 板 PB5/PB6）。<br>多了 PC1 `CAN1_STB`、PC2 `CAN2_STB` 两个推挽输出，初始低电平，见「上板前要知道的」第 1 条 |
| 生成的代码 | `can.c` 只差引脚。<br>`gpio.c` 多了 STB 两个引脚，`main.h` 多了它们的宏。<br>`main.c` 只差 `PLLM = 4`。<br>`stm32f4xx_hal_conf.h` 的 `HSE_VALUE` 是 8000000。<br>启动文件、链接脚本、`STM32F405xx` 宏、`stm32f405xx.h` 换成 F405 的 |
| `CMakeLists.txt` | 工程名 `F405_Project`。工程在仓库根目录下一层，所以 `SHARED_DIR` 是 `../Shared`（Chassis_test 是 `../../../Shared`） |

一样的部分：`App/` 五个文件（`diff -r` 没有差别）、其余 Core 文件、Drivers 的文件列表（只差芯片头文件）、`CMakePresets.json`。

现在跑 M2006 测试，App 和 CMakeLists 的 `target_sources` 跟上表不一样：App 只有 robot.c、robot.h；CMake 不编 chassis_link、dji_m3508、user_math，改编 dji_m2006。.ioc 和生成的代码没变。

### 两个工程怎么保持一致

两边不会自动同步。Chassis_test 的 App 或 .ioc 改了，这边要手动跟着改；反过来也一样，因为 CarSim（整车主机仿真）编的是 Chassis_test 的 App。现在这边是 M2006 测试，App 不用同步，恢复成底盘时直接拷最新的过来。底盘版时的对比方法：

```bash
cd /c/Project/RM_Infantry
diff -r --strip-trailing-cr Test_Module/Chassis/Chassis_test/App F405_Project/App   # 应该没有输出
diff --strip-trailing-cr Test_Module/Chassis/Chassis_test/CMakeLists.txt F405_Project/CMakeLists.txt   # 只差工程名和 SHARED_DIR 两行
```

两边的 .ioc 芯片不同，没法直接 diff。Chassis_test 加了外设，这边要按下面的引脚表，在 CubeMX 里重新选引脚。

## 这块板的硬件（2026-11-09 按网表整理，还没对照实物）

| 功能 | 引脚 / 器件 | 备注 |
|---|---|---|
| 主控 | STM32F405RGT6（LQFP64） | 1 MB Flash、192 KB RAM（含 64 KB CCM），和 C 板的 F407IGH6 一样 |
| 晶振 | HSE 8 MHz（X1，负载电容 20 pF） | 没有 32.768 kHz 低速晶振（PC14/PC15 悬空） |
| SWD | CN5（Molex PicoBlade 1.25 mm 4P）：1 SWDIO（PA13）、2 SWCLK（PA14）、3 GND、4 3.3 V | 4 脚接的是板子的 3.3 V |
| 复位 / 启动 | SW1 按键接 NRST（R25 10 k 上拉）；BOOT0 经 R9 10 k 下拉 | 没有用户按键 |
| CAN1 | PB8 RX / PB9 TX → U3 TJA1044GTK/3，STB 接 PC1。插座 CN7、CN15（GH 1.25 2P）：1 CANL、2 CANH | 线序和 C 板的 CAN1 一样。R10 120 Ω 板载终端，没有开关 |
| CAN2 | PB12 RX / PB13 TX → U4 TJA1044GTK/3，STB 接 PC2。插座 CN8、CN9（GH 4P）：1 5V、2 GND、3 CANH、4 CANL | 线序和 C 板的 CAN2 一样。R11 120 Ω 板载终端 |
| BMI088 | SPI1：PA5 SCK / PA7 MOSI / PA6 MISO（SDO1、SDO2 并在一起）。片选：加速度计 PA4、陀螺仪 PC3。INT1（加速度计）接 PC0，INT3（陀螺仪）接 PC4；INT2、INT4 没接 | 和 C 板不一样：C 板 SPI1 是 PB3/PA7/PB4，陀螺仪片选 PB0，中断在 PC4/PC5。这块板没有加热电阻，也没有 IST8310 |
| DBUS | H1（1 5V、2 GND、3 DBUS）→ R16 100 k 下拉 → U6 SN74LVC1G14 反相 → USART3_RX PB11 | 和 C 板一样是硬件反相 |
| 串口 | USART1：PA9 TX / PA10 RX → CN10。USART2：PA2 TX / PA3 RX → CN11。两个都是 GH 4P：1 接 MCU 的 RX、2 接 MCU 的 TX、3 GND、4 5V | 丝印上的编号以实物为准 |
| RS485 ×2 | UART4（PA0 TX / PA1 RX）→ U8 SIT3088，RE#/DE 并在一起接 PC8，A/B 接 CN14 的 3、4 脚。USART6（PC6 TX / PC7 RX）→ U9 SIT3088，RE#/DE 接 PD2，A/B 接 CN13 | 收发器是 3.3 V 供电；DI、RO 各有 5.1 k 上拉；R15、R14 120 Ω 板载终端。CN13、CN14：1 5V、2 GND、3 A、4 B |
| USB | Type-C，PA11 DM / PA12 DP；CC1、CC2 各 5.1 k 下拉 | **VBUS 没有接到板上任何地方** |
| 电源 | 24 V（XT30：CN1、CN2 母座，CN4 公座，三个并联）→ U1 TPS54540 → 5 V → U7 SY8089 → 3.3 V；24 V 上并了 D1 SMBJ30CA（TVS 管） | 两个 TJA1044 的 VCC 和各插座的 5V 都来自这路 5 V，只能由 24 V 降压得到 |
| 网表里只连到 MCU 的网络 | TIM3_CH1~4（PB4、PB5、PB0、PB1）、TIM12_CH1/2（PB14、PB15）、UART3_TX（PB10）、I2C3（PA8 SCL、PC9 SDA）、SPI3（PA15 CS、PC10 SCK、PC11 MISO、PC12 MOSI） | 起了网络名，但网表里没接到任何插座或器件。板上有没有做成焊盘，要看实物或 PCB |
| 没接 | PC13、PC14、PC15、PC5、PB2、PB3、PB6、PB7；CN6（XT30）两个脚都没接 | |
| 没有 | LED、蜂鸣器、用户按键、电池电压检测、IMU 加热 | 调试只能靠调试器看变量（以后可加 USB 虚拟串口） |

## 上板前要知道的（和 C 板不一样的地方）

1. **CAN 收发器的 STB 由 MCU 控制。** C 板上 STB 直接接地；这块板接在 PC1、PC2。TJA1044 手册（NXP Rev. 8.0，7.1 节、7.3.2 节）的规定：
   - STB 低电平是正常模式；高电平是待机，待机时不能发送，也不能正常接收。
   - STB 内部上拉，悬空就是待机。

   所以 .ioc 里 PC1、PC2 必须是输出、初始低电平。现在 `MX_GPIO_Init` 在 CAN 初始化之前就把它们拉低了。以后改 .ioc 别删这两个引脚：删了编译不会报错，但两路 CAN 都不通。复位后到 `MX_GPIO_Init` 之前 STB 是悬空的，收发器处于待机，不会往总线上发东西；从待机切到正常最长 47 µs，而 CAN 要到 `Robot_Init` 里才启动，早就过了这个时间。
2. **只接 DAPLink 时 CAN 不能用。** 收发器的 VCC 是 5 V，只能由 24 V 降压得到，USB 的 VBUS 也没接。TJA1044 手册 7.3.3 节：VCC 欠压时进待机，不管 STB 是什么电平。这一点和 C 板只接调试器时一样（根目录 notes 第 5 节），要接 24 V。
3. **只插 USB 板子不上电。** 以后做 USB 虚拟串口也要接 24 V；还要把 PLLQ 改成 7（现在是 4，即 84 MHz，和 C 板一样）。USB 没有 VBUS 检测，CubeMX 里 VBUS sensing 保持关闭。
4. **下载。** CLion 打开 `F405_Project`，照 Chassis_test 建「OpenOCD 下载并运行」配置。DAPLink 的 board config 用同一个 `D:\OpenOCD_Config\DAPLink\stm32f4.cfg`（用的是通用的 `target/stm32f4x.cfg`，F405 也适用）。SWD 线按 CN5 的线序接（1 SWDIO、2 SWCLK、3 GND、4 3V3），和 C 板的接口不一定一样，接之前先核对。
5. **以后要用 BMI088 时：**
   - SPI1 引脚是 PA5/PA6/PA7。
   - .ioc 里把 PA4、PC3 的标签起成 `BMI088_ACC_CS`、`BMI088_GYRO_CS`，生成的 `main.h` 宏就会覆盖 `Shared/Devices/BMI088/BMI088.h` 里 C 板的默认值（那里有 `#ifndef`）。
   - 底盘现在不用 IMU，没配置。

## 工程是怎么建的（2026-11-09）

1. 网表是 JSON。用 Python 读出每个器件 `pinInfoMap` 里的网络名，得到 MCU 每个引脚接到哪里（脚本放在临时目录，没进仓库），整理成上面的表。
2. .ioc 的做法：
   - 以队友那份 F405 .ioc（CubeMX 6.15.0）为底，芯片相关的行保留。
   - 照 Chassis_test.ioc 加上 CAN1/CAN2、NVIC、RCC、Project Manager 的设置；Code Generator 只拷用到的库（`LibraryCopy=1`），Drivers 共 76 个文件、6.3 MB。
   - 引脚按上表改，再加上 PC1、PC2。
   - 在空目录里用 CubeMX 6.15 命令行生成（做法见 Chassis_test/notes.md「工程是怎么建的」），再整体替换原来生成的文件；`405板子的网表.enet` 没动。
   - CubeMX 回写 .ioc 时只多了一行 `RCC.EthernetFreq_Value`。
3. CubeMX 加载时报了 3 条 `OptionalMessage_ERROR`：`VP_RIF_VS_RIF1`；`48MHZClocksFreq_Value`（84 MHz）；`RTCHSEDivFreq_Value`（4 MHz）。用同一个 CubeMX 加载 Chassis_test.ioc 也报这三条（RTC 那条是 6 MHz）。生成的代码不受影响，因为 48 MHz 和 RTC 这两路时钟都没用。
4. App 五个文件和顶层 CMakeLists.txt 从 Chassis_test 复制过来。Chassis_test 当时没有未提交的改动，内容就是 `989ac1e` 那次提交的。CMakeLists 改了工程名和 `SHARED_DIR`；main.c 的 USER CODE 段加了 `#include "robot.h"`、`Robot_Init()`、`Robot_Loop()`，和 Chassis_test 一样。
5. 编译：`cmake --preset Debug`，再 `cmake --build --preset Debug`，输出 `build/Debug/F405_Project.elf`。

## 上板验收（建议顺序，第 1~3 步只要这块板 + 24 V）

1. 接 24 V 和 DAPLink，下载并运行。程序没停在 `Error_Handler`（HSE 起振失败会停在那里），`SystemCoreClock` 是 168000000。
2. STB：PC1、PC2 是低电平。可以在调试器里看 `GPIOC->ODR` 的 bit1、bit2，也可以用万用表量 U3、U4 的 8 脚。
3. CAN1：2026-11-09 用 M2006 测试验过了（那时程序用 CAN1，M2006 平均稳在 500 rpm）。
   - 不接电机时每 1 ms 发一帧全 0 的 0x200；用 USB-CAN 接 CN7 或 CN15 能看到（USB-CAN 会应答）。
   - 接上 M2006（C610 ID 1）后，`robot_m2006.feedback_count` 在涨，`speed_rpm` 稳在 500 附近。
   - `feedback_count` 不涨时依次查：24 V、PC1 是不是低电平（第 2 步）、CANH / CANL 有没有接反、电调 ID。
4. CAN2：现在的 M2006 测试（程序已改回 CAN2）就验这一步。电机接 CN8 或 CN9（3 CANH、4 CANL、2 GND，1 脚 5 V 别接到分电板上）；判断方法同第 3 步，`feedback_count` 不涨时把 PC1 换成 PC2 查。
5. 板间 CAN1（底盘版）：云台 C 板发 0x110，底盘板这边收到，见 Chassis_test/notes.md「上板步骤与验收」第 3 步、Gimbal_test/notes.md「板间通信」的验收。两块板的 CAN1 都是 2-pin、线序一样（1 CANL、2 CANH），两头的板载 120 Ω 正好当两个终端。
6. 接电机以后按 Chassis_test/notes.md「上板步骤与验收」做。

## 上板记录

### 2026-11-09 第一次上板：M2006 不转（原因还没最终确认）

用 OpenOCD（DAPLink，`D:\OpenOCD_Config\DAPLink\stm32f4.cfg`）连上，不复位、不停核，直接读内存和寄存器。最后只停核一下看 PC，然后恢复运行。脚本放在临时目录，没进仓库。

- **软件这边都对**：
  - 程序在跑：`uwTick` 200 ms 涨了 205；PC 在 `Robot_Loop` 里，处于 Thread 模式，没有进异常。
  - 时钟：`SystemCoreClock` = 168000000，HSE、PLL 都已就绪，PLLM 4 / PLLN 168 / PLLQ 4，APB1 4 分频。
  - 引脚：PC1、PC2 是输出低；PB8/PB9、PB12/PB13 是 AF9。
  - CAN1：BTR 0x00380002（1 Mbps），ABOM、NART 都开了；过滤器组 0 收 0x200~0x20F；FIFO0 中断已开。
  - `robot_m2006`：类型、速度模式、ID 1、目标 500、限幅 2000、PID 3/20/0 都对。
- **CAN1 一直在 bus-off 和错误被动之间来回**：
  - 恢复以后一发送就出错，有位错误（LEC 4，发隐性读到显性）和格式错误（LEC 2）。TEC 很快涨到 232，接着又进 bus-off；之后 REC 慢慢数到 128 才恢复，要几百毫秒。
  - 三个发送邮箱都是 TERR，一次 TXOK 都没有。
  - 从来没收到过一帧：接收邮箱里是上电后的随机值。`feedback_count` = 0，所以输出电流一直是 0。
- **CAN1_RX 引脚** 1.5 s 内采样 2000 次，963 次是显性。总线上有节点几乎不停地在发，应该是 C610 一直收不到应答、一直在重发。
- **CAN2**（没接东西）：用寄存器发了一帧测试帧（ID 0x123、DLC 0、不重发），发完放回初始化模式。
  - 结果是 LEC 5（发显性、读回隐性），TEC 一次到 136：错误标志的每一位也读不回来，每次加 8，一直加到错误被动。
  - 如果 U4 能驱动总线，空总线上应该只报"没有应答"（LEC 3、TEC 8）。所以 U4 也没往总线上驱动。
- **推断（待确认）**：两个收发器都发不出去，而 MCU 给的 STB 都是低。它们的共同点是 5 V（VCC）。TJA1044 手册 7.3.3 节：VCC 欠压时进待机，不管 STB；待机时低功耗接收器由 VIO 供电，RXD 还会跟着总线变，所以 CAN1_RX 能看到 C610 的帧。所以板子的 5 V 应该没到 5 V。可能是只接了 DAPLink、没接 24 V，也可能是 5 V 降压（U1 TPS54540）没工作。
- **用户确认（同日）**：XT30 上接了 24 V，TJA1044 的 5 V 也正常。所以上面"5 V 没有"的推断不成立。
- **CAN1 静默模式实验**（同日）：用寄存器把 CAN1 临时切到静默模式（BTR.SILM，只收不发、应答只在内部给自己），约 0.5 s 后恢复正常模式。
  - 结果：REC 一直是 255，LEC 一直是 2（格式错误），一帧都没收进 FIFO，`feedback_count` 不变。
  - C610 一直没人应答，应该早就是错误被动了，它的帧对接收方是完整有效的。这时还是一帧都解不出来，说明接收方向也坏了，不只是发送。
- **推断（待确认）**：发不出去，加上接收能看到电平在变（CAN1_RX 一半时间是显性）但解不出帧，正是 TJA1044 处于待机时的表现：发送器关闭，RXD 由低功耗唤醒接收器给出，跟不上 1 Mbps。5 V 正常时会进待机，就只剩芯片第 8 脚（STB）实际不是低电平这一个原因。MCU 这边 PC1、PC2 输出低、读回来也是 0，所以问题应该在 MCU 引脚到芯片之间（走线、焊接），或者在芯片本身（方向贴反、焊接不良、料不对）。两片同时这样，更像同一个设计或装配问题。
- **要量的**（板子接 24 V、程序在跑）：
  1. U3、U4 的 8 脚（STB）对地：应该接近 0 V。如果是 3.3 V 左右，就是 STB 没接上（MCU 是 LQFP64 的 9、10 脚）。
  2. CAN2 什么都不接时，CN8 的 3 脚（CANH）、4 脚（CANL）对地：正常模式应在 2.5 V 左右；待机时手册说总线被偏置到地，接近 0 V。
  3. 对照丝印，看两片 TJA1044 的 1 脚方向。
- **确认（同日）**：用户用电压表量 U3 的 STB（红表笔接 STB、黑表笔接 GND），电机马上转了。
  - 事后读状态：表笔碰着的那段时间里，CAN1 一共收到 6071 帧 0x201，按 1 kHz 算约 6 s。C610 的 ID 确实是 1。
  - 拿开表笔以后，`feedback_count` 不再增加，CAN1 又回到错误被动（TEC 88、REC 255），电机层判断掉线，输出归 0。
  - 结论：U3 第 8 脚（STB）平时没有被 PC1 拉低，所以收发器一直在待机；表笔碰上去时 STB 临时变低才恢复。可能是表笔压住了没焊好的脚，也可能是表笔同时碰到了旁边的接地焊盘。U4 在 CAN2 测试里的表现一样，应该也是同样的问题，还没单独验证。**软件不用改。**
- **用户量通断（同日）**：MCU 9 脚（PC1）到 U3 8 脚是通的。不过 HVSON8 的引脚在芯片底下，表笔只能碰到伸出芯片的焊盘，所以这只说明焊盘到 MCU 这段走线是好的，芯片引脚和焊盘之间焊没焊上，这样量不出来。
- **用 MCU 测芯片每个脚有没有焊上**（同日，OpenOCD 改 GPIO 寄存器，测完都恢复了）。原理：TJA1044 的 TXD、STB 内部有上拉到 VIO（手册 7.3.2 节，约 1~15 µA）；RXD 是推挽输出，空闲时为高。
  - STB：PC1 / PC2 先输出低，再改成浮空输入，看会不会被拉高。
  - TXD：PB9 / PB13 先当普通 GPIO 输出低，再改成浮空输入，看会不会被拉高。
  - RXD：PB8 / PB12 打开 MCU 内部下拉，看芯片能不能把它顶成高电平。

  | 芯片 | 1 脚 TXD | 4 脚 RXD | 5 脚 VIO（看上拉在不在） | 8 脚 STB |
  |---|---|---|---|---|
  | U3（CAN1） | 有上拉 ✓ | 能顶住下拉 ✓ | ✓ | 浮空 262 ms 一直是 0，没有上拉 ✗ |
  | U4（CAN2） | 浮空一直是 0 ✗ | 200 次都是 0 ✗ | ✓（STB 有上拉） | 有上拉 ✓ |

  - U3 只有 8 脚（STB）没焊上。CANH/CANL 是好的，因为它的低功耗接收器看得到 C610 的帧。这和"表笔一压 STB 就转"吻合。
  - U4 的 1、4 脚没接上，5、8 脚是好的。HVSON8 的 1~4 脚在一边、5~8 脚在另一边，所以 U4 很可能整个 1~4 脚那一边都没焊上，包括 2 脚 GND、3 脚 VCC。
  - 板子只量了 U3 的 5 V。U4 的 VCC 引脚有没有焊上，这个方法测不出来。
- **修法**：两片 TJA1044 都重新焊。用热风枪加助焊剂，底部的散热焊盘（EP，接 GND）也要焊上。U3 补 8 脚；U4 至少补 1~4 脚，最好整片重焊。光把焊盘接到 GND 没用：断的是芯片引脚和焊盘之间。
- **修完再测一遍**：上面三种测法，正常时应该是 STB、TXD 浮空读 1，RXD 在下拉下读 1。然后让 M2006 跑，连续采样 `speed_rpm` 和 `output_current` 看速度环。之前只有掉线前最后一帧的 567 rpm 一个样本，判断不了稳不稳。

### 2026-11-09 重焊以后（用户重焊了两片 TJA1044）

先确认板子里的 Flash 和 `build/Debug/F405_Project.elf`（16:19 编的）一致：读 `Robot_Loop` 开头 4 个字对比。

- **CAN1（U3）修好了，M2006 转起来了**：
  - CAN1 的 TEC、REC、LEC 都是 0；发送邮箱是 TXOK；1 s 收到 1015~1021 帧 0x201（1 kHz）。
  - 速度环（读 `robot_m2006`，没停核）：

    | 采样方式 | 时间跨度 | 转速平均 | 标准差 | 范围 | 电流平均 / 范围 |
    |---|---|---|---|---|---|
    | 连续读，约 2 ms 一次，150 次 | 0.33 s | 473 | 54 | 351~607 | 481 / 44~913 |
    | 每 50 ms 一次，60 次 | 3.7 s | 503 | 50 | 391~640 | 331 / −42~706 |

  - 平均值稳在 500 附近，但波动约 ±10%，相邻两次采样（约 2 ms）就能差 100 rpm。电流和转速反着变，是 P 项在响应。
  - 分不清是反馈噪声（转子 500 rpm 只有每秒 8.3 圈，低速时电调算的转速会抖，减速箱也有齿槽脉动）还是 Kp 3 偏大引起的小振荡。可以在调试器里直接改 `robot_m2006.speed.pid.Kp`（比如 1.5、1），看标准差变不变：Kp 降了波动也跟着降，说明是增益偏大；基本不变，就是反馈本身的噪声。
- **CAN2（U4）还没好**：
  - STB（PC2）、TXD（PB13）浮空都会被拉回 1，现在焊上了（重焊前 TXD 是 0）。
  - RXD（PB12）在空总线上自己在跳：不加上下拉 60% 高；加 MCU 下拉还是约 60% 高；加 MCU 上拉 100% 高。说明 RXD 输出高电平有力，拉低很弱。
  - CAN2 测试帧还是 LEC 5、TEC 136。
  - 推断：U4 的地没接上，即 2 脚 GND 和底部散热焊盘 EP（9 脚）。没有地，RXD 输出级的下管拉不低，发送也不会工作；STB、TXD 的上拉接在 VIO 上，所以照样好。3 脚 VCC 有没有焊上，这个方法测不出来。
  - 修法：U4 再焊一次，重点是底部的 EP（要在焊盘上加锡、用热风焊，烙铁很难焊到）和 2、3 脚。现在的 M2006 测试不用 CAN2；底盘版的电机在 CAN2 上，那之前要修好。

### 2026-11-09 U4 再焊以后

- 板子刚重新上电（运行约 37 s），Flash 和 ELF 一致。
- **两片芯片的信号脚都好了**：
  - U3：STB、TXD 浮空都会被拉回 1；RXD 不管加不加上下拉，都稳定是 1。
  - U4：同上，RXD 不再乱跳。
- **但两片都发不出去**：
  - CAN2 测试帧还是 LEC 5、TEC 136。
  - CAN1 又回到最开始的样子：TEC 192~240、LEC 5、进 bus-off，上电以来一帧反馈都没收到。
  - CAN1_RX 采样 1000 次全是 1。重焊前 C610 一直在重发，线上约一半时间是显性；现在 C610 不出声了。
- **推断**：两片同时发不出去，C610 也不出声，最像这次上电没接 24 V：板子只靠 DAPLink 的 3.3 V 在跑，收发器没有 5 V，C610 也没电。VCC（3 脚）和 GND（2 脚、EP）这几个脚用 MCU 测不出来，所以也不能排除重焊时伤到了 5 V 这一路。
- **要查**：XT30 的 24 V 接上了没有；C610 的灯亮不亮；量 CN8 的 1 脚、U3 和 U4 的 3 脚是不是 5 V。
- **用户确认（同日）**：24 V 接着，C610 的灯也亮，所以不是没上电。
  - 改为怀疑 CANH 和 CANL 短路。短路时总线建立不起压差，我们发显性读回隐性（LEC 5）；C610 发的帧在线上也看不到，它一直出错，可能就此 bus-off、不发了。热风重焊时，容易在 6、7 脚（CANL、CANH）之间连锡，或者把旁边的 U3 吹动。
  - 也可能是 5 V 被短到地，比如 2、3 脚（GND、VCC）连锡。但那样 C610 应该还在重发，接收脚能看到动静，所以可能性小一些。
  - 待量：断电时 CN7 的 1、2 脚之间、CN8 的 3、4 脚之间的电阻（正常约 120 Ω，C610 也带终端电阻时约 60 Ω；接近 0 Ω 就是短路）；通电时 U3、U4 的 3 脚和 CN8 的 1 脚的电压；放大镜看 6、7 脚和 2、3 脚有没有连锡。

### 2026-11-09 重测 CAN2：两路都好了

- 板子这段时间没有重启：运行时间从上一次测时的 75 s，一直涨到这次的 227 s。
- **CAN1**：TEC、REC、LEC 都是 0，0.5 s 收到 518 帧（1 kHz）。CAN1_RX 约 17% 的时间是显性，是我们的 0x200 和 C610 的反馈，属于正常通信。M2006 在转。
- **CAN2（U4）**：STB、TXD 浮空读 1；RXD 不管加上拉还是下拉都稳定是 1。测试帧报 **LEC 3（没有应答）**：自己发的帧完整读回来了，只是 CAN2 上没有别的节点应答，这是 U4 正常时应有的结果。TEC 停在之前的 136，因为错误被动时，按协议应答错误不再加计数。
- **上一次两路都不行的原因**：按累计帧数 126300 倒推，CAN1 是上电后约 101 s 才开始收到反馈的，那时板子没有重启、程序也没改。最可能是那时 24 V 还没接上：板子只靠 DAPLink 供电，收发器没有 5 V，C610 也没电，线上就没有动静。上面怀疑的 CANH-CANL 短路，现在看不需要。
- **结论**：两片 TJA1044 都是焊接问题，重焊后都正常了：U3 是 8 脚，U4 是 1、4 脚，以及地。软件没改过。
- 还没验的：CAN2 上真的接一个电调收发（现在只验了自己发、自己读回）。底盘版的电机在 CAN2 上，到时候一接就能看出来；想现在验，可以把 robot.c 里的 `hcan1` / `robot_can1` 临时换成 CAN2，电机接 CN8 或 CN9。

### 2026-11-09 CAN2 接分电板（分电板上接了电调）以后

- 用户把 CAN2 接到分电板上，分电板接了电调，让我测 CAN2。我没改程序，用寄存器测：临时用过滤器组 14 全收，CAN2 退出初始化模式后直接读 FIFO0，再发一帧测试帧，最后都恢复原状。
- **结果是两路都发不出去**：
  - CAN2：接收脚一直是 1，一帧都没收到；测试帧报 LEC 5，TEC 从 136 涨到 144。
  - CAN1：TEC 184~240、LEC 5，CAN1_RX 一直是 1。
- **断的时间**：最后一帧 0x201 是在上电后 318.5 s 收到的（`last_feedback_tick`）。反馈从约 101 s 到 318.5 s 一直没断，共 218197 帧；板子一直没重启。所以 318.5 s 时出了事，应该就是接分电板或者挪线的时候。
- **推断（待确认）**：
  - CAN1 断开后报的是 LEC 5（发不出去），不是 LEC 3（没人应答），说明那时 U3 没了 5 V；U4 也一样。两条总线都没动静，和没接 24 V 时一模一样。
  - 可能是接线时 24 V 断了；也可能是分电板接上 CN8 以后把 5 V 拉垮了。CN8/CN9 是 1 脚 5V、2 脚 GND、3 脚 CANH、4 脚 CANL，分电板那头线序不一样的话可能短路，甚至伤到板子。
- **要查**：先把分电板的 CAN 线从 CN8 拔下来；看 XT30 的 24 V 还在不在；量 CN8 的 1 脚是不是 5 V；对照分电板 CAN 口的线序，一致了再接。
- **用户说"接上了"以后再测**（运行到 646 s，没重启）：和上一次一样。CAN1 是 LEC 5、TEC 208~240，318.5 s 以后再也没收到 0x201；CAN2 收不到帧，测试帧也是 LEC 5。U3 的 STB、TXD、RXD 都测过是好的，STB 也被拉低，这种情况下 CAN1 什么都不接也该报 LEC 3。现在报 LEC 5，所以收发器的 5 V 仍然没有。用户还没报 5 V 的读数。
  - 待量：接着分电板时 CN8 的 1 脚对 GND 的电压。不是 5 V 就拔掉分电板再量：拔了就恢复，说明分电板线序不对，把 5 V 拉垮了；拔了还是没有，就要确认 24 V 接的是这块板自己的 XT30，再查 U1（TPS54540）。
- **用户量了（同日）**：CN8 的 1 脚不到 5 V，具体读数和是否接着分电板还没报。所以两路 CAN 都发不出去，是收发器 5 V 欠压，进了待机。
  - 注意：3.3 V 那一路有电时（比如 DAPLink 在供电），即使 U1 不工作，5 V 线也会被倒灌到两三伏，所以读数不一定是 0。
- **找到了（同日）**：用户把分电板的 CAN 线从 CN8 拔掉，5 V 马上恢复。
  - 再测（运行到 868 s，没重启）：CAN1 测试帧报 LEC 3（没人应答，C610 已经挪到分电板上了）；CAN2 测试帧也报 LEC 3。两个收发器都正常。
  - 结论：是分电板或那根线把这块板的 5 V 拉低了，收发器欠压，两路一起罢工；收发器本身没坏。
  - 最可能是线序不对：CN8/CN9 是 1 脚 5V、2 脚 GND、3 脚 CANH、4 脚 CANL，分电板那头的 1 脚可能接到了地、负载或别的网络上。
  - 接回去之前，先用万用表对清楚分电板那个口每个脚是什么。最稳妥的做法是只接 CANH、CANL、GND，不接 5 V 那根：分电板一般有自己的电源，两边的 5 V 本来就不该连在一起。
  - 下一步：
    1. 拔掉分电板的 CAN 线再量：恢复了就是分电板或那根线的问题。
    2. 量板上 XT30 的 24 V。
    3. 有 24 V、5 V 还不对，就摸 U1、L1 烫不烫；断电量 CN8 的 1 脚对 GND 的电阻（只有几欧或接近 0 就是短路）。
