# BMI088 和 Attitude 姿态解算

`BMI088.c/.h` 来自 `C:\Clion\IMU\Core\Drivers\BMI088`，默认适配 C 板片选 PA4（加速度计）、PB0（陀螺仪）。

`Shared/Algorithm/Attitude.c/.h` 与 `C:\Clion\IMU\Core\Algorithm` 对应文件 SHA-256 相同，保留原来的 Mahony 算法、名称和接口。它引用 BMI088 与 IST8310 设备类型：给 `Attitude_Update()` 提供磁力计时参与磁场修正，传入 NULL 时仅使用六轴，yaw 会漂移。

`Shared/Devices/IST8310` 来自原工程的磁力计驱动。适配头文件以允许在未启用 I2C 的工程中编译；DRDY 中断只记录事件，由 `IST8310_Update()` 在主循环启动 I2C DMA，避免在中断中启动 HAL I2C 时潜在等待。需要确保主循环持续调用 Update，清除 DMA 出错状态并处理采样超时。

## 硬件接入状态

云台工程已由 CubeMX 配置 BMI088 SPI1（PB3 SCK、PB4 MISO、PA7 MOSI）、片选、DMA 和 PC4/PC5 普通输入；尚未接入 BMI088 初始化、采样和姿态解算调用。IST8310 的 I2C3（PA8/PC9）、PG6 复位和 PG3 DRDY 尚未配置，故不能使用磁力计。

原版 `Attitude_CollectStartupAverage()` 等待 500 帧 BMI088 与 50 帧 IST8310，没有超时；当采样未运行或磁力计不可用时会一直等待。不要在当前工程直接调用该函数。只有 BMI088 时可以 `Attitude_Init()` 后用完整 IMU 样本调用 `Attitude_Update(&imu, NULL, dt_s)`；传入磁力计前需完成其外设和采样接入，统一坐标系并确认有效数据。`dt_s` 应为实际样本时间间隔。

2026-10-27：原始 Attitude 源码两文件与源工程 SHA-256 一致；目标 ARM 编译器单独编译 Attitude 和 IST8310 通过；主机姿态解算及 IST8310 延迟处理模拟通过。完整工程构建因用户云台代码的 Gimbal_Update 声明和定义参数不一致而失败；未烧录或进行实物测试。
