#ifndef VOFA_H
#define VOFA_H

#include "main.h"

/*
 * 调试波形：按 VOFA+ 的 JustFloat 格式，经 USB 虚拟串口（CubeMX 生成的 USB_DEVICE CDC）发给电脑。
 * 一帧 = count 个小端 float + 帧尾 00 00 80 7F。VOFA+ 里数据引擎选 JustFloat，端口选 C 板的虚拟串口，
 * 波特率随便填（USB 虚拟串口不按波特率传）。
 * 要求工程用 CubeMX 开了 USB_OTG_FS（Device_Only）+ USB_DEVICE 的 CDC，且已调过 MX_USB_DEVICE_Init()。
 * 帧缓冲只有一份：只在主循环一个地方调，不要在中断里调。
 */

#define VOFA_MAX_CH  16U

/* 不等待：USB 没连上（电脑还没配置好设备），或上一帧还没被电脑取走（VOFA+ 没打开串口时一直这样），
   就丢掉这一帧，返回 HAL_BUSY。count 为 0 或大于 VOFA_MAX_CH 返回 HAL_ERROR。 */
HAL_StatusTypeDef Vofa_Send(const float *ch, uint8_t count);

#endif /* VOFA_H */
