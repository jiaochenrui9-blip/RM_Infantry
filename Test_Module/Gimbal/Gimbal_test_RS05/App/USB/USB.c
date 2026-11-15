#include "USB.h"

#include "BMI088.h"
#include "vofa.h"

#define USB_GYRO_PERIOD_MS 20U

static uint32_t last_usb_tick;

void Usb_SendGyro(void)
{
    uint32_t now = HAL_GetTick();
    if (now - last_usb_tick < USB_GYRO_PERIOD_MS)
    {
        return;
    }
    last_usb_tick = now;

    /* 读取惯导已处理的样本，Z 轴沿 C 板坐标系，单位为度每秒。 */
    BMI088_Data_t sample;
    BMI088_GetData(&sample);
    if (sample.initialized == 0U)
    {
        return;
    }

    /* 防止 USB 中断在检查端点到提交发送之间复位设备。忙时跳过本帧。 */
    uint32_t primask = __get_PRIMASK();
    __disable_irq();
    (void)Vofa_Send(&sample.gyro_dps[2], 1U);
    __set_PRIMASK(primask);
}
