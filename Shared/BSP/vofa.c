#include "vofa.h"

#include <string.h>
#include "usbd_cdc_if.h"    /* CubeMX 生成：CDC_Transmit_FS；也带进 USBD_CDC_HandleTypeDef */

extern USBD_HandleTypeDef hUsbDeviceFS;     /* CubeMX 生成，定义在 usb_device.c */

/* JustFloat 帧尾，就是小端的 +Inf */
static const uint8_t vofa_tail[4] = {0x00U, 0x00U, 0x80U, 0x7FU};
/* USB 发送时直接从这里读，发完之前不能改 */
static uint8_t vofa_frame[VOFA_MAX_CH * 4U + sizeof(vofa_tail)];

HAL_StatusTypeDef Vofa_Send(const float *ch, uint8_t count)
{
    /* 电脑配置好设备之前、USB 复位之后，这个指针是空的 */
    const USBD_CDC_HandleTypeDef *cdc = (const USBD_CDC_HandleTypeDef *)hUsbDeviceFS.pClassData;
    uint16_t len = (uint16_t)(count * 4U);

    if ((ch == NULL) || (count == 0U) || (count > VOFA_MAX_CH))
    {
        return HAL_ERROR;
    }

    /* 先查再拷：上一帧还在发（TxState 为 1）时帧缓冲不能动，这一帧直接丢掉。 */
    if ((cdc == NULL) || (hUsbDeviceFS.dev_state != USBD_STATE_CONFIGURED) || (cdc->TxState != 0U))
    {
        return HAL_BUSY;
    }

    memcpy(vofa_frame, ch, len);            /* Cortex-M4 是小端，float 原样拷就是 JustFloat 要的字节序 */
    memcpy(&vofa_frame[len], vofa_tail, sizeof(vofa_tail));
    len = (uint16_t)(len + sizeof(vofa_tail));

    return (CDC_Transmit_FS(vofa_frame, len) == USBD_OK) ? HAL_OK : HAL_BUSY;
}
