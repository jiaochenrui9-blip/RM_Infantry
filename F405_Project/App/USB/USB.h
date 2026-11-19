#ifndef APP_USB_H
#define APP_USB_H

/* 在主循环调用，每 20 ms 尝试发送一次板子 Z 轴角速度。 */
void Usb_SendGyro(void);

#endif
