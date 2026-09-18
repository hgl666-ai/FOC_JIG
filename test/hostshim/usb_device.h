/* 宿主替身: usb_device.h —— 只提供 app.c 用到的 hUsbDeviceFS.pClassData */
#ifndef HOST_SHIM_USB_DEVICE_H
#define HOST_SHIM_USB_DEVICE_H

#include "usbd_cdc_if.h"

typedef struct { void *pClassData; } USBD_HandleTypeDef;

extern USBD_HandleTypeDef hUsbDeviceFS;

void MX_USB_DEVICE_Init(void);

#endif /* HOST_SHIM_USB_DEVICE_H */
