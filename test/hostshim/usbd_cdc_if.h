/* 宿主替身: usbd_cdc_if.h —— 只提供 app.c 用到的 USBD_BUSY / TxState / CDC_Transmit_FS */
#ifndef HOST_SHIM_USBD_CDC_IF_H
#define HOST_SHIM_USBD_CDC_IF_H

#include <stdint.h>

#define USBD_OK   0
#define USBD_BUSY 1
#define USBD_FAIL 2

typedef struct { volatile uint8_t TxState; } USBD_CDC_HandleTypeDef;

uint8_t CDC_Transmit_FS(uint8_t *Buf, uint16_t Len);

#endif /* HOST_SHIM_USBD_CDC_IF_H */
