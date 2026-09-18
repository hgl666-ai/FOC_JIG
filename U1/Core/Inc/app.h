/**
  ******************************************************************************
  * @file    app.h
  * @brief   治具主控 U1 应用层总入口
  ******************************************************************************
  */
#ifndef __APP_H
#define __APP_H

#include "main.h"
#include <stdint.h>

/*===== 顶层系统状态 =====*/
typedef enum {
    APP_STATE_INIT = 0,
    APP_STATE_IDLE,
    APP_STATE_UPLOADING,     /* 固件上传中 (只接受 0x000A / 0x000B) */
    APP_STATE_PROGRAMMING    /* ISP 烧录中   (只接受 0x0112, 回 BUSY) */
} APP_State_t;

extern APP_State_t app_state;
extern uint16_t    pc_seq;      /* 最近一帧的帧序号, ACK 回显用 */

/*===== API =====*/
void APP_Init(void);
void APP_Run(void);
void APP_USB_Receive(uint8_t *data, uint32_t len);   /* 由 CDC_Receive_FS 回调 */

/*===== 帧发送 =====*/
void APP_SendAck(uint16_t cmd, const uint8_t *data, uint16_t len);   /* SEQ = pc_seq */
void APP_SendEvent(uint16_t cmd, const uint8_t *data, uint16_t len); /* SEQ = 0      */

/*===== SPI Flash 槽头部操作 =====*/
uint8_t Flash_ReadHead(uint8_t slot, uint32_t *p_size, uint8_t ver[3], uint8_t *p_flag);
void    Flash_WriteCompleteFlag(uint8_t slot, uint8_t flag);

#endif /* __APP_H */
