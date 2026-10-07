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
#include "uart_link.h"      /* link_status_t (链路状态访问接口用) */

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

/*===== 到 U2 的内部链路 (串口) =====*/
/** U2 是否还活着 (最近一次探测有应答)。诊断/自检用, 不影响协议流程 */
uint8_t APP_U2Alive(void);
/** 探测计数: [out] ok=成功次数, fail=超时次数 (任一可传 0) */
void    APP_U2ProbeStats(uint16_t *ok, uint16_t *fail);
/**
  * @brief  两片的链路代码版本是否一致 (1 = 一致)
  * @note   两片各有一份 uart_link.c/uart_port.c 副本。若改了副本却只重烧一片,
  *         链路字段布局可能对不上 → 这里会变成 0。还没收到过 STATUS 时返回 1。
  */
uint8_t APP_U2VerOk(void);
/** 最近一次 U2 上报的状态 (③ 测量交叉校验会用): 1 = 有数据, 0 = 还没收到过 */
uint8_t APP_U2Status(link_status_t *out);

#endif /* __APP_H */
