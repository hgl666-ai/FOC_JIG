/**
  ******************************************************************************
  * @file    fw_upload.h
  * @brief   PC→治具 固件上传三段式状态机 (CMD 0x0009 / 0x000A / 0x000B)
  ******************************************************************************
  */
#ifndef __FW_UPLOAD_H
#define __FW_UPLOAD_H

#include <stdint.h>

void    FwUpload_Init(void);

/**
  * @brief  处理上传相关命令
  * @retval 1 = 本命令已被上传模块处理 (ACK 已发出)
  *         0 = 非上传命令, 交由上层继续分发
  */
uint8_t FwUpload_Handle(uint16_t cmd, const uint8_t *data, uint16_t len);

/** @brief 上传超时检查 — 必须在主循环每轮调用 */
void    FwUpload_TimeoutCheck(void);

/** @retval 当前待写入的槽位 (0=Bootloader, 1=App) */
uint8_t FwUpload_CurrentSlot(void);

#endif /* __FW_UPLOAD_H */
