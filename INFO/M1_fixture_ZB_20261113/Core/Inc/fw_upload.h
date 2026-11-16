/**
  ******************************************************************************
  * @file    fw_upload.h
  * @brief   PC→治具 固件上传三段式状态机 (CMD 0x0009 / 0x000A / 0x000B)
  *
  * 实现放在 data_process.c（受"不改 Keil 工程文件"约束，不新增 .c）。
  * 接口与 FOC_JIG/U1 的 fw_upload.h 保持一致，方便后续继续移植其它命令。
  ******************************************************************************
  */
#ifndef __FW_UPLOAD_H
#define __FW_UPLOAD_H

#include <stdint.h>

/** @brief 复位上传会话（上电时调用一次） */
void    FwUpload_Init(void);

/**
  * @brief  处理上传相关命令
  * @retval 1 = 本命令已被上传模块处理 (ACK 已发出)
  *         0 = 非上传命令, 交由上层继续分发
  */
uint8_t FwUpload_Handle(uint16_t cmd, const uint8_t *data, uint16_t len);

/** @brief 上传超时检查 — 必须在主循环周期任务里调用 (建议 1ms) */
void    FwUpload_TimeoutCheck(void);

/** @retval 当前待写入的槽位 (0=Bootloader, 1=App) */
uint8_t FwUpload_CurrentSlot(void);

/** @retval 1 = 正在上传, 其它命令应回 BUSY */
uint8_t FwUpload_IsBusy(void);

#endif /* __FW_UPLOAD_H */
