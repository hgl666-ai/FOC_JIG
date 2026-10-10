/**
  ******************************************************************************
  * @file    bsp_uart.h
  * @brief   调试打印口（U1 的 USART1 / PA9）—— 仅供台架 bring-up 打点
  *
  * 历史：本文件原来还带一套 **UART ISP 烧录**用的阻塞收发（走 USART3）。
  * 2026-10 已整体删除：烧录改走 SWD 两线（见 docs/09），而 USART3 现在是
  * U1↔U2 的内部链路（见 docs/10），两者引脚冲突 —— ISP 那套已无立锥之地。
  *
  * 开关：BSP_DBG_ENABLE 由 jig_mode.h 统一管理（调试模式且有调试串口才开）。
  *   ⚠ USART1 同时是 DUT 串口（读被测板数据用），打开打印就等于占用它。
  ******************************************************************************
  */
#ifndef __BSP_UART_H
#define __BSP_UART_H

#include <stdint.h>
#include "jig_mode.h"

/* 调试打印: BSP_DBG_ENABLE=0 时是空函数 (调用点不用改) */
void BSP_DBG_Send(const char *str);
void BSP_DBG_SendHexByte(uint8_t b);

#endif /* __BSP_UART_H */
