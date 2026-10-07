/**
  ******************************************************************************
  * @file    uart_cfg.h
  * @brief   U2 (测量/校验片) 的串口端口配置
  *
  *   U2 端口0 = USART1  PA9(TX) / PA10(RX)  → 排针 (对外, 预留)
  *   U2 端口1 = USART3  PB10(TX)/ PB11(RX)  → H3/H4 跳线 → U1 的 USART3 (内部链路)
  *
  * 参数与 U1 完全一致: 115200 8N1 无流控。
  * 详细引脚表见 docs/10_UART四路驱动设计.md。
  ******************************************************************************
  */
#ifndef __UART_CFG_H
#define __UART_CFG_H

#include <stdint.h>
#include "uart_port.h"

#define UART_CFG_PORT_EXT   (UART_PORT_0)   /* USART1: 对外排针(预留)        */
#define UART_CFG_PORT_LINK  (UART_PORT_1)   /* USART3: 到 U1 的内部链路      */

/** 绑定句柄 + 打开两路的接收中断。必须在 MX_USARTx_UART_Init 之后调用 */
void UartCfg_Init(void);

#endif /* __UART_CFG_H */
