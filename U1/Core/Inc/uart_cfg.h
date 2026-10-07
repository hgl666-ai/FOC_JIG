/**
  ******************************************************************************
  * @file    uart_cfg.h
  * @brief   U1 (主控) 的串口端口配置 — 把 HAL 句柄绑到 uart_port 的端口号上
  *
  * 治具一共有 4 路串口，两片各 2 路：
  *
  *   ┌─ U1 (STM32F103C8T6, 主控 / USB + SPI Flash + SWD 烧录)
  *   │    端口0 = USART1  PA9(TX) / PA10(RX)  → 排针 → 被测 FOC 板串口
  *   │           用途: **读 FOC 板上报的数据, 供治具做测试判定**
  *   │           (判定流程与"PC 怎么取这些数据"的命令码待定, 见 docs/10 §10)
  *   │    端口1 = USART3  PB10(TX)/ PB11(RX)  → H3/H4 跳线 → U2 的 USART3 (内部链路)
  *   └─ U2 (STM32F103C8T6, 测量 / 校验片)
  *        端口0 = USART1  PA9(TX) / PA10(RX)  → 排针 (对外/预留)
  *        端口1 = USART3  PB10(TX)/ PB11(RX)  → H3/H4 跳线 → U1 的 USART3 (内部链路)
  *
  * 参数：4 路统一 115200 8N1，无硬件流控（CubeMX 生成值，勿改，改了两片会不一致）。
  *
  * ⚠ 上电前确认：H3/H4 跳线必须把 U1.PB10 接 U2.PB11、U1.PB11 接 U2.PB10
  *   （TX 对 RX 交叉），接成直连两端都收不到东西。
  ******************************************************************************
  */
#ifndef __UART_CFG_H
#define __UART_CFG_H

#include <stdint.h>
#include "uart_port.h"

#define UART_CFG_PORT_DUT   (UART_PORT_0)   /* USART1: 被测板串口(读数据供判定)  */
#define UART_CFG_PORT_LINK  (UART_PORT_1)   /* USART3: 到 U2 的内部链路     */

/** 绑定句柄 + 打开两路的接收中断。必须在 MX_USARTx_UART_Init 之后调用 */
void UartCfg_Init(void);

#endif /* __UART_CFG_H */
