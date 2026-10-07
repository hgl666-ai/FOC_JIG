/**
  ******************************************************************************
  * @file    bsp_uart.h
  * @brief   串口底层: ISP 用的阻塞收发(已停用) + 调试打印开关
  *
  * ⚠ 2026-?? 引脚分配变更（见 docs/10_UART四路驱动设计.md）：
  *   U1 USART1(PA9/PA10) = 被测 FOC 板串口 (DUT 口)
  *   U1 USART3(PB10/PB11) = 到 U2 的内部链路
  *   而本文件原来的 ISP 方案用的是 **USART3 → 被测板**，与现在的链路冲突；
  *   调试打印原来走 USART1，现在会污染 DUT 串口。
  *   所以两者都改成默认关闭的编译开关：
  *     BSP_ISP_UART_ENABLE = 1  → 编进 ISP 阻塞收发（需同时把 isp_programmer.c 加回工程）
  *     BSP_DBG_ENABLE      = 1  → 调试打印真正输出到 USART1（只在台架 bring-up 时开）
  ******************************************************************************
  */
#ifndef __BSP_UART_H
#define __BSP_UART_H

#include <stdint.h>

/*==============================================================================
 * 编译开关 (可在 Keil 的 C/C++ → Define 里覆盖，或在这儿改)
 *============================================================================*/
#ifndef BSP_DBG_ENABLE
#define BSP_DBG_ENABLE      0u      /* 默认关: USART1 已改作 DUT 串口 */
#endif

#ifndef BSP_ISP_UART_ENABLE
#define BSP_ISP_UART_ENABLE 0u      /* 默认关: ISP 方案已被 SWD 取代，且引脚与链路冲突 */
#endif

/*==============================================================================
 * ISP 阻塞收发 (历史方案, 默认不编进固件)
 *----------------------------------------------------------------------------
 * ⚠ 声明一直保留(宿主测试 isp_test 会编译 isp_programmer.c, 它要用这些声明),
 *   但**实现**在 bsp_uart.c 里被 BSP_ISP_UART_ENABLE 关掉了。
 *   真要把 ISP 加回固件, 必须同时:
 *     1) 打开 BSP_ISP_UART_ENABLE
 *     2) 接受它占用 USART3 —— 而 USART3 现在是到 U2 的内部链路, 二者冲突
 *        (要么把链路换到别的 USART, 要么放弃 ISP)。
 *============================================================================*/
/**
  * @brief  ISP 用串口 (isp_config.h 里的 ISP_UART_INSTANCE) 初始化
  *         清掉上电残留标志, 默认配置为 8N1 (运行时), 进 ISP 会话前切 8E1
  */
void BSP_ISP_UART_Init(void);

/**
  * @brief  重新配置 ISP 串口帧格式
  * @param  even_parity 1 = 8E1 (bootloader), 0 = 8N1 (应用运行态)
  */
void BSP_ISP_UART_SetParity(uint8_t even_parity);

/* 阻塞发送 */
void BSP_ISP_UART_Send(const uint8_t *data, uint16_t len);
void BSP_ISP_UART_SendByte(uint8_t b);

/**
  * @brief  阻塞收 1 字节
  * @retval 1 = 收到, 0 = 超时
  */
uint8_t BSP_ISP_UART_RecvByte(uint8_t *out, uint32_t timeout_ms);

/* 清空接收侧的 ORE/FE/NE 标志与 DR, 避免上一命令的残留污染下一命令 */
void BSP_ISP_UART_FlushRx(void);

/* 调试打印: BSP_DBG_ENABLE=0 时是空函数 (调用点不用改) */
void BSP_DBG_Send(const char *str);
void BSP_DBG_SendHexByte(uint8_t b);

#endif /* __BSP_UART_H */
