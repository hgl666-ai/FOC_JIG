/**
  ******************************************************************************
  * @file    bsp_uart.h
  * @brief   串口底层: ISP 用的阻塞收发 + 波特率/校验位切换
  ******************************************************************************
  */
#ifndef __BSP_UART_H
#define __BSP_UART_H

#include <stdint.h>

/**
  * @brief  ISP 用串口 (isp_config.h 里的 ISP_UART_INSTANCE) 初始化
  *         清掉上电残留标志, 默认配置为 8E1 (STM32 系统 bootloader 要求)
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

/* 把治具自己的调试口 (USART1) 当 printf 用: 未接 PC 时静默丢弃 */
void BSP_DBG_Send(const char *str);
void BSP_DBG_SendHexByte(uint8_t b);

#endif /* __BSP_UART_H */
