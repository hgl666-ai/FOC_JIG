/**
  ******************************************************************************
  * @file    uart_cfg.c
  * @brief   U2 (测量/校验片) 串口端口装配: HAL 句柄 ↔ uart_port 端口号 + 中断分发
  *
  * 中断优先级: 串口放 3。U2 上没有 USB, 这个值只是与 U1 保持一致, 便于两片
  * 用同一份约定描述 (见 docs/10_UART四路驱动设计.md)。
  ******************************************************************************
  */
#include "uart_cfg.h"
#include "usart.h"

void UartCfg_Init(void)
{
    UartPort_InitPort(UART_CFG_PORT_EXT,  &huart1);   /* USART1 PA9/PA10 → 排针 */
    UartPort_InitPort(UART_CFG_PORT_LINK, &huart3);   /* USART3 PB10/PB11 → U1  */

    UartPort_Start(UART_CFG_PORT_EXT);
    UartPort_Start(UART_CFG_PORT_LINK);
}

/*==============================================================================
 * HAL 回调分发 (中断上下文: 只搬字节, 不做解析)
 *============================================================================*/
void HAL_UART_RxCpltCallback(UART_HandleTypeDef *huart)
{
    if (huart == &huart1) {
        UartPort_OnRxComplete(UART_CFG_PORT_EXT);
    } else if (huart == &huart3) {
        UartPort_OnRxComplete(UART_CFG_PORT_LINK);
    }
}

void HAL_UART_TxCpltCallback(UART_HandleTypeDef *huart)
{
    if (huart == &huart1) {
        UartPort_OnTxComplete(UART_CFG_PORT_EXT);
    } else if (huart == &huart3) {
        UartPort_OnTxComplete(UART_CFG_PORT_LINK);
    }
}

void HAL_UART_ErrorCallback(UART_HandleTypeDef *huart)
{
    /* 先清 ORE 再重挂接收, 否则会进错误中断风暴 (详见 uart_port.c 的 OnError) */
    __HAL_UART_CLEAR_OREFLAG(huart);

    if (huart == &huart1) {
        UartPort_OnError(UART_CFG_PORT_EXT);
    } else if (huart == &huart3) {
        UartPort_OnError(UART_CFG_PORT_LINK);
    }
}
