#include "bsp_uart.h"
#include "isp_config.h"
#include "usart.h"
#include "main.h"
#include <string.h>

/*==============================================================================
 * ISP 串口 (默认 USART3)
 *============================================================================*/
void BSP_ISP_UART_Init(void)
{
    BSP_ISP_UART_FlushRx();
    /* 上电默认按**运行态 8N1** 配置: 此时还没有任何 ISP 会话,
     * 若默认 8E1, 后续任何走这个口的应用层(8N1)收发都会失败。
     * ISP_Start() 会在进入 bootloader 会话时切到 8E1, 结束后切回 8N1。 */
    BSP_ISP_UART_SetParity(0u);
}

void BSP_ISP_UART_SetParity(uint8_t even_parity)
{
    uint32_t parity = even_parity ? UART_PARITY_EVEN : UART_PARITY_NONE;

    if (huart3.Init.Parity == parity) return;

    (void)HAL_UART_DeInit(&huart3);
    huart3.Init.BaudRate     = 115200u;
    huart3.Init.WordLength   = UART_WORDLENGTH_8B;
    huart3.Init.StopBits     = UART_STOPBITS_1;
    huart3.Init.Parity       = parity;
    huart3.Init.Mode         = UART_MODE_TX_RX;
    huart3.Init.HwFlowCtl    = UART_HWCONTROL_NONE;
    huart3.Init.OverSampling = UART_OVERSAMPLING_16;
    (void)HAL_UART_Init(&huart3);
    BSP_ISP_UART_FlushRx();
}

void BSP_ISP_UART_SendByte(uint8_t b)
{
    (void)HAL_UART_Transmit(&huart3, &b, 1, 100);
}

void BSP_ISP_UART_Send(const uint8_t *data, uint16_t len)
{
    if (len == 0u) return;
    (void)HAL_UART_Transmit(&huart3, (uint8_t *)data, len, 1000);
}

uint8_t BSP_ISP_UART_RecvByte(uint8_t *out, uint32_t timeout_ms)
{
    return (HAL_UART_Receive(&huart3, out, 1, timeout_ms) == HAL_OK) ? 1u : 0u;
}

void BSP_ISP_UART_FlushRx(void)
{
    __HAL_UART_CLEAR_FLAG(&huart3, UART_FLAG_ORE | UART_FLAG_FE | UART_FLAG_NE | UART_FLAG_PE);
    (void)huart3.Instance->DR;      /* 读 DR 清 RXNE */
    (void)huart3.Instance->SR;
}

/*==============================================================================
 * 调试口 (USART1, PA9/PA10) — 仅用于 bring-up 打点, 不参与协议
 *============================================================================*/
void BSP_DBG_Send(const char *str)
{
    if (str == 0) return;
    uint16_t len = (uint16_t)strlen(str);
    if (len == 0u) return;
    (void)HAL_UART_Transmit(&huart1, (uint8_t *)str, len, 100);
}

void BSP_DBG_SendHexByte(uint8_t b)
{
    static const char hexd[] = "0123456789ABCDEF";
    char s[3];
    s[0] = hexd[(b >> 4) & 0x0Fu];
    s[1] = hexd[b & 0x0Fu];
    s[2] = '\0';
    BSP_DBG_Send(s);
}
