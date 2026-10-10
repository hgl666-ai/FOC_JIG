/**
  ******************************************************************************
  * @file    bsp_uart.c
  * @brief   调试打印口实现（U1 的 USART1 / PA9）
  *
  * 开关见 jig_mode.h：BSP_DBG_ENABLE=0（默认）时这里是两个空函数，
  * swd_programmer.c / measure.c 里的打点调用不用改，也不会往线上灌字符。
  *
  * ⚠ USART1 现在是 DUT 串口（读被测板数据）。要打调试日志就得二选一：
  *   打开 BSP_DBG_ENABLE（占 USART1），或改用其它空闲串口。
  ******************************************************************************
  */
#include "bsp_uart.h"

#if (BSP_DBG_ENABLE != 0u)

#include "usart.h"          /* huart1 */
#include <string.h>

void BSP_DBG_Send(const char *str)
{
    uint16_t len;

    if (str == 0) {
        return;
    }
    len = (uint16_t)strlen(str);
    if (len == 0u) {
        return;
    }
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

#else
void BSP_DBG_Send(const char *str)
{
    (void)str;
}

void BSP_DBG_SendHexByte(uint8_t b)
{
    (void)b;
}
#endif /* BSP_DBG_ENABLE */
