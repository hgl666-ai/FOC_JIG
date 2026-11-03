/**
  ******************************************************************************
  * @file    uart_cfg.c
  * @brief   U1 (主控) 串口端口装配: HAL 句柄 ↔ uart_port 端口号 + 中断分发
  *
  * 引脚与用途见 uart_cfg.h 顶部表格。本文件只做三件事:
  *   1) 把 huart1/huart3 绑到端口 0/1
  *   2) 打开两路的接收中断 (每字节一个中断)
  *   3) 把 HAL 的 RxCplt/TxCplt/Error 回调分发到对应端口
  *
  * 中断优先级: USB 是 0 (最高, usbd_conf.c 里设的), 串口放在 3 —— 让 USB
  * 的 CDC 收发永远先于串口, 避免串口把 USB 的响应时间顶掉。
  *
  * 特殊测试 (jig_mode.h §2.1 JIG_PC_LINK=UART): USART1 从"被测板口"改成
  * "PC 口", 绑定的端口号不变 (都是 UART_PORT_0), 所以中断分发不用改一行 ——
  * 统一用别名 UART_CFG_PORT_UART1 表示"USART1 那一路", 由 uart_cfg.h 决定它是谁。
  ******************************************************************************
  */
#include "uart_cfg.h"
#include "usart.h"
#include "jig_mode.h"

void UartCfg_Init(void)
{
#if (JIG_PC_LINK == JIG_PC_LINK_UART)
    /* 特殊测试：USART1 (PA9/PA10) 让给 PC 的 USB-TTL。
     * JIG_DUT_LINK 此时必为 0（jig_mode.h 里有 #error 保证），
     * 所以 APP_Init 里也不会去建 DUT 链路对象。 */
    UartPort_InitPort(UART_CFG_PORT_UART1, &huart1);
#else
    UartPort_InitPort(UART_CFG_PORT_DUT,  &huart1);   /* USART1 PA9/PA10 → 被测板 */
#endif
    UartPort_InitPort(UART_CFG_PORT_LINK, &huart3);   /* USART3 PB10/PB11 → U2     */

    /* 先把 TX 队列状态清零再开接收: Start 内部只会挂上第一个 RX 中断 */
    UartPort_Start(UART_CFG_PORT_UART1);
    UartPort_Start(UART_CFG_PORT_LINK);
}

/*==============================================================================
 * HAL 回调分发 (中断上下文: 只搬字节, 不做解析 —— 见 uart_port.c 的约定)
 *============================================================================*/
void HAL_UART_RxCpltCallback(UART_HandleTypeDef *huart)
{
    if (huart == &huart1) {
        UartPort_OnRxComplete(UART_CFG_PORT_UART1);
    } else if (huart == &huart3) {
        UartPort_OnRxComplete(UART_CFG_PORT_LINK);
    }
}

void HAL_UART_TxCpltCallback(UART_HandleTypeDef *huart)
{
    if (huart == &huart1) {
        UartPort_OnTxComplete(UART_CFG_PORT_UART1);
    } else if (huart == &huart3) {
        UartPort_OnTxComplete(UART_CFG_PORT_LINK);
    }
}

void HAL_UART_ErrorCallback(UART_HandleTypeDef *huart)
{
    /* 先清 ORE 再让 uart_port 重挂接收：ORE 若不清，重挂后会立刻再进一次
     * 错误中断，形成"错误中断风暴"。清 ORE 的寄存器侧动作属于 HAL 细节，
     * 所以放在这个文件里，而"重挂接收"是驱动不变量，放在 uart_port.c。 */
    __HAL_UART_CLEAR_OREFLAG(huart);

    if (huart == &huart1) {
        UartPort_OnError(UART_CFG_PORT_UART1);
    } else if (huart == &huart3) {
        UartPort_OnError(UART_CFG_PORT_LINK);
    }
}
