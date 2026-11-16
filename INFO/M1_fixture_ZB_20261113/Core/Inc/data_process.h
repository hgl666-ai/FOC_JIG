#ifndef DATA_PROCESS_H
#define DATA_PROCESS_H

#include <stdint.h>

/** @brief 默认模式下，是否把 UART5 收到的数据原样复制一份到 USART1 TX。
 *  由 0x0098 控制：LEN=0 翻转；LEN>=1 按 DATA[0] 设置。 */
extern uint8_t g_u5_copy_to_log;

/** @brief USART1 调试口接收处理（1ms 周期任务） */
void uart_rx_handle(void);

/** @brief PC 协议层初始化：帧解析器 + 上传状态机（main 初始化末尾调用一次） */
void pc_app_init(void);

/** @brief PC 协议层周期任务：上传超时检查（1ms 周期任务） */
void pc_app_poll(void);

#endif // DATA_PROCESS_H
