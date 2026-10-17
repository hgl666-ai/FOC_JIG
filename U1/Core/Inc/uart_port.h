/**
  ******************************************************************************
  * @file    uart_port.h
  * @brief   通用串口端口驱动（U1 / U2 **各一份逐字节相同的副本**，改一份必须同步另一份）
  *
  * 设计要点：
  *  - **一路一个实例**：每路自带收发环形缓冲 + 统计计数，互不干扰。
  *  - **全程非阻塞**：收用"每字节中断"，发用"TXE 中断链式发"；
  *    主循环只搬数据、不做解析（和 USB 侧 app_frame 的做法一致）。
  *  - **缓冲满时丢新字节并计数**（不是丢最旧）：半截帧比丢整帧更难查，
  *    丢字节计数暴露在 UartPort_Stats() 里，排障时一眼看出是谁丢的。
  *  - 本文件不认识"这一路接什么"，端口名与用途由各板的 uart_cfg.c 决定。
  *
  * 发送侧的核心不变式（`tx_len` 是为此加的，见 docs/03 §4 D6）：
  *     `tx_busy != 0`  ⟺  `tx_len != 0`  ⟺  HAL 手里正有一段在发
  *     `tx_tail` 只在"发送完成"时前进，且恰好前进 **kick 当时交给 HAL 的那一段长度**
  *   ——**不允许**用"现在的 tx_head − tx_tail"倒推段长：主循环可以在这一段
  *   在飞的时候继续往环里塞新字节，head 早就往前走了，倒推会把还没发出去的
  *   字节一起算成"已发"，于是 tx_tail 跳过它们 → 静默丢字节（对端只看到半截帧）。
  *
  * 中断入口：HAL 的 RxCplt/TxCplt/Error 回调由各板的 uart_cfg.c 分发进来
  * （见 UartPort_OnRxComplete 等），so 本文件不需要知道 Instance 的映射。
  ******************************************************************************
  */
#ifndef __UART_PORT_H
#define __UART_PORT_H

#include <stdint.h>
#include "main.h"        /* 宿主测试下由 -I test/hostshim 提供 HAL 替身 */

typedef enum {
    UART_PORT_0 = 0,                 /* 每片的 USART1：U1=被测板口 / U2=对外口 */
    UART_PORT_1,                     /* 每片的 USART3：U1<->U2 链路 */
    UART_PORT_COUNT
} uart_id_t;

#define UART_PORT_RX_BUF_SIZE   256u
#define UART_PORT_TX_BUF_SIZE   256u

/* 统计（诊断用；全部是单调计数，不清零） */
typedef struct {
    uint32_t rx_bytes;      /* 收到的字节总数 */
    uint32_t tx_bytes;      /* 发出去的字节总数 */
    uint32_t rx_drop;       /* 接收环满 → 丢弃的字节数 */
    uint32_t tx_drop;       /* 发送队列满 → 丢弃的字节数 */
    uint32_t rx_err;        /* HAL 报的帧错/噪声/溢出次数 */
    uint32_t rx_wrap;       /* 接收环回卷次数（诊断"是不是一直在塞"） */
    uint32_t tx_err;        /* HAL 拒绝发起发送的次数（正常应恒为 0；非 0 = 驱动/外设状态不同步） */
} uart_stats_t;

typedef struct {
    UART_HandleTypeDef *huart;
    uint8_t             rx_buf[UART_PORT_RX_BUF_SIZE];
    volatile uint16_t   rx_head;        /* 中断/收端写 */
    volatile uint16_t   rx_tail;        /* 主循环读 */
    uint8_t             rx_ch;          /* HAL 单字节接收的落点 */
    uint8_t             tx_buf[UART_PORT_TX_BUF_SIZE];
    volatile uint16_t   tx_head;        /* 主循环写 */
    volatile uint16_t   tx_tail;        /* 中断/发端读 */
    volatile uint16_t   tx_len;         /* 正在飞的那一段的长度（0 = 没有在发） */
    volatile uint8_t    tx_busy;
    volatile uint8_t    started;
    uart_stats_t        st;
} uart_port_t;

/** 把一个 HAL 句柄挂到某个端口（只做静态绑定，不开中断） */
void      UartPort_InitPort(uart_id_t id, UART_HandleTypeDef *huart);

/** 打开这一路的接收中断（上电初始化最后调用） */
void      UartPort_Start(uart_id_t id);

/** 把数据放进发送队列。返回实际入队的字节数（队列满则小于 len） */
uint16_t  UartPort_Write(uart_id_t id, const uint8_t *buf, uint16_t len);

/** 从接收缓冲取数据。返回实际取到的字节数 */
uint16_t  UartPort_Read(uart_id_t id, uint8_t *buf, uint16_t max);

/** 接收缓冲里还有多少字节可读 */
uint16_t  UartPort_Available(uart_id_t id);

/** 丢弃接收缓冲里尚未取走的字节（重新同步/换波特率时用） */
void      UartPort_FlushRx(uart_id_t id);

/** 统计计数（只读） */
const uart_stats_t *UartPort_Stats(uart_id_t id);

/** 取端口对象（给中断分发与测试用） */
uart_port_t *UartPort_Get(uart_id_t id);

/* ---- 中断回调（由各板 uart_cfg.c 从 HAL 回调里分发进来） ---- */
void      UartPort_OnRxComplete(uart_id_t id);
void      UartPort_OnTxComplete(uart_id_t id);
void      UartPort_OnError(uart_id_t id);

#endif /* __UART_PORT_H */
