/**
  ******************************************************************************
  * @file    uart_port.c
  * @brief   通用串口端口驱动实现（环形缓冲 + 中断收发）
  *
  * 并发约定（中断上下文 vs 主循环上下文）：
  *   rx_head / rx_tail —— 中断写 head、主循环读 tail；**单生产者单消费者**，
  *                        各自只改自己的索引，不需要临界区。
  *   tx_head / tx_tail —— 主循环写 head、中断读 tail；同上，单生产者单消费者。
  *   tx_busy / started —— 只在"发起发送"处（主循环或发送完成中断）改动；
  *                        由于 HAL 的 TxCplt 回调可能同步返回（宿主测试就是同步的），
  *                        这里用"先置 busy 再调用 HAL"的顺序保证不会被重入打乱。
  ******************************************************************************
  */
#include "uart_port.h"
#include <string.h>

static uart_port_t s_ports[UART_PORT_COUNT];

static uint8_t uart_id_valid(uart_id_t id)
{
    return ((uint8_t)id < (uint8_t)UART_PORT_COUNT) ? 1u : 0u;
}

uart_port_t *UartPort_Get(uart_id_t id)
{
    return uart_id_valid(id) ? &s_ports[id] : 0;
}

void UartPort_InitPort(uart_id_t id, UART_HandleTypeDef *huart)
{
    uart_port_t *p;

    if (!uart_id_valid(id)) {
        return;
    }
    p = &s_ports[id];
    memset(p, 0, sizeof(*p));
    p->huart = huart;
}

/* 领一段待发数据交给 HAL（不跨缓冲区回卷，发完再领下一段） */
static void uart_tx_kick(uart_port_t *p)
{
    uint16_t head;
    uint16_t tail;
    uint16_t run;
    uint16_t space;

    if ((p->started == 0u) || (p->tx_busy != 0u) || (p->huart == 0)) {
        return;
    }
    head = p->tx_head;
    tail = p->tx_tail;
    if (head == tail) {
        return;                          /* 没数据 */
    }
    if (head > tail) {
        run = (uint16_t)(head - tail);
    } else {
        run = (uint16_t)((UART_PORT_TX_BUF_SIZE - tail) + head);
    }
    if ((uint16_t)(tail + run) > UART_PORT_TX_BUF_SIZE) {
        run = (uint16_t)(UART_PORT_TX_BUF_SIZE - tail);   /* 先发到末尾 */
    }
    space = run;                          /* 本段长度 */

    p->tx_busy = 1u;                      /* 必须在调用之前置位（回调可能同步到达）*/
    if (HAL_UART_Transmit_IT(p->huart, &p->tx_buf[tail], space) != HAL_OK) {
        p->tx_busy = 0u;
    }
}

void UartPort_Start(uart_id_t id)
{
    uart_port_t *p;

    if (!uart_id_valid(id)) {
        return;
    }
    p = &s_ports[id];
    if (p->huart == 0) {
        return;
    }
    p->started = 1u;
    /* 开接收：每次收 1 字节，收满后在 UartPort_OnRxComplete 里重新挂上 */
    (void)HAL_UART_Receive_IT(p->huart, &p->rx_ch, 1u);
}

uint16_t UartPort_Write(uart_id_t id, const uint8_t *buf, uint16_t len)
{
    uart_port_t *p;
    uint16_t     n = 0u;
    uint16_t     next;

    if (!uart_id_valid(id) || (buf == 0)) {
        return 0u;
    }
    p = &s_ports[id];
    while (n < len) {
        next = (uint16_t)((p->tx_head + 1u) % UART_PORT_TX_BUF_SIZE);
        if (next == p->tx_tail) {
            break;                                        /* 队列满：丢新字节 */
        }
        p->tx_buf[p->tx_head] = buf[n];
        p->tx_head = next;
        n++;
    }
    if (n < len) {
        p->st.tx_drop += (uint32_t)(len - n);
    }
    p->st.tx_bytes += n;
    uart_tx_kick(p);
    return n;
}

uint16_t UartPort_Read(uart_id_t id, uint8_t *buf, uint16_t max)
{
    uart_port_t *p;
    uint16_t     n = 0u;

    if (!uart_id_valid(id) || (buf == 0)) {
        return 0u;
    }
    p = &s_ports[id];
    while ((n < max) && (p->rx_tail != p->rx_head)) {
        buf[n] = p->rx_buf[p->rx_tail];
        p->rx_tail = (uint16_t)((p->rx_tail + 1u) % UART_PORT_RX_BUF_SIZE);
        n++;
    }
    return n;
}

uint16_t UartPort_Available(uart_id_t id)
{
    uart_port_t *p;

    if (!uart_id_valid(id)) {
        return 0u;
    }
    p = &s_ports[id];
    if (p->rx_head >= p->rx_tail) {
        return (uint16_t)(p->rx_head - p->rx_tail);
    }
    return (uint16_t)((UART_PORT_RX_BUF_SIZE - p->rx_tail) + p->rx_head);
}

void UartPort_FlushRx(uart_id_t id)
{
    if (!uart_id_valid(id)) {
        return;
    }
    s_ports[id].rx_tail = s_ports[id].rx_head;
}

const uart_stats_t *UartPort_Stats(uart_id_t id)
{
    if (!uart_id_valid(id)) {
        return 0;
    }
    return &s_ports[id].st;
}

void UartPort_OnRxComplete(uart_id_t id)
{
    uart_port_t *p;
    uint16_t     next;

    if (!uart_id_valid(id)) {
        return;
    }
    p = &s_ports[id];
    p->st.rx_bytes++;
    next = (uint16_t)((p->rx_head + 1u) % UART_PORT_RX_BUF_SIZE);
    if (next == p->rx_tail) {
        p->st.rx_drop++;                 /* 缓冲满：丢新字节，主循环取走后自动恢复 */
    } else {
        p->rx_buf[p->rx_head] = p->rx_ch;
        if (next < p->rx_head) {
            p->st.rx_wrap++;             /* 回卷次数：诊断用 */
        }
        p->rx_head = next;
    }
    /* 重新挂上下一字节的接收 */
    if (p->huart != 0) {
        (void)HAL_UART_Receive_IT(p->huart, &p->rx_ch, 1u);
    }
}

void UartPort_OnTxComplete(uart_id_t id)
{
    uart_port_t *p;
    uint16_t     tail;
    uint16_t     head;
    uint16_t     sent;

    if (!uart_id_valid(id)) {
        return;
    }
    p = &s_ports[id];
    /* 这一段是从 tx_tail 开始发的，长度 = 到 head 或到缓冲区末尾（与 kick 一致） */
    head = p->tx_head;
    tail = p->tx_tail;
    if (head >= tail) {
        sent = (uint16_t)(head - tail);
    } else {
        sent = (uint16_t)(UART_PORT_TX_BUF_SIZE - tail);
    }
    p->tx_tail = (uint16_t)((tail + sent) % UART_PORT_TX_BUF_SIZE);
    p->tx_busy = 0u;
    uart_tx_kick(p);                     /* 还有数据就继续发 */
}

void UartPort_OnError(uart_id_t id)
{
    uart_port_t *p;

    if (!uart_id_valid(id)) {
        return;
    }
    p = &s_ports[id];
    p->st.rx_err++;

    /* ⚠ 必须重新挂接收：
     * F1 HAL 收到 ORE(溢出) 时走的是"阻塞错误"分支 —— 它先调
     * UART_EndRxTransfer() 关掉 RXNEIE/PEIE/EIE 并把 RxState 置 READY，
     * 再调 HAL_UART_ErrorCallback()。如果这里不重新挂上，这一路从此再也收不到
     * 任何字节（而且不会有任何报错），是"串口突然哑了"最典型的成因。
     * 非 ORE 的软错误(帧错/噪声)下 HAL 仍在收，重挂会返回 HAL_BUSY，无害。 */
    if ((p->started != 0u) && (p->huart != 0)) {
        (void)HAL_UART_Receive_IT(p->huart, &p->rx_ch, 1u);
    }
}
