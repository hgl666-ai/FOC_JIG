/**
  ******************************************************************************
  * @file    uart_port.c
  * @brief   通用串口端口驱动实现（环形缓冲 + 中断收发）
  *
  * 并发约定（中断上下文 vs 主循环上下文）：
  *   rx_head / rx_tail —— 中断写 head、主循环读 tail；**单生产者单消费者**，
  *                        各自只改自己的索引，不需要临界区。
  *   tx_head / tx_tail —— 主循环写 head、中断读 tail；同上，单生产者单消费者。
  *   tx_len / tx_busy  —— 由"发起发送"的临界区独占写（见 uart_tx_kick），
  *                        "发送完成"中断只读 tx_len、只清 tx_busy。
  *
  * ⚠ 两个踩过的坑（docs/03 §4 的 D6，2026-10 修）：
  *   1) **段长不能倒推**。老代码在发送完成中断里用"当前的 tx_head − tx_tail"
  *      算这一段发了多长；可是主循环在发送期间还能继续往环里塞字节（head 已经
  *      往前走），倒推出来的长度会比实际发的长 → tx_tail 跳过还没发的字节
  *      → **静默丢字节**，对端只看到半截帧/CRC 错，本地计数器上一点异常都没有。
  *      现在改成：kick 把这一段的长度存进 tx_len，完成时**按 tx_len 推进 tail**。
  *   2) **kick 的判断和 HAL 调用必须在同一个临界区里**。先判 `tx_busy == 0`
  *      再关中断是不够的 —— 在这两步之间发送完成中断可以插进来，把 busy 清 0
  *      并由它自己 kick 出去一段；随后主循环那一次 HAL 调用就会撞上 HAL_BUSY，
  *      回退又把 busy/tx_len 清掉 → 与中断手里那一段彻底错位。
  *      所以：**先进临界区，再判断**（不是"先判断，再进临界区"）。
  *      临界区只有几十个周期（约 1us），115200 下一个字节要 87us，挡不住收。
  ******************************************************************************
  */
#include "uart_port.h"
#include <string.h>

/* 关中断/开中断：目标工程用 CMSIS 的 intrinsic；
 * 宿主测试的 main.h 来自 test/hostshim，那两个名字在那里是空动作
 * （宿主是单线程，没有中断上下文）。 */
#define UART_TX_CRIT_ENTER()   __disable_irq()
#define UART_TX_CRIT_EXIT()    __enable_irq()

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

/* 领一段待发数据交给 HAL（不跨缓冲区回卷，发完再领下一段）
 *
 * ⚠ 整个函数体都在临界区里跑，理由见文件头第 2 条：判断 + 置位 + 调用 HAL
 *   必须相对"发送完成中断"原子，否则会与中断手里那一段错位。
 *   tx_len 必须在调用 HAL **之前**写好 —— HAL 的回调有可能是同步的
 *   （宿主测试就是同步的；真实硬件上 TXE 中断也可能在函数返回前就被取走）。 */
static void uart_tx_kick(uart_port_t *p)
{
    uint16_t head;
    uint16_t tail;
    uint16_t run;

    UART_TX_CRIT_ENTER();

    if ((p->started == 0u) || (p->tx_busy != 0u) || (p->huart == 0)) {
        UART_TX_CRIT_EXIT();
        return;
    }
    head = p->tx_head;
    tail = p->tx_tail;
    if (head == tail) {
        UART_TX_CRIT_EXIT();
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

    p->tx_len  = run;                     /* 本段长度：完成中断按它推进 tail */
    p->tx_busy = 1u;                      /* 必须在调用之前置位（回调可能同步到达）*/
    if (HAL_UART_Transmit_IT(p->huart, &p->tx_buf[tail], run) != HAL_OK) {
        /* HAL 拒收（正常应恒不成立）。此时**没有任何一段在我手里**，
         * 两个字段一起清回"空闲"，字节留在环里等下一次 kick 重试；
         * tx_err 计数让这种"静默卡死"变成看得见的东西。 */
        p->tx_len  = 0u;
        p->tx_busy = 0u;
        p->st.tx_err++;
    }

    UART_TX_CRIT_EXIT();
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
    uint16_t     sent;

    if (!uart_id_valid(id)) {
        return;
    }
    p = &s_ports[id];
    /* 这一段发了多长 = kick 当时交给 HAL 的那一段（tx_len）。
     * ⚠ 不能读"现在的 tx_head"倒推：发送期间主循环可以继续往环里塞字节，
     *   head 已经往前走了，倒推出来的长度会把**还没发出去**的字节也算进去，
     *   于是 tx_tail 跳过它们 —— 静默丢字节（docs/03 §4 D6）。 */
    sent = p->tx_len;
    p->tx_len = 0u;
    if (sent != 0u) {
        p->tx_tail = (uint16_t)((p->tx_tail + sent) % UART_PORT_TX_BUF_SIZE);
    }
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
