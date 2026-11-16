/**
  ******************************************************************************
  * @file    cdc_transport.c
  * @brief   USB CDC 接收帧缓存 + 发送队列实现。
  ******************************************************************************
  */
#include "cdc_transport.h"
#include "main.h"
#include "usbd_cdc_if.h"

#include <string.h>

#if (CDC_RX_FRAME_MAX < CDC_RX_PACKET_SIZE)
#error "CDC_RX_FRAME_MAX must be >= CDC_RX_PACKET_SIZE"
#endif

/* ============================== 内部类型 ============================== */
typedef struct
{
  uint16_t len;
  uint8_t  data[CDC_RX_PACKET_SIZE];
} CDC_RxPacketTypeDef;

typedef struct
{
  uint16_t len;
  uint8_t  data[CDC_TX_FRAME_MAX];
} CDC_TxFrameTypeDef;

#define CDC_RX_PKT_MASK   (CDC_RX_PACKET_QUEUE_DEPTH - 1U)
#define CDC_TX_Q_MASK     (CDC_TX_QUEUE_DEPTH - 1U)

/* ============================== 内部变量 ============================== */
/* 接收包队列：ISR 写 head，主循环写 tail。 */
static CDC_RxPacketTypeDef s_rx_pkt_q[CDC_RX_PACKET_QUEUE_DEPTH];
static volatile uint16_t   s_rx_pkt_head;
static volatile uint16_t   s_rx_pkt_tail;
static volatile uint32_t   s_rx_overflow;

/* 接收帧组装缓冲，仅主循环访问。 */
static uint8_t  s_rx_frame[CDC_RX_FRAME_MAX];
static uint16_t s_rx_frame_len;

/* 发送帧队列，仅主循环访问。 */
static CDC_TxFrameTypeDef s_tx_q[CDC_TX_QUEUE_DEPTH];
static uint16_t s_tx_head;
static uint16_t s_tx_tail;
static uint8_t  s_tx_inflight;
static volatile uint32_t s_tx_drop;

/* USB 复位请求：ISR 只置位，主循环负责清理。 */
static volatile uint8_t s_reset_req;

/* ============================== 内部函数 ============================== */
static uint16_t cdc_rx_next(uint16_t idx)
{
  return (uint16_t)((idx + 1U) & CDC_RX_PKT_MASK);
}

static uint16_t cdc_tx_next(uint16_t idx)
{
  return (uint16_t)((idx + 1U) & CDC_TX_Q_MASK);
}

/* 默认帧边界：USB 短包（含 0 长度 ZLP）表示一帧结束。
 * 后续如果 PC 协议改成“长度头/分隔符”，只改这个函数即可。 */
static uint8_t cdc_rx_frame_complete(const CDC_RxPacketTypeDef *pkt)
{
  return (pkt->len < CDC_RX_PACKET_SIZE) ? 1U : 0U;
}

static void cdc_rx_finalize(void)
{
  if (s_rx_frame_len > 0U)
  {
    CDC_FrameHandler(s_rx_frame, s_rx_frame_len);
  }
  s_rx_frame_len = 0U;
}

/* 主上下文：处理 USB 复位请求，丢弃旧数据。 */
static void cdc_handle_reset(void)
{
  if (s_reset_req == 0U)
  {
    return;
  }
  s_reset_req = 0U;

  /* RX 的 head 归 ISR 所有，主循环只把 tail 追到 head 来清空队列。 */
  s_rx_pkt_tail = s_rx_pkt_head;
  s_rx_frame_len = 0U;

  /* TX 全是主上下文变量，可直接清空。 */
  s_tx_head = 0U;
  s_tx_tail = 0U;
  s_tx_inflight = 0U;
}

/* 主上下文：从包队列拼帧。每轮最多处理一个队列深度的包，避免占死主循环。 */
static void cdc_rx_poll(void)
{
  uint16_t processed = 0U;

  while ((s_rx_pkt_tail != s_rx_pkt_head) &&
         (processed < CDC_RX_PACKET_QUEUE_DEPTH))
  {
    const CDC_RxPacketTypeDef *pkt;
    uint16_t i;

    /* 保证先看到 ISR 写入的负载，再读包内容。 */
    __DMB();
    pkt = &s_rx_pkt_q[s_rx_pkt_tail];

    if (((uint32_t)s_rx_frame_len + (uint32_t)pkt->len) > CDC_RX_FRAME_MAX)
    {
      cdc_rx_finalize();
    }

    for (i = 0U; i < pkt->len; i++)
    {
      s_rx_frame[s_rx_frame_len] = pkt->data[i];
      s_rx_frame_len++;
    }

    if (cdc_rx_frame_complete(pkt) != 0U)
    {
      cdc_rx_finalize();
    }

    s_rx_pkt_tail = cdc_rx_next(s_rx_pkt_tail);
    processed++;
  }
}

/* 主上下文：发送泵。 */
static void cdc_tx_pump(void)
{
  /* 上一帧尚未传完：F1 的后续包仍从该槽位取数据，不能释放。 */
  if (s_tx_inflight != 0U)
  {
    if (CDC_IsTxBusy() == 0U)
    {
      s_tx_inflight = 0U;
      s_tx_tail = cdc_tx_next(s_tx_tail);
    }
    else
    {
      return;
    }
  }

  while ((s_tx_head != s_tx_tail) && (CDC_IsReady() != 0U))
  {
    CDC_TxFrameTypeDef *frame = &s_tx_q[s_tx_tail];
    uint8_t result = CDC_Transmit_FS(frame->data, frame->len);

    if (result == USBD_OK)
    {
      s_tx_inflight = 1U;
      return;
    }
    else if (result == USBD_BUSY)
    {
      /* 等一下轮轮询。 */
      return;
    }
    else
    {
      /* 底层错误：丢帧，避免队列卡死。 */
      s_tx_drop++;
      s_tx_tail = cdc_tx_next(s_tx_tail);
    }
  }
}

/* ============================== 对外实现 ============================== */
void CDC_Transport_Init(void)
{
  uint32_t primask = __get_PRIMASK();

  __disable_irq();
  s_rx_pkt_head = 0U;
  s_rx_pkt_tail = 0U;
  s_rx_overflow = 0U;
  s_rx_frame_len = 0U;
  s_tx_head = 0U;
  s_tx_tail = 0U;
  s_tx_inflight = 0U;
  s_tx_drop = 0U;
  s_reset_req = 0U;
  __set_PRIMASK(primask);
}

void CDC_Transport_Poll(void)
{
  cdc_handle_reset();
  cdc_rx_poll();
  cdc_tx_pump();
}

void CDC_Transport_RxPush(const uint8_t *data, uint32_t len)
{
  uint16_t idx;
  uint16_t next;
  uint32_t i;

  if ((data == NULL) && (len != 0U))
  {
    s_rx_overflow++;
    return;
  }

  if (len > CDC_RX_PACKET_SIZE)
  {
    /* USB FS CDC 不应超过 64 字节；兜底截断。 */
    len = CDC_RX_PACKET_SIZE;
  }

  idx = s_rx_pkt_head;
  next = cdc_rx_next(idx);
  if (next == s_rx_pkt_tail)
  {
    /* 队列满：丢包并计数。 */
    s_rx_overflow++;
    return;
  }

  s_rx_pkt_q[idx].len = (uint16_t)len;
  for (i = 0U; i < len; i++)
  {
    s_rx_pkt_q[idx].data[i] = data[i];
  }

  /* 先保证负载写完，再发布新的 head。 */
  __DMB();
  s_rx_pkt_head = next;
}

CDC_TxResult CDC_Transport_Send(const uint8_t *data, uint16_t len, bool immediate)
{
  uint16_t next;
  uint32_t start_tick;

  if ((data == NULL) || (len == 0U) || (len > CDC_TX_FRAME_MAX))
  {
    return CDC_TX_TOO_LARGE;
  }

  /* immediate 会阻塞，绝不能在中断里调用。 */
  if (__get_IPSR() != 0U)
  {
    return CDC_TX_CONTEXT_ERR;
  }

  cdc_handle_reset();

  if (immediate == false)
  {
    next = cdc_tx_next(s_tx_head);
    if (next == s_tx_tail)
    {
      return CDC_TX_FULL;
    }
    s_tx_q[s_tx_head].len = len;
    (void)memcpy(s_tx_q[s_tx_head].data, data, len);
    s_tx_head = next;
    return CDC_TX_OK;
  }

  start_tick = HAL_GetTick();

  /* 先等一个空槽（期间只泵发送，不处理接收，避免帧处理函数递归）。 */
  for (;;)
  {
    next = cdc_tx_next(s_tx_head);
    if (next != s_tx_tail)
    {
      break;
    }

    cdc_tx_pump();

    if ((CDC_TX_BLOCK_TIMEOUT_MS != 0U) &&
        ((HAL_GetTick() - start_tick) >= CDC_TX_BLOCK_TIMEOUT_MS))
    {
      return (CDC_IsReady() == 0U) ? CDC_TX_NOT_READY : CDC_TX_TIMEOUT;
    }
  }

  s_tx_q[s_tx_head].len = len;
  (void)memcpy(s_tx_q[s_tx_head].data, data, len);
  s_tx_head = next;

  /* 阻塞到队列里的所有帧都发完（含最后一帧的 USB 完成中断）。 */
  while ((s_tx_head != s_tx_tail) || (s_tx_inflight != 0U))
  {
    cdc_tx_pump();

    if ((CDC_TX_BLOCK_TIMEOUT_MS != 0U) &&
        ((HAL_GetTick() - start_tick) >= CDC_TX_BLOCK_TIMEOUT_MS))
    {
      return (CDC_IsReady() == 0U) ? CDC_TX_NOT_READY : CDC_TX_TIMEOUT;
    }
  }

  return CDC_TX_OK;
}

void CDC_Transport_Reset(void)
{
  s_reset_req = 1U;
}

uint32_t CDC_Transport_RxOverflowCount(void)
{
  return s_rx_overflow;
}

uint32_t CDC_Transport_TxDropCount(void)
{
  return s_tx_drop;
}

/* 默认空实现；应用层定义同名函数即可覆盖。 */
__WEAK void CDC_FrameHandler(const uint8_t *data, uint16_t len)
{
  (void)data;
  (void)len;
}
