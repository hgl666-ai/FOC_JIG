/**
  ******************************************************************************
  * @file    cdc_transport.h
  * @brief   USB CDC 接收帧缓存 + 发送队列。
  *
  * 设计要点：
  *  - 接收在 USB 中断里只做“拷贝进包队列”，主循环轮询拼帧；
  *  - 发送由主循环按帧出队，immediate=true 时阻塞到队列清空；
  *  - 接收与发送使用各自独立的队列，互不阻塞。
  ******************************************************************************
  */
#ifndef __CDC_TRANSPORT_H__
#define __CDC_TRANSPORT_H__

#ifdef __cplusplus
extern "C" {
#endif

#include <stdint.h>
#include <stdbool.h>
#include <stddef.h>

/* ============================== 容量配置 ============================== */
/* USB FS CDC 的 OUT 包最大 64 字节，这里不要改小。 */
#ifndef CDC_RX_PACKET_SIZE
#define CDC_RX_PACKET_SIZE            64U
#endif

/* 接收包队列深度，必须是 2 的幂。
 * 1KB 帧 = 16 个 64B 数据包 + 1 个 ZLP；取 32 深，
 * 保证主循环写 Flash 期间整帧数据仍能进队列。 */
#ifndef CDC_RX_PACKET_QUEUE_DEPTH
#define CDC_RX_PACKET_QUEUE_DEPTH     32U
#endif

/* 单帧最大长度（拼帧缓冲）：1024 字节数据 + 16 字节帧头/帧尾/CRC。 */
#ifndef CDC_RX_FRAME_MAX
#define CDC_RX_FRAME_MAX              1040U
#endif

/* 单帧最大发送长度。 */
#ifndef CDC_TX_FRAME_MAX
#define CDC_TX_FRAME_MAX              256U
#endif

/* 发送帧队列深度，必须是 2 的幂。 */
#ifndef CDC_TX_QUEUE_DEPTH
#define CDC_TX_QUEUE_DEPTH            8U
#endif

/* immediate=true 时最长阻塞时间，单位 ms；设为 0 表示无限等待。 */
#ifndef CDC_TX_BLOCK_TIMEOUT_MS
#define CDC_TX_BLOCK_TIMEOUT_MS       1000U
#endif

#if (CDC_RX_PACKET_SIZE < 64U)
#error "CDC_RX_PACKET_SIZE must be at least 64 for USB FS CDC"
#endif

#if ((CDC_RX_PACKET_QUEUE_DEPTH & (CDC_RX_PACKET_QUEUE_DEPTH - 1U)) != 0U)
#error "CDC_RX_PACKET_QUEUE_DEPTH must be a power of two"
#endif

#if (CDC_RX_PACKET_QUEUE_DEPTH < 2U)
#error "CDC_RX_PACKET_QUEUE_DEPTH must be at least 2"
#endif

#if ((CDC_TX_QUEUE_DEPTH & (CDC_TX_QUEUE_DEPTH - 1U)) != 0U)
#error "CDC_TX_QUEUE_DEPTH must be a power of two"
#endif

#if (CDC_TX_QUEUE_DEPTH < 2U)
#error "CDC_TX_QUEUE_DEPTH must be at least 2"
#endif

/* ============================== 返回码 ============================== */
typedef enum
{
  CDC_TX_OK = 0,
  CDC_TX_FULL,          /* 非阻塞发送时队列已满 */
  CDC_TX_TOO_LARGE,     /* 长度为 0 或超过 CDC_TX_FRAME_MAX */
  CDC_TX_NOT_READY,     /* 等待超时且 USB 未配置（未枚举/已拔出） */
  CDC_TX_TIMEOUT,       /* 等待超时 */
  CDC_TX_CONTEXT_ERR,   /* 在中断上下文调用（immediate 不能在 ISR 里阻塞） */
  CDC_TX_USB_ERR        /* USB 底层返回错误 */
} CDC_TxResult;

/* ============================== 对外接口 ============================== */
void CDC_Transport_Init(void);

/* 主循环轮询：先处理接收拼帧，再泵发送队列。 */
void CDC_Transport_Poll(void);

/* 供 USB 中断的 CDC_Receive_FS 调用，只入包队列，不做拼帧。 */
void CDC_Transport_RxPush(const uint8_t *data, uint32_t len);

/*
 * 发送一帧：
 *  - immediate == false：拷贝入队后立即返回；队列满返回 CDC_TX_FULL。
 *  - immediate == true ：入队后阻塞，直到队列内所有数据发送完成；
 *                         只能在线程/主循环上下文调用。
 */
CDC_TxResult CDC_Transport_Send(const uint8_t *data, uint16_t len, bool immediate);

/* 由 CDC_DeInit_FS（USB 总线复位/断开）调用，只置复位请求标志。 */
void CDC_Transport_Reset(void);

/* 统计信息，便于调试。 */
uint32_t CDC_Transport_RxOverflowCount(void);
uint32_t CDC_Transport_TxDropCount(void);

/*
 * 接收帧处理函数，默认空实现（__WEAK）。
 * 在 main 上下文中由 CDC_Transport_Poll() 调用。
 * data 指向内部帧缓冲，只在回调期间有效；需要保存请自行拷贝。
 * 当前默认帧边界为“USB 短包/ZLP”；后续有协议头/分隔符时，
 * 修改 cdc_transport.c 里的 cdc_rx_frame_complete() 即可。
 */
void CDC_FrameHandler(const uint8_t *data, uint16_t len);

#ifdef __cplusplus
}
#endif

#endif /* __CDC_TRANSPORT_H__ */
