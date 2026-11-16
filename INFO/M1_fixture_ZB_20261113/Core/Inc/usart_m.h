#ifndef __USART_M_H
#define __USART_M_H

#include <stdint.h>
#include <string.h>
#include "usart.h"

#define UART1_ENABLE   1
#define UART2_ENABLE   0

#define UART1_RX_DMA_ENABLE
#define UART_TX_DMA_ENABLE

/* ----------------------- UART Init UserCode -----------------------*/

/**
 * @brief 请在串口1初始化函数末尾的 USER CODE 字段中调用此函数
 */
void UART1_Init_UserCode(void);

/* ----------------------- UART IRQ UserCode -----------------------*/

/**
 * @brief 请在串口中断处理函数 UART1_IRQHandler 开头的 USER CODE 字段中调用此函数
 */
void UART1_IRQHandler_UserCode(void);

/* ----------------------- UART TX Methods -----------------------*/

#ifdef UART_TX_DMA_ENABLE
  void WAIT_UART_DMA_READY(UART_HandleTypeDef *huart);
  #define WAIT_UART1_DMA_READY()  WAIT_UART_DMA_READY(&huart1)

  /* UART TX DMA: tx bytes */
  #define UART_TX_CACHE_SIZE   256
  void UART_DMA_TX_U8(UART_HandleTypeDef *huart, uint8_t u8data, int flush);

  static inline void UART1_DMA_TX_U8(uint8_t u8data, int flush) {
    UART_DMA_TX_U8(&huart1, u8data, flush);
  }
  
  /* UART TX DMA: tx one frame */
  #define M_UART_Transmit(huart, ptr, len)  HAL_UART_Transmit_DMA(huart, ptr, len)

  static inline void M_UART_Transmit_AndWait(UART_HandleTypeDef *huart, const uint8_t *ptr, uint16_t len) {
    WAIT_UART_DMA_READY(huart);
    HAL_UART_Transmit_DMA(huart, ptr, len);
  }

  /* UART1 TX DMA: tx one frame */
  #define M_UART1_Transmit(ptr, len)  HAL_UART_Transmit_DMA(&huart1, ptr, len)

  static inline void M_UART1_Transmit_AndWait(const uint8_t *ptr, uint16_t len) {
    WAIT_UART_DMA_READY(&huart1);
    HAL_UART_Transmit_DMA(&huart1, ptr, len);
  }
#else
  #define M_UART_Transmit(huart, ptr, len)  HAL_UART_Transmit(huart, ptr, len, 50)
#endif


/* ----------------------- UART5: DUT bootloader link -----------------------*/
/* UART5 接收数据的归属模式：
 *   DEFAULT：默认模式。收到的字节由上层默认处理（0x0099 透传 CDC、
 *            0x0098 复制到 USART1，或直接丢弃）。
 *   FLASH  ：烧录模式。收到的字节由 UART_FLASH.c 的 bootloader 协议消费，
 *            默认处理绝对不能动这些字节。
 * 上电默认 DEFAULT；UARTF_Start/UARTF_Probe 进 FLASH，结束后回 DEFAULT。 */
typedef enum {
    UART5_MODE_DEFAULT = 0,
    UART5_MODE_FLASH   = 1
} uart5_mode_t;

void         UART5_SetMode(uart5_mode_t mode);
uart5_mode_t UART5_GetMode(void);
/*
 * UART5(PD2=RX / PC12=TX) 专用作 DUT USART1 的系统 bootloader 链路。
 * 与 USART1 的日志口完全独立：
 *   - RX 用 HAL 接收中断逐字节进 1KB 环形缓冲（UART5 在 F1 上没有 DMA）；
 *   - TX 用阻塞发送（只在烧录期间使用，不会和 USART1 日志抢 DMA）。
 * 中断钩子放在 stm32f1xx_it.c 的 UART5_IRQHandler USER CODE 段里，
 * 实现在 usart_m.c，符合"UART5 的中断/收发都放 usart_m"的约定。
 */
void     UART5_Init_UserCode(void);
void     UART5_IRQHandler_UserCode(void);

void     UART5_SetBaud(uint32_t baud);
/* 同时设置波特率和校验位：
 *   UART_PARITY_NONE = 8N1（DUT 用户程序）
 *   UART_PARITY_EVEN = 8E1（STM32F105 系统 bootloader 要求） */
void     UART5_SetConfig(uint32_t baud, uint32_t parity);
void     UART5_FlushRx(void);
uint16_t UART5_RxCount(void);
uint8_t  UART5_ReadByte(uint8_t *b);
uint16_t UART5_Read(uint8_t *buf, uint16_t len, uint32_t timeout_ms);
HAL_StatusTypeDef UART5_Transmit(const uint8_t *data, uint16_t len, uint32_t timeout_ms);

/* ----------------------- UART RX Queue -----------------------*/

#define UART_RX_QUEUE_LEN 8
#define UART_RX_QUEUE_BUF_SIZE 64

typedef struct {
  uint8_t pData[UART_RX_QUEUE_LEN][UART_RX_QUEUE_BUF_SIZE];
  uint8_t pDataLen[UART_RX_QUEUE_LEN];
  uint16_t head;
  uint16_t tail;
  uint16_t used;
} uart_rx_queue_t;

extern uart_rx_queue_t gUart1RxQueue;

static inline void uart_rx_queue_push(uart_rx_queue_t *q, uint8_t *data, uint16_t len) {
  if (q->used < UART_RX_QUEUE_LEN && len > 0) {
    if (len > UART_RX_QUEUE_BUF_SIZE) {
      len = UART_RX_QUEUE_BUF_SIZE;
    }
    memcpy(q->pData[q->tail], data, len);
    q->pDataLen[q->tail] = len;
    q->tail = (q->tail + 1) % UART_RX_QUEUE_LEN;
    q->used++;
  }
}

static inline void uart_rx_queue_push_join(uart_rx_queue_t *q, uint8_t *data, uint16_t len, uint8_t end) {
  if (q->used < UART_RX_QUEUE_LEN && len > 0) {
    if (q->pDataLen[q->tail]) {
      if (len > UART_RX_QUEUE_BUF_SIZE - q->pDataLen[q->tail]) {
        len = UART_RX_QUEUE_BUF_SIZE - q->pDataLen[q->tail];
      }
    } else if (len > UART_RX_QUEUE_BUF_SIZE) {
      len = UART_RX_QUEUE_BUF_SIZE;
    }
    memcpy(&(q->pData[q->tail][q->pDataLen[q->tail]]), data, len);
    q->pDataLen[q->tail] += len;
    if (end) {
      q->tail = (q->tail + 1) % UART_RX_QUEUE_LEN;
      q->used++;
    }
  }
}

static inline void uart_rx_queue_pop(uart_rx_queue_t *q, uint8_t *data, uint16_t *len) {
  if (q->used > 0) {
    memcpy(data, q->pData[q->head], q->pDataLen[q->head]);
    *len = q->pDataLen[q->head];
    q->pDataLen[q->head] = 0;
    q->head = (q->head + 1) % UART_RX_QUEUE_LEN;
    q->used--;
  } else {
    *len = 0;
  }
}

static inline void uart_rx_queue_out(uart_rx_queue_t *q) {
  if (q->used > 0) {
    q->pDataLen[q->head] = 0;
    q->head = (q->head + 1) % UART_RX_QUEUE_LEN;
    q->used--;
  }
}

static inline uint8_t uart_rx_queue_get_head(uart_rx_queue_t *q, uint8_t **p) {
  if (q->used > 0) {
    *p = q->pData[q->head];
    return q->pDataLen[q->head];
  }
  return 0;
}

/*
usage:

void uart_rx_handle_example(void) {
  uint8_t *buf;
  uint8_t len = uart_rx_queue_get_head(&gUart1RxQueue, &buf);
  if (len > 0) {
    data_handle(buf, len);
    uart_rx_queue_out(&gUart1RxQueue);
  }
}
*/

#endif // __USART_M_H
