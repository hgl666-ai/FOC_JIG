#include "usart.h"
#include "usart_m.h"

#include "stm32f1xx_it.h"

#ifdef UART_TX_DMA_ENABLE
uint8_t uart1_tx_cache[UART_TX_CACHE_SIZE];
uint16_t uart1_tx_i = 0;
#endif

#ifdef UART1_RX_DMA_ENABLE
    #define UART1_RX_CACHE_LEN   128
    static uint16_t uart1_last_pos = 0;
    
    void UART_IDLECallback(UART_HandleTypeDef *huart);
#else
    #define UART1_RX_CACHE_LEN   1
#endif

uint8_t uart1_rx_cache[UART1_RX_CACHE_LEN] = {0};

uart_rx_queue_t gUart1RxQueue = {0};

/**
 * @brief 请在串口1初始化函数末尾的 USER CODE 字段中调用此函数
 */
void UART1_Init_UserCode(void)
{
#ifdef UART1_RX_DMA_ENABLE
  __HAL_UART_ENABLE_IT(&huart1, UART_IT_IDLE); // Enable idle interrupts
  HAL_UART_Receive_DMA(&huart1, uart1_rx_cache, UART1_RX_CACHE_LEN);
#else
  HAL_UART_Receive_IT(&huart1, uart1_rx_cache, UART1_RX_CACHE_LEN);
#endif
}

/**
 * @brief 请在串口中断处理函数 UART1_IRQHandler 开头的 USER CODE 字段中调用此函数
 */
void UART1_IRQHandler_UserCode(void)
{
  if ((__HAL_UART_GET_FLAG(&huart1, UART_FLAG_IDLE) != RESET) &&
      (__HAL_UART_GET_IT_SOURCE(&huart1, UART_IT_IDLE) != RESET))
  {
    UART_IDLECallback(&huart1);          /* 先按 DMA 计数取出新数据 */
    __HAL_UART_CLEAR_IDLEFLAG(&huart1);  /* 读 SR + 读 DR，清 IDLE */
  }
}

#ifdef UART1_RX_DMA_ENABLE
void UART_IDLECallback(UART_HandleTypeDef *huart)
{
  if (huart->Instance == USART1)
  {
    /*
       -  -  -  -  -  -  -  -
       0  1  2  3  4  5  6  7     index
       |              |
       8              3  2  1     __HAL_DMA_GET_COUNTER
       |              |
       0              5  6  7     next_pos_to_recv
    */
    uint16_t received_len = 0;
    uint16_t current_pos = UART1_RX_CACHE_LEN - __HAL_DMA_GET_COUNTER(huart->hdmarx);

    if (current_pos > uart1_last_pos) {
      received_len = current_pos - uart1_last_pos;
      // M_UART_Transmit(&huart1, &uart1_rx_cache[uart1_last_pos], received_len);
      uart_rx_queue_push(&gUart1RxQueue, &uart1_rx_cache[uart1_last_pos], received_len);
    } else {
      // The data crosses the buffer boundary
      received_len = (UART1_RX_CACHE_LEN - uart1_last_pos) + (current_pos - 0);
      // M_UART_Transmit(&huart1, &uart1_rx_cache[uart1_last_pos], (UART1_RX_CACHE_LEN - uart1_last_pos));
      if (current_pos > 0) {
        // M_UART_Transmit(&huart1, uart1_rx_cache, (current_pos - 0));
        uart_rx_queue_push_join(&gUart1RxQueue, &uart1_rx_cache[uart1_last_pos], (UART1_RX_CACHE_LEN - uart1_last_pos), 0);
        uart_rx_queue_push_join(&gUart1RxQueue, uart1_rx_cache, (current_pos - 0), 1);
      } else {
        uart_rx_queue_push(&gUart1RxQueue, &uart1_rx_cache[uart1_last_pos], (UART1_RX_CACHE_LEN - uart1_last_pos));
      }
    }
    uart1_last_pos = current_pos;
    
    // Reset the DMA counter (optional, the DMA_CIRCULAR mode will handle it automatically)
    // __HAL_DMA_SET_COUNTER(huart->hdmarx, UART1_RX_CACHE_LEN);
  }
}
#else
static void uart1_rx_it_complete(void)
{
  // M_UART_Transmit(&huart1, uart1_rx_cache, UART1_RX_CACHE_LEN);
  HAL_UART_Receive_IT(&huart1, uart1_rx_cache, UART1_RX_CACHE_LEN);
}
#endif

#ifdef UART_TX_DMA_ENABLE
void WAIT_UART_DMA_READY(UART_HandleTypeDef *huart) {
  uint32_t t = HAL_GetTick();
  while (HAL_UART_STATE_BUSY_TX == huart->gState) {
    /* 115200 8N1 下 256B ≈ 22.2ms；100ms 留足余量，避免还没发完就复用缓存 */
    if (HAL_GetTick() - t > 100) {
      break;
    }
  }
}

void UART_DMA_TX_U8(UART_HandleTypeDef *huart, uint8_t u8data, int flush) {
  if (uart1_tx_i < UART_TX_CACHE_SIZE) {
    uart1_tx_cache[uart1_tx_i] = u8data;
    uart1_tx_i++;
  }
  if (flush || uart1_tx_i >= UART_TX_CACHE_SIZE) {
    if (uart1_tx_i > 0) {
      /* 上一笔必须彻底发完，才能复用 uart1_tx_cache；否则 DMA 还在读缓存，
       * 新写入的字符会把正在发送的内容覆盖掉（日志乱码/两行粘连）。 */
      WAIT_UART_DMA_READY(huart);
      (void)HAL_UART_Transmit_DMA(huart, uart1_tx_cache, uart1_tx_i);
    }
    uart1_tx_i = 0;
    if (!flush) {
      /* 缓冲区满但一行还没写完：等这一段发完再继续填，保证不会覆盖 */
      WAIT_UART_DMA_READY(huart);
    }
  }
}
#endif

/*==============================================================================
 * UART5 —— DUT 系统 bootloader 链路（PD2=RX / PC12=TX）
 *----------------------------------------------------------------------------
 * F1 的 UART5 没有 DMA 请求（DMA2 表里只有 UART4），所以 RX 用接收中断
 * 逐字节收，ISR 里塞进 1KB 环形缓冲；主循环按 AN3155 协议的节奏取。
 * TX 用阻塞发送：只在烧录期间使用，和 USART1 的日志 DMA 互不影响。
 *============================================================================*/
#define UART5_RX_RING_SIZE    1024u
#define UART5_RX_RING_MASK    (UART5_RX_RING_SIZE - 1u)

static uint8_t  s_u5_rx_ring[UART5_RX_RING_SIZE];
static volatile uint16_t s_u5_rx_head = 0u;   /* 只由中断写 */
static volatile uint16_t s_u5_rx_tail = 0u;   /* 只由主循环写 */
static uint8_t  s_u5_rx_byte;
static uart5_mode_t s_u5_mode = UART5_MODE_DEFAULT;

void UART5_SetMode(uart5_mode_t mode)
{
  if (mode != s_u5_mode) {
    /* 换模式时清掉上一种模式的残留字节，避免 bootloader 读到 App 数据，
     * 或默认模式把 bootloader 的 ACK 当成 DUT 数据转发。 */
    UART5_FlushRx();
    s_u5_mode = mode;
  }
}

uart5_mode_t UART5_GetMode(void)
{
  return s_u5_mode;
}

static void uart5_rx_push(uint8_t b)
{
  uint16_t next = (uint16_t)((s_u5_rx_head + 1u) & UART5_RX_RING_MASK);

  if (next != s_u5_rx_tail) {
    s_u5_rx_ring[s_u5_rx_head] = b;
    s_u5_rx_head = next;
  }
  /* 满了就丢弃最新字节；bootloader 每包都有 ACK，丢了会超时/重发 */
}

void UART5_Init_UserCode(void)
{
  GPIO_InitTypeDef rx_init = {0};

  /* PD2(UART5_RX) 在 CubeMX 里是浮空输入；DUT 复位期间 PA9 可能浮空，
   * 内部上拉把空闲电平钉在高电平，避免噪声被当成起始位。 */
  rx_init.Pin  = GPIO_PIN_2;
  rx_init.Mode = GPIO_MODE_INPUT;
  rx_init.Pull = GPIO_PULLUP;
  HAL_GPIO_Init(GPIOD, &rx_init);

  s_u5_rx_head = 0u;
  s_u5_rx_tail = 0u;
  (void)HAL_UART_Receive_IT(&huart5, &s_u5_rx_byte, 1u);
}

void UART5_IRQHandler_UserCode(void)
{
  /* RXNE/错误由 HAL_UART_IRQHandler 处理，收完 1 字节后进
   * HAL_UART_RxCpltCallback（见文件末尾）。这里保留与 USART1
   * 相同的钩子位置，方便以后加计数器或错误统计。 */
}

void UART5_SetConfig(uint32_t baud, uint32_t parity)
{
  uint32_t word_len;

  if (huart5.Instance == 0) {
    return;
  }

  /* STM32 的“字长”包含校验位：
   *   NONE + 8B → 8 数据位
   *   EVEN + 9B → 8 数据位 + 偶校验（bootloader 要的 8E1）
   *   如果写成 EVEN + 8B，就变成 7 数据位 + 校验，最高位会丢。 */
  word_len = (parity == UART_PARITY_NONE) ? UART_WORDLENGTH_8B
                                          : UART_WORDLENGTH_9B;

  if ((huart5.Init.BaudRate == baud) &&
      (huart5.Init.Parity == parity) &&
      (huart5.Init.WordLength == word_len)) {
    return;
  }

  huart5.Init.BaudRate = baud;
  huart5.Init.WordLength = word_len;
  huart5.Init.Parity = parity;
  if (HAL_UART_Init(&huart5) == HAL_OK) {
    /* HAL_UART_Init 会清掉 RXNEIE，重新挂接收 */
    UART5_Init_UserCode();
  }
}

void UART5_SetBaud(uint32_t baud)
{
  /* DUT 用户程序是 8N1；bootloader 路径用 UART5_SetConfig(..., EVEN) */
  UART5_SetConfig(baud, UART_PARITY_NONE);
}

void UART5_FlushRx(void)
{
  uint32_t primask = __get_PRIMASK();

  __disable_irq();
  s_u5_rx_tail = s_u5_rx_head;
  if (primask == 0u) {
    __enable_irq();
  }
}

uint16_t UART5_RxCount(void)
{
  return (uint16_t)((s_u5_rx_head - s_u5_rx_tail) & UART5_RX_RING_MASK);
}

uint8_t UART5_ReadByte(uint8_t *b)
{
  if (b == 0 || s_u5_rx_tail == s_u5_rx_head) {
    return 0u;
  }
  *b = s_u5_rx_ring[s_u5_rx_tail];
  s_u5_rx_tail = (uint16_t)((s_u5_rx_tail + 1u) & UART5_RX_RING_MASK);
  return 1u;
}

uint16_t UART5_Read(uint8_t *buf, uint16_t len, uint32_t timeout_ms)
{
  uint32_t start = HAL_GetTick();
  uint16_t n = 0u;

  if (buf == 0 || len == 0u) {
    return 0u;
  }
  while (n < len) {
    if (UART5_ReadByte(&buf[n])) {
      n++;
      continue;
    }
    if ((uint32_t)(HAL_GetTick() - start) >= timeout_ms) {
      break;
    }
  }
  return n;
}

HAL_StatusTypeDef UART5_Transmit(const uint8_t *data, uint16_t len, uint32_t timeout_ms)
{
  uint32_t start = HAL_GetTick();
  uint16_t i;

  if (data == 0 || len == 0u) {
    return HAL_OK;
  }
  if (huart5.Instance == 0) {
    return HAL_ERROR;
  }

  /* 轮询 TXE/TC 直接写 DR，不用 HAL_UART_Transmit：
   * HAL 的发送会锁 gState，如果发送期间 UART5 出错，ErrorCallback 里的
   * HAL_UART_Receive_IT 会拿不到锁、接收就断了。这里不碰 HAL 状态，
   * RX 中断/错误回调可以独立工作。 */
  for (i = 0u; i < len; i++) {
    while (__HAL_UART_GET_FLAG(&huart5, UART_FLAG_TXE) == RESET) {
      if ((uint32_t)(HAL_GetTick() - start) >= timeout_ms) {
        return HAL_TIMEOUT;
      }
    }
    huart5.Instance->DR = data[i];
  }
  while (__HAL_UART_GET_FLAG(&huart5, UART_FLAG_TC) == RESET) {
    if ((uint32_t)(HAL_GetTick() - start) >= timeout_ms) {
      return HAL_TIMEOUT;
    }
  }
  return HAL_OK;
}

/*==============================================================================
 * UART 接收完成/错误回调 —— USART1（可选 IT 模式）与 UART5 共用。
 * UART1 默认走 DMA + IDLE，下面的 USART1 分支只在关掉 DMA 时编译。
 *============================================================================*/
void HAL_UART_RxCpltCallback(UART_HandleTypeDef *huart)
{
  if (huart->Instance == UART5) {
    uart5_rx_push(s_u5_rx_byte);
    (void)HAL_UART_Receive_IT(&huart5, &s_u5_rx_byte, 1u);
  }
#if !defined(UART1_RX_DMA_ENABLE)
  else if (huart->Instance == USART1) {
    uart1_rx_it_complete();
  }
#endif
}

void HAL_UART_ErrorCallback(UART_HandleTypeDef *huart)
{
  if (huart->Instance == UART5) {
    /* ORE/FE/NE 会让 HAL 结束接收；读 SR+DR 清标志后重新挂上 */
    (void)huart5.Instance->SR;
    (void)huart5.Instance->DR;
    (void)HAL_UART_Receive_IT(&huart5, &s_u5_rx_byte, 1u);
  }
}
