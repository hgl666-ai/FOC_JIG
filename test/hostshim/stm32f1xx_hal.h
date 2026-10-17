/*
 * 宿主侧最小 HAL 替身 —— 仅供 test 目录下的测试编译固件真源码使用。
 *
 * 为什么替身的是 stm32f1xx_hal.h 而不是 main.h:
 *   固件 Core/Inc/main.h 第一行就 #include "stm32f1xx_hal.h";
 *   而 #include "..." 先搜「包含它的文件所在目录」, Core/Inc/main.h 必然被选中,
 *   所以只能从它依赖的 HAL 头下手。本目录在宿主测试里排在 -I 列表最前,
 *   目标工程 (Keil) 完全不含本目录, 因此不影响固件构建。
 */
#ifndef HOST_SHIM_STM32F1XX_HAL_H
#define HOST_SHIM_STM32F1XX_HAL_H

#include <stdint.h>

typedef enum { HAL_OK = 0, HAL_ERROR, HAL_BUSY, HAL_TIMEOUT } HAL_StatusTypeDef;

/* ---- GPIO 类型与常量 ----
 * 与 test/swdsim/sim_hal.h 共用同一套定义（app_test 同时要用 SWD 线级模型）。
 * 谁先被包含谁定义，另一个跳过，避免枚举/结构体重复定义。 */
#ifndef SIM_GPIO_TYPES_DEFINED
#define SIM_GPIO_TYPES_DEFINED

/* ---- 只需要类型名存在 (各模块头里声明了 extern 句柄) ---- */
typedef struct { uint32_t _id; } USART_TypeDef;
typedef struct { int _unused; } SPI_HandleTypeDef;

#define USART1 ((USART_TypeDef *)0x40013800u)
#define USART2 ((USART_TypeDef *)0x40004400u)
#define USART3 ((USART_TypeDef *)0x40004800u)

/* uart_port 驱动需要 Instance 来区分端口，所以这里给出真实字段 */
typedef struct { USART_TypeDef *Instance; } UART_HandleTypeDef;
typedef struct { int _unused; } GPIO_TypeDef;

#define GPIOA ((GPIO_TypeDef *)0x40010800u)
#define GPIOB ((GPIO_TypeDef *)0x40010C00u)
#define GPIOC ((GPIO_TypeDef *)0x40011000u)

typedef enum { GPIO_PIN_RESET = 0, GPIO_PIN_SET } GPIO_PinState;

#define GPIO_PIN_0   ((uint16_t)0x0001)
#define GPIO_PIN_1   ((uint16_t)0x0002)
#define GPIO_PIN_2   ((uint16_t)0x0004)
#define GPIO_PIN_3   ((uint16_t)0x0008)
#define GPIO_PIN_4   ((uint16_t)0x0010)
#define GPIO_PIN_5   ((uint16_t)0x0020)
#define GPIO_PIN_6   ((uint16_t)0x0040)
#define GPIO_PIN_7   ((uint16_t)0x0080)
#define GPIO_PIN_8   ((uint16_t)0x0100)
#define GPIO_PIN_9   ((uint16_t)0x0200)
#define GPIO_PIN_10  ((uint16_t)0x0400)
#define GPIO_PIN_11  ((uint16_t)0x0800)
#define GPIO_PIN_12  ((uint16_t)0x1000)
#define GPIO_PIN_13  ((uint16_t)0x2000)
#define GPIO_PIN_14  ((uint16_t)0x4000)
#define GPIO_PIN_15  ((uint16_t)0x8000)

#define GPIO_MODE_INPUT        0x00u
#define GPIO_MODE_OUTPUT_PP   0x01u
#define GPIO_NOPULL           0x00u
#define GPIO_PULLUP           0x01u
#define GPIO_SPEED_FREQ_LOW   0x01u
#define GPIO_SPEED_FREQ_MEDIUM 0x02u
#define GPIO_SPEED_FREQ_HIGH  0x03u

typedef struct {
    uint32_t Pin;
    uint32_t Mode;
    uint32_t Pull;
    uint32_t Speed;
} GPIO_InitTypeDef;

#endif /* SIM_GPIO_TYPES_DEFINED */

/* ---- ADC 通道号 (measure.h 用它做通道映射, 宿主侧只需值存在) ---- */
#define ADC_CHANNEL_0   0u
#define ADC_CHANNEL_1   1u
#define ADC_CHANNEL_2   2u
#define ADC_CHANNEL_3   3u
#define ADC_CHANNEL_4   4u
#define ADC_CHANNEL_5   5u
#define ADC_CHANNEL_6   6u
#define ADC_CHANNEL_7   7u
#define ADC_CHANNEL_8   8u
#define ADC_CHANNEL_9   9u
#define ADC_CHANNEL_VREFINT 17u

/* ---- HAL API 替身 (由各测试文件实现) ---- */
uint32_t HAL_GetTick(void);
void     HAL_Delay(uint32_t ms);

void HAL_GPIO_Init(GPIO_TypeDef *port, GPIO_InitTypeDef *init);
void HAL_GPIO_WritePin(GPIO_TypeDef *port, uint16_t pin, GPIO_PinState state);
GPIO_PinState HAL_GPIO_ReadPin(GPIO_TypeDef *port, uint16_t pin);

/* ---- 临界区替身 ----
 * 固件里 uart_port.c 的发送路径用 __disable_irq()/__enable_irq() 做几拍的
 * 临界区（挡"发送完成中断"插进来）。宿主测试是**单线程顺序执行**的：
 * HAL 替身在 HAL_UART_Transmit_IT 里同步回调 TxCplt，也就是说"中断"
 * 是被显式调用出来的，不会有真正并发的中断上下文 → 这里退化成空动作。
 * 目标工程用的是 CMSIS 的真身（cmsis_armcc.h / cmsis_gcc.h），不经过本文件。 */
#ifndef __disable_irq
#define __disable_irq()  do { } while (0)
#endif
#ifndef __enable_irq
#define __enable_irq()   do { } while (0)
#endif

/* ---- UART 替身 (uart_port 驱动 / uart_cfg 需要) ---- */
HAL_StatusTypeDef HAL_UART_Receive_IT(UART_HandleTypeDef *huart, uint8_t *pData, uint16_t Size);
HAL_StatusTypeDef HAL_UART_Transmit_IT(UART_HandleTypeDef *huart, uint8_t *pData, uint16_t Size);
void HAL_UART_RxCpltCallback(UART_HandleTypeDef *huart);
void HAL_UART_TxCpltCallback(UART_HandleTypeDef *huart);
void HAL_UART_ErrorCallback(UART_HandleTypeDef *huart);

/* 真实 HAL 里这是"读 SR 再读 DR"来清溢出标志; 宿主侧没有寄存器, 空动作即可
 * (uart_cfg.c 的错误回调会调它, 所以要能编译过) */
#define __HAL_UART_CLEAR_OREFLAG(__HANDLE__)  ((void)(__HANDLE__))

HAL_StatusTypeDef HAL_SPI_TransmitReceive(SPI_HandleTypeDef *hspi,
                                          uint8_t *pTxData, uint8_t *pRxData,
                                          uint16_t Size, uint32_t Timeout);

/* ---- RCC 替身 ----
 * 目标工程里 __HAL_RCC_GPIOx_CLK_ENABLE() 由 HAL 提供, 最终写 RCC->APB2ENR。
 * 宿主侧用一块真实内存代替寄存器, 让测试可以断言"某端口的时钟确实被使能了"
 * —— 这能抓到"配置了引脚却忘了开时钟"这类 STM32 上典型的静默失效。 */
typedef struct { volatile uint32_t APB2ENR; } RCC_TypeDef;
extern RCC_TypeDef g_host_rcc;

#define RCC_HOST_IOPAEN  (1u << 2)
#define RCC_HOST_IOPBEN  (1u << 3)
#define RCC_HOST_IOPCEN  (1u << 4)

#define __HAL_RCC_GPIOA_CLK_ENABLE()  (g_host_rcc.APB2ENR |= RCC_HOST_IOPAEN)
#define __HAL_RCC_GPIOB_CLK_ENABLE()  (g_host_rcc.APB2ENR |= RCC_HOST_IOPBEN)
#define __HAL_RCC_GPIOC_CLK_ENABLE()  (g_host_rcc.APB2ENR |= RCC_HOST_IOPCEN)

#endif /* HOST_SHIM_STM32F1XX_HAL_H */
