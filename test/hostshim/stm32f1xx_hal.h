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

/* ---- 只需要类型名存在 (isp_config.h / spi.h 里声明了 extern 句柄) ---- */
typedef struct { int _unused; } UART_HandleTypeDef;
typedef struct { int _unused; } SPI_HandleTypeDef;
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

#define GPIO_MODE_OUTPUT_PP   0x01u
#define GPIO_NOPULL           0x00u
#define GPIO_SPEED_FREQ_LOW   0x01u
#define GPIO_SPEED_FREQ_MEDIUM 0x02u
#define GPIO_SPEED_FREQ_HIGH  0x03u

typedef struct {
    uint32_t Pin;
    uint32_t Mode;
    uint32_t Pull;
    uint32_t Speed;
} GPIO_InitTypeDef;

/* ---- HAL API 替身 (由各测试文件实现) ---- */
uint32_t HAL_GetTick(void);
void     HAL_Delay(uint32_t ms);

void HAL_GPIO_Init(GPIO_TypeDef *port, GPIO_InitTypeDef *init);
void HAL_GPIO_WritePin(GPIO_TypeDef *port, uint16_t pin, GPIO_PinState state);

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
