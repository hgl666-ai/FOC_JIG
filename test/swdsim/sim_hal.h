/*
 * 宿主侧最小 HAL 替身（SWD 线级仿真的 GPIO/RCC 定义）
 *
 * 让固件源码在 PC 上原样编译：GPIO 动作被路由到 sim_target 的线级模型。
 * 定义与 test/hostshim/stm32f1xx_hal.h 共用同一把锁
 * （app_test 同时要线级模型和宿主 shim，谁先包含谁定义）。
 */
#ifndef SIM_HAL_H
#define SIM_HAL_H

#include <stdint.h>

#ifndef SIM_GPIO_TYPES_DEFINED
#define SIM_GPIO_TYPES_DEFINED

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
#define GPIO_MODE_OUTPUT_PP    0x01u
#define GPIO_NOPULL            0x00u
#define GPIO_PULLUP            0x01u
#define GPIO_SPEED_FREQ_LOW    0x01u
#define GPIO_SPEED_FREQ_HIGH   0x03u

typedef struct {
    uint32_t Pin;
    uint32_t Mode;
    uint32_t Pull;
    uint32_t Speed;
} GPIO_InitTypeDef;

#endif /* SIM_GPIO_TYPES_DEFINED */

void          HAL_GPIO_Init(GPIO_TypeDef *port, GPIO_InitTypeDef *init);
void          HAL_GPIO_WritePin(GPIO_TypeDef *port, uint16_t pin, GPIO_PinState state);
GPIO_PinState HAL_GPIO_ReadPin(GPIO_TypeDef *port, uint16_t pin);
uint32_t      HAL_GetTick(void);
void          HAL_Delay(uint32_t ms);

#endif /* SIM_HAL_H */
