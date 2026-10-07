/*
 * HAL 替身实现：把 GPIO 动作转成仿真目标模型的事件
 */
#include "sim_hal.h"
#include "sim_target.h"

static uint32_t s_tick = 0u;

void HAL_GPIO_WritePin(GPIO_TypeDef *port, uint16_t pin, GPIO_PinState state)
{
    sim_gpio_write(port, pin, (state == GPIO_PIN_SET) ? 1u : 0u);
}

GPIO_PinState HAL_GPIO_ReadPin(GPIO_TypeDef *port, uint16_t pin)
{
    return sim_gpio_read(port, pin) ? GPIO_PIN_SET : GPIO_PIN_RESET;
}

/* Mode == INPUT 视为"释放总线"（对应目标构建里换 CRL 方向位） */
void HAL_GPIO_Init(GPIO_TypeDef *port, GPIO_InitTypeDef *init)
{
    sim_gpio_dir(port, (uint16_t)init->Pin,
                 (init->Mode == GPIO_MODE_INPUT) ? 0u : 1u);
}

/* 每次读取前进 1ms：让 F4_FLASH_TIMEOUT_MS 这类超时逻辑在仿真里也能被触发 */
uint32_t HAL_GetTick(void)
{
    return s_tick++;
}

void HAL_Delay(uint32_t ms)
{
    s_tick += ms;
}
