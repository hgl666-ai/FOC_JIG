/*
 * 寄存器级 TIM / NVIC / CMSIS 替身 —— **只给 test/pulse_cfg_test.c 用**
 *
 * 为什么需要它：
 *   pulse_cfg.c 是"ISR + 主循环"两层里唯一碰寄存器的那一层，
 *   而它没有被任何宿主测试编译过（app_test.c 里是把 PULSE_CfgStart 整个换掉的
 *   替身函数）。也就是说：**开窗临界区（docs/03 §4 的 D4/D5）此前完全没有验证**。
 *   本替身把 TIM 寄存器、NVIC 的使能/排队位、PRIMASK 都做成可观测的宿主对象，
 *   于是"关中断 → 清 pending → 复位 → 再打开"这个顺序可以被逐条断言。
 *
 * 用法（run_all_tests.cmd 第 13 阶段的 -I 顺序）：
 *   -I test\timsim -I U1\Core\Inc
 *   test\timsim 排在 hostshim **前面**：Core/Inc/main.h 里那句
 *   `#include "stm32f1xx_hal.h"` 就会解析到本文件，而不是 hostshim 的那份。
 *
 * 忠实度：
 *   - 位定义（TIM_SR_CC2IF 等）与 ST 官方 stm32f103xb.h 的值**逐位相同**；
 *   - 中断向量号与真芯片相同（TIM1_UP=25 / TIM1_CC=27 / TIM3=29）；
 *   - NVIC 的"使能"和"排队(pending)"是两个独立位图 —— 这正是 D4 的关键：
 *     **关掉使能位并不会清掉已经排队的请求**。
 */
#ifndef TIMSIM_STM32F1XX_HAL_H
#define TIMSIM_STM32F1XX_HAL_H

#include <stdint.h>

typedef enum { HAL_OK = 0, HAL_ERROR, HAL_BUSY, HAL_TIMEOUT } HAL_StatusTypeDef;

/* ---- TIM 寄存器：只放 pulse_cfg.c 真正碰到的那些 ---- */
typedef struct {
    volatile uint32_t CR1;
    volatile uint32_t SR;
    volatile uint32_t CNT;
    volatile uint32_t DIER;
    volatile uint32_t EGR;
    volatile uint32_t CCER;
    volatile uint32_t CCR1;
    volatile uint32_t CCR2;
} TIM_TypeDef;

/* 两个定时器的寄存器块（定义在 pulse_cfg_test.c 里） */
extern TIM_TypeDef *const TIM1;
extern TIM_TypeDef *const TIM3;

/* tim.h 里声明了 extern TIM_HandleTypeDef htim1/3/4，类型必须存在 */
typedef struct { TIM_TypeDef *Instance; } TIM_HandleTypeDef;

/* ---- 位定义：值与 stm32f103xb.h 一致 ---- */
#define TIM_CR1_CEN      0x00000001u
#define TIM_EGR_UG       0x00000001u
#define TIM_SR_UIF       0x00000001u
#define TIM_SR_CC1IF     0x00000002u
#define TIM_SR_CC2IF     0x00000004u
#define TIM_DIER_UIE     0x00000001u
#define TIM_DIER_CC1IE   0x00000002u
#define TIM_DIER_CC2IE   0x00000004u
#define TIM_CCER_CC1P    0x00000002u
#define TIM_CCER_CC2P    0x00000020u

/* ---- 向量号：与真芯片一致 ---- */
typedef enum {
    TIM1_UP_IRQn = 25,
    TIM1_CC_IRQn = 27,
    TIM3_IRQn    = 29
} IRQn_Type;

#define TIMSIM_IRQ_N   64

/* ---- NVIC 调用日志：每条都记下"当时 PRIMASK 是不是 1" ---- */
#define TIMSIM_EV_SETPRIO    1u
#define TIMSIM_EV_ENABLE     2u
#define TIMSIM_EV_DISABLE    3u
#define TIMSIM_EV_CLEARPEND  4u
#define TIMSIM_LOG_MAX       64

typedef struct {
    uint8_t ev;         /* TIMSIM_EV_* */
    uint8_t irq;
    uint8_t primask;    /* 这一条发生时 PRIMASK 是不是 1（1 = 中断被关着） */
} timsim_ev_t;

extern volatile uint8_t g_primask;
extern volatile uint8_t g_nvic_en[TIMSIM_IRQ_N];
extern volatile uint8_t g_nvic_pend[TIMSIM_IRQ_N];
extern timsim_ev_t      g_nvic_log[TIMSIM_LOG_MAX];
extern int              g_nvic_log_n;

void HAL_NVIC_SetPriority(IRQn_Type irqn, uint32_t pre, uint32_t sub);
void HAL_NVIC_EnableIRQ(IRQn_Type irqn);
void HAL_NVIC_DisableIRQ(IRQn_Type irqn);
void NVIC_ClearPendingIRQ(IRQn_Type irqn);

/* CMSIS 的两个 intrinsic：宿主里换成普通函数，好让替身能记 PRIMASK */
void timsim_disable_irq(void);
void timsim_enable_irq(void);
#define __disable_irq()   timsim_disable_irq()
#define __enable_irq()    timsim_enable_irq()

uint32_t HAL_GetTick(void);

#endif /* TIMSIM_STM32F1XX_HAL_H */
