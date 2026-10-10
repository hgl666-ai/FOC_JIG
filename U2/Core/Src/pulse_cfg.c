/**
  ******************************************************************************
  * @file    pulse_cfg.c
  * @brief   U2 输入捕获的硬件层：TIM3(CH2/CH1) + TIM1(CH1) ← 主控驱动信号
  *
  * 与 U1 同一套写法（自己写的精简中断，不走 HAL_TIM_IRQHandler），
  * 差别只是通道分散在两个定时器、各自维护自己的溢出计数：
  *   TIM3_IRQn     : CH1(B) / CH2(A) / 溢出   —— 一个向量全包
  *   TIM1_CC_IRQn  : CH1(C)                   —— 高级定时器的捕获单独一个向量
  *   TIM1_UP_IRQn  : 溢出
  * 每个相位的时间戳只来自它自己的定时器，所以两套时基互不影响。
  ******************************************************************************
  */
#include "pulse_cfg.h"
#include "main.h"
#include "tim.h"

/* ── TIM3: A 相 = CH2, B 相 = CH1 ── */
#define PCFG3_TIM       TIM3
#define PCFG3_HANDLER   TIM3_IRQHandler
#define PCFG3_CC_A      TIM_SR_CC2IF
#define PCFG3_CC_B      TIM_SR_CC1IF
#define PCFG3_CCP_A     TIM_CCER_CC2P
#define PCFG3_CCP_B     TIM_CCER_CC1P
#define PCFG3_DIER_CC   (TIM_DIER_CC1IE | TIM_DIER_CC2IE | TIM_DIER_UIE)

/* ── TIM1: C 相 = CH1 ── */
#define PCFG1_TIM       TIM1
#define PCFG1_CC        TIM_SR_CC1IF
#define PCFG1_CCP       TIM_CCER_CC1P
#define PCFG1_DIER_CC   (TIM_DIER_CC1IE | TIM_DIER_UIE)

static volatile uint16_t s_ovf3;       /* TIM3 溢出次数 */
static volatile uint16_t s_ovf1;       /* TIM1 溢出次数 */
static volatile uint32_t s_lost3, s_lost1;
/* ⚠ CCER 的读-改-写只能碰这两个影子（TIM3 / TIM1 各一个）：
 *    中断里翻极性、主循环清极性，直接对寄存器做 ^=/&= 会被对方打断，
 *    丢掉一次翻转 → 该相高低电平判反（占空比 ≈ 1-duty）。 */
static volatile uint16_t s_ccer3;
static volatile uint16_t s_ccer1;

/* ------------------------------------------------------------------ */
void PULSE_CfgInit(void)
{
    /* 两个定时器都在 MX_TIMx_Init 里配成输入捕获（见 tim.c）。
     * 这里只做：清标志 → 关捕获中断 → 计数器自由运行 → 开 NVIC。 */
    PCFG3_TIM->CR1 = 0u;
    PCFG3_TIM->SR  = 0u;
    PCFG3_TIM->CNT = 0u;
    PCFG3_TIM->DIER &= (uint16_t)~PCFG3_DIER_CC;
    PCFG3_TIM->EGR = TIM_EGR_UG;
    PCFG3_TIM->SR  = 0u;
    PCFG3_TIM->CR1 = TIM_CR1_CEN;

    PCFG1_TIM->CR1 = 0u;
    PCFG1_TIM->SR  = 0u;
    PCFG1_TIM->CNT = 0u;
    PCFG1_TIM->DIER &= (uint16_t)~PCFG1_DIER_CC;
    PCFG1_TIM->EGR = TIM_EGR_UG;
    PCFG1_TIM->SR  = 0u;
    PCFG1_TIM->CR1 = TIM_CR1_CEN;

    s_ovf3 = 0u; s_ovf1 = 0u;
    s_lost3 = 0u; s_lost1 = 0u;
    s_ccer3 = PCFG3_TIM->CCER;                 /* 影子初值 = 定时器当前极性配置 */
    s_ccer1 = PCFG1_TIM->CCER;

    HAL_NVIC_SetPriority(TIM3_IRQn, 4, 0);
    HAL_NVIC_EnableIRQ(TIM3_IRQn);
    HAL_NVIC_SetPriority(TIM1_CC_IRQn, 4, 0);
    HAL_NVIC_EnableIRQ(TIM1_CC_IRQn);
    HAL_NVIC_SetPriority(TIM1_UP_IRQn, 4, 0);
    HAL_NVIC_EnableIRQ(TIM1_UP_IRQn);
}

void PULSE_CfgStart(uint16_t window_ms)
{
    PCFG3_TIM->DIER &= (uint16_t)~PCFG3_DIER_CC;
    PCFG1_TIM->DIER &= (uint16_t)~PCFG1_DIER_CC;

    PCFG3_TIM->CNT = 0u; PCFG3_TIM->SR = 0u; s_ovf3 = 0u; s_lost3 = 0u;
    PCFG1_TIM->CNT = 0u; PCFG1_TIM->SR = 0u; s_ovf1 = 0u; s_lost1 = 0u;

    /* 起始极性：都等上升沿；之后每个边沿翻一次。
     * 和中断里一样走影子 + 关中断几拍，避免丢掉中断已做过的翻转。 */
    __disable_irq();
    s_ccer3 &= (uint16_t)~(TIM_CCER_CC1P | TIM_CCER_CC2P);
    s_ccer1 &= (uint16_t)~(TIM_CCER_CC1P);
    PCFG3_TIM->CCER = s_ccer3;
    PCFG1_TIM->CCER = s_ccer1;
    __enable_irq();

    PULSE_Start(window_ms);

    PCFG3_TIM->DIER |= PCFG3_DIER_CC;
    PCFG1_TIM->DIER |= PCFG1_DIER_CC;
}

void PULSE_CfgStop(void)
{
    PCFG3_TIM->DIER &= (uint16_t)~PCFG3_DIER_CC;
    PCFG1_TIM->DIER &= (uint16_t)~PCFG1_DIER_CC;
    PULSE_Stop();
}

uint8_t PULSE_CfgPoll(void)
{
    uint8_t st = PULSE_Poll();

    if (st != 0u) {
        PCFG3_TIM->DIER &= (uint16_t)~PCFG3_DIER_CC;
        PCFG1_TIM->DIER &= (uint16_t)~PCFG1_DIER_CC;
    }
    return st;
}

uint32_t PULSE_CfgLostEdges(void)
{
    return s_lost3 + s_lost1;
}

/* ------------------------------------------------------------------ */
/* TIM3：A 相(CH2) + B 相(CH1) + 溢出                                  */
/* ------------------------------------------------------------------ */
void PCFG3_HANDLER(void)
{
    uint32_t sr = PCFG3_TIM->SR;
    uint32_t extra;
    uint32_t ccr;

    if ((sr & PCFG3_CC_A) != 0u) {
        ccr = PCFG3_TIM->CCR2;
        extra = (((sr & TIM_SR_UIF) != 0u) && (ccr < 0x8000u)) ? 1u : 0u;
        PULSE_OnEdge(PULSE_PH_A, ((((uint32_t)s_ovf3 + extra) << 16) | (ccr & 0xFFFFu)));
        s_ccer3 ^= PCFG3_CCP_A;
        PCFG3_TIM->CCER = s_ccer3;
    }
    if ((sr & PCFG3_CC_B) != 0u) {
        ccr = PCFG3_TIM->CCR1;
        extra = (((sr & TIM_SR_UIF) != 0u) && (ccr < 0x8000u)) ? 1u : 0u;
        PULSE_OnEdge(PULSE_PH_B, ((((uint32_t)s_ovf3 + extra) << 16) | (ccr & 0xFFFFu)));
        s_ccer3 ^= PCFG3_CCP_B;
        PCFG3_TIM->CCER = s_ccer3;
    }
    if ((sr & TIM_SR_UIF) != 0u) {
        PCFG3_TIM->SR = (uint16_t)~TIM_SR_UIF;
        s_ovf3++;
    }
}

/* ------------------------------------------------------------------ */
/* TIM1：C 相(CH1) 的捕获                                            */
/* ------------------------------------------------------------------ */
void TIM1_CC_IRQHandler(void)
{
    uint32_t sr = PCFG1_TIM->SR;
    uint32_t extra;
    uint32_t ccr;

    if ((sr & PCFG1_CC) != 0u) {
        ccr = PCFG1_TIM->CCR1;
        extra = (((sr & TIM_SR_UIF) != 0u) && (ccr < 0x8000u)) ? 1u : 0u;
        PULSE_OnEdge(PULSE_PH_C, ((((uint32_t)s_ovf1 + extra) << 16) | (ccr & 0xFFFFu)));
        s_ccer1 ^= PCFG1_CCP;
        PCFG1_TIM->CCER = s_ccer1;
    }
}

/* TIM1 的溢出（单独向量）*/
void TIM1_UP_IRQHandler(void)
{
    if ((PCFG1_TIM->SR & TIM_SR_UIF) != 0u) {
        PCFG1_TIM->SR = (uint16_t)~TIM_SR_UIF;
        s_ovf1++;
    }
}
