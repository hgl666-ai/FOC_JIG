/**
  ******************************************************************************
  * @file    enc_cap.c
  * @brief   ⑤ 编码器回环采集（U2 专用）：TIM4 三通道数边沿
  *
  * 设计说明见 `enc_cap.h` 的文件头。
  ******************************************************************************
  */
#include "enc_cap.h"
#include "main.h"
#include "tim.h"

#define ENC_TIM             TIM4
#define ENC_IRQn            TIM4_IRQn
#define ENC_HANDLER         TIM4_IRQHandler

/* 与 U1 的 enc_gen.c **同一张表**（同一根线、同一个定时器）：
 *   A → CH3 → PB8 ; B → CH2 → PB7 ; Z → CH1 → PB6
 * 若上板发现 A/B 反了，两个文件要一起改（U2 采的相位要和 U1 发的对得上）。 */
#define ENC_CCIF_A          TIM_SR_CC3IF
#define ENC_CCIF_B          TIM_SR_CC2IF
#define ENC_CCIF_Z          TIM_SR_CC1IF
#define ENC_CCP_A           TIM_CCER_CC3P
#define ENC_CCP_B           TIM_CCER_CC2P
#define ENC_CCP_Z           TIM_CCER_CC1P

#define ENC_DIER_CC         (TIM_DIER_CC1IE | TIM_DIER_CC2IE | TIM_DIER_CC3IE)

static volatile uint32_t s_cnt_a;
static volatile uint32_t s_cnt_b;
static volatile uint32_t s_cnt_z;
static volatile uint16_t s_lost;        /* 诊断：清标志之后马上又置位（边沿比中断还快）*/
/* ⚠ CCER 的读-改-写只碰影子（同 pulse_cfg.c 的 D3 教训）：中断里翻极性。 */
static volatile uint16_t s_ccer;
static uint8_t s_enabled;

/* ------------------------------------------------------------------ */
void EncCap_Init(void)
{
    /* 时基/通道已在 MX_TIM4_Init 里配成输入捕获（见 tim.c）。这里只做：
     * 清标志 → 开计数器自由运行 → 开 NVIC → 开捕获中断。 */
    ENC_TIM->CR1  = 0u;
    ENC_TIM->SR   = 0u;
    ENC_TIM->CNT  = 0u;
    ENC_TIM->DIER &= (uint16_t)~ENC_DIER_CC;
    ENC_TIM->EGR  = TIM_EGR_UG;             /* 装载 PSC/ARR */
    ENC_TIM->SR   = 0u;
    ENC_TIM->CR1  = TIM_CR1_CEN;            /* 自由运行（数边沿不需要溢出计数） */

    s_ccer = ENC_TIM->CCER;
    __disable_irq();
    s_ccer &= (uint16_t)~(TIM_CCER_CC1P | TIM_CCER_CC2P | TIM_CCER_CC3P);  /* 先等上升沿 */
    ENC_TIM->CCER = s_ccer;
    __enable_irq();

    s_cnt_a = s_cnt_b = s_cnt_z = 0u;
    s_lost  = 0u;
    s_enabled = 0u;

    HAL_NVIC_SetPriority(ENC_IRQn, 4, 0);   /* 同捕获：低于 USB(0)/串口(3) */
    HAL_NVIC_EnableIRQ(ENC_IRQn);
    ENC_TIM->DIER |= ENC_DIER_CC;           /* 一直使能：自由计数 */
    s_enabled = 1u;
}

void EncCap_Clear(void)
{
    /* 关中断再清：否则"清完立刻被中断加回去"会让计数不是从 0 开始。
     * 这就是 docs/03 §4 里 D4/D5 那类"复位与 ISR 抢计数"的修法。 */
    __disable_irq();
    s_cnt_a = 0u;
    s_cnt_b = 0u;
    s_cnt_z = 0u;
    s_lost  = 0u;
    __enable_irq();
}

void EncCap_Read(link_enc_cnt_t *out)
{
    if (out == 0) {
        return;
    }
    /* 逐字段读 volatile：三个 u32 之间可能被中断插入，
     * 但每个字段本身是 32 位对齐读，不会撕裂；⑤ 的判据是"个数对不对"，
     * 差一个边沿的最坏情况不影响结论（真要求严格就关中断读）。 */
    __disable_irq();
    out->a    = s_cnt_a;
    out->b    = s_cnt_b;
    out->z    = s_cnt_z;
    out->lost = s_lost;
    __enable_irq();
}

uint8_t EncCap_Enabled(void) { return s_enabled; }

void EncCap_Stop(void)
{
    ENC_TIM->DIER &= (uint16_t)~ENC_DIER_CC;
    s_enabled = 0u;
}

/* ------------------------------------------------------------------ */
/* 捕获中断：只做"计数 + 翻极性"，其它什么都不干（才会够快）           */
/* ------------------------------------------------------------------ */
void ENC_HANDLER(void)
{
    uint32_t sr = ENC_TIM->SR;
    uint32_t ccr;

    if ((sr & ENC_CCIF_A) != 0u) {
        ccr = ENC_TIM->CCR3;                /* 读 CCR 自动清 CC3IF（输入捕获） */
        (void)ccr;
        s_cnt_a++;
        s_ccer ^= ENC_CCP_A;                /* 下次捕反方向的沿 → 上升下降都算 */
        ENC_TIM->CCER = s_ccer;
    }
    if ((sr & ENC_CCIF_B) != 0u) {
        ccr = ENC_TIM->CCR2;
        (void)ccr;
        s_cnt_b++;
        s_ccer ^= ENC_CCP_B;
        ENC_TIM->CCER = s_ccer;
    }
    if ((sr & ENC_CCIF_Z) != 0u) {
        ccr = ENC_TIM->CCR1;
        (void)ccr;
        s_cnt_z++;
        s_ccer ^= ENC_CCP_Z;
        ENC_TIM->CCER = s_ccer;
    }

    /* 诊断：清完之后立刻又置位 = 边沿比中断还快，可能数漏了。
     * 正常情况（几 kHz~几十 kHz）永远是 0；上板看到非 0 就要查中断负载。 */
    if ((ENC_TIM->SR & (ENC_CCIF_A | ENC_CCIF_B | ENC_CCIF_Z)) != 0u) {
        s_lost++;
    }
}
