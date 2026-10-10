/**
  ******************************************************************************
  * @file    pulse_cfg.c
  * @brief   U1 输入捕获的硬件层：TIM4 CH1/CH2/CH3 ← 三相输出端
  *
  * 中断写法是**自己写的精简版**（不走 HAL_TIM_IRQHandler）：
  *   高速 PWM 时 3 相 × 2 边沿 × 20kHz = 12 万次/秒，
  *   HAL 的通用处理函数每次要跑一圈状态机太贵；这里只做三件事：
  *     1) 读 SR 一次（同时拿到"哪个通道捕获了"和"有没有溢出"）
  *     2) 读 CCR（读动作本身就清标志）→ 拼出 32 位时间戳 → 交给算法层
  *     3) 翻转该通道的捕获极性（下一次捕反方向的沿）
  *
  * 溢出与捕获的相对顺序（会算错时间戳的那个坑）：
  *   读 SR 时若 UIF 置位、且捕获值很小（<0x8000），说明**先溢出、后捕获**，
  *   这一拍要按 overflow+1 记，否则时间戳会少 65536 个 tick（区间直接算错）。
  ******************************************************************************
  */
#include "pulse_cfg.h"
#include "main.h"
#include "tim.h"

/* ── 相位 → 通道绑定（改接线只改这里） ── */
#define PCFG_TIM        TIM4
#define PCFG_IRQn       TIM4_IRQn
#define PCFG_HANDLER    TIM4_IRQHandler
#define PCFG_HTIM       htim4

#define PCFG_CC_A       TIM_SR_CC3IF      /* A 相 = CH3 */
#define PCFG_CC_B       TIM_SR_CC2IF      /* B 相 = CH2 */
#define PCFG_CC_C       TIM_SR_CC1IF      /* C 相 = CH1 */
#define PCFG_CCP_A      TIM_CCER_CC3P
#define PCFG_CCP_B      TIM_CCER_CC2P
#define PCFG_CCP_C      TIM_CCER_CC1P

#define PCFG_DIER_CC    (TIM_DIER_CC1IE | TIM_DIER_CC2IE | TIM_DIER_CC3IE | TIM_DIER_UIE)

static volatile uint16_t s_ovf;        /* 16 位计数器溢出次数（时间戳高位） */
static volatile uint32_t s_lost;       /* 诊断：极短间隔导致标志被覆盖的次数 */
/* ⚠ CCER 的读-改-写只能碰这个影子：中断里翻极性、主循环清极性，
 *    直接对寄存器做 ^= / &= 会被对方打断、丢掉一次翻转 → 该相高低电平判反。 */
static volatile uint16_t s_ccer;

/* ------------------------------------------------------------------ */
void PULSE_CfgInit(void)
{
    /* 定时器时基/通道已在 MX_TIM4_Init 里配成输入捕获（见 tim.c）。
     * 这里只负责：清标志 → 关捕获中断 → 让计数器自由运行 → 开 NVIC。 */
    PCFG_TIM->CR1  = 0u;                       /* 先停 */
    PCFG_TIM->SR   = 0u;                       /* 写 0 清所有标志 */
    PCFG_TIM->CNT  = 0u;
    PCFG_TIM->DIER &= (uint16_t)~PCFG_DIER_CC; /* 捕获中断先关着（等窗口） */
    PCFG_TIM->EGR  = TIM_EGR_UG;               /* 立刻装载 PSC/ARR */
    PCFG_TIM->SR   = 0u;
    PCFG_TIM->CR1  = TIM_CR1_CEN;              /* 计数器自由运行（72MHz） */

    s_ovf  = 0u;
    s_lost = 0u;
    s_ccer = PCFG_TIM->CCER;                   /* 影子初值 = 定时器当前极性配置 */
    HAL_NVIC_SetPriority(PCFG_IRQn, 4, 0);     /* 优先级 4：低于 USB(0)/串口(3) */
    HAL_NVIC_EnableIRQ(PCFG_IRQn);
}

void PULSE_CfgStart(uint16_t window_ms)
{
    PCFG_TIM->DIER &= (uint16_t)~PCFG_DIER_CC; /* 清场期间不响应 */
    PCFG_TIM->CNT   = 0u;
    PCFG_TIM->SR    = 0u;
    s_ovf  = 0u;
    s_lost = 0u;
    s_ccer = PCFG_TIM->CCER;                   /* 影子初值 = 定时器当前极性配置 */
    /* 起始极性：三相都等上升沿；之后每个边沿翻一次。
     * 这里和中断里都会改 CCER，所以走影子 + 关中断几拍，避免丢掉中断的翻转。 */
    __disable_irq();
    s_ccer &= (uint16_t)~(TIM_CCER_CC1P | TIM_CCER_CC2P | TIM_CCER_CC3P);
    PCFG_TIM->CCER = s_ccer;
    __enable_irq();

    PULSE_Start(window_ms);                    /* 算法层开窗 */
    PCFG_TIM->DIER |= PCFG_DIER_CC;            /* 开捕获 + 溢出中断 */
}

void PULSE_CfgStop(void)
{
    PCFG_TIM->DIER &= (uint16_t)~PCFG_DIER_CC;
    PULSE_Stop();
}

uint8_t PULSE_CfgPoll(void)
{
    uint8_t st = PULSE_Poll();

    if (st != 0u) {                            /* 结算完/没在采 → 关掉中断 */
        PCFG_TIM->DIER &= (uint16_t)~PCFG_DIER_CC;
    }
    return st;
}

uint32_t PULSE_CfgLostEdges(void)
{
    return s_lost;
}

/* ------------------------------------------------------------------ */
/* 捕获中断：只做"取时间戳 + 翻极性"，不做除法/滤波                     */
/* ------------------------------------------------------------------ */
void PCFG_HANDLER(void)
{
    uint32_t sr;
    uint32_t extra;
    uint32_t ccr;
    uint32_t ts;

    sr = PCFG_TIM->SR;

    /* 溢出与捕获谁先谁后：UIF 置位且 CCR 很小 → 这次捕获在溢出之后 */
    extra = 0u;

    if ((sr & PCFG_CC_A) != 0u) {
        ccr = PCFG_TIM->CCR3;
        extra = (((sr & TIM_SR_UIF) != 0u) && (ccr < 0x8000u)) ? 1u : 0u;
        ts = (((uint32_t)s_ovf + extra) << 16) | (ccr & 0xFFFFu);
        PULSE_OnEdge(PULSE_PH_A, ts);
        s_ccer ^= PCFG_CCP_A;
        PCFG_TIM->CCER = s_ccer;          /* 下次捕反方向的沿 */
    }
    if ((sr & PCFG_CC_B) != 0u) {
        ccr = PCFG_TIM->CCR2;
        extra = (((sr & TIM_SR_UIF) != 0u) && (ccr < 0x8000u)) ? 1u : 0u;
        ts = (((uint32_t)s_ovf + extra) << 16) | (ccr & 0xFFFFu);
        PULSE_OnEdge(PULSE_PH_B, ts);
        s_ccer ^= PCFG_CCP_B;
        PCFG_TIM->CCER = s_ccer;
    }
    if ((sr & PCFG_CC_C) != 0u) {
        ccr = PCFG_TIM->CCR1;
        extra = (((sr & TIM_SR_UIF) != 0u) && (ccr < 0x8000u)) ? 1u : 0u;
        ts = (((uint32_t)s_ovf + extra) << 16) | (ccr & 0xFFFFu);
        PULSE_OnEdge(PULSE_PH_C, ts);
        s_ccer ^= PCFG_CCP_C;
        PCFG_TIM->CCER = s_ccer;
    }
    if ((sr & TIM_SR_UIF) != 0u) {
        PCFG_TIM->SR = (uint16_t)~TIM_SR_UIF;  /* 只清 UIF（其它位写 1 无效） */
        s_ovf++;
    }
}
