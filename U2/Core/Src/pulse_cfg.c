/**
  ******************************************************************************
  * @file    pulse_cfg.c
  * @brief   输入捕获的硬件层：TIM3(CH2=A / CH1=B) + TIM1(CH1=C) ← 主控驱动信号
  *
  * ⚠ **两片逐字节相同**（同一组信号、同一套定时器），见 `pulse_cfg.h` 的说明。
  *
  * 中断写法是**自己写的精简版**（不走 HAL_TIM_IRQHandler）：
  *   高速 PWM 时 3 相 × 2 边沿 × 20kHz = 12 万次/秒，
  *   HAL 的通用处理函数每次要跑一圈状态机太贵；这里只做三件事：
  *     1) 读 SR 一次（同时拿到"哪个通道捕获了"和"有没有溢出"）
  *     2) 读 CCR（读动作本身就清标志）→ 拼出 32 位时间戳 → 交给算法层
  *     3) 翻转该通道的捕获极性（下一次捕反方向的沿）
  *
  * 通道分散在两个定时器、各自维护自己的溢出计数：
  *   TIM3_IRQn     : CH1(B) / CH2(A) / 溢出   —— 一个向量全包
  *   TIM1_CC_IRQn  : CH1(C)                   —— 高级定时器的捕获单独一个向量
  *   TIM1_UP_IRQn  : 溢出
  * 每个相位的时间戳只来自它自己的定时器，所以两套时基互不影响。
  *
  * 溢出与捕获的相对顺序（会算错时间戳的那个坑）：
  *   读 SR 时若 UIF 置位、且捕获值很小（<0x8000），说明**先溢出、后捕获**，
  *   这一拍要按 overflow+1 记，否则时间戳会少 65536 个 tick（区间直接算错）。
  *
  * 开窗 `PULSE_CfgStart` 是一整段临界区（关 NVIC + 清 pending → 清寄存器/计数
  *   → 清算法层边沿环 → 再打开）：只关 DIER 挡不住已经排在 NVIC 里的那一发。
  *   原因与顺序见该函数里的注释（docs/03 §4 的 D4/D5）。
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
    /* ── 开窗临界区（docs/03 §4 的 D4 / D5，2026-10 修）───────────────────
     * 这里要复位三组"中断也在动"的状态：溢出计数 s_ovf3/s_ovf1、
     * 边沿环 s_ph[]（在 PULSE_Start 里清）、极性影子 s_ccer3/s_ccer1。
     *
     * ⚠ 只关 DIER 是不够的（老代码就是这么写的，所以有 D4/D5）：
     *   DIER 只挡"以后新产生"的中断请求；**在本函数进来之前就已经置起、
     *   正排在 NVIC 里等着执行**的那一次溢出/捕获，关 DIER 挡不住 ——
     *   它会在本函数返回后的某个时刻跑进来：
     *     · 把刚清零的 s_ovf3 自增一次 → 该相时间戳的高位突然多 65536 个
     *       tick（≈0.91 ms）→ 相邻两沿的区间、周期、占空比全错；
     *     · 或者往刚清空的环里追加一个边沿 → head/count 不成对 → 奇偶判错。
     *
     * 顺序不能换（换了就等于没修）：
     *   ① 关中断（PRIMASK）
     *   ② 关 NVIC 的三个向量，并清它们的 pending 位
     *      —— 清了 pending 才真正挡住"已经排上队的那一次"
     *   ③ 清外设：CNT/SR/计数/极性影子
     *      —— 必须在②之后：SR 里残留的 CCxIF/UIF 会在重开 NVIC 的瞬间
     *         又置起一次 pending
     *   ④ 复位算法层状态（PULSE_Start 清边沿环）—— 也必须在临界区里
     *   ⑤ 先开 NVIC，再开 DIER，最后才放中断
     *
     * 代价：整段约几十个周期（<2 us）。115200 下一个字节要 87 us，
     *       不会把串口收字节挡丢。 */
    __disable_irq();

    HAL_NVIC_DisableIRQ(TIM3_IRQn);
    HAL_NVIC_DisableIRQ(TIM1_CC_IRQn);
    HAL_NVIC_DisableIRQ(TIM1_UP_IRQn);
    NVIC_ClearPendingIRQ(TIM3_IRQn);
    NVIC_ClearPendingIRQ(TIM1_CC_IRQn);
    NVIC_ClearPendingIRQ(TIM1_UP_IRQn);

    PCFG3_TIM->DIER &= (uint16_t)~PCFG3_DIER_CC;
    PCFG1_TIM->DIER &= (uint16_t)~PCFG1_DIER_CC;

    PCFG3_TIM->CNT = 0u; PCFG3_TIM->SR = 0u; s_ovf3 = 0u; s_lost3 = 0u;
    PCFG1_TIM->CNT = 0u; PCFG1_TIM->SR = 0u; s_ovf1 = 0u; s_lost1 = 0u;

    /* 起始极性：都等上升沿；之后每个边沿翻一次。
     * 和中断里一样走影子，避免丢掉中断已做过的翻转。 */
    s_ccer3 &= (uint16_t)~(TIM_CCER_CC1P | TIM_CCER_CC2P);
    s_ccer1 &= (uint16_t)~(TIM_CCER_CC1P);
    PCFG3_TIM->CCER = s_ccer3;
    PCFG1_TIM->CCER = s_ccer1;

    PULSE_Start(window_ms);

    HAL_NVIC_EnableIRQ(TIM3_IRQn);
    HAL_NVIC_EnableIRQ(TIM1_CC_IRQn);
    HAL_NVIC_EnableIRQ(TIM1_UP_IRQn);

    PCFG3_TIM->DIER |= PCFG3_DIER_CC;
    PCFG1_TIM->DIER |= PCFG1_DIER_CC;

    __enable_irq();
}

/* 关窗不需要那套"清 pending"的动作：关掉 DIER 之后即使还有一次中断排队，
 * 它跑进来时 s_running 已经是 0（PULSE_Stop 置的），PULSE_OnEdge 会直接返回，
 * 因此不会再往环里追加边沿；残留的 s_ovf 自增会被下一次开窗的临界区清掉。 */
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
