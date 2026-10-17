/**
  ******************************************************************************
  * @file    enc_gen.c
  * @brief   ⑤ 编码器脉冲发生（U1 专用）：TIM4 三通道输出比较翻转，发定量脉冲串
  *
  * 设计说明见 `enc_gen.h` 的文件头。纯算术在 `enc_plan.c`。
  ******************************************************************************
  */
#include "enc_gen.h"
#include "main.h"
#include "tim.h"

/* ── 相位 → 通道 / 引脚（若上板实测发现 A/B 反了，只改这三行） ── */
#define ENC_TIM             TIM4
#define ENC_IRQn            TIM4_IRQn
#define ENC_HANDLER         TIM4_IRQHandler

#define ENC_CH_A            TIM_CHANNEL_3        /* → PB8，CN2 pin2 */
#define ENC_CH_B            TIM_CHANNEL_2        /* → PB7，CN2 pin3 */
#define ENC_CH_Z            TIM_CHANNEL_1        /* → PB6，CN2 pin4 */

#define ENC_PIN_A           GPIO_PIN_8
#define ENC_PIN_B           GPIO_PIN_7
#define ENC_PIN_Z           GPIO_PIN_6
#define ENC_GPIO            GPIOB

#define ENC_CCIF_A          TIM_SR_CC3IF
#define ENC_CCIF_B          TIM_SR_CC2IF
#define ENC_CCIF_Z          TIM_SR_CC1IF
#define ENC_CCIE_A          TIM_DIER_CC3IE
#define ENC_CCIE_B          TIM_DIER_CC2IE
#define ENC_CCIE_Z          TIM_DIER_CC1IE
#define ENC_CCE_A           TIM_CCER_CC3E
#define ENC_CCE_B           TIM_CCER_CC2E
#define ENC_CCE_Z           TIM_CCER_CC1E

/* 输出比较"翻转"模式：CCMRx 的 OCxM[2:0] = 011。
 * CCMR1 管 CH1/CH2（OC2M 在 bit14:12，OC1M 在 bit6:4），CCMR2 管 CH3（bit6:4）。
 * CCxS 都留 00 = 输出模式（这是本文件与 pulse_cfg.c 最大的不同：那边是输入捕获）。 */
#define ENC_CCMR_TOGGLE_CH1  (0x3u << 4)
#define ENC_CCMR_TOGGLE_CH2  (0x3u << 12)
#define ENC_CCMR_TOGGLE_CH3  (0x3u << 4)

/* ------------------------------------------------------------------ */
/* 状态（只在中主循环 / 中断里动，规则见各处的注释）                    */
/* ------------------------------------------------------------------ */
static volatile uint16_t s_flip[3];      /* 已翻转次数 [0]=A [1]=B [2]=Z */
static volatile uint16_t s_need[3];      /* 需要翻转的次数 = 2×脉冲数 */
static volatile uint8_t  s_done;         /* ENC_GEN_DONE_* */
/* ⚠ CCER 的读-改-写只能碰这个影子：中断里翻位、主循环收尾，
 *    直接对寄存器 &= 会被中断打断、把中断刚置的位擦掉（同 pulse_cfg.c 的 D3 教训）。 */
static volatile uint16_t s_ccer;

static uint8_t  s_busy;
static uint8_t  s_aborted;
static uint32_t s_t0;
static uint32_t s_dur_ms;
static uint16_t s_psc;
static uint32_t s_clk;
static uint8_t  s_ph_used;               /* bit0=A bit1=B bit2=Z，哪几路在发 */

/* ------------------------------------------------------------------ */
static void enc_pins_to_input(void)
{
    GPIO_InitTypeDef g;

    g.Pin   = ENC_PIN_A | ENC_PIN_B | ENC_PIN_Z;
    g.Mode  = GPIO_MODE_INPUT;
    g.Pull  = GPIO_PULLDOWN;
    g.Speed = GPIO_SPEED_FREQ_LOW;
    HAL_GPIO_Init(ENC_GPIO, &g);
}

static uint16_t enc_pins_of(uint8_t ph_mask)
{
    uint16_t m = 0u;

    if ((ph_mask & ENC_GEN_DONE_A) != 0u) { m |= ENC_PIN_A; }
    if ((ph_mask & ENC_GEN_DONE_B) != 0u) { m |= ENC_PIN_B; }
    if ((ph_mask & ENC_GEN_DONE_Z) != 0u) { m |= ENC_PIN_Z; }
    return m;
}

static void enc_pins_to_af(uint16_t pins)
{
    GPIO_InitTypeDef g;

    if (pins == 0u) {
        return;
    }
    g.Pin   = pins;
    g.Mode  = GPIO_MODE_AF_PP;              /* 推挽输出：3kΩ 上拉只多灌 1.1mA，没问题 */
    g.Pull  = GPIO_NOPULL;
    g.Speed = GPIO_SPEED_FREQ_HIGH;
    HAL_GPIO_Init(ENC_GPIO, &g);
}

/* 关掉所有输出与中断，并把定时器停住。**不改引脚**（由调用方决定收回还是保留） */
static void enc_hw_off(void)
{
    ENC_TIM->DIER = 0u;
    ENC_TIM->CR1  = 0u;
    s_ccer = 0u;
    ENC_TIM->CCER = 0u;
    ENC_TIM->SR   = 0u;
}

/* ------------------------------------------------------------------ */
void EncGen_Init(void)
{
    s_busy    = 0u;
    s_aborted = 0u;
    s_done    = 0u;
    s_ph_used = 0u;
    s_flip[0] = s_flip[1] = s_flip[2] = 0u;
    s_need[0] = s_need[1] = s_need[2] = 0u;
    s_psc     = 0u;
    s_clk     = ENC_PLAN_TIM_CLK_HZ;
    s_dur_ms  = 0u;

    enc_hw_off();
    enc_pins_to_input();                    /* 空闲态：三根线都不驱动 */
    HAL_NVIC_SetPriority(ENC_IRQn, 4, 0);   /* 同捕获：低于 USB(0)/串口(3) */
    HAL_NVIC_EnableIRQ(ENC_IRQn);
}

uint8_t EncGen_Start(const enc_plan_t *p)
{
    uint32_t f_min = 0xFFFFFFFFu;
    uint8_t  ph_mask = 0u;
    uint16_t pins;

    if (s_busy != 0u) {
        return 0u;                          /* 上一次还没收尾 */
    }
    if (EncPlan_Check(p) == 0u) {
        return 0u;
    }

    if (p->cnt_a > 0u) { ph_mask |= ENC_GEN_DONE_A; if (p->freq_a_hz < f_min) { f_min = p->freq_a_hz; } }
    if (p->cnt_b > 0u) { ph_mask |= ENC_GEN_DONE_B; if (p->freq_b_hz < f_min) { f_min = p->freq_b_hz; } }
    if (p->cnt_z > 0u) { ph_mask |= ENC_GEN_DONE_Z; if (p->freq_z_hz < f_min) { f_min = p->freq_z_hz; } }

    s_psc = EncPlan_PscFor(ENC_PLAN_TIM_CLK_HZ, f_min);
    s_clk = ENC_PLAN_TIM_CLK_HZ / ((uint32_t)s_psc + 1u);

    /* ── 先停、再复位计数（顺序很重要，见下） ── */
    enc_hw_off();
    HAL_NVIC_DisableIRQ(ENC_IRQn);          /* 挡掉"已经 pending 的 NVIC 请求" */
    HAL_NVIC_ClearPendingIRQ(ENC_IRQn);
    s_flip[0] = s_flip[1] = s_flip[2] = 0u;
    s_need[0] = (uint16_t)((uint32_t)p->cnt_a * 2u);
    s_need[1] = (uint16_t)((uint32_t)p->cnt_b * 2u);
    s_need[2] = (uint16_t)((uint32_t)p->cnt_z * 2u);
    s_done    = 0u;
    s_aborted = 0u;
    s_ph_used = ph_mask;
    s_dur_ms  = EncPlan_DurationMs(p);
    s_t0      = HAL_GetTick();

    /* ── 时基与通道 ── */
    ENC_TIM->PSC  = s_psc;
    ENC_TIM->ARR  = 0xFFFFu;                /* 翻转模式靠 CCR 定频，ARR 拉满 */
    ENC_TIM->CNT  = 0u;

    /* OCxM = toggle；清掉 CCxS/CCxPE，确保是输出比较 */
    ENC_TIM->CCMR1 = (uint16_t)(ENC_CCMR_TOGGLE_CH1 | ENC_CCMR_TOGGLE_CH2);
    ENC_TIM->CCMR2 = (uint16_t)(ENC_CCMR_TOGGLE_CH3);

    /* CCR 各填各的 → 三路频率互不干扰；不发的通道 CCR 无所谓 */
    ENC_TIM->CCR1 = (p->cnt_z > 0u) ? EncPlan_CcrFor(s_clk, p->freq_z_hz) : 0u;
    ENC_TIM->CCR2 = (p->cnt_b > 0u) ? EncPlan_CcrFor(s_clk, p->freq_b_hz) : 0u;
    ENC_TIM->CCR3 = (p->cnt_a > 0u) ? EncPlan_CcrFor(s_clk, p->freq_a_hz) : 0u;

    /* 只使能要发的那几路输出（极性高：先输出低，第一次翻转拉高） */
    s_ccer = 0u;
    if ((ph_mask & ENC_GEN_DONE_Z) != 0u) { s_ccer |= ENC_CCE_Z; }
    if ((ph_mask & ENC_GEN_DONE_B) != 0u) { s_ccer |= ENC_CCE_B; }
    if ((ph_mask & ENC_GEN_DONE_A) != 0u) { s_ccer |= ENC_CCE_A; }
    ENC_TIM->CCER = s_ccer;

    /* ── 引脚先切成 AF 推挽，再开计数器 ── */
    pins = enc_pins_of(ph_mask);
    enc_pins_to_af(pins);

    ENC_TIM->EGR = TIM_EGR_UG;              /* 立刻装载 PSC/ARR，并复位计数器 */
    ENC_TIM->SR  = 0u;                      /* 清掉 UG 带来的 UIF 与所有 CCxIF */

    {
        uint16_t dier = 0u;
        if ((ph_mask & ENC_GEN_DONE_Z) != 0u) { dier |= ENC_CCIE_Z; }
        if ((ph_mask & ENC_GEN_DONE_B) != 0u) { dier |= ENC_CCIE_B; }
        if ((ph_mask & ENC_GEN_DONE_A) != 0u) { dier |= ENC_CCIE_A; }
        ENC_TIM->DIER = dier;               /* 不开 UIE：本文件不需要溢出 */
    }

    s_busy = 1u;
    ENC_TIM->CR1 = TIM_CR1_CEN;             /* 开跑 */
    return 1u;
}

void EncGen_Poll(void)
{
    if (s_busy == 0u) {
        return;
    }
    /* 正常发完：要发的每一路都数够了 */
    if ((s_done & s_ph_used) == s_ph_used) {
        EncGen_Stop();
        return;
    }
    /* 超时兜底：中断没来、或某一路永远数不够（上板故障）→ 强停并标记 */
    if ((uint32_t)(HAL_GetTick() - s_t0) > (s_dur_ms + 100u)) {
        EncGen_Stop();
        s_aborted = 1u;
    }
}

void EncGen_Stop(void)
{
    enc_hw_off();                           /* 关输出、关中断、停计数 */
    HAL_NVIC_DisableIRQ(ENC_IRQn);
    HAL_NVIC_ClearPendingIRQ(ENC_IRQn);
    enc_pins_to_input();                    /* ★ 收回输入态（见头文件的约束） */
    s_busy = 0u;
}

uint8_t EncGen_Busy(void)      { return s_busy; }
uint8_t EncGen_DoneMask(void)  { return s_done; }
uint8_t EncGen_Aborted(void)   { return s_aborted; }
uint16_t EncGen_LastPsc(void)  { return s_psc; }
uint32_t EncGen_LastClkHz(void){ return s_clk; }

uint16_t EncGen_Flips(uint8_t ph)
{
    if (ph > 2u) {
        return 0u;
    }
    return s_flip[ph];
}

/* ------------------------------------------------------------------ */
/* 翻转中断：每翻转一次进来一次，数够 2×脉冲数就把这一路单独停掉        */
/* ------------------------------------------------------------------ */
void ENC_HANDLER(void)
{
    uint32_t sr  = ENC_TIM->SR;
    uint32_t clr = 0u;

    if ((sr & ENC_CCIF_A) != 0u) {
        clr |= ENC_CCIF_A;
        if ((s_done & ENC_GEN_DONE_A) == 0u) {
            s_flip[0]++;
            if (s_flip[0] >= s_need[0]) {
                s_done |= ENC_GEN_DONE_A;
                s_ccer &= (uint16_t)~ENC_CCE_A;      /* 不再翻转，电平保持 */
                ENC_TIM->CCER  = s_ccer;
                ENC_TIM->DIER &= (uint16_t)~ENC_CCIE_A;
            }
        }
    }
    if ((sr & ENC_CCIF_B) != 0u) {
        clr |= ENC_CCIF_B;
        if ((s_done & ENC_GEN_DONE_B) == 0u) {
            s_flip[1]++;
            if (s_flip[1] >= s_need[1]) {
                s_done |= ENC_GEN_DONE_B;
                s_ccer &= (uint16_t)~ENC_CCE_B;
                ENC_TIM->CCER  = s_ccer;
                ENC_TIM->DIER &= (uint16_t)~ENC_CCIE_B;
            }
        }
    }
    if ((sr & ENC_CCIF_Z) != 0u) {
        clr |= ENC_CCIF_Z;
        if ((s_done & ENC_GEN_DONE_Z) == 0u) {
            s_flip[2]++;
            if (s_flip[2] >= s_need[2]) {
                s_done |= ENC_GEN_DONE_Z;
                s_ccer &= (uint16_t)~ENC_CCE_Z;
                ENC_TIM->CCER  = s_ccer;
                ENC_TIM->DIER &= (uint16_t)~ENC_CCIE_Z;
            }
        }
    }

    /* ⚠ 输出比较的 CCxIF **必须写 0 清**（不像输入捕获那样"读 CCR 就清"）。
     *   忘了这一步的表现是：中断反复进来、计数飞涨、发出来的脉冲数完全不对。 */
    if (clr != 0u) {
        ENC_TIM->SR = (uint16_t)~clr;
    }
}
