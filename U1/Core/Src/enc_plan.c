/**
  ******************************************************************************
  * @file    enc_plan.c
  * @brief   ⑤ 编码器脉冲发生的纯规划层（参数校验 + 定时器寄存器换算）
  *
  * 纯函数：不碰 HAL / 全局状态 → 宿主单测见 `test/enc_plan_test.c`。
  * 设计取舍见 `enc_plan.h` 的文件头。
  ******************************************************************************
  */
#include "enc_plan.h"

/* ------------------------------------------------------------------ */
uint32_t EncPlan_Edges(uint16_t pulses)
{
    return (uint32_t)pulses * 2u;
}

/* ------------------------------------------------------------------ */
uint32_t EncPlan_PhaseMs(uint16_t pulses, uint16_t freq_hz)
{
    if ((pulses == 0u) || (freq_hz == 0u)) {
        return 0u;
    }
    /* 向上取整：宁可多等一点，也不要把最后一个脉冲切掉。
     * 1000 个 @5kHz = 200ms（整除）；1 个 @5kHz = 0.2ms → 1ms。 */
    return (((uint32_t)pulses * 1000u) + (uint32_t)freq_hz - 1u) / (uint32_t)freq_hz;
}

/* ------------------------------------------------------------------ */
uint16_t EncPlan_PscFor(uint32_t tim_clk_hz, uint32_t f_min_hz)
{
    uint64_t need;      /* PSC+1 至少要是多少 */
    uint64_t psc;

    if (tim_clk_hz == 0u) {
        return 0xFFFFu;
    }
    if (f_min_hz == 0u) {
        f_min_hz = ENC_PLAN_FREQ_MIN_HZ;
    }

    /* 要 CCR = clk/(2f) <= 65535，而 clk_after_psc = tim_clk/(PSC+1) →
     *   (PSC+1) >= tim_clk / (2 × f × 65535)
     * 用 64 位算，避免 2×f×65535 顶掉 u32。 */
    need = ((uint64_t)tim_clk_hz + (2u * (uint64_t)f_min_hz * 65535u) - 1u) /
           (2u * (uint64_t)f_min_hz * 65535u);
    if (need == 0u) {
        need = 1u;                          /* 至少不分频 */
    }
    psc = need - 1u;
    if (psc > 0xFFFFu) {
        return 0xFFFFu;                     /* PSC 都塞不下：这个频率本实现发不出来 */
    }
    return (uint16_t)psc;
}

/* ------------------------------------------------------------------ */
uint16_t EncPlan_CcrFor(uint32_t clk_after_psc_hz, uint16_t freq_hz)
{
    uint64_t ccr;

    if ((clk_after_psc_hz == 0u) || (freq_hz == 0u)) {
        return 0u;
    }
    ccr = ((uint64_t)clk_after_psc_hz + (uint64_t)freq_hz) / (2u * (uint64_t)freq_hz);
    /* 上面 +freq 是四舍五入；翻转模式下频率误差 = 舍入量/(2*CCR)，远小于 1%。 */
    if ((ccr == 0u) || (ccr > 65535u)) {
        return 0u;
    }
    return (uint16_t)ccr;
}

/* ------------------------------------------------------------------ */
uint32_t EncPlan_DurationMs(const enc_plan_t *p)
{
    uint32_t d = 0u;
    uint32_t t;

    if (p == 0) {
        return 0u;
    }
    t = EncPlan_PhaseMs(p->cnt_a, p->freq_a_hz);
    if (t > d) { d = t; }
    t = EncPlan_PhaseMs(p->cnt_b, p->freq_b_hz);
    if (t > d) { d = t; }
    t = EncPlan_PhaseMs(p->cnt_z, p->freq_z_hz);
    if (t > d) { d = t; }

    if (d == 0u) {
        return 0u;                          /* 三路都不发 */
    }
    return d + ENC_PLAN_MARGIN_MS;
}

/* ------------------------------------------------------------------ */
static uint8_t freq_ok(uint16_t f)
{
    return (uint8_t)(((f >= ENC_PLAN_FREQ_MIN_HZ) && (f <= ENC_PLAN_FREQ_MAX_HZ)) ? 1u : 0u);
}

uint8_t EncPlan_Check(const enc_plan_t *p)
{
    if (p == 0) {
        return 0u;
    }
    /* 三路频率都必须在量程内 —— 即使某一路 cnt=0 也校验，
     * 免得 PC 传了乱七八糟的数却因为"恰好那一路不发"而蒙混过关。 */
    if ((freq_ok(p->freq_a_hz) == 0u) ||
        (freq_ok(p->freq_b_hz) == 0u) ||
        (freq_ok(p->freq_z_hz) == 0u)) {
        return 0u;
    }
    if ((p->cnt_a > ENC_PLAN_PULSE_MAX) ||
        (p->cnt_b > ENC_PLAN_PULSE_MAX) ||
        (p->cnt_z > ENC_PLAN_PULSE_MAX)) {
        return 0u;
    }
    /* 三路都不发没意义（多半是 PC 传错了） */
    if ((p->cnt_a == 0u) && (p->cnt_b == 0u) && (p->cnt_z == 0u)) {
        return 0u;
    }
    /* 要发的那一路，PSC/CCR 得算得出来 */
    {
        uint32_t f_min = 0xFFFFFFFFu;
        uint16_t psc;
        uint32_t clk;

        if ((p->cnt_a > 0u) && (p->freq_a_hz < f_min)) { f_min = p->freq_a_hz; }
        if ((p->cnt_b > 0u) && (p->freq_b_hz < f_min)) { f_min = p->freq_b_hz; }
        if ((p->cnt_z > 0u) && (p->freq_z_hz < f_min)) { f_min = p->freq_z_hz; }

        psc = EncPlan_PscFor(ENC_PLAN_TIM_CLK_HZ, f_min);
        if (psc == 0xFFFFu) {
            return 0u;                      /* 频率太低，PSC 都压不住 */
        }
        clk = ENC_PLAN_TIM_CLK_HZ / ((uint32_t)psc + 1u);

        if ((p->cnt_a > 0u) && (EncPlan_CcrFor(clk, p->freq_a_hz) == 0u)) { return 0u; }
        if ((p->cnt_b > 0u) && (EncPlan_CcrFor(clk, p->freq_b_hz) == 0u)) { return 0u; }
        if ((p->cnt_z > 0u) && (EncPlan_CcrFor(clk, p->freq_z_hz) == 0u)) { return 0u; }
    }
    return 1u;
}

/* ------------------------------------------------------------------ */
void EncPlan_Default(enc_plan_t *p)
{
    if (p == 0) {
        return;
    }
    p->cnt_a     = 1000u;                   /* A 相 1000 个脉冲 */
    p->cnt_b     = 2000u;                   /* B 相 2000 个（与 A 不同 → 可分辨通道） */
    p->cnt_z     = 1u;                      /* Z 是索引，真实编码器每转 1 个 */
    p->freq_a_hz = 5000u;
    p->freq_b_hz = 10000u;
    p->freq_z_hz = 5000u;
}
