/**
  ******************************************************************************
  * @file    enc_plan.h
  * @brief   ⑤ 编码器脉冲发生的**纯规划层**：参数校验 + 定时器寄存器换算
  *
  * 为什么单独一层：`enc_gen.c` 要直接写 TIM4/GPIO 寄存器，宿主机上编译不了；
  * 而"频率能不能实现、CCR 该填多少、这一串要发多久"全是纯算术，
  * 恰恰是最容易算错的地方 → 拆出来单独做宿主单测（`test/enc_plan_test.c`）。
  *
  * 硬件事实（见 docs/00 §3.5，两片都接在同一根线上）
  * ---------------------------------------------------
  *   CN2（XH-5PA，5 脚：5V / A / B / Z / GND）的三个信号脚
  *     → U1 的 PB8/PB7/PB6 = TIM4_CH3/CH2/CH1
  *     → **U2 的同一组脚也挂着**
  *   线上还有 3kΩ 上拉到 3.3V + SMF3.3A 对地。
  *
  * 所以 ⑤ 的做法是：**U1 把 TIM4 配成输出发定量脉冲串，U2 用 TIM4 采集做回环**，
  * 被测板再从这三根线上数它自己的计数（用 `DUT_GET_ENC` 要回来）→ 三方对比。
  *
  * ⚠ **同一时刻只能有一片驱动这三根线**：U1 输出时 U2 必须保持输入（反之亦然），
  *   否则两片推挽输出 + 编码器 = 三个驱动源抢同一根线（docs/12 §2）。
  *
  * 为什么用"输出比较翻转"模式（OCxM = Toggle）而不是 PWM
  * ------------------------------------------------------
  * 三路要发**不同的个数**，而 A/B 的频率还可能不同。
  * 翻转模式下每路的翻转时刻由**自己的 CCR** 决定 → 三路可以各走各的频率、
  * 各自数自己的翻转次数、到数了就单独停，互不干扰。
  * （用 PWM 模式的话三路共用一个 ARR，只能同频。）
  ******************************************************************************
  */
#ifndef __ENC_PLAN_H
#define __ENC_PLAN_H

#include <stdint.h>

/* ------------------------------------------------------------------ */
/* 1. 参数范围                                                        */
/* ------------------------------------------------------------------ */
#define ENC_PLAN_FREQ_MIN_HZ    100u      /* 下限：再低 PSC 也压不住（见 PscFor） */
#define ENC_PLAN_FREQ_MAX_HZ    20000u    /* 上限：受中断负载限制（三路翻转中断之和） */
#define ENC_PLAN_PULSE_MAX      32767u    /* 单路脉冲数上限（翻转次数 = 2×脉冲数，别溢出 u16） */
#define ENC_PLAN_TIM_CLK_HZ     72000000u /* TIM4 在 APB1 上，定时器时钟 72MHz */
#define ENC_PLAN_MARGIN_MS      5u        /* 时长估算的安全余量（最后一个翻转 + 主循环抖动） */

/* ------------------------------------------------------------------ */
/* 2. 规划                                                           */
/* ------------------------------------------------------------------ */
typedef struct {
    uint16_t cnt_a;        /* A 相要发几个脉冲（0 = 这一路不发） */
    uint16_t cnt_b;
    uint16_t cnt_z;
    uint16_t freq_a_hz;    /* 各相频率；该路 cnt=0 时频率被忽略（但也必须在量程内才校验通过） */
    uint16_t freq_b_hz;
    uint16_t freq_z_hz;
} enc_plan_t;

/* ------------------------------------------------------------------ */
/* 3. 纯函数                                                          */
/* ------------------------------------------------------------------ */
/**
  * @brief  参数校验
  * @retval 1 = 合法；0 = 非法（调用方应回 ERR_PARAM，且**不能有任何输出**）
  * @note   规则：三路都必须在 [100, 20000] Hz；脉冲数 ≤ ENC_PLAN_PULSE_MAX；
  *         至少有一路要发（三路全 0 没意义）；cnt>0 的那一路频率必须非 0。
  */
uint8_t  EncPlan_Check(const enc_plan_t *p);

/**
  * @brief  按最低的那一路频率算预分频 PSC
  * @param  tim_clk_hz 定时器输入时钟（ENC_PLAN_TIM_CLK_HZ）
  * @param  f_min_hz   本次要发的最低频率（<=0 时按下限算）
  * @retval PSC 值；若连最低频率都压不住（返回值会 > 0xFFFF）则返回 0xFFFF 表示不可实现
  * @note   翻转模式下要保证 CCR = clk/(2f) 能塞进 16 位，所以低频要靠 PSC 把 clk 降下来。
  */
uint16_t EncPlan_PscFor(uint32_t tim_clk_hz, uint32_t f_min_hz);

/**
  * @brief  频率 → 翻转模式的 CCR
  * @param  clk_after_psc_hz 预分频之后的计数时钟（= tim_clk / (PSC+1)）
  * @retval CCR；0 表示该频率在这套时钟下实现不了（CCR 算出来是 0 或 > 65535）
  * @note   翻转模式下一个完整周期 = 2 次翻转 = 2×ARR 个计数，
  *         所以 f = clk / (2 × CCR) → CCR = clk / (2f)。ARR 固定 0xFFFF。
  */
uint16_t EncPlan_CcrFor(uint32_t clk_after_psc_hz, uint16_t freq_hz);

/**
  * @brief  脉冲数 → 边沿数（治具侧和 DUT 侧对表用：1 个脉冲 = 2 个边沿）
  */
uint32_t EncPlan_Edges(uint16_t pulses);

/**
  * @brief  某一路发完需要多少毫秒（向上取整，0 个脉冲返回 0）
  */
uint32_t EncPlan_PhaseMs(uint16_t pulses, uint16_t freq_hz);

/**
  * @brief  整个脉冲串要发多久 = 最慢那一路 + 安全余量
  * @retval 毫秒；三路都不发时返回 0
  */
uint32_t EncPlan_DurationMs(const enc_plan_t *p);

/**
  * @brief  默认参数（PC 不指定时用）：A=1000@5kHz、B=2000@10kHz、Z=1@5kHz
  * @note   三路个数**故意各不相同** → DUT 报回来就能唯一确定是哪一路接错/没接。
  */
void     EncPlan_Default(enc_plan_t *p);

#endif /* __ENC_PLAN_H */
