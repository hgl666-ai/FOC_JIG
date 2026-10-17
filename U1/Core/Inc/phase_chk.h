/**
  ******************************************************************************
  * @file    phase_chk.h
  * @brief   ④ 三相输出检测的**判据层**（走 ADC 相电压，不走定时器）
  *
  * 为什么走 ADC（硬件决定，见 docs/00 §3.5）
  * ----------------------------------------
  * `MA0_OUT/MB0_OUT/MC0_OUT` 在板上**只有 ADC 通路**（140k/20k 分压 → 跟随器 →
  * 100Ω+1nF+TVS → U1 的 IN8/IN7/IN6、U2 的 IN4/IN3/IN2），**没有任何定时器通路**。
  * 所以"捕获电机输出端的周期/占空比"做不到；能做的是读**相电压的平均值**。
  *
  * 一条判据覆盖两种用法
  * --------------------
  * 相输出是 PWM 时，平均值 = 占空比 × 母线电压；静态电平时，占空比就是 0 或 1000。
  * 所以统一成：
  *
  *        期望相电压 = expect_permille(‰) × 母线电压 / 1000
  *        实测相电压 = 该相 ADC 的平均值（V，已按分压比折回信号侧）
  *        判定       = |实测 − 期望| ≤ 容差
  *
  * 这条判据比"两端周期一致"信息量更大：它同时验证了**驱动级有没有正常工作**
  * 和**母线电压采样对不对**。③ 给的占空比也从这里进来做交叉核对。
  *
  * 边界与"不判"的三种情况（都不能算失败）
  * --------------------------------------
  *   1. **高阻/关断**（`PHASE_CHK_LVL_HIZ`）：节点浮空，读什么都没意义 → 不判；
  *   2. **母线太低**（< `PHASE_CHK_VBUS_MIN_MV`）：期望值太小、噪声占主导 → 不判；
  *   3. **电压/母线数据不可用**（快照无效或该通道贴边）→ 不判。
  * 这三种都置 `NODATA`/`HIZ` 旗标，**不计入失败**（与 measure/pulse 的"先报数后收紧"一致）。
  *
  * 只 U1 用（判定只在 U1 做，同 measure 的既有原则）→ **不做两片副本**。
  ******************************************************************************
  */
#ifndef __PHASE_CHK_H
#define __PHASE_CHK_H

#include <stdint.h>
#include "measure.h"
#include "pulse_cap.h"

/* ------------------------------------------------------------------ */
/* 1. 判据参数（要按上板实测收紧，见 docs/02 的待标定项）               */
/* ------------------------------------------------------------------ */
#ifndef PHASE_CHK_TOL_PCT
#define PHASE_CHK_TOL_PCT       5u      /* 电压容差 = 母线电压的 5% */
#endif
#ifndef PHASE_CHK_TOL_MIN_MV
#define PHASE_CHK_TOL_MIN_MV    300u    /* 容差绝对下限（占空比很小时用） */
#endif
#ifndef PHASE_CHK_DUTY_TOL_PM
#define PHASE_CHK_DUTY_TOL_PM   50u     /* 占空比容差 50‰ = 5%（③ 交叉核对用） */
#endif
#ifndef PHASE_CHK_VBUS_MIN_MV
#define PHASE_CHK_VBUS_MIN_MV   5000u   /* 母线低于 5V 不做判定 */
#endif

/* ------------------------------------------------------------------ */
/* 2. 期望电级的两种特殊值                                              */
/* ------------------------------------------------------------------ */
#define PHASE_CHK_LVL_LOW       0u      /* 静态低：期望 = 0 */
#define PHASE_CHK_LVL_HIGH      1000u   /* 静态高：期望 = 全母线 */
#define PHASE_CHK_LVL_HIZ       0xFFFFu /* 高阻/关断：**不判** */

/* ------------------------------------------------------------------ */
/* 3. 旗标                                                            */
/* ------------------------------------------------------------------ */
#define PHASE_CHK_F_OK          0x0001u  /* 这一相判定通过（或按规则不判） */
#define PHASE_CHK_F_HIZ         0x0002u  /* 该相高阻 → 不判 */
#define PHASE_CHK_F_NODATA      0x0004u  /* 数据不全（快照无效/贴边）→ 不判 */
#define PHASE_CHK_F_VBUS_LOW    0x0008u  /* 母线太低 → 不判 */
#define PHASE_CHK_F_NOSIGNAL    0x0010u  /* 期望有 PWM 输出但一个边沿都没测到 */
#define PHASE_CHK_F_DEVIATION   0x0020u  /* 相电压平均值偏离期望超容差 */
#define PHASE_CHK_F_DUTY_DIFF   0x0040u  /* 捕获到的占空比与命令不符（③ 的活，这里顺带核） */

/* PhaseChk_All 的汇总位 */
#define PHASE_CHK_SUM_DONE      0x0001u  /* 三相都跑过判据 */
#define PHASE_CHK_SUM_ANY_BAD   0x0002u  /* 至少一相真不合格（不含"不判"） */

/* ------------------------------------------------------------------ */
/* 4. 输入 / 输出                                                      */
/* ------------------------------------------------------------------ */
typedef struct {
    uint16_t expect_permille;  /* 期望占空比 0..1000，或 PHASE_CHK_LVL_HIZ */
    uint8_t  expect_pwm;       /* 1 = 期望的是 PWM（要核脉冲）；0 = 静态电平（不核脉冲） */
    uint32_t vbus_mv;          /* 母线电压（MEAS_CH_V24 的 mv） */
    uint32_t vph_mv;           /* 该相电压平均值（MEAS_CH_PH_x 的 mv，信号侧） */
    uint8_t  meas_ok;          /* 电压数据可用（快照 VALID 且该通道没贴边） */
    uint16_t duty_pm;          /* 脉冲捕获测到的占空比（0 = 没测到） */
    uint8_t  pulse_flags;      /* PULSE_F_* */
} phase_chk_in_t;

typedef struct {
    uint32_t expect_mv;        /* 期望的相电压平均值 */
    int32_t  err_mv;           /* vph_mv − expect_mv（有符号，便于看偏高还是偏低） */
    uint32_t tol_mv;           /* 本次实际用的容差 */
    uint16_t flags;            /* PHASE_CHK_F_* */
} phase_chk_out_t;

/* ------------------------------------------------------------------ */
/* 5. API                                                             */
/* ------------------------------------------------------------------ */
/**
  * @brief  从快照 + 脉冲结果组装一相的输入（母线、相电压、占空比、旗标）
  * @param  snap   本片电压快照（可传 0 = 没有电压数据）
  * @param  pulse  本片脉冲结果（可传 0 = 没有脉冲数据）
  * @param  ph     相位（PULSE_PH_A/B/C，与 MEAS_CH_PH_x 的偏移一致）
  * @param  expect_permille 期望占空比（或 PHASE_CHK_LVL_HIZ）
  * @param  expect_pwm      1 = PWM 模式，0 = 静态电平模式
  * @param  in     [out] 组装好的输入（其余字段由本函数填）
  * @note   ph 用 PULSE_PH_*，内部映射到 MEAS_CH_PH_A/B/C（两者都是 0/1/2 起、同序）
  */
void     PhaseChk_Make(const meas_snapshot_t *snap, const pulse_snapshot_t *pulse,
                       uint8_t ph, uint16_t expect_permille, uint8_t expect_pwm,
                       phase_chk_in_t *in);

/**
  * @brief  判一相
  * @retval 该相的 flags（PHASE_CHK_F_*）；参数为空返回 0
  */
uint16_t PhaseChk_Phase(const phase_chk_in_t *in, phase_chk_out_t *out);

/**
  * @brief  三相一起判 + 汇总
  * @param  out [out] 3 个结果（按 PULSE_PH_A/B/C）
  * @retval PHASE_CHK_SUM_*
  */
uint16_t PhaseChk_All(const phase_chk_in_t in[3], phase_chk_out_t out[3]);

/**
  * @brief  期望的相电压平均值（纯算术，单独暴露便于单测与复用）
  * @retval permille * vbus_mv / 1000（四舍五入）
  */
uint32_t PhaseChk_ExpectMv(uint16_t permille, uint32_t vbus_mv);

/**
  * @brief  人体可读的旗标文本（调试打印用；返回静态字符串，不要 free）
  */
const char *PhaseChk_FlagText(uint16_t flags);

#endif /* __PHASE_CHK_H */
