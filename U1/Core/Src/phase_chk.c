/**
  ******************************************************************************
  * @file    phase_chk.c
  * @brief   ④ 三相输出检测的判据层（走 ADC 相电压）
  *
  * 纯函数：不碰 HAL、不碰全局状态 → 宿主单测见 `test/phase_chk_test.c`。
  * 设计取舍与边界见 `phase_chk.h` 的文件头。
  ******************************************************************************
  */
#include "phase_chk.h"

/* ------------------------------------------------------------------ */
static uint32_t abs_i32(int32_t v)
{
    return (uint32_t)((v < 0) ? -v : v);
}

/* ph 用 PULSE_PH_*（0/1/2），与 MEAS_CH_PH_A/B/C 的偏移一致 —— 这里显式映射，
 * 不靠"碰巧相等"，将来任何一边改了序号都会在这里露出来。 */
static uint8_t ph_to_meas_ch(uint8_t ph)
{
    switch (ph) {
    case PULSE_PH_A: return (uint8_t)MEAS_CH_PH_A;
    case PULSE_PH_B: return (uint8_t)MEAS_CH_PH_B;
    case PULSE_PH_C: return (uint8_t)MEAS_CH_PH_C;
    default:         return 0xFFu;
    }
}

/* ------------------------------------------------------------------ */
uint32_t PhaseChk_ExpectMv(uint16_t permille, uint32_t vbus_mv)
{
    if (permille > 1000u) {
        permille = 1000u;
    }
    /* 先乘后除；1000 × 26 050 000 µV 也只在 u32 里，安全。
     * +500 是四舍五入（不要截断：占空比 500、母线 24V 时截断会少 12 mV）。 */
    return ((uint32_t)permille * vbus_mv + 500u) / 1000u;
}

void PhaseChk_Make(const meas_snapshot_t *snap, const pulse_snapshot_t *pulse,
                   uint8_t ph, uint16_t expect_permille, uint8_t expect_pwm,
                   phase_chk_in_t *in)
{
    uint8_t mc;

    if (in == 0) {
        return;
    }
    in->expect_permille = expect_permille;
    in->expect_pwm      = (expect_pwm != 0u) ? 1u : 0u;
    in->vbus_mv         = 0u;
    in->vph_mv          = 0u;
    in->meas_ok         = 0u;
    in->duty_pm         = 0u;
    in->pulse_flags     = 0u;

    mc = ph_to_meas_ch(ph);
    if (mc == 0xFFu) {
        return;                             /* 相位号不对：meas_ok 保持 0 → 判 NODATA */
    }

    if (snap != 0) {
        /* 快照整体有效 + 母线/相电压两路都没贴边，才算"数据可用" */
        uint8_t snap_ok = ((snap->flags & MEAS_SNAP_F_VALID) != 0u) &&
                          ((snap->flags & MEAS_SNAP_F_OVERFLOW) == 0u);
        uint8_t bus_ok  = ((snap->ch[MEAS_CH_V24].flags & MEAS_CHF_SAT) == 0u);
        uint8_t ph_ok   = ((snap->ch[mc].flags & MEAS_CHF_SAT) == 0u);

        in->vbus_mv = snap->ch[MEAS_CH_V24].mv;
        in->vph_mv  = snap->ch[mc].mv;
        in->meas_ok = (uint8_t)((snap_ok && bus_ok && ph_ok) ? 1u : 0u);
    }

    if (pulse != 0) {
        in->duty_pm     = pulse->ph[ph].duty_permille;
        in->pulse_flags = pulse->ph[ph].flags;
    }
}

uint16_t PhaseChk_Phase(const phase_chk_in_t *in, phase_chk_out_t *out)
{
    uint16_t f = 0u;
    int32_t  err;
    uint32_t tol;

    if (in == 0) {
        return 0u;
    }
    if (out != 0) {
        out->expect_mv = 0u;
        out->err_mv    = 0;
        out->tol_mv    = 0u;
        out->flags     = 0u;
    }

    /* ── 1. 高阻：节点浮空，读什么都没意义 → 不判（不算失败） ── */
    if (in->expect_permille == PHASE_CHK_LVL_HIZ) {
        f |= PHASE_CHK_F_HIZ | PHASE_CHK_F_OK;
        if (out != 0) { out->flags = f; }
        return f;
    }

    /* ── 2. 母线太低：期望值本身太小、噪声占主导 → 不判 ── */
    if (in->vbus_mv < PHASE_CHK_VBUS_MIN_MV) {
        f |= PHASE_CHK_F_VBUS_LOW | PHASE_CHK_F_NODATA;
        if (out != 0) { out->flags = f; }
        return f;
    }

    /* ── 3. 电压数据不可用 → 不判 ── */
    if (in->meas_ok == 0u) {
        f |= PHASE_CHK_F_NODATA;
        if (out != 0) { out->flags = f; }
        return f;
    }

    /* ── 4. 主判据：相电压平均值 vs 期望值 ── */
    {
        uint32_t expect = PhaseChk_ExpectMv(in->expect_permille, in->vbus_mv);

        tol = (in->vbus_mv / 100u) * PHASE_CHK_TOL_PCT;
        if (tol < PHASE_CHK_TOL_MIN_MV) {
            tol = PHASE_CHK_TOL_MIN_MV;
        }
        err = (int32_t)in->vph_mv - (int32_t)expect;

        if (abs_i32(err) > tol) {
            f |= PHASE_CHK_F_DEVIATION;
        }
        if (out != 0) {
            out->expect_mv = expect;
            out->err_mv    = err;
            out->tol_mv    = tol;
        }
    }

    /* ── 5. 脉冲交叉核对（只在 PWM 模式做；静态电平没有边沿，查了必误报） ── */
    if (in->expect_pwm != 0u) {
        uint8_t no_sig = (uint8_t)((in->pulse_flags & PULSE_F_NOSIGNAL) != 0u);
        uint8_t valid  = (uint8_t)((in->pulse_flags & PULSE_F_VALID) != 0u);

        /* 命令它输出、却一个边沿都没测到 → 驱动级/接线问题 */
        if ((in->expect_permille > 0u) && (no_sig != 0u)) {
            f |= PHASE_CHK_F_NOSIGNAL;
        }
        /* 测到了，但占空比与命令不符（③ 的活，这里顺带核） */
        if ((valid != 0u) && (no_sig == 0u)) {
            uint32_t d = abs_i32((int32_t)in->duty_pm - (int32_t)in->expect_permille);
            if (d > PHASE_CHK_DUTY_TOL_PM) {
                f |= PHASE_CHK_F_DUTY_DIFF;
            }
        }
    }

    if (f == 0u) {
        f = PHASE_CHK_F_OK;
    }
    if (out != 0) {
        out->flags = f;
    }
    return f;
}

uint16_t PhaseChk_All(const phase_chk_in_t in[3], phase_chk_out_t out[3])
{
    uint16_t sum = PHASE_CHK_SUM_DONE;
    uint8_t  i;

    if (in == 0) {
        return 0u;
    }
    for (i = 0u; i < 3u; i++) {
        uint16_t f = PhaseChk_Phase(&in[i], (out != 0) ? &out[i] : 0);
        /* 「不判」的三种旗标不算失败；其余任何旗标都算 */
        if (f == 0u) {
            continue;
        }
        if ((f & (PHASE_CHK_F_HIZ | PHASE_CHK_F_NODATA)) != 0u) {
            continue;
        }
        if (f != PHASE_CHK_F_OK) {
            sum |= PHASE_CHK_SUM_ANY_BAD;
        }
    }
    return sum;
}

const char *PhaseChk_FlagText(uint16_t flags)
{
    if (flags == 0u)                 { return "none"; }
    if (flags == PHASE_CHK_F_OK)     { return "OK"; }
    if ((flags & PHASE_CHK_F_HIZ) != 0u)       { return "HIZ(not judged)"; }
    if ((flags & PHASE_CHK_F_VBUS_LOW) != 0u)  { return "VBUS_LOW(not judged)"; }
    if ((flags & PHASE_CHK_F_NODATA) != 0u)    { return "NODATA(not judged)"; }
    if ((flags & PHASE_CHK_F_NOSIGNAL) != 0u)  { return "NOSIGNAL"; }
    if ((flags & PHASE_CHK_F_DEVIATION) != 0u) { return "DEVIATION"; }
    if ((flags & PHASE_CHK_F_DUTY_DIFF) != 0u) { return "DUTY_DIFF"; }
    return "mixed";
}
