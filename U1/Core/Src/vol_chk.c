/**
  ******************************************************************************
  * @file    vol_chk.c
  * @brief   ② 电压检测的判据层（治具 9 路 vs 被测板自己报的）
  *
  * 纯函数：不碰 HAL / 全局状态 → 宿主单测见 `test/vol_chk_test.c`。
  * 设计取舍见 `vol_chk.h` 的文件头。
  ******************************************************************************
  */
#include "vol_chk.h"

/* ------------------------------------------------------------------ */
static int32_t abs_i32(int32_t v)
{
    return (v < 0) ? -v : v;
}

/* ------------------------------------------------------------------ */
uint8_t VolChk_TakeJig(const meas_snapshot_t *snap, uint8_t ch, uint16_t *mv)
{
    if ((snap == 0) || (mv == 0) || (ch >= (uint8_t)MEAS_CH_COUNT)) {
        return 0u;
    }
    *mv = snap->ch[ch].mv;

    /* 快照整体有效 + 采集没溢出 + 这一路没贴边，才算可用 */
    if (((snap->flags & MEAS_SNAP_F_VALID) == 0u) ||
        ((snap->flags & MEAS_SNAP_F_OVERFLOW) != 0u)) {
        return 0u;
    }
    if ((snap->ch[ch].flags & MEAS_CHF_SAT) != 0u) {
        return 0u;
    }
    return 1u;
}

uint8_t VolChk_TakeDut(const dut_adc_t *a, uint8_t ch, uint16_t *mv)
{
    uint8_t i;

    if ((a == 0) || (mv == 0)) {
        return 0u;
    }
    for (i = 0u; i < a->n; i++) {
        if (a->item[i].ch == ch) {
            *mv = a->item[i].mv;
            return 1u;
        }
    }
    return 0u;                              /* DUT 没报这一路 */
}

uint16_t VolChk_Judge(const vol_chk_in_t *in, vol_chk_out_t *out)
{
    uint16_t f = 0u;
    uint32_t tol;
    int32_t  err;

    if (in == 0) {
        return 0u;
    }
    if (out != 0) {
        out->err_mv = 0; out->tol_mv = 0u; out->flags = 0u;
    }

    /* ── 数据不全 → 不判（不是失败）── */
    if ((in->jig_ok == 0u) || (in->dut_ok == 0u)) {
        f |= VOL_CHK_F_NODATA;
        if (out != 0) { out->flags = f; }
        return f;
    }

    /* 容差：按对侧（DUT）读数的百分比，配绝对下限。
     * 用 DUT 的读数而不是治具的，是为了让"两边都偏"和"一边偏"给出一致的容差。 */
    tol = ((uint32_t)in->dut_mv * VOL_CHK_TOL_PCT) / 100u;
    if (tol < VOL_CHK_TOL_MIN_MV) {
        tol = VOL_CHK_TOL_MIN_MV;
    }
    err = (int32_t)in->jig_mv - (int32_t)in->dut_mv;

    if ((uint32_t)abs_i32(err) > tol) {
        f |= VOL_CHK_F_DEVIATION;
    } else {
        f |= VOL_CHK_F_OK;
    }
    if (out != 0) {
        out->err_mv = err;
        out->tol_mv = tol;
        out->flags  = f;
    }
    return f;
}

uint16_t VolChk_All(const meas_snapshot_t *jig, const dut_adc_t *dut, uint16_t dut_mask,
                    vol_chk_out_t out[MEAS_CH_COUNT])
{
    uint16_t sum = VOL_CHK_SUM_DONE;
    uint8_t  i;

    for (i = 0u; i < (uint8_t)MEAS_CH_COUNT; i++) {
        vol_chk_in_t in;
        uint16_t     mv = 0u;
        uint16_t     f;

        in.jig_mv = 0u;
        in.dut_mv = 0u;
        in.jig_ok = VolChk_TakeJig(jig, i, &mv);
        in.jig_mv = mv;

        mv = 0u;
        /* DUT 在 INFO 里没声明支持这一路 → 不去问，直接算"没报" */
        if (((dut_mask & (uint16_t)(1u << i)) != 0u) &&
            (VolChk_TakeDut(dut, i, &mv) != 0u)) {
            in.dut_ok = 1u;
            in.dut_mv = mv;
        } else {
            in.dut_ok = 0u;
        }

        f = VolChk_Judge(&in, (out != 0) ? &out[i] : 0);

        if ((f & VOL_CHK_F_NODATA) != 0u) {
            sum |= VOL_CHK_SUM_ANY_NODATA;
        } else if (f != VOL_CHK_F_OK) {
            sum |= VOL_CHK_SUM_ANY_BAD;
        }
    }
    return sum;
}

const char *VolChk_FlagText(uint16_t flags)
{
    if (flags == 0u)                          { return "none"; }
    if (flags == VOL_CHK_F_OK)                { return "OK"; }
    if ((flags & VOL_CHK_F_NODATA) != 0u)     { return "NODATA(not judged)"; }
    if ((flags & VOL_CHK_F_DEVIATION) != 0u)  { return "DEVIATION"; }
    return "mixed";
}
