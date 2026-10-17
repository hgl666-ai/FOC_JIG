/**
  ******************************************************************************
  * @file    cur_chk.c
  * @brief   ⑥ 相电流检测的判据层
  *
  * 纯函数：不碰 HAL / 全局状态 → 宿主单测见 `test/cur_chk_test.c`。
  * 设计取舍（为什么比差值、三个判据分别管什么）见 `cur_chk.h` 的文件头。
  ******************************************************************************
  */
#include "cur_chk.h"

/* ------------------------------------------------------------------ */
static int32_t abs_i32(int32_t v)
{
    return (v < 0) ? -v : v;
}

static uint8_t ph_to_meas_ch(uint8_t which)
{
    switch (which) {
    case CUR_CHK_CH1: return (uint8_t)MEAS_CH_CUR1;
    case CUR_CHK_CH2: return (uint8_t)MEAS_CH_CUR2;
    default:          return 0xFFu;
    }
}

/* ------------------------------------------------------------------ */
uint8_t CurChk_TakeJig(const meas_snapshot_t *snap, uint8_t which, uint16_t *mv)
{
    uint8_t mc;

    if ((snap == 0) || (mv == 0)) {
        return 0u;
    }
    mc = ph_to_meas_ch(which);
    if (mc == 0xFFu) {
        return 0u;
    }
    *mv = snap->ch[mc].mv;

    /* 快照整体有效 + 采集没溢出 + 这一路没贴边，才算"可用" */
    if (((snap->flags & MEAS_SNAP_F_VALID) == 0u) ||
        ((snap->flags & MEAS_SNAP_F_OVERFLOW) != 0u)) {
        return 0u;
    }
    if ((snap->ch[mc].flags & MEAS_CHF_SAT) != 0u) {
        return 0u;
    }
    return 1u;
}

uint16_t CurChk_Judge(const cur_chk_in_t *in, cur_chk_out_t *out)
{
    uint16_t f = 0u;
    uint32_t tol;
    int32_t  ad, aerr;

    if (in == 0) {
        return 0u;
    }
    if (out != 0) {
        out->dj = 0; out->dd = 0; out->err = 0; out->tol = 0u; out->flags = 0u;
    }

    /* ── 数据不全 → 不判（不是失败）── */
    if ((in->jig_ok == 0u) || (in->dut_ok == 0u)) {
        f |= CUR_CHK_F_NODATA;
        if (out != 0) { out->flags = f; }
        return f;
    }

    /* ── 变化量（有符号）── */
    {
        int32_t dj = (int32_t)in->jig_on_mv - (int32_t)in->jig_idle_mv;
        int32_t dd = (int32_t)in->dut_on_mv - (int32_t)in->dut_idle_mv;
        if (out != 0) {
            out->dj  = dj;
            out->dd  = dd;
            out->err = dj - dd;
        }
        ad = abs_i32(dd);

        /* 容差：按对侧变化量的百分比，配一个绝对下限 */
        tol = ((uint32_t)ad * CUR_CHK_TOL_PCT) / 100u;
        if (tol < CUR_CHK_TOL_MV) {
            tol = CUR_CHK_TOL_MV;
        }
        if (out != 0) { out->tol = tol; }

        /* ── 判据① 通流检查：两边都没怎么动 → 电流根本没流起来 ── */
        if ((abs_i32(dj) < (int32_t)CUR_CHK_MIN_DELTA_MV) &&
            (ad < (int32_t)CUR_CHK_MIN_DELTA_MV)) {
            f |= CUR_CHK_F_NOSIGNAL;
        } else {
            /* ── 判据② 一致性：两边的变化量之差 ── */
            aerr = abs_i32(dj - dd);
            if ((uint32_t)aerr > tol) {
                f |= CUR_CHK_F_DEVIATION;
            }
        }
    }

    if (f == 0u) {
        f = CUR_CHK_F_OK;
    }
    if (out != 0) { out->flags = f; }
    return f;
}

const char *CurChk_FlagText(uint16_t flags)
{
    if (flags == 0u)                        { return "none"; }
    if (flags == CUR_CHK_F_OK)              { return "OK"; }
    if ((flags & CUR_CHK_F_NODATA) != 0u)   { return "NODATA(not judged)"; }
    if ((flags & CUR_CHK_F_NOSIGNAL) != 0u) { return "NOSIGNAL"; }
    if ((flags & CUR_CHK_F_DEVIATION) != 0u){ return "DEVIATION"; }
    return "mixed";
}
