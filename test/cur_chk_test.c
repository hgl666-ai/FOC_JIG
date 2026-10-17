/*
 * ⑥ 相电流判据层宿主单测 (cur_chk.c)
 *
 * 被测的是真源码。重点覆盖三类：
 *   1) **"不判"（数据不全）不能算失败**；
 *   2) **有符号变化量** —— 被测板的电流放大器可能是中点偏置的（零电流输出 1.65V），
 *      拓扑还没确认（docs/15 待确认项 D4），所以两种方向都要对；
 *   3) **容差边界**（正好等于容差算过、多 1mV 算不过）。
 */
#include <stdio.h>
#include <string.h>

#include "cur_chk.h"

static int g_pass = 0;
static int g_fail = 0;

static void check(int cond, const char *name)
{
    if (cond) {
        g_pass++;
        printf("  [ok]   %s\n", name);
    } else {
        g_fail++;
        printf("  [FAIL] %s\n", name);
    }
}

static void mk_in(cur_chk_in_t *in, uint16_t ji, uint16_t jo, uint16_t di, uint16_t d_o)
{
    memset(in, 0, sizeof(*in));
    in->jig_idle_mv = ji; in->jig_on_mv = jo;
    in->dut_idle_mv = di; in->dut_on_mv = d_o;
    in->jig_ok = 1u;      in->dut_ok = 1u;
}

static void mk_snap(meas_snapshot_t *s, uint32_t cur1_mv, uint32_t cur2_mv)
{
    memset(s, 0, sizeof(*s));
    s->flags    = MEAS_SNAP_F_VALID;
    s->ch_count = MEAS_CH_COUNT;
    s->ch[MEAS_CH_CUR1].mv = (uint16_t)cur1_mv;
    s->ch[MEAS_CH_CUR2].mv = (uint16_t)cur2_mv;
}

int main(void)
{
    cur_chk_in_t  in;
    cur_chk_out_t o;

    printf("===== cur_chk 单测（⑥ 相电流判据）=====\n");

    /* ---------------- 1. 从快照取电流通道 ---------------- */
    printf("\n-- 1. CurChk_TakeJig --\n");
    {
        meas_snapshot_t s;
        uint16_t mv = 0u;

        mk_snap(&s, 1650u, 1655u);
        check(CurChk_TakeJig(&s, CUR_CHK_CH1, &mv) == 1u && mv == 1650u,
              "电流1 取到 1650mV（走的是 MEAS_CH_CUR1，不是别的通道）");
        check(CurChk_TakeJig(&s, CUR_CHK_CH2, &mv) == 1u && mv == 1655u, "电流2 取到 1655mV");

        s.flags = 0u;
        check(CurChk_TakeJig(&s, CUR_CHK_CH1, &mv) == 0u, "快照 VALID 没置 → 不可用");
        s.flags = MEAS_SNAP_F_VALID | MEAS_SNAP_F_OVERFLOW;
        check(CurChk_TakeJig(&s, CUR_CHK_CH1, &mv) == 0u, "采集溢出 → 不可用");
        s.flags = MEAS_SNAP_F_VALID;
        s.ch[MEAS_CH_CUR1].flags = MEAS_CHF_SAT;
        check(CurChk_TakeJig(&s, CUR_CHK_CH1, &mv) == 0u, "该通道贴边 → 不可用");
        check(CurChk_TakeJig(&s, CUR_CHK_CH2, &mv) == 1u, "只有电流1 贴边，电流2 仍可用");
        s.ch[MEAS_CH_CUR1].flags = 0u;

        check(CurChk_TakeJig(&s, 9u, &mv) == 0u, "通道号非法 → 不可用");
        check(CurChk_TakeJig(0, CUR_CHK_CH1, &mv) == 0u, "快照传 NULL → 不可用");
        check(CurChk_TakeJig(&s, CUR_CHK_CH1, 0) == 0u, "出参传 NULL → 不可用");
    }

    /* ---------------- 2. 数据不全 → 不判 ---------------- */
    printf("\n-- 2. 数据不全 → NODATA（**不算失败**）--\n");
    mk_in(&in, 1650u, 2000u, 1650u, 2000u);
    in.jig_ok = 0u;
    check(CurChk_Judge(&in, &o) == CUR_CHK_F_NODATA, "治具侧不可用 → NODATA");
    mk_in(&in, 1650u, 2000u, 1650u, 2000u);
    in.dut_ok = 0u;
    check(CurChk_Judge(&in, &o) == CUR_CHK_F_NODATA, "DUT 没报这一路 → NODATA");
    check((o.flags & CUR_CHK_F_DEVIATION) == 0u, "NODATA 时不会误报 DEVIATION");
    mk_in(&in, 1650u, 2000u, 1650u, 2000u);
    in.jig_ok = 0u; in.dut_ok = 0u;
    check(CurChk_Judge(&in, &o) == CUR_CHK_F_NODATA, "两边都不可用 → NODATA");

    /* ---------------- 3. 一致性 ---------------- */
    printf("\n-- 3. 一致性（中点偏置拓扑：1650 → 2000，变化量 +350）--\n");
    mk_in(&in, 1650u, 2000u, 1650u, 2000u);
    check(CurChk_Judge(&in, &o) == CUR_CHK_F_OK, "两边完全一致 → OK");
    check(o.dj == 350 && o.dd == 350 && o.err == 0, "三个数都对（dj/dd/err）");

    mk_in(&in, 1650u, 2000u, 1660u, 2015u);      /* dj=350, dd=355 */
    check(CurChk_Judge(&in, &o) == CUR_CHK_F_OK, "差 5mV（在容差内）→ OK");
    check(o.tol == CUR_CHK_TOL_MV, "容差取到绝对下限 30");

    mk_in(&in, 1650u, 2000u, 1660u, 2060u);      /* dj=350, dd=400 → err=−50 */
    check((CurChk_Judge(&in, &o) & CUR_CHK_F_DEVIATION) != 0u, "差 50mV → DEVIATION");
    check(o.err == -50, "err 带符号（治具偏低是负数）");

    /* 容差边界：dd=400 → tol = max(400*5/100, 30) = 30 */
    mk_in(&in, 1650u, 2080u, 1660u, 2060u);      /* dj=430, dd=400 → err=+30 */
    check(CurChk_Judge(&in, &o) == CUR_CHK_F_OK, "偏差正好等于容差（30）→ OK");
    mk_in(&in, 1650u, 2081u, 1660u, 2060u);      /* dj=431 → err=+31 */
    check((CurChk_Judge(&in, &o) & CUR_CHK_F_DEVIATION) != 0u, "偏差 31（超 1mV）→ DEVIATION");

    /* 大变化量时容差按百分比放大：dd=2000 → tol = 100 */
    mk_in(&in, 1650u, 3650u, 1650u, 3650u);      /* dj=dd=2000 */
    (void)CurChk_Judge(&in, &o);
    check(o.tol == 100u, "变化量 2000mV 时容差放大到 100mV（5%）");
    mk_in(&in, 1650u, 3750u, 1650u, 3650u);      /* err=100 */
    check(CurChk_Judge(&in, &o) == CUR_CHK_F_OK, "正好 100 → OK");
    mk_in(&in, 1650u, 3751u, 1650u, 3650u);
    check((CurChk_Judge(&in, &o) & CUR_CHK_F_DEVIATION) != 0u, "101 → DEVIATION");

    /* ---------------- 4. 有符号：零电流输出 0V 的拓扑 ---------------- */
    printf("\n-- 4. 负方向（零电流输出 0V 的拓扑：0 → 350，变化量也是 +350）--\n");
    mk_in(&in, 0u, 350u, 0u, 352u);
    check(CurChk_Judge(&in, &o) == CUR_CHK_F_OK, "0 起步：两边一致 → OK");
    check(o.dj == 350 && o.dd == 352, "变化量算对");

    printf("\n-- 4b. 反方向电流（变化量为负也要认）--\n");
    mk_in(&in, 1650u, 1300u, 1650u, 1305u);      /* dj=-350, dd=-345 */
    check(CurChk_Judge(&in, &o) == CUR_CHK_F_OK, "负方向、两边一致 → OK");
    check(o.dj == -350 && o.dd == -345, "负变化量算对（不是当成 0 或大正数）");
    mk_in(&in, 1650u, 1300u, 1650u, 1000u);      /* dj=-350, dd=-650 → err=+300 */
    check((CurChk_Judge(&in, &o) & CUR_CHK_F_DEVIATION) != 0u, "负方向差太多 → DEVIATION");

    /* ---------------- 5. 通流检查 ---------------- */
    printf("\n-- 5. 通流检查（电流没流起来）--\n");
    mk_in(&in, 1650u, 1650u, 1650u, 1650u);
    check((CurChk_Judge(&in, &o) & CUR_CHK_F_NOSIGNAL) != 0u,
          "两边都没动 → NOSIGNAL（真故障，不是 NODATA）");
    check((o.flags & CUR_CHK_F_OK) == 0u, "NOSIGNAL 时不能同时标 OK");

    mk_in(&in, 1650u, 1660u, 1650u, 1655u);
    check((CurChk_Judge(&in, &o) & CUR_CHK_F_NOSIGNAL) != 0u,
          "只动了 10mV（低于 50 的门限）→ 仍算没通上");

    mk_in(&in, 1650u, 1700u, 1650u, 1700u);
    check(CurChk_Judge(&in, &o) == CUR_CHK_F_OK, "动 50mV（正好到门限）→ 算通上了");

    /* 只有一边动了：不能算 NOSIGNAL，要算 DEVIATION */
    mk_in(&in, 1650u, 1650u, 1650u, 2000u);      /* dj=0, dd=350 */
    check((CurChk_Judge(&in, &o) & CUR_CHK_F_NOSIGNAL) == 0u,
          "只有 DUT 一边动了 → **不**算 NOSIGNAL");
    check((o.flags & CUR_CHK_F_DEVIATION) != 0u, "一边动一边不动 → DEVIATION（这才对）");

    /* ---------------- 6. 其它 ---------------- */
    printf("\n-- 6. 边界与文本 --\n");
    check(CurChk_Judge(0, &o) == 0u, "in 传 NULL → 返回 0");
    mk_in(&in, 1650u, 2000u, 1650u, 2000u);
    check(CurChk_Judge(&in, 0) == CUR_CHK_F_OK, "out 传 NULL 也能判");
    o.dj = 999; o.dd = 999; o.err = 999; o.tol = 999u; o.flags = 0xFFFFu;
    (void)CurChk_Judge(&in, &o);
    check(o.dj == 350 && o.tol == CUR_CHK_TOL_MV, "out 里的旧值被覆盖（不会读到上一轮的数）");
    check(strcmp(CurChk_FlagText(CUR_CHK_F_OK), "OK") == 0, "FlagText(OK)");
    check(strcmp(CurChk_FlagText(0u), "none") == 0, "FlagText(0)");
    check(strcmp(CurChk_FlagText(CUR_CHK_F_NOSIGNAL), "NOSIGNAL") == 0, "FlagText(NOSIGNAL)");
    check(strcmp(CurChk_FlagText(0x8000u), "mixed") == 0, "FlagText(未定义位)");
    check(CUR_CHK_CH_COUNT == 2u, "两路电流");

    printf("\n===== cur_chk: %d 项通过, %d 项失败 =====\n", g_pass, g_fail);
    return (g_fail == 0) ? 0 : 1;
}
