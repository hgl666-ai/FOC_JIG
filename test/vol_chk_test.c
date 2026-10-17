/*
 * ② 电压检测判据层宿主单测 (vol_chk.c)
 *
 * 被测的是真源码。重点：
 *   1) **"不判"（数据不全 / DUT 没声明 / 没报）不能算失败**；
 *   2) **容差按对侧读数算**（24V 那一路 5% = 1.2V，小信号靠 200mV 兜底）；
 *   3) **9 路一起判时的汇总**（DONE / ANY_BAD / ANY_NODATA 三个位互不误伤）；
 *   4) 从快照/DUT 应答取值的**通道对应关系**（别取错路）。
 */
#include <stdio.h>
#include <string.h>

#include "vol_chk.h"

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

static void mk_snap(meas_snapshot_t *s)
{
    int i;
    memset(s, 0, sizeof(*s));
    s->flags    = MEAS_SNAP_F_VALID;
    s->ch_count = MEAS_CH_COUNT;
    /* 全部给一个"和 DUT 一致"的值：4 路电源 + 3 相 + 2 电流 */
    for (i = 0; i < (int)MEAS_CH_COUNT; i++) {
        s->ch[i].mv    = (uint16_t)(1000u + (uint16_t)i);   /* 1000..1008 */
        s->ch[i].flags = 0u;
    }
}

static void mk_dut(dut_adc_t *a, uint16_t mask)
{
    uint8_t i;
    memset(a, 0, sizeof(*a));
    a->req_mask = mask;
    a->n = 0u;
    for (i = 0u; i < (uint8_t)MEAS_CH_COUNT; i++) {
        if ((mask & (uint16_t)(1u << i)) != 0u) {
            a->item[a->n].ch  = i;
            a->item[a->n].raw = (uint16_t)(100u + i);
            a->item[a->n].mv  = (uint16_t)(1000u + i);      /* 和治具一致 */
            a->n++;
        }
    }
}

int main(void)
{
    vol_chk_in_t  in;
    vol_chk_out_t o;

    printf("===== vol_chk 单测（② 电压检测判据）=====\n");

    /* ---------------- 1. 取通道 ---------------- */
    printf("\n-- 1. 从快照 / DUT 应答取值 --\n");
    {
        meas_snapshot_t s;
        dut_adc_t       a;
        uint16_t mv = 0u;

        mk_snap(&s);
        check(VolChk_TakeJig(&s, (uint8_t)MEAS_CH_V24, &mv) == 1u && mv == 1000u,
              "24V 通道取到 1000mV");
        check(VolChk_TakeJig(&s, (uint8_t)MEAS_CH_PH_C, &mv) == 1u && mv == 1006u,
              "C 相取到 1006mV（通道对应没串）");
        check(VolChk_TakeJig(&s, (uint8_t)MEAS_CH_CUR2, &mv) == 1u && mv == 1008u,
              "电流2 取到 1008mV");
        check(VolChk_TakeJig(&s, 9u, &mv) == 0u, "通道号超范围 → 不可用");
        check(VolChk_TakeJig(0, 0u, &mv) == 0u, "快照 NULL → 不可用");
        check(VolChk_TakeJig(&s, 0u, 0) == 0u, "出参 NULL → 不可用");

        s.flags = 0u;
        check(VolChk_TakeJig(&s, 0u, &mv) == 0u, "快照 VALID 没置 → 不可用");
        s.flags = MEAS_SNAP_F_VALID | MEAS_SNAP_F_OVERFLOW;
        check(VolChk_TakeJig(&s, 0u, &mv) == 0u, "采集溢出 → 不可用");
        s.flags = MEAS_SNAP_F_VALID;
        s.ch[MEAS_CH_V24].flags = MEAS_CHF_SAT;
        check(VolChk_TakeJig(&s, 0u, &mv) == 0u, "该通道贴边 → 不可用");
        check(VolChk_TakeJig(&s, 1u, &mv) == 1u, "只有 24V 贴边，别的通道仍可用");

        mk_dut(&a, 0x0007u);            /* 只报 ch0/ch1/ch2 */
        check(VolChk_TakeDut(&a, 0u, &mv) == 1u && mv == 1000u, "DUT 的 ch0 取到 1000mV");
        check(VolChk_TakeDut(&a, 2u, &mv) == 1u && mv == 1002u, "DUT 的 ch2 取到 1002mV");
        check(VolChk_TakeDut(&a, 3u, &mv) == 0u, "DUT 没报 ch3 → 取不到");
        check(VolChk_TakeDut(0, 0u, &mv) == 0u, "DUT 应答 NULL → 取不到");
    }

    /* ---------------- 2. 不判 ---------------- */
    printf("\n-- 2. 数据不全 → NODATA（**不算失败**）--\n");
    memset(&in, 0, sizeof(in));
    in.jig_mv = 1000u; in.dut_mv = 1000u; in.jig_ok = 0u; in.dut_ok = 1u;
    check(VolChk_Judge(&in, &o) == VOL_CHK_F_NODATA, "治具侧不可用 → NODATA");
    in.jig_ok = 1u; in.dut_ok = 0u;
    check(VolChk_Judge(&in, &o) == VOL_CHK_F_NODATA, "DUT 没报 → NODATA");
    in.jig_ok = 0u; in.dut_ok = 0u;
    check(VolChk_Judge(&in, &o) == VOL_CHK_F_NODATA, "两边都不可用 → NODATA");

    /* ---------------- 3. 一致性与容差 ---------------- */
    printf("\n-- 3. 一致性与容差 --\n");
    in.jig_ok = 1u; in.dut_ok = 1u;

    in.jig_mv = 1000u; in.dut_mv = 1000u;
    check(VolChk_Judge(&in, &o) == VOL_CHK_F_OK, "完全一致 → OK");
    check(o.err_mv == 0 && o.tol_mv == VOL_CHK_TOL_MIN_MV, "误差 0、容差取下限 200");

    /* 小信号：tol = max(1000*5/100, 200) = 200 */
    in.jig_mv = 1200u; in.dut_mv = 1000u;       /* err=+200 */
    check(VolChk_Judge(&in, &o) == VOL_CHK_F_OK, "差 200mV（正好等于容差）→ OK");
    in.jig_mv = 1201u;
    check((VolChk_Judge(&in, &o) & VOL_CHK_F_DEVIATION) != 0u, "差 201mV → DEVIATION");
    check(o.err_mv == 201, "err 带符号（治具偏高是正数）");
    in.jig_mv = 799u;
    check((VolChk_Judge(&in, &o) & VOL_CHK_F_DEVIATION) != 0u, "差 -201mV → DEVIATION");
    check(o.err_mv == -201, "err 偏低是负数");

    /* 大信号：24V 那一路 24000mV → tol = 1200 */
    in.jig_mv = 24000u; in.dut_mv = 24000u;
    (void)VolChk_Judge(&in, &o);
    check(o.tol_mv == 1200u, "24V 读数 24000mV → 容差 1200mV（5%）");
    in.jig_mv = 25200u;
    check(VolChk_Judge(&in, &o) == VOL_CHK_F_OK, "差 1200mV（正好）→ OK");
    in.jig_mv = 25201u;
    check((VolChk_Judge(&in, &o) & VOL_CHK_F_DEVIATION) != 0u, "差 1201mV → DEVIATION");

    /* 容差按 **DUT 的读数** 算：dut=10000 → tol=500 */
    in.jig_mv = 10500u; in.dut_mv = 10000u;
    (void)VolChk_Judge(&in, &o);
    check(o.tol_mv == 500u, "对侧读数 10000mV → 容差 500mV");

    /* ---------------- 4. 9 路汇总 ---------------- */
    printf("\n-- 4. 9 路一起判 --\n");
    {
        meas_snapshot_t s;
        dut_adc_t       a;
        vol_chk_out_t   outs[MEAS_CH_COUNT];

        /* 全好 */
        mk_snap(&s); mk_dut(&a, 0x01FFu);
        check(VolChk_All(&s, &a, 0x01FFu, outs) == VOL_CHK_SUM_DONE,
              "9 路全一致 → DONE，无 ANY_BAD / ANY_NODATA");

        /* 一路超差 */
        mk_snap(&s); mk_dut(&a, 0x01FFu);
        s.ch[MEAS_CH_V24].mv = 30000u;              /* DUT 报 1000，差 29000 */
        {
            uint16_t sum = VolChk_All(&s, &a, 0x01FFu, outs);
            check((sum & VOL_CHK_SUM_ANY_BAD) != 0u, "一路超差 → ANY_BAD");
            check((sum & VOL_CHK_SUM_ANY_NODATA) == 0u, "没有「没判」的路时不置 ANY_NODATA");
            check((outs[MEAS_CH_V24].flags & VOL_CHK_F_DEVIATION) != 0u,
                  "超差的那一路旗标落在正确的下标上");
            check((outs[MEAS_CH_V10].flags & VOL_CHK_F_OK) != 0u, "别的路仍然是 OK");
        }

        /* DUT 只声明支持一部分 → 没声明的算"没判"，不算失败 */
        mk_snap(&s); mk_dut(&a, 0x0007u);           /* 只报 ch0/1/2 */
        {
            uint16_t sum = VolChk_All(&s, &a, 0x0007u, outs);
            check((sum & VOL_CHK_SUM_ANY_BAD) == 0u, "DUT 只支持 3 路 → **不算失败**");
            check((sum & VOL_CHK_SUM_ANY_NODATA) != 0u, "其余 6 路算 NODATA");
            check((outs[MEAS_CH_V24].flags & VOL_CHK_F_OK) != 0u, "声明支持的 ch0 判成 OK");
            check((outs[MEAS_CH_PH_C].flags & VOL_CHK_F_NODATA) != 0u, "没声明的 ch6 是 NODATA");
        }

        /* DUT 能力位图里声明了、但应答里没报 → 也算 NODATA（不能当成 0 去比） */
        mk_snap(&s); mk_dut(&a, 0x0003u);           /* 只报 ch0/1 */
        {
            uint16_t sum = VolChk_All(&s, &a, 0x01FFu, outs);   /* 声明支持 9 路 */
            check((sum & VOL_CHK_SUM_ANY_BAD) == 0u, "声明了但没报的路**不算失败**");
            check((outs[MEAS_CH_PH_A].flags & VOL_CHK_F_NODATA) != 0u, "ch4 没报 → NODATA");
            check(outs[MEAS_CH_PH_A].err_mv == 0, "NODATA 时误差是 0（没拿 0 当读数去比）");
        }

        /* 治具侧整片不可用 */
        mk_snap(&s); mk_dut(&a, 0x01FFu);
        s.flags = 0u;
        {
            uint16_t sum = VolChk_All(&s, &a, 0x01FFu, outs);
            check((sum & VOL_CHK_SUM_ANY_BAD) == 0u, "治具快照无效 → 9 路全不判，不算失败");
            check((sum & VOL_CHK_SUM_ANY_NODATA) != 0u, "并且汇总里置了 ANY_NODATA");
        }

        /* out 传 NULL 也能只求汇总 */
        mk_snap(&s); mk_dut(&a, 0x01FFu);
        check(VolChk_All(&s, &a, 0x01FFu, 0) == VOL_CHK_SUM_DONE, "out 传 NULL → 只求汇总");
        /* 快照传 NULL 也不炸 */
        check((VolChk_All(0, &a, 0x01FFu, outs) & VOL_CHK_SUM_ANY_NODATA) != 0u,
              "快照传 NULL → 全 NODATA（不崩）");
    }

    /* ---------------- 5. 其它 ---------------- */
    printf("\n-- 5. 边界与文本 --\n");
    check(VolChk_Judge(0, &o) == 0u, "in 传 NULL → 返回 0");
    in.jig_ok = 1u; in.dut_ok = 1u; in.jig_mv = 1000u; in.dut_mv = 1000u;
    check(VolChk_Judge(&in, 0) == VOL_CHK_F_OK, "out 传 NULL 也能判");
    o.err_mv = 999; o.tol_mv = 999u; o.flags = 0xFFFFu;
    (void)VolChk_Judge(&in, &o);
    check(o.err_mv == 0 && o.tol_mv == VOL_CHK_TOL_MIN_MV, "out 旧值被覆盖");
    check(strcmp(VolChk_FlagText(VOL_CHK_F_OK), "OK") == 0, "FlagText(OK)");
    check(strcmp(VolChk_FlagText(0u), "none") == 0, "FlagText(0)");
    check(strcmp(VolChk_FlagText(VOL_CHK_F_DEVIATION), "DEVIATION") == 0, "FlagText(DEVIATION)");
    check(strcmp(VolChk_FlagText(0x8000u), "mixed") == 0, "FlagText(未定义位)");

    printf("\n===== vol_chk: %d 项通过, %d 项失败 =====\n", g_pass, g_fail);
    return (g_fail == 0) ? 0 : 1;
}
