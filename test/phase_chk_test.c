/*
 * ④ 三相输出检测判据层宿主单测 (phase_chk.c)
 *
 * 被测的是真源码。重点覆盖三类"容易写错"的地方：
 *   1) **"不判"不能被当成"失败"**（高阻 / 母线太低 / 数据不全）；
 *   2) **静态电平模式不能查脉冲**（静态高电平没有边沿，查了必然误报 NOSIGNAL）；
 *   3) **容差边界**（正好等于容差算过、多 1 mV 算不过）。
 * 期望值来自 docs/12 §1 的判据：V_平均 ≈ 占空比 × 母线电压。
 */
#include <stdio.h>
#include <string.h>

#include "phase_chk.h"

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

/* 造一个"一切正常"的快照：母线 24000mV，三相都 12000mV，无贴边 */
static void mk_snap(meas_snapshot_t *s, uint32_t vbus_mv,
                    uint32_t va, uint32_t vb, uint32_t vc)
{
    memset(s, 0, sizeof(*s));
    s->flags     = MEAS_SNAP_F_VALID;
    s->ch_count  = MEAS_CH_COUNT;
    s->ch[MEAS_CH_V24].mv   = (uint16_t)vbus_mv;
    s->ch[MEAS_CH_PH_A].mv  = (uint16_t)va;
    s->ch[MEAS_CH_PH_B].mv  = (uint16_t)vb;
    s->ch[MEAS_CH_PH_C].mv  = (uint16_t)vc;
}

static void mk_in(phase_chk_in_t *in, uint16_t expect_pm, uint32_t vbus_mv, uint32_t vph_mv)
{
    memset(in, 0, sizeof(*in));
    in->expect_permille = expect_pm;
    in->expect_pwm      = 1u;
    in->vbus_mv         = vbus_mv;
    in->vph_mv          = vph_mv;
    in->meas_ok         = 1u;
    in->duty_pm         = expect_pm;
    in->pulse_flags     = PULSE_F_VALID;
}

int main(void)
{
    phase_chk_in_t  in;
    phase_chk_out_t o;

    printf("===== phase_chk 单测（④ 三相输出检测判据）=====\n");

    /* ---------------- 1. 期望值算术 ---------------- */
    printf("\n-- 1. PhaseChk_ExpectMv --\n");
    check(PhaseChk_ExpectMv(0u, 24000u)    == 0u,     "0‰ → 0");
    check(PhaseChk_ExpectMv(1000u, 24000u) == 24000u, "1000‰ → 全母线");
    check(PhaseChk_ExpectMv(500u, 24000u)  == 12000u, "500‰、母线 24V → 12V");
    check(PhaseChk_ExpectMv(500u, 24001u)  == 12001u, "500‰、母线 24001 → 12001（四舍五入）");
    check(PhaseChk_ExpectMv(1000u, 1u)     == 1u,     "极小母线也不炸");
    check(PhaseChk_ExpectMv(1001u, 1000u)  == 1000u,  ">1000‰ 被夹到 1000");
    check(PhaseChk_ExpectMv(500u, 26050u)  == 13025u, "满量程母线 26050 → 13025");

    /* ---------------- 2. "不判"的三种情况都不能算失败 ---------------- */
    printf("\n-- 2. 三种「不判」（都不能算失败）--\n");
    mk_in(&in, PHASE_CHK_LVL_HIZ, 24000u, 0u);
    check(PhaseChk_Phase(&in, &o) == (PHASE_CHK_F_HIZ | PHASE_CHK_F_OK),
          "高阻 → HIZ|OK（不是失败）");

    mk_in(&in, 500u, 4999u, 0u);
    check((PhaseChk_Phase(&in, &o) & PHASE_CHK_F_VBUS_LOW) != 0u,
          "母线 4999mV < 5000 → VBUS_LOW");
    check((o.flags & PHASE_CHK_F_NODATA) != 0u, "母线太低同时置 NODATA（表示没判）");
    check((o.flags & PHASE_CHK_F_DEVIATION) == 0u, "母线太低不会误报 DEVIATION");

    mk_in(&in, 500u, 24000u, 12000u);
    in.meas_ok = 0u;
    check(PhaseChk_Phase(&in, &o) == PHASE_CHK_F_NODATA, "数据不可用 → NODATA（不是失败）");

    /* ---------------- 3. 主判据与容差边界 ---------------- */
    printf("\n-- 3. 主判据（母线 24000、期望 500‰ → 期望 12000，容差 1200）--\n");
    mk_in(&in, 500u, 24000u, 12000u);
    check(PhaseChk_Phase(&in, &o) == PHASE_CHK_F_OK, "完全一致 → OK");
    check(o.expect_mv == 12000u && o.err_mv == 0 && o.tol_mv == 1200u,
          "期望/误差/容差三个数都对");

    mk_in(&in, 500u, 24000u, 10800u);
    check(PhaseChk_Phase(&in, &o) == PHASE_CHK_F_OK, "低 1200mV（正好等于容差）→ OK");

    mk_in(&in, 500u, 24000u, 10799u);
    check((PhaseChk_Phase(&in, &o) & PHASE_CHK_F_DEVIATION) != 0u,
          "低 1201mV（超出容差 1mV）→ DEVIATION");

    mk_in(&in, 500u, 24000u, 13200u);
    check(PhaseChk_Phase(&in, &o) == PHASE_CHK_F_OK, "高 1200mV → OK（对称）");
    mk_in(&in, 500u, 24000u, 13201u);
    check((PhaseChk_Phase(&in, &o) & PHASE_CHK_F_DEVIATION) != 0u, "高 1201mV → DEVIATION");
    check(o.err_mv == 1201, "err_mv 带符号（偏高是正数）");
    mk_in(&in, 500u, 24000u, 10700u);
    (void)PhaseChk_Phase(&in, &o);
    check(o.err_mv == -1300, "err_mv 偏低是负数");

    /* 容差绝对下限 */
    printf("\n-- 3b. 容差绝对下限 300mV（母线 5000 时 5%% 只有 250）--\n");
    mk_in(&in, 500u, 5000u, 2750u);
    check(PhaseChk_Phase(&in, &o) == PHASE_CHK_F_OK, "母线 5V、误差 250mV → OK（用 300 的底）");
    check(o.tol_mv == 300u, "容差取到下限 300");
    mk_in(&in, 500u, 5000u, 2801u);
    check((PhaseChk_Phase(&in, &o) & PHASE_CHK_F_DEVIATION) != 0u, "误差 301mV → DEVIATION");
    mk_in(&in, 500u, 5000u, 2500u);
    (void)PhaseChk_Phase(&in, &o);
    check(o.tol_mv == 300u, "母线 5V 时容差用下限 300（再确认一次）");

    /* out 里的旧值必须被清掉，否则调用方会读到上一轮的数 */
    mk_in(&in, PHASE_CHK_LVL_HIZ, 24000u, 0u);
    o.expect_mv = 0xDEADu; o.err_mv = -12345; o.tol_mv = 0xBEEFu; o.flags = 0xFFFFu;
    (void)PhaseChk_Phase(&in, &o);
    check(o.expect_mv == 0u && o.err_mv == 0 && o.tol_mv == 0u,
          "out 里的旧值被清掉（不会读到上一轮的数）");
    check(o.flags == (PHASE_CHK_F_HIZ | PHASE_CHK_F_OK), "out.flags 被覆盖成本次结果");

    /* ---------------- 4. 静态电平模式不能查脉冲（关键） ---------------- */
    printf("\n-- 4. 静态电平模式（expect_pwm=0）--\n");
    mk_in(&in, PHASE_CHK_LVL_HIGH, 24000u, 24000u);
    in.expect_pwm  = 0u;                    /* 静态高：没有 PWM */
    in.pulse_flags = PULSE_F_NOSIGNAL;      /* 捕获窗口里当然一个边沿都没有 */
    in.duty_pm     = 0u;
    check(PhaseChk_Phase(&in, &o) == PHASE_CHK_F_OK,
          "静态高电平 + 捕获无信号 → OK（**不能**误报 NOSIGNAL）");

    mk_in(&in, PHASE_CHK_LVL_LOW, 24000u, 200u);
    in.expect_pwm  = 0u;
    in.pulse_flags = PULSE_F_NOSIGNAL;
    check(PhaseChk_Phase(&in, &o) == PHASE_CHK_F_OK, "静态低电平 + 捕获无信号 → OK");

    mk_in(&in, PHASE_CHK_LVL_HIGH, 24000u, 200u);
    in.expect_pwm  = 0u;
    in.pulse_flags = PULSE_F_NOSIGNAL;
    check((PhaseChk_Phase(&in, &o) & PHASE_CHK_F_DEVIATION) != 0u,
          "静态高电平却测到 200mV → DEVIATION（这才是真故障）");

    /* ---------------- 5. PWM 模式的脉冲交叉核对 ---------------- */
    printf("\n-- 5. PWM 模式：脉冲交叉核对 --\n");
    mk_in(&in, 500u, 24000u, 12000u);
    in.pulse_flags = PULSE_F_NOSIGNAL;
    check((PhaseChk_Phase(&in, &o) & PHASE_CHK_F_NOSIGNAL) != 0u,
          "命令输出 500‰ 却测不到边沿 → NOSIGNAL");

    mk_in(&in, 0u, 24000u, 0u);
    in.pulse_flags = PULSE_F_NOSIGNAL;
    in.duty_pm     = 0u;
    check(PhaseChk_Phase(&in, &o) == PHASE_CHK_F_OK,
          "命令 0‰（本来就该没输出）+ 没边沿 → OK（不该报 NOSIGNAL）");

    mk_in(&in, 500u, 24000u, 12000u);
    in.duty_pm     = 545u;                  /* 差 45‰，在 50 的容差内 */
    check((PhaseChk_Phase(&in, &o) & PHASE_CHK_F_DUTY_DIFF) == 0u, "占空比差 45‰ → 不报");
    in.duty_pm = 551u;                      /* 差 51‰，超了 */
    check((PhaseChk_Phase(&in, &o) & PHASE_CHK_F_DUTY_DIFF) != 0u, "占空比差 51‰ → DUTY_DIFF");
    in.duty_pm = 449u;
    check((PhaseChk_Phase(&in, &o) & PHASE_CHK_F_DUTY_DIFF) != 0u, "反向差 51‰ → 也报");
    /* VALID 没置位时不该拿 duty 去比 */
    in.duty_pm = 0u;
    in.pulse_flags = PULSE_F_RANGE;         /* 有数据但没 VALID */
    check((PhaseChk_Phase(&in, &o) & PHASE_CHK_F_DUTY_DIFF) == 0u,
          "没有 VALID 时不拿占空比做比对");

    /* 电压偏差 + 占空比不符可以同时报 */
    mk_in(&in, 500u, 24000u, 9000u);
    in.duty_pm = 700u;
    {
        uint16_t f = PhaseChk_Phase(&in, &o);
        check(((f & PHASE_CHK_F_DEVIATION) != 0u) && ((f & PHASE_CHK_F_DUTY_DIFF) != 0u),
              "两个问题同时存在 → 两个旗标都置");
    }

    /* ---------------- 6. PhaseChk_Make 组装 ---------------- */
    printf("\n-- 6. PhaseChk_Make 组装 --\n");
    {
        meas_snapshot_t  snap;
        pulse_snapshot_t pul;

        mk_snap(&snap, 24000u, 12000u, 11000u, 13000u);
        memset(&pul, 0, sizeof(pul));
        pul.phase_count = PULSE_PH_COUNT;
        pul.flags       = 1u;
        pul.ph[PULSE_PH_A].duty_permille = 500u;
        pul.ph[PULSE_PH_A].flags         = PULSE_F_VALID;
        pul.ph[PULSE_PH_B].duty_permille = 460u;
        pul.ph[PULSE_PH_B].flags         = PULSE_F_VALID;
        pul.ph[PULSE_PH_C].flags         = PULSE_F_NOSIGNAL;

        PhaseChk_Make(&snap, &pul, PULSE_PH_A, 500u, 1u, &in);
        check(in.vph_mv == 12000u && in.vbus_mv == 24000u && in.meas_ok == 1u,
              "A 相取到对的通道（12000）、母线 24000、数据可用");
        check(in.duty_pm == 500u && in.pulse_flags == PULSE_F_VALID, "A 相的脉冲数据也带过来了");

        PhaseChk_Make(&snap, &pul, PULSE_PH_B, 500u, 1u, &in);
        check(in.vph_mv == 11000u, "B 相取到 11000（通道映射正确，不是都取 A）");

        PhaseChk_Make(&snap, &pul, PULSE_PH_C, 500u, 1u, &in);
        check(in.vph_mv == 13000u, "C 相取到 13000");
        check((in.pulse_flags & PULSE_F_NOSIGNAL) != 0u, "C 相带出 NOSIGNAL");

        /* 快照无效 → meas_ok=0 */
        snap.flags = 0u;
        PhaseChk_Make(&snap, &pul, PULSE_PH_A, 500u, 1u, &in);
        check(in.meas_ok == 0u, "快照 VALID 没置 → meas_ok=0");
        snap.flags = MEAS_SNAP_F_VALID | MEAS_SNAP_F_OVERFLOW;
        PhaseChk_Make(&snap, &pul, PULSE_PH_A, 500u, 1u, &in);
        check(in.meas_ok == 0u, "采集溢出（OVERFLOW）→ meas_ok=0");
        snap.flags = MEAS_SNAP_F_VALID;

        /* 该相贴边 → meas_ok=0 */
        snap.ch[MEAS_CH_PH_B].flags = MEAS_CHF_SAT;
        PhaseChk_Make(&snap, &pul, PULSE_PH_B, 500u, 1u, &in);
        check(in.meas_ok == 0u, "该相贴边 → meas_ok=0");
        PhaseChk_Make(&snap, &pul, PULSE_PH_A, 500u, 1u, &in);
        check(in.meas_ok == 1u, "只有 B 相贴边，A 相仍然可用");
        snap.ch[MEAS_CH_PH_B].flags = 0u;

        /* 母线贴边 → 三相都不可用 */
        snap.ch[MEAS_CH_V24].flags = MEAS_CHF_SAT;
        PhaseChk_Make(&snap, &pul, PULSE_PH_A, 500u, 1u, &in);
        check(in.meas_ok == 0u, "母线贴边 → 三相都判不了");
        snap.ch[MEAS_CH_V24].flags = 0u;

        /* 没有脉冲数据（传 0）也不能炸 */
        PhaseChk_Make(&snap, 0, PULSE_PH_A, 500u, 1u, &in);
        check(in.duty_pm == 0u && in.pulse_flags == 0u && in.meas_ok == 1u,
              "pulse 传 NULL 时脉冲字段清零、电压字段照常");
        /* 没有电压数据 */
        PhaseChk_Make(0, &pul, PULSE_PH_A, 500u, 1u, &in);
        check(in.meas_ok == 0u && in.vbus_mv == 0u, "snap 传 NULL 时 meas_ok=0");
        /* 相位号非法 */
        PhaseChk_Make(&snap, &pul, 9u, 500u, 1u, &in);
        check(in.meas_ok == 0u, "相位号非法 → meas_ok=0（不越界读）");
    }

    /* ---------------- 7. PhaseChk_All 汇总 ---------------- */
    printf("\n-- 7. 三相汇总 --\n");
    {
        phase_chk_in_t  ins[3];
        phase_chk_out_t outs[3];

        mk_in(&ins[0], 500u, 24000u, 12000u);
        mk_in(&ins[1], 500u, 24000u, 12000u);
        mk_in(&ins[2], 500u, 24000u, 12000u);
        check(PhaseChk_All(ins, outs) == PHASE_CHK_SUM_DONE, "三相全好 → DONE，无 ANY_BAD");

        mk_in(&ins[2], 500u, 24000u, 9000u);
        check((PhaseChk_All(ins, outs) & PHASE_CHK_SUM_ANY_BAD) != 0u,
              "一相超差 → ANY_BAD");

        /* 三相全高阻：DONE 但不算失败 */
        mk_in(&ins[0], PHASE_CHK_LVL_HIZ, 24000u, 0u);
        mk_in(&ins[1], PHASE_CHK_LVL_HIZ, 24000u, 0u);
        mk_in(&ins[2], PHASE_CHK_LVL_HIZ, 24000u, 0u);
        check(PhaseChk_All(ins, outs) == PHASE_CHK_SUM_DONE,
              "三相全高阻 → DONE，**不算失败**");

        /* 三相全"数据不全"：同样不算失败 */
        mk_in(&ins[0], 500u, 1000u, 0u);
        mk_in(&ins[1], 500u, 1000u, 0u);
        mk_in(&ins[2], 500u, 1000u, 0u);
        check(PhaseChk_All(ins, outs) == PHASE_CHK_SUM_DONE,
              "三相都因母线太低而没判 → 不算失败");

        /* 混合：一相不判、一相好、一相坏 → 仍然 ANY_BAD */
        mk_in(&ins[0], PHASE_CHK_LVL_HIZ, 24000u, 0u);
        mk_in(&ins[1], 500u, 24000u, 12000u);
        mk_in(&ins[2], 500u, 24000u, 100u);
        check((PhaseChk_All(ins, outs) & PHASE_CHK_SUM_ANY_BAD) != 0u,
              "一相不判 + 一相好 + 一相坏 → ANY_BAD");

        check((PhaseChk_All(ins, 0) & PHASE_CHK_SUM_DONE) != 0u,
              "out 传 NULL 也能跑（只求汇总；这组里有一相坏的，所以汇总里仍有 ANY_BAD）");
        check(PhaseChk_All(0, outs) == 0u, "in 传 NULL → 返回 0");
    }

    /* ---------------- 8. 其它 ---------------- */
    printf("\n-- 8. 边界与文本 --\n");
    check(PhaseChk_Phase(0, &o) == 0u, "in 传 NULL → 返回 0");
    mk_in(&in, 500u, 24000u, 12000u);
    check(PhaseChk_Phase(&in, 0) == PHASE_CHK_F_OK, "out 传 NULL 也能判（返回棋标）");
    check(strcmp(PhaseChk_FlagText(PHASE_CHK_F_OK), "OK") == 0, "FlagText(OK)");
    check(strcmp(PhaseChk_FlagText(0u), "none") == 0, "FlagText(0)");
    check(strcmp(PhaseChk_FlagText(PHASE_CHK_F_DEVIATION), "DEVIATION") == 0, "FlagText(DEVIATION)");
    check(strcmp(PhaseChk_FlagText(0x8000u), "mixed") == 0, "FlagText(未定义位) → mixed");

    printf("\n===== phase_chk: %d 项通过, %d 项失败 =====\n", g_pass, g_fail);
    return (g_fail == 0) ? 0 : 1;
}
