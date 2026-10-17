/*
 * ⑤ 编码器三方对齐判据（enc_chk）—— 宿主单测
 *
 * 判据层是纯函数，所以这里可以**穷举**各种组合；真机上难复现的情况
 * （U2 少数一个沿、DUT 报了半边、只有一边有数据…）在这里都能直接摆出来。
 *
 * 覆盖：
 *   1) 脉冲数 → 边沿数（×2、饱和）
 *   2) 三边一致 / U2 不符 / DUT 不符 / 两边都不符
 *   3) 只有一侧有数据 → NODATA（不判），但**有数据那侧不符仍要指出来**
 *   4) 容差边界（=tol 算合格，tol+1 算不合格）
 *   5) 汇总位：DONE / ANY_BAD / ANY_NODATA / U2_LOST / DIR_ZERO
 *   6) 输出结构体（err_u2 / err_dut 的有符号值）
 */
#include <stdio.h>
#include <string.h>
#include <stdint.h>

#include "enc_chk.h"

static int g_pass = 0;
static int g_fail = 0;

#define CHECK(cond, msg)                                                      \
    do {                                                                      \
        if (cond) { g_pass++; }                                               \
        else { g_fail++; printf("  FAIL: %s  (line %d)\n", msg, __LINE__); }   \
    } while (0)

#define SECTION(t) printf("\n[%s]\n", t)

static void make_in(enc_chk_in_t *in, uint16_t exp, uint32_t u2, uint32_t dut,
                    uint8_t has_u2, uint8_t has_dut)
{
    memset(in, 0, sizeof(*in));
    in->exp_edges = exp;
    in->u2_edges  = u2;
    in->dut_edges = dut;
    in->has_u2    = has_u2;
    in->has_dut   = has_dut;
}

/*============================================================================*/
static void test_edges_from_pulses(void)
{
    SECTION("1) 脉冲数 → 边沿数");
    CHECK(EncChk_EdgesFromPulses(0u) == 0u, "0 个脉冲 = 0 个边沿");
    CHECK(EncChk_EdgesFromPulses(1u) == 2u, "1 个脉冲 = 2 个边沿（一升一降）");
    CHECK(EncChk_EdgesFromPulses(1000u) == 2000u, "默认 A 相 1000 → 2000");
    CHECK(EncChk_EdgesFromPulses(2000u) == 4000u, "默认 B 相 2000 → 4000");
    CHECK(EncChk_EdgesFromPulses(32767u) == 65534u, "上限 32767 → 65534（还在 u16 里）");
    CHECK(EncChk_EdgesFromPulses(32768u) == 0xFFFFu, "超过 32767 → 饱和成 0xFFFF（不再乘 2）");
    CHECK(EncChk_EdgesFromPulses(65535u) == 0xFFFFu, "65535 同样饱和");
}

static void test_judge_basic(void)
{
    enc_chk_in_t  in;
    enc_chk_out_t out;
    uint16_t      f;

    SECTION("2) 三边一致 / 单边不符 / 两边都不符");

    /* 三边一致 */
    make_in(&in, 2000u, 2000u, 2000u, 1u, 1u);
    f = EncChk_Judge(&in, 0u, &out);
    CHECK(f == ENC_CHK_F_OK, "三边相等 → OK");
    CHECK(out.err_u2 == 0, "err_u2 = 0");
    CHECK(out.err_dut == 0, "err_dut = 0");
    CHECK(out.exp_edges == 2000u && out.u2_edges == 2000u && out.dut_edges == 2000u,
          "输出结构体原样带回三个数");

    /* U2 少一个边沿（典型：最后一个沿落在窗口外） */
    make_in(&in, 2000u, 1999u, 2000u, 1u, 1u);
    f = EncChk_Judge(&in, 0u, &out);
    CHECK((f & ENC_CHK_F_U2_DIFF) != 0u, "U2 1999 ≠ 意图 2000 → U2_DIFF");
    CHECK((f & ENC_CHK_F_DUT_DIFF) == 0u, "DUT 对得上，不应报 DUT_DIFF");
    CHECK((f & ENC_CHK_F_OK) == 0u, "有差异时不能同时给 OK");
    CHECK(out.err_u2 == -1, "err_u2 = -1（少了）");

    /* DUT 多一倍（最经典的错：把脉冲当边沿数了） */
    make_in(&in, 2000u, 2000u, 1000u, 1u, 1u);
    f = EncChk_Judge(&in, 0u, &out);
    CHECK((f & ENC_CHK_F_DUT_DIFF) != 0u, "DUT 1000 ≠ 2000 → DUT_DIFF");
    CHECK((f & ENC_CHK_F_U2_DIFF) == 0u, "U2 对得上，不应报 U2_DIFF");
    CHECK(out.err_dut == -1000, "err_dut = -1000（少了，多半是按脉冲数的）");

    /* 两边都不符（线接错/没发出去） */
    make_in(&in, 2000u, 0u, 0u, 1u, 1u);
    f = EncChk_Judge(&in, 0u, &out);
    CHECK((f & ENC_CHK_F_U2_DIFF) != 0u, "两边都是 0：U2_DIFF");
    CHECK((f & ENC_CHK_F_DUT_DIFF) != 0u, "两边都是 0：DUT_DIFF");
    CHECK(f == (ENC_CHK_F_U2_DIFF | ENC_CHK_F_DUT_DIFF), "flags 恰好是这两位的或");

    /* 这一路本来就不发脉冲（exp=0），两边都报 0 */
    make_in(&in, 0u, 0u, 0u, 1u, 1u);
    f = EncChk_Judge(&in, 0u, &out);
    CHECK(f == ENC_CHK_F_OK, "不发的那一路，两边都报 0 → OK");
}

static void test_judge_nodata(void)
{
    enc_chk_in_t  in;
    enc_chk_out_t out;
    uint16_t      f;

    SECTION("3) 数据不全 → 不判（但已有一侧不符仍要指出）");

    /* 两侧都没数据 */
    make_in(&in, 2000u, 0u, 0u, 0u, 0u);
    f = EncChk_Judge(&in, 0u, &out);
    CHECK(f == ENC_CHK_F_NODATA, "两侧都没数据 → 只有 NODATA");

    /* 只有 U2，且对得上 → NODATA（**不能**给 OK：缺的那条腿可能正是坏的） */
    make_in(&in, 2000u, 2000u, 0u, 1u, 0u);
    f = EncChk_Judge(&in, 0u, &out);
    CHECK(f == ENC_CHK_F_NODATA, "只有 U2 且相符 → 仍然是 NODATA，不给 OK");

    /* 只有 U2，且对不上 → NODATA + U2_DIFF（故障要指出来） */
    make_in(&in, 2000u, 1500u, 0u, 1u, 0u);
    f = EncChk_Judge(&in, 0u, &out);
    CHECK((f & ENC_CHK_F_NODATA) != 0u, "只有 U2 且不符 → 带 NODATA");
    CHECK((f & ENC_CHK_F_U2_DIFF) != 0u, "只有 U2 且不符 → 也要带 U2_DIFF");

    /* 只有 DUT，且对不上 */
    make_in(&in, 4000u, 0u, 4000u, 0u, 1u);
    f = EncChk_Judge(&in, 0u, &out);
    CHECK(f == ENC_CHK_F_NODATA, "只有 DUT 且相符 → NODATA");

    make_in(&in, 4000u, 0u, 2000u, 0u, 1u);
    f = EncChk_Judge(&in, 0u, &out);
    CHECK((f & ENC_CHK_F_DUT_DIFF) != 0u, "只有 DUT 且不符 → 带 DUT_DIFF");
    CHECK(out.err_dut == -2000, "err_dut = -2000");
}

static void test_judge_tol(void)
{
    enc_chk_in_t in;
    uint16_t     f;

    SECTION("4) 容差边界");
    make_in(&in, 2000u, 2005u, 1995u, 1u, 1u);
    f = EncChk_Judge(&in, 0u, 0);
    CHECK((f & (ENC_CHK_F_U2_DIFF | ENC_CHK_F_DUT_DIFF)) != 0u, "tol=0 时 ±5 都算不符");

    f = EncChk_Judge(&in, 5u, 0);
    CHECK(f == ENC_CHK_F_OK, "tol=5 时 |±5| 恰好合格（边界取等号）");

    make_in(&in, 2000u, 2006u, 2000u, 1u, 1u);
    f = EncChk_Judge(&in, 5u, 0);
    CHECK((f & ENC_CHK_F_U2_DIFF) != 0u, "tol=5 时 +6 不符");

    /* 大容差下"少了一大半"也能放过 —— 提醒：容差不是越大越好 */
    make_in(&in, 2000u, 1000u, 1000u, 1u, 1u);
    f = EncChk_Judge(&in, 1000u, 0);
    CHECK(f == ENC_CHK_F_OK, "tol=1000 时差 1000 也判合格（所以默认 tol=0）");
}

static void test_all(void)
{
    enc_chk_in_t  in[ENC_CHK_PH_COUNT];
    enc_chk_out_t out[ENC_CHK_PH_COUNT];
    uint16_t      sum;
    uint8_t       i;

    SECTION("5) 三相汇总");

    /* 全部一致 */
    for (i = 0u; i < (uint8_t)ENC_CHK_PH_COUNT; i++) {
        uint16_t exp = (i == 0u) ? 2000u : ((i == 1u) ? 4000u : 2u);
        make_in(&in[i], exp, exp, exp, 1u, 1u);
    }
    sum = EncChk_All(in, 0u, 0u, 1, out);
    CHECK((sum & ENC_CHK_SUM_DONE) != 0u, "DONE 置位");
    CHECK((sum & ENC_CHK_SUM_ANY_BAD) == 0u, "没有不合格");
    CHECK((sum & ENC_CHK_SUM_ANY_NODATA) == 0u, "没有缺数据");
    CHECK((sum & ENC_CHK_SUM_U2_LOST) == 0u, "U2 没丢沿");
    CHECK((sum & ENC_CHK_SUM_DIR_ZERO) == 0u, "DUT 报了方向 → 不置 DIR_ZERO");
    CHECK(out[2].exp_edges == 2u, "Z 相意图 1 个脉冲 = 2 个边沿");

    /* Z 相 DUT 少一个（只有 Z 错） */
    in[2].dut_edges = 1u;
    sum = EncChk_All(in, 0u, 0u, 1, out);
    CHECK((sum & ENC_CHK_SUM_ANY_BAD) != 0u, "Z 相不符 → ANY_BAD");
    CHECK((out[2].flags & ENC_CHK_F_DUT_DIFF) != 0u, "错误落在 Z 相上");
    CHECK((out[0].flags & ENC_CHK_F_OK) != 0u, "A 相仍然是 OK");

    /* U2 丢过边沿 → 警告位（先把 Z 相改回一致，否则 ANY_BAD 是上一小节的残留） */
    in[2].dut_edges = 2u;
    sum = EncChk_All(in, 0u, 3u, 1, out);
    CHECK((sum & ENC_CHK_SUM_U2_LOST) != 0u, "u2_lost=3 → 置 U2_LOST");
    CHECK((sum & ENC_CHK_SUM_ANY_BAD) == 0u, "丢沿只是警告，不单独判失败");

    /* DUT 报方向 = 没动，但确实发了脉冲 → 警告位 */
    make_in(&in[0], 2000u, 2000u, 2000u, 1u, 1u);
    make_in(&in[1], 4000u, 4000u, 4000u, 1u, 1u);
    make_in(&in[2], 2u, 2u, 2u, 1u, 1u);
    sum = EncChk_All(in, 0u, 0u, 0, out);
    CHECK((sum & ENC_CHK_SUM_DIR_ZERO) != 0u, "发了脉冲但 dir=0 → 置 DIR_ZERO");

    /* 完全没发脉冲、dir 也是 0 → 不该报 DIR_ZERO */
    for (i = 0u; i < (uint8_t)ENC_CHK_PH_COUNT; i++) {
        make_in(&in[i], 0u, 0u, 0u, 1u, 1u);
    }
    sum = EncChk_All(in, 0u, 0u, 0, out);
    CHECK((sum & ENC_CHK_SUM_DIR_ZERO) == 0u, "没发脉冲时 dir=0 是正常的，不报警");

    /* 全线缺数据 */
    for (i = 0u; i < (uint8_t)ENC_CHK_PH_COUNT; i++) {
        make_in(&in[i], 2000u, 0u, 0u, 0u, 0u);
    }
    sum = EncChk_All(in, 0u, 0u, 0, out);
    CHECK((sum & ENC_CHK_SUM_ANY_NODATA) != 0u, "全缺数据 → ANY_NODATA");
    CHECK((sum & ENC_CHK_SUM_ANY_BAD) == 0u, "缺数据不等于不合格");

    /* 传 0 不崩 */
    CHECK(EncChk_All(0, 0u, 0u, 0, 0) == 0u, "In 传空指针返回 0");
    CHECK(EncChk_Judge(0, 0u, 0) == 0u, "Judge 传空指针返回 0");
}

static void test_text(void)
{
    SECTION("6) 文本接口");
    CHECK(EncChk_FlagText(ENC_CHK_F_OK) != 0, "FlagText(OK) 有文本");
    CHECK(EncChk_FlagText(ENC_CHK_F_U2_DIFF) != 0, "FlagText(U2_DIFF) 有文本");
    CHECK(EncChk_FlagText(ENC_CHK_F_DUT_DIFF) != 0, "FlagText(DUT_DIFF) 有文本");
    CHECK(EncChk_FlagText(ENC_CHK_F_NODATA) != 0, "FlagText(NODATA) 有文本");
    CHECK(EncChk_FlagText(0u) != 0, "FlagText(0) 也有兜底文本");
    CHECK(EncChk_SumText(ENC_CHK_SUM_ANY_BAD | ENC_CHK_SUM_DONE) != 0, "SumText(坏) 有文本");
    CHECK(EncChk_SumText(ENC_CHK_SUM_DONE) != 0, "SumText(好) 有文本");
    CHECK(EncChk_SumText(0u) != 0, "SumText(0) 也有兜底文本");
}

/*============================================================================*/
int main(void)
{
    printf("========================================\n");
    printf(" enc_chk: encoder three-way judgement\n");
    printf("========================================\n");

    test_edges_from_pulses();
    test_judge_basic();
    test_judge_nodata();
    test_judge_tol();
    test_all();
    test_text();

    printf("\n===== enc_chk: %d 项通过, %d 项失败 =====\n", g_pass, g_fail);
    return (g_fail == 0) ? 0 : 1;
}
