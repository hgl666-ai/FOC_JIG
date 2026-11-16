/*
 * 板子 SN 上报（sn_report）—— 宿主单测
 *
 * 纯逻辑层，所以能穷举：XOR 范围、字段落位、去重（首次/同板/换板）、空指针。
 * 真机上难复现的"读 UID 失败"由调用方静默跳过，这里只测算法本身。
 */
#include <stdio.h>
#include <string.h>
#include <stdint.h>

#include "sn_report.h"

static int g_pass = 0;
static int g_fail = 0;

#define CHECK(cond, msg)                                                      \
    do {                                                                      \
        if (cond) { g_pass++; }                                               \
        else { g_fail++; printf("  FAIL: %s  (line %d)\n", msg, __LINE__); }   \
    } while (0)

#define SECTION(t) printf("\n[%s]\n", t)

static void mk_uid(uint8_t *u, uint8_t seed)
{
    uint8_t i;

    for (i = 0; i < (uint8_t)SN_UID_LEN; i++) {
        u[i] = (uint8_t)(seed + i);
    }
}

int main(void)
{
    uint8_t u1[SN_UID_LEN], u2[SN_UID_LEN], out[SN_REPORT_LEN];

    printf("========================================\n");
    printf(" sn_report: board SN upload (0x0024)\n");
    printf("========================================\n");

    SECTION("1) XOR 校验：只算 12B UID，不含版本号");
    mk_uid(u1, 0x10u);                       /* 0x10..0x1B */
    {
        /* 手算：0x10^0x11^...^0x1B —— 两两成对异或为 0x01，6 对 → 0x00 */
        CHECK(SnReport_UidXor(u1) == 0x00u, "0x10..0x1B 逐字节异或 = 0x00（手算核对）");
        u1[0] = 0xFFu;
        CHECK(SnReport_UidXor(u1) == (uint8_t)(0x00u ^ 0x10u ^ 0xFFu),
              "改一个字节，XOR 只随之改变这一个字节的影响");
        CHECK(SnReport_UidXor(0) == 0u, "空指针返回 0（不崩）");
    }

    SECTION("2) 组帧：16 字节逐位置核对");
    mk_uid(u1, 0xA0u);
    CHECK(SnReport_Build(u1, out) == SN_REPORT_LEN, "返回 16");
    CHECK(SN_REPORT_LEN == 16u && SN_UID_LEN == 12u, "常量：DATA 16B / UID 12B");
    CHECK(out[0] == SN_ALGO_VER, "[0] = 算法版本 0x01");
    CHECK(memcmp(&out[1], u1, SN_UID_LEN) == 0, "[1..12] = UID 原样（大端已由调用方保证）");
    CHECK(out[13] == SnReport_UidXor(u1), "[13] = UID 的 XOR（不含 out[0]）");
    CHECK(out[13] != (uint8_t)(SnReport_UidXor(u1) ^ SN_ALGO_VER) ||
          SnReport_UidXor(u1) == (uint8_t)(SnReport_UidXor(u1) ^ SN_ALGO_VER),
          "[13] 不把版本号算进去（这条防的是一次踩过的 13 字节 XOR）");
    CHECK(out[14] == 0u && out[15] == 0u, "[14..15] = 预留 0x0000");
    CHECK(SnReport_Build(0, out) == 0u, "uid 为空返回 0");
    CHECK(SnReport_Build(u1, 0) == 0u, "out 为空返回 0");

    SECTION("3) 去重：首次报 / 同板不报 / 换板再报");
    SnReport_Reset();
    mk_uid(u1, 0x01u);
    mk_uid(u2, 0x80u);
    CHECK(SnReport_IsNewBoard(u1) == 1u, "上电后第一块板 → 要报");
    SnReport_MarkReported(u1);
    CHECK(SnReport_IsNewBoard(u1) == 0u, "同一块板再烧一次 → 不重复上报");
    CHECK(SnReport_IsNewBoard(u2) == 1u, "换了一块板 → 要报");
    SnReport_MarkReported(u2);
    CHECK(SnReport_IsNewBoard(u2) == 0u, "新板报过之后 → 不再重复");
    CHECK(SnReport_IsNewBoard(u1) == 1u, "又换回原来那块 → 也要报（只记最近一次）");
    SnReport_Reset();
    CHECK(SnReport_IsNewBoard(u1) == 1u, "Reset 之后重新当成第一块板");
    CHECK(SnReport_IsNewBoard(0) == 0u, "空指针 → 不报（不能推脏数据）");
    SnReport_MarkReported(0);                      /* 空指针也不该崩 */
    CHECK(1, "MarkReported(空指针) 不崩");

    SECTION("4) 只差 1 bit 也算换板（UID 是身份证，不能模糊匹配）");
    SnReport_Reset();
    mk_uid(u1, 0x30u);
    SnReport_MarkReported(u1);
    memcpy(u2, u1, SN_UID_LEN);
    u2[SN_UID_LEN - 1] ^= 0x01u;
    CHECK(SnReport_IsNewBoard(u2) == 1u, "最后 1 bit 不同 → 视为新板");
    CHECK(SnReport_Build(u2, out) == SN_REPORT_LEN, "新板也能正常组帧");

    SECTION("5) 自动检测：去抖 4 次 / 每次插入只报一次 / 拔板清零");
    {
        uint32_t t = 0u;
        uint8_t  done = 0u, cnt = 0u;
        uint8_t  i, trig;

        /* 5.1 连续 3 次在位还不够（去抖 4 次）*/
        t = 0u;
        for (i = 0u; i < 3u; i++) {
            t += SN_DET_POLL_MS;
            trig = SnReport_AutoPoll(SN_DET_ST_PRESENT, t);
            CHECK(trig == 0u, "第 3 次在位还不够（去抖要 4 次）");
        }
        /* 5.2 第 4 次触发读 UID */
        t += SN_DET_POLL_MS;
        trig = SnReport_AutoPoll(SN_DET_ST_PRESENT, t);
        CHECK(trig == 1u, "第 4 次在位 → 触发读 UID");
        SnReport_AutoGetState(&done, &cnt);
        CHECK(done == 1u && cnt == 4u, "标志位：done=1，in_cnt=4");

        /* 5.3 同一块板还插着 → 不再触发（本次插入只报一次）*/
        t += SN_DET_POLL_MS;
        CHECK(SnReport_AutoPoll(SN_DET_ST_PRESENT, t) == 0u,
              "同一次插入不重复触发");

        /* 5.4 拔板 → 两个标志清零 */
        t += SN_DET_POLL_MS;
        CHECK(SnReport_AutoPoll(SN_DET_ST_ABSENT, t) == 0u, "拔板不触发");
        SnReport_AutoGetState(&done, &cnt);
        CHECK(done == 0u && cnt == 0u, "拔板后 done/in_cnt 都清 0（允许下次插入再报）");

        /* 5.5 再插一次 → 又能触发（同一块板插两次就报两次）*/
        for (i = 0u; i < 4u; i++) {
            t += SN_DET_POLL_MS;
            trig = SnReport_AutoPoll(SN_DET_ST_PRESENT, t);
        }
        CHECK(trig == 1u, "重新插入 → 再次触发（同板插两次报两次）");

        /* 5.6 读 UID 失败 → 清 done，下轮重试 */
        SnReport_AutoResult(0u);
        SnReport_AutoGetState(&done, &cnt);
        CHECK(done == 0u, "读失败 → done 清 0（下轮重试）");
        t += SN_DET_POLL_MS;
        CHECK(SnReport_AutoPoll(SN_DET_ST_PRESENT, t) == 1u, "下一轮会再试");

        /* 5.7 读成功 → 保持 done，不再触发 */
        SnReport_AutoResult(1u);
        t += SN_DET_POLL_MS;
        CHECK(SnReport_AutoPoll(SN_DET_ST_PRESENT, t) == 0u, "读成功后就停手了");

        /* 5.8 中间值不影响状态（滞回）*/
        SnReport_AutoResult(0u);                    /* 造成 done=0，允许再触发 */
        t += SN_DET_POLL_MS;
        CHECK(SnReport_AutoPoll(SN_DET_ST_PRESENT, t) == 1u, "重新武装");
        SnReport_AutoResult(1u);
        t += SN_DET_POLL_MS;
        CHECK(SnReport_AutoPoll(SN_DET_ST_MID, t) == 0u, "中间值不触发也不清标志");
        SnReport_AutoGetState(&done, &cnt);
        CHECK(done == 1u, "中间值保持 done 不变");

        /* 5.9 节流：不到 200ms 不重复判断 */
        t += SN_DET_POLL_MS;
        CHECK(SnReport_AutoPoll(SN_DET_ST_ABSENT, t) == 0u, "拔板（到点了，清标志）");
        CHECK(SnReport_AutoPoll(SN_DET_ST_PRESENT, t) == 0u, "同一毫秒内再调 → 被节流");

        /* 5.10 阈值：实测量级 */
        CHECK(SN_DET_PRESENT_RAW == 2000u && SN_DET_ABSENT_RAW == 100u,
              "阈值与实测匹配（在位 3707 / 拔板 1）");
    }

    printf("\n===== sn_report: %d 项通过, %d 项失败 =====\n", g_pass, g_fail);
    return (g_fail == 0) ? 0 : 1;
}
