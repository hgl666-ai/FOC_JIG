/*
 * ⑤ 编码器脉冲规划层宿主单测 (enc_plan.c)
 *
 * 被测的是真源码。这一层全是算术，但**算术错一点点，上板就是"数出来的个数不对"**，
 * 所以每个期望值都是手算出来写死在断言里的，不拿实现自己的结果当期望。
 *
 * 翻转模式的换算关系（写死在注释里，方便对照）：
 *   f = clk / (2 × CCR)            ARR 固定 0xFFFF
 *   PSC+1 >= tim_clk / (2 × f × 65535)
 */
#include <stdio.h>
#include <string.h>

#include "enc_plan.h"

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

static void check_u32(unsigned long got, unsigned long want, const char *name)
{
    if (got == want) {
        g_pass++;
        printf("  [ok]   %s (= %lu)\n", name, got);
    } else {
        g_fail++;
        printf("  [FAIL] %s (got %lu, want %lu)\n", name, got, want);
    }
}

int main(void)
{
    enc_plan_t p;

    printf("===== enc_plan 单测（⑤ 编码器脉冲规划）=====\n");

    /* ---------------- 1. 边沿数 ---------------- */
    printf("\n-- 1. 脉冲 → 边沿 --\n");
    check_u32(EncPlan_Edges(0u),     0u,    "0 个脉冲 → 0 个边沿");
    check_u32(EncPlan_Edges(1u),     2u,    "1 个脉冲 → 2 个边沿（上+下）");
    check_u32(EncPlan_Edges(1000u),  2000u, "1000 个 → 2000 个边沿");
    check_u32(EncPlan_Edges(2000u),  4000u, "2000 个 → 4000 个边沿");
    check_u32(EncPlan_Edges(32767u), 65534u, "上限 32767 → 65534（不溢出 u16）");

    /* ---------------- 2. 单路时长 ---------------- */
    printf("\n-- 2. 单路时长 --\n");
    check_u32(EncPlan_PhaseMs(1000u, 5000u),  200u, "1000 个 @5kHz = 200ms");
    check_u32(EncPlan_PhaseMs(2000u, 10000u), 200u, "2000 个 @10kHz = 200ms");
    check_u32(EncPlan_PhaseMs(1u, 5000u),     1u,   "1 个 @5kHz = 0.2ms → 向上取整 1ms");
    check_u32(EncPlan_PhaseMs(1u, 3u),        334u, "1 个 @3Hz = 333.3ms → 334ms");
    check_u32(EncPlan_PhaseMs(0u, 5000u),     0u,   "0 个脉冲 → 0ms");
    check_u32(EncPlan_PhaseMs(1000u, 0u),     0u,   "频率 0 → 0ms（不除零）");
    check_u32(EncPlan_PhaseMs(300u, 1000u),   300u, "300 个 @1kHz = 300ms");

    /* ---------------- 3. PSC ---------------- */
    printf("\n-- 3. 预分频 PSC --\n");
    check_u32(EncPlan_PscFor(72000000u, 5000u),  0u,  "5kHz：不分频就够（CCR=7200）");
    check_u32(EncPlan_PscFor(72000000u, 10000u), 0u,  "10kHz：不分频");
    check_u32(EncPlan_PscFor(72000000u, 600u),   0u,  "600Hz：刚好不用分频");
    check_u32(EncPlan_PscFor(72000000u, 100u),   5u,  "100Hz：要 PSC=5（clk 降到 12MHz）");
    check_u32(EncPlan_PscFor(72000000u, 0u),     5u,  "f=0 时按下限 100Hz 算 → PSC=5");
    check_u32(EncPlan_PscFor(0u, 5000u),     65535u,  "时钟为 0 → 不可实现（0xFFFF）");

    /* ---------------- 4. CCR ---------------- */
    printf("\n-- 4. CCR（翻转模式：f = clk/(2×CCR)）--\n");
    check_u32(EncPlan_CcrFor(72000000u, 5000u),  7200u, "72MHz / 5kHz  → CCR 7200");
    check_u32(EncPlan_CcrFor(72000000u, 10000u), 3600u, "72MHz / 10kHz → CCR 3600");
    check_u32(EncPlan_CcrFor(72000000u, 20000u), 1800u, "72MHz / 20kHz → CCR 1800");
    check_u32(EncPlan_CcrFor(12000000u, 100u),  60000u, "12MHz / 100Hz → CCR 60000（PSC=5 之后）");
    check_u32(EncPlan_CcrFor(12000000u, 5000u),  1200u, "12MHz / 5kHz  → CCR 1200");
    check_u32(EncPlan_CcrFor(12000000u, 10000u),  600u, "12MHz / 10kHz → CCR 600");
    check_u32(EncPlan_CcrFor(72000000u, 100u),      0u, "72MHz / 100Hz 直接算会 360000 → 超 16 位，返回 0");
    check_u32(EncPlan_CcrFor(0u, 5000u),            0u, "时钟 0 → 0");
    check_u32(EncPlan_CcrFor(72000000u, 0u),        0u, "频率 0 → 0（不除零）");
    check_u32(EncPlan_CcrFor(2000000u, 20000u),    50u, "2MHz / 20kHz → CCR 50（仍 ≥1，可用）");

    /* ---------------- 5. 默认参数 ---------------- */
    printf("\n-- 5. 默认参数（用户给的：A1000/B2000/Z1）--\n");
    memset(&p, 0, sizeof(p));
    EncPlan_Default(&p);
    check(p.cnt_a == 1000u && p.cnt_b == 2000u && p.cnt_z == 1u,
          "个数：A=1000、B=2000、Z=1（三路各不相同）");
    check(p.freq_a_hz == 5000u && p.freq_b_hz == 10000u && p.freq_z_hz == 5000u,
          "频率：A=5kHz、B=10kHz、Z=5kHz");
    check(EncPlan_Check(&p) == 1u, "默认参数校验通过");
    check_u32(EncPlan_DurationMs(&p), 205u, "整串时长 = max(200,200,1) + 5ms 余量 = 205ms");
    check_u32(EncPlan_Edges(p.cnt_a), 2000u, "A 应产生 2000 个边沿");
    check_u32(EncPlan_Edges(p.cnt_b), 4000u, "B 应产生 4000 个边沿");
    check_u32(EncPlan_Edges(p.cnt_z), 2u,    "Z 应产生 2 个边沿");
    EncPlan_Default(0);
    check(1, "Default(NULL) 不炸");

    /* ---------------- 6. 校验规则 ---------------- */
    printf("\n-- 6. 校验规则（非法必须挡住，且不能有任何输出）--\n");
    check(EncPlan_Check(0) == 0u, "空指针 → 非法");

    EncPlan_Default(&p);
    p.cnt_a = p.cnt_b = p.cnt_z = 0u;
    check(EncPlan_Check(&p) == 0u, "三路都不发 → 非法");
    check_u32(EncPlan_DurationMs(&p), 0u, "三路都不发 → 时长 0");

    EncPlan_Default(&p);
    p.freq_a_hz = 99u;
    check(EncPlan_Check(&p) == 0u, "频率 99Hz（低于下限）→ 非法");
    EncPlan_Default(&p);
    p.freq_b_hz = 20001u;
    check(EncPlan_Check(&p) == 0u, "频率 20001Hz（高于上限）→ 非法");
    EncPlan_Default(&p);
    p.freq_a_hz = 100u;
    check(EncPlan_Check(&p) == 1u, "频率踩下限 100Hz → 合法");
    p.freq_a_hz = 20000u;
    check(EncPlan_Check(&p) == 1u, "频率踩上限 20000Hz → 合法");

    EncPlan_Default(&p);
    p.freq_z_hz = 0u;                       /* 即使 Z 那一路不发，也要校验 */
    p.cnt_z = 0u;
    check(EncPlan_Check(&p) == 0u, "不发的那一路频率非法也要挡住（不许蒙混过关）");

    EncPlan_Default(&p);
    p.cnt_a = ENC_PLAN_PULSE_MAX + 1u;
    check(EncPlan_Check(&p) == 0u, "脉冲数超上限 → 非法");
    p.cnt_a = ENC_PLAN_PULSE_MAX;
    check(EncPlan_Check(&p) == 1u, "脉冲数踩上限 → 合法");

    /* 低频 + 高频混发：PSC 按最低那一路算，两路都要能算出来 */
    EncPlan_Default(&p);
    p.cnt_a = 10u; p.freq_a_hz = 100u;      /* 最低 100Hz → PSC=5 → clk 12MHz */
    p.cnt_b = 100u; p.freq_b_hz = 20000u;
    p.cnt_z = 0u;   p.freq_z_hz = 5000u;
    check(EncPlan_Check(&p) == 1u, "100Hz 与 20kHz 混发 → 合法（PSC 按最低那路算）");
    check_u32(EncPlan_DurationMs(&p), 105u, "混发时长 = max(100ms, 5ms) + 5 = 105ms");

    /* ---------------- 7. 只有一路发 ---------------- */
    printf("\n-- 7. 只发一路 --\n");
    memset(&p, 0, sizeof(p));
    p.cnt_z = 1u; p.freq_z_hz = 5000u;
    p.freq_a_hz = 5000u;                    /* 不发的那两路也要给合法频率 */
    p.freq_b_hz = 5000u;
    check(EncPlan_Check(&p) == 1u, "只发 Z 一个脉冲 → 合法");
    check_u32(EncPlan_DurationMs(&p), 6u, "时长 = 1ms + 5ms 余量");

    printf("\n===== enc_plan: %d 项通过, %d 项失败 =====\n", g_pass, g_fail);
    return (g_fail == 0) ? 0 : 1;
}
