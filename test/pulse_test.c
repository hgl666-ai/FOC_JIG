/*
 * TIM 输入捕获宿主单测（pulse_cap.c + pulse_pack.c 真源码）
 *
 * 被测的是固件真源码，只提供 HAL_GetTick 替身（可以手动推进"时间"）。
 * 硬件那部分（定时器寄存器、捕获中断）在宿主上没法跑，所以这里直接往
 * PULSE_OnEdge() 灌"某相某时刻来了一个边沿"——**算法对不对全靠这个测试**。
 *
 * 覆盖：
 *   1) 常见频率/占空比：20kHz-50%、10kHz-25%、1kHz-90%、40kHz-10%
 *   2) 低端：10Hz（窗口内只有 2 个周期）
 *   3) 无信号 / 边沿太少
 *   4) 丢边沿（异常间隔）被丢弃并标记
 *   5) 毛刺（同一时刻两个边沿）
 *   6) 超范围（周期过小）
 *   7) 环缓冲覆盖（高速信号，边沿数远多于 32）
 *   8) 窗口还没到 / 不在采
 *   9) 线格式打包往返 + 布局逐字节 + 越界拒绝
 *  10) 两端对比：有/无、频率一致/不一致、容差边界
 */
#include <stdio.h>
#include <string.h>
#include <stdint.h>

#include "pulse_cap.h"
#include "pulse_pack.h"

static int g_checks = 0;
static int g_fails  = 0;

#define CHECK(cond, msg)                                                     \
    do {                                                                     \
        g_checks++;                                                          \
        if (!(cond)) { printf("  FAIL: %s  (line %d)\n", msg, __LINE__); g_fails++; } \
    } while (0)

#define SECTION(t) printf("\n[%s]\n", t)

/* ---- HAL 替身：时间可以手动推进 ---- */
static uint32_t g_tick = 0u;
uint32_t HAL_GetTick(void) { return g_tick; }

/* ---- 工具：按 tick 生成一相 PWM 的边沿序列 ---- */
/* 从 t0 开始，产生 n 个周期：上升沿在 t0 + k*period，下降沿在 +high */
static void feed_pwm(pulse_ph_t ph, uint32_t t0, uint32_t period, uint32_t high,
                     uint32_t cycles)
{
    uint32_t k;
    for (k = 0u; k < cycles; k++) {
        PULSE_OnEdge(ph, t0 + k * period);
        PULSE_OnEdge(ph, t0 + k * period + high);
    }
}

/* 三个边沿：给"最少边沿数"之类用例用 */
static void feed_pwm_all(uint32_t t0, uint32_t period, uint32_t high, uint32_t cycles)
{
    feed_pwm(PULSE_PH_A, t0, period, high, cycles);
    feed_pwm(PULSE_PH_B, t0, period, high, cycles);
    feed_pwm(PULSE_PH_C, t0, period, high, cycles);
}

/* tick 与 ns 的换算（与被测代码独立地再写一遍） */
static uint32_t ns_of(uint32_t tick) { return (uint32_t)(((uint64_t)tick * 125u) / 9u); }

/* 跑一个完整窗口：灌边沿 → 推时间 → 结算 */
static void run_window(uint32_t period, uint32_t high, uint32_t cycles)
{
    PULSE_Start(PULSE_WINDOW_MS);
    feed_pwm_all(1000u, period, high, cycles);
    g_tick += (uint32_t)PULSE_WINDOW_MS + 1u;
    PULSE_Poll();
}

/*============================================================================*/
int main(void)
{
    pulse_snapshot_t s;
    int i;

    printf("========================================\n");
    printf(" pulse_cap / pulse_pack host test\n");
    printf("========================================\n");

    PULSE_Init();

    /* ============ 1. 常见频率与占空比 ============ */
    SECTION("1) 频率/占空比");
    {
        struct { uint32_t hz; uint32_t duty_ppm; const char *name; } cases[] = {
            {  20000u, 500000u, "20kHz 50%"  },
            {  10000u, 250000u, "10kHz 25%"  },
            {   1000u, 900000u, "1kHz 90%"   },
            {   40000u, 100000u, "40kHz 10%" },
            {   50000u, 333000u, "50kHz 33.3%"},
        };
        int n = (int)(sizeof(cases) / sizeof(cases[0]));

        for (i = 0; i < n; i++) {
            /* 周期(tick) = 72e6 / f；高电平 = 周期 × 占空比 */
            uint32_t period_t = (uint32_t)(72000000u / cases[i].hz);
            uint32_t high_t   = (uint32_t)(((uint64_t)period_t * cases[i].duty_ppm) / 1000000u);
            uint32_t want_period_ns = ns_of(period_t);
            uint32_t want_high_ns   = ns_of(high_t);
            uint32_t want_duty      = (uint32_t)(((uint64_t)high_t * 1000u) / period_t);
            char nm[64];

            PULSE_Init();
            run_window(period_t, high_t, 50u);
            PULSE_Snapshot(&s);

            sprintf(nm, "%s: 周期", cases[i].name);
            CHECK(s.ph[PULSE_PH_A].period_ns == want_period_ns, nm);
            sprintf(nm, "%s: 高电平", cases[i].name);
            CHECK(s.ph[PULSE_PH_A].high_ns == want_high_ns, nm);
            sprintf(nm, "%s: 占空比 ±2‰", cases[i].name);
            CHECK((s.ph[PULSE_PH_A].duty_permille + 2u) >= want_duty &&
                  (s.ph[PULSE_PH_A].duty_permille <= (uint16_t)(want_duty + 2u)), nm);
            sprintf(nm, "%s: 三相都对", cases[i].name);
            CHECK((s.ph[PULSE_PH_B].period_ns == want_period_ns) &&
                  (s.ph[PULSE_PH_C].period_ns == want_period_ns), nm);
            sprintf(nm, "%s: 有效标志", cases[i].name);
            CHECK((s.ph[PULSE_PH_A].flags & PULSE_F_VALID) != 0u, nm);
        }
        printf("        20kHz: 周期=%u ns 高=%u ns 占空比=%u‰ 边沿=%u\n",
               s.ph[PULSE_PH_A].period_ns, s.ph[PULSE_PH_A].high_ns,
               s.ph[PULSE_PH_A].duty_permille, s.ph[PULSE_PH_A].edges);
    }

    /* ============ 2. 低端：10Hz（窗口内只有 2 个周期） ============ */
    SECTION("2) 低频边界 10Hz");
    {
        uint32_t period_t = 7200000u;          /* 0.1 s */
        uint32_t high_t   = 3600000u;          /* 50% */

        PULSE_Init();
        PULSE_Start(PULSE_WINDOW_MS);          /* 200ms 窗口 = 2 个周期 */
        feed_pwm_all(0u, period_t, high_t, 2u);
        g_tick += (uint32_t)PULSE_WINDOW_MS + 1u;
        PULSE_Poll();
        PULSE_Snapshot(&s);
        CHECK(s.ph[PULSE_PH_A].period_ns == ns_of(period_t), "10Hz 周期 = 100ms");
        CHECK(s.ph[PULSE_PH_A].duty_permille == 500u, "10Hz 占空比 = 50%");
        CHECK((s.ph[PULSE_PH_A].flags & PULSE_F_VALID) != 0u, "10Hz 判定有效");
    }

    /* ============ 3. 无信号 / 边沿太少 ============ */
    SECTION("3) 无信号");
    {
        PULSE_Init();
        PULSE_Start(PULSE_WINDOW_MS);
        g_tick += (uint32_t)PULSE_WINDOW_MS + 1u;
        CHECK(PULSE_Poll() == 1u, "窗口到时返回完成");
        PULSE_Snapshot(&s);
        CHECK((s.ph[PULSE_PH_A].flags & PULSE_F_NOSIGNAL) != 0u, "完全无边沿 → NOSIGNAL");
        CHECK(s.ph[PULSE_PH_A].period_ns == 0u, "没信号时周期为 0（不硬算）");
        CHECK(s.ph[PULSE_PH_A].edges == 0u, "边沿计数为 0");
        CHECK((s.flags & PULSE_SNAP_F_NOSIGNAL) != 0u, "整体标志也带 NOSIGNAL");

        /* 只有 3 个边沿（< PULSE_MIN_EDGES=4）→ 也算没信号 */
        PULSE_Init();
        PULSE_Start(PULSE_WINDOW_MS);
        PULSE_OnEdge(PULSE_PH_A, 1000u);
        PULSE_OnEdge(PULSE_PH_A, 2000u);
        PULSE_OnEdge(PULSE_PH_A, 3000u);
        g_tick += (uint32_t)PULSE_WINDOW_MS + 1u;
        PULSE_Poll();
        PULSE_Snapshot(&s);
        CHECK((s.ph[PULSE_PH_A].flags & PULSE_F_NOSIGNAL) != 0u, "3 个边沿 → 仍判没信号");
        CHECK(s.ph[PULSE_PH_A].edges == 3u, "但边沿计数如实上报");
    }

    /* ============ 4. 丢边沿（异常间隔） ============ */
    SECTION("4) 丢边沿 / 毛刺");
    {
        uint32_t period_t = 3600u;      /* 20kHz */
        uint32_t high_t   = 1800u;

        PULSE_Init();
        PULSE_Start(PULSE_WINDOW_MS);
        /* 正常 5 个周期，中间插一个"少了半个周期"的异常间隔 */
        feed_pwm(PULSE_PH_A, 1000u, period_t, high_t, 5u);
        PULSE_OnEdge(PULSE_PH_A, 1000u + 5u * period_t);            /* 正常下一个上升沿 */
        PULSE_OnEdge(PULSE_PH_A, 1000u + 5u * period_t + 30u);      /* 毛刺：间隔过短 */
        PULSE_OnEdge(PULSE_PH_A, 1000u + 6u * period_t);
        PULSE_OnEdge(PULSE_PH_A, 1000u + 6u * period_t + high_t);
        g_tick += (uint32_t)PULSE_WINDOW_MS + 1u;
        PULSE_Poll();
        PULSE_Snapshot(&s);
        CHECK((s.ph[PULSE_PH_A].flags & PULSE_F_DIRTY) != 0u, "异常间隔被标记 DIRTY");
        CHECK((s.ph[PULSE_PH_A].flags & PULSE_F_VALID) != 0u, "其余正常周期仍算出结果");
        CHECK(s.ph[PULSE_PH_A].period_ns == ns_of(period_t), "毛刺没有污染周期值");

        /* 同一时刻两个边沿（dt=0）也不能算出 0 周期 */
        PULSE_Init();
        PULSE_Start(PULSE_WINDOW_MS);
        feed_pwm(PULSE_PH_A, 1000u, period_t, high_t, 4u);
        PULSE_OnEdge(PULSE_PH_A, 1000u + 4u * period_t);
        PULSE_OnEdge(PULSE_PH_A, 1000u + 4u * period_t);   /* dt = 0 */
        PULSE_OnEdge(PULSE_PH_A, 1000u + 4u * period_t + high_t);
        PULSE_OnEdge(PULSE_PH_A, 1000u + 5u * period_t);
        g_tick += (uint32_t)PULSE_WINDOW_MS + 1u;
        PULSE_Poll();
        PULSE_Snapshot(&s);
        CHECK(s.ph[PULSE_PH_A].period_ns == ns_of(period_t), "dt=0 被丢弃，周期仍正确");
        CHECK((s.ph[PULSE_PH_A].flags & PULSE_F_DIRTY) != 0u, "dt=0 也标记 DIRTY");
    }

    /* ============ 5. 超范围 ============ */
    SECTION("5) 超范围");
    {
        /* 周期 100 tick ≈ 1.39us → 720kHz，超过可测上限 */
        PULSE_Init();
        run_window(100u, 50u, 50u);
        PULSE_Snapshot(&s);
        CHECK((s.ph[PULSE_PH_A].flags & PULSE_F_RANGE) != 0u, "周期过小 → RANGE");
        CHECK((s.ph[PULSE_PH_A].flags & PULSE_F_VALID) != 0u, "但仍给出了读数（供诊断）");
    }

    /* ============ 6. 高速信号把环覆盖 ============ */
    SECTION("6) 环覆盖（边沿数 >> 32）");
    {
        uint32_t period_t = 3600u;
        PULSE_Init();
        PULSE_Start(PULSE_WINDOW_MS);
        feed_pwm_all(0u, period_t, 1800u, 1000u);      /* 2000 个边沿 */
        g_tick += (uint32_t)PULSE_WINDOW_MS + 1u;
        PULSE_Poll();
        PULSE_Snapshot(&s);
        CHECK(s.ph[PULSE_PH_A].edges == 2000u, "总边沿数如实统计（不受环长度限制）");
        CHECK(s.ph[PULSE_PH_A].period_ns == ns_of(period_t), "环被覆盖后仍算得对");
        CHECK(s.ph[PULSE_PH_A].duty_permille == 500u, "占空比仍为 50%");
    }

    /* ============ 7. 窗口/状态机 ============ */
    SECTION("7) 窗口与状态机");
    {
        PULSE_Init();
        CHECK(PULSE_Poll() == 2u, "没在采时 Poll 返回 2");
        PULSE_Start(0u);                     /* 传 0 → 用默认窗口 */
        CHECK(PULSE_IsRunning() == 1u, "Start 后处于采集态");
        CHECK(PULSE_Poll() == 0u, "窗口未到 → 返回 0（进行中）");
        g_tick += (uint32_t)PULSE_WINDOW_MS + 1u;
        CHECK(PULSE_Poll() == 1u, "窗口到 → 返回 1（完成）");
        CHECK(PULSE_IsRunning() == 0u, "完成后自动停止");
        CHECK(PULSE_Poll() == 2u, "完成后不再重复结算");

        /* 强制停止：窗口没到也能停，且不再收边沿 */
        PULSE_Start(PULSE_WINDOW_MS);
        PULSE_Stop();
        PULSE_OnEdge(PULSE_PH_A, 12345u);
        CHECK(PULSE_IsRunning() == 0u, "Stop 后不在采集态");
        g_tick += (uint32_t)PULSE_WINDOW_MS + 1u;
        PULSE_Poll();
        PULSE_Snapshot(&s);
        CHECK(s.ph[PULSE_PH_A].edges == 0u, "停止后灌进来的边沿被忽略");
    }

    /* ============ 8. 线格式（45B） ============ */
    SECTION("8) pulse_pack 线格式");
    {
        uint8_t buf[PULSE_PACK_LEN + 8];
        uint16_t n;
        pulse_snapshot_t back;

        CHECK(PULSE_PACK_LEN == 45u, "布局长度 45B");
        CHECK(PULSE_PACK_LEN <= 64u, "装得进链路单帧(64)");
        CHECK(PULSE_PACK_RESP_LEN == 91u, "0x000F 应答总长 91B(状态+本片45+对侧45)");
        CHECK(PULSE_PACK_RESP_LEN <= 256u, "装得进 PC 单帧(256)");

        PULSE_Init();
        run_window(3600u, 900u, 50u);          /* 20kHz 25% */
        PULSE_Snapshot(&s);

        n = PulsePack_Pack(buf, sizeof(buf), &s);
        CHECK(n == PULSE_PACK_LEN, "打包长度 45");
        CHECK(buf[0] == 1u, "ver = 1");
        CHECK(buf[2] == (uint8_t)(PULSE_WINDOW_MS >> 8), "window_ms 大端在 [2..3]");
        CHECK(buf[4] == (uint8_t)PULSE_PH_COUNT, "phase_count = 3");
        CHECK(buf[6] == (uint8_t)(s.ph[0].period_ns >> 24), "A 相周期起点在 [6]");
        CHECK(buf[6 + 13] == (uint8_t)(s.ph[1].period_ns >> 24), "B 相从 [19] 开始");
        CHECK(buf[6 + 26] == (uint8_t)(s.ph[2].period_ns >> 24), "C 相从 [32] 开始");

        memset(&back, 0, sizeof(back));
        CHECK(PulsePack_Unpack(buf, n, &back) == 1u, "解包成功");
        for (i = 0; i < (int)PULSE_PH_COUNT; i++) {
            CHECK(back.ph[i].period_ns == s.ph[i].period_ns &&
                  back.ph[i].high_ns == s.ph[i].high_ns &&
                  back.ph[i].duty_permille == s.ph[i].duty_permille &&
                  back.ph[i].edges == s.ph[i].edges &&
                  back.ph[i].flags == s.ph[i].flags, "9 个字段逐相往返一致");
        }
        CHECK(PulsePack_Pack(buf, 10u, &s) == 0u, "缓冲太小被拒");
        CHECK(PulsePack_Pack(0, sizeof(buf), &s) == 0u, "NULL 被拒");
        CHECK(PulsePack_Unpack(buf, 10u, &back) == 0u, "长度不足被拒");
        CHECK(PulsePack_Pack(buf, sizeof(buf), 0) == PULSE_PACK_LEN,
              "空指针按无数据打包（不崩）");
        CHECK(buf[1] == PULSE_SNAP_F_NOSIGNAL, "空指针打包带无信号标志");
    }

    /* ============ 9. 两端对比（U1 输出端 vs U2 输入端） ============ */
    SECTION("9) 两端对比");
    {
        pulse_snapshot_t out_side, in_side;
        uint8_t diff[PULSE_PH_COUNT];

        /* 两端都有、频率一致 → 无差异 */
        PULSE_Init();
        run_window(3600u, 1800u, 50u);
        PULSE_Snapshot(&out_side);
        in_side = out_side;
        CHECK(PulsePack_Compare(&out_side, &in_side, diff) == 0u, "两端一致 → 0 个差异");
        CHECK(diff[PULSE_PH_A] == 0u, "差异数组清零");

        /* 频率差 2%（超过 1% 容差）→ 差异 */
        in_side = out_side;
        in_side.ph[PULSE_PH_B].period_ns = (uint32_t)(out_side.ph[PULSE_PH_B].period_ns * 102u / 100u);
        CHECK(PulsePack_Compare(&out_side, &in_side, diff) == 1u, "周期差 2% → 1 个差异");
        CHECK(diff[PULSE_PH_B] == 1u, "差异落在 B 相");
        CHECK(diff[PULSE_PH_A] == 0u, "A 相不受影响");

        /* 频率差 0.5%（容差内）→ 不算差异 */
        in_side = out_side;
        in_side.ph[PULSE_PH_C].period_ns = (uint32_t)(out_side.ph[PULSE_PH_C].period_ns * 1005u / 1000u);
        CHECK(PulsePack_Compare(&out_side, &in_side, diff) == 0u, "周期差 0.5% → 容差内");

        /* 一端有信号、另一端没有 → 差异（驱动级/接线问题） */
        in_side = out_side;
        in_side.ph[PULSE_PH_A].flags = PULSE_F_NOSIGNAL;
        in_side.ph[PULSE_PH_A].period_ns = 0u;
        in_side.ph[PULSE_PH_A].edges = 0u;
        CHECK(PulsePack_Compare(&out_side, &in_side, diff) == 1u, "输入端没信号 → 差异");
        CHECK(diff[PULSE_PH_A] == 1u, "差异落在 A 相");

        /* 占空比不同**不判**（死区/驱动延时会让两端占空比天然不同） */
        in_side = out_side;
        in_side.ph[PULSE_PH_A].duty_permille = 400u;
        in_side.ph[PULSE_PH_A].high_ns = out_side.ph[PULSE_PH_A].high_ns / 2u;
        CHECK(PulsePack_Compare(&out_side, &in_side, diff) == 0u, "占空比差异不判");

        /* 两端都没信号 → 不算差异（被测板没输出，由 NOSIGNAL 标志体现） */
        {
            pulse_snapshot_t none;
            memset(&none, 0, sizeof(none));
            for (i = 0; i < (int)PULSE_PH_COUNT; i++) {
                none.ph[i].flags = PULSE_F_NOSIGNAL;
            }
            CHECK(PulsePack_Compare(&none, &none, diff) == 0u, "两端都无信号 → 不算差异");
            CHECK(PulsePack_Compare(0, &none, diff) == 0u, "空指针安全");
        }
    }

    printf("\n========================================\n");
    printf(" checks: %d, failed: %d\n", g_checks, g_fails);
    printf(" %s\n", (g_fails == 0) ? "ALL PASS" : "FAILURES PRESENT");
    printf("========================================\n");
    return (g_fails == 0) ? 0 : 1;
}
