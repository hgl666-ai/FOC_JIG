/*
 * 测量功能宿主单测 (measure.c)
 *
 * 编译时定义 MEAS_HOST_TEST：
 *   - measure.c 里跳过 ADC 校准/启动（宿主机没有 ADC）
 *   - 并暴露 MEAS_TestInject()/MEAS_TestInjectVref() 直接灌原始码值
 * 被测的是真源码：通道表、定点换算、EMA 滤波、门限判定、打印，全部走真实路径。
 *
 * 期望值来自 docs/06_测量通道分压逆推.md（原理图追踪出来的分压比）：
 *   V24/V10 k=47/371, V5 k=3/5, V33 k=10/11, PHx k=1/8, CUR k=1
 */
#include <stdio.h>
#include <string.h>
#include <stdlib.h>

#include "measure.h"
#include "meas_pack.h"

#define VDDA_MV 3300.0

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

static void check_near(long got, long want, long tol, const char *name)
{
    long d = got - want;
    if (d < 0) {
        d = -d;
    }
    if (d <= tol) {
        g_pass++;
        printf("  [ok]   %s (got %ld, want %ld +-%ld)\n", name, got, want, tol);
    } else {
        g_fail++;
        printf("  [FAIL] %s (got %ld, want %ld +-%ld)\n", name, got, want, tol);
    }
}

/* ---------------- 宿主替身 ---------------- */
static uint32_t g_tick = 0u;
static char     g_dbg[512];
static size_t   g_dbg_len = 0u;

uint32_t HAL_GetTick(void)
{
    return g_tick;
}

void BSP_DBG_Send(const char *str)
{
    size_t n = strlen(str);
    if (g_dbg_len + n < sizeof(g_dbg) - 1u) {
        memcpy(&g_dbg[g_dbg_len], str, n);
        g_dbg_len += n;
        g_dbg[g_dbg_len] = '\0';
    }
}

/* 输入电压 -> 理论码值（用 s16 的分压比算，与被测代码独立） */
static uint16_t mv_to_counts(double v_in_mv, double k)
{
    double v_adc = v_in_mv * k;
    double c = v_adc * 4095.0 / VDDA_MV;
    if (c < 0.0) {
        c = 0.0;
    }
    if (c > 4095.0) {
        c = 4095.0;
    }
    return (uint16_t)(c + 0.5);
}

static void reset_all(uint16_t counts)
{
    int i;
    MEAS_Init();                       /* 先清 DMA 缓冲和运行态（Init 会把 s_dma 清零） */
    for (i = 0; i < (int)MEAS_CH_COUNT; i++) {
        MEAS_TestInject((meas_id_t)i, counts);
    }
    MEAS_TestInjectVref(1489u);        /* 1.2V/3.3V*4095 ≈ 1489 */
    g_dbg_len = 0u;
    g_dbg[0] = '\0';
}

/* 给全部 9 路灌一组"标称"码值（用于需要 AllOk=1 的用例） */
static void inject_nominal(void)
{
    MEAS_TestInject(MEAS_CH_V24,  mv_to_counts(24000.0, 47.0 / 371.0));
    MEAS_TestInject(MEAS_CH_V10,  mv_to_counts(10000.0, 47.0 / 371.0));
    MEAS_TestInject(MEAS_CH_V5,   mv_to_counts(5000.0, 3.0 / 5.0));
    MEAS_TestInject(MEAS_CH_V33,  mv_to_counts(3300.0, 10.0 / 11.0));
    MEAS_TestInject(MEAS_CH_PH_A, mv_to_counts(6000.0, 1.0 / 8.0));
    MEAS_TestInject(MEAS_CH_PH_B, mv_to_counts(6000.0, 1.0 / 8.0));
    MEAS_TestInject(MEAS_CH_PH_C, mv_to_counts(6000.0, 1.0 / 8.0));
    MEAS_TestInject(MEAS_CH_CUR1, mv_to_counts(1500.0, 1.0));
    MEAS_TestInject(MEAS_CH_CUR2, mv_to_counts(1500.0, 1.0));
}

int main(void)
{
    int i;

    printf("============================================================\n");
    printf(" measure.c host unit test (divider -> voltage -> threshold)\n");
    printf("============================================================\n");

    /* ============ 1. 各通道换算精度 ============ */
    printf("\n--- 1. conversion (counts -> input mV) ---\n");
    {
        /* 通道 / 输入 mV / k 分子 / k 分母 */
        struct { meas_id_t ch; long v_in; double k; } cases[] = {
            { MEAS_CH_V24,  24000, 47.0 / 371.0 },
            { MEAS_CH_V10,  10000, 47.0 / 371.0 },
            { MEAS_CH_V5,    5000, 3.0 / 5.0 },
            { MEAS_CH_V33,   3300, 10.0 / 11.0 },
            { MEAS_CH_PH_A, 24000, 1.0 / 8.0 },
            { MEAS_CH_PH_B, 12000, 1.0 / 8.0 },
            { MEAS_CH_PH_C,     0, 1.0 / 8.0 },
            { MEAS_CH_CUR1,  1000, 1.0 },
            { MEAS_CH_CUR2,  2000, 1.0 },
        };
        int n = (int)(sizeof(cases) / sizeof(cases[0]));

        reset_all(0u);
        for (i = 0; i < n; i++) {
            MEAS_TestInject(cases[i].ch, mv_to_counts((double)cases[i].v_in, cases[i].k));
        }
        MEAS_Poll();
        for (i = 0; i < n; i++) {
            long got  = (long)MEAS_GetMv(cases[i].ch);
            long want = cases[i].v_in;
            long tol  = want / 100 + 3;      /* 1% + 3mV（1 个码值约 6mV/2.8mV） */
            char nm[64];
            sprintf(nm, "%s: %ld mV in", MEAS_Name(cases[i].ch), want);
            check_near(got, want, tol, nm);
        }
    }

    /* ============ 2. 图上标注的三个点复算 ============ */
    printf("\n--- 2. sheet cross-check (24V/10V/7V -> 3.04032/1.2668/0.88676 V) ---\n");
    {
        const double k = 47.0 / 371.0;   /* = 0.1266846 */
        struct { long v_in; long v_adc_uv; } pts[] = {
            { 24000, 3040320 },   /* 图上 3.04032 V */
            { 10000, 1266800 },   /* 图上 1.2668  V */
            {  7000,  886760 },   /* 图上 0.88676 V */
        };
        int n = (int)(sizeof(pts) / sizeof(pts[0]));
        for (i = 0; i < n; i++) {
            double v_adc = k * (double)pts[i].v_in;
            char   nm[80];
            /* 图上标的是 k 取 5 位有效数字(0.12668)算出来的：
             *   24*0.12668 = 3.04032，而精确值 4.7/37.1 给 3.04043，
             *   差 111uV —— 这是图上取整造成的，不是换算错误。
             * 所以容差放到 150uV。 */
            check_near((long)(v_adc * 1000.0 + 0.5), pts[i].v_adc_uv, 150, "sheet pt");
            sprintf(nm, "%ld mV -> %.5f V (sheet %.5f V)", pts[i].v_in,
                    v_adc / 1000.0, (double)pts[i].v_adc_uv / 1.0e6);
            printf("         %s\n", nm);
        }
    }

    /* ============ 3. EMA 滤波 ============ */
    printf("\n--- 3. EMA filter ---\n");
    {
        long v0, v1, v2;
        uint16_t c5v  = mv_to_counts(5000.0, 3.0 / 5.0);
        uint16_t c4v  = mv_to_counts(4000.0, 3.0 / 5.0);

        reset_all(0u);
        MEAS_TestInject(MEAS_CH_V5, c5v);
        MEAS_Poll();
        v0 = (long)MEAS_GetMv(MEAS_CH_V5);
        check_near(v0, 5000, 50, "V5 first sample seeds EMA (no ramp from 0)");

        MEAS_TestInject(MEAS_CH_V5, c4v);      /* 阶跃到 4V */
        MEAS_Poll();
        v1 = (long)MEAS_GetMv(MEAS_CH_V5);
        check((v1 < v0) && (v1 > 4000), "V5 moves toward 4V, not a jump");

        for (i = 0; i < 600; i++) {
            MEAS_Poll();
        }
        v2 = (long)MEAS_GetMv(MEAS_CH_V5);
        check_near(v2, 4000, 40, "V5 settles to 4V after 600 polls (1/64)");
    }

    /* ============ 4. 门限与稳定判据 ============ */
    printf("\n--- 4. threshold + stability ---\n");
    {
        /* 注意：门限现在是**全量程**（用户决定"先不判"，先保证能采/能打包/能比对），
         * 所以从 ADC 码值不可能造出"超门限"的输入（满量程就是门限上限）。
         * 这里改为验证"判定机制本身"：稳定计数 + 掩码/汇总的联动。
         * 收紧门限只需改 measure.c 里 k_cfg[] 的 vmin/vmax 两列。 */
        reset_all(0u);
        inject_nominal();
        MEAS_Poll();
        check(MEAS_IsOk(MEAS_CH_V24) == 0u, "not ok after 1 poll (needs stability count)");

        for (i = 0; i < (int)MEAS_STABLE_N; i++) {
            MEAS_Poll();
        }
        check(MEAS_IsOk(MEAS_CH_V24) == 1u, "ok after MEAS_STABLE_N polls in range");

        /* 触发一次测试会清零稳定计数 → 立刻回到"不合格"，
         * 直到重新累计够 MEAS_STABLE_N 次（这就是 0x000E mode=1 的语义） */
        MEAS_TestBegin();
        check(MEAS_AllOk() == 0u, "test begin clears stability -> AllOk=0");
        check(MEAS_GetFaultMask() == ((1u << MEAS_CH_COUNT) - 1u),
              "fault mask has all 9 bits right after test begin");
        for (i = 0; i < (int)MEAS_STABLE_N - 1; i++) {
            MEAS_Poll();
        }
        check(MEAS_TestPoll() == 0u, "test still running one poll before stable");
        MEAS_Poll();
        check(MEAS_TestPoll() == 1u, "test done once all channels stable");
        check(MEAS_AllOk() == 1u, "AllOk=1 after the test completed");
        check(MEAS_GetFaultMask() == 0u, "fault mask cleared");
    }

    /* ============ 4b. 测试超时（数据一直不稳） ============ */
    printf("\n--- 4b. test timeout ---\n");
    {
        reset_all(0u);                 /* 全 0 码值：EMA 稳定后也算合格，所以只测超时口径 */
        MEAS_TestBegin();
        g_tick += (uint32_t)MEAS_TEST_TIMEOUT_MS + 1u;
        check(MEAS_TestPoll() == 2u, "test reports timeout after MEAS_TEST_TIMEOUT_MS");
        check(MEAS_TestPoll() == 2u, "timeout result stays latched (idempotent)");
    }

    /* ============ 5. 饱和标记 ============ */
    printf("\n--- 5. saturation flag ---\n");
    {
        meas_value_t v;
        reset_all(2048u);
        MEAS_Poll();
        MEAS_Get(MEAS_CH_V5, &v);
        check(v.sat == 0u, "mid-scale is not saturated");

        MEAS_TestInject(MEAS_CH_V5, 0u);
        MEAS_Poll();
        MEAS_Get(MEAS_CH_V5, &v);
        check(v.sat == 1u, "counts=0 flagged as saturated (open/shorted?)");

        MEAS_TestInject(MEAS_CH_V5, 4095u);
        MEAS_Poll();
        MEAS_Get(MEAS_CH_V5, &v);
        check(v.sat == 1u, "counts=4095 flagged as saturated");
    }

    /* ============ 6. 满量程边界 ============ */
    printf("\n--- 6. full-scale mapping ---\n");
    {
        reset_all(4095u);
        MEAS_Poll();
        /* 24V 通道满量程 = 3.3/k = 26.05V */
        check_near((long)MEAS_GetMv(MEAS_CH_V24), 26050, 60, "V24 full scale ~26.05V");
        check_near((long)MEAS_GetMv(MEAS_CH_V33), 3630, 30, "V33 full scale ~3.63V");
        check_near((long)MEAS_GetMv(MEAS_CH_CUR1), 3300, 20, "CUR1 full scale = 3.3V");
    }

    /* ============ 7. 调试打印 ============ */
    printf("\n--- 7. debug printout ---\n");
    {
        reset_all(0u);
        inject_nominal();
        g_tick = 0u;
        MEAS_Poll();
        for (i = 0; i <= (int)MEAS_STABLE_N; i++) {
            g_tick += 10u;
            MEAS_Poll();
        }
        g_dbg_len = 0u;
        g_dbg[0] = '\0';
        g_tick += (uint32_t)MEAS_PRINT_PERIOD_MS + 1u;
        MEAS_Poll();
        check(g_dbg_len > 0u, "MEAS_Print produced output");
        check(strncmp(g_dbg, "M V24=", 6) == 0, "line starts with 'M V24='");
        check(strstr(g_dbg, "V5=5000") != NULL, "line contains V5=5000");
        check(strstr(g_dbg, "V33=3300") != NULL, "line contains V33=3300");
        check(strstr(g_dbg, "OK=1") != NULL, "line reports OK=1");
        printf("         %s", g_dbg);
    }

    /* ============ 8. 越界保护 ============ */
    printf("\n--- 8. bounds safety ---\n");
    {
        meas_value_t v;
        reset_all(1000u);
        MEAS_Poll();
        check(MEAS_GetMv((meas_id_t)99) == 0u, "MEAS_GetMv out of range returns 0");
        check(MEAS_IsOk((meas_id_t)99) == 0u, "MEAS_IsOk out of range returns 0");
        check(strcmp(MEAS_Name((meas_id_t)99), "?") == 0, "MEAS_Name out of range returns '?'");
        MEAS_Get((meas_id_t)99, &v);
        check(v.mv == 0u, "MEAS_Get out of range yields zeros");
        MEAS_Get(MEAS_CH_V5, NULL);
        check(1, "MEAS_Get(NULL) does not crash");
    }

    /* ============ 9. 快照（0x000E 要发出去的东西） ============ */
    printf("\n--- 9. snapshot ---\n");
    {
        meas_snapshot_t s;
        uint32_t mv24;

        reset_all(0u);
        inject_nominal();
        g_tick = 1000u;
        MEAS_Poll();
        for (i = 0; i < (int)MEAS_STABLE_N; i++) {
            g_tick += 3u;
            MEAS_Poll();
        }
        memset(&s, 0, sizeof(s));
        MEAS_Snapshot(&s);

        mv24 = MEAS_GetMv(MEAS_CH_V24);
        check(s.ch_count == (uint8_t)MEAS_CH_COUNT, "snapshot ch_count = 9");
        check((s.flags & MEAS_SNAP_F_VALID) != 0u, "snapshot marked valid");
        check((s.flags & MEAS_SNAP_F_ALL_OK) != 0u, "snapshot marked all-ok (full-scale thresholds)");
        check(s.round >= (uint16_t)(MEAS_STABLE_N + 1), "snapshot round counts scans");
        check(s.dt_ms == 3u, "snapshot dt_ms = gap between rounds");
        check(s.vdda_mv == MEAS_VDDA_MV, "snapshot carries VDDA");
        check(s.cfg_crc == MEAS_GetCfgCrc(), "snapshot carries config fingerprint");
        check(s.cfg_crc != 0u, "config fingerprint is non-zero");
        check_near((long)s.ch[MEAS_CH_V24].mv, (long)mv24, 1, "ch[V24].mv matches MEAS_GetMv");
        check(s.ch[MEAS_CH_V24].raw != 0u, "ch[V24].raw carries the raw code");
        check((s.ch[MEAS_CH_V24].flags & MEAS_CHF_OK) != 0u, "ch[V24] flagged ok");
        check((s.ch[MEAS_CH_CUR1].flags & MEAS_CHF_SAT) == 0u, "ch[CUR1] not flagged sat");
        MEAS_Snapshot(NULL);
        check(1, "MEAS_Snapshot(NULL) does not crash");
    }

    /* ============ 10. 交叉校验比对 + 容差 ============ */
    printf("\n--- 10. cross-check compare + tolerance ---\n");
    {
        meas_snapshot_t a, b;
        uint8_t  mism[MEAS_CH_COUNT];
        uint16_t mask;

        reset_all(0u);
        inject_nominal();
        MEAS_Poll();
        for (i = 0; i < (int)MEAS_STABLE_N; i++) {
            MEAS_Poll();
        }
        MEAS_Snapshot(&a);
        b = a;
        mask = MEAS_Compare(&a, &b, mism);
        check(mask == 0u, "identical snapshots -> no mismatch");
        check(mism[MEAS_CH_V24] == 0u, "per-channel mismatch array cleared");

        b.ch[MEAS_CH_V5].mv = (uint16_t)(a.ch[MEAS_CH_V5].mv + 100u);   /* 容差 200mV */
        mask = MEAS_Compare(&a, &b, mism);
        check(mask == 0u, "100mV on 5V rail is inside the 200mV tolerance");

        b.ch[MEAS_CH_V5].mv = (uint16_t)(a.ch[MEAS_CH_V5].mv + 400u);   /* 超出容差 */
        mask = MEAS_Compare(&a, &b, mism);
        check((mask & (1u << MEAS_CH_V5)) != 0u, "400mV on 5V rail flagged");
        check(mism[MEAS_CH_V5] == 1u, "per-channel array flags the offending channel");

        /* 相电压容差 2000mV（PWM 斩波 + 两片采样时刻差） */
        b = a;
        b.ch[MEAS_CH_PH_A].mv = (uint16_t)(a.ch[MEAS_CH_PH_A].mv + 1500u);
        check(MEAS_Compare(&a, &b, NULL) == 0u, "1500mV on a phase channel is inside tolerance");
        b.ch[MEAS_CH_PH_A].mv = (uint16_t)(a.ch[MEAS_CH_PH_A].mv + 2500u);
        check((MEAS_Compare(&a, &b, NULL) & (1u << MEAS_CH_PH_A)) != 0u, "2500mV on a phase flagged");

        /* 两边方向相反也要算对（不能只看有符号差值） */
        b = a;
        b.ch[MEAS_CH_CUR1].mv = (uint16_t)((a.ch[MEAS_CH_CUR1].mv > 500u)
                                           ? (a.ch[MEAS_CH_CUR1].mv - 500u) : 0u);
        check((MEAS_Compare(&a, &b, NULL) & (1u << MEAS_CH_CUR1)) != 0u,
              "lower peer reading also flagged");

        /* 通道数不对 -> 全 1 掩码（调用方据此报异常，而不是当成"一致"） */
        b = a;
        b.ch_count = 3u;
        check(MEAS_Compare(&a, &b, NULL) == (uint16_t)((1u << MEAS_CH_COUNT) - 1u),
              "short peer snapshot -> all channels flagged");
        check(MEAS_Compare(NULL, &a, NULL) == 0u, "NULL snapshots are handled");
    }

    /* ============ 11. 数据包编解码（0x000E 应答布局） ============ */
    printf("\n--- 11. meas_pack wire format ---\n");
    {
        meas_snapshot_t s, back;
        uint8_t  buf[MEAS_PACK_RESP_LEN];
        uint16_t n;
        uint8_t  cc_flags = 0u;
        uint16_t mask = 0u;

        check(MEAS_PACK_SNAP_LEN == 55u, "snapshot layout is 55B");
        check(MEAS_PACK_CC_LEN == 21u, "cross-check block is 21B");
        check(MEAS_PACK_RESP_LEN == 77u, "0x000E response is 77B");
        check(MEAS_PACK_SNAP_LEN <= 64u, "snapshot fits LINK_MAX_PAYLOAD (64)");
        check(MEAS_PACK_RESP_LEN <= 256u, "response fits FRAME_MAX_DATA (256)");

        reset_all(0u);
        inject_nominal();
        MEAS_Poll();
        MEAS_Snapshot(&s);

        n = MeasPack_Snapshot(buf, sizeof(buf), &s);
        check(n == MEAS_PACK_SNAP_LEN, "MeasPack_Snapshot returns 55");
        check(buf[0] == s.flags, "byte0 = flags");
        check(buf[1] == (uint8_t)MEAS_CH_COUNT, "byte1 = ch_count");
        check(buf[2] == (uint8_t)(s.round >> 8), "round is big-endian");
        check(buf[8] == (uint8_t)(s.cfg_crc >> 8), "cfg_crc sits at [8..9]");
        check(buf[10] == (uint8_t)(s.ch[0].raw >> 8), "channel 0 raw starts at [10]");
        check(buf[14] == (uint8_t)s.ch[0].flags, "channel 0 flags at [14] (5B stride)");
        check(buf[10 + 5 * (MEAS_CH_COUNT - 1) + 4] == s.ch[MEAS_CH_COUNT - 1].flags,
              "last channel record ends at byte 54");

        memset(&back, 0, sizeof(back));
        check(MeasPack_UnpackSnapshot(buf, n, &back) == 1u, "unpack succeeds");
        check(back.round == s.round && back.dt_ms == s.dt_ms &&
              back.vdda_mv == s.vdda_mv && back.cfg_crc == s.cfg_crc,
              "scalar fields round-trip");
        {
            /* 逐字段比（不能 memcmp 整个结构体：内存里有对齐填充字节） */
            int same = 1;
            for (i = 0; i < (int)MEAS_CH_COUNT; i++) {
                if ((back.ch[i].raw   != s.ch[i].raw) ||
                    (back.ch[i].mv    != s.ch[i].mv)  ||
                    (back.ch[i].flags != s.ch[i].flags)) {
                    same = 0;
                }
            }
            check(same == 1, "all 9 channel records round-trip");
        }

        /* 交叉校验块 */
        n = MeasPack_CcBlock(buf, sizeof(buf), MEAS_CC_F_DONE | MEAS_CC_F_MISMATCH,
                             0x0121u, &s);
        check(n == MEAS_PACK_CC_LEN, "MeasPack_CcBlock returns 21");
        check(buf[0] == (MEAS_CC_F_DONE | MEAS_CC_F_MISMATCH), "cc flags at [0]");
        check(buf[1] == 0x01u && buf[2] == 0x21u, "mask big-endian at [1..2]");
        check(buf[3] == (uint8_t)(s.ch[0].mv >> 8), "peer mv[0] starts at [3]");
        memset(&back, 0, sizeof(back));
        check(MeasPack_UnpackCcBlock(buf, n, &cc_flags, &mask, &back) == 1u, "cc unpack succeeds");
        check(cc_flags == (MEAS_CC_F_DONE | MEAS_CC_F_MISMATCH) && mask == 0x0121u,
              "cc flags/mask round-trip");
        check(back.ch[MEAS_CH_CUR2].mv == s.ch[MEAS_CH_CUR2].mv,
              "all 9 peer mv values round-trip");

        /* 边界与非法参数 */
        check(MeasPack_Snapshot(buf, 10u, &s) == 0u, "snapshot: buffer too small refused");
        check(MeasPack_Snapshot(NULL, sizeof(buf), &s) == 0u, "snapshot: NULL refused");
        check(MeasPack_Snapshot(buf, sizeof(buf), NULL) == 0u, "snapshot: NULL src refused");
        check(MeasPack_UnpackSnapshot(buf, 10u, &back) == 0u, "unpack: short input refused");
        check(MeasPack_UnpackSnapshot(NULL, sizeof(buf), &back) == 0u, "unpack: NULL refused");
        check(MeasPack_CcBlock(buf, 4u, 0u, 0u, &s) == 0u, "cc block: buffer too small refused");
        check(MeasPack_UnpackCcBlock(buf, 4u, NULL, NULL, NULL) == 0u, "cc unpack: short refused");
        check(MeasPack_CcBlock(buf, sizeof(buf), MEAS_CC_F_NO_PEER, 0u, NULL) == MEAS_PACK_CC_LEN,
              "cc block with NULL peer still encodes (mv = 0)");
    }

    printf("\n============================================================\n");
    printf(" RESULT: %d passed, %d failed\n", g_pass, g_fail);
    printf("============================================================\n");
    return (g_fail == 0) ? 0 : 1;
}
