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
        uint16_t ok24  = mv_to_counts(24000.0, 47.0 / 371.0);
        uint16_t hi24  = mv_to_counts(26000.0, 47.0 / 371.0);  /* > 25000 上限 */

        reset_all(0u);
        MEAS_TestInject(MEAS_CH_V24, ok24);
        for (i = 0; i < (int)MEAS_CH_COUNT; i++) {          /* 其余通道也给正常值 */
            if (i != (int)MEAS_CH_V24) {
                MEAS_TestInject((meas_id_t)i, 2000u);
            }
        }
        MEAS_Poll();
        check(MEAS_IsOk(MEAS_CH_V24) == 0u, "not ok after 1 poll (needs stability count)");
        for (i = 0; i < (int)MEAS_STABLE_N; i++) {
            MEAS_Poll();
        }
        check(MEAS_IsOk(MEAS_CH_V24) == 1u, "ok after MEAS_STABLE_N polls in range");

        MEAS_TestInject(MEAS_CH_V24, hi24);
        for (i = 0; i < 400; i++) {
            MEAS_Poll();                                    /* 等 EMA 爬过门限 */
        }
        check(MEAS_IsOk(MEAS_CH_V24) == 0u, "26V is above 25V limit -> not ok");
        check((MEAS_GetFaultMask() & (1u << MEAS_CH_V24)) != 0u, "fault mask bit for V24");
        check(MEAS_AllOk() == 0u, "AllOk is false while V24 is faulted");
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
        check(MEAS_CrossCheck() == MEAS_CC_DISABLED, "cross-check reports disabled (U2 not wired yet)");
    }

    printf("\n============================================================\n");
    printf(" RESULT: %d passed, %d failed\n", g_pass, g_fail);
    printf("============================================================\n");
    return (g_fail == 0) ? 0 : 1;
}
