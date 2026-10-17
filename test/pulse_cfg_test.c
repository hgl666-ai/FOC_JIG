/*
 * pulse_cfg.c（TIM 输入捕获的**硬件层**）宿主测试 —— 走寄存器级替身 test/timsim
 *
 * 为什么单独开这一层：pulse_cfg.c 是"ISR ↔ 主循环"里唯一碰寄存器的模块，
 * 而它此前**没有被任何宿主测试编译过**（app_test.c 里是把 PULSE_CfgStart
 * 整个换成替身函数），于是"开窗临界区"这条最要命的顺序从来没被验证过。
 * 本文件把 TIM 寄存器、NVIC 的使能位/排队位、PRIMASK 都做成可观测对象，
 * 逐条断言开窗时那套"关 NVIC → 清 pending → 复位 → 再打开"的动作。
 *
 * 覆盖点：
 *   1) PULSE_CfgInit：计数器自由运行 / 初始化时不响应中断 / 三个向量已使能
 *   2) 开窗的**临界区结构与顺序**（docs/03 §4 的 D4/D5 核心保证）：
 *      9 条 NVIC 动作的次序、全部发生在关中断状态下、收尾必须放开中断
 *   3) 开窗会清掉"已排队但还没执行"的中断请求（否则它会在新窗口里跑起来）
 *   4) 边沿入队 + 周期/占空比结算（三相都走一遍：TIM3_CH2 / TIM3_CH1 / TIM1_CH1）
 *   5) UIF 与捕获同拍时的 +1 补偿（先溢出后捕获，时间戳高位要 +1）
 *   6) 真实溢出把时间戳高位抬高 65536，跨溢出两沿的差值仍然正确
 *   7) 重开窗口不许残留上一窗口的边沿计数（U2 的 ARM 路径会这么干）
 *   8) 关窗 / 窗口到点的收尾动作
 *
 * ⚠ 本测试**不能**证明的事（写清楚，别当成证明过了）：
 *   - "中断恰好插在 PULSE_Start 清环的中间"这种真并发交错，单线程宿主复现不了。
 *     能证明的是**结构保证**：走到 PULSE_Start 时捕获中断一定处于
 *     "NVIC 已关 + pending 已清 + PRIMASK=1"的状态，所以那种交错不可能发生。
 *   - 替身的 SR 是**普通存储**，不是真芯片的 rc_w0（写 1 不生效）语义；
 *     而且读 CCR 不会自动清 CCxIF。所以：断言 SR 之前必须先由代码显式写过它
 *     （PULSE_CfgStart 会写 0），测试注入也一律显式设置 SR。
 */
#include <stdio.h>
#include <string.h>
#include <stdint.h>

#include "main.h"            /* ← 本文件把自己当成一个"宿主 TU"：它会引到 test/timsim 的替身 */
#include "pulse_cfg.h"
#include "pulse_cap.h"

/* 中断入口（固件里由启动文件的向量表调用） */
extern void TIM3_IRQHandler(void);
extern void TIM1_CC_IRQHandler(void);
extern void TIM1_UP_IRQHandler(void);

static int g_checks = 0;
static int g_fails  = 0;

#define CHECK(cond, msg)                                                     \
    do {                                                                     \
        g_checks++;                                                          \
        if (!(cond)) { printf("  FAIL: %s  (line %d)\n", msg, __LINE__); g_fails++; } \
    } while (0)

#define SECTION(t) printf("\n[%s]\n", t)

/*==============================================================================
 * 替身的实体部分
 *============================================================================*/
static TIM_TypeDef s_tim1_regs;
static TIM_TypeDef s_tim3_regs;

TIM_TypeDef *const TIM1 = &s_tim1_regs;
TIM_TypeDef *const TIM3 = &s_tim3_regs;

volatile uint8_t g_primask;
volatile uint8_t g_nvic_en[TIMSIM_IRQ_N];
volatile uint8_t g_nvic_pend[TIMSIM_IRQ_N];
timsim_ev_t      g_nvic_log[TIMSIM_LOG_MAX];
int              g_nvic_log_n;

static uint32_t g_tick;

uint32_t HAL_GetTick(void) { return g_tick; }

void timsim_disable_irq(void) { g_primask = 1u; }
void timsim_enable_irq(void)  { g_primask = 0u; }

static void log_ev(uint8_t ev, IRQn_Type irqn)
{
    if (g_nvic_log_n < TIMSIM_LOG_MAX) {
        g_nvic_log[g_nvic_log_n].ev      = ev;
        g_nvic_log[g_nvic_log_n].irq     = (uint8_t)irqn;
        g_nvic_log[g_nvic_log_n].primask = g_primask;
        g_nvic_log_n++;
    }
}

void HAL_NVIC_SetPriority(IRQn_Type irqn, uint32_t pre, uint32_t sub)
{
    (void)pre;
    (void)sub;
    log_ev(TIMSIM_EV_SETPRIO, irqn);
}

void HAL_NVIC_EnableIRQ(IRQn_Type irqn)
{
    g_nvic_en[irqn] = 1u;
    log_ev(TIMSIM_EV_ENABLE, irqn);
}

/* ⚠ 只清"使能位"：**不动** pending 位 —— 真芯片就是这样（D4 的关键） */
void HAL_NVIC_DisableIRQ(IRQn_Type irqn)
{
    g_nvic_en[irqn] = 0u;
    log_ev(TIMSIM_EV_DISABLE, irqn);
}

void NVIC_ClearPendingIRQ(IRQn_Type irqn)
{
    g_nvic_pend[irqn] = 0u;
    log_ev(TIMSIM_EV_CLEARPEND, irqn);
}

/*==============================================================================
 * 测试用的小工具
 *============================================================================*/
static void sim_reset(void)
{
    memset(&s_tim1_regs, 0, sizeof(s_tim1_regs));
    memset(&s_tim3_regs, 0, sizeof(s_tim3_regs));
    memset((void *)g_nvic_en, 0, sizeof(g_nvic_en));
    memset((void *)g_nvic_pend, 0, sizeof(g_nvic_pend));
    g_nvic_log_n = 0;
    g_primask    = 0u;
    g_tick       = 1000u;
    PULSE_Init();
}

/* 交付一次"排队中的中断"（真硬件里由 NVIC 做；这里显式调一次，好断言） */
static int sim_serve(IRQn_Type irqn)
{
    if ((g_nvic_en[irqn] == 0u) || (g_nvic_pend[irqn] == 0u)) {
        return 0;
    }
    g_nvic_pend[irqn] = 0u;
    switch (irqn) {
    case TIM3_IRQn:    TIM3_IRQHandler();    break;
    case TIM1_CC_IRQn: TIM1_CC_IRQHandler(); break;
    case TIM1_UP_IRQn: TIM1_UP_IRQHandler(); break;
    default: break;
    }
    return 1;
}

/* 注入 A 相（TIM3_CH2）一个边沿：ccr = 捕获值；with_uif = 同一次服务里还带溢出标志 */
static void inject_a(uint32_t ccr, int with_uif)
{
    TIM3->CCR2 = ccr;
    TIM3->SR   = TIM_SR_CC2IF | (with_uif ? TIM_SR_UIF : 0u);
    TIM3_IRQHandler();
}

/* 注入 B 相（TIM3_CH1）一个边沿 */
static void inject_b(uint32_t ccr, int with_uif)
{
    TIM3->CCR1 = ccr;
    TIM3->SR   = TIM_SR_CC1IF | (with_uif ? TIM_SR_UIF : 0u);
    TIM3_IRQHandler();
}

/* 注入 C 相（TIM1_CH1）一个边沿 */
static void inject_c(uint32_t ccr, int with_uif)
{
    TIM1->CCR1 = ccr;
    TIM1->SR   = TIM_SR_CC1IF | (with_uif ? TIM_SR_UIF : 0u);
    TIM1_CC_IRQHandler();
}

/* 只看一次真实溢出（不带捕获） */
static void inject_ovf3(void)
{
    TIM3->SR = TIM_SR_UIF;
    TIM3_IRQHandler();
}

/* 把 NVIC 调用日志压成 "D29,D27,C25,E29" 这样的短串，方便逐字比对 */
static void log_str(char *out, int cap)
{
    int  i = 0;
    int  n = 0;
    char c;

    out[0] = '\0';
    for (i = 0; i < g_nvic_log_n && n < (cap - 8); i++) {
        if (g_nvic_log[i].ev == TIMSIM_EV_SETPRIO)        c = 'S';
        else if (g_nvic_log[i].ev == TIMSIM_EV_ENABLE)    c = 'E';
        else if (g_nvic_log[i].ev == TIMSIM_EV_DISABLE)   c = 'D';
        else if (g_nvic_log[i].ev == TIMSIM_EV_CLEARPEND) c = 'C';
        else c = '?';
        out[n++] = c;
        if (g_nvic_log[i].irq >= 10u) {
            out[n++] = (char)('0' + (g_nvic_log[i].irq / 10u));
        }
        out[n++] = (char)('0' + (g_nvic_log[i].irq % 10u));
        out[n++] = ',';
    }
    if (n > 0) {
        out[n - 1] = '\0';
    }
}

/* 一条"窗口跑完"的收尾：把 tick 推到窗口之后，取快照 */
static pulse_snapshot_t finish_window(uint16_t win_ms)
{
    pulse_snapshot_t snap;
    uint8_t r;

    g_tick += win_ms;
    r = PULSE_CfgPoll();
    memset(&snap, 0, sizeof(snap));
    PULSE_Snapshot(&snap);
    CHECK(r == 1u, "窗口到点 → PULSE_CfgPoll 返回 1");
    return snap;
}

/*==============================================================================
 * 1) 初始化
 *============================================================================*/
static void test_init(void)
{
    SECTION("1) PULSE_CfgInit：计数器自由运行 / 初始化时不响应中断");
    sim_reset();
    PULSE_CfgInit();

    CHECK(TIM3->CR1 == TIM_CR1_CEN, "TIM3 计数器已开（自由运行）");
    CHECK(TIM1->CR1 == TIM_CR1_CEN, "TIM1 计数器已开（自由运行）");
    CHECK((TIM3->DIER & (TIM_DIER_CC1IE | TIM_DIER_CC2IE | TIM_DIER_UIE)) == 0u,
          "TIM3 初始化时不开捕获/溢出中断");
    CHECK((TIM1->DIER & (TIM_DIER_CC1IE | TIM_DIER_UIE)) == 0u,
          "TIM1 初始化时不开捕获/溢出中断");
    CHECK(TIM3->CNT == 0u && TIM3->SR == 0u, "TIM3 计数器与标志已清");
    CHECK(g_nvic_en[TIM3_IRQn] != 0u && g_nvic_en[TIM1_CC_IRQn] != 0u
              && g_nvic_en[TIM1_UP_IRQn] != 0u,
          "三个向量都已使能（开窗即生效，不用再动 NVIC）");
    CHECK(PULSE_IsRunning() == 0u, "初始化时不处在采集中");
}

/*==============================================================================
 * 2) 开窗的临界区结构与顺序（D4/D5 的核心保证）
 *============================================================================*/
static void test_start_critical_section(void)
{
    char s[128];
    int  i;
    int  all_masked = 1;
    int  last_clear = -1;
    int  first_enable = -1;

    SECTION("2) 开窗：NVIC 动作的顺序 + 全部在关中断状态下做");
    sim_reset();
    PULSE_CfgInit();

    g_nvic_log_n = 0;                    /* 只留本函数产生的动作 */
    PULSE_CfgStart(200u);

    log_str(s, (int)sizeof(s));
    CHECK(strcmp(s, "D29,D27,D25,C29,C27,C25,E29,E27,E25") == 0,
          "顺序必须是：关三个向量 → 清三个 pending → 再使能三个向量");
    if (strcmp(s, "D29,D27,D25,C29,C27,C25,E29,E27,E25") != 0) {
        printf("      实到: %s\n", s);
    }

    for (i = 0; i < g_nvic_log_n; i++) {
        if (g_nvic_log[i].primask == 0u) {
            all_masked = 0;
        }
        if (g_nvic_log[i].ev == TIMSIM_EV_CLEARPEND) {
            last_clear = i;
        }
        if ((g_nvic_log[i].ev == TIMSIM_EV_ENABLE) && (first_enable < 0)) {
            first_enable = i;
        }
    }
    CHECK(all_masked == 1, "9 条 NVIC 动作全部发生在 PRIMASK=1（关中断）时");
    CHECK((last_clear >= 0) && (first_enable > last_clear),
          "清 pending 必须全部排在重新使能之前（顺序反了等于没清）");
    CHECK(g_primask == 0u, "开窗结束必须把中断放开（不能关着走人）");

    /* 结束状态 */
    CHECK((TIM3->DIER & (TIM_DIER_CC1IE | TIM_DIER_CC2IE | TIM_DIER_UIE)) != 0u,
          "TIM3 捕获 + 溢出中断已开");
    CHECK((TIM1->DIER & (TIM_DIER_CC1IE | TIM_DIER_UIE)) != 0u, "TIM1 已开");
    CHECK(g_nvic_pend[TIM3_IRQn] == 0u && g_nvic_pend[TIM1_CC_IRQn] == 0u
              && g_nvic_pend[TIM1_UP_IRQn] == 0u,
          "开窗后 NVIC 里没有排队的旧请求");
    CHECK(TIM3->CNT == 0u && TIM1->CNT == 0u, "计数器归零（新窗口的起点）");
    CHECK(TIM3->SR == 0u && TIM1->SR == 0u, "标志归零");
    CHECK((TIM3->CCER & (TIM_CCER_CC1P | TIM_CCER_CC2P)) == 0u,
          "起始极性 = 都等上升沿");
    CHECK((TIM1->CCER & TIM_CCER_CC1P) == 0u, "TIM1 侧同样");
    CHECK(PULSE_IsRunning() != 0u, "算法层进入采集中");
}

/*==============================================================================
 * 3) 排队中的旧中断不许在新窗口里跑起来
 *============================================================================*/
static void test_stale_pending_dropped(void)
{
    pulse_snapshot_t snap;

    SECTION("3) 开窗清掉排队中的中断请求（否则它会在新窗口里跑）");
    sim_reset();
    PULSE_CfgInit();

    PULSE_CfgStart(200u);                /* 上一个窗口（U2 的滚动窗口就是这样跑着） */
    inject_a(5000u, 0);                  /* 它里面来过 1 个边沿 */
    TIM3->SR = TIM_SR_CC2IF;             /* 这一发捕获的标志还挂着 */
    g_nvic_pend[TIM3_IRQn] = 1u;         /* 请求已经排在 NVIC 里，还没被服务 */

    PULSE_CfgStart(200u);                /* 重开窗口（U2 收到 ARM 就是这么干的） */
    CHECK(g_nvic_pend[TIM3_IRQn] == 0u, "开窗把排队中的请求清掉了");
    CHECK(TIM3->SR == 0u, "SR 里的旧标志也被清掉");
    CHECK(sim_serve(TIM3_IRQn) == 0, "开窗之后那次旧请求不会再被执行（不再有 ISR 跑）");

    /* 新窗口里正常来 4 个沿：计数必须从 0 开始 */
    inject_a(1000u, 0);
    inject_a(2000u, 0);
    inject_a(3000u, 0);
    inject_a(4000u, 0);
    snap = finish_window(PULSE_WINDOW_MS);

    CHECK(snap.ph[PULSE_PH_A].edges == 4u, "新窗口边沿数 = 4（上一窗口的没有残留）");
    CHECK((snap.ph[PULSE_PH_A].flags & PULSE_F_VALID) != 0u, "A 相结算有效");
    CHECK(snap.ph[PULSE_PH_A].period_ns == 27777u, "周期 = 2000 tick = 27777 ns");
    CHECK(snap.ph[PULSE_PH_A].duty_permille == 500u, "占空比 = 500‰");
}

/*==============================================================================
 * 4) 三相都走一遍 + UIF 与捕获同拍的 +1 补偿
 *============================================================================*/
static void test_edges_and_extra_tick(void)
{
    pulse_snapshot_t snap;

    SECTION("4) 边沿入队 / 结算（三相）+ 先溢出后捕获的 +1 补偿");
    sim_reset();
    PULSE_CfgInit();
    PULSE_CfgStart(200u);

    /* A 相：第 1 个边沿与溢出标志同拍（先溢出、后捕获）→ 时间戳高位要 +1。
     * 若不补偿，第 1 个沿会记成 1000，第 2 个沿记成 65536+2000，
     * 两沿之差变成 66536 tick（≈0.92 ms）→ 周期/占空比全错。 */
    inject_a(1000u, 1);                  /* 补偿后 ts = 65536 + 1000 */
    inject_a(2000u, 0);                  /* ts = 65536 + 2000 */
    inject_a(3000u, 0);                  /* ts = 65536 + 3000 */
    inject_a(4000u, 0);                  /* ts = 65536 + 4000 */

    /* B 相（TIM3_CH1）和 C 相（TIM1_CH1）：走另一条通道/另一个定时器 */
    inject_b(100u, 0);
    inject_b(1100u, 0);
    inject_b(2100u, 0);
    inject_b(3100u, 0);

    inject_c(200u, 0);
    inject_c(1200u, 0);
    inject_c(2200u, 0);
    inject_c(3200u, 0);

    snap = finish_window(PULSE_WINDOW_MS);

    CHECK(snap.phase_count == (uint8_t)PULSE_PH_COUNT, "快照覆盖三相");
    CHECK((snap.ph[PULSE_PH_A].flags & PULSE_F_VALID) != 0u, "A 相有效");
    CHECK(snap.ph[PULSE_PH_A].edges == 4u, "A 相 4 个边沿");
    CHECK(snap.ph[PULSE_PH_A].period_ns == 27777u,
          "A 相周期 2000 tick（+1 补偿对了才是这个值；不补偿会差 65536 tick）");
    CHECK(snap.ph[PULSE_PH_A].high_ns == 13888u, "A 相高电平 = 1000 tick");
    CHECK(snap.ph[PULSE_PH_A].duty_permille == 500u, "A 相占空比 50%");

    CHECK((snap.ph[PULSE_PH_B].flags & PULSE_F_VALID) != 0u, "B 相有效（走 TIM3_CH1）");
    CHECK(snap.ph[PULSE_PH_B].period_ns == 27777u, "B 相周期与 A 相同");
    CHECK((snap.ph[PULSE_PH_C].flags & PULSE_F_VALID) != 0u, "C 相有效（走 TIM1_CH1）");
    CHECK(snap.ph[PULSE_PH_C].period_ns == 27777u, "C 相周期与 A 相同");
    CHECK((snap.flags & PULSE_SNAP_F_NOSIGNAL) == 0u, "三相都有信号 → 快照不带 NOSIGNAL");
    CHECK(PULSE_CfgLostEdges() == 0u, "没有丢边沿（ISR 只做一次读-记-翻）");
}

/*==============================================================================
 * 5) 跨真实溢出的时间戳扩展
 *============================================================================*/
static void test_overflow_extension(void)
{
    pulse_snapshot_t snap;

    SECTION("5) 真实溢出把时间戳高位抬高 65536，跨溢出的差值仍要对");
    sim_reset();
    PULSE_CfgInit();
    PULSE_CfgStart(200u);

    inject_a(60000u, 0);                 /* ts = 60000 */
    inject_ovf3();                       /* 真溢出：计数器绕了一圈 */
    inject_a(1000u, 0);                  /* ts = 65536 + 1000  = 66536 */
    inject_a(7536u, 0);                  /* ts = 65536 + 7536  = 73072 */
    inject_a(14072u, 0);                 /* ts = 65536 + 14072 = 79608 */

    snap = finish_window(PULSE_WINDOW_MS);

    CHECK((snap.ph[PULSE_PH_A].flags & PULSE_F_VALID) != 0u, "A 相有效");
    CHECK(snap.ph[PULSE_PH_A].edges == 4u, "4 个边沿");
    CHECK(snap.ph[PULSE_PH_A].period_ns == 181555u,
          "周期 = 13072 tick（含跨溢出的那一格；不扩展高位会算出离谱值）");
    CHECK(snap.ph[PULSE_PH_A].duty_permille == 500u, "占空比 50%");
    CHECK((snap.ph[PULSE_PH_A].flags & PULSE_F_DIRTY) == 0u, "没有异常间隔被丢");
}

/*==============================================================================
 * 6) TIM1 的溢出走独立向量
 *============================================================================*/
static void test_tim1_ovf_vector(void)
{
    pulse_snapshot_t snap;

    SECTION("6) TIM1 的溢出是独立向量（TIM1_UP），C 相时间戳也要跟着扩");
    sim_reset();
    PULSE_CfgInit();
    PULSE_CfgStart(200u);

    /* 让两沿刚好跨过 65536 的折返点，且间隔仍是 1000 tick：
     * 65036 --(1000)--> 65536+500 --(1000)--> 65536+1500 --(1000)--> 65536+2500 */
    inject_c(65036u, 0);                 /* ts = 65036 */
    /* C 相的溢出走 TIM1_UP_IRQn。真硬件里"标志置位 + DIER 使能"会让 NVIC
     * 自动把请求挂上队；替身不拦标志写入，所以这里手势补一次排队位。 */
    TIM1->SR = TIM_SR_UIF;
    g_nvic_pend[TIM1_UP_IRQn] = 1u;
    CHECK(sim_serve(TIM1_UP_IRQn) == 1, "TIM1 溢出中断确实排在队里并被服务");
    inject_c(500u, 0);                   /* ts = 65536 + 500  = 66036 */
    inject_c(1500u, 0);                  /* ts = 65536 + 1500 = 67036 */
    inject_c(2500u, 0);                  /* ts = 65536 + 2500 = 68036 */

    snap = finish_window(PULSE_WINDOW_MS);
    CHECK((snap.ph[PULSE_PH_C].flags & PULSE_F_VALID) != 0u, "C 相有效");
    CHECK(snap.ph[PULSE_PH_C].edges == 4u, "C 相 4 个边沿");
    CHECK(snap.ph[PULSE_PH_C].period_ns == 27777u, "C 相周期 2000 tick");
}

/*==============================================================================
 * 7) 关窗 / 到点收尾
 *============================================================================*/
static void test_stop_and_poll(void)
{
    pulse_snapshot_t snap;

    SECTION("7) 关窗与到点收尾");
    sim_reset();
    PULSE_CfgInit();
    PULSE_CfgStart(200u);

    PULSE_CfgStop();
    CHECK((TIM3->DIER & (TIM_DIER_CC1IE | TIM_DIER_CC2IE | TIM_DIER_UIE)) == 0u,
          "关窗后捕获/溢出中断已关");
    CHECK((TIM1->DIER & (TIM_DIER_CC1IE | TIM_DIER_UIE)) == 0u, "TIM1 侧也关");
    CHECK(PULSE_IsRunning() == 0u, "算法层不在采集");
    CHECK(PULSE_CfgPoll() == 2u, "没在采时 Poll 返回 2");

    /* 被强制关掉的窗口：关窗之后到达的边沿不许再进环 */
    inject_a(5000u, 0);
    CHECK(PULSE_CfgPoll() == 2u, "关窗后仍是没在采");

    /* 正常到点：Poll 结算并自己关掉捕获 */
    PULSE_CfgStart(200u);
    inject_a(1000u, 0);
    inject_a(2000u, 0);
    inject_a(3000u, 0);
    inject_a(4000u, 0);
    snap = finish_window(200u);
    CHECK(snap.window_ms == 200u, "快照记下窗口长度");
    CHECK((TIM3->DIER & (TIM_DIER_CC1IE | TIM_DIER_CC2IE | TIM_DIER_UIE)) == 0u,
          "到点后 Poll 自己把捕获关掉（不用等应用层再调 Stop）");
    CHECK(PULSE_CfgPoll() == 2u, "结算之后再 Poll 返回 2");
}

/*============================================================================*/
int main(void)
{
    printf("========================================\n");
    printf(" pulse_cfg (TIM capture HW layer) host test\n");
    printf("========================================\n");

    test_init();
    test_start_critical_section();
    test_stale_pending_dropped();
    test_edges_and_extra_tick();
    test_overflow_extension();
    test_tim1_ovf_vector();
    test_stop_and_poll();

    printf("\n========================================\n");
    printf(" checks: %d, failed: %d\n", g_checks, g_fails);
    printf(" %s\n", (g_fails == 0) ? "ALL PASS" : "FAILURES PRESENT");
    printf("========================================\n");
    return (g_fails == 0) ? 0 : 1;
}
