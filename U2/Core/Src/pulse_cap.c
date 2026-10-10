/**
  ******************************************************************************
  * @file    pulse_cap.c
  * @brief   输入捕获的算法层：边沿时间戳 → 周期 / 高电平时间 / 占空比
  *
  * 关键约定（决定了算法能不能算对）：
  *   1) 采集层**交替**捕获上升沿和下降沿（每来一个边沿就把极性翻一次），
  *      所以从窗口开始算起，第 0、2、4… 个边沿是上升沿，第 1、3、5… 个是下降沿。
  *   2) 因此相邻两个边沿之间的时间就是：
  *        (上升→下降) = **高电平时间**
  *        (下降→上升) = **低电平时间**
  *   3) 环形缓冲只保留**最近 32 个**边沿；窗口内总边沿数另外用计数器统计
  *      （高速信号时环会被覆盖，但平均只需要最近若干个周期，正好够用）。
  *   4) 时间戳是 32 位 tick，单调递增（溢出由采集层扩展），所以
  *      区间 = 后一个 - 前一个，直接无符号相减即可（回绕也自动正确）。
  *
  * 本文件在两片里逐字节一致：不认识定时器、不认识引脚、不认识 HAL。
  ******************************************************************************
  */
#include "pulse_cap.h"
#include "main.h"     /* HAL_GetTick（窗口计时） */

/* ------------------------------------------------------------------ */
/* 内部状态                                                            */
/* ------------------------------------------------------------------ */
typedef struct {
    uint32_t ts[PULSE_RING_LEN];    /* 最近若干边沿的时间戳（tick） */
    uint32_t total;                 /* 窗口内边沿总数（用于判断"有没有信号"） */
    uint8_t  count;                 /* 环里有效元素个数（<= PULSE_RING_LEN） */
    uint8_t  head;                  /* 下一个写入位置 */
} pulse_phase_state_t;

static pulse_phase_state_t s_ph[PULSE_PH_COUNT];
static pulse_snapshot_t    s_snap;
static uint8_t             s_running;
static uint32_t            s_start_tick;    /* 窗口开始时刻（HAL_GetTick） */
static uint16_t            s_window_ms;
static uint32_t            s_win_ticks;     /* 窗口长度（tick），用于区间合理性检查 */

/* ------------------------------------------------------------------ */
static uint8_t ph_valid(pulse_ph_t ph)
{
    return ((uint8_t)ph < (uint8_t)PULSE_PH_COUNT) ? 1u : 0u;
}

/* tick → ns：ns = tick * 125 / 9（tick 最大约 7.2e6/窗口 → 用 64 位防溢出） */
static uint32_t tick_to_ns(uint32_t tick)
{
    uint64_t ns = ((uint64_t)tick * (uint64_t)PULSE_NS_NUM) / (uint64_t)PULSE_NS_DEN;

    return (ns > 0xFFFFFFFFu) ? 0xFFFFFFFFu : (uint32_t)ns;
}

/* ------------------------------------------------------------------ */
/* 生命周期                                                            */
/* ------------------------------------------------------------------ */
void PULSE_Init(void)
{
    uint8_t i;

    for (i = 0u; i < (uint8_t)PULSE_PH_COUNT; i++) {
        uint8_t j;
        for (j = 0u; j < PULSE_RING_LEN; j++) {
            s_ph[i].ts[j] = 0u;
        }
        s_ph[i].total = 0u;
        s_ph[i].count = 0u;
        s_ph[i].head  = 0u;
    }
    s_running   = 0u;
    s_start_tick = 0u;
    s_window_ms = PULSE_WINDOW_MS;
    s_win_ticks = 0u;

    s_snap.phase_count = (uint8_t)PULSE_PH_COUNT;
    s_snap.flags       = 0u;
    s_snap.window_ms   = PULSE_WINDOW_MS;
    for (i = 0u; i < (uint8_t)PULSE_PH_COUNT; i++) {
        s_snap.ph[i].period_ns     = 0u;
        s_snap.ph[i].high_ns       = 0u;
        s_snap.ph[i].duty_permille = 0u;
        s_snap.ph[i].edges         = 0u;
        s_snap.ph[i].flags         = 0u;
    }
}

void PULSE_Start(uint16_t window_ms)
{
    uint8_t i;

    if (window_ms == 0u) {
        window_ms = PULSE_WINDOW_MS;
    }
    for (i = 0u; i < (uint8_t)PULSE_PH_COUNT; i++) {
        s_ph[i].total = 0u;
        s_ph[i].count = 0u;
        s_ph[i].head  = 0u;
    }
    s_window_ms  = window_ms;
    s_win_ticks  = ((uint32_t)window_ms * (PULSE_TICK_HZ / 1000u));
    s_start_tick = HAL_GetTick();
    s_running    = 1u;
}

void PULSE_Stop(void)
{
    s_running = 0u;
}

uint8_t PULSE_IsRunning(void)
{
    return s_running;
}


/* ------------------------------------------------------------------ */
/* 边沿入队（由各片的捕获中断调用；中断里只做这一件事）                 */
/* ------------------------------------------------------------------ */
void PULSE_OnEdge(pulse_ph_t ph, uint32_t tick)
{
    pulse_phase_state_t *p;

    if ((s_running == 0u) || (ph_valid(ph) == 0u)) {
        return;
    }
    p = &s_ph[ph];
    if (p->total != 0xFFFFFFFFu) {
        p->total++;
    }
    p->ts[p->head] = tick;
    p->head = (uint8_t)((p->head + 1u) % PULSE_RING_LEN);
    if (p->count < (uint8_t)PULSE_RING_LEN) {
        p->count++;
    }
}

/* ------------------------------------------------------------------ */
/* 结算：把边沿序列算成周期/高电平/占空比                               */
/* ------------------------------------------------------------------ */
/* 小数组取中位数（选择法；n <= 32，只在窗口结束时算一次，不在中断里跑） */
static uint32_t median_of(const uint32_t *v, uint8_t n)
{
    uint32_t tmp[PULSE_RING_LEN];
    uint8_t  i, j;

    if (n == 0u) {
        return 0u;
    }
    for (i = 0u; i < n; i++) {
        tmp[i] = v[i];
    }
    for (i = 0u; i <= (uint8_t)(n / 2u); i++) {
        uint8_t m = i;
        for (j = (uint8_t)(i + 1u); j < n; j++) {
            if (tmp[j] < tmp[m]) {
                m = j;
            }
        }
        if (m != i) {
            uint32_t s = tmp[i];
            tmp[i] = tmp[m];
            tmp[m] = s;
        }
    }
    return tmp[n / 2u];
}

/* 组内离群判定：落在中位数的 [1/2, 3/2] 之外就丢掉（乘 2 比较，避免除法） */
static uint8_t within_half(uint32_t x, uint32_t med)
{
    return ((x * 2u >= med) && (x * 2u <= med * 3u)) ? 1u : 0u;
}
static void phase_settle(pulse_ph_t ph)
{
    const pulse_phase_state_t *p = &s_ph[ph];
    pulse_result_t *r = &s_snap.ph[ph];
    uint8_t  n = p->count;
    uint8_t  i;
    uint8_t  idx_first;
    uint8_t  n_high = 0u, n_low = 0u;
    uint8_t  dirty = 0u;
    uint32_t sum_high = 0u, sum_low = 0u;
    uint32_t hi[PULSE_RING_LEN];
    uint32_t lo[PULSE_RING_LEN];
    uint8_t  nhi = 0u, nlo = 0u;

    r->period_ns     = 0u;
    r->high_ns       = 0u;
    r->duty_permille = 0u;
    r->edges         = (p->total > 0xFFFFu) ? 0xFFFFu : (uint16_t)p->total;
    r->flags         = 0u;

    /* 边沿太少 → 没信号（不硬算） */
    if (p->total < PULSE_MIN_EDGES || n < 2u) {
        r->flags = PULSE_F_NOSIGNAL;
        return;
    }

    /* 环里最老的那个元素在数组中的下标；它的序号 = total - count，
     * 用它的奇偶决定"第 0 个区间是高还是低"（见文件头约定）。 */
    idx_first = (uint8_t)((p->head + (uint8_t)PULSE_RING_LEN - n) % (uint8_t)PULSE_RING_LEN);

    for (i = 0u; (uint8_t)(i + 1u) < n; i++) {
        uint8_t  ia = (uint8_t)((idx_first + i) % (uint8_t)PULSE_RING_LEN);
        uint8_t  ib = (uint8_t)((idx_first + i + 1u) % (uint8_t)PULSE_RING_LEN);
        uint32_t dt = p->ts[ib] - p->ts[ia];
        /* 这个边沿的序号（从窗口开始的第几个）——决定这一段的性质 */
        uint32_t edge_no = (p->total - (uint32_t)n) + (uint32_t)i;

        /* 0（同刻双沿）或超过整个窗口（丢边沿）→ 直接丢，不算进平均 */
        if ((dt == 0u) || (dt > (s_win_ticks + (s_win_ticks / 4u)))) {
            dirty = 1u;
            continue;
        }
        if ((edge_no & 1u) == 0u) {          /* 上升→下降 = 高电平时间 */
            hi[nhi++] = dt;
        } else {                             /* 下降→上升 = 低电平时间 */
            lo[nlo++] = dt;
        }
    }
    if ((nhi == 0u) || (nlo == 0u)) {
        r->flags = (uint8_t)(PULSE_F_NOSIGNAL | (dirty ? PULSE_F_DIRTY : 0u));
        return;
    }

    /* ── 第二遍：高、低两组各自用中位数剔除离群（毛刺 / 丢沿） ──
     * 必须**分组**做：占空比 10% 时高电平段天生比低电平段短得多，
     * 拿全体中位数当基准会把正常的高电平段全当成异常丢掉。 */
    {
        uint32_t mh = median_of(hi, nhi);
        uint32_t ml = median_of(lo, nlo);

        for (i = 0u; i < nhi; i++) {
            if (within_half(hi[i], mh) != 0u) {
                sum_high += hi[i];
                n_high++;
            } else {
                dirty = 1u;
            }
        }
        for (i = 0u; i < nlo; i++) {
            if (within_half(lo[i], ml) != 0u) {
                sum_low += lo[i];
                n_low++;
            } else {
                dirty = 1u;
            }
        }
        if ((n_high == 0u) || (n_low == 0u)) {
            r->flags = (uint8_t)(PULSE_F_NOSIGNAL | PULSE_F_DIRTY);
            return;
        }
    }

    /* ── 平均：高、低各自取平均后再合成 ──
     * ⚠ 不能用"高电平总和 / 总时间"：高低两组的样本数可能差 1（最后一个
     * 区间不完整），直接相除会让占空比整体偏移（20kHz/50% 会算成 34.7%）。 */
    {
        uint32_t avg_high = sum_high / n_high;
        uint32_t avg_low  = sum_low / n_low;
        uint32_t period   = avg_high + avg_low;

        r->flags     = PULSE_F_VALID | (dirty ? PULSE_F_DIRTY : 0u);
        r->high_ns   = tick_to_ns(avg_high);
        r->period_ns = tick_to_ns(period);
        r->duty_permille = (uint16_t)((((uint64_t)avg_high * 1000u) + (period / 2u)) / period);

        if ((period < PULSE_MIN_PERIOD_TICK) || (period > PULSE_MAX_PERIOD_TICK)) {
            r->flags |= PULSE_F_RANGE;
        }
    }
}

uint8_t PULSE_Poll(void)
{
    uint8_t i;
    uint8_t any_no_signal = 0u;

    if (s_running == 0u) {
        return 2u;
    }
    if ((uint32_t)(HAL_GetTick() - s_start_tick) < (uint32_t)s_window_ms) {
        return 0u;                       /* 窗口还没到 */
    }

    s_running = 0u;
    for (i = 0u; i < (uint8_t)PULSE_PH_COUNT; i++) {
        phase_settle((pulse_ph_t)i);
        if ((s_snap.ph[i].flags & PULSE_F_VALID) == 0u) {
            any_no_signal = 1u;
        }
    }
    s_snap.phase_count = (uint8_t)PULSE_PH_COUNT;
    s_snap.window_ms   = s_window_ms;
    s_snap.flags       = PULSE_SNAP_F_VALID |
                         (any_no_signal ? PULSE_SNAP_F_NOSIGNAL : 0u);
    return 1u;
}

void PULSE_Snapshot(pulse_snapshot_t *out)
{
    uint8_t i;

    if (out == 0) {
        return;
    }
    out->phase_count = s_snap.phase_count;
    out->flags       = s_snap.flags;
    out->window_ms   = s_snap.window_ms;
    for (i = 0u; i < (uint8_t)PULSE_PH_COUNT; i++) {
        out->ph[i] = s_snap.ph[i];
    }
}
