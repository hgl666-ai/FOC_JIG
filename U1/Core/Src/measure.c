/**
  ******************************************************************************
  * @file    measure.c
  * @brief   测量功能实现：通道表 + 换算 + 滤波 + 判定 + 调试打印
  *
  * 分压/通道对应关系见 docs/06_测量通道分压逆推.md：
  *   每个通道前级都是"分压（或直连）→ 电压跟随器 → 100Ω+1nF+TVS → ADC"，
  *   跟随器增益为 1，所以 V_in = V_adc / k，k 就是分压比。
  ******************************************************************************
  */
#include "measure.h"
#include "main.h"

#ifndef MEAS_HOST_TEST
#include "adc.h"
#endif

#include "bsp_uart.h"

/* ------------------------------------------------------------------ */
/* 通道表                                                              */
/* ------------------------------------------------------------------ */
typedef struct {
    const char *name;
    uint16_t    k_num;      /* 分压比 k = k_num / k_den（已是约分后的整数） */
    uint16_t    k_den;
    uint16_t    vmin_mv;    /* 合格下限（换算到输入侧 mV） */
    uint16_t    vmax_mv;    /* 合格上限 */
    uint8_t     ema_shift;  /* 指数滑动平均 alpha = 1/(1<<shift)，越大越慢 */
} meas_cfg_t;

/* 说明：
 *  - k 的取值来自原理图：24V/10V 用 4.7k/(32.4k+4.7k)=47/371；
 *    5V 用 30k/(20k+30k)=3/5；3.3V 用 30k/(3k+30k)=10/11；
 *    相电压用 20k/(140k+20k)=1/8；电流两路直连 k=1。
 *  - 门限是**默认值**，只为"先跑起来 + 能打印"：24V 下限用图上标注的 7V，
 *    上限按满量程 26.05V 留 4% 余量；其余按标称 ±10%；相电压/电流的判据
 *    要按被测板规格再定（所以先给全量程）。
 *  - 相电压是被 PWM 斩波的，窗口取 1/256（约 10ms）做平均；
 *    直流通道取 1/64（约 2.6ms）。 */
static const meas_cfg_t k_cfg[MEAS_CH_COUNT] = {
    /* name    k_num  k_den  vmin   vmax   ema_shift */
    {  "V24",    47,   371,   7000, 25000, 6 },
    {  "V10",    47,   371,   9000, 11000, 6 },
    {  "V5",      3,     5,   4500,  5500, 6 },
    {  "V33",    10,    11,   3000,  3600, 6 },
    {  "PHA",     1,     8,      0, 26400, 8 },
    {  "PHB",     1,     8,      0, 26400, 8 },
    {  "PHC",     1,     8,      0, 26400, 8 },
    {  "CUR1",    1,     1,      0,  3300, 6 },
    {  "CUR2",    1,     1,      0,  3300, 6 },
};

/* ------------------------------------------------------------------ */
/* 运行态                                                              */
/* ------------------------------------------------------------------ */
/* 内部一律用**微伏**而不是毫伏：
 * 指数滑动平均是整数运算，"每步增量 = (目标-当前)>>shift" 在 |差值| < 2^shift
 * 时会截断成 0，形成死区。若以毫伏为单位，shift=6 的死区就是 ±63mV，
 * 相电压通道(shift=8)更是 ±255mV —— 不能接受。
 * 以微伏为单位后死区降到 64uV / 256uV，折算到输入侧 < 2mV，可以忽略。 */
static uint16_t s_dma[MEAS_SCAN_COUNT];          /* DMA 循环搬运的原始码值 */
static uint32_t s_scale_q4_uv[MEAS_CH_COUNT];    /* uV/count，Q4 定点 */
static uint32_t s_ema_uv[MEAS_CH_COUNT];         /* 滤波后的输入侧电压 uV */
static uint8_t  s_ema_inited[MEAS_CH_COUNT];
static uint16_t s_raw[MEAS_CH_COUNT];
static uint8_t  s_in_cnt[MEAS_CH_COUNT];
static uint8_t  s_ok[MEAS_CH_COUNT];
static uint8_t  s_sat[MEAS_CH_COUNT];
static uint32_t s_vdda_mv = MEAS_VDDA_MV;
static uint32_t s_print_tick;
#ifndef MEAS_HOST_TEST
static uint32_t s_last_ndtr;
#endif

/* DMA 是否刚完成一整轮扫描？
 * 主机环路频率远高于 ADC 扫描率（10 通道约 40us 一轮 ≈ 25kHz）。
 * 若每次 MEAS_Poll 都更新 EMA，等效平均的**样本数**会大幅减少
 * （噪声抑制变差），而且滤波窗口的时间常数随环路频率漂移。
 * 所以用 DMA 剩余计数回卷来判定"有新的一轮数据"，一轮只更新一次。 */
static uint8_t meas_new_scan(void)
{
#ifdef MEAS_HOST_TEST
    return 1u;                       /* 宿主单测：每次调用都当作有新数据 */
#else
    uint32_t cnt = DMA1_Channel1->CNDTR;   /* 递减计数 */
    if (cnt > s_last_ndtr) {               /* 回卷 = 完成一轮 */
        s_last_ndtr = cnt;
        return 1u;
    }
    s_last_ndtr = cnt;
    return 0u;
#endif
}

/* 预计算 uV/count（Q4：低 4 位是小数）：
 *   scale = (VDDA_uV/4095) * (k_den/k_num) * 16
 * 只在初始化时算一次，用 64 位中间量避免溢出（24V 通道分子约 2e10）。
 * 运行时 raw*scale 最大 4095*101776 = 4.2e8 < 2^32，安全。 */
static void meas_recalc_scales(uint32_t vdda_mv)
{
    uint8_t  i;
    uint64_t vdda_uv = (uint64_t)vdda_mv * 1000u;

    for (i = 0u; i < (uint8_t)MEAS_CH_COUNT; i++) {
        uint64_t num = vdda_uv * (uint64_t)k_cfg[i].k_den * 16u;
        uint64_t den = 4095u * (uint64_t)k_cfg[i].k_num;
        s_scale_q4_uv[i] = (den != 0u) ? (uint32_t)(num / den) : 0u;
    }
}

void MEAS_Init(void)
{
    uint8_t i;

    for (i = 0u; i < (uint8_t)MEAS_SCAN_COUNT; i++) {
        s_dma[i] = 0u;
    }
    for (i = 0u; i < (uint8_t)MEAS_CH_COUNT; i++) {
        s_ema_uv[i]    = 0u;
        s_ema_inited[i]= 0u;
        s_raw[i]       = 0u;
        s_in_cnt[i]    = 0u;
        s_ok[i]        = 0u;
        s_sat[i]       = 0u;
    }
    s_print_tick = 0u;

#if MEAS_USE_VREFINT
    s_vdda_mv = MEAS_VDDA_MV;      /* 第一次轮询后用 VREFINT 修正 */
#else
    s_vdda_mv = MEAS_VDDA_MV;
#endif
    meas_recalc_scales(s_vdda_mv);

#ifndef MEAS_HOST_TEST
    /* ADC 自校准（F1：HAL_ADCEx_Calibration_Start），然后启动 DMA 循环扫描 */
    if (HAL_ADCEx_Calibration_Start(&hadc1) != HAL_OK) {
        Error_Handler();
    }
    if (HAL_ADC_Start_DMA(&hadc1, (uint32_t *)s_dma, (uint32_t)MEAS_SCAN_COUNT) != HAL_OK) {
        Error_Handler();
    }
    s_last_ndtr = DMA1_Channel1->CNDTR;
#endif

    MEAS_PrintHeader();
}

void MEAS_Poll(void)
{
    uint8_t i;

    if (meas_new_scan() == 0u) {
        return;                      /* 还没有新的一轮 ADC 数据 */
    }

#if MEAS_USE_VREFINT
    /* 用内部基准反算 VDDA，跟踪 LDO 漂移（需要一次性标定 MEAS_VREFINT_MV） */
    if (s_dma[MEAS_CH_VREF] != 0u) {
        uint32_t vdda = (MEAS_VREFINT_MV * 4095u) / (uint32_t)s_dma[MEAS_CH_VREF];
        if (vdda != s_vdda_mv) {
            s_vdda_mv = vdda;
            meas_recalc_scales(s_vdda_mv);
        }
    }
#endif

    for (i = 0u; i < (uint8_t)MEAS_CH_COUNT; i++) {
        uint32_t raw = s_dma[i];
        uint32_t uv;
        int32_t  d;
        uint8_t  shift;

        if (raw > 4095u) {
            raw = 4095u;
        }
        s_raw[i] = (uint16_t)raw;

        /* 码值 → 输入侧电压 (uV) */
        uv = (raw * s_scale_q4_uv[i]) >> 4;

        /* 指数滑动平均：首次直接置初值，避免上电后几十毫秒的爬升 */
        shift = k_cfg[i].ema_shift;
        if (s_ema_inited[i] == 0u) {
            s_ema_uv[i]    = uv;
            s_ema_inited[i]= 1u;
        } else {
            d = (int32_t)uv - (int32_t)s_ema_uv[i];
            /* 不依赖算术右移的实现定义行为，显式取绝对值右移 */
            if (d >= 0) {
                d = d >> shift;
            } else {
                d = -((-d) >> shift);
            }
            s_ema_uv[i] = (uint32_t)((int32_t)s_ema_uv[i] + d);
        }
        uv = s_ema_uv[i];

        /* 门限判定：连续 MEAS_STABLE_N 次都在范围内才算合格 */
        if (uv >= ((uint32_t)k_cfg[i].vmin_mv * 1000u) &&
            uv <= ((uint32_t)k_cfg[i].vmax_mv * 1000u)) {
            if (s_in_cnt[i] < MEAS_STABLE_N) {
                s_in_cnt[i]++;
            }
        } else {
            s_in_cnt[i] = 0u;
        }
        s_ok[i]  = (s_in_cnt[i] >= MEAS_STABLE_N) ? 1u : 0u;
        s_sat[i] = ((raw <= MEAS_SAT_LOW) || (raw >= MEAS_SAT_HIGH)) ? 1u : 0u;
    }

#if (MEAS_PRINT_PERIOD_MS > 0u)
    if ((HAL_GetTick() - s_print_tick) >= MEAS_PRINT_PERIOD_MS) {
        s_print_tick = HAL_GetTick();
        MEAS_Print();
    }
#endif
}

/* ------------------------------------------------------------------ */
/* 取值接口                                                            */
/* ------------------------------------------------------------------ */
uint32_t MEAS_GetMv(meas_id_t ch)
{
    if ((uint8_t)ch >= (uint8_t)MEAS_CH_COUNT) {
        return 0u;
    }
    /* uV -> mV，四舍五入 */
    return (s_ema_uv[ch] + 500u) / 1000u;
}

uint8_t MEAS_IsOk(meas_id_t ch)
{
    if ((uint8_t)ch >= (uint8_t)MEAS_CH_COUNT) {
        return 0u;
    }
    return s_ok[ch];
}

void MEAS_Get(meas_id_t ch, meas_value_t *out)
{
    if (out == 0) {
        return;
    }
    out->raw = 0u;
    out->mv  = 0u;
    out->ok  = 0u;
    out->sat = 0u;
    if ((uint8_t)ch >= (uint8_t)MEAS_CH_COUNT) {
        return;
    }
    out->raw = s_raw[ch];
    out->mv  = (s_ema_uv[ch] + 500u) / 1000u;
    out->ok  = s_ok[ch];
    out->sat = s_sat[ch];
}

uint8_t MEAS_AllOk(void)
{
    uint8_t i;
    for (i = 0u; i < (uint8_t)MEAS_CH_COUNT; i++) {
        if (s_ok[i] == 0u) {
            return 0u;
        }
    }
    return 1u;
}

uint32_t MEAS_GetFaultMask(void)
{
    uint32_t m = 0u;
    uint8_t  i;
    for (i = 0u; i < (uint8_t)MEAS_CH_COUNT; i++) {
        if (s_ok[i] == 0u) {
            m |= (1u << i);
        }
    }
    return m;
}

uint32_t MEAS_GetVddaMv(void)
{
    return s_vdda_mv;
}

const char *MEAS_Name(meas_id_t ch)
{
    if ((uint8_t)ch >= (uint8_t)MEAS_CH_COUNT) {
        return "?";
    }
    return k_cfg[ch].name;
}

meas_cc_t MEAS_CrossCheck(void)
{
    /* 交叉校验：U1 与 U2 各自独立采样同一路信号（硬件上是两套独立链路），
     * 比对两套读数即可发现"运放虚焊 / 分压漂移 / ADC 坏"这类治具自身故障——
     * 生产线上最怕的是治具悄悄测错还判合格。
     *
     * 启用需要三件事（当前都还没做）：
     *   1) U2 用同一张表采样（measure.c 是 U1/U2 共用模块）；
     *   2) U2 经 PB9 <-> PA11 上报读数（每秒十几字节，那根硬线足够）；
     *   3) 逐通道比对，超过 MEAS_CC_TOL_MV 返回 MEAS_CC_MISMATCH。
     * 注意两片 ADC 的基准是同一路 3.3V，所以容差可以取 2~3%，不必要求相等。 */
    return MEAS_CC_DISABLED;
}

/* ------------------------------------------------------------------ */
/* 调试打印（USART1 / PA9）                                            */
/* ------------------------------------------------------------------ */
static uint16_t fmt_u32(char *p, uint32_t v)
{
    char     tmp[10];
    uint8_t  n = 0u;
    uint16_t i;

    if (v == 0u) {
        p[0] = '0';
        return 1u;
    }
    while (v > 0u && n < (uint8_t)sizeof(tmp)) {
        tmp[n] = (char)('0' + (v % 10u));
        v /= 10u;
        n++;
    }
    for (i = 0u; i < n; i++) {
        p[i] = tmp[n - 1u - i];
    }
    return n;
}

void MEAS_PrintHeader(void)
{
    BSP_DBG_Send("\r\n[MEAS] k: V24/V10=47/371 V5=3/5 V33=10/11 PHx=1/8 CUR=1\r\n");
    BSP_DBG_Send("[MEAS] VDDA=3300mV, ema: DC=1/64 PHx=1/256\r\n");
}

void MEAS_Print(void)
{
    char    line[128];
    uint16_t n = 0u;
    uint8_t  i;

    line[n++] = 'M';
    line[n++] = ' ';
    for (i = 0u; i < (uint8_t)MEAS_CH_COUNT; i++) {
        const char *nm = k_cfg[i].name;
        uint8_t     j;

        if (n > (uint16_t)(sizeof(line) - 16u)) {
            break;
        }
        for (j = 0u; nm[j] != '\0' && n < (uint16_t)(sizeof(line) - 12u); j++) {
            line[n++] = nm[j];
        }
        line[n++] = '=';
        n = (uint16_t)(n + fmt_u32(&line[n], MEAS_GetMv((meas_id_t)i)));
        line[n++] = ' ';
    }
    /* 判定与告警 */
    line[n++] = 'O';
    line[n++] = 'K';
    line[n++] = '=';
    line[n++] = (char)('0' + MEAS_AllOk());
    line[n++] = ' ';
    line[n++] = 'F';
    line[n++] = '=';
    line[n++] = '0';
    line[n++] = 'x';
    {
        uint8_t  k;
        uint32_t m = MEAS_GetFaultMask();
        for (k = 0u; k < 3u; k++) {
            uint8_t d = (uint8_t)((m >> ((2u - k) * 4u)) & 0xFu);
            line[n++] = (char)((d < 10u) ? ('0' + (char)d) : ('A' + (char)(d - 10u)));
        }
    }
    line[n++] = '\r';
    line[n++] = '\n';
    line[n]   = '\0';
    BSP_DBG_Send(line);
}

#ifdef MEAS_HOST_TEST
void MEAS_TestInject(meas_id_t ch, uint16_t counts)
{
    if ((uint8_t)ch < (uint8_t)MEAS_SCAN_COUNT) {
        s_dma[ch] = counts;
    }
}

void MEAS_TestInjectVref(uint16_t counts)
{
    s_dma[MEAS_CH_VREF] = counts;
}
#endif
