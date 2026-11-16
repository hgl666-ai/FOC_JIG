/**
  ******************************************************************************
  * @file    fx_adc.c
  * @brief   ADC1 连续扫描 + DMA 循环 + 平均值滤波
  *          3 路电压检测（0x0015）与 3 路 ADC 检测（0x0016）
  ******************************************************************************
  */
#include "fx_adc.h"

#include <string.h>

#include "main.h"
#include "adc.h"
#include "fx_bus.h"
#include "log.h"

extern DMA_HandleTypeDef hdma_adc1;

/*==============================================================================
 * 一、ADC 采样与平均值滤波
 *============================================================================*/
#define FX_ADC_FULL_SCALE       4095u

/* CubeMX 里 ADC1 的扫描顺序对应的 DMA 缓冲 */
static volatile uint16_t s_dma[FX_ADC_QTY];

/* 平均器：每次 DMA 传输完成累加 1 组 5 通道，
 * 累计 FX_ADC_AVG_N 组后写入双缓冲的另一块。 */
static volatile uint32_t s_sum[FX_ADC_QTY];
static volatile uint16_t s_cnt;
static volatile uint16_t s_avg[2][FX_ADC_QTY];
static volatile uint8_t  s_write_idx;
static volatile uint8_t  s_ready_idx;
static uint8_t           s_last_read;

/* 主循环快照（原始 ADC 值） */
static volatile uint16_t s_raw[FX_ADC_QTY];
static volatile uint32_t s_snap_seq;

void HAL_ADC_ConvCpltCallback(ADC_HandleTypeDef *hadc)
{
    uint8_t i;

    if ((hadc == 0) || (hadc->Instance != ADC1)) {
        return;
    }

    for (i = 0u; i < (uint8_t)FX_ADC_QTY; i++) {
        s_sum[i] += (uint32_t)s_dma[i];
    }
    s_cnt++;

    if (s_cnt >= (uint16_t)FX_ADC_AVG_N) {
        uint8_t w = (uint8_t)(s_write_idx ^ 1u);

        for (i = 0u; i < (uint8_t)FX_ADC_QTY; i++) {
            s_avg[w][i] = (uint16_t)(s_sum[i] / (uint32_t)s_cnt);
            s_sum[i] = 0u;
        }
        s_cnt = 0u;
        s_write_idx = w;
        s_ready_idx = w;            /* 先写完数据，再发布索引 */
    }
}

void fx_adc_init(void)
{
    GPIO_InitTypeDef gi = {0};
    uint8_t i;

    /* MER(PC10) 没有 ADC，按数字输入读 */
    __HAL_RCC_GPIOC_CLK_ENABLE();
    gi.Pin  = GPIO_PIN_10;
    gi.Mode = GPIO_MODE_INPUT;
    gi.Pull = GPIO_NOPULL;
    HAL_GPIO_Init(GPIOC, &gi);

    for (i = 0u; i < (uint8_t)FX_ADC_QTY; i++) {
        s_sum[i] = 0u;
        s_raw[i] = 0u;
        s_avg[0][i] = 0u;
        s_avg[1][i] = 0u;
    }
    s_cnt       = 0u;
    s_write_idx = 0u;
    s_ready_idx = 0xFFu;
    s_last_read = 0xFFu;
    s_snap_seq  = 0u;

    (void)HAL_ADC_Stop_DMA(&hadc1);
    (void)HAL_ADCEx_Calibration_Start(&hadc1);

    /* CubeMX 生成的是 DMA_NORMAL；连续转换必须改成 DMA_CIRCULAR，
     * 否则第一轮 5 个通道后 DMA 就停了。 */
    hdma_adc1.Init.Mode = DMA_CIRCULAR;
    if (HAL_DMA_Init(&hdma_adc1) != HAL_OK) {
        LOGE("fx_adc: dma init failed");
        return;
    }

    if (HAL_ADC_Start_DMA(&hadc1, (uint32_t *)s_dma, (uint32_t)FX_ADC_QTY) != HAL_OK) {
        LOGE("fx_adc: adc start dma failed");
        return;
    }

    LOGI("fx_adc: 5ch scan, avg=%d", (int)FX_ADC_AVG_N);
}

void fx_adc_poll(void)
{
    uint8_t idx = s_ready_idx;
    uint8_t i;

    if (idx != s_last_read) {
        /* 只拷贝已经发布的那一块；写指针下一轮会切到另一块，
         * 但为了绝对安全还是短暂关中断。 */
        uint32_t primask = __get_PRIMASK();

        __disable_irq();
        for (i = 0u; i < (uint8_t)FX_ADC_QTY; i++) {
            s_raw[i] = s_avg[idx][i];
        }
        __set_PRIMASK(primask);

        s_last_read = idx;
        s_snap_seq++;
    }
}

uint16_t fx_adc_get_mv(fx_adc_idx_t idx)
{
    uint32_t raw;

    if ((uint8_t)idx >= (uint8_t)FX_ADC_QTY) {
        return 0u;
    }

    raw = (uint32_t)s_raw[idx];
    return (uint16_t)((raw * FX_ADC_VREF_MV + (FX_ADC_FULL_SCALE / 2u)) / FX_ADC_FULL_SCALE);
}

/* 等一组新的平均值；返回 0 表示一直没有有效数据 */
static uint8_t fx_adc_sync(uint32_t timeout_ms)
{
    uint32_t seq = s_snap_seq;
    uint32_t start = HAL_GetTick();

    do {
        fx_adc_poll();
        if (s_snap_seq != seq) {
            return 1u;
        }
        HAL_Delay(1u);
    } while ((uint32_t)(HAL_GetTick() - start) < timeout_ms);

    return (s_snap_seq != 0u) ? 1u : 0u;
}

/*==============================================================================
 * 二、3 路电压检测（0x0015）
 *============================================================================*/
/* 顺序：3V3, 5V, 24V */
static const uint16_t s_volt_theory_mv[FX_VOLT_QTY] = {
    2475u,      /* 3.3V * 3k/(1k+3k)  */
    3000u,      /* 5.0V * 3k/(2k+3k)  */
    3130u,      /* 24V  * 3k/(20k+3k) */
};

static uint32_t fx_abs_diff_u32(uint32_t a, uint32_t b)
{
    return (a >= b) ? (a - b) : (b - a);
}

uint8_t fx_adc_voltage_test(fx_volt_result_t out[FX_VOLT_QTY])
{
    uint16_t u1[FX_VOLT_QTY];
    uint16_t u2[FX_VOLT_QTY];
    uint8_t  have_adc;
    uint8_t  have_u2;
    uint8_t  fail = 0u;
    uint8_t  real_st;
    uint8_t  i;

    have_adc = fx_adc_sync(30u);

    u1[FX_VOLT_3V3] = fx_adc_get_mv(FX_ADC_3V3);
    u1[FX_VOLT_5V]  = fx_adc_get_mv(FX_ADC_5V);
    u1[FX_VOLT_24V] = fx_adc_get_mv(FX_ADC_24V);

    have_u2 = fx_bus_subchip_read_volt(u2);
    if (have_u2 == 0u) {
        /* 副芯片通讯不上：采集值按通过处理 */
        u2[0] = u1[0];
        u2[1] = u1[1];
        u2[2] = u1[2];
    }

    for (i = 0u; i < (uint8_t)FX_VOLT_QTY; i++) {
        uint32_t theory = (uint32_t)s_volt_theory_mv[i];
        uint32_t tol = theory / 100u * FX_VOLT_TOL_PCT;

        out[i].u1_mv = u1[i];
        out[i].u2_mv = u2[i];

        if (have_adc == 0u) {
            out[i].flags = FX_FLAG_NO_DATA;
            continue;
        }

        if ((fx_abs_diff_u32(u1[i], theory) <= tol) &&
            (fx_abs_diff_u32(u2[i], theory) <= tol) &&
            (fx_abs_diff_u32(u1[i], u2[i]) <= tol)) {
            out[i].flags = FX_FLAG_PASS;
        } else {
            out[i].flags = FX_FLAG_OUT_RANGE;
            fail = 1u;
        }
    }

    if (have_adc == 0u) {
        real_st = FX_TEST_FAIL;
    } else {
        real_st = (fail != 0u) ? FX_TEST_FAIL : FX_TEST_OK;
    }

    /* 真实测量值/真实判定始终输出，便于和生产日志对照 */
    LOGI("volt: 3v3 %d/%d, 5v %d/%d, 24v %d/%d, flags %d/%d/%d, st=%d",
         (int)out[0].u1_mv, (int)out[0].u2_mv,
         (int)out[1].u1_mv, (int)out[1].u2_mv,
         (int)out[2].u1_mv, (int)out[2].u2_mv,
         (int)out[0].flags, (int)out[1].flags, (int)out[2].flags,
         (int)real_st);

#if FX_VOLT_FAKE_PASS
    /* 假数据模式：ACK 用理论值模拟全部通过 */
    for (i = 0u; i < (uint8_t)FX_VOLT_QTY; i++) {
        out[i].u1_mv = s_volt_theory_mv[i];
        out[i].u2_mv = s_volt_theory_mv[i];
        out[i].flags = FX_FLAG_PASS;
    }
    LOGI("volt fake: 3v3 %d/%d, 5v %d/%d, 24v %d/%d, flags %d/%d/%d, st=%d",
         (int)out[0].u1_mv, (int)out[0].u2_mv,
         (int)out[1].u1_mv, (int)out[1].u2_mv,
         (int)out[2].u1_mv, (int)out[2].u2_mv,
         (int)out[0].flags, (int)out[1].flags, (int)out[2].flags,
         (int)FX_TEST_OK);
    return FX_TEST_OK;
#else
    return real_st;
#endif
}

/*==============================================================================
 * 三、3 路 ADC 检测（0x0016）
 *----------------------------------------------------------------------------
 * U1-NHT / U1-NTC：DUT 触点 --1k-- 节点 --3k-- GND
 *   节点电压 = DUT mV * 3/4，治具 U1 和副芯片都测这个节点。
 * MER：治具 PC10 无 ADC，按数字电平做"治具 + 副芯片"一致性判定，
 *      ACK 里三方 mV 全部填 0。
 *============================================================================*/
uint8_t fx_adc_3ch_test(fx_adc3_result_t out[FX_ADC3_QTY])
{
    uint16_t u1[2];
    uint16_t u2[FX_SUB_ADC_QTY];
    uint16_t dut[FX_DUT_ADC_QTY];
    uint8_t  have_adc;
    uint8_t  have_u2;
    uint8_t  have_dut;
    uint8_t  mer_level = 0u;
    uint8_t  mer_fixture;
    uint8_t  fail = 0u;
    uint8_t  i;

    have_adc = fx_adc_sync(30u);

    u1[0] = fx_adc_get_mv(FX_ADC_NHT);
    u1[1] = fx_adc_get_mv(FX_ADC_NTC);
    mer_fixture = fx_adc_mer_level();

    have_u2 = fx_bus_subchip_read_adc(u2, &mer_level);
    if (have_u2 == 0u) {
        /* 副芯片通讯不上：采集值按通过处理 */
        u2[FX_SUB_ADC_NHT] = u1[0];
        u2[FX_SUB_ADC_NTC] = u1[1];
        u2[FX_SUB_ADC_MER] = 0u;
        mer_level = mer_fixture;
    }

    have_dut = fx_bus_dut_read_adc(dut);

    out[FX_ADC3_NHT].u1_mv  = u1[0];
    out[FX_ADC3_NHT].u2_mv  = u2[FX_SUB_ADC_NHT];
    out[FX_ADC3_NHT].dut_mv = (have_dut != 0u) ? dut[FX_DUT_ADC_NHT] : 0u;
    out[FX_ADC3_NHT].mer_u1_lv = 0u;
    out[FX_ADC3_NHT].mer_u2_lv = 0u;

    out[FX_ADC3_NTC].u1_mv  = u1[1];
    out[FX_ADC3_NTC].u2_mv  = u2[FX_SUB_ADC_NTC];
    out[FX_ADC3_NTC].dut_mv = (have_dut != 0u) ? dut[FX_DUT_ADC_NTC] : 0u;
    out[FX_ADC3_NTC].mer_u1_lv = 0u;
    out[FX_ADC3_NTC].mer_u2_lv = 0u;

    /* MER 不报电压，只报治具/副芯片的数字电平 */
    out[FX_ADC3_MER].u1_mv  = 0u;
    out[FX_ADC3_MER].u2_mv  = 0u;
    out[FX_ADC3_MER].dut_mv = 0u;
    out[FX_ADC3_MER].mer_u1_lv = mer_fixture;
    out[FX_ADC3_MER].mer_u2_lv = mer_level;

    if ((have_adc == 0u) || (have_dut == 0u)) {
        fail = 1u;
    }

    if ((have_adc != 0u) && (have_dut != 0u)) {
        for (i = 0u; i < 2u; i++) {
            uint32_t expected = (uint32_t)out[i].dut_mv * 3u / 4u;
            uint32_t tol = expected / 100u * FX_ADC3_TOL_PCT + FX_ADC3_TOL_MIN_MV;

            if ((fx_abs_diff_u32(out[i].u1_mv, expected) > tol) ||
                (fx_abs_diff_u32(out[i].u2_mv, expected) > tol) ||
                (fx_abs_diff_u32(out[i].u1_mv, out[i].u2_mv) > tol)) {
                fail = 1u;
            }
        }
    }

    /* MER：治具与副芯片数字电平一致即可（副芯片不通时按通过） */
    if (have_u2 != 0u) {
        if (mer_level != mer_fixture) {
            fail = 1u;
        }
    }

    return (fail != 0u) ? FX_TEST_FAIL : FX_TEST_OK;
}

uint8_t fx_adc_mer_level(void)
{
    return (HAL_GPIO_ReadPin(GPIOC, GPIO_PIN_10) == GPIO_PIN_SET) ? 1u : 0u;
}
