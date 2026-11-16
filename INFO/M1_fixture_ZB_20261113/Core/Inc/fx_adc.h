/**
  ******************************************************************************
  * @file    fx_adc.h
  * @brief   M1 治具 —— ADC 采集（5 路，DMA + 平均值滤波）
  *          + 3 路电压检测（0x0015）
  *          + 3 路 ADC 检测（0x0016）
  *
  * CubeMX 已配置 ADC1 扫描 5 个通道，顺序如下（DMA 缓冲下标）：
  *   0: PA0  ADC1_IN0   U1-5V
  *   1: PA1  ADC1_IN1   U1-24V
  *   2: PC1  ADC1_IN11  U1-NHT
  *   3: PC2  ADC1_IN12  U1-3V3
  *   4: PC3  ADC1_IN13  U1-NTC
  *
  * 电压检测理论值（治具分压后）：
  *   U1-3V3 : 3.3V * 3k/(1k+3k)  = 2475 mV
  *   U1-5V  : 5.0V * 3k/(2k+3k)  = 3000 mV
  *   U1-24V : 24V  * 3k/(20k+3k) = 3130 mV
  *
  * ADC 检测：
  *   U1-NHT / U1-NTC：DUT 触点经 1k 串到治具节点，节点有 3k 下拉，
  *                     所以治具/副芯片读到的是 DUT mV 的 3/4。
  *   MER：治具 PC10 无 ADC，直接用数字电平按 IO 输入方式判，
  *        ACK 里三方的 mV 都填 0。
  ******************************************************************************
  */
#ifndef __FX_ADC_H
#define __FX_ADC_H

#include "fx_common.h"

/* 可调参数：Vref、平均次数、判定容差 */
#ifndef FX_ADC_VREF_MV
#define FX_ADC_VREF_MV      3300u
#endif

#ifndef FX_ADC_AVG_N
#define FX_ADC_AVG_N        32u
#endif

/* 电压检测：理论值允许的偏差百分比 */
#ifndef FX_VOLT_TOL_PCT
#define FX_VOLT_TOL_PCT     10u
#endif

/* 1 = 电压检测使用假数据（理论值）模拟通过。
 *     返回给 PC 的 ACK 是假数据 + 状态 0x00；
 *     日志仍先输出真实测量值和真实判定，再额外输出一条 fake 日志。 */
#ifndef FX_VOLT_FAKE_PASS
#define FX_VOLT_FAKE_PASS   1
#endif

/* ADC 检测：节点电压相对 DUT mV*3/4 的允许偏差 */
#ifndef FX_ADC3_TOL_PCT
#define FX_ADC3_TOL_PCT     10u
#endif

#ifndef FX_ADC3_TOL_MIN_MV
#define FX_ADC3_TOL_MIN_MV  50u
#endif

/* DMA 缓冲下标 / 治具 ADC 通道 */
typedef enum {
    FX_ADC_5V = 0,
    FX_ADC_24V,
    FX_ADC_NHT,
    FX_ADC_3V3,
    FX_ADC_NTC,
    FX_ADC_QTY
} fx_adc_idx_t;

/* 3 路电压检测结果顺序：3V3, 5V, 24V（与 0x0015 ACK 一致） */
typedef enum {
    FX_VOLT_3V3 = 0,
    FX_VOLT_5V,
    FX_VOLT_24V,
    FX_VOLT_QTY
} fx_volt_id_t;

/* 3 路 ADC 检测结果顺序：NHT, NTC, MER（与 0x0016 ACK 一致） */
typedef enum {
    FX_ADC3_NHT = 0,
    FX_ADC3_NTC,
    FX_ADC3_MER,
    FX_ADC3_QTY
} fx_adc3_id_t;

typedef struct {
    uint16_t u1_mv;     /* 治具 U1 读到的 mV */
    uint16_t u2_mv;     /* 副芯片读到的 mV（不通时填 u1_mv） */
    uint8_t  flags;     /* FX_FLAG_xxx */
} fx_volt_result_t;

typedef struct {
    uint16_t u1_mv;     /* 治具 U1 读到的 mV */
    uint16_t u2_mv;     /* 副芯片读到的 mV */
    uint16_t dut_mv;    /* DUT M105 V 读到的 mV */
    uint8_t  mer_u1_lv; /* MER：治具 PC10 数字电平（仅 MER 通道使用） */
    uint8_t  mer_u2_lv; /* MER：副芯片数字电平（副芯片不通时等于 u1） */
} fx_adc3_result_t;

/** 启动 ADC1 连续扫描 + DMA 循环 + 32 次平均 */
void fx_adc_init(void);

/** 主循环 1ms 任务：把 DMA 平均结果搬到快照，保证读取一致 */
void fx_adc_poll(void);

/** 返回某通道最近一次平均后的 mV；没有有效数据时返回 0 */
uint16_t fx_adc_get_mv(fx_adc_idx_t idx);

/** 3 路电压检测（0x0015），结果填 out[3]；返回状态码 */
uint8_t fx_adc_voltage_test(fx_volt_result_t out[FX_VOLT_QTY]);

/** 3 路 ADC 检测（0x0016），结果填 out[3]；返回状态码。
 *  注意：out[].u1_mv/u2_mv 是治具节点原始读数，dut_mv 是 DUT 原始读数。
 *  data_process.c 打包 ACK 时会把 NHT/NTC 的 U1/U2 乘 4/3 折算到 DUT 侧，
 *  DUT 字段保持原始值，使 PC 端三个值可直接比较。
 *  原始值输出在 "adc3:" 日志里，转换后的值输出在 "adc3 conv:" 日志里。 */
uint8_t fx_adc_3ch_test(fx_adc3_result_t out[FX_ADC3_QTY]);

/** MER(PC10) 当前数字电平，0/1 */
uint8_t fx_adc_mer_level(void);

#endif /* __FX_ADC_H */
