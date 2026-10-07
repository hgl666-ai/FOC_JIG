/**
  ******************************************************************************
  * @file    measure.h
  * @brief   测量功能：9 路模拟量的采样、换算、滤波、判定
  *
  * 依据 docs/06_测量通道分压逆推.md（通道/分压/k 全部来自原理图追踪结果）。
  *
  * 分层：
  *   adc.c     只负责把 ADC1 配成 10 通道扫描 + DMA 循环搬运（9 路 + VREFINT）
  *   measure.c 通道表 + 码值→电压换算 + 指数滑动平均 + 门限判定 + 调试打印
  *
  * 换算：V_in = counts * (VDDA / 4095) / k      （k = 分压比，全部是电压跟随器, 增益 1）
  ******************************************************************************
  */
#ifndef __MEASURE_H
#define __MEASURE_H

#include <stdint.h>

/* ------------------------------------------------------------------ */
/* 通道定义：顺序 = ADC 扫描顺序 = DMA 缓冲下标                        */
/* ------------------------------------------------------------------ */
typedef enum {
    MEAS_CH_V24 = 0,    /* 24V 母线       ADC1_IN5  PA5 */
    MEAS_CH_V10,        /* 10V 驱动供电   ADC1_IN4  PA4 */
    MEAS_CH_V5,         /* 5V 电源        ADC1_IN0  PA0 */
    MEAS_CH_V33,        /* 3.3V 电源      ADC1_IN1  PA1 */
    MEAS_CH_PH_A,       /* A 相对功率地   ADC1_IN8  PB0 */
    MEAS_CH_PH_B,       /* B 相对功率地   ADC1_IN7  PA7 */
    MEAS_CH_PH_C,       /* C 相对功率地   ADC1_IN6  PA6 */
    MEAS_CH_CUR1,       /* 电流检测 1     ADC1_IN2  PA2 */
    MEAS_CH_CUR2,       /* 电流检测 2     ADC1_IN3  PA3 */
    MEAS_CH_COUNT,      /* = 9，对外可读的通道数 */
    MEAS_CH_VREF = MEAS_CH_COUNT,   /* 内部基准（仅用于 VDDA 校准，无门限） */
    MEAS_SCAN_COUNT                 /* = 10，DMA 缓冲长度 */
} meas_id_t;

/* ADC 通道映射：adc.c 配置扫描时也用这几个宏，避免两处写岔 */
#define MEAS_ADC_CH_V24     ADC_CHANNEL_5
#define MEAS_ADC_CH_V10     ADC_CHANNEL_4
#define MEAS_ADC_CH_V5      ADC_CHANNEL_0
#define MEAS_ADC_CH_V33     ADC_CHANNEL_1
#define MEAS_ADC_CH_PH_A    ADC_CHANNEL_8
#define MEAS_ADC_CH_PH_B    ADC_CHANNEL_7
#define MEAS_ADC_CH_PH_C    ADC_CHANNEL_6
#define MEAS_ADC_CH_CUR1    ADC_CHANNEL_2
#define MEAS_ADC_CH_CUR2    ADC_CHANNEL_3

/* ------------------------------------------------------------------ */
/* 配置                                                                */
/* ------------------------------------------------------------------ */
/* VDDA：默认写死 3300mV。LDO(AMS1117) 精度约 ±1%，
 * 比 F103 内部 VREFINT 的 ±3% 还准，所以默认不用 VREFINT 换算。
 * 若要做漂移跟踪：先用万用表标定出本板 VREFINT 的真实值填进
 * MEAS_VREFINT_MV，再把 MEAS_USE_VREFINT 置 1。 */
#ifndef MEAS_USE_VREFINT
#define MEAS_USE_VREFINT    0
#endif
#define MEAS_VDDA_MV        3300u
#define MEAS_VREFINT_MV     1200u

/* 连续多少次轮询都落在门限内才判合格（主循环约 1kHz，8 次 ≈ 8ms） */
#define MEAS_STABLE_N       8u

/* 原始码值贴边（<=2 或 >=4093）说明可能开路/短路，单独标记 */
#define MEAS_SAT_LOW        2u
#define MEAS_SAT_HIGH       4093u

/* 调试打印周期（ms）。量产治具不接调试串口时置 0 关掉。 */
#ifndef MEAS_PRINT_PERIOD_MS
#define MEAS_PRINT_PERIOD_MS 1000u
#endif

/* ------------------------------------------------------------------ */
/* 对外结果                                                            */
/* ------------------------------------------------------------------ */
typedef struct {
    uint16_t raw;       /* 最近一次原始码值 */
    uint32_t mv;        /* 滤波后的**输入侧**电压 (mV) */
    uint8_t  ok;        /* 1 = 连续 MEAS_STABLE_N 次都在门限内 */
    uint8_t  sat;       /* 1 = 码值贴边（可疑） */
} meas_value_t;

/* 与 U2 的交叉校验结果（当前未启用，接口先留着） */
typedef enum {
    MEAS_CC_DISABLED = 0,   /* 未启用 */
    MEAS_CC_OK,             /* 两边读数一致 */
    MEAS_CC_MISMATCH        /* 超差：治具自身可能有问题 */
} meas_cc_t;

void        MEAS_Init(void);
void        MEAS_Poll(void);                 /* 主循环周期调用 */

void        MEAS_Get(meas_id_t ch, meas_value_t *out);
uint32_t    MEAS_GetMv(meas_id_t ch);
uint8_t     MEAS_IsOk(meas_id_t ch);
uint8_t     MEAS_AllOk(void);
uint32_t    MEAS_GetFaultMask(void);         /* bit n = 通道 n 不合格 */
uint32_t    MEAS_GetVddaMv(void);
const char *MEAS_Name(meas_id_t ch);

void        MEAS_Print(void);                /* 经 USART1(PA9) 打印一行 */
void        MEAS_PrintHeader(void);

/**
  * @brief  与 U2 的读数做交叉校验
  * @note   当前返回 MEAS_CC_DISABLED。启用时：U2 用同一张表采样，经
  *         U1<->U2 链路（PB9 <-> U2.PA11）上报，本函数逐通道比对，
  *         差异超过 MEAS_CC_TOL_MV 即报 MISMATCH。
  */
meas_cc_t   MEAS_CrossCheck(void);

#ifdef MEAS_HOST_TEST
/* 宿主单测用：直接注入 ADC 原始码值（源文件里对应的缓冲是 static） */
void MEAS_TestInject(meas_id_t ch, uint16_t counts);
void MEAS_TestInjectVref(uint16_t counts);
#endif

#endif /* __MEASURE_H */
