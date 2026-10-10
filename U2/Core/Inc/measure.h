/**
  ******************************************************************************
  * @file    measure.h
  * @brief   测量功能：9 路模拟量的采样、换算、滤波、判定、快照与交叉校验
  *
  * 依据 docs/06_测量通道分压逆推.md（通道/分压/k 全部来自原理图追踪结果）
  * 与 docs/07_测量功能固件框架.md（分层与数据包设计）。
  *
  * 分层：
  *   adc.c        只负责把 ADC1 配成 10 通道扫描 + DMA 循环搬运（9 路 + VREFINT）
  *   measure.c    本文件：信号表 + 码值→电压换算 + EMA + 判定 + 交叉校验 + 快照
  *   meas_pack.c  把快照编成**定长线格式**（PC 上报与 U1↔U2 链路共用同一份布局）
  *
  * 换算：V_in = counts * (VDDA / 4095) / k      （k = 分压比，前级是增益 1 的跟随器）
  *
  * ⚠ 两片各有一份本文件与 measure.c 的**逐字节副本**：
  *   - 表里的"信号序号"两片一致（V24,V10,V5,V33,PHA,PHB,PHC,CUR1,CUR2）；
  *   - **信号 → ADC 通道号**的映射每片不同，放在各自的 adc.c 扫描表里，
  *     所以本文件里**不允许出现** ADC_CHANNEL_x 之类的引脚/通道号。
  ******************************************************************************
  */
#ifndef __MEASURE_H
#define __MEASURE_H

#include <stdint.h>

/* ------------------------------------------------------------------ */
/* 通道定义：顺序 = ADC 扫描顺序 = DMA 缓冲下标（两片必须一致）        */
/* ------------------------------------------------------------------ */
typedef enum {
    MEAS_CH_V24 = 0,    /* 24V 母线                     */
    MEAS_CH_V10,        /* 10V 驱动供电                 */
    MEAS_CH_V5,         /* 5V 电源                      */
    MEAS_CH_V33,        /* 3.3V 电源                    */
    MEAS_CH_PH_A,       /* A 相对功率地                 */
    MEAS_CH_PH_B,       /* B 相对功率地                 */
    MEAS_CH_PH_C,       /* C 相对功率地                 */
    MEAS_CH_CUR1,       /* 电流检测 1                   */
    MEAS_CH_CUR2,       /* 电流检测 2                   */
    MEAS_CH_COUNT,      /* = 9，对外可读的通道数         */
    MEAS_CH_VREF = MEAS_CH_COUNT,   /* 内部基准（仅用于 VDDA 校准，无门限） */
    MEAS_SCAN_COUNT                 /* = 10，DMA 缓冲长度            */
} meas_id_t;

/* ------------------------------------------------------------------ */
/* 配置                                                                */
/* ------------------------------------------------------------------ */
#include "jig_mode.h"          /* 运行模式/开关的唯一入口（含 MEAS_* 开关） */

/* VDDA：见 jig_mode.h §3.5（MEAS_USE_VREFINT / MEAS_VREFINT_MV） */
#define MEAS_VDDA_MV        3300u

/* 连续多少次轮询都落在门限内才判合格（主循环约 1kHz，8 次 ≈ 8ms） */
#define MEAS_STABLE_N       8u

/* 一次"测试"的最长等待：稳定计数一直凑不齐就别卡着不放（PC 在等 ACK） */
#define MEAS_TEST_TIMEOUT_MS 200u

/* 原始码值贴边（<=2 或 >=4093）说明可能开路/短路，单独标记 */
#define MEAS_SAT_LOW        2u
#define MEAS_SAT_HIGH       4093u

/* 交叉校验是否"必须"、调试打印开关：见 jig_mode.h §3.1 / §3.4（随 JIG_MODE 联动） */

/* ------------------------------------------------------------------ */
/* 对外结果                                                            */
/* ------------------------------------------------------------------ */
typedef struct {
    uint16_t raw;       /* 最近一次原始码值 */
    uint32_t mv;        /* 滤波后的**输入侧**电压 (mV) */
    uint8_t  ok;        /* 1 = 连续 MEAS_STABLE_N 次都在门限内 */
    uint8_t  sat;       /* 1 = 码值贴边（可疑） */
} meas_value_t;

/* 快照的整体标志位 */
#define MEAS_SNAP_F_VALID     (1u << 0)   /* 数据有效 */
#define MEAS_SNAP_F_VDDA_CAL  (1u << 1)   /* VDDA 用 VREFINT 校准过 */
#define MEAS_SNAP_F_OVERFLOW  (1u << 2)   /* 采集溢出/被截断 */
#define MEAS_SNAP_F_ALL_OK    (1u << 3)   /* 9 路全部合格 */

/* 快照里每个通道的标志位 */
#define MEAS_CHF_OK           (1u << 0)   /* 该通道合格 */
#define MEAS_CHF_SAT          (1u << 1)   /* 该通道码值贴边（硬件可疑） */

/* 交叉校验块的标志位（PC 应答里用） */
#define MEAS_CC_F_DONE        (1u << 0)   /* 这次做过交叉校验 */
#define MEAS_CC_F_MISMATCH    (1u << 1)   /* 至少一路超差 */
#define MEAS_CC_F_NO_PEER     (1u << 2)   /* 对侧（U2）没有应答 */

/* 一次采集的完整快照（内存表示；线格式由 meas_pack.c 编码，两者布局无关） */
typedef struct {
    uint8_t  flags;                     /* MEAS_SNAP_F_* */
    uint8_t  ch_count;                  /* = MEAS_CH_COUNT */
    uint16_t round;                     /* 采集回合号：每完成一轮扫描 +1 */
    uint16_t dt_ms;                     /* 距上一回合的毫秒数（饱和 65535） */
    uint16_t vdda_mv;                   /* 本回合实际使用的 VDDA */
    uint16_t cfg_crc;                   /* 门限/容差表的指纹（追溯"判定用的哪套配置"） */
    struct {
        uint16_t raw;
        uint16_t mv;
        uint8_t  flags;                 /* MEAS_CHF_* */
    } ch[MEAS_CH_COUNT];
} meas_snapshot_t;

/* ------------------------------------------------------------------ */
/* 生命周期与周期任务                                                  */
/* ------------------------------------------------------------------ */
void        MEAS_Init(void);
void        MEAS_Poll(void);                 /* 主循环周期调用（采一轮就更新一次） */

/* ------------------------------------------------------------------ */
/* 取值接口                                                            */
/* ------------------------------------------------------------------ */
void        MEAS_Get(meas_id_t ch, meas_value_t *out);
uint32_t    MEAS_GetMv(meas_id_t ch);
uint8_t     MEAS_IsOk(meas_id_t ch);
uint8_t     MEAS_AllOk(void);
uint32_t    MEAS_GetFaultMask(void);         /* bit n = 通道 n 不合格 */
uint32_t    MEAS_GetVddaMv(void);
uint16_t    MEAS_GetCfgCrc(void);            /* 配置表指纹 */
const char *MEAS_Name(meas_id_t ch);

/** 取当前这一批数据的快照（含回合号/时间戳/每通道 raw+mv+flags） */
void        MEAS_Snapshot(meas_snapshot_t *out);

/* ------------------------------------------------------------------ */
/* "跑一次测试"的握手（供 app.c 的 0x000E 命令用）                     */
/* ------------------------------------------------------------------ */
/**
  * @brief  开始一次测试：清掉所有稳定计数，从零开始累计 MEAS_STABLE_N 次
  * @note   MEAS_Poll 负责累计；测试是否完成由 MEAS_TestPoll 判断
  */
void        MEAS_TestBegin(void);

/**
  * @brief  推进一次测试判定（每轮主循环调一次）
  * @retval 0 = 还在等稳定, 1 = 已完成（可以取快照了）, 2 = 超时
  */
uint8_t     MEAS_TestPoll(void);

/* ------------------------------------------------------------------ */
/* 与 U2 的交叉校验                                                    */
/* ------------------------------------------------------------------ */
/**
  * @brief  逐通道比对两份快照（本片 vs U2）
  * @param  mine  本片快照
  * @param  peer  对侧快照
  * @param  mism  [out, 可空] 每通道 0/1；本函数只写 MEAS_CH_COUNT 个元素
  * @retval 超差掩码 u16（bit0..bit8 对应信号序号 0..8，1 = 超差）
  * @note   容差逐通道不同（相电压被 PWM 斩波，容差要宽得多），
  *         表在 measure.c 的 k_cfg[].cc_tol_mv，改判定标准只改这一张表。
  */
uint16_t    MEAS_Compare(const meas_snapshot_t *mine,
                         const meas_snapshot_t *peer,
                         uint8_t *mism);

/* ------------------------------------------------------------------ */
/* 调试打印                                                            */
/* ------------------------------------------------------------------ */
void        MEAS_Print(void);                /* 经 U1 的调试口打印一行（默认关） */
void        MEAS_PrintHeader(void);

#ifdef MEAS_HOST_TEST
/* 宿主单测用：直接注入 ADC 原始码值（源文件里对应的缓冲是 static） */
void MEAS_TestInject(meas_id_t ch, uint16_t counts);
void MEAS_TestInjectVref(uint16_t counts);
#endif

#endif /* __MEASURE_H */
