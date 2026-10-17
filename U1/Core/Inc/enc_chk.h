/**
  ******************************************************************************
  * @file    enc_chk.h
  * @brief   ⑤ 编码器检测的**判据层**：治具意图 vs U2 实测 vs 被测板上报
  *
  * 任务 ⑤ 要回答的问题
  * ------------------
  * 被测板的编码器接口（接线 → 引脚 → 定时器解码）对不对？
  * 做法是**三方对齐**：
  *     `治具意图(我发了几个边沿) == U2 实测(同一组线上数到几个) == DUT 上报(它自己数到几个)`
  * 三边都对 → 编码器通路（含 DUT 的解码和计数）是好的。
  * 任何一边断了都能**指出来是哪一环**：
  *   - U2 对不上 → 信号根本没出去（治具的 TIM4 输出 / 接线 / 三根线被拉死）；
  *   - 只有 DUT 对不上 → DUT 的解码或计数有问题（极性、倍频、边沿判定）；
  *   - 两边都对不上 → 线接错了（A/B/Z 三者互相串）。
  *
  * 量纲：**边沿个数**
  * ------------------
  * 治具每发 1 个完整方波 = 2 个边沿（上升 + 下降），所以：
  *     `意图边沿数 = 2 × 发出的脉冲数`
  * U2 和 DUT 都必须按"边沿个数"上报（`docs/15` §7.9 已经这么规定），
  * 否则一边数脉冲一边数边沿，数字天生差一倍 —— 这正是本层要挡住的错。
  *
  * ⚠ 与 ②④⑥ 同一条规矩：**"不判"不等于"失败"**（数据不全时不能冤判）。
  *
  * 只 U1 用（判定只在 U1 做，同 measure 的既有原则）→ **不做两片副本**。
  ******************************************************************************
  */
#ifndef __ENC_CHK_H
#define __ENC_CHK_H

#include <stdint.h>

/* ------------------------------------------------------------------ */
/* 1. 判据参数                                                        */
/* ------------------------------------------------------------------ */
#ifndef ENC_CHK_TOL_EDGES
#define ENC_CHK_TOL_EDGES   0u      /* 允许的边沿数偏差，默认 0（干净线上应该完全相等） */
#endif

/* 三相：A / B / Z（与 pulse/measure 的相位序号一致） */
#define ENC_CHK_PH_A        0u
#define ENC_CHK_PH_B        1u
#define ENC_CHK_PH_Z        2u
#define ENC_CHK_PH_COUNT    3u

/* ------------------------------------------------------------------ */
/* 2. 旗标（单路）                                                     */
/* ------------------------------------------------------------------ */
#define ENC_CHK_F_OK        0x0001u  /* 三边一致（只在 U2 与 DUT 都有数据时才给） */
#define ENC_CHK_F_NODATA    0x0002u  /* 有一侧没有数据 → **不判**（不是失败） */
#define ENC_CHK_F_U2_DIFF   0x0004u  /* U2 实测 ≠ 治具意图（信号没出去 / U2 数漏） */
#define ENC_CHK_F_DUT_DIFF  0x0008u  /* DUT 上报 ≠ 治具意图（DUT 解码/计数问题） */

/* 汇总位（整份报告） */
#define ENC_CHK_SUM_DONE       0x0001u  /* 三相都过了一遍 */
#define ENC_CHK_SUM_ANY_BAD    0x0002u  /* 至少一路不一致（真故障） */
#define ENC_CHK_SUM_ANY_NODATA 0x0004u  /* 至少一路没判成（数据不全） */
#define ENC_CHK_SUM_U2_LOST    0x0008u  /* U2 报它自己丢过边沿（诊断/警告，不单独判失败） */
#define ENC_CHK_SUM_DIR_ZERO   0x0010u  /* 发了脉冲但 DUT 报"方向 = 没动"（警告） */

/* ------------------------------------------------------------------ */
/* 3. 输入 / 输出                                                      */
/* ------------------------------------------------------------------ */
typedef struct {
    uint16_t exp_edges;     /* 治具意图的边沿数（= 2 × 脉冲数，见 EncChk_EdgesFromPulses） */
    uint32_t u2_edges;      /* U2 实测的边沿数 */
    uint32_t dut_edges;     /* 被测板上报的边沿数 */
    uint8_t  has_u2;        /* 1 = 这一相有 U2 数据 */
    uint8_t  has_dut;       /* 1 = 这一相有 DUT 数据 */
} enc_chk_in_t;

typedef struct {
    int32_t  err_u2;        /* u2 − exp（有符号，方便看"多了还是少了"） */
    int32_t  err_dut;       /* dut − exp */
    uint16_t exp_edges;
    uint32_t u2_edges;
    uint32_t dut_edges;
    uint16_t flags;         /* ENC_CHK_F_* */
} enc_chk_out_t;

/* ------------------------------------------------------------------ */
/* 4. API                                                             */
/* ------------------------------------------------------------------ */
/**
  * @brief  脉冲数 → 边沿数（×2，饱和到 0xFFFF）
  * @note   脉冲数超过 32767 时返回 0xFFFF（再大就没有可靠的 u16 表示了）
  */
uint16_t EncChk_EdgesFromPulses(uint16_t pulses);

/**
  * @brief  判一路（A / B / Z）
  * @param  in  [in]  三个数 + 可用标志
  * @param  tol [in]  允许的边沿数偏差
  * @param  out [out] 详细结果（可传 0）
  * @retval 该路的 flags（ENC_CHK_F_*）；参数为空返回 0
  */
uint16_t EncChk_Judge(const enc_chk_in_t *in, uint32_t tol, enc_chk_out_t *out);

/**
  * @brief  判三相 + 出汇总
  * @param  in       [in]  ENC_CHK_PH_COUNT 路输入
  * @param  tol      [in]  允许的边沿数偏差
  * @param  u2_lost  [in]  U2 报的丢边沿次数（诊断，0 = 没丢）
  * @param  dut_dir  [in]  DUT 报的解码方向（+1 / -1 / 0）
  * @param  out      [out] 三相详细结果（可传 0）
  * @retval 汇总位（ENC_CHK_SUM_*）
  */
uint16_t EncChk_All(const enc_chk_in_t in[ENC_CHK_PH_COUNT], uint32_t tol,
                    uint16_t u2_lost, int8_t dut_dir,
                    enc_chk_out_t out[ENC_CHK_PH_COUNT]);

/** 单路旗标的人体可读文本（调试打印用；返回静态字符串） */
const char *EncChk_FlagText(uint16_t flags);

/** 汇总位的人体可读文本（调试打印用；返回静态字符串） */
const char *EncChk_SumText(uint16_t sum);

#endif /* __ENC_CHK_H */
