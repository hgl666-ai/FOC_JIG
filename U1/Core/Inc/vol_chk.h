/**
  ******************************************************************************
  * @file    vol_chk.h
  * @brief   ② 电压检测的**判据层**：治具读到的 9 路 vs 被测板自己报的
  *
  * 这一层补的是什么（别重复造轮子）
  * --------------------------------
  * 9 路模拟量的**采集、换算、门限判定、U1↔U2 交叉校验**早在 `0x000E` 里就有了
  * （`measure.c` / `meas_pack.c`，见 `docs/07`）—— 那部分不用再写一遍。
  * 这里补的是**第二条腿**：**被测板自己也在采同一批电压**（`DUT_GET_ADC`，`docs/15` §7.8），
  * 把它报回来的值和治具读到的逐通道比 —— 对得上，说明治具的测量通道和被测板的
  * 采样/换算都是好的；对不上，就按通道定位是哪一路的问题。
  *
  * 于是 ② 的判定一共有三份数据可比（这就是"冗余"的价值）：
  *   ① U1 读的（治具主控）
  *   ② U2 读的（治具副控）—— 已有，走 `MEAS_XCHK`，21B 交叉校验块
  *   ③ **DUT 自己报的** —— 本模块负责的这一条
  *
  * 判定规矩（与 ④⑥ 一致）
  * ----------------------
  *   - **不判**：任一侧数据不可用（快照无效 / 通道贴边 / DUT 没报这一路）→ `NODATA`；
  *   - **失败**：两边差超容差 → `DEVIATION`；
  *   - 否则 `OK`。
  * 容差 = `max(对侧读数的 5%, 200mV)`；`24V` 那一路 5% 就是 1.2V，够宽；
  * 小信号通道靠 200mV 的绝对下限兜底。
  *
  * ⚠ **门限值本身仍是占位**：`measure.c` 的 `k_lim_min/k_lim_max` 现在还是全量程
  *   （"先不判"），真正的规格范围要等被测板规格 + 上板标定（`docs/04` Q5）。
  *   本模块**不碰**那些门限 —— 它只管"两边读数是否一致"。
  *
  * 只 U1 用（判定只在 U1 做）→ **不做两片副本**。
  ******************************************************************************
  */
#ifndef __VOL_CHK_H
#define __VOL_CHK_H

#include <stdint.h>
#include "measure.h"
#include "dut_proto.h"

/* ------------------------------------------------------------------ */
/* 1. 判据参数                                                        */
/* ------------------------------------------------------------------ */
#ifndef VOL_CHK_TOL_PCT
#define VOL_CHK_TOL_PCT     5u      /* 容差 = 对侧读数的 5% */
#endif
#ifndef VOL_CHK_TOL_MIN_MV
#define VOL_CHK_TOL_MIN_MV  200u    /* 容差绝对下限 */
#endif

/* ------------------------------------------------------------------ */
/* 2. 旗标                                                            */
/* ------------------------------------------------------------------ */
#define VOL_CHK_F_OK        0x0001u  /* 这一路判定通过 */
#define VOL_CHK_F_NODATA    0x0002u  /* 数据不全 → **不判**（不是失败） */
#define VOL_CHK_F_DEVIATION 0x0008u  /* 与 DUT 的读数差超容差（真故障） */

/* VolChk_All 的汇总位 */
#define VOL_CHK_SUM_DONE       0x0001u  /* 9 路都过了一遍 */
#define VOL_CHK_SUM_ANY_BAD    0x0002u  /* 至少一路真不合格（不含"不判"） */
#define VOL_CHK_SUM_ANY_NODATA 0x0004u  /* 至少一路没判成（DUT 没报 / 数据不可用） */

/* ------------------------------------------------------------------ */
/* 3. 输入 / 输出                                                      */
/* ------------------------------------------------------------------ */
typedef struct {
    uint16_t jig_mv;    /* 治具读到的（信号侧 mV） */
    uint16_t dut_mv;    /* 被测板自己报的（信号侧 mV，见 docs/15 §7.8） */
    uint8_t  jig_ok;    /* 治具侧可用（快照有效 + 通道没贴边） */
    uint8_t  dut_ok;    /* 被测板报了这一路 */
} vol_chk_in_t;

typedef struct {
    int32_t  err_mv;    /* jig − dut（有符号，便于看偏高还是偏低） */
    uint32_t tol_mv;    /* 本次用的容差 */
    uint16_t flags;     /* VOL_CHK_F_* */
} vol_chk_out_t;

/* ------------------------------------------------------------------ */
/* 4. API                                                             */
/* ------------------------------------------------------------------ */
/**
  * @brief  判一路
  * @retval 该路的 flags（VOL_CHK_F_*）；参数为空返回 0
  */
uint16_t VolChk_Judge(const vol_chk_in_t *in, vol_chk_out_t *out);

/**
  * @brief  从治具快照取某一路的 mV
  * @retval 1 = 可用；0 = 不可用（快照无效 / 溢出 / 该通道贴边 / 参数不对）
  */
uint8_t  VolChk_TakeJig(const meas_snapshot_t *snap, uint8_t ch, uint16_t *mv);

/**
  * @brief  从 DUT 的 GET_ADC 应答里取某一路的 mV
  * @retval 1 = DUT 报了这一路；0 = 没报
  */
uint8_t  VolChk_TakeDut(const dut_adc_t *a, uint8_t ch, uint16_t *mv);

/**
  * @brief  9 路一起判
  * @param  dut_mask DUT 在 INFO 里声明的能力位图（bit i = 支持通道 i）；
  *                  没声明的通道直接算 `NODATA`，不去问
  * @param  out      [out] 9 个结果（按 MEAS_CH_* 顺序）；可传 0
  * @retval VOL_CHK_SUM_*
  */
uint16_t VolChk_All(const meas_snapshot_t *jig, const dut_adc_t *dut, uint16_t dut_mask,
                    vol_chk_out_t out[MEAS_CH_COUNT]);

/** 人体可读的旗标文本（调试打印用） */
const char *VolChk_FlagText(uint16_t flags);

#endif /* __VOL_CHK_H */
