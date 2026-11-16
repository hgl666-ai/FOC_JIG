/**
  ******************************************************************************
  * @file    sn_report.h
  * @brief   板子 SN 码上报（治具 → PC，**主动帧** 0x0024）
  *
  * 干什么
  * ------
  * 烧录成功后，读被测板（STM32F405RGT6）的 **96-bit 唯一 ID**，
  * 算出一个可追溯的 SN 主动推给 PC —— PC 不用轮询，收到就落库（板子身份证）。
  *
  * 线格式（16 字节，见 INFO/命令大全.txt 第 22 条）
  * ------------------------------------------------
  *   [0]       SN 算法版本 = 0x01
  *   [1..12]   96-bit UID（12B，大端，来自 @0x1FFF7A10，即 RM0090 的
  *             Unique device ID register；数据手册已确认是 96 bit = 12B）
  *   [13]      UID 校验字节 = **12B UID 逐字节 XOR**（不含版本号本身）
  *   [14..15]  预留 = 0x0000
  *
  * 三条行为约定（和参考项目一致）
  * ------------------------------
  *   1. **触发点**：0x0112 烧录握手成功后（app.c 的烧录终态分支调本模块）；
  *   2. **去重**：和上次上报过的 UID 相同（同板重复烧录）→ **不重复上报**；
  *   3. **失败静默**：读 UID 失败就跳过，**不上报脏数据、也不影响烧录结果**。
  *
  * 分层
  * ----
  *   本模块只做**纯逻辑**（组帧 / 去重 / XOR），不知道 SWD、不知道引脚，
  *   所以能在宿主上穷举测试。真正的"读 UID"由调用方（app.c + SWD 层）负责。
  *   SWD 引脚的占位宏也放在这里，等 docs/15 D9 定了只改这一处。
  ******************************************************************************
  */
#ifndef __SN_REPORT_H
#define __SN_REPORT_H

#include <stdint.h>

/* ------------------------------------------------------------------ */
/* 1. 常量与占位宏                                                     */
/* ------------------------------------------------------------------ */
#define SN_ALGO_VER      0x01u      /* SN 算法版本；算法改了必须升这个号 */

#define SN_UID_LEN       12u        /* 96 bit = 12 字节（STM32F4 数据手册） */
#define SN_UID_WORDS     3u         /* 12B = 3 个 32 位字 */
#define SN_REPORT_LEN    16u        /* 0x0024 的 DATA 长度 */

/* UID 基址：STM32F405/407/415/417 = RM0090 Unique device ID register。
 * 三个 32 位字分别是 UID[31:0] / UID[63:32] / UID[95:64]，
 * 拼成 12B 大端时顺序是 UID0、UID1、UID2。 */
#ifndef SN_UID_BASE
#define SN_UID_BASE      0x1FFF7A10u
#endif

/* SWD 引脚：**本模块不再定义**（2026-10-27 清理死宏，全工程从未引用过它们）。
 * 历史留档（便于回溯）：这里曾有两个占位宏
 *     SN_SWD_PIN_CLK / SN_SWD_PIN_DIO   —— 都是 0u，从没被任何代码读取；
 * 真正决定 SWD 引脚的是 `swd_config.h`（现为 SWCLK = PA14 / SWDIO = PA13，
 * 2026-10-27 新版治具原理图确认）。以后别再往本文件加引脚宏。 */

/* ------------------------------------------------------------------ */
/* 2. 自动检测（"插上一块板就报 SN"）                                   */
/* ------------------------------------------------------------------ */
/* 2026-11-16 实测（治具 3.3V 通道 raw）：
 *     板子在场：3707 ~ 3708
 *     板子拔掉：1
 * 中间值一个都没出现过，所以阈值取得很宽、滞回极大。 */
#define SN_DET_POLL_MS       200u    /* 轮询周期 */
#define SN_DET_DEBOUNCE        4u    /* 连续 N 次确认才算稳定（800 ms）*/
#define SN_DET_PRESENT_RAW  2000u    /* > 此值认为板子在场（≈1800 mV）*/
#define SN_DET_ABSENT_RAW    100u    /* < 此值认为板子已拔掉 */

/* 在位状态（三态：中间值不作判断，保持原状态）*/
typedef enum {
    SN_DET_ST_MID     = 0u,          /* 在"在位/拔出"阈值之间：状态不变 */
    SN_DET_ST_PRESENT = 1u,          /* 板子在场 */
    SN_DET_ST_ABSENT  = 2u           /* 板子拔掉 */
} sn_det_state_t;

/* ------------------------------------------------------------------ */
/* 3. API                                                             */
/* ------------------------------------------------------------------ */
/**
  * @brief  把 12B UID 组装成 0x0024 的 16B DATA
  * @param  uid [in]  12 字节 UID（大端序，见 SN_UID_BASE 说明）
  * @param  out [out] 至少 SN_REPORT_LEN(16) 字节
  * @retval 写入字节数（= 16）；参数为空返回 0
  */
uint8_t SnReport_Build(const uint8_t uid[SN_UID_LEN], uint8_t out[SN_REPORT_LEN]);

/**
  * @brief  12B UID 的逐字节 XOR（就是我们上报的校验字节）
  * @retval 0~255；uid 为空返回 0
  */
uint8_t SnReport_UidXor(const uint8_t uid[SN_UID_LEN]);

/**
  * @brief  这块板要不要上报？（去重）
  * @retval 1 = 是新板（从没报过，或与上次不同）；0 = 与上次上报的 UID 相同，不必重复
  * @note   只读，不改状态；确认要上报后再调 SnReport_MarkReported()
  */
uint8_t SnReport_IsNewBoard(const uint8_t uid[SN_UID_LEN]);

/** 记录"这块板已经报过了"（供下一次去重比较） */
void    SnReport_MarkReported(const uint8_t uid[SN_UID_LEN]);

/** 清掉去重缓存（上电初始化 / 测试用） */
void    SnReport_Reset(void);

/**
  * @brief  自动检测轮询（主循环调用，内部按 SN_DET_POLL_MS 限速）
  * @param  st      当前在位状态（调用方把 3.3V 的 raw 按阈值转成三态）
  * @param  now_ms  当前毫秒时基（HAL_GetTick）
  * @retval 1 = **现在该读 UID 并上报了**；0 = 什么都不用做
  * @note   返回 1 时调用方必须读 UID，然后调 SnReport_AutoResult() 回报结果。
  *         **只在系统空闲（不在烧录/自检）时才能返回 1** —— 由调用方保证：
  *         非空闲时请传 SN_DET_ST_ABSENT，本模块会把标志清掉。
  */
uint8_t SnReport_AutoPoll(sn_det_state_t st, uint32_t now_ms);

/**
  * @brief  回报"刚才那次读 UID + 上报"的结果
  * @param  ok  1 = 读到了并已上报；0 = 失败（清标志，下个轮询再试）
  * @note   参考项目没有失败冷却：产线上 SN 是第一步，读不到就不往下走，
  *         所以这里失败就原样清标志、下轮重试。
  */
void    SnReport_AutoResult(uint8_t ok);

/** 自动检测的内部状态（测试用；生产代码不需要读）*/
void    SnReport_AutoGetState(uint8_t *done, uint8_t *in_cnt);

#endif /* __SN_REPORT_H */
