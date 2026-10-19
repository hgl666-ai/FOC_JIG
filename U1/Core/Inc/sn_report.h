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

/* SWD 引脚占位：等 docs/15 D9（治具用哪两个脚烧被测板）确认后改这里 */
#ifndef SN_SWD_PIN_CLK
#define SN_SWD_PIN_CLK   0u
#endif
#ifndef SN_SWD_PIN_DIO
#define SN_SWD_PIN_DIO   0u
#endif

/* ------------------------------------------------------------------ */
/* 2. API                                                             */
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

#endif /* __SN_REPORT_H */
