/**
  ******************************************************************************
  * @file    sn_report.c
  * @brief   板子 SN 上报的纯逻辑层（组帧 / XOR / 去重）
  *
  * 不碰 SWD、不碰引脚、不碰 HAL —— 所以能宿主穷举测试（test/sn_report_test.c）。
  * 真正"读 UID"在调用方：app.c 在 0x0112 烧录成功后调 SWD 读 12B，
  * 再交给本模块组帧、去重、经 APP_SendEvent(0x0024, ...) 主动发出。
  *
  * ⚠ 本文件里**不许出现中文字符串字面量**：ARMCC 5.06 会把 UTF-8 中文
  *   字符串字面量解析成一堆 "missing closing quote"（本轮打 enc_chk 时踩过）。
  ******************************************************************************
  */
#include "sn_report.h"
#include <string.h>

/* 上一次上报过的 UID（RAM 里，掉电即失；上电第一块板一定会报） */
static uint8_t  s_last_uid[SN_UID_LEN];
static uint8_t  s_have_last;

uint8_t SnReport_UidXor(const uint8_t uid[SN_UID_LEN])
{
    uint8_t x = 0u;
    uint8_t i;

    if (uid == 0) {
        return 0u;
    }
    for (i = 0u; i < (uint8_t)SN_UID_LEN; i++) {
        x = (uint8_t)(x ^ uid[i]);
    }
    return x;
}

uint8_t SnReport_Build(const uint8_t uid[SN_UID_LEN], uint8_t out[SN_REPORT_LEN])
{
    if ((uid == 0) || (out == 0)) {
        return 0u;
    }
    out[0] = SN_ALGO_VER;                                   /* [0]  算法版本 */
    memcpy(&out[1], uid, SN_UID_LEN);                       /* [1..12] UID  */
    out[13] = SnReport_UidXor(uid);                         /* [13] 校验    */
    out[14] = 0u;                                           /* [14..15] 预留 */
    out[15] = 0u;
    return (uint8_t)SN_REPORT_LEN;
}

uint8_t SnReport_IsNewBoard(const uint8_t uid[SN_UID_LEN])
{
    if (uid == 0) {
        return 0u;
    }
    if (s_have_last == 0u) {
        return 1u;                                          /* 上电后第一块 */
    }
    return (memcmp(s_last_uid, uid, SN_UID_LEN) != 0) ? 1u : 0u;
}

void SnReport_MarkReported(const uint8_t uid[SN_UID_LEN])
{
    if (uid == 0) {
        return;
    }
    memcpy(s_last_uid, uid, SN_UID_LEN);
    s_have_last = 1u;
}

void SnReport_Reset(void)
{
    memset(s_last_uid, 0, sizeof(s_last_uid));
    s_have_last = 0u;
}

/*==============================================================================
 * 自动检测："插上一块板就报 SN"
 *----------------------------------------------------------------------------
 * 标志位与参考项目 diameter_JIG 的 SN_AutoPoll **一一对应**：
 *     s_done     ← sn_done    本次插入已处理过（拔板清零）
 *     s_in_cnt   ← in_cnt     连续在位计数（去抖）
 *     s_poll_tick← poll_tick  轮询节拍
 *
 * 和参考项目的区别：
 *   1. 判在位用 **3.3V 通道 raw**（实测插板 3707 / 拔板 1），它用的是 U1 的 ADC 通道；
 *   2. 读 SN 走 **SWD 读 UID**（不停机、无扰），它要 UART ISP 复位进 bootloader；
 *   3. 不加失败冷却（用户 2026-11-16 定：产线上 SN 是第一步，读不到就不往下走）。
 *
 * 去重分工（这是"烧录上报"和"插板上报"不重复的关键）：
 *   · 本路径**每次都报**（同板插两次就报两次），报完调 SnReport_MarkReported()；
 *   · 烧录路径先 SnReport_IsNewBoard() 比对 —— 插板已经报过 → 烧录不再报。
 *============================================================================*/
static uint8_t  s_done;              /* 1 = 本次插入已处理 */
static uint8_t  s_in_cnt;            /* 连续在位次数 */
static uint32_t s_poll_tick;         /* 上次轮询时刻 */

uint8_t SnReport_AutoPoll(sn_det_state_t st, uint32_t now_ms)
{
    /* 中间值：不做判断，保持现有状态（滞回）*/
    if (st == SN_DET_ST_MID) {
        return 0u;
    }

    /* 节流：不到周期直接返回（拔板检测也要限速，否则每轮都清标志）*/
    if ((uint32_t)(now_ms - s_poll_tick) < (uint32_t)SN_DET_POLL_MS) {
        return 0u;
    }
    s_poll_tick = now_ms;

    if (st == SN_DET_ST_PRESENT) {
        if (s_in_cnt < 0xFFu) {
            s_in_cnt++;
        }
        /* 三个条件同时满足才动手：连续 4 次在位 + 本次还没读过 */
        if ((s_in_cnt >= (uint8_t)SN_DET_DEBOUNCE) && (s_done == 0u)) {
            s_done = 1u;             /* 先置位，防重入 */
            return 1u;
        }
        return 0u;
    }

    /* 拔板：清两个标志，允许下次插入再报 */
    s_in_cnt = 0u;
    s_done   = 0u;
    return 0u;
}

void SnReport_AutoResult(uint8_t ok)
{
    if (ok == 0u) {
        s_done = 0u;                 /* 读失败 → 清标志，下轮重试 */
    }
}

void SnReport_AutoGetState(uint8_t *done, uint8_t *in_cnt)
{
    if (done != 0) {
        *done = s_done;
    }
    if (in_cnt != 0) {
        *in_cnt = s_in_cnt;
    }
}
