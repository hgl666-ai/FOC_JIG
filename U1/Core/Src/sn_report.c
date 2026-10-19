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
