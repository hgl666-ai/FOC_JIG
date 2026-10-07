/*
 * AHB-AP 实现
 *
 * CSW 的 AddrInc 字段语义（ADIv5 IHI0031H 表 C2-1）：
 *   0b00 = 自增关闭        ← 连续 DRW 访问会一直打同一个地址（不是我们想要的"自增"）
 *   0b01 = increment single ← 每次 DRW 访问后按访问宽度自增（块读写用这个）
 * 参考实现：DAPLink swd_host.c 的 CSW_VALUE 就是 0x23000052（含 CSW_SADDRINC）。
 */
#include "swd_config.h"
#include "swd_phy.h"
#include "swd_dp.h"
#include "swd_memap.h"

static uint32_t s_csw  = 0u;   /* 0 = 尚未写入 */
static uint8_t  s_apok = 0u;

static int memap_prepare(uint32_t csw)
{
    int r;

    if (s_apok == 0u) {
        r = SWD_SelectAP(0u, 0u);          /* 单 AP（APSEL=0），bank0 */
        if (r != SWD_OK) {
            return r;
        }
        s_apok = 1u;
        s_csw  = 0u;
    }
    if (s_csw != csw) {
        uint32_t v = csw;
        r = SWD_APWrite(SWD_AP_CSW, v);
        if (r != SWD_OK) {
            return r;
        }
        s_csw = csw;
    }
    return SWD_OK;
}

int SWD_MemInit(void)
{
    s_apok = 0u;
    s_csw  = 0u;
    return memap_prepare(AHBAP_CSW_WORD);
}

/* 单次读的通用实现（可指定 CSW，供测试验证字段语义） */
int SWD_MemRead32WithCsw(uint32_t addr, uint32_t csw, uint32_t *value)
{
    uint32_t dummy = 0u;
    int      r;

    r = memap_prepare(csw);
    if (r != SWD_OK) {
        return r;
    }
    r = SWD_APWrite(SWD_AP_TAR, addr & ~3u);
    if (r != SWD_OK) {
        return r;
    }
    r = SWD_APRead(SWD_AP_DRW, &dummy);        /* posted：发起本次传输，回的是上次的值 */
    if (r != SWD_OK) {
        return r;
    }
    r = SWD_DPRead(SWD_DP_RDBUFF, value);      /* 本次结果 */
    if (r != SWD_OK) {
        return r;
    }
    return SWD_OK;
}

int SWD_MemRead32(uint32_t addr, uint32_t *value)
{
    return SWD_MemRead32WithCsw(addr, AHBAP_CSW_WORD, value);
}

int SWD_MemWrite32(uint32_t addr, uint32_t value)
{
    uint32_t v = value;
    int      r;

    r = memap_prepare(AHBAP_CSW_WORD);
    if (r != SWD_OK) {
        return r;
    }
    r = SWD_APWrite(SWD_AP_TAR, addr & ~3u);
    if (r != SWD_OK) {
        return r;
    }
    r = SWD_APWrite(SWD_AP_DRW, v);
    if (r != SWD_OK) {
        return r;
    }
    return SWD_OK;
}

int SWD_MemReadBlock(uint32_t addr, uint32_t *buf, uint32_t n)
{
    uint32_t dummy = 0u;
    uint32_t i;
    int      r;

    if (n == 0u) {
        return SWD_OK;
    }
    /* TAR 只写一次，之后靠 CSW.AddrInc=0b01 自动递增 */
    r = memap_prepare(AHBAP_CSW_WORD);
    if (r != SWD_OK) {
        return r;
    }
    r = SWD_APWrite(SWD_AP_TAR, addr & ~3u);
    if (r != SWD_OK) {
        return r;
    }
    r = SWD_APRead(SWD_AP_DRW, &dummy);        /* 丢弃脏数据，发起第 1 次传输 */
    if (r != SWD_OK) {
        return r;
    }
    for (i = 0u; i < n; i++) {
        r = SWD_APRead(SWD_AP_DRW, &buf[i]);   /* 第 i 次：拿到第 i 个数据 */
        if (r != SWD_OK) {
            return r;
        }
    }
    return SWD_OK;
}

int SWD_MemWriteBlock(uint32_t addr, const uint32_t *buf, uint32_t n)
{
    uint32_t i;
    int      r;

    if (n == 0u) {
        return SWD_OK;
    }
    r = memap_prepare(AHBAP_CSW_WORD);
    if (r != SWD_OK) {
        return r;
    }
    r = SWD_APWrite(SWD_AP_TAR, addr & ~3u);
    if (r != SWD_OK) {
        return r;
    }
    for (i = 0u; i < n; i++) {
        uint32_t v = buf[i];
        r = SWD_APWrite(SWD_AP_DRW, v);
        if (r != SWD_OK) {
            return r;
        }
    }
    return SWD_OK;
}
