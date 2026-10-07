/*
 * SW-DP 层实现
 */
#include "swd_config.h"
#include "swd_phy.h"
#include "swd_dp.h"

uint8_t SWD_LastAck(void)
{
    return SWD_PhyLastAck();
}

/* 裸事务：不做任何重试（供清 sticky 等内部路径使用，避免递归） */
static int dp_xfer_raw(uint8_t apndp, uint8_t rnw, uint8_t addr, uint32_t *value)
{
    return SWD_Transfer(apndp, rnw, addr, value);
}

/* 带重试的事务：WAIT = 目标忙，重试；FAULT = 先清 sticky 再试一次 */
static int dp_xfer(uint8_t apndp, uint8_t rnw, uint8_t addr, uint32_t *value)
{
    uint32_t tries;
    int      r;

    for (tries = 0u; ; tries++) {
        r = SWD_Transfer(apndp, rnw, addr, value);
        if (r == SWD_OK) {
            return SWD_OK;
        }
        if (r == SWD_ERR_WAIT && tries < SWD_RETRY_WAIT) {
            continue;
        }
        if (r == SWD_ERR_FAULT && tries < SWD_RETRY_FAULT) {
            (void)SWD_ClearSticky();
            continue;
        }
        return r;
    }
}

int SWD_ClearSticky(void)
{
    /* SW-DP 上用 ABORT 清 sticky（ADIv5：SW-DP 的 CTRL/STAT sticky 位是 RO/WI）。
     * 用"写 CTRL/STAT"的做法既清不掉 sticky，又会把 CDBGPWRUPREQ 撤掉。 */
    uint32_t v = (uint32_t)SWD_ABORT_CLEAR_ALL;
    return dp_xfer_raw(0u, 0u, SWD_DP_ABORT, &v);
}

int SWD_SelectAP(uint8_t apsel, uint8_t bank)
{
    uint32_t sel = ((uint32_t)apsel << 24) | ((uint32_t)(bank & 0x0Fu) << 4);
    return dp_xfer(0u, 0u, SWD_DP_SELECT, &sel);
}

int SWD_DPRead(uint8_t addr, uint32_t *value)
{
    return dp_xfer(0u, 1u, addr, value);
}

int SWD_DPWrite(uint8_t addr, uint32_t value)
{
    uint32_t v = value;
    return dp_xfer(0u, 0u, addr, &v);
}

int SWD_APRead(uint8_t addr, uint32_t *value)
{
    return dp_xfer(1u, 1u, addr, value);
}

int SWD_APWrite(uint8_t addr, uint32_t value)
{
    uint32_t v = value;
    return dp_xfer(1u, 0u, addr, &v);
}

int SWD_Connect(uint32_t *idcode)
{
    uint32_t id;
    uint32_t i;
    int      r;

    SWD_PhyInit();

    for (i = 0u; i < SWD_CONNECT_RETRY; i++) {
        /* 1) 线复位：≥50 个周期 SWDIO 高 */
        SWD_LineReset();

        /* 2) JTAG → SWD 切换序列：16 位 0xE79E，LSB first
         *    （对应 OpenOCD 发送的 0x9E,0xE7 两字节，同样 LSB first） */
        SWD_WriteBits(0xE79Eu, 16u);

        /* 3) 再线复位 */
        SWD_LineReset();

        /* 4) 一个空闲周期：把线拉低，使包头 Start 位成为确定的上升沿 */
        SWD_IdleClock();

        /* 5) 读 IDCODE */
        id = 0u;
        r  = dp_xfer_raw(0u, 1u, SWD_DP_IDCODE, &id);
        if (r == SWD_OK && id != 0u && id != 0xFFFFFFFFu) {
            if (idcode != 0) {
                *idcode = id;
            }
            return SWD_OK;
        }
    }
    return SWD_ERR_NO_ACK;
}

int SWD_IsAlive(void)
{
    uint32_t id = 0u;
    int      r  = dp_xfer_raw(0u, 1u, SWD_DP_IDCODE, &id);
    if (r != SWD_OK || id == 0u || id == 0xFFFFFFFFu) {
        return SWD_ERR_NO_ACK;
    }
    return SWD_OK;
}

int SWD_PowerUp(void)
{
    uint32_t req  = (uint32_t)(SWD_CTRLSTAT_CDBGPWRUPREQ | SWD_CTRLSTAT_CSYSPWRUPREQ);
    uint32_t stat = 0u;
    uint32_t i;
    int      r;

    r = dp_xfer(0u, 0u, SWD_DP_CTRLSTAT, &req);
    if (r != SWD_OK) {
        return r;
    }

    for (i = 0u; i < SWD_RETRY_WAIT; i++) {
        r = dp_xfer(0u, 1u, SWD_DP_CTRLSTAT, &stat);
        if (r != SWD_OK) {
            return r;
        }
        if ((stat & (SWD_CTRLSTAT_CDBGPWRUPACK | SWD_CTRLSTAT_CSYSPWRUPACK)) ==
            (SWD_CTRLSTAT_CDBGPWRUPACK | SWD_CTRLSTAT_CSYSPWRUPACK)) {
            return SWD_OK;
        }
    }
    return SWD_ERR_TIMEOUT;
}
