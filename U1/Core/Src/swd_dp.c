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

/* 带重试的事务：WAIT = 目标忙，重试；FAULT = 先清 sticky 再试一次；
 * NO_ACK（ACK 位被噪声打坏，例如读到 0b101 这种非法值）= 退避后重发这笔。
 * 2026-11-03 实测：DRW 写之后的 DP 读偶发畸形 ACK，加这条后即可自愈。 */
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
        if (r == SWD_ERR_NO_ACK && tries < SWD_RETRY_NOACK) {
            continue;                   /* SWD_Transfer 内部已做退避对齐 */
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
        uint8_t k;

        /* 以下四步**完全照搬 DAPProg 的连接序列**（SWD_host.c:702-729）：
         *   ① SWJ_Sequence(51, 0xFF...)   线复位：51 拍 SWDIO=1
         *   ② SWJ_Sequence(16, {0x9E,0xE7}) JTAG → SWD 切换序列（LSB first）
         *   ③ SWJ_Sequence(8,  {0x00})     ★ 8 拍 SWDIO=0（**不再做第二次线复位**）
         *   ④ 直接读 IDCODE
         * 我们原来在 ② 之后又做了一次 56 拍线复位 + 1 拍空闲，
         * 与参考实现不一致 —— 2026-11-02 按参考实现改齐。 */

        /* ① 线复位：≥50 个周期 SWDIO 高 */
        SWD_LineReset();

        /* ② JTAG → SWD 切换序列：16 位 0xE79E，LSB first */
        SWD_WriteBits(0xE79Eu, 16u);

        /* ③ 8 拍 SWDIO=0（对应参考实现的 0x00 序列，替代原来的第二次线复位） */
        for (k = 0u; k < 8u; k++) {
            SWD_WriteBit(0u);
        }

        /* ④ 读 IDCODE */
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
            /* ★ 2026-11-03：上电成功后**立刻清一次 sticky**（写 DP ABORT）。
             * 参考实现 DAPProg 也是在连接后马上写 ABORT（SWD_host.c:781）。
             * 如果不清，第一笔"上电刚过、AP 还没就绪"的 AP 访问一旦 FAULT，
             * STICKYERR 会一直锁着 → 之后**所有 AP 访问都返回 FAULT**，
             * 而 DP 访问完全正常（实测现象：AP IDR 读、AP CSW 读全部失败）。 */
            (void)SWD_ClearSticky();
            return SWD_OK;
        }
    }
    return SWD_ERR_TIMEOUT;
}
