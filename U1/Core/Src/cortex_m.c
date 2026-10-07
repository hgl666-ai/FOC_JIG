/*
 * Cortex-M 内核调试控制实现
 */
#include "swd_config.h"
#include "swd_phy.h"
#include "swd_dp.h"
#include "swd_memap.h"
#include "cortex_m.h"

int CM_IsHalted(uint8_t *halted)
{
    uint32_t v = 0u;
    int      r = SWD_MemRead32(CM_DHCSR, &v);

    if (r != SWD_OK) {
        return r;
    }
    if (halted != 0) {
        *halted = ((v & CM_DHCSR_S_HALT) != 0u) ? 1u : 0u;
    }
    return SWD_OK;
}

int CM_Halt(void)
{
    uint32_t v = CM_DHCSR_KEY | CM_DHCSR_C_DEBUGEN | CM_DHCSR_C_HALT;
    uint8_t  halted = 0u;
    uint32_t i;
    int      r;

    r = SWD_MemWrite32(CM_DHCSR, v);
    if (r != SWD_OK) {
        return r;
    }
    for (i = 0u; i < 100u; i++) {
        r = CM_IsHalted(&halted);
        if (r != SWD_OK) {
            return r;
        }
        if (halted != 0u) {
            return SWD_OK;
        }
    }
    return SWD_ERR_NOT_HALTED;
}

int CM_Run(void)
{
    uint32_t v = CM_DHCSR_KEY | CM_DHCSR_C_DEBUGEN;   /* 清 C_HALT */
    return SWD_MemWrite32(CM_DHCSR, v);
}

int CM_ResetAndRun(void)
{
    int r;

    r = CM_Run();
    if (r != SWD_OK) {
        return r;
    }
    r = SWD_MemWrite32(CM_AIRCR, CM_AIRCR_VECTKEY | CM_AIRCR_SYSRESETREQ);
    return r;
}

int CM_FreezeWatchdogs(void)
{
    uint32_t v = 0u;
    int      r;

    r = SWD_MemRead32(CM_DBGMCU_APB1_FZ, &v);
    if (r != SWD_OK) {
        return r;
    }
    v |= (CM_DBGMCU_APB1_FZ_IWDG_STOP | CM_DBGMCU_APB1_FZ_WWDG_STOP);
    return SWD_MemWrite32(CM_DBGMCU_APB1_FZ, v);
}

int CM_ReadDevId(uint16_t *dev_id)
{
    uint32_t v = 0u;
    int      r = SWD_MemRead32(CM_DBGMCU_IDCODE, &v);

    if (r != SWD_OK) {
        return r;
    }
    if (dev_id != 0) {
        *dev_id = (uint16_t)(v & 0x0FFFu);
    }
    return SWD_OK;
}

int CM_ReadReg(uint8_t reg, uint32_t *value)
{
    uint32_t v = 0u;
    uint32_t i;
    int      r;

    r = SWD_MemWrite32(CM_DCRSR, (uint32_t)reg);        /* REGSEL=0：读 */
    if (r != SWD_OK) {
        return r;
    }
    for (i = 0u; i < 100u; i++) {
        r = SWD_MemRead32(CM_DHCSR, &v);
        if (r != SWD_OK) {
            return r;
        }
        if ((v & CM_DHCSR_S_REGRDY) != 0u) {
            break;
        }
    }
    if ((v & CM_DHCSR_S_REGRDY) == 0u) {
        return SWD_ERR_TIMEOUT;
    }
    return SWD_MemRead32(CM_DCRDR, value);
}

int CM_WriteReg(uint8_t reg, uint32_t value)
{
    uint32_t v = 0u;
    uint32_t i;
    int      r;

    r = SWD_MemWrite32(CM_DCRDR, value);
    if (r != SWD_OK) {
        return r;
    }
    r = SWD_MemWrite32(CM_DCRSR, 0x00010000u | (uint32_t)reg);   /* REGSEL=1：写 */
    if (r != SWD_OK) {
        return r;
    }
    for (i = 0u; i < 100u; i++) {
        r = SWD_MemRead32(CM_DHCSR, &v);
        if (r != SWD_OK) {
            return r;
        }
        if ((v & CM_DHCSR_S_REGRDY) != 0u) {
            break;
        }
    }
    if ((v & CM_DHCSR_S_REGRDY) == 0u) {
        return SWD_ERR_TIMEOUT;
    }
    return SWD_OK;
}
