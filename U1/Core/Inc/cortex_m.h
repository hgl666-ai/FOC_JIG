/*
 * Cortex-M 内核调试控制（通过 MEM-AP 读写调试寄存器）
 *
 * 覆盖烧录流程真正需要的四件事：
 *   1. 停住内核并确认停住了（DHCSR + S_HALT）——不停住就写不了 Flash
 *   2. 冻住看门狗（DBGMCU），否则擦写期间可能被 IWDG 复位
 *   3. 识别目标（DBGMCU_IDCODE，F405 的 DEV_ID = 0x413）
 *   4. 复位并运行（AIRCR SYSRESETREQ）
 */
#ifndef CORTEX_M_H
#define CORTEX_M_H

#include <stdint.h>

#define CM_DHCSR        0xE000EDF0u
#define CM_DCRSR        0xE000EDF4u
#define CM_DCRDR        0xE000EDF8u
#define CM_DEMCR        0xE000EDFCu
#define CM_AIRCR        0xE000ED0Cu

#define CM_DHCSR_KEY     0xA05F0000u
#define CM_DHCSR_C_DEBUGEN 0x00000001u
#define CM_DHCSR_C_HALT    0x00000002u
#define CM_DHCSR_S_REGRDY  0x00010000u
#define CM_DHCSR_S_HALT    0x00020000u
#define CM_DHCSR_S_RESET_ST 0x02000000u

#define CM_DEMCR_VC_CORERESET 0x00000001u
#define CM_AIRCR_VECTKEY      0x05FA0000u
#define CM_AIRCR_SYSRESETREQ  0x00000004u

#define CM_DBGMCU_IDCODE    0xE0042000u
#define CM_DBGMCU_CR        0xE0042004u
#define CM_DBGMCU_APB1_FZ   0xE0042008u
#define CM_DBGMCU_APB1_FZ_IWDG_STOP  (1u << 12)
#define CM_DBGMCU_APB1_FZ_WWDG_STOP  (1u << 11)

#define CM_DEV_ID_STM32F405  0x413u

/** 内核寄存器编号（DCRSR REGSEL） */
#define CM_REG_R0   0u
#define CM_REG_PC   15u
#define CM_REG_XPSR 16u
#define CM_REG_MSP  17u
#define CM_REG_PSP  18u

int CM_Halt(void);
int CM_Run(void);
int CM_IsHalted(uint8_t *halted);
int CM_ResetAndRun(void);
int CM_FreezeWatchdogs(void);
int CM_ReadDevId(uint16_t *dev_id);

int CM_ReadReg(uint8_t reg, uint32_t *value);
int CM_WriteReg(uint8_t reg, uint32_t value);

#endif /* CORTEX_M_H */
