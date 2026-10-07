/*
 * SWD 烧录引擎 spike —— 纯仿真验证
 *
 * 目的：在没有任何硬件的情况下回答一个问题——
 *   "U1 用 GPIO 位翻转做 SWD 主机，能不能读出 IDCODE、把 F405 的 Flash 擦干净、
 *    把镜像写进去并回读校验？"
 *
 * 被测代码是将来直接搬进固件的那几个模块（swd_phy/swd_dp/swd_memap/
 * cortex_m/stm32f4_flash），只把最底层 5 个 GPIO 原语换成 HAL 替身，
 * 由 sim_target 在线级解码真实的波形。
 *
 * 输出全部使用 ASCII，避免 Windows 控制台代码页问题。
 */
#include <stdio.h>
#include <string.h>

#include "sim_hal.h"
#include "sim_target.h"
#include "swd_config.h"
#include "swd_phy.h"
#include "swd_dp.h"
#include "swd_memap.h"
#include "cortex_m.h"
#include "stm32f4_flash.h"

/* ------------------------------------------------------------------ */
/* 断言框架                                                            */
/* ------------------------------------------------------------------ */
static int g_pass = 0;
static int g_fail = 0;

static void section(const char *name)
{
    printf("\n--- %s ---\n", name);
    fflush(stdout);
}

static void check(int cond, const char *name)
{
    if (cond) {
        g_pass++;
        printf("  [ok]   %s\n", name);
    } else {
        g_fail++;
        printf("  [FAIL] %s\n", name);
    }
}

static void check_eq_u32(uint32_t got, uint32_t want, const char *name)
{
    if (got == want) {
        g_pass++;
        printf("  [ok]   %s (= 0x%08lX)\n", name, (unsigned long)got);
    } else {
        g_fail++;
        printf("  [FAIL] %s: got 0x%08lX want 0x%08lX\n",
               name, (unsigned long)got, (unsigned long)want);
    }
}

static const char *status_name(int r)
{
    switch (r) {
    case SWD_OK:             return "OK";
    case SWD_ERR_NO_ACK:     return "NO_ACK";
    case SWD_ERR_WAIT:       return "WAIT";
    case SWD_ERR_FAULT:      return "FAULT";
    case SWD_ERR_PARITY:     return "PARITY";
    case SWD_ERR_PARAM:      return "PARAM";
    case SWD_ERR_TIMEOUT:    return "TIMEOUT";
    case SWD_ERR_FLASH:      return "FLASH_ERR";
    case SWD_ERR_NOT_HALTED: return "NOT_HALTED";
    case SWD_ERR_VERIFY:     return "VERIFY_MISMATCH";
    default:                 return "?";
    }
}

static void check_status(int r, int want, const char *name)
{
    if (r == want) {
        g_pass++;
        printf("  [ok]   %s -> %s\n", name, status_name(r));
    } else {
        g_fail++;
        printf("  [FAIL] %s: got %s(%d) want %s\n",
               name, status_name(r), r, status_name(want));
    }
}

/* ------------------------------------------------------------------ */
/* 测试用镜像                                                          */
/* ------------------------------------------------------------------ */
#define BOOT_BASE   0x08000000u
#define BOOT_LEN    512u
#define APP_BASE    0x08020000u        /* sector 5（128KB 扇区） */
#define APP_LEN     254u               /* 故意不是 4 的倍数，测尾部补齐 */

static uint8_t g_boot[BOOT_LEN];
static uint8_t g_app[APP_LEN];

static void make_image(uint8_t *buf, uint32_t len, uint32_t seed)
{
    uint32_t i;
    uint32_t x = seed;

    for (i = 0u; i < len; i++) {
        x = x * 1103515245u + 12345u;
        buf[i] = (uint8_t)((x >> 16) & 0xFFu);
    }
}

static void make_boot_image(void)
{
    make_image(g_boot, BOOT_LEN, 0x1234u);
    /* 向量表：初始栈顶 + 复位向量（复位后要能核对 PC/SP） */
    g_boot[0] = 0x00; g_boot[1] = 0x10; g_boot[2] = 0x00; g_boot[3] = 0x20;  /* SP = 0x20001000 */
    g_boot[4] = 0x01; g_boot[5] = 0x01; g_boot[6] = 0x00; g_boot[7] = 0x08;  /* PC = 0x08000101 */
}

/* ------------------------------------------------------------------ */
int main(void)
{
    uint32_t idcode = 0u;
    uint32_t clk_before;
    uint32_t clk_burn;
    uint32_t tx_before;
    uint32_t tx_burn;
    uint16_t dev_id = 0u;
    uint32_t v = 0u;
    uint32_t i;
    uint8_t  halted = 0xFFu;
    uint32_t blocksum = 0u;
    uint32_t singlesum = 0u;
    int      r;

    setvbuf(stdout, NULL, _IONBF, 0);   /* 无缓冲：卡住时也能看到进度 */

    printf("============================================================\n");
    printf(" SWD programming engine spike (pure simulation)\n");
    printf(" target model: STM32F405RGT6 / SW-DP idcode 0x2BA01477\n");
    printf("============================================================\n");
    fflush(stdout);

    make_boot_image();
    make_image(g_app, APP_LEN, 0xBEEFu);

    /* ============================ 1. 连接 ============================ */
    section("1. connect (line reset + 0xE79E switch + IDCODE)");
    sim_target_reset();
    sim_target_set_read_ta(SWD_TA_BEFORE_READ_DATA);

    r = SWD_Connect(&idcode);
    check_status(r, SWD_OK, "SWD_Connect");
    check_eq_u32(idcode, 0x2BA01477u, "DP IDCODE");
    check_eq_u32(sim_target_switch_seen(), 1u, "JTAG->SWD switch sequence recognised");
    check_eq_u32(sim_target_line_resets(), 2u, "line resets seen by target");
    check_eq_u32(sim_target_contention(), 0u, "no SWDIO drive contention");

    check_status(SWD_IsAlive(), SWD_OK, "SWD_IsAlive (second probe, no reset)");
    check_status(SWD_PowerUp(), SWD_OK, "SWD_PowerUp (CDBG/CSYS power-up ACK)");
    check_status(SWD_MemInit(), SWD_OK, "SWD_MemInit (SELECT AP0 + CSW)");

    /* ============================ 2. 内核 ============================ */
    section("2. core control (halt / identify / freeze watchdog)");
    check_status(CM_Halt(), SWD_OK, "CM_Halt");
    check_status(CM_IsHalted(&halted), SWD_OK, "CM_IsHalted read");
    check(halted == 1u, "core reports S_HALT=1");
    check(sim_target_halted() == 1u, "target core is actually halted");
    check_status(CM_ReadDevId(&dev_id), SWD_OK, "CM_ReadDevId");
    check(dev_id == 0x0413u, "DBGMCU DEV_ID = 0x413 (STM32F405/407)");
    check_status(CM_FreezeWatchdogs(), SWD_OK, "CM_FreezeWatchdogs");
    check_eq_u32(sim_target_iwdg_frozen(), 1u, "DBGMCU_APB1_FZ: IWDG stopped in debug");

    /* ==================== 3. MEM-AP 基本读写 ==================== */
    section("3. MEM-AP access + posted-read correctness");
    check_status(SWD_MemRead32(0x1FFF7A10u, &v), SWD_OK, "read target UID word");
    check(v != 0u && v != 0xFFFFFFFFu, "UID word is sane (posted read not stale)");
    check_status(SWD_MemRead32(0xE0042000u, &v), SWD_OK, "read DBGMCU_IDCODE via MEM-AP");
    check_eq_u32(v & 0x0FFFu, 0x0413u, "DEV_ID through generic memory path");
    check_status(SWD_MemRead32(0x08000000u, &v), SWD_OK, "read erased flash word");
    check_eq_u32(v, 0xFFFFFFFFu, "flash is erased (0xFFFFFFFF)");

    /* ======================== 4. 烧录 boot ======================== */
    section("4. burn: erase + program + verify (boot image, 512B @ 0x08000000)");
    clk_before = SWD_PhyClockCount();
    tx_before  = sim_target_transactions();

    check_status(F4_EraseRange(BOOT_BASE, BOOT_LEN), SWD_OK, "F4_EraseRange sector0");
    check_eq_u32(sim_target_erases(), 1u, "one sector erase happened");
    check_status(F4_ProgramImage(BOOT_BASE, g_boot, BOOT_LEN), SWD_OK, "F4_ProgramImage");
    check_status(F4_VerifyImage(BOOT_BASE, g_boot, BOOT_LEN), SWD_OK, "F4_VerifyImage");
    check(memcmp(sim_target_flash(), g_boot, BOOT_LEN) == 0,
          "target flash content == image byte for byte");
    check_eq_u32(sim_target_programs(), BOOT_LEN / 4u, "word program count");

    clk_burn = SWD_PhyClockCount() - clk_before;
    tx_burn  = sim_target_transactions() - tx_before;

    /* ======================== 5. 尾部补齐 ======================== */
    section("5. burn: non-word-aligned tail (254B @ 0x08020000, sector5)");
    check_status(F4_EraseRange(APP_BASE, 1u), SWD_OK, "erase sector5 only");
    check_status(F4_ProgramImage(APP_BASE, g_app, APP_LEN), SWD_OK, "F4_ProgramImage (tail)");
    check_status(F4_VerifyImage(APP_BASE, g_app, APP_LEN), SWD_OK, "F4_VerifyImage (tail)");
    check(memcmp(sim_target_flash() + (APP_BASE - BOOT_BASE), g_app, APP_LEN) == 0,
          "tail bytes match");
    check(sim_target_flash()[(APP_BASE - BOOT_BASE) + APP_LEN] == 0xFFu,
          "byte 254 padded with 0xFF");
    check(sim_target_flash()[(APP_BASE - BOOT_BASE) + APP_LEN + 1u] == 0xFFu,
          "byte 255 padded with 0xFF");
    check(sim_target_flash()[(APP_BASE - BOOT_BASE) + 256u] == 0xFFu,
          "next word untouched");
    check(sim_target_flash()[(BOOT_BASE - BOOT_BASE) + 128u] != 0xFFu,
          "sector0 still holds boot image (no cross-sector damage)");

    /* ============ 6. 两种读路径一致性（单次读 vs 块读） ============ */
    section("6. read path cross-check: single word vs auto-increment block");
    for (i = 0u; i < 32u; i++) {
        uint32_t w = 0u;
        r = SWD_MemRead32(BOOT_BASE + i * 4u, &w);
        if (r != SWD_OK) { g_fail++; printf("  [FAIL] single read %lu\n", (unsigned long)i); }
        singlesum = (singlesum * 31u) + w;
    }
    {
        uint32_t buf[32];
        uint32_t j;
        r = SWD_MemReadBlock(BOOT_BASE, buf, 32u);
        check_status(r, SWD_OK, "SWD_MemReadBlock x32");
        for (j = 0u; j < 32u; j++) {
            blocksum = (blocksum * 31u) + buf[j];
        }
        check(singlesum == blocksum, "block read == 32 single reads");
        check(buf[0] == 0x20001000u, "block read[0] == SP from vector table");
        check(buf[1] == 0x08000101u, "block read[1] == reset vector");
    }

    /* ======================== 7. 复位运行 ======================== */
    section("7. reset and run");
    check_status(CM_ResetAndRun(), SWD_OK, "CM_ResetAndRun (AIRCR SYSRESETREQ)");
    check(sim_target_halted() == 0u, "core is running after reset");
    check_eq_u32(sim_target_pc(), 0x08000101u, "PC loaded from reset vector");
    check_eq_u32(sim_target_sp(), 0x20001000u, "SP loaded from vector table");

    /* ======================== 8. 负向用例 ======================== */
    section("8. negative cases");

    /* 8a. 目标无应答 */
    sim_target_attach(0u);
    r = SWD_Connect(&idcode);
    check_status(r, SWD_ERR_NO_ACK, "connect with target detached");
    check(1, "no hang on missing target");
    sim_target_attach(1u);
    check_status(SWD_Connect(&idcode), SWD_OK, "reconnect after re-attach");

    /* 8b. WAIT 重试 */
    sim_target_set_wait(4u);
    r = SWD_MemRead32(0x08000000u, &v);
    check_status(r, SWD_OK, "read succeeds across 4 injected WAITs");
    check_eq_u32(sim_target_wait_sent(), 4u, "target really sent 4 WAITs");

    /* 8c. FAULT + sticky 清除后重试 */
    sim_target_set_fault(1u);
    r = SWD_MemRead32(0x08000000u, &v);
    check_status(r, SWD_OK, "read recovers after 1 injected FAULT");
    check_eq_u32(sim_target_fault_sent(), 1u, "target really sent 1 FAULT");

    /* 8d. 未擦除就重复编程 -> PGAERR */
    r = F4_ProgramImage(BOOT_BASE, g_boot, BOOT_LEN);
    check_status(r, SWD_ERR_FLASH, "re-program without erase is rejected (PGAERR)");
    check(sim_target_flash_err() > 0u, "target reported a flash error flag");

    /* 8e. 回读校验能抓出被改动的字节 */
    sim_target_flash_corrupt(BOOT_BASE + 100u, 0x5Au);
    r = F4_VerifyImage(BOOT_BASE, g_boot, BOOT_LEN);
    check_status(r, SWD_ERR_VERIFY, "verify detects corrupted byte");
    sim_target_flash_corrupt(BOOT_BASE + 100u, g_boot[100]);

    /* 8f. BSY 永不落 -> 超时而不是死循环 */
    sim_target_set_stuck_busy(1u);
    r = F4_EraseRange(APP_BASE, 1u);
    check_status(r, SWD_ERR_TIMEOUT, "erase times out when BSY never clears");
    sim_target_set_stuck_busy(0u);

    /* ======================== 9. 统计与推算 ======================== */
    section("9. counters (simulation)");
    printf("  transactions        : %lu\n", (unsigned long)sim_target_transactions());
    printf("  SWCLK cycles total  : %lu\n", (unsigned long)SWD_PhyClockCount());
    printf("  burn 512B: clocks   : %lu  transactions: %lu\n",
           (unsigned long)clk_burn, (unsigned long)tx_burn);
    printf("  contention          : %lu (must be 0)\n", (unsigned long)sim_target_contention());
    printf("  parity errors       : %lu (must be 0)\n", (unsigned long)sim_target_parity_errors());
    printf("  host TA in data ph. : %lu\n", (unsigned long)sim_target_ta_skipped());
    printf("  locked-flash reject : %lu\n", (unsigned long)sim_target_locked_reject());
    printf("  bad psize           : %lu\n", (unsigned long)sim_target_bad_psize());
    printf("  unmapped accesses   : %lu\n", (unsigned long)sim_target_unmapped());
    printf("  rdbuff reads        : %lu\n", (unsigned long)sim_target_rdbuff_reads());
    printf("  sector erases       : %lu\n", (unsigned long)sim_target_erases());
    printf("  word programs       : %lu\n", (unsigned long)sim_target_programs());

    check_eq_u32(sim_target_contention(), 0u, "zero contention overall");
    check_eq_u32(sim_target_parity_errors(), 0u, "zero line-level parity errors");
    check_eq_u32(sim_target_bad_psize(), 0u, "never programmed with wrong PSIZE");
    check_eq_u32(sim_target_locked_reject(), 0u, "never hit locked-flash rejection");

    section("10. timing projection (from measured cycle counts)");
    {
        uint32_t per_kb = (uint32_t)(((uint64_t)clk_burn * 1024u) / (uint64_t)BOOT_LEN);
        double   cyc_mb = (double)per_kb * 1024.0;
        double   cyc_300 = (double)per_kb * 300.0;

        printf("  512B burn (erase+program+verify): %lu SWCLK cycles / %lu transactions\n",
               (unsigned long)clk_burn, (unsigned long)tx_burn);
        printf("  measured            : %lu SWCLK cycles per KB\n", (unsigned long)per_kb);
        printf("  full 1MB  @2MHz     : %.1f s      @4MHz : %.1f s\n",
               cyc_mb / 2.0e6, cyc_mb / 4.0e6);
        printf("  300KB app @2MHz     : %.1f s      @4MHz : %.1f s\n",
               cyc_300 / 2.0e6, cyc_300 / 4.0e6);
        printf("  note: this code is the unoptimised stage-1 path\n");
        printf("        (TAR rewritten per access, per-word flag clear, 3-transaction SR poll).\n");
        printf("        stage-2 (block writes + 1-transaction poll) should cut this ~3x.\n");
    }

    /* ============ 11. 规范语义自检（这次就是靠它抓出两个 bug） ============ */
    section("11. spec semantics pins (register bits + CSW.AddrInc)");

    /* 11a. 关键寄存器位必须与手册一致（改错任何一位都在这里爆） */
    {
        /* FLASH_SR：RM0090 §3.9.6 */
        check_eq_u32(F4_SR_EOP,    0x00000001u, "FLASH_SR.EOP   = bit0");
        check_eq_u32(F4_SR_OPERR,  0x00000002u, "FLASH_SR.OPERR = bit1");
        check_eq_u32(F4_SR_WRPERR, 0x00000010u, "FLASH_SR.WRPERR= bit4");
        check_eq_u32(F4_SR_PGAERR, 0x00000020u, "FLASH_SR.PGAERR= bit5");
        check_eq_u32(F4_SR_PGPERR, 0x00000040u, "FLASH_SR.PGPERR= bit6");
        check_eq_u32(F4_SR_PGSERR, 0x00000080u, "FLASH_SR.PGSERR= bit7");
        check_eq_u32(F4_SR_BSY,    0x00010000u, "FLASH_SR.BSY   = bit16");
        /* FLASH_CR：RM0090 §3.9.7（F40x/F41x 单 bank） */
        check_eq_u32(F4_CR_PG,     0x00000001u, "FLASH_CR.PG    = bit0");
        check_eq_u32(F4_CR_SER,    0x00000002u, "FLASH_CR.SER   = bit1");
        check_eq_u32(F4_CR_MER,    0x00000004u, "FLASH_CR.MER   = bit2");
        check_eq_u32(F4_CR_SNB_MASK, 0x00000078u, "FLASH_CR.SNB   = bits6:3");
        check_eq_u32(F4_CR_PSIZE_X32, 0x00000200u, "FLASH_CR.PSIZE=x32 (bits9:8)");
        check_eq_u32(F4_CR_STRT,   0x00010000u, "FLASH_CR.STRT  = bit16 <<< 第一版写成 bit15");
        check_eq_u32(F4_CR_LOCK,   0x80000000u, "FLASH_CR.LOCK  = bit31");
        /* 钥匙：RM0090 §3.9.2 */
        check_eq_u32(F4_FLASH_KEY1, 0x45670123u, "FLASH_KEYR KEY1");
        check_eq_u32(F4_FLASH_KEY2, 0xCDEF89ABu, "FLASH_KEYR KEY2");
        /* DP/AP：ADIv5 §B4.2 */
        check_eq_u32(SWD_ABORT_CLEAR_ALL, 0x0000001Eu, "DP ABORT 清 sticky = 0x1E (bits1..4)");
        check_eq_u32(SWD_CTRLSTAT_CDBGPWRUPREQ, 0x10000000u, "CTRL/STAT CDBGPWRUPREQ = bit28");
        check_eq_u32(SWD_CTRLSTAT_CSYSPWRUPACK, 0x80000000u, "CTRL/STAT CSYSPWRUPACK = bit31");
        check_eq_u32(AHBAP_CSW_WORD, 0x23000052u, "CSW = 0x23000052 (AddrInc=0b01 自增)");
        /* Cortex-M 调试：PM0214 / DAPLink 用法 */
        check_eq_u32(CM_DHCSR_C_HALT, 0x00000002u, "DHCSR.C_HALT = bit1");
        check_eq_u32(CM_DHCSR_KEY, 0xA05F0000u, "DHCSR DBGKEY = 0xA05F<<16");
        check_eq_u32(CM_DHCSR_S_HALT, 0x00020000u, "DHCSR.S_HALT = bit17");
        check_eq_u32(CM_AIRCR_SYSRESETREQ, 0x00000004u, "AIRCR.SYSRESETREQ = bit2");
    }

    /* 11b. CSW.AddrInc 语义：0b01 才是"每次访问后自增"，0b00 是"不自增" */
    {
        uint32_t a0 = 0u, a1 = 0u, base = BOOT_BASE + 0x40u;
        uint32_t w0 = 0u, w1 = 0u;
        uint32_t dummy = 0u;

        /* 先读出相邻两个字作为基准 */
        (void)SWD_MemRead32(base, &w0);
        (void)SWD_MemRead32(base + 4u, &w1);
        check(w0 != w1, "two adjacent boot words differ (test needs distinct data)");

        /* AddrInc=0b01（increment single）：TAR 写一次，连续 DRW 读应拿到递增的地址 */
        (void)SWD_APWrite(SWD_AP_CSW, AHBAP_CSW_WORD);
        (void)SWD_APWrite(SWD_AP_TAR, base);
        (void)SWD_APRead(SWD_AP_DRW, &dummy);       /* posted 滞后一拍 */
        (void)SWD_APRead(SWD_AP_DRW, &a0);          /* = mem[base] */
        (void)SWD_APRead(SWD_AP_DRW, &a1);          /* = mem[base+4] */
        check(a0 == w0 && a1 == w1, "AddrInc=0b01: consecutive DRW reads advance");

        /* AddrInc=0b00（off）：TAR 不增，连续读应拿到同一个字 */
        (void)SWD_APWrite(SWD_AP_CSW, AHBAP_CSW_WORD_NOINC);
        (void)SWD_APWrite(SWD_AP_TAR, base);
        (void)SWD_APRead(SWD_AP_DRW, &dummy);
        (void)SWD_APRead(SWD_AP_DRW, &a0);
        (void)SWD_APRead(SWD_AP_DRW, &a1);
        check(a0 == w0 && a1 == w0, "AddrInc=0b00: consecutive DRW reads stay at TAR");

        /* 收尾：把 CSW 恢复成正常值，避免影响后续用例 */
        (void)SWD_MemInit();
    }

    printf("\n============================================================\n");
    printf(" RESULT: %d passed, %d failed\n", g_pass, g_fail);
    printf("============================================================\n");
    return (g_fail == 0) ? 0 : 1;
}
