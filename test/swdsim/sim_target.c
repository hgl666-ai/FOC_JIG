/*
 * 仿真目标模型实现（线级 SW-DP + AHB-AP + STM32F405）
 */
#include <string.h>
#include <stdio.h>
#include <stdlib.h>

#include "sim_hal.h"
#include "swd_config.h"
#include "sim_target.h"

/* ================================================================== */
/* 目标侧常量                                                          */
/* 说明：这里**故意**不引用主机侧的 stm32f4_flash.h / swd_dp.h，       */
/*       避免"主机常量写错、模型跟着一起错"的共同错误。               */
/* ================================================================== */
#define T_FLASH_BASE      0x08000000u
#define T_FLASH_SIZE      (1024u * 1024u)

#define T_FLASH_ACR       0x40023C00u
#define T_FLASH_KEYR      0x40023C04u
#define T_FLASH_OPTKEYR   0x40023C08u
#define T_FLASH_SR        0x40023C0Cu
#define T_FLASH_CR        0x40023C10u
#define T_FLASH_OPTCR     0x40023C14u

#define T_KEY1            0x45670123u
#define T_KEY2            0xCDEF89ABu

#define T_SR_EOP          (1u << 0)
#define T_SR_OPERR        (1u << 1)
#define T_SR_WRPERR       (1u << 4)
#define T_SR_PGAERR       (1u << 5)
#define T_SR_PGPERR       (1u << 6)
#define T_SR_PGSERR       (1u << 7)
#define T_SR_RDERR        (1u << 8)
#define T_SR_BSY          (1u << 16)
#define T_SR_ERRMASK      (T_SR_OPERR | T_SR_WRPERR | T_SR_PGAERR | \
                           T_SR_PGPERR | T_SR_PGSERR | T_SR_RDERR)

#define T_CR_PG           (1u << 0)
#define T_CR_SER          (1u << 1)
#define T_CR_MER          (1u << 2)
#define T_CR_SNB_POS      3u
#define T_CR_SNB_MASK     (0xFu << T_CR_SNB_POS)
#define T_CR_PSIZE_POS    8u
#define T_CR_PSIZE_MASK   (3u << T_CR_PSIZE_POS)
#define T_CR_PSIZE_X32    (2u << T_CR_PSIZE_POS)
#define T_CR_STRT         (1u << 16)     /* RM0090 §3.9.7：STRT 是 bit16（F40x/F41x） */
#define T_CR_LOCK         (1u << 31)

/* 扇区表：本文件独立再写一遍 */
static const uint32_t k_sec_off[8] = {
    0x00000u, 0x04000u, 0x08000u, 0x0C000u, 0x10000u, 0x20000u, 0x40000u, 0x60000u
};
static const uint32_t k_sec_kb[8] = { 16u, 16u, 16u, 16u, 64u, 128u, 128u, 128u };

#define T_DHCSR           0xE000EDF0u
#define T_DCRSR           0xE000EDF4u
#define T_DCRDR           0xE000EDF8u
#define T_DEMCR           0xE000EDFCu
#define T_AIRCR           0xE000ED0Cu

#define T_DBGMCU_IDCODE   0xE0042000u
#define T_DBGMCU_CR       0xE0042004u
#define T_DBGMCU_APB1FZ   0xE0042008u
#define T_DEV_ID          0x413u
#define T_REV_ID          0x1000u

#define T_UID_BASE        0x1FFF7A10u

/* DP / AP 寄存器地址（A[3:2]） */
#define T_DP_IDCODE       0x00u
#define T_DP_CTRLSTAT     0x04u
#define T_DP_SELECT       0x08u
#define T_DP_RDBUFF       0x0Cu
#define T_AP_CSW          0x00u
#define T_AP_TAR          0x04u
#define T_AP_DRW          0x0Cu

#define T_STICKY_MASK     0x1Eu          /* bits1..4：ORUN/ERR/CMP/EXTERN */
#define T_PWRUP_REQ       0x50000000u
#define T_PWRUP_ACK       0xA0000000u

#define T_SWITCH_SEQ      0xE79Eu

/* 编译期保证两边对 Flash 大小的认知一致 */
typedef char sim_assert_flash_size[(T_FLASH_SIZE == F4_FLASH_SIZE) ? 1 : -1];
typedef char sim_assert_flash_base[(T_FLASH_BASE == F4_FLASH_BASE) ? 1 : -1];

/* ================================================================== */
/* 状态                                                                */
/* ================================================================== */
typedef enum {
    PH_IDLE = 0,
    PH_ACK,
    PH_WR_DATA,
    PH_RD_DATA
} sim_phase_t;

static uint8_t  s_flash[T_FLASH_SIZE];
static uint8_t  s_progmap[T_FLASH_SIZE / 32u];    /* 每 bit 一个 4 字节字 */

static uint8_t  s_attached;
static uint8_t  s_clk;
static uint8_t  s_host_drives, s_host_val;
static uint8_t  s_tgt_drives, s_tgt_val;
static uint8_t  s_nrst_level;

/* 线级 / 位引擎 */
static sim_phase_t s_phase;
static uint8_t  s_prev_bit;               /* 上一个主机驱动位（用于 Start 位上升沿同步） */
static uint8_t  s_hdr;                    /* 正在收集的请求头，bit0=最早收到的位 */
static uint8_t  s_hdr_bits;               /* 0 = 未开始收集 */
static uint32_t s_high_run;
static uint8_t  s_pending_switch;
static uint8_t  s_switching, s_switched, s_switch_bits;
static uint32_t s_switch_val;

static uint8_t  s_pend_apndp, s_pend_rnw, s_pend_addr;
static uint8_t  s_ack;
static uint8_t  s_out[40];
static uint8_t  s_out_n, s_out_i, s_skip_falls;
static uint32_t s_in_data, s_in_parity;
static uint8_t  s_in_bits;
static uint8_t  s_read_ta;

/* 注入 */
static uint32_t s_wait_left, s_fault_left;
static uint8_t  s_stuck_busy;
static uint32_t s_idcode;
static uint32_t s_busy_polls;

/* DP / AP */
static uint32_t s_dp_ctrlstat, s_dp_select, s_ap_rdbuff;
static uint32_t s_ap_csw, s_ap_tar;
static uint8_t  s_stickyerr;

/* Flash 控制器 */
static uint32_t s_fcr, s_fsr, s_facr, s_foptcr;
static uint32_t s_key_state;
static uint32_t s_busy;
static uint32_t s_pend_erase;             /* 0xFFFFFFFF = 无 */
static uint32_t s_pend_mer;
static uint32_t s_pend_prog_addr;
static uint32_t s_pend_prog_val;
static uint8_t  s_pend_prog;

/* 内核与调试 */
static uint8_t  s_halted;
static uint32_t s_dhcsr_ctl;
static uint32_t s_dcrsr, s_dcrdr, s_demcr;
static uint32_t s_regs[21];
static uint32_t s_dbgmcu_cr, s_dbgmcu_apb1fz;

/* 统计 */
static uint32_t c_clocks, c_transactions, c_contention, c_parity_errors;
static uint32_t c_wait_sent, c_fault_sent, c_ta_skipped, c_locked_reject;
static uint32_t c_bad_psize, c_unmapped, c_switch_seen, c_line_resets;
static uint32_t c_erases, c_programs, c_rdbuff_reads, c_flash_err, c_nrst;

/* ================================================================== */
/* 小工具                                                              */
/* ================================================================== */
static uint8_t sim_parity(uint32_t v)
{
    v ^= v >> 16;
    v ^= v >> 8;
    v ^= v >> 4;
    v ^= v >> 2;
    v ^= v >> 1;
    return (uint8_t)(v & 1u);
}

static uint32_t rd32(const uint8_t *p)
{
    return (uint32_t)p[0] | ((uint32_t)p[1] << 8) |
           ((uint32_t)p[2] << 16) | ((uint32_t)p[3] << 24);
}

static void wr32(uint8_t *p, uint32_t v)
{
    p[0] = (uint8_t)(v & 0xFFu);
    p[1] = (uint8_t)((v >> 8) & 0xFFu);
    p[2] = (uint8_t)((v >> 16) & 0xFFu);
    p[3] = (uint8_t)((v >> 24) & 0xFFu);
}

static uint8_t progmap_get(uint32_t word_idx)
{
    return (uint8_t)((s_progmap[word_idx >> 3] >> (word_idx & 7u)) & 1u);
}

static void progmap_set(uint32_t word_idx)
{
    s_progmap[word_idx >> 3] |= (uint8_t)(1u << (word_idx & 7u));
}

/* ================================================================== */
/* Flash 数组与控制器                                                  */
/* ================================================================== */
static void flash_erase_bytes(uint32_t off, uint32_t len)
{
    uint32_t i;
    for (i = 0u; i < len; i++) {
        s_flash[off + i] = 0xFFu;
    }
    for (i = 0u; i < (len / 4u); i++) {
        s_progmap[(off / 4u + i) >> 3] &= (uint8_t)~(1u << ((off / 4u + i) & 7u));
    }
}

static void flash_do_erase(void)
{
    uint32_t s;

    if (s_pend_mer != 0u) {
        flash_erase_bytes(0u, T_FLASH_SIZE);
        s_pend_mer = 0u;
        c_erases++;
        return;
    }
    if (s_pend_erase != 0xFFFFFFFFu) {
        s = s_pend_erase;
        if (s < 8u) {
            flash_erase_bytes(k_sec_off[s], k_sec_kb[s] * 1024u);
            c_erases++;
        }
        s_pend_erase = 0xFFFFFFFFu;
    }
}

static void flash_do_program(void)
{
    uint32_t off = s_pend_prog_addr - T_FLASH_BASE;
    wr32(&s_flash[off], s_pend_prog_val);
    progmap_set(off / 4u);
    c_programs++;
    s_pend_prog = 0u;
}

/* 由 SR 读推动"操作完成" */
static void flash_tick(void)
{
    if (s_stuck_busy != 0u) {
        s_fsr |= T_SR_BSY;
        return;
    }
    if (s_busy == 0u) {
        return;
    }
    s_busy--;
    if (s_busy > 0u) {
        s_fsr |= T_SR_BSY;
        return;
    }
    s_fsr &= ~T_SR_BSY;
    if (s_pend_prog != 0u) {
        flash_do_program();
    }
    flash_do_erase();
    s_fsr |= T_SR_EOP;
}

static uint32_t flash_reg_read(uint32_t addr)
{
    switch (addr) {
    case T_FLASH_ACR:    return s_facr;
    case T_FLASH_KEYR:   return 0u;
    case T_FLASH_OPTKEYR:return 0u;
    case T_FLASH_SR:     flash_tick(); return s_fsr;
    case T_FLASH_CR:     return s_fcr;
    case T_FLASH_OPTCR:  return s_foptcr;
    default:             return 0u;
    }
}

static void flash_key_write(uint32_t v)
{
    if (s_key_state == 0u) {
        s_key_state = (v == T_KEY1) ? 1u : 0u;
    } else if (v == T_KEY2) {
        s_fcr &= ~T_CR_LOCK;
        s_key_state = 0u;
    } else {
        s_key_state = 0u;
    }
}

static void flash_reg_write(uint32_t addr, uint32_t v)
{
    switch (addr) {
    case T_FLASH_ACR:
        s_facr = v;
        return;
    case T_FLASH_KEYR:
        flash_key_write(v);
        return;
    case T_FLASH_OPTKEYR:
        return;
    case T_FLASH_SR:
        /* EOP 与各错误位写 1 清零；BSY 只读 */
        s_fsr &= ~(v & (T_SR_EOP | T_SR_ERRMASK));
        return;
    case T_FLASH_CR:
        if ((s_fcr & T_CR_LOCK) != 0u) {
            if ((v & T_CR_LOCK) != 0u) {
                s_fcr |= T_CR_LOCK;
            } else {
                c_locked_reject++;      /* 未解锁：整次写被忽略 */
            }
            return;
        }
        s_fcr = (v & ~T_CR_STRT);
        if ((v & T_CR_STRT) != 0u) {
            if ((v & T_CR_SER) != 0u) {
                s_pend_erase = (v & T_CR_SNB_MASK) >> T_CR_SNB_POS;
                s_busy = s_busy_polls;
                s_fsr |= T_SR_BSY;
            } else if ((v & T_CR_MER) != 0u) {
                s_pend_mer = 1u;
                s_busy = s_busy_polls;
                s_fsr |= T_SR_BSY;
            } else {
                s_fsr |= T_SR_PGSERR;   /* 序列错：SER/MER 都没置就 STRT */
            }
        }
        return;
    case T_FLASH_OPTCR:
        s_foptcr = v;
        return;
    default:
        return;
    }
}

static void flash_program_word(uint32_t addr, uint32_t v)
{
    uint32_t off;

    if ((s_fcr & T_CR_LOCK) != 0u) {
        c_locked_reject++;
        return;
    }
    if ((s_fcr & T_CR_PSIZE_MASK) != T_CR_PSIZE_X32) {
        c_bad_psize++;
        s_fsr |= T_SR_PGPERR;
        return;
    }
    if ((addr & 3u) != 0u || addr < T_FLASH_BASE ||
        addr >= (T_FLASH_BASE + T_FLASH_SIZE)) {
        s_fsr |= T_SR_PGSERR;
        return;
    }
    off = addr - T_FLASH_BASE;
    if (progmap_get(off / 4u) != 0u) {
        c_flash_err++;
        s_fsr |= T_SR_PGAERR;      /* 擦除前重复编程同一字 */
        return;
    }
    s_pend_prog_addr = addr;
    s_pend_prog_val  = v;
    s_pend_prog      = 1u;
    s_busy           = s_busy_polls;
    s_fsr |= T_SR_BSY;
}

/* ================================================================== */
/* SCS / DBGMCU / UID                                                  */
/* ================================================================== */
static uint32_t scs_read(uint32_t addr)
{
    switch (addr) {
    case T_DHCSR:
        return (s_halted ? 0x00020000u : 0u) | 0x00010000u | s_dhcsr_ctl;
    case T_DCRSR: return s_dcrsr;
    case T_DCRDR: return s_dcrdr;
    case T_DEMCR: return s_demcr;
    case T_AIRCR: return 0u;
    default:      return 0u;
    }
}

static void scs_write(uint32_t addr, uint32_t v)
{
    switch (addr) {
    case T_DHCSR:
        if ((v & 0xFFFF0000u) != 0xA05F0000u) {
            return;                              /* 钥匙不对：忽略 */
        }
        s_dhcsr_ctl = v & 0x0000FFFFu;
        s_halted    = ((v & 0x00000002u) != 0u) ? 1u : 0u;
        return;
    case T_DCRSR:
        s_dcrsr = v;
        if ((v & 0x00010000u) != 0u) {
            s_regs[v & 0x1Fu] = s_dcrdr;         /* 写内核寄存器 */
        } else {
            s_dcrdr = s_regs[v & 0x1Fu];         /* 读内核寄存器 */
        }
        return;
    case T_DCRDR:
        s_dcrdr = v;
        return;
    case T_DEMCR:
        s_demcr = v;
        return;
    case T_AIRCR:
        if ((v & 0xFFFF0000u) != 0x05FA0000u) {
            return;
        }
        if ((v & 0x00000004u) != 0u) {           /* SYSRESETREQ：系统复位 */
            s_halted = 0u;
            s_dhcsr_ctl &= ~0x00000002u;
            s_regs[15] = rd32(&s_flash[4]);      /* PC = 复位向量 */
            s_regs[13] = rd32(&s_flash[0]);      /* SP = 初始栈顶 */
            s_fcr  = T_CR_LOCK;                  /* 系统复位会复位 Flash 控制器 */
            s_fsr  = 0u;
            s_busy = 0u;
            s_pend_prog = 0u;
            s_key_state = 0u;
        }
        return;
    default:
        return;
    }
}

static uint32_t dbgmcu_read(uint32_t addr)
{
    switch (addr) {
    case T_DBGMCU_IDCODE: return (T_REV_ID << 16) | T_DEV_ID;
    case T_DBGMCU_CR:     return s_dbgmcu_cr;
    case T_DBGMCU_APB1FZ: return s_dbgmcu_apb1fz;
    default:              return 0u;
    }
}

static void dbgmcu_write(uint32_t addr, uint32_t v)
{
    switch (addr) {
    case T_DBGMCU_CR:     s_dbgmcu_cr = v; return;
    case T_DBGMCU_APB1FZ: s_dbgmcu_apb1fz = v; return;
    default: return;
    }
}

/* ================================================================== */
/* 存储访问                                                            */
/* ================================================================== */
static uint32_t sim_mem_read(uint32_t addr, uint32_t size)
{
    uint32_t v;

    if (addr >= T_FLASH_BASE && addr < (T_FLASH_BASE + T_FLASH_SIZE)) {
        v = rd32(&s_flash[(addr - T_FLASH_BASE) & ~3u]);
    } else if (addr >= 0x40023C00u && addr <= 0x40023C17u) {
        v = flash_reg_read(addr & ~3u);
    } else if ((addr & 0xFFFFF000u) == 0xE000E000u) {
        v = scs_read(addr);
    } else if (addr >= 0xE0042000u && addr <= 0xE004200Fu) {
        v = dbgmcu_read(addr);
    } else if (addr >= T_UID_BASE && addr < (T_UID_BASE + 12u)) {
        v = 0x00310000u + (addr - T_UID_BASE);
    } else {
        c_unmapped++;
        v = 0u;
    }
    if (size == 4u) {
        return v;
    }
    if (size == 2u) {
        return (v >> ((addr & 2u) * 8u)) & 0xFFFFu;
    }
    return (v >> ((addr & 3u) * 8u)) & 0xFFu;
}

static void sim_mem_write(uint32_t addr, uint32_t v, uint32_t size)
{
    if (addr >= T_FLASH_BASE && addr < (T_FLASH_BASE + T_FLASH_SIZE)) {
        if (size != 4u) {
            c_bad_psize++;
            s_fsr |= T_SR_PGPERR;
            return;
        }
        if ((s_fcr & T_CR_PG) != 0u) {
            flash_program_word(addr, v);
        } else {
            c_locked_reject++;      /* 没置 PG：AHB 写不改 Flash（真机同样） */
        }
        return;
    }
    if (addr >= 0x40023C00u && addr <= 0x40023C17u) {
        flash_reg_write(addr & ~3u, v);
        return;
    }
    if ((addr & 0xFFFFF000u) == 0xE000E000u) {
        scs_write(addr, v);
        return;
    }
    if (addr >= 0xE0042000u && addr <= 0xE004200Fu) {
        dbgmcu_write(addr, v);
        return;
    }
    c_unmapped++;
}

/* ================================================================== */
/* DP / AP                                                             */
/* ================================================================== */
static uint32_t sim_dp_read(uint8_t addr)
{
    switch (addr) {
    case T_DP_IDCODE:
        return s_idcode;
    case T_DP_CTRLSTAT:
        return s_dp_ctrlstat | (s_stickyerr ? T_STICKY_MASK : 0u);
    case T_DP_SELECT:
        return s_dp_select;
    case T_DP_RDBUFF:
        c_rdbuff_reads++;
        return s_ap_rdbuff;
    default:
        return 0u;
    }
}

static void sim_dp_write(uint8_t addr, uint32_t v)
{
    switch (addr) {
    case T_DP_CTRLSTAT:
        if ((v & T_STICKY_MASK) != 0u) {
            s_stickyerr = 0u;                    /* W1C */
        }
        s_dp_ctrlstat = v & 0x00000001u;         /* ORUNDETECT */
        if ((v & T_PWRUP_REQ) == T_PWRUP_REQ) {
            s_dp_ctrlstat |= T_PWRUP_ACK;        /* 上电请求立即确认 */
        }
        return;
    case T_DP_SELECT:
        s_dp_select = v;
        return;
    case 0x00u:                                  /* ABORT */
        if ((v & T_STICKY_MASK) != 0u) {
            s_stickyerr = 0u;
        }
        return;
    default:
        return;
    }
}

static uint32_t sim_ap_transfer_addr(void)
{
    return s_ap_tar;
}

/* CSW.AddrInc 语义（ADIv5 IHI0031H 表 C2-1，与 DAPLink 的用法一致）：
 *   0b00 = Auto-increment off  → TAR 不变（连续访问都打同一个地址）
 *   0b01 = Increment single    → 每次 DRW 访问后按访问宽度自增
 *   0b10 = Increment packed    → 本模型不实现，按"自增关闭"处理
 * 注意：第一版模型和固件都把 0b00/0b01 记反了（模型跟着固件一起错，
 * 所以测试全绿却与真机不符）——见 docs/02 §7.19。 */
static uint32_t sim_ap_inc(void)
{
    uint32_t inc_mode  = (s_ap_csw >> 4) & 3u;
    uint32_t size_log  = s_ap_csw & 3u;
    uint32_t size      = (size_log == 0u) ? 1u : ((size_log == 1u) ? 2u : 4u);

    if (inc_mode == 1u) {
        return size;                             /* increment single */
    }
    return 0u;                                   /* off / packed：都不自增 */
}

static uint32_t sim_ap_read(uint8_t addr)
{
    uint32_t inc;
    uint32_t v;

    switch (addr) {
    case T_AP_CSW: return s_ap_csw;
    case T_AP_TAR: return s_ap_tar;
    case T_AP_DRW:
        v   = s_ap_rdbuff;                       /* posted read：先回上一次的结果 */
        inc = sim_ap_inc();
        s_ap_rdbuff = sim_mem_read(sim_ap_transfer_addr(), 4u);   /* 再发起本次传输 */
        s_ap_tar += inc;
        return v;
    default:
        return 0u;
    }
}

static void sim_ap_write(uint8_t addr, uint32_t v)
{
    uint32_t inc;

    switch (addr) {
    case T_AP_CSW:
        s_ap_csw = v;
        return;
    case T_AP_TAR:
        s_ap_tar = v;
        return;
    case T_AP_DRW:
        sim_mem_write(sim_ap_transfer_addr(), v, 4u);
        inc = sim_ap_inc();
        s_ap_tar += inc;
        return;
    default:
        return;
    }
}

/* ================================================================== */
/* 包处理                                                              */
/* ================================================================== */
static uint8_t sim_window_is_header(uint8_t hdr)
{
    uint32_t four = (uint32_t)((hdr >> 1) & 0x0Fu);
    uint8_t  par  = (uint8_t)((hdr >> 5) & 1u);

    if ((hdr & 0x01u) == 0u) { return 0u; }      /* Start 必须为 1 */
    if ((hdr & 0x40u) != 0u) { return 0u; }      /* Stop 必须为 0  */
    if ((hdr & 0x80u) == 0u) { return 0u; }      /* Park 必须为 1  */
    return (par == sim_parity(four)) ? 1u : 0u;  /* 偶校验 */
}

static void sim_prep_read(void)
{
    uint32_t v = s_pend_apndp ? sim_ap_read(s_pend_addr) : sim_dp_read(s_pend_addr);
    uint8_t  i;

    for (i = 0u; i < 32u; i++) {
        s_out[i] = (uint8_t)((v >> i) & 1u);
    }
    s_out[32] = sim_parity(v);
    s_out_n   = 33u;
    s_out_i   = 0u;
}

static void sim_exec_write(void)
{
    if (s_in_parity != sim_parity(s_in_data)) {
        c_parity_errors++;
        s_stickyerr = 1u;
        return;                                   /* 数据校验错：不执行 */
    }
    if (s_pend_apndp != 0u) {
        sim_ap_write(s_pend_addr, s_in_data);
    } else {
        sim_dp_write(s_pend_addr, s_in_data);
    }
}

static void sim_process_header(uint8_t hdr)
{
    c_transactions++;

    s_pend_apndp = (uint8_t)((hdr >> 1) & 1u);
    s_pend_rnw   = (uint8_t)((hdr >> 2) & 1u);
    s_pend_addr  = (uint8_t)((((hdr >> 3) & 1u) << 2) | (((hdr >> 4) & 1u) << 3));

    if (s_fault_left > 0u) {
        s_fault_left--;
        s_ack = 0x04u;
        c_fault_sent++;
        s_stickyerr = 1u;
    } else if (s_wait_left > 0u) {
        s_wait_left--;
        s_ack = 0x02u;
        c_wait_sent++;
    } else {
        s_ack = 0x01u;
    }

    s_out[0] = (uint8_t)(s_ack & 1u);             /* ACK 也是 LSB first */
    s_out[1] = (uint8_t)((s_ack >> 1) & 1u);
    s_out[2] = (uint8_t)((s_ack >> 2) & 1u);
    s_out_n  = 3u;
    s_out_i  = 0u;

    s_skip_falls = 1u;                            /* 请求头之后 1 个 turnaround */
    s_tgt_drives = 0u;
    s_phase      = PH_ACK;
}

static void sim_phase_end(void)
{
    if (s_phase == PH_ACK) {
        if (s_ack == 0x01u && s_pend_rnw != 0u) {
            sim_prep_read();
            s_phase      = PH_RD_DATA;
            s_out_i      = 0u;
            s_skip_falls = s_read_ta;
            if (s_skip_falls == 0u) {
                s_tgt_val    = s_out[0];
                s_out_i      = 1u;
                s_tgt_drives = 1u;
            } else {
                s_tgt_drives = 0u;
            }
        } else if (s_ack == 0x01u) {
            s_phase      = PH_WR_DATA;
            s_in_bits    = 0u;
            s_in_data    = 0u;
            s_in_parity  = 0u;
            s_tgt_drives = 0u;
        } else {
            s_phase      = PH_IDLE;
            s_tgt_drives = 0u;
        }
        return;
    }
    if (s_phase == PH_RD_DATA) {
        s_tgt_drives = 0u;
        s_phase      = PH_IDLE;
    }
}

static void sim_line_reset(void)
{
    c_line_resets++;
    s_phase      = PH_IDLE;
    s_tgt_drives = 0u;
    s_hdr_bits   = 0u;
    s_prev_bit   = 1u;                            /* 复位结束时主机在驱动高 */
    if (s_switched == 0u) {
        s_pending_switch = 1u;                    /* 未切换：高电平结束后收切换序列 */
    }
}

static void sim_do_rising(void)
{
    c_clocks++;

    if (s_host_drives != 0u && s_tgt_drives != 0u) {
        c_contention++;
    }

    if (s_phase == PH_IDLE) {
        if (s_host_drives == 0u) {
            return;
        }
        if (s_host_val != 0u) {
            s_high_run++;
        } else {
            s_high_run = 0u;
        }
        if (s_high_run >= 50u) {
            if (s_high_run == 50u) {
                sim_line_reset();
            }
            s_prev_bit = s_host_val;
            return;
        }
        /* 线复位结束后出现的第一个低电平，就是 JTAG→SWD 切换序列的第 0 位 */
        if (s_pending_switch != 0u && s_host_val == 0u) {
            s_pending_switch = 0u;
            s_switching      = 1u;
            s_switch_bits    = 0u;
            s_switch_val     = 0u;
        }
        if (s_switching != 0u) {
            if (s_switch_bits < 16u) {
                s_switch_val |= (uint32_t)s_host_val << s_switch_bits;
            }
            s_switch_bits++;
            if (s_switch_bits == 16u) {
                s_switching = 0u;
                if (s_switch_val == T_SWITCH_SEQ) {
                    s_switched = 1u;
                    c_switch_seen++;
                }
            }
            s_prev_bit = s_host_val;
            return;
        }
        /* 请求包同步：空闲电平为低，Start 位为 1，因此包一定以一个上升沿开始 */
        if (s_hdr_bits == 0u) {
            if (s_host_val != 0u && s_prev_bit == 0u) {
                s_hdr      = 0x80u;               /* 本 bit = 包头 bit0 = Start = 1 */
                s_hdr_bits = 1u;
            }
        } else {
            s_hdr = (uint8_t)((s_hdr >> 1) | (s_host_val << 7));
            s_hdr_bits++;
            if (s_hdr_bits == 8u) {
                s_hdr_bits = 0u;
                if (sim_window_is_header(s_hdr) != 0u) {
                    sim_process_header(s_hdr);
                }
            }
        }
        s_prev_bit = s_host_val;
        return;
    }

    if (s_phase == PH_WR_DATA) {
        if (s_host_drives != 0u) {
            if (s_in_bits < 32u) {
                s_in_data |= (uint32_t)s_host_val << s_in_bits;
            } else {
                s_in_parity = s_host_val;
            }
            s_in_bits++;
            if (s_in_bits == 33u) {
                sim_exec_write();
                s_phase = PH_IDLE;
            }
        } else {
            c_ta_skipped++;                       /* 主机插入的额外 turnaround */
        }
        return;
    }

    /* PH_ACK / PH_RD_DATA：目标在驱动，主机只采样 */
}

static void sim_do_falling(void)
{
    if (s_phase != PH_ACK && s_phase != PH_RD_DATA) {
        return;
    }
    if (s_skip_falls > 0u) {
        s_skip_falls--;
        s_tgt_drives = 0u;
        return;
    }
    if (s_out_i < s_out_n) {
        s_tgt_val = s_out[s_out_i];
        s_out_i++;
        s_tgt_drives = 1u;
    } else {
        s_tgt_drives = 0u;
        sim_phase_end();
    }
}

/* ================================================================== */
/* GPIO 事件                                                           */
/* ================================================================== */
static void sim_clk_set(uint8_t level)
{
    c_clocks++;
    if (c_clocks > 20000000u) {
        /* 保护：仿真里出现异常长的时钟序列说明主机或模型卡住了，
         * 直接带状态退出，避免测试进程无限挂起。 */
        printf("\nSIM ABORT: clock limit exceeded\n");
        printf("  phase=%d host_drv=%u tgt_drv=%u out_i=%u/%u in_bits=%u hdr=0x%02X hdr_bits=%u\n",
               (int)s_phase, (unsigned)s_host_drives, (unsigned)s_tgt_drives,
               (unsigned)s_out_i, (unsigned)s_out_n, (unsigned)s_in_bits,
               (unsigned)s_hdr, (unsigned)s_hdr_bits);
        printf("  transactions=%lu wait=%lu fault=%lu ta_skip=%lu\n",
               (unsigned long)c_transactions, (unsigned long)c_wait_sent,
               (unsigned long)c_fault_sent, (unsigned long)c_ta_skipped);
        exit(9);
    }
    if (level == s_clk) {
        return;
    }
    s_clk = level;
    if (s_attached == 0u) {
        return;                                   /* 目标不在：不响应 */
    }
    if (level != 0u) {
        sim_do_rising();
    } else {
        sim_do_falling();
    }
}

void sim_gpio_write(GPIO_TypeDef *port, uint16_t pin, uint8_t level)
{
    if (port == SWD_CLK_PORT && pin == SWD_CLK_PIN) {
        sim_clk_set(level);
        return;
    }
    if (port == SWD_DIO_PORT && pin == SWD_DIO_PIN) {
        s_host_val = level;
        return;
    }
    if (port == SWD_NRST_PORT && pin == SWD_NRST_PIN) {
        if (level == 0u && s_nrst_level != 0u) {
            c_nrst++;
        }
        s_nrst_level = level;
        return;
    }
}

void sim_gpio_dir(GPIO_TypeDef *port, uint16_t pin, uint8_t output)
{
    if (port == SWD_DIO_PORT && pin == SWD_DIO_PIN) {
        s_host_drives = (output != 0u) ? 1u : 0u;
    }
}

uint8_t sim_gpio_read(GPIO_TypeDef *port, uint16_t pin)
{
    if (port != SWD_DIO_PORT || pin != SWD_DIO_PIN) {
        return 1u;
    }
    if (s_attached == 0u) {
        return 1u;                                /* 悬空：上拉为高 */
    }
    if (s_tgt_drives != 0u) {
        return s_tgt_val;
    }
    if (s_host_drives != 0u) {
        return s_host_val;
    }
    return 1u;
}

/* ================================================================== */
/* 控制与观察                                                          */
/* ================================================================== */
void sim_target_reset(void)
{
    memset(s_flash, 0xFF, sizeof(s_flash));
    memset(s_progmap, 0x00, sizeof(s_progmap));
    memset(s_regs, 0x00, sizeof(s_regs));

    s_attached      = 1u;
    s_clk           = 0u;
    s_host_drives   = 0u;
    s_host_val      = 0u;
    s_tgt_drives    = 0u;
    s_tgt_val       = 0u;
    s_nrst_level    = 1u;

    s_phase         = PH_IDLE;
    s_hdr           = 0u;
    s_hdr_bits      = 0u;
    s_prev_bit      = 1u;
    s_high_run      = 0u;
    s_pending_switch= 0u;
    s_switching     = 0u;
    s_switched      = 0u;
    s_switch_bits   = 0u;
    s_switch_val    = 0u;
    s_read_ta       = 0u;
    s_out_n         = 0u;
    s_out_i         = 0u;
    s_skip_falls    = 0u;
    s_in_bits       = 0u;

    s_wait_left     = 0u;
    s_fault_left    = 0u;
    s_stuck_busy    = 0u;
    s_idcode        = 0x2BA01477u;      /* Cortex-M4 SW-DP */
    s_busy_polls    = F4_BUSY_POLLS;

    s_dp_ctrlstat   = 0u;
    s_dp_select     = 0u;
    s_ap_rdbuff     = 0u;
    s_ap_csw        = 0u;
    s_ap_tar        = 0u;
    s_stickyerr     = 0u;

    s_fcr           = T_CR_LOCK;        /* 复位后 Flash 是锁着的 */
    s_fsr           = 0u;
    s_facr          = 0u;
    s_foptcr        = 0u;
    s_key_state     = 0u;
    s_busy          = 0u;
    s_pend_erase    = 0xFFFFFFFFu;
    s_pend_mer      = 0u;
    s_pend_prog     = 0u;

    s_halted        = 0u;               /* 上电后内核在跑 */
    s_dhcsr_ctl     = 0u;
    s_dcrsr         = 0u;
    s_dcrdr         = 0u;
    s_demcr         = 0u;
    s_dbgmcu_cr     = 0u;
    s_dbgmcu_apb1fz = 0u;

    c_clocks = c_transactions = c_contention = c_parity_errors = 0u;
    c_wait_sent = c_fault_sent = c_ta_skipped = c_locked_reject = 0u;
    c_bad_psize = c_unmapped = c_switch_seen = c_line_resets = 0u;
    c_erases = c_programs = c_rdbuff_reads = c_flash_err = c_nrst = 0u;
}

void sim_target_attach(uint8_t on)          { s_attached = (on != 0u) ? 1u : 0u; }
void sim_target_set_wait(uint32_t n)        { s_wait_left = n; }
void sim_target_set_fault(uint32_t n)       { s_fault_left = n; }
void sim_target_set_read_ta(uint8_t n)      { s_read_ta = n; }
void sim_target_set_stuck_busy(uint8_t on)  { s_stuck_busy = (on != 0u) ? 1u : 0u; }
void sim_target_set_idcode(uint32_t id)     { s_idcode = id; }

uint8_t *sim_target_flash(void)             { return s_flash; }
uint32_t sim_target_flash_size(void)        { return T_FLASH_SIZE; }

void sim_target_flash_corrupt(uint32_t addr, uint8_t value)
{
    if (addr >= T_FLASH_BASE && addr < (T_FLASH_BASE + T_FLASH_SIZE)) {
        s_flash[addr - T_FLASH_BASE] = value;
    }
}

uint32_t sim_target_clocks(void)        { return c_clocks; }
uint32_t sim_target_transactions(void)  { return c_transactions; }
uint32_t sim_target_contention(void)    { return c_contention; }
uint32_t sim_target_parity_errors(void) { return c_parity_errors; }
uint32_t sim_target_wait_sent(void)     { return c_wait_sent; }
uint32_t sim_target_fault_sent(void)    { return c_fault_sent; }
uint32_t sim_target_ta_skipped(void)    { return c_ta_skipped; }
uint32_t sim_target_locked_reject(void) { return c_locked_reject; }
uint32_t sim_target_bad_psize(void)     { return c_bad_psize; }
uint32_t sim_target_unmapped(void)      { return c_unmapped; }
uint32_t sim_target_switch_seen(void)   { return c_switch_seen; }
uint32_t sim_target_line_resets(void)   { return c_line_resets; }
uint32_t sim_target_erases(void)        { return c_erases; }
uint32_t sim_target_programs(void)      { return c_programs; }
uint32_t sim_target_rdbuff_reads(void)  { return c_rdbuff_reads; }
uint32_t sim_target_flash_err(void)     { return c_flash_err; }
uint32_t sim_target_pc(void)            { return s_regs[15]; }
uint32_t sim_target_sp(void)            { return s_regs[13]; }
uint8_t  sim_target_halted(void)        { return s_halted; }
uint32_t sim_target_iwdg_frozen(void)   { return (s_dbgmcu_apb1fz >> 12) & 1u; }
uint32_t sim_target_nrst_asserts(void)  { return c_nrst; }
