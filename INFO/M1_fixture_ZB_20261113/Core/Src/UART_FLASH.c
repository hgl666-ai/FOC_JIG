/**
  ******************************************************************************
  * @file    UART_FLASH.c
  * @brief   M1 治具 —— UART5 → DUT USART1 系统 bootloader 烧录（AN3155）
  *
  * 适用：DUT = STM32F105RBT6（128KB Flash / 64 个 2KB 页）。
  *
  * 接线：
  *   治具 UART5_TX(PC12) → DUT USART1_RX(PA10)
  *   治具 UART5_RX(PD2)  ← DUT USART1_TX(PA9)
  *   治具 PC6(NRST)      → DUT NRST（开漏，低有效）
  *   治具 PC7(BOOT0)     → DUT BOOT0（高 = 系统存储器 bootloader）
  *   DUT BOOT1(PB2) 必须为低（多数板已下拉，否则要补一根线）。
  *
  * 烧录流程：
  *   ① BOOT0 低 + NRST 20ms 复位 → 让用户程序先跑一下；
  *   ② BOOT0 高保持 100ms → NRST 20ms 复位 → 进系统 bootloader；
  *   ③ 0x7F 同步 → Get / Get Version / Get ID（读命令表、确认目标在线）；
  *   ④ 校验治具 SPI Flash 里两份镜像头（擦除之前！）；
  *   ⑤ 擦除（优先整片擦除，NACK 时退回逐页擦除）；
  *   ⑥ 256B/包写入 Boot，再写 App；
  *   ⑦ 首字交叉检查（Boot/App 栈顶）；可选逐块回读；
  *   ⑧ BOOT0 低 + NRST 复位 → 运行新程序。
  *
  * 与 SWD_FLASH.c 完全独立：FLASH_MODE_UART5=1 时只编译本文件。
  * 协议细节参考 AN3155（CRC 用异或校验，不是 CRC16）。
  ******************************************************************************
  */
#include "flash_mode.h"

#if FLASH_MODE_UART5

#include "UART_FLASH.h"

#include "usart.h"
#include "usart_m.h"
#include "log.h"
#include "gd25q128.h"
#include "flash_map.h"

#include <string.h>

/*==============================================================================
 * §1 AN3155 常量与超时
 *============================================================================*/
#define UF_ACK_BYTE             0x79u
#define UF_NACK_BYTE            0x1Fu
#define UF_BUSY_BYTE            0x76u
#define UF_SYNC_BYTE            0x7Fu

#define UF_CMD_GET              0x00u
#define UF_CMD_GET_VERSION      0x01u
#define UF_CMD_GET_ID           0x02u
#define UF_CMD_READ_MEMORY      0x11u
#define UF_CMD_GO               0x21u
#define UF_CMD_WRITE_MEMORY     0x31u
#define UF_CMD_WRITE_NO_STRETCH 0x32u
#define UF_CMD_ERASE            0x43u
#define UF_CMD_EXT_ERASE        0x44u
#define UF_CMD_EXT_ERASE_NS     0x45u
#define UF_CMD_WRITE_UNPROTECT  0x73u

#define UF_SYNC_TIMEOUT_MS      200u
#define UF_ACK_TIMEOUT_MS       1000u
#define UF_BLOCK_TIMEOUT_MS     3000u     /* 256B 写入 + F1 编程约 7ms，留足余量 */
#define UF_READ_TIMEOUT_MS      1000u
#define UF_ERASE_TIMEOUT_MS     10000u    /* 128KB 整片擦除远小于 1s */
#define UF_SYNC_RETRY           5u
#define UF_ENTER_RETRY          2u        /* 同步失败后重新复位进 bootloader 的次数 */

/* 目标 Flash 基址/容量（与 flash_map.h 的 DUT_* 配合使用） */
#define UF_FLASH_BASE           0x08000000u
#define UF_FLASH_PAGES          (DUT_FLASH_SIZE / DUT_PAGE_SIZE)

/*==============================================================================
 * §2 日志与内部状态
 *============================================================================*/
static uartf_state_t s_state = UFP_ST_IDLE;
static uint8_t  s_result = APP_STATUS_ERR_PROG;
static uint8_t  s_last_err = UF_OK;
static uint8_t  s_fail_state = 0u;

static uint32_t s_img_len_boot = 0u;
static uint32_t s_img_len_app  = 0u;

static uint8_t  s_phase = 0u;
static uint32_t s_slot_base = 0u;
static uint32_t s_dst_addr = 0u;
static uint32_t s_img_len = 0u;
static uint32_t s_offset = 0u;

static uint32_t s_page_first = 0u;
static uint32_t s_page_last = 0u;

static uint8_t  s_erase_cmd = UF_CMD_ERASE;
static uint8_t  s_write_cmd = UF_CMD_WRITE_MEMORY;
static uint8_t  s_read_cmd  = UF_CMD_READ_MEMORY;

static uint8_t  s_buf[UARTF_CHUNK_BYTES];
static uint8_t  s_vbuf[UARTF_CHUNK_BYTES];   /* 只在 UARTF_VERIFY_ENABLE=1 时使用 */

static uint32_t s_t_start = 0u;
static uint32_t s_t_erase = 0u;

static uint8_t  s_dut_ctrl_ready = 0u;

/*==============================================================================
 * §3 目标板 NRST / BOOT0（与 SWD_FLASH.c 的同名逻辑一致，但独立实现）
 *============================================================================*/
static void uf_nrst_drive(uint8_t assert_reset)
{
#if (UF_DUT_NRST_ACTIVE_LOW != 0)
    HAL_GPIO_WritePin(UF_DUT_NRST_PORT, UF_DUT_NRST_PIN,
                      (assert_reset != 0u) ? GPIO_PIN_RESET : GPIO_PIN_SET);
#else
    HAL_GPIO_WritePin(UF_DUT_NRST_PORT, UF_DUT_NRST_PIN,
                      (assert_reset != 0u) ? GPIO_PIN_SET : GPIO_PIN_RESET);
#endif
}

static void uf_boot_drive(uint8_t bootloader)
{
#if (UF_DUT_BOOT_ACTIVE_HIGH != 0)
    HAL_GPIO_WritePin(UF_DUT_BOOT_PORT, UF_DUT_BOOT_PIN,
                      (bootloader != 0u) ? GPIO_PIN_SET : GPIO_PIN_RESET);
#else
    HAL_GPIO_WritePin(UF_DUT_BOOT_PORT, UF_DUT_BOOT_PIN,
                      (bootloader != 0u) ? GPIO_PIN_RESET : GPIO_PIN_SET);
#endif
}

void UARTF_DutCtrlInit(void)
{
    GPIO_InitTypeDef init;

    if (s_dut_ctrl_ready != 0u) {
        return;
    }
    __HAL_RCC_GPIOC_CLK_ENABLE();

    /* 上电默认让 DUT 运行用户程序：NRST 释放、BOOT0 低。
     * （SWD 方案初始化时 BOOT0 是高，这里刻意不同：UART 方案平时不需要
     *   目标停在 bootloader，烧录时 UARTF_Start 会重新走一遍时序。） */
    uf_nrst_drive(0u);
    uf_boot_drive(0u);

    init.Pin   = UF_DUT_NRST_PIN;
    init.Mode  = UF_DUT_NRST_MODE;
    init.Pull  = GPIO_NOPULL;
    init.Speed = GPIO_SPEED_FREQ_LOW;
    HAL_GPIO_Init(UF_DUT_NRST_PORT, &init);

    init.Pin   = UF_DUT_BOOT_PIN;
    init.Mode  = GPIO_MODE_OUTPUT_PP;
    init.Pull  = GPIO_NOPULL;
    init.Speed = GPIO_SPEED_FREQ_LOW;
    HAL_GPIO_Init(UF_DUT_BOOT_PORT, &init);

    s_dut_ctrl_ready = 1u;

    /* 上电/空闲默认：8N1 256000，和 DUT App 的 USART1 一致。
     * 烧录时 UARTF_Start/UARTF_Probe 才临时切到 115200 8E1（AN2606 Table 48），
     * 烧完先恢复这个默认配置，再复位 DUT。 */
    if (huart5.Instance != 0) {
        UART5_SetConfig(UARTF_TEST_BAUD, UART_PARITY_NONE);
        UART5_SetMode(UART5_MODE_DEFAULT);
    }
}

/* 进系统 bootloader：先让 App 跑一下，再 BOOT0 高 + 复位 */
static void uf_enter_bootloader(void)
{
    uf_boot_drive(0u);
    uf_nrst_drive(1u);
    HAL_Delay(UF_RESET_HOLD_MS);
    uf_nrst_drive(0u);
    HAL_Delay(UF_APP_RUN_MS);

    uf_boot_drive(1u);
    HAL_Delay(UF_BOOT_SETUP_MS);
    uf_nrst_drive(1u);
    HAL_Delay(UF_RESET_HOLD_MS);
    uf_nrst_drive(0u);
    HAL_Delay(UF_BOOT_READY_MS);

    UART5_FlushRx();
}

/* 运行新程序：BOOT0 低 + 复位 */
static void uf_exit_bootloader(void)
{
    uf_boot_drive(0u);
    uf_nrst_drive(1u);
    HAL_Delay(UF_RESET_HOLD_MS);
    uf_nrst_drive(0u);
    HAL_Delay(5u);
}

/* 失败时：释放 NRST、BOOT0 保持高，目标留在 bootloader 方便重试 */
static void uf_hold_bootloader(void)
{
    uf_boot_drive(1u);
    uf_nrst_drive(0u);
}

/*==============================================================================
 * §4 AN3155 基础收发
 *============================================================================*/
static uint8_t uf_xor(const uint8_t *p, uint16_t len)
{
    uint8_t x = 0u;
    uint16_t i;

    for (i = 0u; i < len; i++) {
        x ^= p[i];
    }
    return x;
}

static uint8_t uf_send_raw(const uint8_t *data, uint16_t len)
{
    /* 1000ms 是上限，不是固定延迟：256B 包 @115200 约 22ms 就发完。
     * 留大一点是为了以后把 UARTF_BAUD 降到 9600 也能用。 */
    if (UART5_Transmit(data, len, 1000u) != HAL_OK) {
        return UF_ERR_UART;
    }
    return UF_OK;
}

/** 等一个 ACK/NACK；0x76(BUSY) 继续等，0x00 当线路噪声忽略 */
static uint8_t uf_wait_ack(uint32_t timeout_ms)
{
    uint32_t start = HAL_GetTick();

    for (;;) {
        uint8_t b;

        if (UART5_ReadByte(&b)) {
            if (b == UF_ACK_BYTE) {
                return UF_OK;
            }
            if (b == UF_NACK_BYTE) {
                return UF_ERR_NACK;
            }
            if (b == UF_BUSY_BYTE) {
                start = HAL_GetTick();        /* 目标忙，重新计时 */
                continue;
            }
            if (b == 0x00u) {
                continue;                     /* 复位期间地线/浮空脚噪声 */
            }
            /* 收到既不是 ACK 也不是 NACK 的字节：如果目标是 App（256000）
             * 而治具在 115200 听，就会看到这种“乱码”。打出来便于判断。 */
            LOGW("uartf: unexpected byte 0x%x (want ACK/NACK)", (int)b);
            return UF_ERR_PROTO;
        }

        if ((uint32_t)(HAL_GetTick() - start) >= timeout_ms) {
            return UF_ERR_TIMEOUT;
        }
    }
}

/** 发命令 + 反码，等 ACK（命令字节和反码的异或校验 = 0xFF） */
static uint8_t uf_send_cmd(uint8_t cmd, uint32_t timeout_ms)
{
    uint8_t buf[2];

    UART5_FlushRx();                          /* 丢掉上一条命令可能残留的字节 */
    buf[0] = cmd;
    buf[1] = (uint8_t)(cmd ^ 0xFFu);

    if (uf_send_raw(buf, 2u) != UF_OK) {
        return UF_ERR_UART;
    }
    return uf_wait_ack(timeout_ms);
}

/** 发 4 字节地址（大端）+ 异或校验，等 ACK */
static uint8_t uf_send_addr(uint32_t addr, uint32_t timeout_ms)
{
    uint8_t buf[5];

    buf[0] = (uint8_t)(addr >> 24);
    buf[1] = (uint8_t)(addr >> 16);
    buf[2] = (uint8_t)(addr >> 8);
    buf[3] = (uint8_t)(addr);
    buf[4] = uf_xor(buf, 4u);

    if (uf_send_raw(buf, 5u) != UF_OK) {
        return UF_ERR_UART;
    }
    return uf_wait_ack(timeout_ms);
}

/** 0x7F 同步（bootloader 用它做自动波特率检测） */
static uint8_t uf_sync(void)
{
    uint8_t i;

    for (i = 0u; i < UF_SYNC_RETRY; i++) {
        uint8_t b = UF_SYNC_BYTE;
        uint8_t r;

        UART5_FlushRx();
        if (UART5_Transmit(&b, 1u, 100u) != HAL_OK) {
            return UF_ERR_UART;
        }
        r = uf_wait_ack(UF_SYNC_TIMEOUT_MS);
        if (r == UF_OK) {
            LOGI("uartf: sync ACK on try %d", (int)(i + 1u));
            return UF_OK;
        }
        /* 第一次 0x7F 被当成命令起始、第二次回 NACK：接口已经对齐。
         * 但这只说明"收到了 NACK"，并不证明对面是 bootloader；后面的
         * Get Version/Get ID 会再确认一次。 */
        if (r == UF_ERR_NACK) {
            LOGW("uartf: sync via NACK (not a clean ACK)");
            return UF_OK;
        }
        LOGW("uartf: sync try %d err=%d", (int)(i + 1u), (int)r);
    }
    return UF_ERR_TIMEOUT;
}

/* Get Version：UART bootloader 返回 3 字节 + ACK */
static uint8_t uf_get_version(uint8_t *ver, uint8_t *opt1, uint8_t *opt2)
{
    uint8_t buf[3];
    uint8_t r = uf_send_cmd(UF_CMD_GET_VERSION, UF_ACK_TIMEOUT_MS);

    if (r != UF_OK) {
        return r;
    }
    if (UART5_Read(buf, 3u, UF_ACK_TIMEOUT_MS) != 3u) {
        return UF_ERR_TIMEOUT;
    }
    if (ver != 0)  { *ver  = buf[0]; }
    if (opt1 != 0) { *opt1 = buf[1]; }
    if (opt2 != 0) { *opt2 = buf[2]; }

    return uf_wait_ack(UF_ACK_TIMEOUT_MS);
}

/* Get ID：ACK → N(=1) → PID 高/低 */
static uint8_t uf_get_id(uint16_t *pid)
{
    uint8_t n;
    uint8_t buf[2];
    uint8_t r = uf_send_cmd(UF_CMD_GET_ID, UF_ACK_TIMEOUT_MS);

    if (r != UF_OK) {
        return r;
    }
    if (UART5_Read(&n, 1u, UF_ACK_TIMEOUT_MS) != 1u) {
        return UF_ERR_TIMEOUT;
    }
    if (n != 1u) {
        uint8_t tmp[8];
        uint16_t len = (uint16_t)n + 1u;

        if (len > sizeof(tmp)) {
            len = (uint16_t)sizeof(tmp);
        }
        (void)UART5_Read(tmp, len, UF_ACK_TIMEOUT_MS);
        return UF_ERR_PROTO;
    }
    if (UART5_Read(buf, 2u, UF_ACK_TIMEOUT_MS) != 2u) {
        return UF_ERR_TIMEOUT;
    }
    if (pid != 0) {
        *pid = ((uint16_t)buf[0] << 8) | (uint16_t)buf[1];
    }
    return UF_OK;
}

/* Get：返回 bootloader 版本 + 支持的命令码表 */
static uint8_t uf_get(uint8_t *ver, uint8_t *cmds, uint16_t max_cmds, uint16_t *n_cmds)
{
    uint8_t n;
    uint8_t buf[32];
    uint16_t len;
    uint8_t r = uf_send_cmd(UF_CMD_GET, UF_ACK_TIMEOUT_MS);

    if (r != UF_OK) {
        return r;
    }
    if (UART5_Read(&n, 1u, UF_ACK_TIMEOUT_MS) != 1u) {
        return UF_ERR_TIMEOUT;
    }

    len = (uint16_t)n + 1u;
    if (len > sizeof(buf)) {
        /* 命令表比预期长：读掉剩余字节保持链路同步，然后报协议错误 */
        uint8_t tmp[16];
        uint16_t remain = len;

        while (remain > 0u) {
            uint16_t take = (remain > sizeof(tmp)) ? (uint16_t)sizeof(tmp) : remain;
            if (UART5_Read(tmp, take, UF_ACK_TIMEOUT_MS) != take) {
                return UF_ERR_TIMEOUT;
            }
            remain = (uint16_t)(remain - take);
        }
        return UF_ERR_PROTO;
    }
    if (UART5_Read(buf, len, UF_ACK_TIMEOUT_MS) != len) {
        return UF_ERR_TIMEOUT;
    }

    if (ver != 0) {
        *ver = buf[0];
    }
    n = (uint8_t)(n > max_cmds ? max_cmds : n);
    if (cmds != 0 && n > 0u) {
        memcpy(cmds, &buf[1], n);
    }
    if (n_cmds != 0) {
        *n_cmds = n;
    }
    return UF_OK;
}

static uint8_t uf_cmd_supported(const uint8_t *cmds, uint16_t n, uint8_t cmd)
{
    uint16_t i;

    for (i = 0u; i < n; i++) {
        if (cmds[i] == cmd) {
            return 1u;
        }
    }
    return 0u;
}

/* 从 Get 的命令表里挑擦除/写/读命令；都挑不到就保持默认值 */
static void uf_detect_commands(const uint8_t *cmds, uint16_t n)
{
    if (uf_cmd_supported(cmds, n, UF_CMD_ERASE)) {
        s_erase_cmd = UF_CMD_ERASE;
    } else if (uf_cmd_supported(cmds, n, UF_CMD_EXT_ERASE)) {
        s_erase_cmd = UF_CMD_EXT_ERASE;
    } else if (uf_cmd_supported(cmds, n, UF_CMD_EXT_ERASE_NS)) {
        s_erase_cmd = UF_CMD_EXT_ERASE_NS;
    }

    if (uf_cmd_supported(cmds, n, UF_CMD_WRITE_MEMORY)) {
        s_write_cmd = UF_CMD_WRITE_MEMORY;
    } else if (uf_cmd_supported(cmds, n, UF_CMD_WRITE_NO_STRETCH)) {
        s_write_cmd = UF_CMD_WRITE_NO_STRETCH;
    }

    if (uf_cmd_supported(cmds, n, UF_CMD_READ_MEMORY)) {
        s_read_cmd = UF_CMD_READ_MEMORY;
    }
}

/*==============================================================================
 * §5 擦除 / 写入 / 回读
 *============================================================================*/
static uint8_t uf_erase_pages(uint32_t first, uint32_t last)
{
    static uint8_t buf[3u + 256u + 2u];
    uint16_t idx = 0u;
    uint32_t count;
    uint32_t p;
    uint8_t  r;

    if (last < first) {
        return UF_ERR_PARAM;
    }
    count = last - first + 1u;
    if (count == 0u || count > 256u) {
        return UF_ERR_PARAM;
    }
    /* 0x44/0x45 每页 2 字节，缓冲区只能容纳 129 页；先挡住越界 */
    if ((s_erase_cmd == UF_CMD_EXT_ERASE || s_erase_cmd == UF_CMD_EXT_ERASE_NS) &&
        (count > ((sizeof(buf) - 3u) / 2u))) {
        return UF_ERR_PARAM;
    }

    if (s_erase_cmd == UF_CMD_EXT_ERASE || s_erase_cmd == UF_CMD_EXT_ERASE_NS) {
        uint16_t n = (uint16_t)(count - 1u);

        buf[idx++] = (uint8_t)(n >> 8);
        buf[idx++] = (uint8_t)(n);
        for (p = first; p <= last; p++) {
            buf[idx++] = (uint8_t)(p >> 8);
            buf[idx++] = (uint8_t)(p);
        }
    } else {
        buf[idx++] = (uint8_t)(count - 1u);
        for (p = first; p <= last; p++) {
            buf[idx++] = (uint8_t)p;
        }
    }
    buf[idx] = uf_xor(buf, idx);
    idx++;

    r = uf_send_cmd(s_erase_cmd, UF_ACK_TIMEOUT_MS);
    if (r != UF_OK) {
        return r;
    }
    if (uf_send_raw(buf, idx) != UF_OK) {
        return UF_ERR_UART;
    }
    return uf_wait_ack(UF_ERASE_TIMEOUT_MS);
}

/** Write Unprotect：清 WRP。0x73 正常回两次 ACK（收到命令一次、擦完一次）。 */
static uint8_t uf_write_unprotect(void)
{
    uint8_t r = uf_send_cmd(UF_CMD_WRITE_UNPROTECT, UF_ACK_TIMEOUT_MS);

    if (r != UF_OK) {
        return r;
    }
    return uf_wait_ack(UF_ERASE_TIMEOUT_MS);
}

/** 优先整片擦除（0x43 用 FF 00；0x44/0x45 用 FF FF 00），NACK 退回逐页 */
static uint8_t uf_erase_all(uint32_t first, uint32_t last)
{
    uint8_t r;
    uint8_t alt;

    /* 1) 发擦除命令。Get 命令表读不到时默认 0x43；如果 bootloader 只支持
     *    0x44/0x45，会在命令字节就 NACK，这里换一种再试。 */
    r = uf_send_cmd(s_erase_cmd, UF_ACK_TIMEOUT_MS);
    if (r == UF_ERR_NACK) {
        alt = (s_erase_cmd == UF_CMD_ERASE) ? UF_CMD_EXT_ERASE : UF_CMD_ERASE;
        LOGW("uartf: erase cmd 0x%x NACK, try 0x%x", (int)s_erase_cmd, (int)alt);
        r = uf_send_cmd(alt, UF_ACK_TIMEOUT_MS);
        if (r == UF_OK) {
            s_erase_cmd = alt;
        }
    }

    /* 2) 两种擦除命令都不收：可能是 WRP。Write Unprotect 只清保护、不擦数据。 */
    if (r == UF_ERR_NACK) {
        LOGW("uartf: erase NACK, try write unprotect 0x73");
        r = uf_write_unprotect();
        if (r == UF_OK) {
            LOGW("uartf: write unprotect ok, retry erase");
            r = uf_send_cmd(s_erase_cmd, UF_ACK_TIMEOUT_MS);
            if (r == UF_ERR_NACK) {
                alt = (s_erase_cmd == UF_CMD_ERASE) ? UF_CMD_EXT_ERASE : UF_CMD_ERASE;
                r = uf_send_cmd(alt, UF_ACK_TIMEOUT_MS);
                if (r == UF_OK) {
                    s_erase_cmd = alt;
                }
            }
        }
    }
    if (r != UF_OK) {
        return r;
    }

    if (s_erase_cmd == UF_CMD_EXT_ERASE || s_erase_cmd == UF_CMD_EXT_ERASE_NS) {
        uint8_t buf[3];

        buf[0] = 0xFFu;
        buf[1] = 0xFFu;
        buf[2] = 0x00u;                       /* (FF ^ FF ^ 00) = 00 */
        if (uf_send_raw(buf, 3u) != UF_OK) {
            return UF_ERR_UART;
        }
    } else {
        uint8_t buf[2];

        buf[0] = 0xFFu;                       /* 0xFF 命令 + 0x00 反码 */
        buf[1] = 0x00u;
        if (uf_send_raw(buf, 2u) != UF_OK) {
            return UF_ERR_UART;
        }
    }

    r = uf_wait_ack(UF_ERASE_TIMEOUT_MS);
    if (r == UF_OK) {
        return UF_OK;
    }
    if (r == UF_ERR_NACK) {
        LOGW("uartf: mass erase NACK, try page erase");
        return uf_erase_pages(first, last);
    }
    return r;
}

/** 写一块：地址 4 字节对齐，长度 1..256，不足 4 字节补 0xFF */
static uint8_t uf_write_block(uint32_t addr, const uint8_t *data, uint16_t len)
{
    static uint8_t pkt[UARTF_CHUNK_BYTES + 2u];
    uint16_t aligned;
    uint16_t i;
    uint8_t  cs;
    uint8_t  r;

    if (len == 0u || len > UARTF_CHUNK_BYTES) {
        return UF_ERR_PARAM;
    }
    if ((addr & 0x3u) != 0u) {
        return UF_ERR_PARAM;
    }

    r = uf_send_cmd(s_write_cmd, UF_ACK_TIMEOUT_MS);
    if (r != UF_OK) {
        return r;
    }
    r = uf_send_addr(addr, UF_ACK_TIMEOUT_MS);
    if (r != UF_OK) {
        return r;
    }

    aligned = (uint16_t)((len + 3u) & (uint16_t)~3u);
    pkt[0] = (uint8_t)(aligned - 1u);
    cs = pkt[0];
    for (i = 0u; i < len; i++) {
        pkt[1u + i] = data[i];
        cs ^= data[i];
    }
    for (i = len; i < aligned; i++) {
        pkt[1u + i] = 0xFFu;
        cs ^= 0xFFu;
    }
    pkt[1u + aligned] = cs;

    if (uf_send_raw(pkt, (uint16_t)(aligned + 2u)) != UF_OK) {
        return UF_ERR_UART;
    }
    return uf_wait_ack(UF_BLOCK_TIMEOUT_MS);
}

/** 回读一块：ACK → 地址 → 长度(N-1+反码) → N 字节 */
static uint8_t uf_read_block(uint32_t addr, uint8_t *data, uint16_t len)
{
    uint8_t n;
    uint8_t lc[2];
    uint8_t r;

    if (len == 0u || len > UARTF_CHUNK_BYTES) {
        return UF_ERR_PARAM;
    }

    r = uf_send_cmd(s_read_cmd, UF_ACK_TIMEOUT_MS);
    if (r != UF_OK) {
        return r;
    }
    r = uf_send_addr(addr, UF_ACK_TIMEOUT_MS);
    if (r != UF_OK) {
        return r;
    }

    n = (uint8_t)(len - 1u);
    lc[0] = n;
    lc[1] = (uint8_t)(n ^ 0xFFu);
    if (uf_send_raw(lc, 2u) != UF_OK) {
        return UF_ERR_UART;
    }
    r = uf_wait_ack(UF_ACK_TIMEOUT_MS);
    if (r != UF_OK) {
        return r;
    }

    if (UART5_Read(data, len, UF_READ_TIMEOUT_MS) != len) {
        return UF_ERR_TIMEOUT;
    }
    return UF_OK;
}

/*==============================================================================
 * §6 镜像与状态机
 *============================================================================*/
static uint32_t uf_min_u32(uint32_t a, uint32_t b)
{
    return (a < b) ? a : b;
}

/* 跟 SWD 版 swdp_load_image_head 完全同义：
 *   1 = 可用；0 = 空/不完整/超分区；2 = 治具 SPI Flash 读失败 */
static uint8_t uf_load_image_head(uint32_t slot_base, uint32_t max_len, uint32_t *p_len)
{
    uint8_t  hdr[FW_HEAD_SIZE];
    uint32_t size;

    if (!GD25Q_ReadBuffer(hdr, slot_base, (uint16_t)FW_HEAD_SIZE)) {
        return 2u;
    }

    size = (uint32_t)hdr[FW_HEAD_OFF_SIZE]
         | ((uint32_t)hdr[FW_HEAD_OFF_SIZE + 1u] << 8)
         | ((uint32_t)hdr[FW_HEAD_OFF_SIZE + 2u] << 16)
         | ((uint32_t)hdr[FW_HEAD_OFF_SIZE + 3u] << 24);

    if (hdr[FW_HEAD_OFF_FLAG] != FW_FLAG_COMPLETE) {
        return 0u;
    }
    if (size == 0u || size > max_len) {
        return 0u;
    }
    *p_len = size;
    return 1u;
}

static void uf_select_image(uint8_t which)
{
    s_phase = which;
    if (which == 0u) {
        s_slot_base = FW_SLOT0_BASE;
        s_dst_addr  = DUT_BOOT_ADDR;
        s_img_len   = s_img_len_boot;
    } else {
        s_slot_base = FW_SLOT1_BASE;
        s_dst_addr  = DUT_APP_ADDR;
        s_img_len   = s_img_len_app;
    }
    s_offset = 0u;
}

static void uartf_fail(uint8_t status, uint8_t raw_err)
{
    s_last_err   = raw_err;
    s_result     = status;
    s_fail_state = (uint8_t)s_state;
    s_state      = UFP_ST_FAIL;

    /* 失败不上锁：释放 NRST，但 BOOT0 保持高，目标留在 bootloader，
     * 方便 PC 直接重发 0x0012 再试。 */
    uf_hold_bootloader();

    LOGE("uartf: FAIL step=%d status=%d err=%d phase=%d off=%d",
         (int)s_fail_state, (int)status, (int)raw_err,
         (int)s_phase, (int)s_offset);

    if (s_fail_state == (uint8_t)UFP_ST_ENTER) {
        LOGE("uartf: check DUT BOOT1(PB2)=0, BOOT0=1 at NRST rising edge, PC7->BOOT0 wire");
    }

    /* 失败也把 UART5 恢复默认模式；DUT 仍停在 bootloader（BOOT0=高，
     * NRST 已释放），重发 0x0012 会重新切到 8E1。 */
    UART5_SetConfig(UARTF_TEST_BAUD, UART_PARITY_NONE);
    UART5_SetMode(UART5_MODE_DEFAULT);
}

/** 写完两个镜像后，读回 Boot/App 第一个字与 SPI 源比对（快速兜底） */
static uint8_t uf_word0_check(void)
{
    uint8_t src[4];
    uint8_t dst[4];

    if (s_img_len_boot >= 4u) {
        if (!GD25Q_ReadBuffer(src, FW_SLOT0_BASE + FW_HEAD_SIZE, 4u)) {
            return UF_ERR_FLASH;
        }
        if (uf_read_block(DUT_BOOT_ADDR, dst, 4u) != UF_OK) {
            return UF_ERR_READ;
        }
        if (memcmp(src, dst, 4u) != 0) {
            LOGE("uartf: word0 boot exp=%x,%x,%x,%x got=%x,%x,%x,%x",
                 (int)src[0], (int)src[1], (int)src[2], (int)src[3],
                 (int)dst[0], (int)dst[1], (int)dst[2], (int)dst[3]);
            return UF_ERR_VERIFY;
        }
    }

    if (s_img_len_app >= 4u) {
        if (!GD25Q_ReadBuffer(src, FW_SLOT1_BASE + FW_HEAD_SIZE, 4u)) {
            return UF_ERR_FLASH;
        }
        if (uf_read_block(DUT_APP_ADDR, dst, 4u) != UF_OK) {
            return UF_ERR_READ;
        }
        if (memcmp(src, dst, 4u) != 0) {
            LOGE("uartf: word0 app exp=%x,%x,%x,%x got=%x,%x,%x,%x",
                 (int)src[0], (int)src[1], (int)src[2], (int)src[3],
                 (int)dst[0], (int)dst[1], (int)dst[2], (int)dst[3]);
            return UF_ERR_VERIFY;
        }
    }
    return UF_OK;
}

/*==============================================================================
 * §7 对外接口
 *============================================================================*/
void UARTF_Start(void)
{
    UARTF_DutCtrlInit();
    /* 烧录模式：115200 8E1；结束后会恢复默认 256000 8N1 */
    UART5_SetConfig(UARTF_BAUD, UART_PARITY_EVEN);
    UART5_SetMode(UART5_MODE_FLASH);

    s_result       = APP_STATUS_ERR_PROG;
    s_last_err     = UF_OK;
    s_fail_state   = 0u;
    s_img_len_boot = 0u;
    s_img_len_app  = 0u;
    s_phase        = 0u;
    s_offset       = 0u;
    s_page_first   = 0u;
    s_page_last    = 0u;
    s_erase_cmd    = UF_CMD_ERASE;
    s_write_cmd    = UF_CMD_WRITE_MEMORY;
    s_read_cmd     = UF_CMD_READ_MEMORY;

    s_t_start = HAL_GetTick();
    s_state   = UFP_ST_ENTER;

    LOGI("uartf: start, boot0 high + nrst %dms, baud=%d",
         (int)UF_RESET_HOLD_MS, (int)UARTF_BAUD);
    uf_enter_bootloader();
}

void UARTF_RunApp(void)
{
    UARTF_DutCtrlInit();
    /* BOOT0 低 + 复位；失败时也能把目标从 bootloader 里放出来 */
    uf_exit_bootloader();
}

uint8_t UARTF_Busy(void)
{
    return ((s_state != UFP_ST_IDLE) &&
            (s_state != UFP_ST_OK) &&
            (s_state != UFP_ST_FAIL)) ? 1u : 0u;
}

uint8_t UARTF_Result(void)
{
    return (s_state == UFP_ST_OK) ? APP_STATUS_OK : s_result;
}

uint8_t UARTF_State(void)
{
    return (uint8_t)s_state;
}

uint32_t UARTF_TotalBytes(void)
{
    return s_img_len_boot + s_img_len_app;
}

uint32_t UARTF_LastOffset(void)
{
    return s_offset;
}

uint8_t UARTF_LastError(void)
{
    return s_last_err;
}

uint8_t UARTF_FailState(void)
{
    return s_fail_state;
}

void UARTF_Step(void)
{
    uint8_t r;

    switch (s_state) {

    /* ── 1. 同步 + 读目标信息 ── */
    case UFP_ST_ENTER: {
        uint8_t  attempt;
        uint8_t  ver = 0u, opt1 = 0u, opt2 = 0u;
        uint16_t pid = 0u;
        uint8_t  cmds[32];
        uint16_t ncmds = 0u;
        uint8_t  got_ver;
        uint8_t  got_id;
        uint8_t  rv;

        for (attempt = 0u; ; attempt++) {
            r = uf_sync();
            if (r == UF_OK) {
                break;
            }
            if ((attempt + 1u) >= UF_ENTER_RETRY) {
                uartf_fail(APP_STATUS_ERR_PROG, r);
                return;
            }
            LOGW("uartf: sync failed err=%d, re-enter bootloader", (int)r);
            uf_enter_bootloader();
        }

        rv = uf_get_version(&ver, &opt1, &opt2);
        got_ver = (rv == UF_OK) ? 1u : 0u;
        if (got_ver != 0u) {
            LOGI("uartf: bootloader ver=%x opt=%x/%x",
                 (int)ver, (int)opt1, (int)opt2);
        } else {
            LOGW("uartf: get version failed err=%d", (int)rv);
        }

        /* 注意：AN3155 规定 Get Version 的 option byte 1/2 为 0x00（兼容保留），
         * 不能拿它当 RDP 标志；真正的保护状态由擦除/读命令是否 NACK 判断。
         * 这里只把两个 option 字节打出来供现场参考。 */

        rv = uf_get_id(&pid);
        got_id = (rv == UF_OK) ? 1u : 0u;
        if (got_id != 0u) {
            LOGI("uartf: pid=%x", (int)pid);
        } else {
            LOGW("uartf: get id failed err=%d", (int)rv);
        }

        /* 两条基本信息都读不到 = 0x7F 只是碰巧收到一个 NACK，链路并没有
         * 真正进入 bootloader。此时绝不能继续擦除（目标可能还在跑 App）。 */
        if ((got_ver == 0u) && (got_id == 0u)) {
            uartf_fail(APP_STATUS_ERR_PROG, UF_ERR_PROTO);
            break;
        }

        rv = uf_get(&ver, cmds, (uint16_t)sizeof(cmds), &ncmds);
        if (rv == UF_OK) {
            uf_detect_commands(cmds, ncmds);
            LOGI("uartf: erase=0x%x write=0x%x read=0x%x",
                 (int)s_erase_cmd, (int)s_write_cmd, (int)s_read_cmd);
        } else {
            LOGW("uartf: get command list failed err=%d, use defaults", (int)rv);
        }

        s_state = UFP_ST_PREPARE;
        break;
    }

    /* ── 2. 校验镜像头、算擦除范围（擦除之前！） ── */
    case UFP_ST_PREPARE: {
        uint8_t r0 = uf_load_image_head(FW_SLOT0_BASE, DUT_BOOT_SIZE, &s_img_len_boot);
        uint8_t r1;

        if (r0 != 1u) {
            uartf_fail((r0 == 2u) ? APP_STATUS_ERR_FLASH : APP_STATUS_ERR_INCOMPLETE, 0u);
            break;
        }
        r1 = uf_load_image_head(FW_SLOT1_BASE, DUT_APP_SIZE, &s_img_len_app);
        if (r1 != 1u) {
            uartf_fail((r1 == 2u) ? APP_STATUS_ERR_FLASH : APP_STATUS_ERR_INCOMPLETE, 0u);
            break;
        }

        s_page_first = (DUT_BOOT_ADDR - UF_FLASH_BASE) / DUT_PAGE_SIZE;
        s_page_last  = (DUT_APP_ADDR + s_img_len_app - 1u - UF_FLASH_BASE) / DUT_PAGE_SIZE;

        if (s_page_last >= UF_FLASH_PAGES) {
            uartf_fail(APP_STATUS_ERR_SIZE, UF_ERR_PARAM);
            break;
        }

        LOGI("uartf: boot=%d app=%d pages=%d..%d",
             (int)s_img_len_boot, (int)s_img_len_app,
             (int)s_page_first, (int)s_page_last);

        s_t_erase = HAL_GetTick();
        s_state   = UFP_ST_ERASE;
        break;
    }

    /* ── 3. 擦除 ── */
    case UFP_ST_ERASE:
        r = uf_erase_all(s_page_first, s_page_last);
        if (r != UF_OK) {
            uartf_fail(APP_STATUS_ERR_PROG, r);
            break;
        }
        LOGI("uartf: erase done, %d ms", (int)(HAL_GetTick() - s_t_erase));

        uf_select_image(0u);
        s_state = UFP_ST_PROGRAM;
        break;

    /* ── 4. 写入：一步一包 256B ── */
    case UFP_ST_PROGRAM: {
        uint32_t remain = (s_offset < s_img_len) ? (s_img_len - s_offset) : 0u;

        if (remain == 0u) {
            if (s_phase == 0u) {
                uf_select_image(1u);          /* 接着写 App */
            } else {
                r = uf_word0_check();
                if (r != UF_OK) {
                    uartf_fail(APP_STATUS_ERR_PROG, r);
                    break;
                }
#if (UARTF_VERIFY_ENABLE != 0)
                uf_select_image(0u);
                s_state = UFP_ST_VERIFY;
#else
                s_state = UFP_ST_GO;
#endif
            }
            break;
        }
        {
            uint16_t chunk = (uint16_t)uf_min_u32(remain, UARTF_CHUNK_BYTES);

            if (!GD25Q_ReadBuffer(s_buf, s_slot_base + FW_HEAD_SIZE + s_offset, chunk)) {
                /* SPI 源读不到：绝不能把缓冲区里的垃圾写进目标 */
                uartf_fail(APP_STATUS_ERR_FLASH, UF_ERR_FLASH);
                break;
            }
            r = uf_write_block(s_dst_addr + s_offset, s_buf, chunk);
            if (r != UF_OK) {
                uartf_fail(APP_STATUS_ERR_PROG, r);
                break;
            }
            s_offset += chunk;

            if ((s_offset == s_img_len) ||
                ((s_offset % (UARTF_CHUNK_BYTES * 64u)) == 0u)) {
                LOGI("uartf: prog %d/%d", (int)s_offset, (int)s_img_len);
            }
        }
        break;
    }

    /* ── 5. 可选：逐块回读比对 ── */
    case UFP_ST_VERIFY: {
        uint32_t remain = (s_offset < s_img_len) ? (s_img_len - s_offset) : 0u;

        if (remain == 0u) {
            if (s_phase == 0u) {
                uf_select_image(1u);
            } else {
                s_state = UFP_ST_GO;
            }
            break;
        }
        {
            uint16_t chunk = (uint16_t)uf_min_u32(remain, UARTF_CHUNK_BYTES);
            /* 读长度也按 4 字节对齐（bootloader 的 Read 最好按字读），
             * 只比较真实数据部分。 */
            uint16_t rd_len = (uint16_t)((chunk + 3u) & (uint16_t)~3u);

            if (!GD25Q_ReadBuffer(s_buf, s_slot_base + FW_HEAD_SIZE + s_offset, chunk)) {
                uartf_fail(APP_STATUS_ERR_FLASH, UF_ERR_FLASH);
                break;
            }
            r = uf_read_block(s_dst_addr + s_offset, s_vbuf, rd_len);
            if (r != UF_OK) {
                uartf_fail(APP_STATUS_ERR_PROG, r);
                break;
            }
            if (memcmp(s_buf, s_vbuf, chunk) != 0) {
                uartf_fail(APP_STATUS_ERR_PROG, UF_ERR_VERIFY);
                break;
            }
            s_offset += chunk;
        }
        break;
    }

    /* ── 6. 先恢复 UART5 默认 8N1 256000，再复位运行新程序 ── */
    case UFP_ST_GO:
        UART5_SetConfig(UARTF_TEST_BAUD, UART_PARITY_NONE);
        UART5_SetMode(UART5_MODE_DEFAULT);
        UART5_FlushRx();

        uf_exit_bootloader();
        LOGI("uartf: done, %d+%d bytes in %d ms",
             (int)s_img_len_boot, (int)s_img_len_app,
             (int)(HAL_GetTick() - s_t_start));
        s_result = APP_STATUS_OK;
        s_state  = UFP_ST_OK;
        break;

    default:
        break;
    }
}

uint8_t UARTF_Probe(uartf_probe_t *out)
{
    uint8_t  ver = 0u, opt1 = 0u, opt2 = 0u;
    uint16_t pid = 0u;
    uint8_t  b[4];
    uint8_t  r;

    if (out == 0) {
        return APP_STATUS_ERR_PARAM;
    }
    memset(out, 0, sizeof(*out));

    UARTF_DutCtrlInit();
    /* 烧录模式：115200 8E1；结束时恢复默认 256000 8N1 */
    UART5_SetConfig(UARTF_BAUD, UART_PARITY_EVEN);
    UART5_SetMode(UART5_MODE_FLASH);

    /* 直接进 bootloader（不用先跑 App） */
    uf_boot_drive(1u);
    HAL_Delay(UF_BOOT_SETUP_MS);
    uf_nrst_drive(1u);
    HAL_Delay(UF_RESET_HOLD_MS);
    uf_nrst_drive(0u);
    HAL_Delay(UF_BOOT_READY_MS);
    UART5_FlushRx();

    r = uf_sync();
    if (r != UF_OK) {
        out->status   = APP_STATUS_ERR_PROG;
        out->uart_err = r;
        goto probe_exit;
    }

    if (uf_get_version(&ver, &opt1, &opt2) == UF_OK) {
        out->bl_version = ver;
        out->rdp = (uint8_t)(opt1 & 0x01u);   /* OBR bit0 = RDPRT */
    }
    if (uf_get_id(&pid) == UF_OK) {
        out->pid = pid;
    }
    if (uf_read_block(DUT_BOOT_ADDR, b, 4u) == UF_OK) {
        out->boot_sp = (uint32_t)b[0] | ((uint32_t)b[1] << 8) |
                       ((uint32_t)b[2] << 16) | ((uint32_t)b[3] << 24);
    }
    if (uf_read_block(DUT_APP_ADDR, b, 4u) == UF_OK) {
        out->app_sp = (uint32_t)b[0] | ((uint32_t)b[1] << 8) |
                      ((uint32_t)b[2] << 16) | ((uint32_t)b[3] << 24);
    }

    out->status = APP_STATUS_OK;
    LOGI("uartf probe: ver=%x pid=%x rdp=%d boot_sp=%x app_sp=%x",
         (int)out->bl_version, (int)out->pid, (int)out->rdp,
         (unsigned int)out->boot_sp, (unsigned int)out->app_sp);

probe_exit:
    /* 先恢复默认 8N1 256000，再复位 DUT 跑 App */
    UART5_SetConfig(UARTF_TEST_BAUD, UART_PARITY_NONE);
    UART5_SetMode(UART5_MODE_DEFAULT);
    UART5_FlushRx();

    /* 无论成功失败都退出 bootloader，别把目标留在下载模式 */
    uf_exit_bootloader();
    if (out->status != APP_STATUS_OK) {
        LOGE("uartf probe: failed err=%d", (int)out->uart_err);
    }
    return out->status;
}

#endif /* FLASH_MODE_UART5 */
