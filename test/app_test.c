/*
 * PC ↔ 治具 U1 全链路宿主仿真测试
 *
 * 直接把固件的 app.c / fw_upload.c / app_frame.c / protocol_frame.c /
 * gd25q128.c / isp_programmer.c 真源码编译进来, 宿主侧提供:
 *   - GD25Q128 SPI Flash 芯片仿真 (状态寄存器/WEL/扇区擦除/跨页编程)
 *   - STM32 系统 bootloader 仿真 (被烧板)
 *   - USB CDC 收发替身 (捕获 U1 发出的帧 / 注入 PC 发来的帧)
 * 从而在无硬件条件下验证整条链路:
 *   PC 帧 → app 分发 → 上传状态机 → SPI Flash 驱动 → ISP → 被烧板 Flash
 */
#include <stdio.h>
#include <string.h>
#include <stdint.h>

#include "app.h"
#include "app_frame.h"
#include "protocol_frame.h"
#include "flash_map.h"
#include "fw_upload.h"
#include "gd25q128.h"
#include "isp_programmer.h"
#include "isp_config.h"
#include "bsp_uart.h"
#include "usbd_cdc_if.h"
#include "usb_device.h"

/*==============================================================================
 * 虚拟时间
 *============================================================================*/
static uint32_t g_tick = 0;
uint32_t HAL_GetTick(void) { return g_tick++; }
void     HAL_Delay(uint32_t ms) { g_tick += ms; }
static void host_advance(uint32_t ms) { g_tick += ms; }

/*==============================================================================
 * GD25Q128 仿真 (16MB)
 *============================================================================*/
static uint8_t g_spi[16u * 1024u * 1024u];

static struct {
    int      cs;              /* 1 = CS# 拉低 (片选有效) */
    uint8_t  cmd;
    uint8_t  phase;           /* 0=命令 1=地址 2=读输出 3=写SR 4=页编程 9=忽略 */
    uint32_t addr;
    uint8_t  addr_cnt;
    uint8_t  sr1;             /* bit0=WIP */
    uint8_t  sr2;
    uint8_t  wel;
    uint8_t  is_read, is_prog, is_erase;
    uint8_t  win[256];
    uint16_t win_idx;
    uint32_t page_start;
    uint8_t  out[4];
    uint8_t  out_len, out_pos;
    uint8_t  force_wrong_id;   /* 1 = RDID 返回全 0 (模拟 Flash 不在线/型号不对) */
    /* 观测点 */
    uint32_t rdsr_count, wren_count, pp_count, se_count;
    uint32_t wel_violations;
} fl;

static void flash_cs_low(void)
{
    fl.cs = 1; fl.phase = 0; fl.addr_cnt = 0; fl.out_len = 0; fl.out_pos = 0;
    fl.is_read = fl.is_prog = fl.is_erase = 0; fl.win_idx = 0;
}

static void flash_cs_high(void)
{
    if (fl.is_prog && fl.win_idx > 0) {
        if (!fl.wel) fl.wel_violations++;
        else {
            memcpy(&g_spi[fl.page_start], fl.win, fl.win_idx);
            fl.pp_count++;
        }
        fl.wel = 0;
    }
    fl.cs = 0;
    fl.is_prog = 0;
}

static uint8_t flash_xfer(uint8_t tx)
{
    uint8_t rx = 0xFFu;

    if (!fl.cs) return 0xFFu;

    switch (fl.phase) {

    case 0:                                     /* 命令 */
        fl.cmd = tx;
        switch (tx) {
        case 0x06: fl.wel = 1; fl.wren_count++; break;              /* WREN */
        case 0x05: fl.out[0] = fl.sr1; fl.out_len = 1; fl.out_pos = 0;
                   fl.phase = 2; fl.rdsr_count++; break;            /* RDSR1 */
        case 0x9F: fl.out[0] = fl.force_wrong_id ? 0x00u : 0xC8u;
                   fl.out[1] = fl.force_wrong_id ? 0x00u : 0x40u;
                   fl.out[2] = fl.force_wrong_id ? 0x00u : 0x18u;
                   fl.out_len = 3; fl.out_pos = 0; fl.phase = 2; break; /* RDID */
        case 0x01: fl.phase = 3; fl.cmd = 0x01; break;              /* WRSR1 */
        case 0x31: fl.phase = 3; fl.cmd = 0x31; break;              /* WRSR2 */
        case 0x03: fl.addr = 0; fl.addr_cnt = 0; fl.is_read = 1; fl.phase = 1; break; /* READ */
        case 0x02: fl.addr = 0; fl.addr_cnt = 0; fl.is_prog = 1; fl.win_idx = 0; fl.phase = 1; break;
        case 0x20: fl.addr = 0; fl.addr_cnt = 0; fl.is_erase = 1; fl.phase = 1; break;
        default:   fl.phase = 9; break;
        }
        break;

    case 1:                                     /* 3 字节地址 */
        fl.addr = (fl.addr << 8) | (uint32_t)tx;
        if (++fl.addr_cnt == 3) {
            if (fl.is_erase) {
                if (!fl.wel) fl.wel_violations++;
                else { memset(&g_spi[fl.addr & ~0xFFFu], 0xFF, 4096u); fl.se_count++; }
                fl.wel = 0;
                fl.phase = 9;
            } else if (fl.is_prog) {
                fl.page_start = fl.addr;
                fl.phase = 4;
            } else {
                fl.phase = 2;
            }
        }
        break;

    case 2:                                     /* 读输出 */
        if (fl.is_read) {
            rx = g_spi[fl.addr++];
        } else {
            rx = (fl.out_pos < fl.out_len) ? fl.out[fl.out_pos++] : 0xFFu;
        }
        break;

    case 3:                                     /* 写状态寄存器 */
        if (fl.cmd == 0x01) fl.sr1 = tx; else fl.sr2 = tx;
        fl.sr1 &= (uint8_t)~0x01u;              /* 仿真: WIP 立即完成 */
        fl.phase = 9;
        break;

    case 4:                                     /* 页编程数据 */
        fl.win[fl.win_idx++] = tx;
        break;

    default:
        break;
    }
    return rx;
}

/* ---- HAL 替身 ---- */
SPI_HandleTypeDef hspi2;                    /* gd25q128.c 里 &hspi2 需要实体 */

void HAL_GPIO_Init(GPIO_TypeDef *port, GPIO_InitTypeDef *init) { (void)port; (void)init; }

void HAL_GPIO_WritePin(GPIO_TypeDef *port, uint16_t pin, GPIO_PinState state)
{
    if (port == GPIOB && pin == GPIO_PIN_12) {          /* 治具 Flash 片选 */
        if (state == GPIO_PIN_RESET) flash_cs_low();
        else                         flash_cs_high();
    }
}

HAL_StatusTypeDef HAL_SPI_TransmitReceive(SPI_HandleTypeDef *hspi,
                                          uint8_t *pTxData, uint8_t *pRxData,
                                          uint16_t Size, uint32_t Timeout)
{
    (void)hspi; (void)Timeout;
    for (uint16_t i = 0; i < Size; i++) pRxData[i] = flash_xfer(pTxData[i]);
    return HAL_OK;
}

/*==============================================================================
 * 被烧板: STM32 系统 bootloader 仿真
 *============================================================================*/
#define DUT_FLASH_SIZE  (64u * 1024u)
static uint8_t g_dut[DUT_FLASH_SIZE];

typedef enum { SS_CMD = 0, SS_CMDCK, SS_ADDR, SS_ERASE, SS_WN, SS_WDATA } SS_t;

static struct {
    uint8_t  bl_version;
    uint8_t  support_0x43;
    uint8_t  support_0x44;
    uint8_t  silent;
    SS_t     st;
    uint8_t  cur_cmd, idx;
    uint8_t  buf[8];
    uint32_t addr;
    uint16_t wlen, wgot;
    uint8_t  wcksum;
    uint8_t  wbuf[256];
    uint8_t  erase_cmd_used;
    uint8_t  jumped;
    uint32_t write_ok, nack_count, ack_count;
} sim;

static uint8_t  rxq[512];
static uint16_t rxq_h = 0, rxq_t = 0;
static void q_push(uint8_t b) { if (((rxq_h + 1) & 511) != rxq_t) { rxq[rxq_h] = b; rxq_h = (rxq_h + 1) & 511; } }
static int  q_pop(uint8_t *b) { if (rxq_t == rxq_h) return 0; *b = rxq[rxq_t]; rxq_t = (rxq_t + 1) & 511; return 1; }
static void sim_ack(void)  { q_push(0x79); sim.ack_count++; }
static void sim_nack(void) { q_push(0x1F); sim.nack_count++; }

static void sim_reset(uint8_t ver, uint8_t s43, uint8_t s44, uint8_t silent)
{
    memset(&sim, 0, sizeof(sim));
    memset(g_dut, 0xFF, sizeof(g_dut));
    rxq_h = rxq_t = 0;
    sim.bl_version = ver; sim.support_0x43 = s43; sim.support_0x44 = s44;
    sim.silent = silent; sim.st = SS_CMD;
}

static void sim_feed(uint8_t b)
{
    if (sim.silent || sim.jumped) return;

    switch (sim.st) {
    case SS_CMD:
        if (b == 0x7Fu) { sim_ack(); return; }
        sim.cur_cmd = b; sim.st = SS_CMDCK; return;

    case SS_CMDCK:
        if (b != (uint8_t)(sim.cur_cmd ^ 0xFFu)) { sim_nack(); sim.st = SS_CMD; return; }
        switch (sim.cur_cmd) {
        case 0x01: sim_ack(); q_push(0x01); q_push(sim.bl_version); q_push(0x00); sim_ack(); sim.st = SS_CMD; return;
        case 0x31: sim_ack(); sim.idx = 0; sim.st = SS_ADDR; return;
        case 0x21: sim_ack(); sim.idx = 0; sim.st = SS_ADDR; return;
        case 0x43:
            if (!sim.support_0x43) { sim_nack(); sim.st = SS_CMD; return; }
            sim_ack(); sim.idx = 0; sim.st = SS_ERASE; return;
        case 0x44:
            if (!sim.support_0x44) { sim_nack(); sim.st = SS_CMD; return; }
            sim_ack(); sim.idx = 0; sim.st = SS_ERASE; return;
        default: sim_nack(); sim.st = SS_CMD; return;
        }

    case SS_ADDR:
        sim.buf[sim.idx++] = b;
        if (sim.idx == 5) {
            uint8_t x = (uint8_t)(sim.buf[0] ^ sim.buf[1] ^ sim.buf[2] ^ sim.buf[3]);
            if (x != sim.buf[4]) { sim_nack(); sim.st = SS_CMD; return; }
            sim.addr = ((uint32_t)sim.buf[0] << 24) | ((uint32_t)sim.buf[1] << 16)
                     | ((uint32_t)sim.buf[2] << 8) | (uint32_t)sim.buf[3];
            sim_ack();
            if (sim.cur_cmd == 0x21) { sim.jumped = 1; sim.st = SS_CMD; }
            else                     { sim.st = SS_WN; }
        }
        return;

    case SS_ERASE:
        sim.buf[sim.idx++] = b;
        if (sim.idx == 3) {
            if ((uint8_t)(sim.buf[0] ^ sim.buf[1]) != sim.buf[2]) { sim_nack(); sim.st = SS_CMD; return; }
            if (sim.buf[0] == 0xFFu && sim.buf[1] == 0xFFu) {
                memset(g_dut, 0xFF, sizeof(g_dut));
                sim.erase_cmd_used = sim.cur_cmd;
                sim_ack();
            } else sim_nack();
            sim.st = SS_CMD;
        }
        return;

    case SS_WN:
        sim.wlen = (uint16_t)b + 1u; sim.wgot = 0; sim.wcksum = b; sim.st = SS_WDATA; return;

    case SS_WDATA:
        if (sim.wgot < sim.wlen) { sim.wbuf[sim.wgot++] = b; sim.wcksum ^= b; return; }
        if (b == sim.wcksum) {
            uint32_t off = sim.addr - 0x08000000u;
            if (off + sim.wlen <= DUT_FLASH_SIZE) { memcpy(&g_dut[off], sim.wbuf, sim.wlen); sim.write_ok++; }
            sim_ack();
        } else sim_nack();
        sim.st = SS_CMD;
        return;
    }
}

/*==============================================================================
 * BSP 替身
 *============================================================================*/
void BSP_ISP_UART_Init(void) { }
void BSP_ISP_UART_SetParity(uint8_t p) { (void)p; }
void BSP_ISP_UART_FlushRx(void) { }
void BSP_ISP_UART_SendByte(uint8_t b) { sim_feed(b); }
void BSP_ISP_UART_Send(const uint8_t *d, uint16_t n) { for (uint16_t i = 0; i < n; i++) sim_feed(d[i]); }
uint8_t BSP_ISP_UART_RecvByte(uint8_t *out, uint32_t t) { (void)t; return q_pop(out) ? 1u : 0u; }
void BSP_DBG_Send(const char *s) { (void)s; }
void BSP_DBG_SendHexByte(uint8_t b) { (void)b; }

/*==============================================================================
 * USB CDC 替身
 *============================================================================*/
static USBD_CDC_HandleTypeDef g_cdc;
USBD_HandleTypeDef hUsbDeviceFS;

static uint8_t  tx_buf[64];
static uint16_t tx_len = 0;
static uint32_t tx_count = 0;

uint8_t CDC_Transmit_FS(uint8_t *Buf, uint16_t Len)
{
    if (Len <= sizeof(tx_buf)) { memcpy(tx_buf, Buf, Len); tx_len = Len; tx_count++; }
    return USBD_OK;
}
void MX_USB_DEVICE_Init(void) { }

/*==============================================================================
 * 测试辅助
 *============================================================================*/
static int g_checks = 0, g_fails = 0;
#define CHECK(c, m) do { g_checks++; if (!(c)) { printf("  FAIL: %s (line %d)\n", m, __LINE__); g_fails++; } } while (0)

static void pump(int n) { for (int i = 0; i < n; i++) APP_Run(); }

/* 推进主循环直到 U1 主动发出指定命令的帧 (用于 0x0112 这种"烧完才回"的 ACK) */
static int wait_for_cmd(uint16_t cmd, uint16_t *seq_out, uint8_t *st_out)
{
    for (uint32_t i = 0; i < 200000u; i++) {
        tx_len = 0;
        APP_Run();
        if (tx_len > 0) {
            uint16_t c = 0, s = 0, d = 0;
            uint8_t  dd[64];
            if (Frame_Parse(tx_buf, tx_len, &c, &s, dd, &d) && c == cmd) {
                if (seq_out) *seq_out = s;
                if (st_out)  *st_out  = d ? dd[0] : 0xFFu;
                return 1;
            }
        }
    }
    return 0;
}

/* 发一帧给 U1 并收集应答; 返回应答帧长度(0=无应答) */
static uint16_t pc_send(uint16_t cmd, uint16_t seq, const uint8_t *data, uint16_t len)
{
    uint8_t  raw[8 + FRAME_MAX_DATA + 2];
    uint16_t n = Frame_Build(raw, cmd, seq, data, len);
    tx_len = 0;
    APP_USB_Receive(raw, n);
    pump(8);
    return tx_len;
}

/* 断言应答内容 */
static void expect_ack(uint16_t cmd, uint16_t seq, const uint8_t *st, uint16_t stlen, const char *what)
{
    uint16_t cmd_o = 0, seq_o = 0, dlen = 0;
    uint8_t  d[64];
    int ok = (tx_len > 0) && Frame_Parse(tx_buf, tx_len, &cmd_o, &seq_o, d, &dlen);
    CHECK(ok, what);
    if (!ok) { printf("      (%s 无有效应答)\n", what); return; }
    CHECK(cmd_o == cmd, what);
    CHECK(seq_o == seq, "SEQ 回显正确");
    CHECK(dlen == stlen, "ACK LEN 正确");
    if (dlen == stlen) CHECK(memcmp(d, st, stlen) == 0, "ACK 内容正确");
    if (dlen != stlen || memcmp(d, st, stlen) != 0) {
        printf("      得到:"); for (uint16_t i = 0; i < dlen; i++) printf(" %02X", d[i]); printf("\n");
        printf("      期望:"); for (uint16_t i = 0; i < stlen; i++) printf(" %02X", st[i]); printf("\n");
    }
}

/* 走一遍完整上传流程 */
static void upload(uint16_t *seq, const uint8_t *img, uint32_t size, uint8_t ver[3])
{
    uint8_t  d9[9];
    uint16_t frames = (uint16_t)((size + 255u) / 256u);

    d9[0] = (uint8_t)(frames >> 8); d9[1] = (uint8_t)(frames);
    d9[2] = (uint8_t)(size >> 24);  d9[3] = (uint8_t)(size >> 16);
    d9[4] = (uint8_t)(size >> 8);   d9[5] = (uint8_t)(size);
    d9[6] = ver[0]; d9[7] = ver[1]; d9[8] = ver[2];

    pc_send(MGR_CMD_FW_START, (*seq)++, d9, 9);

    for (uint16_t i = 0; i < frames; i++) {
        uint32_t off = (uint32_t)i * 256u;
        uint16_t n = (uint16_t)((size - off > 256u) ? 256u : (size - off));
        pc_send(MGR_CMD_FW_DATA, (*seq)++, &img[off], n);
    }
    pc_send(MGR_CMD_FW_END, (*seq)++, 0, 0);
}

/*==============================================================================
 * 用例
 *============================================================================*/
static uint8_t g_boot[101];
static uint8_t g_app[301];

static void init_env(void)
{
    memset(g_spi, 0xFF, sizeof(g_spi));
    memset(&fl, 0, sizeof(fl));
    fl.sr1 = 0x00;
    g_tick = 0;
    tx_len = 0; tx_count = 0;
    sim_reset(0x22, 1, 1, 0);
    hUsbDeviceFS.pClassData = &g_cdc;
    g_cdc.TxState = 0;
    for (int i = 0; i < 101; i++) g_boot[i] = (uint8_t)(0x11 + i * 3);
    for (int i = 0; i < 301; i++) g_app[i]  = (uint8_t)(0xA0 + i * 7);

    APP_Init();     /* 内部会调 GD25Q 初始化等 */
    GD25Q_Init();
}

static void t1_flash_id(void)
{
    printf("[T1] SPII Flash 器件识别\n");
    init_env();
    CHECK(GD25Q_Test() == 1u, "GD25Q_Test 通过 (制造商 0xC8 + 容量 0x18)");
    printf("    JEDEC ID = 0x%06X (期望 0xC84018)\n", GD25Q_ReadID());
    CHECK(GD25Q_ReadID() == 0xC84018u, "JEDEC ID = 0xC84018");
}

static void t2_empty_and_unknown(void)
{
    printf("[T2] 空 Flash 查询 / 未知命令 / CRC 错误 / SEQ 回显\n");
    init_env();

    { uint8_t e[4] = {0x06, 0x00, 0xFF, 0xFF};
      pc_send(MGR_CMD_QUERY_FLASH_VER, 0x0101, 0, 0);
      expect_ack(MGR_CMD_QUERY_FLASH_VER, 0x0101, e, 4, "空 Flash → ERR_NO_FW"); }

    { uint8_t e[4] = {0x0C, 0x00, 0xFF, 0xFF};
      pc_send(MGR_CMD_QUERY_FOC_VER, 0x0202, 0, 0);
      expect_ack(MGR_CMD_QUERY_FOC_VER, 0x0202, e, 4, "0x000D 未配置 → 0x0C + 00FFFF"); }

    { uint8_t e[1] = {0xFF};
      pc_send(0x1234, 0x0303, 0, 0);
      expect_ack(0x1234, 0x0303, e, 1, "未知命令 → NIY 0xFF"); }

    /* CRC 被破坏 → 按协议不回 ACK */
    { uint8_t raw[16]; uint16_t n = Frame_Build(raw, MGR_CMD_QUERY_FLASH_VER, 0x0404, 0, 0);
      raw[n - 1] ^= 0xFF; tx_len = 0; APP_USB_Receive(raw, n); pump(8);
      CHECK(tx_len == 0, "CRC 错误 → 不回 ACK"); }

    /* 非 IDLE 语义: 空闲态发 DATA/END 应为 BUSY */
    { uint8_t e[1] = {0x01};
      pc_send(MGR_CMD_FW_DATA, 0x0505, (const uint8_t *)"AB", 2);
      expect_ack(MGR_CMD_FW_DATA, 0x0505, e, 1, "空闲态 0x000A → BUSY"); }
    { uint8_t e[1] = {0x01};
      pc_send(MGR_CMD_FW_END, 0x0606, 0, 0);
      expect_ack(MGR_CMD_FW_END, 0x0606, e, 1, "空闲态 0x000B → BUSY"); }
}

static void t3_upload_and_slot_switch(void)
{
    printf("[T3] 上传 → 查询 → 双槽切换\n");
    init_env();
    uint16_t seq = 0x1000;
    uint8_t  ver1[3] = {1, 2, 3};
    uint8_t  ver2[3] = {4, 5, 6};

    upload(&seq, g_boot, sizeof(g_boot), ver1);
    { uint8_t e[4] = {0x00, 1, 2, 3};
      pc_send(MGR_CMD_QUERY_FLASH_VER, seq++, 0, 0);
      expect_ack(MGR_CMD_QUERY_FLASH_VER, seq - 1, e, 4, "第一次上传后查询到 v1.2.3"); }

    /* SPI Flash Slot0 内容逐字节核对 */
    { uint8_t hdr[FW_HEAD_SIZE];
      GD25Q_ReadBuffer(hdr, FW_SLOT0_BASE, FW_HEAD_SIZE);
      uint32_t sz = (uint32_t)hdr[0] | ((uint32_t)hdr[1] << 8) | ((uint32_t)hdr[2] << 16) | ((uint32_t)hdr[3] << 24);
      CHECK(sz == sizeof(g_boot), "Slot0 头部 size 正确");
      CHECK(hdr[4] == 1 && hdr[5] == 2 && hdr[6] == 3, "Slot0 头部版本正确");
      CHECK(hdr[7] == FW_FLAG_COMPLETE, "Slot0 完整标志 = 0xA5");
      static uint8_t rd[128];
      GD25Q_ReadBuffer(rd, FW_SLOT0_BASE + FW_HEAD_SIZE, (uint16_t)sizeof(g_boot));
      CHECK(memcmp(rd, g_boot, sizeof(g_boot)) == 0, "Slot0 镜像逐字节一致"); }

    /* 第二次上传自动切到 Slot1 */
    upload(&seq, g_app, sizeof(g_app), ver2);
    { uint8_t e[4] = {0x00, 4, 5, 6};
      pc_send(MGR_CMD_QUERY_FLASH_VER, seq++, 0, 0);
      expect_ack(MGR_CMD_QUERY_FLASH_VER, seq - 1, e, 4, "第二次上传后查询到 v4.5.6 (App 槽优先)"); }

    { static uint8_t rd[320];
      GD25Q_ReadBuffer(rd, FW_SLOT1_BASE + FW_HEAD_SIZE, (uint16_t)sizeof(g_app));
      CHECK(memcmp(rd, g_app, sizeof(g_app)) == 0, "Slot1 镜像逐字节一致"); }

    printf("    扇区擦除=%u 页编程=%u WREN=%u RDSR=%u WEL违规=%u\n",
           fl.se_count, fl.pp_count, fl.wren_count, fl.rdsr_count, fl.wel_violations);
    CHECK(fl.wel_violations == 0, "无 WEL 违规 (每次擦写前都先 WREN)");
}

static void t4_start_param_errors(void)
{
    printf("[T4] 0x0009 参数异常\n");
    init_env();
    uint16_t seq = 0x2000;
    uint8_t  e2[1] = {0x02};

    { uint8_t d[8] = {0, 2, 0, 0, 1, 0, 1, 2};      /* LEN=8 < 9 */
      pc_send(MGR_CMD_FW_START, seq++, d, 8);
      expect_ack(MGR_CMD_FW_START, seq - 1, e2, 1, "LEN<9 → ERR_SIZE"); }

    { uint8_t d[9] = {0, 2, 0, 0, 0, 0, 1, 2, 3};   /* size=0 */
      pc_send(MGR_CMD_FW_START, seq++, d, 9);
      expect_ack(MGR_CMD_FW_START, seq - 1, e2, 1, "size=0 → ERR_SIZE"); }

    { uint8_t d[9] = {0, 0, 0, 0, 0, 10, 1, 2, 3};  /* frames=0 */
      pc_send(MGR_CMD_FW_START, seq++, d, 9);
      expect_ack(MGR_CMD_FW_START, seq - 1, e2, 1, "frames=0 → ERR_SIZE"); }

    { uint8_t d[9] = {0, 2, 0, 1, 0, 0, 1, 2, 3};   /* size=65536 > 限 */
      pc_send(MGR_CMD_FW_START, seq++, d, 9);
      expect_ack(MGR_CMD_FW_START, seq - 1, e2, 1, "size 越界 → ERR_SIZE"); }
}

static void t5_overflow_and_latch(void)
{
    printf("[T5] 上传溢出 → 错误锁存\n");
    init_env();
    uint16_t seq = 0x3000;
    uint8_t  ver[3] = {1, 0, 0};
    uint8_t  d9[9] = {0, 1, 0, 0, 0, 100, 1, 0, 0};   /* 声报 100 字节 */
    uint8_t  ok[1] = {0x00}, ov[1] = {0x0D};

    pc_send(MGR_CMD_FW_START, seq++, d9, 9);
    expect_ack(MGR_CMD_FW_START, seq - 1, ok, 1, "START OK");

    { uint8_t big[200]; memset(big, 0x5A, sizeof(big));
      pc_send(MGR_CMD_FW_DATA, seq++, big, 200);       /* 超出发报 100 */
      expect_ack(MGR_CMD_FW_DATA, seq - 1, ov, 1, "溢出 → ERR_OVERFLOW 0x0D"); }

    { uint8_t small[10]; memset(small, 0x11, sizeof(small));
      pc_send(MGR_CMD_FW_DATA, seq++, small, 10);      /* 锁存后仍回 0x0D, 不写 Flash */
      expect_ack(MGR_CMD_FW_DATA, seq - 1, ov, 1, "锁存后仍回 0x0D"); }

    pc_send(MGR_CMD_FW_END, seq++, 0, 0);
    expect_ack(MGR_CMD_FW_END, seq - 1, ov, 1, "END 回锁存错误码");

    { uint8_t e[4] = {0x07, 0x00, 0xFF, 0xFF};
      pc_send(MGR_CMD_QUERY_FLASH_VER, seq++, 0, 0);
      expect_ack(MGR_CMD_QUERY_FLASH_VER, seq - 1, e, 4, "未写完整标志 → ERR_INCOMPLETE"); }
    (void)ver;
}

static void t6_count_mismatch(void)
{
    printf("[T6] 字节数与声明不符 → ERR_COUNT 且不写完整标志\n");
    init_env();
    uint16_t seq = 0x4000;
    uint8_t  d9[9] = {0, 1, 0, 0, 0, 200, 2, 0, 0};   /* 声明 200 字节 / 1 帧 */
    uint8_t  ok[1] = {0x00}, ec[1] = {0x0A};

    pc_send(MGR_CMD_FW_START, seq++, d9, 9);
    expect_ack(MGR_CMD_FW_START, seq - 1, ok, 1, "START OK");

    { uint8_t d[100]; memset(d, 0x33, sizeof(d));
      pc_send(MGR_CMD_FW_DATA, seq++, d, 100);        /* 只发 100, 声明 200 */
      expect_ack(MGR_CMD_FW_DATA, seq - 1, ok, 1, "DATA OK"); }

    pc_send(MGR_CMD_FW_END, seq++, 0, 0);
    expect_ack(MGR_CMD_FW_END, seq - 1, ec, 1, "END → ERR_COUNT 0x0A");

    { uint8_t e[4] = {0x07, 0x00, 0xFF, 0xFF};        /* 有内容但无完整标志 */
      pc_send(MGR_CMD_QUERY_FLASH_VER, seq++, 0, 0);
      expect_ack(MGR_CMD_QUERY_FLASH_VER, seq - 1, e, 4, "完整标志未写 → ERR_INCOMPLETE"); }
}

static void t7_upload_timeout(void)
{
    printf("[T7] 上传 10s 无新数据 → ERR_TIMEOUT 并回 IDLE\n");
    init_env();
    uint16_t seq = 0x5000;
    uint8_t  d9[9] = {0, 1, 0, 0, 1, 0, 1, 0, 0};     /* 声明 256 字节 1 帧 */
    uint8_t  ok[1] = {0x00};

    pc_send(MGR_CMD_FW_START, seq++, d9, 9);
    expect_ack(MGR_CMD_FW_START, seq - 1, ok, 1, "START OK");

    /* 不发数据, 直接推进 20 秒 */
    tx_len = 0;
    host_advance(20000u);
    pump(4);
    { uint8_t e[1] = {0x0C};
      expect_ack(MGR_CMD_FW_DATA, seq - 1, e, 1, "超时 → ACK ERR_TIMEOUT 0x0C"); }

    /* 已回 IDLE: 再发 DATA 应为 BUSY */
    { uint8_t e[1] = {0x01};
      pc_send(MGR_CMD_FW_DATA, seq++, (const uint8_t *)"X", 1);
      expect_ack(MGR_CMD_FW_DATA, seq - 1, e, 1, "超时后回到 IDLE (DATA → BUSY)"); }
}

static void t8_program_precheck(void)
{
    printf("[T8] 0x0112 前置检查\n");
    init_env();
    uint16_t seq = 0x6000;
    uint8_t  ok[1] = {0x00};
    uint8_t  ver[3] = {1, 0, 0};

    { uint8_t e[1] = {0x06};
      pc_send(MGR_CMD_PROGRAM, seq++, 0, 0);
      expect_ack(MGR_CMD_PROGRAM, seq - 1, e, 1, "两槽皆空 → ERR_NO_FW"); }

    upload(&seq, g_boot, sizeof(g_boot), ver);        /* 只填 Slot0 */
    { uint8_t e[1] = {0x06};
      pc_send(MGR_CMD_PROGRAM, seq++, 0, 0);
      expect_ack(MGR_CMD_PROGRAM, seq - 1, e, 1, "仅 Slot0 有固件 → ERR_NO_FW"); }

    upload(&seq, g_app, sizeof(g_app), ver);          /* 填 Slot1 */
    (void)ok;
}

static void t9_full_program_chain(void)
{
    printf("[T9] 端到端: PC帧 → 上传 → SPI Flash → 0x0112 ISP → 被烧板 Flash\n");
    init_env();
    uint16_t seq = 0x7000;
    uint8_t  ver1[3] = {1, 2, 3};
    uint8_t  ver2[3] = {4, 5, 6};

    /* 上传两段镜像 (自动 Slot0 → Slot1) */
    upload(&seq, g_boot, sizeof(g_boot), ver1);
    upload(&seq, g_app,  sizeof(g_app),  ver2);

    /* 触发烧录并一直推进到 ACK */
    {
        uint8_t  raw[8 + 2];
        uint16_t n = Frame_Build(raw, MGR_CMD_PROGRAM, seq, 0, 0);
        tx_len = 0;
        APP_USB_Receive(raw, n);
    }
    { uint16_t rseq = 0; uint8_t rst = 0xFF;
      CHECK(wait_for_cmd(MGR_CMD_PROGRAM, &rseq, &rst) == 1, "收到 0x0112 应答");
      CHECK(rseq == seq, "0x0112 ACK 的 SEQ = 受理时的序号 (锁存生效)");
      CHECK(rst == 0x00, "0x0112 ACK 状态 = OK"); }
    seq++;

    /* 被烧板 Flash 逐字节核对 */
    CHECK(memcmp(&g_dut[DUT_BOOT_ADDR - 0x08000000u], g_boot, sizeof(g_boot)) == 0,
          "被烧板 Bootloader 区 == 源镜像");
    CHECK(memcmp(&g_dut[DUT_APP_ADDR - 0x08000000u], g_app, sizeof(g_app)) == 0,
          "被烧板 App 区 == 源镜像");
    CHECK(sim.erase_cmd_used == 0x43, "F1 目标使用 0x43 全片擦除");
    CHECK(sim.jumped == 1, "烧录完成后执行 Go 跳转");
    CHECK(sim.nack_count == 0, "ISP 全程零 NACK");

    printf("    擦除=0x%02X 写块=%u ACK=%u NACK=%u 跳转=%u\n",
           sim.erase_cmd_used, sim.write_ok, sim.ack_count, sim.nack_count, sim.jumped);
}

static void t10_program_busy_and_g0(void)
{
    printf("[T10] 烧录中 BUSY / G0 目标走 0x44\n");
    /* G0 目标: 只支持 0x44 */
    init_env();
    sim_reset(0x31, 0, 1, 0);
    uint16_t seq = 0x8000;
    uint8_t  ver[3] = {7, 7, 7};

    upload(&seq, g_boot, sizeof(g_boot), ver);
    upload(&seq, g_app,  sizeof(g_app),  ver);

    /* 触发烧录 */
    {
        uint8_t  raw[8 + 2];
        uint16_t n = Frame_Build(raw, MGR_CMD_PROGRAM, seq, 0, 0);
        tx_len = 0;
        APP_USB_Receive(raw, n);
    }
    uint16_t prog_seq = seq;
    seq++;

    /* 只推 1 步, 让状态机进入 PROGRAMMING 但烧录尚未结束 */
    APP_Run();
    APP_Run();

    /* 烧录途中插入一条命令 → 必须回 BUSY (且不能被后面的 0x0112 ACK 覆盖) */
    {
        uint8_t  raw[16];
        uint16_t n = Frame_Build(raw, MGR_CMD_QUERY_FLASH_VER, seq++, 0, 0);
        uint8_t  e[1] = {0x01};
        tx_len = 0;
        APP_USB_Receive(raw, n);
        APP_Run();                       /* 只推 1 步: 处理该帧 + ISP 前进一步 */
        expect_ack(MGR_CMD_QUERY_FLASH_VER, seq - 1, e, 1, "烧录中查版本 → BUSY");
    }

    /* 继续推进到 0x0112 完成 */
    { uint16_t rseq = 0; uint8_t rst = 0xFF;
      CHECK(wait_for_cmd(MGR_CMD_PROGRAM, &rseq, &rst) == 1, "G0: 收到 0x0112 应答");
      CHECK(rseq == prog_seq, "G0: 0x0112 ACK SEQ = 受理序号");
      CHECK(rst == 0x00, "G0: 0x0112 → OK"); }

    CHECK(sim.erase_cmd_used == 0x44, "G0 目标使用 0x44");
    CHECK(memcmp(&g_dut[DUT_BOOT_ADDR - 0x08000000u], g_boot, sizeof(g_boot)) == 0, "G0: Boot 区一致");
    CHECK(memcmp(&g_dut[DUT_APP_ADDR - 0x08000000u], g_app, sizeof(g_app)) == 0, "G0: App 区一致");
}

static void t13_flash_offline_detected(void)
{
    printf("[T13] SPI Flash 不在线/型号不对 → 第一条命令就以 ERR_FLASH 暴露\n");
    init_env();
    fl.force_wrong_id = 1u;          /* RDID 返回全 0 */
    CHECK(GD25Q_Test() == 0u, "GD25Q_Test 判定失败");

    uint16_t seq = 0x9000;
    uint8_t  d9[9] = {0, 1, 0, 0, 1, 0, 1, 0, 0};
    uint8_t  e3[1] = {0x03};

    pc_send(MGR_CMD_FW_START, seq++, d9, 9);
    expect_ack(MGR_CMD_FW_START, seq - 1, e3, 1, "0x0009 → ERR_FLASH 0x03");

    CHECK(fl.se_count == 0u && fl.pp_count == 0u, "未做任何擦除/写入");
    CHECK(fl.wel_violations == 0u, "无 WEL 违规");
}

int main(void)
{
    printf("=== FOC JIG U1 全链路宿主仿真 (PC帧→上传→SPI Flash→ISP→被烧板) ===\n\n");
    t1_flash_id();
    t2_empty_and_unknown();
    t3_upload_and_slot_switch();
    t4_start_param_errors();
    t5_overflow_and_latch();
    t6_count_mismatch();
    t7_upload_timeout();
    t8_program_precheck();
    t9_full_program_chain();
    t10_program_busy_and_g0();
    t13_flash_offline_detected();
    printf("\n=== 共 %d 项检查, 失败 %d 项 ===\n", g_checks, g_fails);
    return g_fails == 0 ? 0 : 1;
}
