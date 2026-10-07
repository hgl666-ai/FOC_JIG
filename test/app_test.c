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
#include "swd_programmer.h"
#include "swd_config.h"
#include "swd_phy.h"
#include "sim_target.h"
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

/* 串口替身：app.c 现在会初始化 U1 的两路串口并建立到 U2 的内部链路，所以要有
 * 两个句柄 (uart_cfg.c 里 extern 的就是它们) 和 HAL_UART_*_IT。
 * 本测试关心的是 PC 协议链路，串口只要"能初始化、发了不崩"：
 * 发送永不完成 → 链路保活 PING 必然超时(等于 U2 不在)，正是预期。 */
UART_HandleTypeDef huart1;
UART_HandleTypeDef huart3;

HAL_StatusTypeDef HAL_UART_Receive_IT(UART_HandleTypeDef *huart, uint8_t *pData, uint16_t Size)
{
    (void)huart; (void)pData; (void)Size;
    return HAL_OK;
}

HAL_StatusTypeDef HAL_UART_Transmit_IT(UART_HandleTypeDef *huart, uint8_t *pData, uint16_t Size)
{
    (void)huart; (void)pData; (void)Size;
    return HAL_OK;              /* 故意不回调 TxCplt：模拟"线没接/对端不在" */
}

/* GPIO 替身一肩挑两件事：
 *   1) 治具自己的 SPI Flash 片选 (PB12)
 *   2) SWD 烧录引脚 → 送进线级目标模型 sim_target，由它按 ADIv5 解码波形 */
void HAL_GPIO_Init(GPIO_TypeDef *port, GPIO_InitTypeDef *init)
{
    sim_gpio_dir(port, (uint16_t)init->Pin,
                 (init->Mode == GPIO_MODE_INPUT) ? 0u : 1u);
}

void HAL_GPIO_WritePin(GPIO_TypeDef *port, uint16_t pin, GPIO_PinState state)
{
    if (port == GPIOB && pin == GPIO_PIN_12) {          /* 治具 Flash 片选 */
        if (state == GPIO_PIN_RESET) flash_cs_low();
        else                         flash_cs_high();
    }
    sim_gpio_write(port, pin, (state == GPIO_PIN_SET) ? 1u : 0u);
}

GPIO_PinState HAL_GPIO_ReadPin(GPIO_TypeDef *port, uint16_t pin)
{
    return sim_gpio_read(port, pin) ? GPIO_PIN_SET : GPIO_PIN_RESET;
}

HAL_StatusTypeDef HAL_SPI_TransmitReceive(SPI_HandleTypeDef *hspi,
                                          uint8_t *pTxData, uint8_t *pRxData,
                                          uint16_t Size, uint32_t Timeout)
{
    (void)hspi; (void)Timeout;
    for (uint16_t i = 0; i < Size; i++) pRxData[i] = flash_xfer(pTxData[i]);
    return HAL_OK;
}

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
    sim_target_reset();
    sim_target_set_read_ta((uint8_t)SWD_TA_BEFORE_READ_DATA);
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

    { uint8_t e[4] = {0x06, 0x00, 0xFF, 0xFF};
      pc_send(MGR_CMD_QUERY_FOC_VER, 0x0202, 0, 0);
      expect_ack(MGR_CMD_QUERY_FOC_VER, 0x0202, e, 4, "0x000D 被烧板 App 区为空 → 0x06(无固件) + 00FFFF"); }

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
    printf("[T9] 端到端: PC帧 → 上传 → SPI Flash → 0x0112 SWD → 被烧板 Flash\n");
    init_env();
    uint16_t seq = 0x7000;
    uint8_t  ver1[3] = {1, 2, 3};
    uint8_t  ver2[3] = {4, 5, 6};
    uint8_t *dut = sim_target_flash();          /* 线级模型里的 F405 Flash */
    uint32_t clk0 = SWD_PhyClockCount();

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
      CHECK(rst == APP_STATUS_OK, "0x0112 ACK 状态 = OK"); }
    seq++;

    /* 被烧板 Flash 逐字节核对 */
    CHECK(memcmp(&dut[DUT_BOOT_ADDR - 0x08000000u], g_boot, sizeof(g_boot)) == 0,
          "被烧板 Bootloader 区 == 源镜像");
    CHECK(memcmp(&dut[DUT_APP_ADDR - 0x08000000u], g_app, sizeof(g_app)) == 0,
          "被烧板 App 区 == 源镜像");
    /* 线路层与协议层的不变量 */
    CHECK(sim_target_switch_seen() == 1u,   "JTAG→SWD 切换序列被目标识别");
    CHECK(sim_target_contention() == 0u,    "SWDIO 全程零总线冲突");
    CHECK(sim_target_parity_errors() == 0u, "线路层零奇偶校验错");
    CHECK(sim_target_erases() == 3u,        "按扇区擦除: Boot 2 个 + App 1 个 = 3");
    CHECK(sim_target_locked_reject() == 0u, "没有在未解锁状态下碰过目标 Flash");
    CHECK(sim_target_halted() == 0u,        "烧完目标在运行态 (SYSRESETREQ 后 S_HALT=0)");
    CHECK(SWDP_LastIdcode() == 0x2BA01477u, "SW-DP IDCODE = 0x2BA01477");
    CHECK(SWDP_LastDevId() == 0x0413u,      "目标 DEV_ID = 0x413 (F405/407)");

    printf("    IDCODE=0x%08lX DEV_ID=0x%03lX 擦扇区=%lu 编程字=%lu SWD 事务=%lu\n",
           (unsigned long)SWDP_LastIdcode(), (unsigned long)SWDP_LastDevId(),
           (unsigned long)sim_target_erases(), (unsigned long)sim_target_programs(),
           (unsigned long)sim_target_transactions());
    /* 实测时钟周期 → 上板节拍推算（SWCLK 频率由 SWD_DELAY_LOOPS 决定） */
    {
        uint32_t cycles = SWD_PhyClockCount() - clk0;
        uint32_t bytes  = (uint32_t)(sizeof(g_boot) + sizeof(g_app));
        uint32_t per_kb = (uint32_t)(((unsigned long long)cycles * 1024ull) / bytes);
        printf("    烧 %u B 用了 %lu 个 SWCLK 周期 (%lu 周期/KB)\n",
               bytes, (unsigned long)cycles, (unsigned long)per_kb);
        printf("    推算 300KB: %.1f s @1MHz   %.1f s @2MHz   %.1f s @4MHz\n",
               (double)per_kb * 300.0 / 1.0e6, (double)per_kb * 300.0 / 2.0e6,
               (double)per_kb * 300.0 / 4.0e6);
        printf("    (只算 SWD 链路时间; 真实擦除还要等 Flash: 128KB 扇区约 1.3s/个)\n");
    }
}

static void t10_busy_and_target_absent(void)
{
    printf("[T10] 烧录中 BUSY / 目标不在时干净失败\n");
    uint16_t seq = 0x8000;
    uint8_t  ver[3] = {7, 7, 7};
    uint8_t *dut = sim_target_flash();

    /* ── 10a: 烧录途中插入命令必须回 BUSY ── */
    init_env();
    upload(&seq, g_boot, sizeof(g_boot), ver);
    upload(&seq, g_app,  sizeof(g_app),  ver);
    {
        uint8_t  raw[8 + 2];
        uint16_t n = Frame_Build(raw, MGR_CMD_PROGRAM, seq, 0, 0);
        tx_len = 0;
        APP_USB_Receive(raw, n);
    }
    uint16_t prog_seq = seq;
    seq++;

    APP_Run();
    APP_Run();                        /* 已进入 PROGRAMMING, 但远未烧完 */

    {
        uint8_t  raw[16];
        uint16_t n = Frame_Build(raw, MGR_CMD_QUERY_FLASH_VER, seq++, 0, 0);
        uint8_t  e[1] = {APP_STATUS_BUSY};
        tx_len = 0;
        APP_USB_Receive(raw, n);
        APP_Run();
        expect_ack(MGR_CMD_QUERY_FLASH_VER, seq - 1, e, 1, "烧录中查版本 → BUSY");
    }

    { uint16_t rseq = 0; uint8_t rst = 0xFF;
      CHECK(wait_for_cmd(MGR_CMD_PROGRAM, &rseq, &rst) == 1, "收到 0x0112 应答");
      CHECK(rseq == prog_seq, "0x0112 ACK SEQ = 受理序号");
      CHECK(rst == APP_STATUS_OK, "0x0112 → OK"); }
    CHECK(memcmp(&dut[DUT_BOOT_ADDR - 0x08000000u], g_boot, sizeof(g_boot)) == 0, "Boot 区一致");
    CHECK(memcmp(&dut[DUT_APP_ADDR - 0x08000000u], g_app, sizeof(g_app)) == 0, "App 区一致");

    /* ── 10b: 目标不在（线悬空）→ 必须干净失败, 一个字节都不写 ── */
    init_env();
    sim_target_attach(0u);
    upload(&seq, g_boot, sizeof(g_boot), ver);
    upload(&seq, g_app,  sizeof(g_app),  ver);
    {
        uint8_t  raw[8 + 2];
        uint16_t n = Frame_Build(raw, MGR_CMD_PROGRAM, seq, 0, 0);
        tx_len = 0;
        APP_USB_Receive(raw, n);
    }
    { uint16_t rseq = 0; uint8_t rst = 0xFF;
      CHECK(wait_for_cmd(MGR_CMD_PROGRAM, &rseq, &rst) == 1, "目标不在时仍然回 ACK (不死等)");
      CHECK(rst == APP_STATUS_ERR_PROG, "目标不在 → ERR_PROG");
      CHECK(SWDP_LastSwdError() != 0u, "记录了底层 SWD 错误码供诊断"); }
    CHECK(sim_target_programs() == 0u, "目标不在时一个字节都没写进目标 Flash");
    CHECK(sim_target_erases() == 0u,   "目标不在时没有擦除任何扇区");
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
    t10_busy_and_target_absent();
    t13_flash_offline_detected();
    printf("\n=== 共 %d 项检查, 失败 %d 项 ===\n", g_checks, g_fails);
    return g_fails == 0 ? 0 : 1;
}
