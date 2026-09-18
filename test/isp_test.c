/*
 * ISP 烧录流程字节级验证 —— 宿主侧用「模拟 STM32 系统 bootloader」驱动固件真源码
 *
 * 做法: 直接编译 U1/Core/Src/isp_programmer.c 原文件 (不改一行),
 *       在宿主侧提供 BSP_ISP_UART_* / GD25Q_ReadBuffer / HAL_* 的替身,
 *       替身把固件发出的字节喂给一个 AN3155 bootloader 仿真器,
 *       仿真器严格校验命令码补码、地址 XOR 校验、写入长度 N-1 与数据 XOR 校验,
 *       并把写进「被烧板 Flash」的内容与源镜像逐字节比对。
 *
 * 覆盖场景:
 *   T1 F1 目标 (BL v2.2)     → 必须用 0x43 全片擦除, 不碰 0x44
 *   T2 G0 目标 (BL v3.1)     → 必须用 0x44
 *   T3 F1 但 0x43 被 NACK    → 必须回退到 0x44
 *   T4 目标完全无响应         → 必须报 ERR_PROG, 且不得继续写 Flash
 *   T5 镜像超分区上限         → ISP_Step 前置拒绝, 不擦不写
 *   T6 尾块对齐补齐           → 镜像尾部之外的字节必须是 0xFF, 镜像本身逐字节一致
 *   T7 校验和错误必须被目标拒绝 (仿真器侧验证固件算得对)
 */
#include <stdio.h>
#include <string.h>
#include <stdint.h>

#include "isp_programmer.h"
#include "isp_config.h"
#include "flash_map.h"
#include "bsp_uart.h"
#include "gd25q128.h"
#include "main.h"

/*==============================================================================
 * 虚拟时间 + 引脚/串口波形记录
 *   虚拟时间每次调用 +1, 保证 ISP_WaitAck 的超时循环一定收敛。
 *   同时把 GPIO 写与串口校验位切换记进日志, 用于验证:
 *     - ISP 期间串口是 8E1, 结束后(成功或失败)必须切回 8N1
 *     - BOOT0/RESET 的时序波形符合"参考测径治具"的做法
 *============================================================================*/
static uint32_t g_tick = 0;
uint32_t HAL_GetTick(void) { return g_tick++; }
void     HAL_Delay(uint32_t ms) { g_tick += ms; }

/* GPIO 写日志 */
typedef struct { void *port; uint16_t pin; uint8_t level; uint32_t tick; } GpioEvt_t;
static GpioEvt_t g_gpio[64];
static int       g_gpio_n = 0;
static uint32_t  g_gpio_init_cnt = 0;

void HAL_GPIO_Init(GPIO_TypeDef *port, GPIO_InitTypeDef *init)
{
    (void)port; (void)init;
    g_gpio_init_cnt++;
}

void HAL_GPIO_WritePin(GPIO_TypeDef *port, uint16_t pin, GPIO_PinState state)
{
    if (g_gpio_n < (int)(sizeof(g_gpio) / sizeof(g_gpio[0]))) {
        g_gpio[g_gpio_n].port  = port;
        g_gpio[g_gpio_n].pin   = pin;
        g_gpio[g_gpio_n].level = (state == GPIO_PIN_SET) ? 1u : 0u;
        g_gpio[g_gpio_n].tick  = g_tick;
        g_gpio_n++;
    }
}

/* 串口校验位切换日志 (1 = 8E1 bootloader, 0 = 8N1 运行态) */
static uint8_t  g_parity_log[32];
static int      g_parity_n = 0;
static uint8_t  g_parity_last = 0xFFu;

static void host_reset_logs(void)
{
    g_gpio_n = 0; g_gpio_init_cnt = 0; g_parity_n = 0; g_parity_last = 0xFFu;
}

/*==============================================================================
 * 被烧板 Flash 仿真 (64KB, 索引 = 地址 - 0x08000000)
 *============================================================================*/
#define DUT_FLASH_SIZE  (64u * 1024u)
static uint8_t g_dut[DUT_FLASH_SIZE];

/*==============================================================================
 * 治具侧 SPI Flash 仿真 (只用到前 2 个 64KB 槽 → 128KB)
 *============================================================================*/
#define JIG_FLASH_SIZE  (128u * 1024u)
static uint8_t g_spi[JIG_FLASH_SIZE];

void GD25Q_ReadBuffer(uint8_t *buf, uint32_t addr, uint16_t len)
{
    if (addr + len > JIG_FLASH_SIZE) { memset(buf, 0xFF, len); return; }
    memcpy(buf, &g_spi[addr], len);
}

/*==============================================================================
 * RCC 替身实例 (见 hostshim/stm32f1xx_hal.h)
 *============================================================================*/
RCC_TypeDef g_host_rcc;

/*==============================================================================
 * AN3155 bootloader 仿真器
 *============================================================================*/
typedef enum { SS_CMD = 0, SS_CMDCK, SS_ADDR, SS_ERASE, SS_WN, SS_WDATA, SS_RDN } SS_t;

static struct {
    uint8_t  bl_version;
    uint16_t pid;
    uint8_t  support_0x43;
    uint8_t  support_0x44;
    uint8_t  support_read;       /* 1 = 支持 Read Memory 0x11 */
    uint8_t  align_bytes;        /* 0=不检查; 4=F1 规则; 8=G0 规则 (按最严校验) */
    uint8_t  silent;             /* 1 = 完全不响应 (模拟目标没进 bootloader) */

    SS_t     st;
    uint8_t  cur_cmd;
    uint8_t  idx;
    uint8_t  buf[8];
    uint32_t addr;
    uint16_t wlen, wgot;
    uint8_t  wcksum;
    uint8_t  wbuf[256];

    /* 统计与观测 */
    uint8_t  erase_cmd_used;
    uint32_t write_count;        /* 收到的写块尝试次数 */
    uint32_t write_ok_count;     /* 真正写成功的块数 */
    uint32_t fail_write_at;      /* 第几次写块时故意 NACK (0=从不) */
    uint32_t corrupt_write_n;    /* 第几次写块后偷偷改坏目标 Flash (0=从不) */
    uint32_t align_violations;   /* 长度/地址未 4 字节对齐的次数 */
    uint32_t read_count;         /* 收到 Read Memory 次数 */
    uint32_t nack_count;
    uint32_t ack_count;
    uint8_t  jumped;
    uint32_t mass_erase_count;
} sim;

/* 设备→主机 响应队列 */
static uint8_t  rxq[512];
static uint16_t rxq_h = 0, rxq_t = 0;

static void q_push(uint8_t b) { if (((rxq_h + 1) & 511) != rxq_t) { rxq[rxq_h] = b; rxq_h = (rxq_h + 1) & 511; } }
static int  q_pop(uint8_t *b) { if (rxq_t == rxq_h) return 0; *b = rxq[rxq_t]; rxq_t = (rxq_t + 1) & 511; return 1; }

static void sim_ack(void)  { q_push(0x79); sim.ack_count++; }
static void sim_nack(void) { q_push(0x1F); sim.nack_count++; }

static void sim_reset(uint8_t ver, uint8_t sup43, uint8_t sup44, uint8_t silent)
{
    memset(&sim, 0, sizeof(sim));
    memset(g_dut, 0x00, sizeof(g_dut));
    rxq_h = rxq_t = 0;
    sim.bl_version = ver;
    sim.pid        = 0x0410;
    sim.support_0x43 = sup43;
    sim.support_0x44 = sup44;
    sim.support_read = 1u;
    sim.align_bytes = 8u;         /* 默认按最严的 8 字节对齐校验 */
    sim.silent     = silent;
    sim.st         = SS_CMD;
}

/* 把一个来自治具的字节喂给仿真 bootloader */
static void sim_feed(uint8_t b)
{
    if (sim.silent) return;
    if (sim.jumped) return;              /* 已跳转运行, 不再响应 */

    switch (sim.st) {

    case SS_CMD:
        if (b == 0x7Fu) { sim_ack(); return; }        /* 同步字节 */
        sim.cur_cmd = b;
        sim.st = SS_CMDCK;
        return;

    case SS_CMDCK:
        if (b != (uint8_t)(sim.cur_cmd ^ 0xFFu)) { sim_nack(); sim.st = SS_CMD; return; }
        switch (sim.cur_cmd) {
        case 0x01:                                   /* Get Version */
            sim_ack();
            q_push(0x01);                            /* N = 字节数-1 */
            q_push(sim.bl_version);
            q_push(0x00);
            sim_ack();
            sim.st = SS_CMD;
            return;
        case 0x02:                                   /* Get ID */
            sim_ack();
            q_push(0x01);
            q_push((uint8_t)(sim.pid >> 8));
            q_push((uint8_t)(sim.pid & 0xFF));
            sim_ack();
            sim.st = SS_CMD;
            return;
        case 0x31:                                   /* Write Memory */
            sim_ack(); sim.idx = 0; sim.st = SS_ADDR; return;
        case 0x11:                                   /* Read Memory */
            if (!sim.support_read) { sim_nack(); sim.st = SS_CMD; return; }
            sim_ack(); sim.idx = 0; sim.st = SS_ADDR; return;
        case 0x21:                                   /* Go */
            sim_ack(); sim.idx = 0; sim.st = SS_ADDR; return;
        case 0x43:
            if (!sim.support_0x43) { sim_nack(); sim.st = SS_CMD; return; }
            sim_ack(); sim.idx = 0; sim.buf[0] = 0; sim.st = SS_ERASE; return;
        case 0x44:
            if (!sim.support_0x44) { sim_nack(); sim.st = SS_CMD; return; }
            sim_ack(); sim.idx = 0; sim.buf[0] = 0; sim.st = SS_ERASE; return;
        default:
            sim_nack(); sim.st = SS_CMD; return;
        }

    case SS_ADDR:                                    /* 4B 地址 + XOR */
        if (sim.idx >= (uint8_t)sizeof(sim.buf)) { sim.st = SS_CMD; return; }
        sim.buf[sim.idx++] = b;
        if (sim.idx == 5) {
            uint8_t x = (uint8_t)(sim.buf[0] ^ sim.buf[1] ^ sim.buf[2] ^ sim.buf[3]);
            if (x != sim.buf[4]) { sim_nack(); sim.st = SS_CMD; return; }
            sim.addr = ((uint32_t)sim.buf[0] << 24) | ((uint32_t)sim.buf[1] << 16)
                     | ((uint32_t)sim.buf[2] << 8)  |  (uint32_t)sim.buf[3];
            sim_ack();
            if (sim.cur_cmd == 0x21)      { sim.jumped = 1; sim.st = SS_CMD; }
            else if (sim.cur_cmd == 0x11) { sim.idx = 0; sim.st = SS_RDN; }  /* 等 (N-1)+补码 */
            else                          { sim.st = SS_WN; }
        }
        return;

    case SS_ERASE:                                   /* 2B 页数 + 校验 */
        sim.buf[sim.idx++] = b;
        if (sim.idx == 3) {
            uint8_t x = (uint8_t)(sim.buf[0] ^ sim.buf[1]);
            if (x != sim.buf[2]) { sim_nack(); sim.st = SS_CMD; return; }
            if (sim.buf[0] == 0xFFu && sim.buf[1] == 0xFFu) {   /* 全片擦除 */
                memset(g_dut, 0xFF, sizeof(g_dut));
                sim.mass_erase_count++;
                sim.erase_cmd_used = sim.cur_cmd;
                sim_ack();
            } else {
                /* 本测试只发送全片擦除; 逐页形式一律 NACK, 以便暴露偏差 */
                sim_nack();
            }
            sim.st = SS_CMD;
        }
        return;

    case SS_WN:                                      /* 长度 N-1 */
        sim.wlen   = (uint16_t)b + 1u;
        sim.wgot   = 0;
        sim.wcksum = b;                              /* 校验和初值 = N-1 */
        sim.st     = SS_WDATA;
        return;

    case SS_WDATA:
        if (sim.wgot < sim.wlen) {
            sim.wbuf[sim.wgot++] = b;
            sim.wcksum ^= b;
            return;
        }
        /* 最后一字节: 数据校验和 */
        if (b == sim.wcksum) {
            sim.write_count++;
            /* AN3155: 写入长度与地址必须按器件族对齐 (F1=4, G0=8) */
            if (sim.align_bytes != 0u &&
                (((sim.wlen % sim.align_bytes) != 0u) || ((sim.addr % sim.align_bytes) != 0u))) {
                sim.align_violations++;
                sim_nack();
            } else if (sim.fail_write_at != 0u && sim.write_count == sim.fail_write_at) {
                sim_nack();                       /* 模拟目标写入失败 */
            } else {
                uint32_t off = sim.addr - 0x08000000u;
                if (off + sim.wlen <= DUT_FLASH_SIZE) memcpy(&g_dut[off], sim.wbuf, sim.wlen);
                sim.write_ok_count++;
                /* 模拟"写进去了但存储单元坏了": ACK 正常, 内容却不对 */
                if (sim.corrupt_write_n != 0u && sim.write_ok_count == sim.corrupt_write_n)
                    g_dut[off] ^= 0xFFu;
                sim_ack();
            }
        } else {
            sim_nack();
        }
        sim.st = SS_CMD;
        return;

    case SS_RDN:                                     /* Read Memory: (N-1) + 补码 */
        if (sim.idx >= (uint8_t)sizeof(sim.buf)) { sim.st = SS_CMD; return; }
        sim.buf[sim.idx++] = b;
        if (sim.idx == 2) {
            uint8_t n1 = sim.buf[0];
            if ((uint8_t)(n1 ^ 0xFFu) != sim.buf[1]) { sim_nack(); sim.st = SS_CMD; return; }
            {
                uint16_t n   = (uint16_t)n1 + 1u;
                uint32_t off = sim.addr - 0x08000000u;
                sim_ack();                       /* bootloader 对 (N-1)+校验 回 ACK */
                for (uint16_t i = 0; i < n; i++)
                    q_push((off + i < DUT_FLASH_SIZE) ? g_dut[off + i] : 0xFFu);
                sim_ack();                       /* 尾随 ACK */
                sim.read_count++;
            }
            sim.st = SS_CMD;
        }
        return;

    default:
        sim.st = SS_CMD;
        return;
    }
}

/*==============================================================================
 * BSP UART 替身 (ISP 口)
 *============================================================================*/
static uint8_t g_parity = 0;

void BSP_ISP_UART_SetParity(uint8_t even_parity)
{
    g_parity = even_parity;
    g_parity_last = even_parity;
    if (g_parity_n < (int)sizeof(g_parity_log)) g_parity_log[g_parity_n++] = even_parity;
}
void BSP_ISP_UART_FlushRx(void) { /* 仿真器无残留接收侧 */ }
void BSP_ISP_UART_SendByte(uint8_t b) { (void)g_parity; sim_feed(b); }
void BSP_ISP_UART_Send(const uint8_t *d, uint16_t n) { for (uint16_t i = 0; i < n; i++) sim_feed(d[i]); }

uint8_t BSP_ISP_UART_RecvByte(uint8_t *out, uint32_t timeout_ms)
{
    (void)timeout_ms;
    return q_pop(out) ? 1u : 0u;
}

void BSP_DBG_Send(const char *s) { (void)s; }
void BSP_DBG_SendHexByte(uint8_t b) { (void)b; }

/*==============================================================================
 * 测试框架
 *============================================================================*/
static int g_checks = 0, g_fails = 0;
#define CHECK(c, m) do { g_checks++; if (!(c)) { printf("  FAIL: %s (line %d)\n", m, __LINE__); g_fails++; } } while (0)

/* 往治具 SPI Flash 写一个槽: 8B 头部 + 数据 */
static void put_slot(uint32_t base, const uint8_t *img, uint32_t len, uint8_t ver[3], uint8_t flag)
{
    uint8_t hdr[FW_HEAD_SIZE];
    hdr[0] = (uint8_t)(len);
    hdr[1] = (uint8_t)(len >> 8);
    hdr[2] = (uint8_t)(len >> 16);
    hdr[3] = (uint8_t)(len >> 24);
    hdr[4] = ver[0]; hdr[5] = ver[1]; hdr[6] = ver[2];
    hdr[7] = flag;
    memcpy(&g_spi[base], hdr, FW_HEAD_SIZE);
    memcpy(&g_spi[base + FW_HEAD_SIZE], img, len);
}

static void run_isp(void)
{
    ISP_Start();
    uint32_t guard = 0;
    while (ISP_Busy() && guard < 200000u) { ISP_Step(); guard++; }
}

/* 检查被烧板 Flash 区间内容 */
static int dut_matches(uint32_t addr, const uint8_t *expect, uint32_t len)
{
    return memcmp(&g_dut[addr - 0x08000000u], expect, len) == 0;
}
static int dut_is_ff(uint32_t addr, uint32_t len)
{
    for (uint32_t i = 0; i < len; i++) if (g_dut[addr - 0x08000000u + i] != 0xFFu) return 0;
    return 1;
}

/*==============================================================================
 * 用例
 *============================================================================*/
static uint8_t g_boot[101];    /* 101B: 触发尾块补齐 (101 % 4 = 1) */
static uint8_t g_app[301];     /* 301B: 256 + 45, 45 % 4 = 1 → 补齐 */

static void fill_images(void)
{
    for (int i = 0; i < 101; i++) g_boot[i] = (uint8_t)(0x11 + i * 3);
    for (int i = 0; i < 301; i++) g_app[i]  = (uint8_t)(0xA0 + i * 7);
}

static void prepare_slots(void)
{
    uint8_t v1[3] = {1, 2, 3};
    uint8_t v2[3] = {4, 5, 6};
    memset(g_spi, 0xFF, sizeof(g_spi));
    put_slot(FW_SLOT0_BASE, g_boot, sizeof(g_boot), v1, FW_FLAG_COMPLETE);
    put_slot(FW_SLOT1_BASE, g_app,  sizeof(g_app),  v2, FW_FLAG_COMPLETE);
}

static void t1_f1_uses_0x43(void)
{
    printf("[T1] F1 目标 (BL v0x22, 仅支持 0x43)\n");
    prepare_slots();
    sim_reset(0x22, 1, 1, 0);

    run_isp();

    CHECK(ISP_Result() == APP_STATUS_OK, "烧录结果 = OK");
    CHECK(sim.erase_cmd_used == 0x43, "使用 0x43 全片擦除");
    CHECK(sim.mass_erase_count == 1, "只擦除一次");
    CHECK(sim.jumped == 1, "最后执行了 Go 跳转");
    CHECK(dut_matches(DUT_BOOT_ADDR, g_boot, sizeof(g_boot)), "Bootloader 区逐字节一致");
    CHECK(dut_matches(DUT_APP_ADDR,  g_app,  sizeof(g_app)),  "App 区逐字节一致");
    CHECK(sim.nack_count == 0, "全程无 NACK (说明校验和/长度/XOR 全部正确)");
    printf("    擦除命令=0x%02X 写块数=%u ACK=%u NACK=%u\n",
           sim.erase_cmd_used, sim.write_count, sim.ack_count, sim.nack_count);
}

static void t2_g0_uses_0x44(void)
{
    printf("[T2] G0 目标 (BL v0x31, 仅支持 0x44)\n");
    prepare_slots();
    sim_reset(0x31, 0, 1, 0);

    run_isp();

    CHECK(ISP_Result() == APP_STATUS_OK, "烧录结果 = OK");
    CHECK(sim.erase_cmd_used == 0x44, "使用 0x44 扩展全片擦除");
    CHECK(sim.jumped == 1, "最后执行了 Go 跳转");
    CHECK(dut_matches(DUT_BOOT_ADDR, g_boot, sizeof(g_boot)), "Bootloader 区逐字节一致");
    CHECK(dut_matches(DUT_APP_ADDR,  g_app,  sizeof(g_app)),  "App 区逐字节一致");
    printf("    擦除命令=0x%02X 写块数=%u NACK=%u\n", sim.erase_cmd_used, sim.write_count, sim.nack_count);
}

static void t3_fallback_to_0x44(void)
{
    printf("[T3] F1 目标但 0x43 被 NACK → 必须回退 0x44\n");
    prepare_slots();
    sim_reset(0x22, 0, 1, 0);        /* 0x43 不支持, 0x44 支持 */

    run_isp();

    CHECK(ISP_Result() == APP_STATUS_OK, "烧录结果 = OK");
    CHECK(sim.erase_cmd_used == 0x44, "回退到 0x44");
    CHECK(dut_matches(DUT_BOOT_ADDR, g_boot, sizeof(g_boot)), "Bootloader 区一致");
    CHECK(dut_matches(DUT_APP_ADDR,  g_app,  sizeof(g_app)),  "App 区一致");
    printf("    擦除命令=0x%02X 写块数=%u NACK=%u\n", sim.erase_cmd_used, sim.write_count, sim.nack_count);
}

static void t4_silent_target(void)
{
    printf("[T4] 目标完全无响应\n");
    prepare_slots();
    sim_reset(0x22, 1, 1, 1);        /* silent */

    run_isp();

    CHECK(ISP_Result() == APP_STATUS_ERR_PROG, "结果 = ERR_PROG (0x04)");
    CHECK(sim.write_count == 0, "未写入任何数据");
    CHECK(sim.mass_erase_count == 0, "未执行擦除");
}

static void t5_oversize_rejected(void)
{
    printf("[T5] 镜像超过目标分区上限 → 拒绝且不擦不写\n");
    prepare_slots();
    /* 把 Slot0 头部改成 > DUT_BOOT_SIZE */
    uint32_t big = DUT_BOOT_SIZE + 1u;
    g_spi[FW_SLOT0_BASE + 0] = (uint8_t)(big);
    g_spi[FW_SLOT0_BASE + 1] = (uint8_t)(big >> 8);
    g_spi[FW_SLOT0_BASE + 2] = (uint8_t)(big >> 16);
    g_spi[FW_SLOT0_BASE + 3] = (uint8_t)(big >> 24);
    sim_reset(0x22, 1, 1, 0);

    run_isp();

    CHECK(sim.mass_erase_count == 0, "未擦除被烧板 (边界不合格应在擦除前拒绝)");
    CHECK(sim.write_count == 0, "未写入被烧板");
    CHECK(ISP_Result() != APP_STATUS_OK, "结果非 OK");
}

static void t6_tail_padding(void)
{
    printf("[T6] 尾块对齐补齐: 镜像外必须 0xFF\n");
    prepare_slots();
    sim_reset(0x22, 1, 1, 0);

    run_isp();

    CHECK(ISP_Result() == APP_STATUS_OK, "结果 = OK");
    /* Boot 101B → 补齐到 104B; 101..103 应为 0xFF */
    CHECK(dut_is_ff(DUT_BOOT_ADDR + 101u, 3u), "Bootloader 尾部补齐为 0xFF");
    /* App 301B → 256 + 45 → 补齐到 304; 301..303 应为 0xFF */
    CHECK(dut_is_ff(DUT_APP_ADDR + 301u, 3u), "App 尾部补齐为 0xFF");
    /* 且不能越过分区写入 */
    CHECK(dut_is_ff(DUT_APP_ADDR + 304u, 16u), "补齐未越界继续写");
}

static void t7_checksum_enforced(void)
{
    printf("[T7] 仿真器严格校验 → 证明固件算出的校验和正确\n");
    prepare_slots();
    sim_reset(0x22, 1, 1, 0);

    run_isp();

    /* 仿真器对命令补码/地址XOR/长度XOR/数据校验和全部做了校验,
     * 任何一处算错都会 NACK 并使流程失败。这里断言"零 NACK"。 */
    CHECK(sim.nack_count == 0, "零 NACK");
    CHECK(sim.ack_count > 0, "有正常 ACK");
    printf("    总共 ACK=%u NACK=%u\n", sim.ack_count, sim.nack_count);
}

static void t7b_alignment_enforced(void)
{
    uint8_t  e_ok[1] = {0};
    (void)e_ok;

    printf("[T7b] AN3155 对齐: 按 F1(4 字节) 与 G0(8 字节) 两套最严规则各跑一遍\n");

    /* F1 规则: 4 字节对齐 */
    prepare_slots();                       /* 101B / 301B → 尾块需要补齐 */
    sim_reset(0x22, 1, 1, 0);
    sim.align_bytes = 4u;
    run_isp();
    CHECK(ISP_Result() == APP_STATUS_OK, "F1(4B) 规则下结果 = OK");
    CHECK(sim.align_violations == 0u, "F1(4B) 规则下零对齐违规");
    CHECK(dut_matches(DUT_BOOT_ADDR, g_boot, sizeof(g_boot)), "F1: Bootloader 区一致");
    CHECK(dut_matches(DUT_APP_ADDR,  g_app,  sizeof(g_app)),  "F1: App 区一致");

    /* G0 规则: 8 字节对齐 (更严, 8 是 4 的倍数 → 同一份固件应同时满足) */
    prepare_slots();
    sim_reset(0x31, 0, 1, 0);
    sim.align_bytes = 8u;
    run_isp();
    CHECK(ISP_Result() == APP_STATUS_OK, "G0(8B) 规则下结果 = OK");
    CHECK(sim.align_violations == 0u, "G0(8B) 规则下零对齐违规");
    CHECK(dut_matches(DUT_BOOT_ADDR, g_boot, sizeof(g_boot)), "G0: Bootloader 区一致");
    CHECK(dut_matches(DUT_APP_ADDR,  g_app,  sizeof(g_app)),  "G0: App 区一致");
    printf("    统一 8 字节补齐 → 两套规则都零违规 (写块=%u)\n", sim.write_ok_count);
}

static void t8_target_rejects_write(void)
{
    printf("[T8] 目标在第 2 个写块 NACK → 必须立刻停手并报 ERR_PROG\n");
    prepare_slots();
    sim_reset(0x22, 1, 1, 0);
    sim.fail_write_at = 2u;

    run_isp();

    CHECK(ISP_Result() == APP_STATUS_ERR_PROG, "结果 = ERR_PROG");
    CHECK(sim.write_count == 2u, "只尝试了 2 次写块就停下");
    CHECK(sim.write_ok_count == 1u, "只成功写入 1 块");
    CHECK(sim.jumped == 0, "失败后不执行 Go 跳转");
    printf("    写块尝试=%u 成功=%u\n", sim.write_count, sim.write_ok_count);
}

static void t9_incomplete_image_no_erase(void)
{
    printf("[T9] Slot1 完整标志缺失 → 必须在擦除之前中止\n");
    prepare_slots();
    g_spi[FW_SLOT1_BASE + FW_HEAD_OFF_FLAG] = 0xFFu;   /* 完整标志没写 */
    sim_reset(0x22, 1, 1, 0);

    run_isp();

    CHECK(sim.mass_erase_count == 0, "未擦除被烧板");
    CHECK(sim.write_count == 0, "未写入被烧板");
    CHECK(ISP_Result() == APP_STATUS_ERR_PROG, "结果 = ERR_PROG");
}

static void t10_empty_slot_no_erase(void)
{
    printf("[T10] Slot0 为空 (size=0) → 必须在擦除之前中止\n");
    prepare_slots();
    memset(&g_spi[FW_SLOT0_BASE], 0x00, FW_HEAD_SIZE);  /* size=0 */
    sim_reset(0x22, 1, 1, 0);

    run_isp();

    CHECK(sim.mass_erase_count == 0, "未擦除被烧板");
    CHECK(sim.write_count == 0, "未写入被烧板");
    CHECK(ISP_Result() == APP_STATUS_ERR_PROG, "结果 = ERR_PROG");
}

/*==============================================================================
 * ISP 入口/出口契约: 串口校验位 + BOOT0/RESET 波形
 *============================================================================*/

#if (ISP_CTRL_PINS_CONFIRMED)
/* 在日志里找某引脚最后一次置的电平; 返回 -1 表示没写过 */
static int gpio_last(GPIO_TypeDef *port, uint16_t pin)
{
    for (int i = g_gpio_n - 1; i >= 0; i--) {
        if (g_gpio[i].port == port && g_gpio[i].pin == pin) return (int)g_gpio[i].level;
    }
    return -1;
}
#endif

static void t14_uart_parity_contract(void)
{
    printf("[T14] ISP 串口校验位契约: 进 ISP 切 8E1, 出 ISP (成功或失败) 必须回 8N1\n");

    /* 成功路径: 全程最后一次 SetParity 必须是 0 (8N1) */
    prepare_slots();
    sim_reset(0x22, 1, 1, 0);
    host_reset_logs();
    run_isp();

    CHECK(ISP_Result() == APP_STATUS_OK, "成功路径结果 = OK");
    CHECK(g_parity_n >= 2, "至少切过两次校验位 (进 8E1 / 出 8N1)");
    CHECK(g_parity_n >= 1 && g_parity_log[g_parity_n - 1] == 0u, "结束在校验位 = 0 (8N1 运行态)");
    {
        int saw_even = 0;
        for (int i = 0; i < g_parity_n; i++) if (g_parity_log[i] == 1u) saw_even = 1;
        CHECK(saw_even == 1, "过程中确实切到过 8E1");
    }

    /* 失败路径: 同样必须恢复到 8N1 —— 否则"第一次失败之后全都不对" */
    prepare_slots();
    sim_reset(0x22, 1, 1, 1);          /* silent: 必然失败 */
    host_reset_logs();
    run_isp();

    CHECK(ISP_Result() == APP_STATUS_ERR_PROG, "失败路径结果 = ERR_PROG");
    CHECK(g_parity_last == 0u, "失败后校验位已回 0 (8N1 运行态)");
    printf("    成功: 切换序列 n=%d 末值=%u ; 失败: 末值=%u\n",
           g_parity_n, (unsigned)g_parity_last, (unsigned)g_parity_last);
}

#if (ISP_CTRL_PINS_CONFIRMED)
static void t15_boot0_reset_waveform(void)
{
    printf("[T15] BOOT0/RESET 波形 (ISP_CTRL_PINS_CONFIRMED=1, 参考测径治具的做法)\n");

    prepare_slots();
    sim_reset(0x22, 1, 1, 0);
    host_reset_logs();
    run_isp();

    CHECK(ISP_Result() == APP_STATUS_OK, "结果 = OK");
    CHECK(g_gpio_init_cnt >= 2u, "两个控制脚都被初始化过 (BOOT0 + RESET)");

    /* ISP_GpioInit() 会先把两个脚写成空闲电平 (BOOT0=0, RESET=1),
     * 之后 ISP_ST_CONNECT() 才做进 bootloader 时序。所以这里不假设下标,
     * 而是定位"BOOT0 第一次被拉高"那一步, 再检查它后面的 RESET 波形。 */
    CHECK(g_gpio_n >= 2, "至少有 2 次控制脚写入");
    if (g_gpio_n >= 2) {
        CHECK(g_gpio[0].port == ISP_BOOT0_PORT && g_gpio[0].pin == ISP_BOOT0_PIN
              && g_gpio[0].level == 0u, "初始化: BOOT0 先置于低电平 (空闲态)");
        CHECK(g_gpio[1].port == ISP_RESET_PORT && g_gpio[1].pin == ISP_RESET_PIN
              && g_gpio[1].level == 1u, "初始化: RESET 先置于高电平 (空闲态)");
    }

    int i_boot_hi = -1;
    for (int i = 0; i < g_gpio_n; i++) {
        if (g_gpio[i].port == ISP_BOOT0_PORT && g_gpio[i].pin == ISP_BOOT0_PIN
            && g_gpio[i].level == 1u) { i_boot_hi = i; break; }
    }
    CHECK(i_boot_hi >= 0, "第 1 步: BOOT0 被拉高");

    if (i_boot_hi >= 0) {
        int i_lo = -1, i_hi = -1;
        for (int i = i_boot_hi + 1; i < g_gpio_n; i++) {
            if (g_gpio[i].port != ISP_RESET_PORT || g_gpio[i].pin != ISP_RESET_PIN) continue;
            if (g_gpio[i].level == 0u && i_lo < 0) { i_lo = i; }
            else if (g_gpio[i].level == 1u && i_lo >= 0 && i_hi < 0) { i_hi = i; }
        }
        CHECK(i_lo == i_boot_hi + 1, "第 2 步: BOOT0 拉高后紧接着 RESET 拉低");
        CHECK(i_hi > i_lo, "第 3 步: RESET 拉高 (释放复位)");
        if (i_lo >= 0 && i_hi > i_lo) {
            CHECK((g_gpio[i_hi].tick - g_gpio[i_lo].tick) >= ISP_RESET_LOW_MS,
                  "RESET 拉低维持 >= 10ms (与测径治具一致)");
        }
    }

    /* Go 之后 BOOT0 必须回到低电平 */
    CHECK(gpio_last(ISP_BOOT0_PORT, ISP_BOOT0_PIN) == 0, "烧录成功后 BOOT0 已拉回低电平");

    /* 失败路径也要把 BOOT0 放开 */
    prepare_slots();
    sim_reset(0x22, 1, 1, 1);
    host_reset_logs();
    run_isp();
    CHECK(ISP_Result() == APP_STATUS_ERR_PROG, "失败路径结果 = ERR_PROG");
    CHECK(gpio_last(ISP_BOOT0_PORT, ISP_BOOT0_PIN) == 0, "失败后 BOOT0 已拉回低电平");

    printf("    GPIO 写入次数=%d 初始化次数=%u\n", g_gpio_n, g_gpio_init_cnt);
}
#endif

#if (ISP_CTRL_PINS_CONFIRMED)
static void t16_gpio_clock_enabled(void)
{
    printf("[T16] 控制脚所用 GPIO 端口的时钟必须被使能\n");

    g_host_rcc.APB2ENR = 0u;
    ISP_GpioInit();

    /* BOOT0 = PB1 → GPIOB; RESET = PC13 → GPIOC。GPIOC 在 MX_GPIO_Init() 里没开,
     * 必须由 ISP_GpioInit() 自己开, 否则对 PC13 的写是静默无效的。 */
    CHECK((g_host_rcc.APB2ENR & RCC_HOST_IOPBEN) != 0u, "GPIOB 时钟已使能 (BOOT0)");
    CHECK((g_host_rcc.APB2ENR & RCC_HOST_IOPCEN) != 0u, "GPIOC 时钟已使能 (RESET)");
    CHECK((g_host_rcc.APB2ENR & RCC_HOST_IOPAEN) != 0u, "GPIOA 时钟已使能");
    printf("    APB2ENR = 0x%08X\n", (unsigned)g_host_rcc.APB2ENR);
}
#endif

#if (ISP_VERIFY_ENABLED)
static void t11_verify_catches_corruption(void)
{
    printf("[T11] 回读校验: 目标存坏 1 字节 → 必须报 ERR_PROG\n");
    prepare_slots();
    sim_reset(0x22, 1, 1, 0);
    sim.corrupt_write_n = 1u;          /* 第 1 块写"成功"但内容被改坏 */

    run_isp();

    CHECK(ISP_Result() == APP_STATUS_ERR_PROG, "回读发现不一致 → ERR_PROG");
    CHECK(sim.read_count > 0u, "确实发起了回读 (0x11)");
    CHECK(sim.jumped == 0, "校验失败 → 不执行 Go 跳转");
    printf("    回读次数=%u\n", sim.read_count);
}

static void t12_verify_passes_on_good_flash(void)
{
    printf("[T12] 回读校验: 正常烧录 → 校验通过且逐字节一致\n");
    prepare_slots();
    sim_reset(0x22, 1, 1, 0);

    run_isp();

    CHECK(ISP_Result() == APP_STATUS_OK, "结果 = OK");
    CHECK(sim.read_count > 0u, "发起了回读");
    CHECK(sim.jumped == 1, "校验通过后执行 Go");
    CHECK(dut_matches(DUT_BOOT_ADDR, g_boot, sizeof(g_boot)), "Boot 区一致");
    CHECK(dut_matches(DUT_APP_ADDR,  g_app,  sizeof(g_app)),  "App 区一致");
    /* 101B → 1 块; 301B → 2 块 ⇒ 至少 3 次回读 (每段按 256B 分块) */
    printf("    回读次数=%u 写块=%u\n", sim.read_count, sim.write_ok_count);
    CHECK(sim.read_count >= 3u, "回读覆盖了全部 3 个写块");
}
#endif

int main(void)
{
    printf("=== FOC JIG U1 ISP 流程字节级验证 (模拟 STM32 bootloader) ===\n");
#if (ISP_VERIFY_ENABLED)
    printf("    编译选项: ISP_VERIFY_ENABLED=1 (开启烧录后回读校验)\n\n");
#else
    printf("    编译选项: ISP_VERIFY_ENABLED=0 (默认, 无回读)\n\n");
#endif
    fill_images();
    t1_f1_uses_0x43();
    t2_g0_uses_0x44();
    t3_fallback_to_0x44();
    t4_silent_target();
    t5_oversize_rejected();
    t6_tail_padding();
    t7_checksum_enforced();
    t7b_alignment_enforced();
    t8_target_rejects_write();
    t9_incomplete_image_no_erase();
    t10_empty_slot_no_erase();
    t14_uart_parity_contract();
#if (ISP_CTRL_PINS_CONFIRMED)
    t15_boot0_reset_waveform();
    t16_gpio_clock_enabled();
#endif
#if (ISP_VERIFY_ENABLED)
    t11_verify_catches_corruption();
    t12_verify_passes_on_good_flash();
#endif
    printf("\n=== 共 %d 项检查, 失败 %d 项 ===\n", g_checks, g_fails);
    return g_fails == 0 ? 0 : 1;
}
