/**
  ******************************************************************************
  * @file    SWD_FLASH.c
  * @brief   M1 治具 —— 目标板 SWD 烧录实现（SWDIO/SWCLK + NRST/BOOT0）
  *
  * 本文件按"层"分节，每节之间用 ==== 分隔；改哪层看哪节：
  *   §1 PHY      位/包时序（GPIO 位翻转，唯一与主机硬件相关的部分）
  *   §1.5 DUT    BOOT0(PC7)/NRST(PC6)：烧前跑 App → BOOT 高 100ms → 复位进 bootloader
  *   §2 DP       调试端口：连接/上电/SELECT/带重试的寄存器读写
  *   §3 MEM-AP   AHB-AP 内存访问（含 F1 必需的 16 位半字写）
  *   §4 Cortex-M 停核/冻结看门狗/认型号/复位运行
  *   §5 F105 Flash 页擦除 + 半字编程 + 回读校验
  *   §6 烧录状态机（非阻塞，一次一步）+ 自检
  *
  * 与"FOC 治具 U1 已成功版本"的差异（只这三类）：
  *   - F4 扇区表 → F105 的 64 个 2KB 页；
  *   - F4 32 位字编程 → F105 16 位半字编程（MEM-AP CSW 换 16 位）；
  *   - FLASH 寄存器位序 / DBGMCU 寄存器地址按 F1 改。
  * 协议层（线复位、0xE79E 切换、ACK/WAIT/FAULT 重试、posted read/write、
  * 忙等与退避）逐行照搬已验证版本，不要"顺手优化"。
  ******************************************************************************
  */
#include "flash_mode.h"

/* 全局宏选择：FLASH_MODE_UART5 = 1 时整个文件不参与编译，
 * 由 UART_FLASH.c 接管烧录；SWD 方案的代码原样保留在 #if 内。 */
#if !FLASH_MODE_UART5

#include "SWD_FLASH.h"

#include "log.h"
#include "gd25q128.h"

/* 一步写/校验多少字节（128B = 64 个半字；比 F4 版小，主循环更跟手） */
#define SWDP_CHUNK_BYTES    128u
#define SWDP_CHUNK_WORDS    (SWDP_CHUNK_BYTES / 4u)

/*==============================================================================
 * §1 PHY —— 位与包
 *----------------------------------------------------------------------------
 * 线级约定（ADIv5 IHI0031，与 DAPLink/DAPProg 逐条对齐）：
 *   主机在 SWCLK 下降沿驱动 SWDIO，目标在上升沿采样；读方向相反。
 *   请求/应答/数据一律 LSB first。
 *   请求头 8 位：Start(1) APnDP RnW A2 A3 Parity Stop(0) Park(1)。
 *============================================================================*/
#define SWD_ACK_OK          0x01u
#define SWD_ACK_WAIT        0x02u
#define SWD_ACK_FAULT       0x04u

#define SWD_TA_AFTER_HEADER     1u   /* 请求头 → ACK：必须 1 拍 */
#define SWD_TA_AFTER_ACK_WRITE  1u   /* ACK → 写数据：必须 1 拍 */
#define SWD_TA_BEFORE_READ_DATA 0u   /* ACK → 读数据：必须 0 拍 */
#define SWD_TA_TAIL             1u   /* 事务末 turnaround（读必须） */
#define SWD_TAIL_IDLE_CYCLES    0u   /* 末 turnaround 后的空闲拍（照 DAPProg 用 0） */
#define SWD_IDLE_LEVEL          1u   /* 空闲时 SWDIO 电平 */

static volatile uint16_t s_delay_loops = (uint16_t)SWD_DELAY_LOOPS;
static uint8_t  s_dio_out  = 0u;
static uint8_t  s_last_ack = 0u;
static uint8_t  s_last_req = 0u;

static void phy_clk_set(uint8_t level)
{
    if (level) {
        SWD_CLK_PORT->BSRR = (uint32_t)SWD_CLK_PIN;
    } else {
        SWD_CLK_PORT->BSRR = (uint32_t)SWD_CLK_PIN << 16;
    }
}

/* 方向切换：F1 的 CRL/CRH 每脚 4 bit，MODE[1:0] 在低两位。
 *   输出推挽 50MHz = 0b0011
 *   输入上拉      = CNF=10, MODE=00 = 0b1000（配合 ODR=1，由 BSRR 预置） */
static void phy_dio_dir(uint8_t output)
{
    uint32_t shift;
    volatile uint32_t *cr;

    if (s_dio_out == output) {
        return;
    }
    s_dio_out = output;
    shift = (SWD_DIO_PINNUM & 7u) * 4u;
    cr    = (SWD_DIO_PINNUM < 8u) ? &SWD_DIO_PORT->CRL : &SWD_DIO_PORT->CRH;
    *cr   = (*cr & ~(0xFu << shift)) | ((output ? 0x3u : 0x8u) << shift);
}

static void phy_dio_write(uint8_t level)
{
    if (level) {
        SWD_DIO_PORT->BSRR = (uint32_t)SWD_DIO_PIN;
    } else {
        SWD_DIO_PORT->BSRR = (uint32_t)SWD_DIO_PIN << 16;
    }
}

static uint8_t phy_dio_read(void)
{
    return (SWD_DIO_PORT->IDR & (uint32_t)SWD_DIO_PIN) ? 1u : 0u;
}

/* 半周期延时：次数由 s_delay_loops 决定（运行期可调，见 SWDP_SetDelay） */
static void phy_delay(void)
{
    volatile uint16_t i;
    for (i = 0u; i < s_delay_loops; i++) {
        __NOP();
    }
}

/* SWCLK 闲置高电平，每个时钟拍是一个"低脉冲"（照 DAPProg 的相位） */
static void phy_clk_cycle(void)
{
    phy_clk_set(0u);
    phy_delay();
    phy_clk_set(1u);
    phy_delay();
}

static void swd_phy_init(void)
{
#if (SWD_FREE_DEBUG_PINS != 0)
    /* DUT SWD 占用 PA13/PA14（STLINK_USE_SWDIO_SWCLK=1）：必须先关掉主机自己
     * 的 SW-DP（SWJ_CFG=100），否则这两脚由调试口占用，当不了 GPIO。
     * 写 MAPR 需要 AFIO 时钟。 */
    __HAL_RCC_AFIO_CLK_ENABLE();
    __HAL_AFIO_REMAP_SWJ_DISABLE();
#else
    /* STLINK_USE_SWDIO_SWCLK=0：DUT SWD 用 PD2/PC12，PA13/PA14 要恢复/保持
     * ST-Link 的 SW-DP 功能。写 SWJ_CFG=010（关 JTAG、开 SW-DP），即使上一次
     * 运行把它关成 100，也能不用断电直接恢复。 */
    __HAL_RCC_AFIO_CLK_ENABLE();
    __HAL_AFIO_REMAP_SWJ_NOJTAG();
#endif

    /* SWD_CLK/SWD_DIO 可能在 GPIOA（PA13/PA14）或 GPIOD/GPIOC（PD2/PC12），
     * 相关端口时钟都开，换接法时不用再改这里。 */
    __HAL_RCC_GPIOA_CLK_ENABLE();
    __HAL_RCC_GPIOB_CLK_ENABLE();
    __HAL_RCC_GPIOC_CLK_ENABLE();
    __HAL_RCC_GPIOD_CLK_ENABLE();
    {
        GPIO_InitTypeDef init;
        init.Pin   = SWD_CLK_PIN;
        init.Mode  = GPIO_MODE_OUTPUT_PP;
        init.Pull  = GPIO_NOPULL;
        init.Speed = GPIO_SPEED_FREQ_HIGH;
        HAL_GPIO_Init(SWD_CLK_PORT, &init);
    }

    /* 空闲态：SWDIO 由主机驱动为高、SWCLK 高（与 DAPProg 一致） */
    s_dio_out = 0u;                 /* 置 0 以便下面真正写一次 CR */
    phy_dio_dir(1u);
    phy_dio_write(1u);
    phy_clk_set(1u);

    /* 排障用：打印**实际** SWD_CLK/SWD_DIO 两个引脚的 CR 配置。
     * 当前接法是 PD2=SWCLK、PC12=SWDIO，所以看 GPIOD->CRL / GPIOC->CRH，
     * 不再是 GPIOA 的 PA13/PA14。输出脚应为 0x3（推挽 50MHz）。 */
    {
        volatile uint32_t *clk_cr = (SWD_CLK_PINNUM < 8u) ? &SWD_CLK_PORT->CRL
                                                          : &SWD_CLK_PORT->CRH;
        volatile uint32_t *dio_cr = (SWD_DIO_PINNUM < 8u) ? &SWD_DIO_PORT->CRL
                                                          : &SWD_DIO_PORT->CRH;
#if (SWD_FREE_DEBUG_PINS != 0)
        LOGI("swd: clk_cr=%x dio_cr=%x swj=%d",
             (unsigned int)*clk_cr, (unsigned int)*dio_cr,
             (int)((AFIO->MAPR >> 24) & 7u));
#else
        LOGI("swd: clk_cr=%x dio_cr=%x (host sw-dp kept)",
             (unsigned int)*clk_cr, (unsigned int)*dio_cr);
#endif
    }
}

/*==============================================================================
 * §1.5 目标板 NRST / BOOT0 控制（PC6 / PC7）
 *----------------------------------------------------------------------------
 * 目标板 NRST 有 3.3V 上拉、低电平复位（直连，无反相）：
 *   PC6 高/高阻 → NRST 高（释放）
 *   PC6 低      → NRST 低（复位）
 * NRST 用开漏：只负责拉低，释放时靠目标板的上拉回高，避免和板上复位电路
 * 对顶。BOOT0 是静态配置脚，推挽给出确定电平。
 * 当前流程：
 *   ① 烧录前：BOOT0 低 + NRST 20ms 复位，让用户程序先跑一下；
 *   ② BOOT0 高保持 100ms，再 NRST 20ms 复位，进系统 bootloader；
 *   ③ SWD 连接/擦写/校验；
 *   ④ 烧录完成：BOOT0 低 + NRST 20ms 复位，运行新程序。
 * NRST 低脉宽 20ms 已非常保守（F1 只需几十 µs），原来的 500ms 太长。
 *============================================================================*/
static uint8_t s_dut_ctrl_ready = 0u;

static void dut_nrst_drive(uint8_t assert_reset)
{
#if (DUT_NRST_ACTIVE_LOW != 0)
    HAL_GPIO_WritePin(DUT_NRST_PORT, DUT_NRST_PIN,
                      (assert_reset != 0u) ? GPIO_PIN_RESET : GPIO_PIN_SET);
#else
    HAL_GPIO_WritePin(DUT_NRST_PORT, DUT_NRST_PIN,
                      (assert_reset != 0u) ? GPIO_PIN_SET : GPIO_PIN_RESET);
#endif
}

static void dut_boot_drive(uint8_t bootloader)
{
#if (DUT_BOOT_ACTIVE_HIGH != 0)
    HAL_GPIO_WritePin(DUT_BOOT_PORT, DUT_BOOT_PIN,
                      (bootloader != 0u) ? GPIO_PIN_SET : GPIO_PIN_RESET);
#else
    HAL_GPIO_WritePin(DUT_BOOT_PORT, DUT_BOOT_PIN,
                      (bootloader != 0u) ? GPIO_PIN_RESET : GPIO_PIN_SET);
#endif
}

void SWDP_DutCtrlInit(void)
{
    GPIO_InitTypeDef init;

    if (s_dut_ctrl_ready != 0u) {
        return;
    }
    __HAL_RCC_GPIOC_CLK_ENABLE();

    /* 先写安全电平、再切成输出：NRST 保持释放、BOOT0 拉高（系统 bootloader）。
     * 避免初始化瞬间的毛刺把 DUT 摁进复位或错误的启动模式。 */
    dut_nrst_drive(0u);
    dut_boot_drive(1u);

    init.Pin   = DUT_NRST_PIN;
    init.Mode  = DUT_NRST_MODE;
    init.Pull  = GPIO_NOPULL;
    init.Speed = GPIO_SPEED_FREQ_LOW;
    HAL_GPIO_Init(DUT_NRST_PORT, &init);

    init.Pin   = DUT_BOOT_PIN;
    init.Mode  = GPIO_MODE_OUTPUT_PP;
    init.Pull  = GPIO_NOPULL;
    init.Speed = GPIO_SPEED_FREQ_LOW;
    HAL_GPIO_Init(DUT_BOOT_PORT, &init);

    s_dut_ctrl_ready = 1u;
}

static uint8_t swd_parity_of(uint32_t v)
{
    v ^= v >> 16;
    v ^= v >> 8;
    v ^= v >> 4;
    v ^= v >> 2;
    v ^= v >> 1;
    return (uint8_t)(v & 1u);
}

static void swd_write_bit(uint8_t b)
{
    /* 照搬 DAPProg：OUT(bit) → CLK 低 → 延时 → CLK 高 → 延时 */
    phy_dio_dir(1u);
    phy_dio_write(b ? 1u : 0u);
    phy_clk_set(0u);
    phy_delay();
    phy_clk_set(1u);
    phy_delay();
}

static uint8_t swd_read_bit(void)
{
    /* 照搬 DAPProg：CLK 低 → 延时 → 采样 → CLK 高 → 延时
     * 在时钟低电平期间采样（目标在上升沿驱动，低电平期间数据已稳定）。 */
    uint8_t b;

    phy_clk_set(0u);
    phy_delay();
    b = phy_dio_read();
    phy_clk_set(1u);
    phy_delay();
    return b;
}

static void swd_write_bits(uint32_t v, uint8_t n)
{
    uint8_t i;
    for (i = 0u; i < n; i++) {
        swd_write_bit((uint8_t)((v >> i) & 1u));   /* LSB first */
    }
}

static uint32_t swd_read_bits(uint8_t n)
{
    uint32_t v = 0u;
    uint8_t  i;
    for (i = 0u; i < n; i++) {
        v |= (uint32_t)swd_read_bit() << i;
    }
    return v;
}

static void swd_turnaround(void)
{
    phy_dio_dir(0u);            /* 释放总线 */
    phy_clk_cycle();
}

/* 空闲周期：主机把 SWDIO 驱动到低并打一拍。
 * DP 靠 Start 位的 0→1 跳变识别包边界，每个包之前都必须先把它拉低。 */
static void swd_idle_clock(void)
{
    phy_dio_dir(1u);
    phy_dio_write(0u);
    phy_clk_cycle();
}

static void swd_line_reset(void)
{
    uint8_t i;
    phy_dio_dir(1u);
    phy_dio_write(1u);
    for (i = 0u; i < 56u; i++) {   /* ADIv5 要求 ≥50 个高电平周期 */
        phy_clk_cycle();
    }
}

/* 事务尾部：turnaround（可选）+ 空闲拍，最后把 SWDIO 驱动为高 */
static void swd_idle_tail(uint8_t turnaround)
{
    uint8_t i;
    for (i = 0u; i < turnaround; i++) {
        swd_turnaround();
    }
#if (SWD_TAIL_IDLE_CYCLES > 0u)
    for (i = 0u; i < SWD_TAIL_IDLE_CYCLES; i++) {
        swd_write_bit((uint8_t)SWD_IDLE_LEVEL);
    }
#else
    (void)i;
#endif
    phy_dio_dir(1u);
    phy_dio_write(1u);
}

/* 协议错误退避：补 33 拍（32 数据 + 1 校验），让 DP 重新对齐，
 * 否则下一笔事务会被错位解析（照 DAPLink 的做法）。 */
static void swd_error_backoff(void)
{
    uint8_t i;
    for (i = 0u; i < (SWD_TA_TAIL + 33u); i++) {
        phy_clk_cycle();
    }
    phy_dio_dir(1u);
    phy_dio_write(1u);
}

static int swd_transfer(uint8_t apndp, uint8_t rnw, uint8_t addr, uint32_t *data)
{
    uint8_t req;
    uint8_t a2 = (uint8_t)((addr >> 2) & 1u);
    uint8_t a3 = (uint8_t)((addr >> 3) & 1u);
    uint8_t par;
    uint8_t ack;
    uint8_t i;

    apndp = (uint8_t)(apndp & 1u);
    rnw   = (uint8_t)(rnw & 1u);
    par   = swd_parity_of((uint32_t)(apndp | (rnw << 1) | (a2 << 2) | (a3 << 3)));

    req  = 0x81u;                                        /* Start=1, Park=1 */
    req |= (uint8_t)(apndp << 1);
    req |= (uint8_t)(rnw << 2);
    req |= (uint8_t)(a2 << 3);
    req |= (uint8_t)(a3 << 4);
    req |= (uint8_t)(par << 5);

    s_last_req = req;

    swd_idle_clock();          /* 包边界：先把 SWDIO 拉低，制造 Start 的上升沿 */
    swd_write_bits(req, 8u);

    for (i = 0u; i < SWD_TA_AFTER_HEADER; i++) {
        swd_turnaround();
    }
    phy_dio_dir(0u);

    ack = (uint8_t)swd_read_bits(3u);
    s_last_ack = ack;

    if (ack != SWD_ACK_OK) {
        if (ack == SWD_ACK_WAIT) {
            swd_idle_tail(SWD_TA_TAIL);
            return SWD_ERR_WAIT;
        }
        if (ack == SWD_ACK_FAULT) {
            swd_idle_tail(SWD_TA_TAIL);
            return SWD_ERR_FAULT;
        }
        swd_error_backoff();
        return SWD_ERR_NO_ACK;
    }

    if (rnw) {
        uint32_t v;
        uint8_t  p;

        phy_dio_dir(0u);
        v = swd_read_bits(32u);
        p = swd_read_bit();

        swd_idle_tail(SWD_TA_TAIL);
        if (p != swd_parity_of(v)) {
            return SWD_ERR_PARITY;
        }
        if (data != 0) {
            *data = v;
        }
    } else {
        uint32_t v = (data != 0) ? *data : 0u;

        for (i = 0u; i < SWD_TA_AFTER_ACK_WRITE; i++) {
            swd_turnaround();
        }
        swd_write_bits(v, 32u);
        swd_write_bit(swd_parity_of(v));
    }

    return SWD_OK;
}

/*==============================================================================
 * §2 DP —— 调试端口
 *============================================================================*/
#define SWD_DP_ABORT        0x00u   /* 写：清 sticky */
#define SWD_DP_IDCODE       0x00u   /* 读：IDCODE    */
#define SWD_DP_CTRLSTAT     0x04u
#define SWD_DP_SELECT       0x08u
#define SWD_DP_RDBUFF       0x0Cu

#define SWD_ABORT_STKCMPCLR  (1u << 1)
#define SWD_ABORT_STKERRCLR  (1u << 2)
#define SWD_ABORT_WDERRCLR   (1u << 3)
#define SWD_ABORT_ORUNERRCLR (1u << 4)
#define SWD_ABORT_CLEAR_ALL  (SWD_ABORT_STKCMPCLR | SWD_ABORT_STKERRCLR | \
                              SWD_ABORT_WDERRCLR  | SWD_ABORT_ORUNERRCLR)

#define SWD_CTRLSTAT_CDBGPWRUPREQ (1u << 28)
#define SWD_CTRLSTAT_CDBGPWRUPACK (1u << 29)
#define SWD_CTRLSTAT_CSYSPWRUPREQ (1u << 30)
#define SWD_CTRLSTAT_CSYSPWRUPACK (1u << 31)

static int dp_xfer_raw(uint8_t apndp, uint8_t rnw, uint8_t addr, uint32_t *value)
{
    return swd_transfer(apndp, rnw, addr, value);
}

/* 前置声明：dp_xfer 在 FAULT 重试时要清 sticky，而它的实现在下面。 */
static int swd_clear_sticky(void);

/* 带重试：WAIT=目标忙重试；FAULT=先清 sticky 再试；NO_ACK（畸形 ACK）=退避后重发。
 * 2026-11-03 实测：DRW 写之后的 DP 读偶发畸形 ACK，加这条后即可自愈。 */
static int dp_xfer(uint8_t apndp, uint8_t rnw, uint8_t addr, uint32_t *value)
{
    uint32_t tries;
    int      r;

    for (tries = 0u; ; tries++) {
        r = swd_transfer(apndp, rnw, addr, value);
        if (r == SWD_OK) {
            return SWD_OK;
        }
        if (r == SWD_ERR_WAIT && tries < SWD_RETRY_WAIT) {
            continue;
        }
        if (r == SWD_ERR_FAULT && tries < SWD_RETRY_FAULT) {
            (void)swd_clear_sticky();
            continue;
        }
        if (r == SWD_ERR_NO_ACK && tries < SWD_RETRY_NOACK) {
            continue;
        }
        return r;
    }
}

/* SW-DP 上清 sticky 必须走 ABORT（CTRL/STAT 的 sticky 位是 RO/WI，
 * 而且写 CTRL/STAT 会顺带把 CDBGPWRUPREQ 撤掉）。 */
static int swd_clear_sticky(void)
{
    uint32_t v = (uint32_t)SWD_ABORT_CLEAR_ALL;
    return dp_xfer_raw(0u, 0u, SWD_DP_ABORT, &v);
}

static int swd_select_ap(uint8_t apsel, uint8_t bank)
{
    uint32_t sel = ((uint32_t)apsel << 24) | ((uint32_t)(bank & 0x0Fu) << 4);
    return dp_xfer(0u, 0u, SWD_DP_SELECT, &sel);
}

static int swd_dp_read(uint8_t addr, uint32_t *value)
{
    return dp_xfer(0u, 1u, addr, value);
}

static int swd_ap_read(uint8_t addr, uint32_t *value)
{
    return dp_xfer(1u, 1u, addr, value);
}

static int swd_ap_write(uint8_t addr, uint32_t value)
{
    uint32_t v = value;
    return dp_xfer(1u, 0u, addr, &v);
}

static int swd_connect(uint32_t *idcode)
{
    uint32_t id;
    uint32_t i;
    int      r;

    /* 完全照搬 DAPProg 的连接序列：
     *   ① 线复位（≥50 拍 SWDIO=1）
     *   ② JTAG→SWD 切换序列 0xE79E（LSB first）
     *   ③ 8 拍 SWDIO=0（不再做第二次线复位）
     *   ④ 直接读 IDCODE                                                        */
    for (i = 0u; i < SWD_CONNECT_RETRY; i++) {
        uint8_t k;

        swd_line_reset();
        swd_write_bits(0xE79Eu, 16u);
        for (k = 0u; k < 8u; k++) {
            swd_write_bit(0u);
        }

        id = 0u;
        r  = dp_xfer_raw(0u, 1u, SWD_DP_IDCODE, &id);
        /* 只接受 ARM 的 SW-DP：IDCODE[31:24] = 0x1B(M3) / 0x2B(M4) / 0x0B(M0)。
         * 线长时读回的 IDCODE 可能"低 16 位对、高 16 位错"却仍过奇偶校验，
         * 这里再卡一道：坏值直接重试，不带着错值往下走。 */
        if (r == SWD_OK && id != 0u && id != 0xFFFFFFFFu &&
            (((id >> 24) == 0x1Bu) || ((id >> 24) == 0x2Bu) || ((id >> 24) == 0x0Bu))) {
            if (idcode != 0) {
                *idcode = id;
            }
            return SWD_OK;
        }
    }
    LOGW("swd: connect fail, ack=%x req=%x", (int)s_last_ack, (int)s_last_req);
    return SWD_ERR_NO_ACK;
}

static int swd_power_up(void)
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
            /* 上电成功后立刻清一次 sticky：第一笔 AP 访问一旦 FAULT，
             * STICKYERR 会一直锁着，之后所有 AP 访问都返回 FAULT。 */
            (void)swd_clear_sticky();
            return SWD_OK;
        }
    }
    return SWD_ERR_TIMEOUT;
}

/*==============================================================================
 * §3 MEM-AP —— AHB-AP 内存访问
 *----------------------------------------------------------------------------
 * CSW 取值（ADIv5 IHI0031 表 C2-1）：
 *   HPROT[31:24]=0x23、MSTRDBG=bit29、DBGSTAT=bit6
 *   Size[2:0] = 010(字) / 001(半字)；AddrInc[5:4] = 01（increment single）
 * 单次访问与块访问共用：单次每次重写 TAR，自增与否无所谓；块访问必须自增。
 *============================================================================*/
#define AHBAP_CSW_WORD            0x23000052u   /* 32 位 + 自增 */
#define AHBAP_CSW_HALFWORD_INC    0x23000051u   /* 16 位 + 自增 */

#define SWD_AP_CSW     0x00u
#define SWD_AP_TAR     0x04u
#define SWD_AP_DRW     0x0Cu

static uint32_t s_csw  = 0u;
static uint8_t  s_apok = 0u;

static int memap_prepare(uint32_t csw)
{
    int      r;
    uint32_t flush = 0u;

    if (s_apok == 0u) {
        r = swd_select_ap(0u, 0u);          /* 单 AP（APSEL=0），bank0 */
        if (r != SWD_OK) {
            return r;
        }
        s_apok = 1u;
        s_csw  = 0u;
    }
    if (s_csw != csw) {
        r = swd_ap_write(SWD_AP_CSW, csw);
        if (r != SWD_OK) {
            return r;
        }
        /* AP 写是 posted：读一次 DP RDBUFF 把 CSW 写推完，确保后面的 TAR/DRW
         * 用的是新的 Size/AddrInc。 */
        (void)swd_dp_read(SWD_DP_RDBUFF, &flush);
        if (csw == AHBAP_CSW_HALFWORD_INC) {
            static uint8_t s_csw_dbg = 0u;
            if (s_csw_dbg == 0u) {
                uint32_t rd = 0u;
                uint32_t dummy = 0u;
                s_csw_dbg = 1u;
                (void)swd_ap_read(SWD_AP_CSW, &dummy);
                (void)swd_dp_read(SWD_DP_RDBUFF, &rd);
                LOGI("swd: csw wr=%x rd=%x", (unsigned int)csw, (unsigned int)rd);
            }
        }
        s_csw = csw;
    }
    return SWD_OK;
}

static int swd_mem_init(void)
{
    s_apok = 0u;
    s_csw  = 0u;

    /* 进 AP 之前先清一次 sticky：上电后第一笔 AP 访问很容易 FAULT，
     * 一旦锁上 STICKYERR，后续所有 AP 访问都会返回 FAULT。 */
    (void)swd_clear_sticky();
    return memap_prepare(AHBAP_CSW_WORD);
}

static int swd_mem_read32(uint32_t addr, uint32_t *value)
{
    uint32_t dummy = 0u;
    int      r;

    r = memap_prepare(AHBAP_CSW_WORD);
    if (r != SWD_OK) {
        return r;
    }
    r = swd_ap_write(SWD_AP_TAR, addr & ~3u);
    if (r != SWD_OK) {
        return r;
    }
    r = swd_ap_read(SWD_AP_DRW, &dummy);        /* posted：发起本次传输 */
    if (r != SWD_OK) {
        return r;
    }
    return swd_dp_read(SWD_DP_RDBUFF, value);   /* 本次结果 */
}

static int swd_mem_write32(uint32_t addr, uint32_t value)
{
    uint32_t v     = value;
    uint32_t flush = 0u;
    int      r;

    r = memap_prepare(AHBAP_CSW_WORD);
    if (r != SWD_OK) {
        return r;
    }
    r = swd_ap_write(SWD_AP_TAR, addr & ~3u);
    if (r != SWD_OK) {
        return r;
    }
    r = swd_ap_write(SWD_AP_DRW, v);
    if (r != SWD_OK) {
        return r;
    }
    /* MEM-AP 的 DRW 写是 posted：读一次 RDBUFF 把它推完（DAPLink/OpenOCD 标准做法） */
    (void)swd_dp_read(SWD_DP_RDBUFF, &flush);
    return SWD_OK;
}

static int swd_mem_read_block(uint32_t addr, uint32_t *buf, uint32_t n)
{
    uint32_t dummy = 0u;
    uint32_t i;
    int      r;

    if (n == 0u) {
        return SWD_OK;
    }
    r = memap_prepare(AHBAP_CSW_WORD);
    if (r != SWD_OK) {
        return r;
    }
    r = swd_ap_write(SWD_AP_TAR, addr & ~3u);
    if (r != SWD_OK) {
        return r;
    }
    r = swd_ap_read(SWD_AP_DRW, &dummy);        /* 丢弃脏数据，发起第 1 次传输 */
    if (r != SWD_OK) {
        return r;
    }
    for (i = 0u; i < n; i++) {
        r = swd_ap_read(SWD_AP_DRW, &buf[i]);   /* 第 i 次拿到第 i 个数据 */
        if (r != SWD_OK) {
            return r;
        }
    }
    return SWD_OK;
}

/* F1 编程专用：CSW 切 16 位；每个半字的 TAR 由调用方显式重写
 * （不依赖 AP 的自动递增，F105 上它会按 4 字节跳）。
 * 注意：块内不许再调 swd_mem_read32/write32 —— 那会重写 TAR/CSW。 */
static int swd_mem_begin_halfwords(uint32_t addr)
{
    int r = memap_prepare(AHBAP_CSW_HALFWORD_INC);
    if (r != SWD_OK) {
        return r;
    }
    return swd_ap_write(SWD_AP_TAR, addr & ~1u);
}

/* 16 位 AP 写：数据必须放在 A[1] 选中的半字 lane 上。
 *   addr 的 bit1 = 0（+0/+4…）→ 数据放 DRW 低 16 位；
 *   addr 的 bit1 = 1（+2/+6…）→ 数据放 DRW 高 16 位。
 * 否则 +2 的那一笔会被 Flash 采成 0，表现为每个字高半字为 0。 */
static int swd_mem_write_halfword_at(uint32_t addr, uint16_t v)
{
    uint32_t w = ((addr & 2u) != 0u) ? ((uint32_t)v << 16) : (uint32_t)v;
    return swd_ap_write(SWD_AP_DRW, w);
}

/*==============================================================================
 * §4 Cortex-M —— 内核调试控制
 *----------------------------------------------------------------------------
 * 只做烧录真正需要的四件事：停核、冻看门狗、认型号、复位运行。
 * F1 与 F4 的差别在 DBGMCU：F1 只有一个 DBGMCU_CR(0xE0042004)，
 * IWDG_STOP=bit8、WWDG_STOP=bit9（F4 是 0xE0042008 的 bit12/bit11）。
 *============================================================================*/
#define CM_DHCSR            0xE000EDF0u
#define CM_DEMCR            0xE000EDFCu
#define CM_AIRCR            0xE000ED0Cu

#define CM_DHCSR_KEY        0xA05F0000u
#define CM_DHCSR_C_DEBUGEN  0x00000001u
#define CM_DHCSR_C_HALT     0x00000002u
#define CM_DHCSR_S_HALT     0x00020000u

#define CM_DEMCR_VC_CORERESET 0x00000001u

#define CM_AIRCR_VECTKEY    0x05FA0000u
#define CM_AIRCR_SYSRESETREQ 0x00000004u

#define CM_DBGMCU_IDCODE    0xE0042000u
#define CM_DBGMCU_CR        0xE0042004u
#define CM_DBGMCU_CR_IWDG_STOP (1u << 8)
#define CM_DBGMCU_CR_WWDG_STOP (1u << 9)

static int cm_is_halted(uint8_t *halted)
{
    uint32_t v = 0u;
    int      r = swd_mem_read32(CM_DHCSR, &v);

    if (r != SWD_OK) {
        return r;
    }
    if (halted != 0) {
        *halted = ((v & CM_DHCSR_S_HALT) != 0u) ? 1u : 0u;
    }
    return SWD_OK;
}

static int cm_halt(void)
{
    uint32_t v = CM_DHCSR_KEY | CM_DHCSR_C_DEBUGEN | CM_DHCSR_C_HALT;
    uint8_t  halted = 0u;
    uint32_t i;
    int      r;

    r = swd_mem_write32(CM_DHCSR, v);
    if (r != SWD_OK) {
        return r;
    }
    for (i = 0u; i < 20u; i++) {          /* 正常核几笔内就会置 S_HALT；20 足够 */
        r = cm_is_halted(&halted);
        if (r != SWD_OK) {
            return r;
        }
        if (halted != 0u) {
            return SWD_OK;
        }
    }
    return SWD_ERR_NOT_HALTED;
}

static int cm_run(void)
{
    uint32_t v = CM_DHCSR_KEY | CM_DHCSR_C_DEBUGEN;   /* 清 C_HALT */
    return swd_mem_write32(CM_DHCSR, v);
}

#if (SWD_USE_DUT_NRST == 0)
static int cm_reset_and_run(void)
{
    int r;

    r = cm_run();
    if (r != SWD_OK) {
        return r;
    }
    return swd_mem_write32(CM_AIRCR, CM_AIRCR_VECTKEY | CM_AIRCR_SYSRESETREQ);
}
#endif

/* 冻结 IWDG/WWDG：目标自己的看门狗不会因为内核被停住而暂停，
 * 不冻结的话擦写期间可能被它复位。 */
static int cm_freeze_watchdogs(void)
{
    uint32_t v = 0u;
    int      r;

    r = swd_mem_read32(CM_DBGMCU_CR, &v);
    if (r != SWD_OK) {
        return r;
    }
    v |= (CM_DBGMCU_CR_IWDG_STOP | CM_DBGMCU_CR_WWDG_STOP);
    return swd_mem_write32(CM_DBGMCU_CR, v);
}

static int cm_read_dev_id(uint16_t *dev_id)
{
    uint32_t v = 0u;
    int      r = swd_mem_read32(CM_DBGMCU_IDCODE, &v);

    if (r != SWD_OK) {
        return r;
    }
    if (dev_id != 0) {
        *dev_id = (uint16_t)(v & 0x0FFFu);
    }
    return SWD_OK;
}

/* 空片 / 锁死的核：直接写 C_HALT 可能停不住（内核在 lockup 里）。
 * 标准做法是先置 DEMCR.VC_CORERESET，再 SYSRESETREQ —— 核会在取第一条
 * 指令之前停住（复位向量捕获），此时 S_HALT=1，可以安全擦写。
 * ⚠ GO 阶段必须先把 VC_CORERESET 清掉，否则复位后又会停在复位向量、跑不起来。 */
static int cm_reset_and_halt(void)
{
    uint32_t v = 0u;
    uint32_t i;
    int      r;

    r = swd_mem_write32(CM_DEMCR, CM_DEMCR_VC_CORERESET);
    if (r != SWD_OK) {
        return r;
    }
    r = swd_mem_write32(CM_AIRCR, CM_AIRCR_VECTKEY | CM_AIRCR_SYSRESETREQ);
    if (r != SWD_OK) {
        return r;
    }
    for (i = 0u; i < 100u; i++) {
        r = swd_mem_read32(CM_DHCSR, &v);
        if (r == SWD_OK && (v & CM_DHCSR_S_HALT) != 0u) {
            return SWD_OK;
        }
    }
    return SWD_ERR_NOT_HALTED;
}

/*==============================================================================
 * §5 F105 Flash —— 页擦除 + 半字编程 + 回读校验
 *----------------------------------------------------------------------------
 * 与 F4 的关键差别（换目标最容易踩的地方）：
 *   - 擦除单位是 2KB 页，不是不等大扇区：CR=PER + FLASH_AR=页地址 + CR.STRT；
 *   - 编程用 **16 位半字**（MEM-AP CSW 换 16 位，块内 TAR 只写一次、DRW 自动 +2）；
 *   - LOCK 在 CR bit7（F4 是 bit31），STRT 在 bit6（F4 是 bit16）；
 *   - SR 里 BSY=bit0、EOP=bit5（F4 是 bit16 / bit0）。
 *
 * ⚠ 半字写的节流：FPEC 一次编程约 52.5µs（t_PROG），写完一个半字后
 *   若在 BSY=1 期间再写下一个，会丢数据/置 PGERR。主机侧没法像目标代码
 *   那样"每半字轮询 SR"（每次轮询都要重设 TAR，代价 3 笔事务），所以这里
 *   采用"写一个 → 推完 → 固定等 ≥100µs"的节流方式；整块结束后再查一次 SR。
 *   回读校验是最后一道防线：节流不够时一定会在校验阶段暴露。
 *============================================================================*/
#define F1_FLASH_ACR        0x40022000u
#define F1_FLASH_KEYR       0x40022004u
#define F1_FLASH_SR         0x4002200Cu
#define F1_FLASH_CR         0x40022010u
#define F1_FLASH_AR         0x40022014u
#define F1_FLASH_OBR        0x4002201Cu

#define F1_FLASHSIZE_REG    0x1FFFF7E0u

#define F1_KEY1             0x45670123u
#define F1_KEY2             0xCDEF89ABu

#define F1_SR_BSY           (1u << 0)
#define F1_SR_PGERR         (1u << 2)
#define F1_SR_WRPRTERR      (1u << 4)
#define F1_SR_EOP           (1u << 5)
#define F1_SR_ERRMASK       (F1_SR_PGERR | F1_SR_WRPRTERR)

#define F1_CR_PG            (1u << 0)
#define F1_CR_PER           (1u << 1)
#define F1_CR_STRT          (1u << 6)
#define F1_CR_LOCK          (1u << 7)

#define F1_OBR_RDPRT        (1u << 1)

/* 写半字后的固定节流：72MHz 下每次循环约 5~7 周期，
 * 1200 次 ≈ 83~117µs，覆盖 t_PROG(52.5µs) 并留余量。 */
/* 写一个半字后的固定节流：FPEC 一次编程约 52.5µs；1200 次循环 ≈ 100µs。 */
#define F1_PROG_WAIT_LOOPS  1200u

static int f1_read_sr(uint32_t *sr)
{
    return swd_mem_read32(F1_FLASH_SR, sr);
}

/* SR 的 EOP/错误位都是写 1 清零 */
static int f1_clear_flags(void)
{
    return swd_mem_write32(F1_FLASH_SR, (uint32_t)(F1_SR_EOP | F1_SR_ERRMASK));
}

static int f1_is_locked(uint8_t *locked)
{
    uint32_t cr = 0u;
    int      r  = swd_mem_read32(F1_FLASH_CR, &cr);

    if (r != SWD_OK) {
        return r;
    }
    if (locked != 0) {
        *locked = ((cr & F1_CR_LOCK) != 0u) ? 1u : 0u;
    }
    return SWD_OK;
}

static int f1_unlock(void)
{
    uint8_t locked = 0u;
    int     r;

    r = f1_is_locked(&locked);
    if (r != SWD_OK) {
        return r;
    }
    if (locked == 0u) {
        return SWD_OK;
    }
    r = swd_mem_write32(F1_FLASH_KEYR, F1_KEY1);
    if (r != SWD_OK) {
        return r;
    }
    r = swd_mem_write32(F1_FLASH_KEYR, F1_KEY2);
    if (r != SWD_OK) {
        return r;
    }
    r = f1_is_locked(&locked);
    if (r != SWD_OK) {
        return r;
    }
    return (locked != 0u) ? SWD_ERR_FLASH : SWD_OK;
}

/* 等 BSY 清零 + 查错误位 */
static int f1_wait_done(void)
{
    uint32_t start = HAL_GetTick();
    uint32_t sr    = 0u;
    uint32_t i;
    int      r;

    for (i = 0u; i < SWD_POLL_MAX; i++) {
        r = f1_read_sr(&sr);
        if (r != SWD_OK) {
            return r;
        }
        if ((sr & F1_SR_BSY) == 0u) {
            break;
        }
        if ((HAL_GetTick() - start) > F1_FLASH_TIMEOUT_MS) {
            return SWD_ERR_TIMEOUT;
        }
    }
    if ((sr & F1_SR_BSY) != 0u) {
        return SWD_ERR_TIMEOUT;
    }
    return ((sr & F1_SR_ERRMASK) != 0u) ? SWD_ERR_FLASH : SWD_OK;
}

static int f1_page_of(uint32_t addr, uint32_t *page)
{
    if (addr < F1_FLASH_BASE || addr >= (F1_FLASH_BASE + F1_FLASH_SIZE)) {
        return SWD_ERR_PARAM;
    }
    if (page != 0) {
        *page = (addr - F1_FLASH_BASE) / F1_PAGE_SIZE;
    }
    return SWD_OK;
}

/* 擦除一页。烧录状态机一次只调一次，保持主循环响应性（F105 一页约 20~40ms）。 */
static int f1_erase_page(uint32_t page)
{
    uint32_t addr;
    int      r;

    if (page >= F1_PAGE_COUNT) {
        return SWD_ERR_PARAM;
    }
    addr = F1_FLASH_BASE + page * F1_PAGE_SIZE;

    r = f1_unlock();
    if (r != SWD_OK) {
        return r;
    }
    r = f1_clear_flags();
    if (r != SWD_OK) {
        return r;
    }
    /* 上一次擦/写必须彻底结束（尤其是"整链重连后重来"的路径） */
    r = f1_wait_done();
    if (r != SWD_OK) {
        return r;
    }
    /* RM0008 的顺序：CR.PER=1 → 写 FLASH_AR → CR.STRT=1 */
    r = swd_mem_write32(F1_FLASH_CR, F1_CR_PER);
    if (r != SWD_OK) {
        return r;
    }
    r = swd_mem_write32(F1_FLASH_AR, addr);
    if (r != SWD_OK) {
        return r;
    }
    r = swd_mem_write32(F1_FLASH_CR, F1_CR_PER | F1_CR_STRT);
    if (r != SWD_OK) {
        return r;
    }
    r = f1_wait_done();
    if (r == SWD_OK) {
        /* 24V 供电干扰已通过改用 USB 5V 解决。这里只留 2ms 让擦除电流/地弹
         * 沉降，不再做整链重连 —— 重连里的 DP SELECT 写在擦除后反而会偶发
         * NO_ACK（req=B1 ack=7），把正常烧录卡死。 */
        HAL_Delay(2);
    }
    (void)swd_mem_write32(F1_FLASH_CR, 0u);       /* 清 PER/STRT */
    if (r != SWD_OK) {
        return r;
    }
    return f1_clear_flags();
}

/* 编程会话：解锁 + 清标志 + 置 PG。之后每块只需写数据 + 节流。 */
static int f1_program_begin(void)
{
    int r;

    r = f1_unlock();
    if (r != SWD_OK) {
        return r;
    }
    r = f1_clear_flags();
    if (r != SWD_OK) {
        return r;
    }
    /* 上一次擦/写必须彻底结束（尤其是"整链重连后重来"的路径） */
    r = f1_wait_done();
    if (r != SWD_OK) {
        return r;
    }
    return swd_mem_write32(F1_FLASH_CR, F1_CR_PG);
}

static int f1_program_end(void)
{
    return swd_mem_write32(F1_FLASH_CR, 0u);      /* 清 PG */
}

/* 等 FPEC 完成一个半字的编程。 */
static void f1_prog_delay(void)
{
    volatile uint16_t i;
    for (i = 0u; i < F1_PROG_WAIT_LOOPS; i++) {
        __NOP();
    }
}

/* 在会话内编程一段（尾部不足 2 字节按 0xFF 补齐）。
 * 首块会打印 AP CSW 回读和头两个半字，用来确认 16 位 CSW 是否真正生效。 */
static int f1_program_halfwords(uint32_t addr, const uint8_t *data, uint32_t length)
{
    uint32_t halfwords;
    uint32_t i;
    uint32_t end;
    uint32_t flush;
    int      r = SWD_OK;

    if (length == 0u) {
        return SWD_OK;
    }
    if (data == 0 || (addr & 1u) != 0u) {
        return SWD_ERR_PARAM;
    }
    end = addr + length - 1u;
    if (addr < F1_FLASH_BASE || end >= (F1_FLASH_BASE + F1_FLASH_SIZE)) {
        return SWD_ERR_PARAM;
    }

    halfwords = (length + 1u) / 2u;
    r = swd_mem_begin_halfwords(addr);
    if (r != SWD_OK) {
        return r;
    }

    for (i = 0u; i < halfwords; i++) {
        uint32_t off = i * 2u;
        uint16_t v   = (uint16_t)((uint16_t)data[off] |
                                  ((off + 1u < length) ? ((uint16_t)data[off + 1u] << 8)
                                                       : 0xFF00));
        if (i == 0u) {
            static uint8_t s_hw_dbg = 0u;
            if (s_hw_dbg < 2u) {          /* Boot/App 各打印一次，别刷屏 */
                uint16_t v1 = (uint16_t)((2u < length) ? ((uint16_t)data[2] |
                                       ((3u < length) ? ((uint16_t)data[3] << 8) : 0xFF00))
                                                      : 0xFFFFu);
                s_hw_dbg++;
                LOGI("swd: hw0=%x hw1=%x", (int)v, (int)v1);
            }
        }
        /* 不依赖 AP 的自动递增：实测这颗 F105 的 AHB-AP 在 16 位模式下
         * 仍按 4 字节自增，第二个半字会落到 +4，导致每个字的高半字为 0。
         * 每个半字显式重写一次 TAR，保证严格落在 +0/+2/+4… */
        r = swd_ap_write(SWD_AP_TAR, addr + off);
        if (r != SWD_OK) {
            break;
        }
        r = swd_mem_write_halfword_at(addr + off, v);
        if (r != SWD_OK) {
            break;
        }
        /* posted 写推上总线，再等 FPEC 完成这个半字 */
        (void)swd_dp_read(SWD_DP_RDBUFF, &flush);
        f1_prog_delay();
    }

    /* 块尾统一查一次错误/BSY（这里会切回 32 位 CSW，正好给下一段用） */
    {
        int rw = f1_wait_done();
        if (r != SWD_OK) {
            (void)f1_clear_flags();
            return r;
        }
        if (rw != SWD_OK) {
            (void)f1_clear_flags();
            return rw;
        }
    }
    return SWD_OK;
}

/* 回读一段并逐字节比对。用 auto-increment 块读，比逐字读快 3 倍。 */
static int f1_verify_image(uint32_t addr, const uint8_t *data, uint32_t length)
{
    uint32_t words = (length + 3u) / 4u;
    uint32_t done  = 0u;
    uint32_t chunk[SWDP_CHUNK_WORDS];
    int      r;

    while (done < words) {
        uint32_t n = words - done;
        uint32_t i;

        if (n > (uint32_t)SWDP_CHUNK_WORDS) {
            n = (uint32_t)SWDP_CHUNK_WORDS;
        }
        r = swd_mem_read_block(addr + done * 4u, chunk, n);
        if (r != SWD_OK) {
            return r;
        }
        for (i = 0u; i < n; i++) {
            uint32_t off = (done + i) * 4u;
            uint32_t w   = chunk[i];
            uint8_t  b;
            for (b = 0u; b < 4u; b++) {
                if ((off + b) < length) {
                    if ((uint8_t)((w >> (b * 8u)) & 0xFFu) != data[off + b]) {
                        LOGE("swd: verify bad @%x exp=%x got=%x",
                             (unsigned int)(addr + off + b),
                             (int)data[off + b],
                             (int)((w >> (b * 8u)) & 0xFFu));
                        return SWD_ERR_VERIFY;
                    }
                }
            }
        }
        done += n;
    }
    return SWD_OK;
}

static int f1_read_obr(uint32_t *obr)
{
    return swd_mem_read32(F1_FLASH_OBR, obr);
}

static int f1_read_flash_kb(uint16_t *kb)
{
    uint32_t v = 0u;
    int      r = swd_mem_read32(F1_FLASHSIZE_REG, &v);

    if (r != SWD_OK) {
        return r;
    }
    if (kb != 0) {
        *kb = (uint16_t)(v & 0xFFFFu);
    }
    return SWD_OK;
}

/*==============================================================================
 * §6 烧录状态机（非阻塞，一次一步）
 *----------------------------------------------------------------------------
 * 流程：连接 → 停核+冻结看门狗+认型号+校验两份镜像头（擦除之前！）
 *      → 按页擦除 → 逐块写 → 回读校验 → SYSRESETREQ 复位运行
 * 状态与迁移与 FOC 已验证版本完全一致，只把 F4_* 换成 F1_*。
 *============================================================================*/
static swdp_state_t s_state = SWDP_ST_IDLE;
static uint8_t  s_result = APP_STATUS_ERR_PROG;

static uint32_t s_nrst_tick = 0u;       /* 本次拉低 NRST 的时刻（ms）      */
static uint8_t  s_nrst_done = 1u;       /* 1 = 保持时间已到 / 未启用 NRST */
#if (SWD_RECOVERY_MAX > 0u)
static uint8_t  s_recovery_tries = 0u;  /* 本次烧录已做过的整链重连次数    */
#endif

static uint32_t s_img_len_boot = 0u;
static uint32_t s_img_len_app  = 0u;

static uint8_t  s_phase     = 0u;       /* 0 = Bootloader, 1 = App */
static uint32_t s_slot_base = 0u;       /* 该镜像在治具 SPI Flash 里的槽基址 */
static uint32_t s_dst_addr  = 0u;       /* 该镜像要写到被烧板的绝对地址 */
static uint32_t s_img_len   = 0u;
static uint32_t s_offset    = 0u;
static uint32_t s_page      = 0u;       /* 擦除进度（页号） */

static uint32_t s_idcode    = 0u;
static uint16_t s_dev_id    = 0u;
static uint8_t  s_last_err  = 0u;
static uint8_t  s_fail_state = 0u;   /* 失败时停在哪个状态 */

static uint8_t  s_buf[SWDP_CHUNK_BYTES];

/* 计时（只为日志/现场标定，不影响状态机行为） */
static uint32_t s_t_start = 0u;     /* SWDP_Start 时刻 */
static uint32_t s_t_erase = 0u;     /* 开始擦除的时刻 */

static uint32_t swdp_min_u32(uint32_t a, uint32_t b)
{
    return (a < b) ? a : b;
}

static void swdp_fail(uint8_t status, uint8_t raw_err)
{
#if (SWD_USE_DUT_NRST != 0)
    /* 任何失败都不能把目标一直摁在复位里：释放 NRST（BOOT0 保持高） */
    (void)SWDP_DutCtrlInit();
    dut_nrst_drive(0u);
    dut_boot_drive(1u);
#endif
#if (SWD_RECOVERY_MAX > 0u)
    /* 链路级错误（NO_ACK/WAIT/FAULT/PARITY/TIMEOUT）和回读不符(9)发生在
     * 擦除/编程/校验：DP 可能已经失步，重发同一笔没用，必须整链重连后从
     * CONNECT 重来。重来会重新擦除、重新编程，所以对 PROGRAM 中途失败也安全。
     * FPEC 错误(7)、参数(5) 不在这里恢复。 */
    if ((status == APP_STATUS_ERR_PROG) &&
        ((raw_err == 1u) || (raw_err == 2u) || (raw_err == 3u) ||
         (raw_err == 4u) || (raw_err == 6u) || (raw_err == 9u)) &&
        ((s_state == SWDP_ST_ERASE) || (s_state == SWDP_ST_PROGRAM) ||
         (s_state == SWDP_ST_VERIFY)) &&
        (s_recovery_tries < SWD_RECOVERY_MAX)) {
        s_recovery_tries++;
        LOGW("swd: link err=%d at step=%d, full reconnect #%d",
             (int)raw_err, (int)s_state, (int)s_recovery_tries);
        s_offset = 0u;
        s_page   = 0u;
        s_phase  = 0u;
#if (SWD_USE_DUT_NRST != 0)
        /* 重来前再给一个 SWD_RESET_HOLD_MS 复位脉冲，BOOT0 保持高，重新进 bootloader */
        dut_boot_drive(1u);
        dut_nrst_drive(1u);
        s_nrst_tick = HAL_GetTick();
        s_nrst_done = 0u;
#else
        s_nrst_done = 1u;
#endif
        s_state  = SWDP_ST_CONNECT;
        return;
    }
#endif
    s_last_err   = raw_err;
    s_result     = status;
    s_fail_state = (uint8_t)s_state;
    s_state      = SWDP_ST_FAIL;
    LOGE("swd: FAIL step=%d status=%d err=%d req=%x ack=%x page=%d off=%d",
         (int)s_fail_state, (int)status, (int)raw_err,
         (int)s_last_req, (int)s_last_ack, (int)s_page, (int)s_offset);
}

/* 读槽头并校验：大小非 0、不超分区上限、完整标志为 0xA5
 * @retval 1 = 可用；0 = 槽里没有完整固件；2 = SPI 总线读失败（硬件故障） */
static uint8_t swdp_load_image_head(uint32_t slot_base, uint32_t max_len, uint32_t *p_len)
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
        return 0u;                        /* 不完整 */
    }
    if (size == 0u || size > max_len) {
        return 0u;                        /* 空或越过分区上限 */
    }
    *p_len = size;
    return 1u;
}

static void swdp_select_image(uint8_t which)
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

void SWDP_Init(void)
{
    swd_phy_init();
    s_state  = SWDP_ST_IDLE;
    s_result = APP_STATUS_ERR_PROG;
}

void SWDP_Start(void)
{
    swd_phy_init();                    /* 确保引脚已配好（首次调用会关主机 SW-DP） */
    SWDP_DutCtrlInit();                /* PC6/PC7：NRST 释放、BOOT0=1（幂等） */

#if (SWD_USE_DUT_NRST != 0)
    /* ① 烧录前先让用户程序跑一下：BOOT0 低 + NRST 复位 */
    dut_boot_drive(0u);
    dut_nrst_drive(1u);
    HAL_Delay(SWD_RESET_HOLD_MS);
    dut_nrst_drive(0u);
    HAL_Delay(SWD_APP_RUN_MS);

    /* ② 切下载模式：BOOT0 高保持 SWD_BOOT_SETUP_MS，再拉低 NRST；
     *    CONNECT 阶段等 SWD_RESET_HOLD_MS 后释放并连接。 */
    dut_boot_drive(1u);
    HAL_Delay(SWD_BOOT_SETUP_MS);
    dut_nrst_drive(1u);
    s_nrst_tick = HAL_GetTick();
    s_nrst_done = 0u;
#else
    s_nrst_done = 1u;
#endif

    s_t_start      = HAL_GetTick();
    s_result       = APP_STATUS_ERR_PROG;
    s_last_err     = 0u;
    s_fail_state   = 0u;
    s_idcode       = 0u;
    s_dev_id       = 0u;
    s_img_len_boot = 0u;
    s_img_len_app  = 0u;
    s_offset       = 0u;
    s_phase        = 0u;
    s_page         = 0u;
#if (SWD_RECOVERY_MAX > 0u)
    s_recovery_tries = 0u;
#endif
    s_state        = SWDP_ST_CONNECT;

#if (SWD_USE_DUT_NRST != 0)
    LOGI("swd: boot low+reset app, boot high %dms, nrst %dms",
         (int)SWD_BOOT_SETUP_MS, (int)SWD_RESET_HOLD_MS);
#endif
}

uint8_t SWDP_Busy(void)
{
    return ((s_state != SWDP_ST_IDLE) &&
            (s_state != SWDP_ST_OK) &&
            (s_state != SWDP_ST_FAIL)) ? 1u : 0u;
}

uint8_t SWDP_Result(void)
{
    return (s_state == SWDP_ST_OK) ? APP_STATUS_OK : s_result;
}

uint8_t SWDP_State(void)
{
    return (uint8_t)s_state;
}

uint32_t SWDP_TotalBytes(void)
{
    return s_img_len_boot + s_img_len_app;
}

uint32_t SWDP_LastIdcode(void)   { return s_idcode; }
uint16_t SWDP_LastDevId(void)    { return s_dev_id; }
uint32_t SWDP_LastOffset(void)   { return s_offset; }
uint8_t  SWDP_LastSwdError(void) { return s_last_err; }
uint8_t  SWDP_FailState(void)    { return s_fail_state; }

void SWDP_SetDelay(uint16_t loops)
{
    s_delay_loops = loops;
}

uint16_t SWDP_GetDelay(void)
{
    return s_delay_loops;
}

void SWDP_Step(void)
{
    int r;

    switch (s_state) {

    /* ── 1. 连接：复位脉冲 → 释放 → 线复位/0xE79E/IDCODE → 上电 → 选 AP ── */
    case SWDP_ST_CONNECT:
#if (SWD_USE_DUT_NRST != 0)
        /* NRST 先拉够 SWD_RESET_HOLD_MS，然后**释放**再连接。
         * 不能在 NRST 拉着的时候连接：实测 F105 的 SW-DP 在复位期间不接受
         * 线复位/切换序列，ACK 恒 7（IDCODE 读不到）。释放后多等一个 Step
         * （约 1ms）再连，让复位真正撤掉。 */
        if (s_nrst_done == 0u) {
            if ((uint32_t)(HAL_GetTick() - s_nrst_tick) < SWD_RESET_HOLD_MS) {
                break;
            }
            s_nrst_done = 1u;
            dut_nrst_drive(0u);
            LOGI("swd: nrst released, connecting");
            break;
        }
#endif
        r = swd_connect(&s_idcode);
        if (r != SWD_OK) {
            swdp_fail(APP_STATUS_ERR_PROG, (uint8_t)(-r));
            break;
        }
        r = swd_power_up();
        if (r != SWD_OK) {
            swdp_fail(APP_STATUS_ERR_PROG, (uint8_t)(-r));
            break;
        }
        r = swd_mem_init();
        if (r != SWD_OK) {
            swdp_fail(APP_STATUS_ERR_PROG, (uint8_t)(-r));
            break;
        }
        /* 读保护检查：RDP 开着时擦写一定失败，先给个明确的错，别让人猜 */
        {
            uint32_t obr = 0u;
            if (f1_read_obr(&obr) == SWD_OK && (obr & F1_OBR_RDPRT) != 0u) {
                LOGW("swd: target read protected (RDP)");
                swdp_fail(APP_STATUS_ERR_PROG, 0x40u);
                break;
            }
        }
        LOGI("swd: idcode=%x", s_idcode);
        s_state = SWDP_ST_HALT;
        break;

    /* ── 2. 停核 + 冻结看门狗 + 认型号 + 校验镜像头（擦除之前！） ── */
    case SWDP_ST_HALT:
        r = cm_halt();
        if (r != SWD_OK) {
            /* 空片 / 锁死的核：C_HALT 停不住，改用"复位并停在复位向量"再试一次 */
            LOGW("swd: halt failed %d, retry reset-halt", (int)(-r));
            r = cm_reset_and_halt();
        }
        if (r != SWD_OK) {
            /* 复位脉冲已释放、链路也已连上，仍停不住说明核被锁死或 SWD 访问异常 */
            swdp_fail(APP_STATUS_ERR_PROG, (uint8_t)(-r));
            break;
        }
        (void)swd_mem_write32(CM_DEMCR, 0u);   /* 已停住，撤掉复位向量捕获 */
        (void)cm_freeze_watchdogs();
        (void)cm_read_dev_id(&s_dev_id);

        {
            uint8_t r0 = swdp_load_image_head(FW_SLOT0_BASE, DUT_BOOT_SIZE, &s_img_len_boot);
            uint8_t r1;

            if (r0 != 1u) {
                swdp_fail((r0 == 2u) ? APP_STATUS_ERR_FLASH : APP_STATUS_ERR_INCOMPLETE, 0u);
                break;
            }
            r1 = swdp_load_image_head(FW_SLOT1_BASE, DUT_APP_SIZE, &s_img_len_app);
            if (r1 != 1u) {
                swdp_fail((r1 == 2u) ? APP_STATUS_ERR_FLASH : APP_STATUS_ERR_INCOMPLETE, 0u);
                break;
            }
        }

        LOGI("swd: dev=%x boot=%d app=%d", (int)s_dev_id,
             (int)s_img_len_boot, (int)s_img_len_app);

        /* 擦除计划：Boot 与 App 覆盖到的页（两个分区连续，见 flash_map.h 的断言） */
        r = f1_page_of(DUT_BOOT_ADDR, &s_page);
        if (r != SWD_OK) {
            swdp_fail(APP_STATUS_ERR_PROG, (uint8_t)(-r));
            break;
        }
        s_t_erase = HAL_GetTick();
        s_state = SWDP_ST_ERASE;
        break;

    /* ── 3. 擦除：一步一页（F105 一页 2KB 约 20~40ms） ── */
    case SWDP_ST_ERASE: {
        uint32_t last_page;
        uint32_t app_end = DUT_APP_ADDR + s_img_len_app - 1u;

        if (f1_page_of(app_end, &last_page) != SWD_OK) {
            swdp_fail(APP_STATUS_ERR_PARAM, 0u);
            break;
        }
        r = f1_erase_page(s_page);
        if (r != SWD_OK) {
            swdp_fail(APP_STATUS_ERR_PROG, (uint8_t)(-r));
            break;
        }
        if (s_page >= last_page) {
            LOGI("swd: erase %d pages, %d ms", (int)(last_page + 1u),
                 (int)(HAL_GetTick() - s_t_erase));
            r = f1_program_begin();
            if (r != SWD_OK) {
                swdp_fail(APP_STATUS_ERR_PROG, (uint8_t)(-r));
                break;
            }
            swdp_select_image(0u);
            s_state = SWDP_ST_PROGRAM;
        } else {
            s_page++;
        }
        break;
    }

    /* ── 4. 写：一步 128 字节 ── */
    case SWDP_ST_PROGRAM: {
        uint32_t remain = (s_offset < s_img_len) ? (s_img_len - s_offset) : 0u;

        if (remain == 0u) {
            if (s_phase == 0u) {
                swdp_select_image(1u);            /* 接着写 App */
            } else {
                uint8_t  h4[4];
                uint32_t exp_b = 0u, exp_a = 0u, got_b = 0u, got_a = 0u;

                (void)f1_program_end();           /* 写完清 PG */

                /* 单字读交叉检查：Boot/App 第一个字（栈顶）与 SPI 源比对。
                 * 单字读路径已被 DEV_ID 证明可靠；这里就不一致 = 写没进去，
                 * 而不是后面块读校验的问题。 */
                if (GD25Q_ReadBuffer(h4, FW_SLOT0_BASE + FW_HEAD_SIZE, 4u)) {
                    exp_b = (uint32_t)h4[0] | ((uint32_t)h4[1] << 8) |
                            ((uint32_t)h4[2] << 16) | ((uint32_t)h4[3] << 24);
                }
                if (GD25Q_ReadBuffer(h4, FW_SLOT1_BASE + FW_HEAD_SIZE, 4u)) {
                    exp_a = (uint32_t)h4[0] | ((uint32_t)h4[1] << 8) |
                            ((uint32_t)h4[2] << 16) | ((uint32_t)h4[3] << 24);
                }
                (void)swd_mem_read32(DUT_BOOT_ADDR, &got_b);
                (void)swd_mem_read32(DUT_APP_ADDR, &got_a);
                LOGI("swd: word0 boot exp=%x got=%x, app exp=%x got=%x",
                     exp_b, got_b, exp_a, got_a);

                swdp_select_image(0u);
                s_state = SWDP_ST_VERIFY;
            }
            break;
        }
        {
            uint16_t chunk = (uint16_t)swdp_min_u32(remain, (uint32_t)SWDP_CHUNK_BYTES);

            if (!GD25Q_ReadBuffer(s_buf, s_slot_base + FW_HEAD_SIZE + s_offset, chunk)) {
                /* SPI 读不到镜像源：绝不能把缓冲区里的垃圾写进被烧板 */
                swdp_fail(APP_STATUS_ERR_FLASH, 0xFFu);
                break;
            }
            if (s_offset == 0u) {
                LOGI("swd: prog src0=%x",
                     (unsigned int)((uint32_t)s_buf[0] | ((uint32_t)s_buf[1] << 8) |
                                    ((uint32_t)s_buf[2] << 16) | ((uint32_t)s_buf[3] << 24)));
            }
            r = f1_program_halfwords(s_dst_addr + s_offset, s_buf, (uint32_t)chunk);
            if (r != SWD_OK) {
                swdp_fail(APP_STATUS_ERR_PROG, (uint8_t)(-r));
                break;
            }
            s_offset += chunk;
        }
        break;
    }

    /* ── 5. 回读校验：SWD 读回的是目标 AHB 上的真实内容，可信 ── */
    case SWDP_ST_VERIFY: {
        uint32_t remain = (s_offset < s_img_len) ? (s_img_len - s_offset) : 0u;

        if (remain == 0u) {
            if (s_phase == 0u) {
                swdp_select_image(1u);
            } else {
                s_state = SWDP_ST_GO;
            }
            break;
        }
        {
            uint16_t chunk = (uint16_t)swdp_min_u32(remain, (uint32_t)SWDP_CHUNK_BYTES);

            if (!GD25Q_ReadBuffer(s_buf, s_slot_base + FW_HEAD_SIZE + s_offset, chunk)) {
                swdp_fail(APP_STATUS_ERR_FLASH, 0xFFu);
                break;
            }
            if (s_offset == 0u) {
                LOGI("swd: ver src0=%x",
                     (unsigned int)((uint32_t)s_buf[0] | ((uint32_t)s_buf[1] << 8) |
                                    ((uint32_t)s_buf[2] << 16) | ((uint32_t)s_buf[3] << 24)));
            }
            r = f1_verify_image(s_dst_addr + s_offset, s_buf, (uint32_t)chunk);
            if (r != SWD_OK) {
                swdp_fail(APP_STATUS_ERR_PROG, (uint8_t)(-r));
                break;
            }
            s_offset += chunk;
        }
        break;
    }

    /* ── 6. 复位运行（不需要 NRST：用 AIRCR 的 SYSRESETREQ） ── */
    case SWDP_ST_GO:
        /* 先清 VC_CORERESET：不清的话复位后核会停在复位向量、目标跑不起来 */
        (void)swd_mem_write32(CM_DEMCR, 0u);
#if (SWD_USE_DUT_NRST != 0)
        /* 烧录完成：BOOT0 拉低，清掉调试停核，再用 NRST 复位让新程序跑起来 */
        dut_boot_drive(0u);
        (void)cm_run();
        dut_nrst_drive(1u);
        HAL_Delay(SWD_RESET_HOLD_MS);
        dut_nrst_drive(0u);
        HAL_Delay(5);
        r = SWD_OK;
#else
        r = cm_reset_and_run();
#endif
        if (r != SWD_OK) {
            swdp_fail(APP_STATUS_ERR_PROG, (uint8_t)(-r));
            break;
        }
        LOGI("swd: done, %d+%d bytes in %d ms", (int)s_img_len_boot, (int)s_img_len_app,
             (int)(HAL_GetTick() - s_t_start));
        s_result = APP_STATUS_OK;
        s_state  = SWDP_ST_OK;
        break;

    default:
        break;
    }
}

/* 读 AHB-AP 的 IDR（AP 地址 0xFC）：需要把 SELECT.APBANKSEL 切到 0xF。
 * 读到 0x14770011（F105/Cortex-M3）或 0x24770011（F4/Cortex-M4）= AP 可达；
 * 读不到 = AP 根本进不去。读完把 AP 选择置为"未选"，下一次 memap_prepare
 * 会重新选 bank0 并重写 CSW。 */
static int swd_read_ap_idr(uint32_t *idr)
{
    uint32_t dummy = 0u;
    int      r = swd_select_ap(0u, 0x0Fu);

    if (r != SWD_OK) {
        return r;
    }
    s_apok = 0u;
    s_csw  = 0u;
    /* AP 读是 posted：第一笔 AP 读只是发起访问、回的是上一次的结果，
     * 真正 IDR 要再读一次 DP RDBUFF 才拿得到（否则固定读到 0xFFFFFFFF）。 */
    r = swd_ap_read(SWD_AP_DRW, &dummy);
    if (r != SWD_OK) {
        return r;
    }
    return swd_dp_read(SWD_DP_RDBUFF, idr);
}

/* 一次失败后把链路拉回确定状态：线复位 → JTAG→SWD → 重新上电 → 重选 AP。
 * 不做这一步的话，第一次 NO_ACK 会让后面每一项都跟着失败，逐项错误码就全变 1 了。 */
static void swdp_link_resync(void)
{
    uint32_t id = 0u;

    /* 用完整的 connect 序列：线复位 → 0xE79E → 8 拍 0 → **读 IDCODE**。
     * 只做前三步、不读 IDCODE 时，擦除/编程后紧接着写 SELECT 容易 NO_ACK
     * （实测 req=B1 ack=7）。 */
    (void)swd_connect(&id);
    (void)swd_power_up();
    (void)swd_mem_init();
}

/* RAM 写读自检：证明"写"这条路是否可用（只写 SRAM，绝不碰 flash）。
 * 单字写读 + 4 字块读两条路径都测：烧录校验用的是块读，必须单独证明它没
 * 有地址错位（AP 读是 posted，块读的第 1 个字要靠 dummy 读把流水线推出来）。 */
static int swdp_ram_test(uint8_t *matched)
{
    static const uint32_t pat[4] = {
        0x11223344u, 0x55667788u, 0x99AABBCCu, 0xDDEEFF01u
    };
    uint32_t v = 0u;
    uint32_t rd[4];
    uint32_t i;
    int      r;

    *matched = 0u;

    r = swd_mem_write32(0x20000000u, 0x55AA55AAu);
    if (r != SWD_OK) {
        return r;
    }
    r = swd_mem_read32(0x20000000u, &v);
    if (r != SWD_OK) {
        return r;
    }
    if (v != 0x55AA55AAu) {
        LOGE("swd: ram single exp=55AA55AA got=%x", (unsigned int)v);
        return SWD_OK;
    }

    for (i = 0u; i < 4u; i++) {
        r = swd_mem_write32(0x20000000u + i * 4u, pat[i]);
        if (r != SWD_OK) {
            return r;
        }
    }
    r = swd_mem_read_block(0x20000000u, rd, 4u);
    if (r != SWD_OK) {
        return r;
    }
    for (i = 0u; i < 4u; i++) {
        if (rd[i] != pat[i]) {
            LOGE("swd: ram block[%d] exp=%x got=%x",
                 (int)i, (unsigned int)pat[i], (unsigned int)rd[i]);
            return SWD_OK;
        }
    }

    *matched = 1u;
    return SWD_OK;
}

/*==============================================================================
 * §7 自检（0x0013）：不擦不写；BOOT0=1 保持 100ms + NRST 20ms 进 bootloader
 *----------------------------------------------------------------------------
 * 用途：上板第一件事——证明线通、速度对、认得目标；再去看烧录。
 * 返回 IDCODE / DEV_ID / Flash 容量 / 读保护 / Boot/App 基址的栈顶字。
 *============================================================================*/
uint8_t SWDP_Probe(swd_probe_t *out)
{
    uint32_t v = 0u;
    int      r;

    if (out == 0) {
        return APP_STATUS_ERR_PARAM;
    }

    out->status   = APP_STATUS_OK;
    out->swd_err  = 0u;
    out->rdprt    = 0u;
    out->delay    = (uint16_t)s_delay_loops;
    out->dev_id   = 0u;
    out->flash_kb = 0u;
    out->idcode   = 0u;
    out->boot_sp  = 0u;
    out->app_sp   = 0u;
    out->mem_err  = 0u;
    out->ap_idr   = 0u;
    out->dp_stat  = 0u;
    out->err_dev  = 0u;
    out->err_kb   = 0u;
    out->err_obr  = 0u;
    out->err_boot = 0u;
    out->err_app  = 0u;
    out->err_idr  = 0u;
    out->err_dpstat = 0u;
    out->ram_ok   = 0u;
    out->err_ram  = 0u;

    swd_phy_init();
#if (SWD_USE_DUT_NRST != 0)
    /* 自检：BOOT0 高保持 SWD_BOOT_SETUP_MS，再 NRST 拉低 SWD_RESET_HOLD_MS，
     * 释放后 DUT 进入系统 bootloader，再做 SWD 连接。 */
    SWDP_DutCtrlInit();
    dut_boot_drive(1u);
    HAL_Delay(SWD_BOOT_SETUP_MS);
    dut_nrst_drive(1u);
    HAL_Delay(SWD_RESET_HOLD_MS);
    dut_nrst_drive(0u);
    HAL_Delay(5);
    LOGI("swd: boot high %dms, nrst %dms pulse",
         (int)SWD_BOOT_SETUP_MS, (int)SWD_RESET_HOLD_MS);
#endif

    r = swd_connect(&out->idcode);
    if (r != SWD_OK) {
        out->status  = APP_STATUS_ERR_PROG;
        out->swd_err = (uint8_t)(-r);
        return out->status;
    }
    r = swd_power_up();
    if (r != SWD_OK) {
        out->status  = APP_STATUS_ERR_PROG;
        out->swd_err = (uint8_t)(-r);
        return out->status;
    }
    r = swd_mem_init();
    if (r != SWD_OK) {
        out->status  = APP_STATUS_ERR_PROG;
        out->swd_err = (uint8_t)(-r);
        return out->status;
    }

    /* 逐项读，每项单独记错误码；失败一次就先线复位重同步，保证下一项从干净状态开始。
     *   1 = NO_ACK / 2 = WAIT / 3 = FAULT / 4 = 奇偶错 / 6 = 超时
     * 这样能区分"第一个读就挂"和"只有某一项挂"——两者原因完全不同。 */
    r = cm_read_dev_id(&out->dev_id);
    out->err_dev = (uint8_t)(-r);
    if (r != SWD_OK) {
        swdp_link_resync();
    }

    r = f1_read_flash_kb(&out->flash_kb);
    out->err_kb = (uint8_t)(-r);
    if (r != SWD_OK) {
        swdp_link_resync();
    }

    v = 0u;
    r = f1_read_obr(&v);
    out->err_obr = (uint8_t)(-r);
    if (r == SWD_OK) {
        out->rdprt = ((v & F1_OBR_RDPRT) != 0u) ? 1u : 0u;
    } else {
        swdp_link_resync();
    }

    r = swd_mem_read32(DUT_BOOT_ADDR, &out->boot_sp);
    out->err_boot = (uint8_t)(-r);
    if (r != SWD_OK) {
        swdp_link_resync();
    }

    r = swd_mem_read32(DUT_APP_ADDR, &out->app_sp);
    out->err_app = (uint8_t)(-r);
    if (r != SWD_OK) {
        swdp_link_resync();
    }

    r = swd_read_ap_idr(&out->ap_idr);
    out->err_idr = (uint8_t)(-r);
    if (r != SWD_OK) {
        swdp_link_resync();
    }

    r = swd_dp_read(SWD_DP_CTRLSTAT, &out->dp_stat);
    out->err_dpstat = (uint8_t)(-r);

    /* 最后再做写通路自检，避免它影响前面的读数 */
    r = swdp_ram_test(&out->ram_ok);
    out->err_ram = (uint8_t)(-r);

    /* mem_err 保留为"第一个失败的读"，兼容旧解码 */
    out->mem_err = out->err_dev;
    if (out->mem_err == 0u) out->mem_err = out->err_kb;
    if (out->mem_err == 0u) out->mem_err = out->err_obr;
    if (out->mem_err == 0u) out->mem_err = out->err_boot;
    if (out->mem_err == 0u) out->mem_err = out->err_app;
    if (out->mem_err == 0u) out->mem_err = out->err_idr;
    if (out->mem_err == 0u) out->mem_err = out->err_dpstat;

    LOGI("swd probe: id=%x dev=%x kb=%d rdp=%d", out->idcode, (int)out->dev_id,
         (int)out->flash_kb, (int)out->rdprt);
    LOGI("swd probe err: dev=%d kb=%d obr=%d boot=%d app=%d idr=%d dp=%d",
         (int)out->err_dev, (int)out->err_kb, (int)out->err_obr,
         (int)out->err_boot, (int)out->err_app, (int)out->err_idr,
         (int)out->err_dpstat);
    LOGI("swd probe sp: boot=%x app=%x", out->boot_sp, out->app_sp);
    return APP_STATUS_OK;
}

#endif /* !FLASH_MODE_UART5 */
