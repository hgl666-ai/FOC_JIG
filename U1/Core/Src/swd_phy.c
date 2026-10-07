/*
 * SWD 物理层实现 —— GPIO 位翻转
 *
 * 两种构建：
 *   SWD_SIM_BUILD  仿真：底层原语走 HAL 替身，sim_hal.c 把每次 GPIO 动作
 *                  翻译成目标模型能看见的 SWCLK 边沿 / SWDIO 事件。
 *   目标构建       直写 BSRR / CRL，配合 SWD_DELAY_LOOPS 控制速率。
 * 两种构建下 **上层逻辑完全相同**，因此仿真的结论对固件有效。
 */
#include "swd_config.h"
#include "swd_phy.h"

/* ================================================================== */
/* 1. 底层原语（唯一与构建相关的部分）                                 */
/* ================================================================== */
#if defined(SWD_SIM_BUILD)

static uint8_t s_sim_dio_out = 0u;   /* 当前是否处于输出态 */

static void phy_clk_set(uint8_t level)
{
    HAL_GPIO_WritePin(SWD_CLK_PORT, SWD_CLK_PIN,
                      level ? GPIO_PIN_SET : GPIO_PIN_RESET);
}

static void phy_dio_dir(uint8_t output)
{
    if (s_sim_dio_out == output) {
        return;                       /* 只在变化时通知模型 */
    }
    s_sim_dio_out = output;
    {
        GPIO_InitTypeDef init;
        init.Pin   = SWD_DIO_PIN;
        init.Mode  = output ? GPIO_MODE_OUTPUT_PP : GPIO_MODE_INPUT;
        init.Pull  = output ? GPIO_NOPULL : GPIO_PULLUP;
        init.Speed = GPIO_SPEED_FREQ_HIGH;
        HAL_GPIO_Init(SWD_DIO_PORT, &init);
    }
}

static void phy_dio_write(uint8_t level)
{
    HAL_GPIO_WritePin(SWD_DIO_PORT, SWD_DIO_PIN,
                      level ? GPIO_PIN_SET : GPIO_PIN_RESET);
}

static uint8_t phy_dio_read(void)
{
    return (HAL_GPIO_ReadPin(SWD_DIO_PORT, SWD_DIO_PIN) == GPIO_PIN_SET) ? 1u : 0u;
}

static void phy_delay(void)
{
    /* 仿真不建模时间 */
}

#else  /* ---------------- 目标构建 ---------------- */

static uint8_t s_dio_out = 0u;

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
 *   输入上拉        = CNF=10, MODE=00 = 0b1000（配合 ODR=1，由 BSRR 预置） */
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

static void phy_delay(void)
{
#if (SWD_DELAY_LOOPS > 0u)
    volatile uint32_t i;
    for (i = 0u; i < SWD_DELAY_LOOPS; i++) {
        __NOP();
    }
#endif
}

#endif /* SWD_SIM_BUILD */

/* 统计（便于上板前后观察实际打了多少个时钟） */
static uint32_t s_clk_count = 0u;
static uint8_t  s_last_ack  = 0u;   /* 最近一次读到的 3 位 ACK，诊断用 */

/* ================================================================== */
/* 2. 上层：与构建无关                                                 */
/* ================================================================== */

static void phy_clk_cycle(void)
{
    phy_clk_set(1u);
    phy_delay();
    phy_clk_set(0u);
    phy_delay();
    s_clk_count++;
}

void SWD_PhyInit(void)
{
#if defined(SWD_SIM_BUILD)
    {
        GPIO_InitTypeDef init;
        init.Pin = SWD_CLK_PIN;  init.Mode = GPIO_MODE_OUTPUT_PP;
        init.Pull = GPIO_NOPULL; init.Speed = GPIO_SPEED_FREQ_HIGH;
        HAL_GPIO_Init(SWD_CLK_PORT, &init);
        init.Pin = SWD_NRST_PIN; init.Mode = GPIO_MODE_OUTPUT_PP;
        HAL_GPIO_Init(SWD_NRST_PORT, &init);
    }
    s_sim_dio_out = 0u;              /* 置 0 以便下一步真正切换方向 */
#else  /* ---------------- 目标构建 ---------------- */
    /* 三个端口时钟全开：引脚是占位值，将来换到哪个端口都不用改这里 */
    __HAL_RCC_GPIOA_CLK_ENABLE();
    __HAL_RCC_GPIOB_CLK_ENABLE();
    __HAL_RCC_GPIOC_CLK_ENABLE();
    {
        GPIO_InitTypeDef init;
        init.Pin = SWD_CLK_PIN;
        init.Mode = GPIO_MODE_OUTPUT_PP;
        init.Pull = GPIO_NOPULL;
        init.Speed = GPIO_SPEED_FREQ_HIGH;
        HAL_GPIO_Init(SWD_CLK_PORT, &init);
#if (SWD_USE_NRST != 0)
        init.Pin = SWD_NRST_PIN;
        HAL_GPIO_Init(SWD_NRST_PORT, &init);
#endif
    }
    s_dio_out = 0u;
#endif
    phy_clk_set(0u);                     /* SWCLK 空闲低 */
    phy_dio_dir(1u);                     /* SWDIO 由主机驱动 */
    phy_dio_write(SWD_IDLE_LEVEL);       /* 空闲电平（ADIv5：低） */
    SWD_PhyNrst(0u);                     /* 不拉复位 */
}

void SWD_PhyNrst(uint8_t assert_low)
{
#if (SWD_USE_NRST == 0)
    /* 本方案不接 NRST：烧录不依赖复位线（烧完靠 AIRCR SYSRESETREQ 复位运行） */
    (void)assert_low;
#else
    /* NRST 低有效 */
#if defined(SWD_SIM_BUILD)
    HAL_GPIO_WritePin(SWD_NRST_PORT, SWD_NRST_PIN,
                      assert_low ? GPIO_PIN_RESET : GPIO_PIN_SET);
#else
    if (assert_low) {
        SWD_NRST_PORT->BSRR = (uint32_t)SWD_NRST_PIN << 16;
    } else {
        SWD_NRST_PORT->BSRR = (uint32_t)SWD_NRST_PIN;
    }
#endif
#endif /* SWD_USE_NRST */
}

uint8_t SWD_ParityOf(uint32_t v)
{
    v ^= v >> 16;
    v ^= v >> 8;
    v ^= v >> 4;
    v ^= v >> 2;
    v ^= v >> 1;
    return (uint8_t)(v & 1u);
}

void SWD_WriteBit(uint8_t b)
{
    phy_dio_dir(1u);            /* 输出 */
    phy_dio_write(b ? 1u : 0u); /* 在下降沿(时钟低)期间更新 */
    phy_clk_cycle();            /* 上升沿：目标采样 */
}

uint8_t SWD_ReadBit(void)
{
    uint8_t b;
    phy_clk_set(1u);            /* 上升沿 */
    phy_delay();
    b = phy_dio_read();         /* 主机在上升沿采样 */
    phy_clk_set(0u);
    phy_delay();
    s_clk_count++;
    return b;
}

void SWD_WriteBits(uint32_t v, uint8_t n)
{
    uint8_t i;
    for (i = 0u; i < n; i++) {
        SWD_WriteBit((uint8_t)((v >> i) & 1u));   /* LSB first */
    }
}

uint32_t SWD_ReadBits(uint8_t n)
{
    uint32_t v = 0u;
    uint8_t  i;
    for (i = 0u; i < n; i++) {
        v |= (uint32_t)SWD_ReadBit() << i;        /* 第一位进 bit0 */
    }
    return v;
}

void SWD_Turnaround(void)
{
    phy_dio_dir(0u);            /* 释放总线 */
    phy_clk_cycle();
}

/*
 * 空闲周期：主机把 SWDIO 驱动到空闲电平并打一个时钟。
 * 作用：让下一个请求包的 Start 位(1) 一定表现为 **上升沿**，
 * 目标侧才能无歧义地同步到包头（ADIv5 规定空闲周期由主机把线拉低）。
 */
void SWD_IdleClock(void)
{
    uint8_t i;
    phy_dio_dir(1u);
    phy_dio_write(SWD_IDLE_LEVEL);
    for (i = 0u; i < SWD_TAIL_IDLE_CYCLES; i++) {
        phy_clk_cycle();
    }
}

void SWD_LineReset(void)
{
    uint8_t i;
    phy_dio_dir(1u);
    phy_dio_write(1u);
    for (i = 0u; i < 56u; i++) {   /* ADIv5 要求 ≥50 个高电平周期 */
        phy_clk_cycle();
    }
}

/* 事务尾部的空闲周期：turnaround(可选) + N 个空闲时钟（主机驱动空闲电平） */
static void swd_idle_tail(uint8_t turnaround)
{
    uint8_t i;
    for (i = 0u; i < turnaround; i++) {
        SWD_Turnaround();
    }
    for (i = 0u; i < SWD_TAIL_IDLE_CYCLES; i++) {
        SWD_WriteBit(SWD_IDLE_LEVEL);
    }
}

/* 协议错误（没读到有效 ACK）时的退避：
 * 如果上一笔事务是中途被打断的，DP 仍在等一个数据阶段（32 数据 + 1 校验 = 33 拍）。
 * 这里按 DAPLink 的做法把这 33 拍补掉，让 DP 重新对齐，否则下一笔事务会错位解析。 */
static void swd_error_backoff(void)
{
    uint8_t i;
    for (i = 0u; i < (SWD_TA_TAIL + 33u); i++) {
        phy_clk_cycle();
    }
    phy_dio_dir(1u);
    phy_dio_write(1u);          /* 空闲期间把线拉高 */
}

int SWD_Transfer(uint8_t apndp, uint8_t rnw, uint8_t addr, uint32_t *data)
{
    uint8_t  req;
    uint8_t  a2 = (uint8_t)((addr >> 2) & 1u);
    uint8_t  a3 = (uint8_t)((addr >> 3) & 1u);
    uint8_t  par;
    uint8_t  ack;
    uint8_t  i;

    apndp = (uint8_t)(apndp & 1u);
    rnw   = (uint8_t)(rnw & 1u);
    par   = SWD_ParityOf((uint32_t)(apndp | (rnw << 1) | (a2 << 2) | (a3 << 3)));

    req  = 0x81u;                                        /* Start=1, Park=1, Stop=0 */
    req |= (uint8_t)(apndp << 1);
    req |= (uint8_t)(rnw << 2);
    req |= (uint8_t)(a2 << 3);
    req |= (uint8_t)(a3 << 4);
    req |= (uint8_t)(par << 5);

    SWD_WriteBits(req, 8u);

    /* 请求头 → 应答：方向要变，必须有 turnaround（ADIv5 §B4.1.1） */
    for (i = 0u; i < SWD_TA_AFTER_HEADER; i++) {
        SWD_Turnaround();
    }
    phy_dio_dir(0u);                                     /* 确保已释放 */

    ack = (uint8_t)SWD_ReadBits(3u);
    s_last_ack = ack;

    if (ack != SWD_ACK_OK) {
        if (ack == SWD_ACK_WAIT) {
            /* 目标忙：数据阶段不进行（我们没开 ORUNDETECT），上层会重试 */
            swd_idle_tail(SWD_TA_TAIL);
            return SWD_ERR_WAIT;
        }
        if (ack == SWD_ACK_FAULT) {
            swd_idle_tail(SWD_TA_TAIL);
            return SWD_ERR_FAULT;
        }
        /* 没有任何有效 ACK 位（线断/目标没上电/上笔事务被打断） */
        swd_error_backoff();
        return SWD_ERR_NO_ACK;
    }

    if (rnw) {
        uint32_t v;
        uint8_t  p;
        volatile uint8_t ta_rd = (uint8_t)SWD_TA_BEFORE_READ_DATA;

        /* 读：ACK 与 RDATA 都是目标驱动，**方向没变，不需要 turnaround**（§B4.1.1） */
        phy_dio_dir(0u);
        for (i = 0u; i < ta_rd; i++) {
            phy_clk_cycle();
        }
        v = SWD_ReadBits(32u);
        p = SWD_ReadBit();

        /* RDATA 之后要换方向：turnaround + 主机接管 + 空闲周期 */
        swd_idle_tail(SWD_TA_TAIL);
        if (p != SWD_ParityOf(v)) {
            return SWD_ERR_PARITY;
        }
        if (data != 0) {
            *data = v;
        }
    } else {
        uint32_t v = (data != 0) ? *data : 0u;

        /* 写：ACK(目标驱动) → WDATA(主机驱动)，中间要一个 turnaround */
        for (i = 0u; i < SWD_TA_AFTER_ACK_WRITE; i++) {
            SWD_Turnaround();
        }
        SWD_WriteBits(v, 32u);
        SWD_WriteBit(SWD_ParityOf(v));

        /* ADIv5 §B4.1.1：WDATA 之后主机继续驱动，**没有额外的 turnaround**，
         * 只需补空闲周期。第一版在这里多打了一个 turnaround（白白多 1 拍）。 */
        for (i = 0u; i < SWD_TAIL_IDLE_CYCLES; i++) {
            SWD_WriteBit(SWD_IDLE_LEVEL);
        }
    }

    return SWD_OK;
}

uint32_t SWD_PhyClockCount(void)
{
    return s_clk_count;
}

uint8_t SWD_PhyLastAck(void)
{
    return s_last_ack;
}
