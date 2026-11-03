#include "gd25q128.h"
#include "spi.h"
#include "main.h"

/*===== 片选: PB12 (CubeMX 把它配成 SPI2_NSS/AF_PP, 这里显式改回普通推挽输出) =====*/
#define FLASH_CS_PORT       GPIOB
#define FLASH_CS_PIN        GPIO_PIN_12

#define CS_LOW()    HAL_GPIO_WritePin(FLASH_CS_PORT, FLASH_CS_PIN, GPIO_PIN_RESET)
#define CS_HIGH()   HAL_GPIO_WritePin(FLASH_CS_PORT, FLASH_CS_PIN, GPIO_PIN_SET)

/*===== GD25Q 指令 =====*/
#define CMD_WRITE_ENABLE    0x06u
#define CMD_READ_STATUS1    0x05u
#define CMD_WRITE_STATUS1   0x01u
#define CMD_WRITE_STATUS2   0x31u
#define CMD_READ_DATA       0x03u
#define CMD_PAGE_PROGRAM    0x02u
#define CMD_SECTOR_ERASE    0x20u
#define CMD_JEDEC_ID        0x9Fu
#define CMD_ENABLE_RESET    0x66u
#define CMD_RESET           0x99u

/*==============================================================================
 * SPI 单字节超时 + 失败计数
 *----------------------------------------------------------------------------
 * 原实现写死 1000ms 且**完全不看返回值**，后果（2026-10-28 实测）：
 *   SPI 总线不通时，每个字节白等 1 秒；一次 0x000C 查询要读 2 槽 × 12 字节
 *   = 24 次 → 整整 **24 秒**才回，而且读回全 0 被当成"空槽"，报出误导性的
 *   `06 ERR_NO_FW`（真实原因是"Flash 根本读不到"）。
 *
 * 现在改成：5ms 超时 + 失败计数。总线不通时：
 *   - 每个字节 5ms 就放弃（不再 1s）
 *   - 上层通过 GD25Q_ErrCount() 的增量立刻发现，回 `03 ERR_FLASH`
 * 注意 5ms 对正常读/写完全够用：SPI2 在 36MHz/4 = 9MHz 下，单字节 ≈1.1µs。
 *============================================================================*/
#define GD25Q_SPI_TIMEOUT_MS   5u

static volatile uint32_t s_spi_err;     /* SPI 传输失败累计（只增不清零, 用于判增量） */

/* 累计失败次数。调用方在操作前后各取一次, 相等 = 这次操作全程 SPI 正常 */
uint32_t GD25Q_ErrCount(void)
{
    return s_spi_err;
}

static uint8_t SPI_Swap(uint8_t tx)
{
    uint8_t rx = 0;

    if (HAL_SPI_TransmitReceive(&hspi2, &tx, &rx, 1, GD25Q_SPI_TIMEOUT_MS) != HAL_OK) {
        s_spi_err++;                    /* 总线不通/超时: 计数, 不再静默 */
    }
    return rx;
}

/* 把 PB12 从 CubeMX 的 AF_PP 改回普通推挽输出, 保证 HAL_GPIO_WritePin 真正生效 */
void GD25Q_Init(void)
{
    GPIO_InitTypeDef io = {0};
    io.Pin   = FLASH_CS_PIN;
    io.Mode  = GPIO_MODE_OUTPUT_PP;
    io.Pull  = GPIO_NOPULL;
    io.Speed = GPIO_SPEED_FREQ_HIGH;
    HAL_GPIO_Init(FLASH_CS_PORT, &io);
    CS_HIGH();
}

static void GD25Q_WriteEnable(void)
{
    CS_LOW();
    SPI_Swap(CMD_WRITE_ENABLE);
    CS_HIGH();
}

uint8_t GD25Q_WaitForWriteEnd(void)
{
    uint8_t  status = 0;
    uint32_t tick   = HAL_GetTick();
    uint32_t err0   = s_spi_err;

    CS_LOW();
    SPI_Swap(CMD_READ_STATUS1);
    do {
        status = SPI_Swap(0xFF);
        if (s_spi_err != err0) {                /* SPI 总线不通: 立即退出 */
            CS_HIGH();                          /* 否则会在 3s 里空转几百次 */
            return 0;
        }
        if ((HAL_GetTick() - tick) > 3000u) {   /* 扇区擦除最长需数秒 */
            CS_HIGH();
            return 0;
        }
    } while ((status & 0x01u) == 0x01u);        /* 等 WIP 清零 */
    CS_HIGH();
    return 1;
}

uint8_t GD25Q_ReadSR1(void)
{
    uint8_t sr;
    CS_LOW();
    SPI_Swap(CMD_READ_STATUS1);
    sr = SPI_Swap(0xFF);
    CS_HIGH();
    return sr;
}

/**
  * @brief  读 24 位 JEDEC ID (RDID 0x9F)
  * @retval [23:16]制造商 [15:8]存储类型 [7:0]容量
  * @note   GD25Q128E 数据手册 (Rev1.4) TABLE OF ID DEFINITIONS:
  *           制造商 MID = 0xC8, 存储类型 = 0x40, 容量 = 0x18 (128Mbit)
  *         即完整 ID = 0xC84018。
  *         [已修正] 参考项目原实现先发 3 个 dummy 再读 2 字节, 并与 0xC817 比较——
  *         那读到的是 ID 之后的字节, 在真实芯片上永远不可能相等。
  */
uint32_t GD25Q_ReadID(void)
{
    uint32_t id;
    uint32_t err0 = s_spi_err;

    CS_LOW();
    SPI_Swap(CMD_JEDEC_ID);
    id  = (uint32_t)SPI_Swap(0xFF) << 16;   /* 制造商 ID   */
    id |= (uint32_t)SPI_Swap(0xFF) << 8;    /* 存储类型    */
    id |= (uint32_t)SPI_Swap(0xFF);         /* 容量        */
    CS_HIGH();

    if (s_spi_err != err0) {
        return 0u;                          /* 0 不是合法 JEDEC ID → 调用方按"不在线"处理 */
    }
    return id;
}

/**
  * @brief  自测: 制造商必须是 0xC8 (GigaDevice), 容量必须是 0x18 (128Mbit = 16MB)
  * @retval 1=通过, 0=失败
  */
uint8_t GD25Q_Test(void)
{
    uint32_t id = GD25Q_ReadID();
    return (((id >> 16) & 0xFFu) == 0xC8u && (id & 0xFFu) == 0x18u) ? 1u : 0u;
}

/**
  * @brief 解除块保护
  * @note  BP0-3 若保护低地址区域, 擦除/写入会**静默失败**; 必须在擦写前调用。
  */
void GD25Q_Unprotect(void)
{
    GD25Q_WriteEnable();
    CS_LOW();
    SPI_Swap(CMD_WRITE_STATUS1);
    SPI_Swap(0x00);
    CS_HIGH();
    (void)GD25Q_WaitForWriteEnd();

    GD25Q_WriteEnable();
    CS_LOW();
    SPI_Swap(CMD_WRITE_STATUS2);
    SPI_Swap(0x00);
    CS_HIGH();
    (void)GD25Q_WaitForWriteEnd();
}

uint8_t GD25Q_EraseSector(uint32_t sector_addr)
{
    GD25Q_WriteEnable();
    if (!GD25Q_WaitForWriteEnd()) return 0;

    CS_LOW();
    SPI_Swap(CMD_SECTOR_ERASE);
    SPI_Swap((uint8_t)(sector_addr >> 16));
    SPI_Swap((uint8_t)(sector_addr >> 8));
    SPI_Swap((uint8_t)(sector_addr));
    CS_HIGH();

    return GD25Q_WaitForWriteEnd();
}

uint8_t GD25Q_WritePage(const uint8_t *buf, uint32_t addr, uint16_t len)
{
    /* 跨页安全: 若跨越 256 字节页边界, 自动拆成多次页编程 */
    while (len > 0) {
        uint16_t max_bytes = (uint16_t)(256u - (addr & 0xFFu));
        uint16_t chunk     = (len > max_bytes) ? max_bytes : len;

        GD25Q_WriteEnable();
        if (!GD25Q_WaitForWriteEnd()) return 0;

        CS_LOW();
        SPI_Swap(CMD_PAGE_PROGRAM);
        SPI_Swap((uint8_t)(addr >> 16));
        SPI_Swap((uint8_t)(addr >> 8));
        SPI_Swap((uint8_t)(addr));
        for (uint16_t i = 0; i < chunk; i++) SPI_Swap(buf[i]);
        CS_HIGH();

        if (!GD25Q_WaitForWriteEnd()) return 0;

        addr += chunk;
        buf  += chunk;
        len  -= chunk;
    }
    return 1;
}

uint8_t GD25Q_ReadBuffer(uint8_t *buf, uint32_t addr, uint16_t len)
{
    uint32_t err0 = s_spi_err;

    CS_LOW();
    SPI_Swap(CMD_READ_DATA);
    SPI_Swap((uint8_t)(addr >> 16));
    SPI_Swap((uint8_t)(addr >> 8));
    SPI_Swap((uint8_t)(addr));
    for (uint16_t i = 0; i < len; i++) buf[i] = SPI_Swap(0xFF);
    CS_HIGH();

    /* 1 = 整段读成功; 0 = 期间有 SPI 传输失败(内容不可信, 调用方必须按 ERR_FLASH 处理) */
    return (s_spi_err == err0) ? 1u : 0u;
}

/*==============================================================================
 * 连通性诊断（由 PC 命令 0x001C 调起）
 *----------------------------------------------------------------------------
 * 场景：板上明明有 Flash、VCC 也正常，却一直 ERR_FLASH —— 光靠现象分不清
 *   "外设没在跑" / "引脚被改回 AF 导致 CS 常低" / "芯片不答" / "供电" 哪一条。
 * 这里把 MCU 眼里的现场一次性报出来，不需要示波器：
 *   - CR1 的 SPE/MSTR：外设到底使能没有
 *   - SR 的 MODF：主模式是否被 NSS 冲突打断（这会让 SPE 被硬件清零）
 *   - PB12..PB15 的 GPIO 配置位：CS 是"普通推挽输出"(0x3) 还是"复用推挽"(0xB)
 *   - 重新初始化一次再探：能通就说明"初始化被破坏过"，不能通就是硬件
 *============================================================================*/
uint8_t GD25Q_Diag(uint8_t *out)
{
    uint32_t e0;
    uint32_t err1 = 0u;
    uint32_t err3 = 0u;
    uint32_t id1;
    uint32_t id3 = 0u;
    uint32_t cr1 = 0u;
    uint32_t sr  = 0u;
    uint32_t crh = 0u;
    uint32_t enr = 0u;
    uint32_t apb1_all = 0u;
    uint32_t u3cr1 = 0u;
    uint32_t u3sr  = 0u;
    uint32_t u3brr = 0u;
    uint16_t rb1 = 0u;
    uint16_t rb_spi1 = 0u;
    uint16_t rb3 = 0u;
    uint16_t sr2 = 0u;
    uint16_t rb_cr2 = 0u;
    uint16_t rb_crcpr = 0u;
    uint8_t  cs  = 0u;
    uint8_t  st  = 0u;
    uint8_t  wr1 = 0u;
    uint8_t  wr_spi1 = 0u;
    uint8_t  wr3 = 0u;
    uint8_t  wr_cr2 = 0u;
    uint8_t  wr_crcpr = 0u;
    uint8_t  res;

    if (out == 0) {
        return 3u;
    }

#ifdef JIG_HOST_TEST
    /* 宿主测试没有真寄存器：下面读取它们的 #ifndef 段整段被跳过，显式消告警 */
    (void)enr; (void)rb_spi1; (void)apb1_all; (void)rb3; (void)id3; (void)err3;
    (void)sr2; (void)rb_cr2; (void)rb_crcpr; (void)u3cr1; (void)u3sr; (void)u3brr;
#endif

    /* ── 1) 按当前状态探一次 ── */
    e0   = s_spi_err;
    id1  = GD25Q_ReadID();
    err1 = s_spi_err - e0;

#ifndef JIG_HOST_TEST
    cs  = (HAL_GPIO_ReadPin(GPIOB, GPIO_PIN_12) == GPIO_PIN_SET) ? 1u : 0u;
    st  = (uint8_t)hspi2.State;
    cr1 = (uint32_t)SPI2->CR1;
    sr  = (uint32_t)SPI2->SR;
    crh = (uint32_t)GPIOB->CRH;                 /* PB8..PB15 的配置位 */
    enr = (uint32_t)RCC->APB1ENR;

    /* ★ 只读探针：USART3 是本固件**正在使用**的外设（U2 内部链路）。
     *   低密度(C6) 与中密度(C8) 的外设集合本就不同，用它能判断这颗 die 有没有被裁剪：
     *     USART3->CR1 非 0（如 0x202C）→ 这颗 die 有 USART3（中密度特征）
     *     USART3->CR1 = 0            → 连 USART3 都没有 → 真正的低密度 die
     *   只读，不写任何 USART3 寄存器，不会影响 U2 链路。 */
    u3cr1 = (uint32_t)USART3->CR1;
    u3sr  = (uint32_t)USART3->SR;
    u3brr = (uint32_t)USART3->BRR;
#endif

    /* ── 2) 写回读：往 CR1 写一个已知值再读回来 ──
     *   能读回 = 这块寄存器电路是活的；读回 0 = 写都写不进去。
     *   ⚠ 只碰 SPI2 的 CR1，最后写回 0 关掉；**不碰 GPIO、不调 HAL_SPI_Init**
     *     （2026-10-28 那次就是重初始化把 PB12 打成复用、把 MCU 卡死的）。 */
#ifndef JIG_HOST_TEST
    SPI2->CR1 = 0x0346u;                        /* SSM|SSI|MSTR|SPE|BR/2 */
    rb1       = (uint16_t)SPI2->CR1;
    SPI2->CR1 = 0x0000u;                        /* 立即关闭, 不留副作用 */
    wr1 = (rb1 == 0x0346u) ? 1u : 0u;

    /* ── 3) 对照实验：同一套代码测 **SPI1**（APB2，任何 F103 都有）──
     *   为什么必须有这一步：换了两颗 C6T6 结果字节完全一样 ⇒ 系统性原因。
     *   要么"这颗片子没有 SPI2"，要么"我们的测法/时钟假设有问题"。
     *   SPI1 是判定基准：
     *     SPI1 写回读成功、SPI2 失败 → 测法没问题，问题就在 SPI2 本体（型号/批次）
     *     两个都失败               → 是测法或时钟假设的问题（我继续查）
     *   只开 SPI1 的时钟 + 写一次 CR1 再读回，**不配引脚、不写 DR、不发任何时序**，
     *   结束后恢复 APB2ENR 原值。 */
    {
        uint32_t apb2_old = (uint32_t)RCC->APB2ENR;

        RCC->APB2ENR = apb2_old | (1u << 12);   /* SPI1EN */
        SPI1->CR1    = 0x0346u;
        rb_spi1      = (uint16_t)SPI1->CR1;
        SPI1->CR1    = 0x0000u;
        RCC->APB2ENR = apb2_old;                /* 恢复原状 */
        wr_spi1 = (rb_spi1 == 0x0346u) ? 1u : 0u;
    }

    /* ── 4) 兜底：把 APB1ENR **全部位置 1** 后再试 SPI2 ──
     *   排除最后一种可能："这颗片子的 RCC 位映射和标准不同，SPI2 的时钟门不在 bit14"。
     *   顺手回读 APB1ENR：读回值里**没置起来的位**= 这颗片子没实现的外设
     *   （对识别克隆片/裁剪型号很有用）。
     *   安全：开的是外设时钟，不是外设本身；这些外设没配置、没开中断，不会输出任何东西。
     *   结束后把 APB1ENR 恢复原值。 */
    {
        uint32_t apb1_old = (uint32_t)RCC->APB1ENR;
        volatile uint32_t warm;

        RCC->APB1ENR = 0xFFFFFFFFu;
        apb1_all     = (uint32_t)RCC->APB1ENR;      /* 回读：哪些位真的存在 */
        for (warm = 0u; warm < 200u; warm++) { }    /* 让时钟稳定几个周期 */

        SPI2->CR1 = 0x0346u;
        rb3       = (uint16_t)SPI2->CR1;
        wr3 = (rb3 == 0x0346u) ? 1u : 0u;

        if (wr3 != 0u) {
            /* 寄存器能写了 → CR1 保持已配置状态，直接试读一次 JEDEC ID */
            e0   = s_spi_err;
            id3  = GD25Q_ReadID();
            err3 = s_spi_err - e0;
        }
        SPI2->CR1    = 0x0000u;                     /* 收尾关闭 */
        RCC->APB1ENR = apb1_old;                    /* 恢复原状 */
    }

    /* ── 5) 多寄存器验证：万一 CR1 有一点特殊，把别的寄存器也测一遍 ──
     *   ① 写 SPE=1 之后读 SR：**活着的 SPI 此刻 TXE(bit1) 必然=1**（只读状态位，
     *      不依赖任何写操作）——这是最硬的独立判据。
     *   ② / ③ CR2、CRCPR 各自写一个已知值再读回。
     *   CRCPR 的复位默认值就是 0x0007，所以"读到 0x0007"= 寄存器真的存在。 */
#ifndef JIG_HOST_TEST
    SPI2->CR1 = 0x0346u;                            /* SSM|SSI|MSTR|SPE */
    sr2       = (uint16_t)SPI2->SR;                 /* ← 期望 TXE=1 (0x0002) */
    SPI2->CR1 = 0x0000u;

    SPI2->CR2   = 0x0007u;                          /* RXDMAEN|TXDMAEN|SSOE */
    rb_cr2      = (uint16_t)SPI2->CR2;
    SPI2->CR2   = 0x0000u;
    wr_cr2 = (rb_cr2 == 0x0007u) ? 1u : 0u;

    SPI2->CRCPR = 0x0007u;                          /* 复位默认值 */
    rb_crcpr    = (uint16_t)SPI2->CRCPR;
    SPI2->CRCPR = 0x0007u;                          /* 保持默认值, 不改行为 */
    wr_crcpr = (rb_crcpr == 0x0007u) ? 1u : 0u;
#endif
#endif

    /* ── 4) 判定 ── */
    if (err1 != 0u) {
        res = 1u;                               /* 总线超时：传输根本没完成 */
    } else if ((((id1 >> 16) & 0xFFu) == 0xC8u) && ((id1 & 0xFFu) == 0x18u)) {
        res = 0u;                               /* GigaDevice 128Mbit，正常 */
    } else {
        res = 2u;                               /* 通了但 ID 不对（0xFFFFFF = 没芯片应答）*/
    }

    out[0]  = res;
    out[1]  = (uint8_t)err1;
    out[2]  = (uint8_t)(id1 >> 24);
    out[3]  = (uint8_t)(id1 >> 16);             /* 制造商 */
    out[4]  = (uint8_t)(id1 >> 8);              /* 类型   */
    out[5]  = (uint8_t)id1;                     /* 容量   */
    out[6]  = cs;
    out[7]  = st;
    out[8]  = (uint8_t)(cr1 >> 8);              /* CR1 高字节: bit6=SPE bit2=MSTR */
    out[9]  = (uint8_t)cr1;
    out[10] = 0u;
    out[11] = 0u;
    out[12] = (uint8_t)(sr >> 8);               /* SR 高字节: bit7=BSY, 低字节 bit5=MODF */
    out[13] = (uint8_t)sr;
    out[14] = 0u;
    out[15] = 0u;
    /* PB12..PB15 的 4bit 配置: 0x3=GP推挽输出 0xB=AF推挽 0x4=浮空输入 0x8=上下拉输入 */
    out[16] = (uint8_t)((crh >> 16) & 0x0Fu);   /* PB12 (CS)  ← 期望 0x3 */
    out[17] = (uint8_t)((crh >> 20) & 0x0Fu);   /* PB13 (SCK) ← 期望 0xB */
    out[18] = (uint8_t)((crh >> 24) & 0x0Fu);   /* PB14 (MISO)← 期望 0x4 */
    out[19] = (uint8_t)((crh >> 28) & 0x0Fu);   /* PB15 (MOSI)← 期望 0xB */
    out[20] = wr1;                              /* SPI2 写回读: 0=写不进 1=写进去了 */
    out[21] = (uint8_t)(rb1 >> 8);              /* 写 0x0346 后读回的 CR1 高字节 */
    out[22] = (uint8_t)rb1;                     /* 写 0x0346 后读回的 CR1 低字节 */
    out[23] = wr_spi1;                          /* **SPI1 对照组**写回读: 0/1 */

    /* ── 5) 时钟/总线对照 ──
     *   APB1ENR.bit14 = SPI2EN：为 1 说明"时钟已经请求了"
     *   USART1->CR1 作为对照：非 0 说明外设寄存器访问本身是通的。*/
#ifndef JIG_HOST_TEST
    {
        uint32_t u1cr = (uint32_t)USART1->CR1;

        out[24] = (uint8_t)(enr >> 24);
        out[25] = (uint8_t)(enr >> 16);
        out[26] = (uint8_t)(enr >> 8);
        out[27] = (uint8_t)enr;
        out[28] = (uint8_t)(u1cr >> 8);
        out[29] = (uint8_t)u1cr;
        out[30] = (uint8_t)(rb_spi1 >> 8);      /* 写 0x0346 后 SPI1 读回值 */
        out[31] = (uint8_t)rb_spi1;
    }
#else
    out[24] = 0u; out[25] = 0u; out[26] = 0u; out[27] = 0u;
    out[28] = 0u; out[29] = 0u;
    out[30] = 0u; out[31] = 0u;
#endif

    /* ── 6) "APB1 时钟全开"后的 SPI2 结果 ── */
    out[32] = wr3;                              /* 全开时钟后 SPI2 写回读 */
    out[33] = (uint8_t)(rb3 >> 8);
    out[34] = (uint8_t)rb3;
    out[35] = (uint8_t)(apb1_all >> 24);        /* 写 0xFFFFFFFF 后 APB1ENR 的回读值 */
    out[36] = (uint8_t)(apb1_all >> 16);        /* 哪位读回 0 = 这颗片子没实现该外设 */
    out[37] = (uint8_t)(apb1_all >> 8);
    out[38] = (uint8_t)apb1_all;
    out[39] = (uint8_t)err3;
    out[40] = (uint8_t)(id3 >> 16);
    out[41] = (uint8_t)(id3 >> 8);
    out[42] = (uint8_t)id3;
    out[43] = 0u; out[44] = 0u; out[45] = 0u; out[46] = 0u; out[47] = 0u;

    /* ── 7) 多寄存器验证结果（[48..55]） ── */
    out[48] = (uint8_t)(sr2 >> 8);              /* 写了 SPE=1 之后的 SR：期望 00 02 */
    out[49] = (uint8_t)sr2;                     /*   bit1 = TXE 必须为 1 */
    out[50] = wr_cr2;                           /* CR2 写回读 (写 0x0007) */
    out[51] = (uint8_t)(rb_cr2 >> 8);
    out[52] = (uint8_t)rb_cr2;
    out[53] = wr_crcpr;                         /* CRCPR 写回读 (复位默认 0x0007) */
    out[54] = (uint8_t)(rb_crcpr >> 8);
    out[55] = (uint8_t)rb_crcpr;

    /* ── 8) 只读探针：USART3（本固件正在用的外设）──
     *   用来判断这颗 die 有没有被裁剪：低密度(C6) 与中密度(C8) 外设集合不同。 */
    out[56] = (uint8_t)(u3cr1 >> 8);            /* 期望非 0（如 0x202C） */
    out[57] = (uint8_t)u3cr1;
    out[58] = (uint8_t)(u3sr >> 8);             /* 期望 TXE/TC 置起 → 非 0 */
    out[59] = (uint8_t)u3sr;
    out[60] = (uint8_t)(u3brr >> 8);            /* 115200 的分频值 → 非 0 */
    out[61] = (uint8_t)u3brr;
    out[62] = 0u;
    out[63] = 0u;

    /* ── 5) 收尾：无条件把 CS 收回"普通推挽输出 + 高电平" ──
     *   诊断命令绝不允许把片选留在低电平（那会把总线一直占着）。
     *   GD25Q_Init() 只动 GPIO，不产生任何 SPI 时序。 */
    GD25Q_Init();

    return res;
}
