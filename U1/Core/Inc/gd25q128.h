/**
  ******************************************************************************
  * @file    gd25q128.h
  * @brief   GD25Q128 (16MB SPI NOR) 驱动 — 接在 U1 的 SPI2 上
  * @note    接线: PB12=NSS(软件CS) PB13=SCK PB14=MISO PB15=MOSI
  ******************************************************************************
  */
#ifndef __GD25Q128_H
#define __GD25Q128_H

#include <stdint.h>

void    GD25Q_Init(void);                                   /* 配置 CS 引脚并复位芯片 */
uint8_t GD25Q_Test(void);                                   /* 自测: 1=通过 (制造商标识+容量匹配) */
uint32_t GD25Q_ReadID(void);                                /* 24bit JEDEC ID, GD25Q128 = 0xC84018;
                                                               总线不通时返回 0 */
/**
  * @brief  SPI 传输失败累计次数（只增不清零）
  * @note   调用方在操作前后各取一次：
  *           `e0 = GD25Q_ErrCount(); ...; if (GD25Q_ErrCount() != e0) → 按 ERR_FLASH 处理`
  *         这是"SPI Flash 不在线"的**快速**判据（单个字节 5ms 就放弃），
  *         不再依赖"读回 0xFF 被当成空槽"那种误导性现象。
  */
uint32_t GD25Q_ErrCount(void);
void    GD25Q_Unprotect(void);                              /* 清 BP/TB/SEC/CMP 块保护 */
uint8_t GD25Q_EraseSector(uint32_t sector_addr);            /* 4KB 扇区擦除, 0=失败 */
/**
  * @brief  等芯片内部写/擦完成（读 SR1 的 WIP 位直到清零）
  * @retval 1 = 完成；0 = 超时(3s) 或 **SPI 总线传输失败**（后者立即返回, 不空转）
  * @note   总线不通时立刻返回 0 —— 这是"死总线不拖死主循环"的关键（见 .c 顶部）
  */
uint8_t GD25Q_WaitForWriteEnd(void);
uint8_t GD25Q_WritePage(const uint8_t *buf, uint32_t addr, uint16_t len); /* 跨页安全, 0=失败 */
/**
  * @brief  读一段数据
  * @retval 1 = 整段读成功；0 = 期间有 SPI 传输失败（内容不可信）
  */
uint8_t GD25Q_ReadBuffer(uint8_t *buf, uint32_t addr, uint16_t len);
uint8_t GD25Q_ReadSR1(void);

/**
  * @brief  连通性诊断：把 MCU 眼里的现场一次性报出来（排"读不到 Flash"用）
  * @param  out 输出缓冲，**必须 64 字节**
  * @retval 结果码（同时写在 out[0]）
  *           0 = 正常（JEDEC ID 合法）
  *           1 = 总线超时（SPI 传输不完成 → 外设/引脚/芯片供电问题）
  *           2 = 总线通但 ID 不符（例如 0xFFFFFF = 没有芯片应答）
  * @note   布局（大端）：
  *   [0] 结果码  [1] 首次探测的 SPI 错误增量  [2..5] JEDEC ID
  *   [6] CS 引脚回读电平(0/1)  [7] hspi2.State
  *   [8..11] SPI2->CR1   [12..15] SPI2->SR（bit7 BSY / bit5 MODF / bit1 TXE）
  *   [16..19] PB12..PB15 的 4bit 配置（0x3=GP推挽 0xB=AF推挽 0x4=浮空输入）
  *   **[20] 写回读结论**：1 = 往 SPI2 的 CR1 写 0x0346 能原样读回 → 该寄存器块**是活的**
  *   [21..22] 写 0x0346 后从 SPI2 读回的 CR1（低 16 位）
  *   **[23] 对照组：同样的写回读测试在 SPI1 上的结果（0/1）**
  *          SPI1 在任何 F103 上都存在，是"测法是否成立"的基准：
  *          SPI1=1 而 SPI2=0 → 测法没问题，问题在 SPI2 本体（型号/批次缺这个外设）
  *          两个都=0 → 是测法/时钟假设的问题
  *   [24..27] RCC->APB1ENR（**bit14 = SPI2EN**：为 1 说明时钟已请求）
  *   [28..29] USART1->CR1（对照：非 0 = 外设寄存器访问本身正常）
  *   [30..31] 写 0x0346 后从 **SPI1** 读回的 CR1（低 16 位）
  *   **[32] 把 APB1ENR 全置 1（开放所有 APB1 外设时钟）之后，SPI2 的写回读结果**
  *   [33..34] 该状态下 SPI2 读回的 CR1
  *   [35..38] 写 0xFFFFFFFF 后 APB1ENR 的**回读值**（读回 0 的位 = 这颗片子没实现的外设）
  *   [39] 该状态下读 ID 的错误增量
  *   [40..42] 该状态下读到的 JEDEC ID 低 3 字节
  *   [43..47] 预留 0
  *   **[48..49] 写完 CR1（SPE=1）之后读回的 SPI2->SR**：活着的 SPI 此刻
  *             **bit1(TXE) 必须=1**（期望 `00 02`）——这是不依赖任何写操作的独立判据
  *   [50] CR2 写回读（写 `0x0007`）  [51..52] 读回值
  *   [53] CRCPR 写回读（复位默认 `0x0007`）  [54..55] 读回值
  *   **[56..57] USART3->CR1（只读探针）**：本固件正在用 USART3（U2 链路），
  *             非 0（如 `0x202C`）= 这颗 die **有** USART3；
  *             全 0 = 连 USART3 都没有 → 真正的低密度 die（外设被裁剪）
  *   [58..59] USART3->SR   [60..61] USART3->BRR
  *   [62..63] 预留 0
  *   宿主测试（JIG_HOST_TEST）下寄存器字段填 0。
  */
uint8_t GD25Q_Diag(uint8_t *out);

#endif /* __GD25Q128_H */
