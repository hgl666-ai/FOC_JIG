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

static uint8_t SPI_Swap(uint8_t tx)
{
    uint8_t rx = 0;
    (void)HAL_SPI_TransmitReceive(&hspi2, &tx, &rx, 1, 1000);
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

    CS_LOW();
    SPI_Swap(CMD_READ_STATUS1);
    do {
        status = SPI_Swap(0xFF);
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
    CS_LOW();
    SPI_Swap(CMD_JEDEC_ID);
    id  = (uint32_t)SPI_Swap(0xFF) << 16;   /* 制造商 ID   */
    id |= (uint32_t)SPI_Swap(0xFF) << 8;    /* 存储类型    */
    id |= (uint32_t)SPI_Swap(0xFF);         /* 容量        */
    CS_HIGH();
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

void GD25Q_ReadBuffer(uint8_t *buf, uint32_t addr, uint16_t len)
{
    CS_LOW();
    SPI_Swap(CMD_READ_DATA);
    SPI_Swap((uint8_t)(addr >> 16));
    SPI_Swap((uint8_t)(addr >> 8));
    SPI_Swap((uint8_t)(addr));
    for (uint16_t i = 0; i < len; i++) buf[i] = SPI_Swap(0xFF);
    CS_HIGH();
}
