/*
 * STM32F405RGT6 内部 Flash 编程实现
 */
#include "swd_config.h"
#include "swd_phy.h"
#include "swd_memap.h"
#include "stm32f4_flash.h"

/* 扇区表：F405RG = 1MB = 16K x4 + 64K x1 + 128K x3 */
static const uint32_t k_sec_kb[F4_SECTOR_COUNT] = {
    16u, 16u, 16u, 16u, 64u, 128u, 128u, 128u
};
static const uint32_t k_sec_off[F4_SECTOR_COUNT] = {
    0x00000u, 0x04000u, 0x08000u, 0x0C000u,
    0x10000u, 0x20000u, 0x40000u, 0x60000u
};

uint32_t F4_SectorBase(uint32_t sector)
{
    if (sector >= F4_SECTOR_COUNT) {
        return 0xFFFFFFFFu;
    }
    return F4_FLASH_BASE + k_sec_off[sector];
}

uint32_t F4_SectorSize(uint32_t sector)
{
    if (sector >= F4_SECTOR_COUNT) {
        return 0u;
    }
    return k_sec_kb[sector] * 1024u;
}

int F4_SectorOf(uint32_t addr, uint32_t *sector)
{
    uint32_t off;
    uint32_t i;

    if (addr < F4_FLASH_BASE || addr >= (F4_FLASH_BASE + F4_FLASH_SIZE)) {
        return SWD_ERR_PARAM;
    }
    off = addr - F4_FLASH_BASE;
    for (i = 0u; i < F4_SECTOR_COUNT; i++) {
        if (off < (k_sec_off[i] + k_sec_kb[i] * 1024u)) {
            if (sector != 0) {
                *sector = i;
            }
            return SWD_OK;
        }
    }
    return SWD_ERR_PARAM;
}

int F4_ReadStatus(uint32_t *sr)
{
    return SWD_MemRead32(F4_FLASH_SR, sr);
}

int F4_ClearFlags(void)
{
    /* SR 的 EOP / 各错误位都是写 1 清零 */
    return SWD_MemWrite32(F4_FLASH_SR, (uint32_t)(F4_SR_EOP | F4_SR_ERRMASK));
}

int F4_IsLocked(uint8_t *locked)
{
    uint32_t cr = 0u;
    int      r  = SWD_MemRead32(F4_FLASH_CR, &cr);

    if (r != SWD_OK) {
        return r;
    }
    if (locked != 0) {
        *locked = ((cr & F4_CR_LOCK) != 0u) ? 1u : 0u;
    }
    return SWD_OK;
}

int F4_Unlock(void)
{
    uint8_t locked = 0u;
    int     r;

    r = F4_IsLocked(&locked);
    if (r != SWD_OK) {
        return r;
    }
    if (locked == 0u) {
        return SWD_OK;
    }
    r = SWD_MemWrite32(F4_FLASH_KEYR, F4_FLASH_KEY1);
    if (r != SWD_OK) {
        return r;
    }
    r = SWD_MemWrite32(F4_FLASH_KEYR, F4_FLASH_KEY2);
    if (r != SWD_OK) {
        return r;
    }
    r = F4_IsLocked(&locked);
    if (r != SWD_OK) {
        return r;
    }
    return (locked != 0u) ? SWD_ERR_FLASH : SWD_OK;
}

int F4_Lock(void)
{
    uint32_t cr = 0u;
    uint32_t i;
    int      r;

    r = SWD_MemRead32(F4_FLASH_CR, &cr);
    if (r != SWD_OK) {
        return r;
    }
    for (i = 0u; i < SWD_POLL_MAX; i++) {
        uint32_t sr = 0u;
        r = F4_ReadStatus(&sr);
        if (r != SWD_OK) {
            return r;
        }
        if ((sr & F4_SR_BSY) == 0u) {
            break;
        }
    }
    return SWD_MemWrite32(F4_FLASH_CR, cr | F4_CR_LOCK);
}

/* 等 BSY 清零，并检查错误位。返回 SWD_OK / SWD_ERR_FLASH / SWD_ERR_TIMEOUT */
static int flash_wait(uint32_t *sr_out)
{
    uint32_t start = HAL_GetTick();
    uint32_t sr    = 0u;
    uint32_t i;
    int      r;

    for (i = 0u; i < SWD_POLL_MAX; i++) {
        r = F4_ReadStatus(&sr);
        if (r != SWD_OK) {
            return r;
        }
        if ((sr & F4_SR_BSY) == 0u) {
            break;
        }
        if ((HAL_GetTick() - start) > F4_FLASH_TIMEOUT_MS) {
            return SWD_ERR_TIMEOUT;
        }
    }
    if ((sr & F4_SR_BSY) != 0u) {
        return SWD_ERR_TIMEOUT;
    }
    if (sr_out != 0) {
        *sr_out = sr;
    }
    return ((sr & F4_SR_ERRMASK) != 0u) ? SWD_ERR_FLASH : SWD_OK;
}

/* 擦除单个扇区（含解锁检查）。烧录状态机一次只调一次，保持主循环响应性。 */
int F4_EraseSector(uint32_t sector)
{
    uint8_t  locked = 0u;
    uint32_t cr;
    int      r;

    if (sector >= F4_SECTOR_COUNT) {
        return SWD_ERR_PARAM;
    }
    r = F4_Unlock();
    if (r != SWD_OK) {
        return r;
    }
    r = F4_IsLocked(&locked);
    if (r != SWD_OK) {
        return r;
    }
    if (locked != 0u) {
        return SWD_ERR_FLASH;
    }
    r = F4_ClearFlags();
    if (r != SWD_OK) {
        return r;
    }
    cr = F4_CR_PSIZE_X32 | F4_CR_SER | ((sector & 0x0Fu) << F4_CR_SNB_POS);
    r = SWD_MemWrite32(F4_FLASH_CR, cr);
    if (r != SWD_OK) {
        return r;
    }
    r = SWD_MemWrite32(F4_FLASH_CR, cr | F4_CR_STRT);
    if (r != SWD_OK) {
        return r;
    }
    r = flash_wait(0);
    if (r != SWD_OK) {
        (void)SWD_MemWrite32(F4_FLASH_CR, 0u);
        return r;
    }
    r = SWD_MemWrite32(F4_FLASH_CR, 0u);   /* 清 SER/SNB */
    if (r != SWD_OK) {
        return r;
    }
    return F4_ClearFlags();
}

int F4_EraseRange(uint32_t addr, uint32_t length)
{
    uint32_t first;
    uint32_t last;
    uint32_t s;
    uint32_t end;
    int      r;

    if (length == 0u) {
        return SWD_OK;
    }
    end = addr + length - 1u;
    if (addr < F4_FLASH_BASE || end >= (F4_FLASH_BASE + F4_FLASH_SIZE)) {
        return SWD_ERR_PARAM;                  /* 越界：绝不擦到别的分区 */
    }
    r = F4_SectorOf(addr, &first);
    if (r != SWD_OK) {
        return r;
    }
    r = F4_SectorOf(end, &last);
    if (r != SWD_OK) {
        return r;
    }
    for (s = first; s <= last; s++) {
        r = F4_EraseSector(s);
        if (r != SWD_OK) {
            return r;
        }
    }
    return SWD_OK;
}

/* 编程会话：解锁 + 清标志 + 设好 PSIZE(x32) 与 PG，之后每块只需"写地址 + 等 BSY" */
int F4_ProgramBegin(void)
{
    int r;

    r = F4_Unlock();
    if (r != SWD_OK) {
        return r;
    }
    r = F4_ClearFlags();
    if (r != SWD_OK) {
        return r;
    }
    return SWD_MemWrite32(F4_FLASH_CR, F4_CR_PSIZE_X32 | F4_CR_PG);
}

int F4_ProgramEnd(void)
{
    return SWD_MemWrite32(F4_FLASH_CR, 0u);    /* 清 PG */
}

/* 在会话内编程一段（尾部不足 4 字节按 0xFF 补齐）。
 * 注意：错误标志只在出错时清——每字都清一次会白白多花 2 个 SWD 事务/字。 */
int F4_ProgramWords(uint32_t addr, const uint8_t *data, uint32_t length)
{
    uint32_t words;
    uint32_t i;
    uint32_t end;
    int      r = SWD_OK;

    if (length == 0u) {
        return SWD_OK;
    }
    if (data == 0 || (addr & 3u) != 0u) {
        return SWD_ERR_PARAM;
    }
    end = addr + length - 1u;
    if (addr < F4_FLASH_BASE || end >= (F4_FLASH_BASE + F4_FLASH_SIZE)) {
        return SWD_ERR_PARAM;
    }

    words = (length + 3u) / 4u;
    for (i = 0u; i < words; i++) {
        uint32_t off = i * 4u;
        uint32_t w   = 0xFFFFFFFFu;            /* 尾部按 0xFF 补齐 */
        uint8_t  b;

        for (b = 0u; b < 4u; b++) {
            if ((off + b) < length) {
                w &= ~(0xFFu << (b * 8u));
                w |= (uint32_t)data[off + b] << (b * 8u);
            }
        }
        r = SWD_MemWrite32(addr + off, w);
        if (r != SWD_OK) {
            break;
        }
        r = flash_wait(0);
        if (r != SWD_OK) {
            (void)F4_ClearFlags();             /* 出错时清一次，便于下次重试 */
            break;
        }
    }
    return r;
}

int F4_ProgramImage(uint32_t addr, const uint8_t *data, uint32_t length)
{
    int r;

    if (length == 0u) {
        return SWD_OK;
    }
    r = F4_ProgramBegin();
    if (r != SWD_OK) {
        return r;
    }
    r = F4_ProgramWords(addr, data, length);
    (void)F4_ProgramEnd();
    return r;
}

int F4_ReadImage(uint32_t addr, uint8_t *buf, uint32_t length)
{
    uint32_t words = (length + 3u) / 4u;
    uint32_t i;
    int      r;

    for (i = 0u; i < words; i++) {
        uint32_t w = 0u;
        uint32_t off = i * 4u;
        uint8_t  b;

        r = SWD_MemRead32(addr + off, &w);
        if (r != SWD_OK) {
            return r;
        }
        for (b = 0u; b < 4u; b++) {
            if ((off + b) < length) {
                buf[off + b] = (uint8_t)((w >> (b * 8u)) & 0xFFu);
            }
        }
    }
    return SWD_OK;
}

int F4_VerifyImage(uint32_t addr, const uint8_t *data, uint32_t length)
{
    uint32_t words = (length + 3u) / 4u;
    uint32_t done  = 0u;
    uint32_t chunk[F4_VERIFY_CHUNK_WORDS];
    int      r;

    /* 用 auto-increment 块读：每个字只花 1 个 SWD 事务，
     * 比逐字读（3 个事务）快 3 倍。 */
    while (done < words) {
        uint32_t n = words - done;
        uint32_t i;
        if (n > (uint32_t)F4_VERIFY_CHUNK_WORDS) {
            n = (uint32_t)F4_VERIFY_CHUNK_WORDS;
        }
        r = SWD_MemReadBlock(addr + done * 4u, chunk, n);
        if (r != SWD_OK) {
            return r;
        }
        for (i = 0u; i < n; i++) {
            uint32_t off = (done + i) * 4u;
            uint32_t w   = chunk[i];
            uint8_t  b;
            for (b = 0u; b < 4u; b++) {
                if ((off + b) < length) {
                    uint8_t rd = (uint8_t)((w >> (b * 8u)) & 0xFFu);
                    if (rd != data[off + b]) {
                        return SWD_ERR_VERIFY;
                    }
                }
            }
        }
        done += n;
    }
    return SWD_OK;
}
