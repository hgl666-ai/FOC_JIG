/*
 * STM32F405RGT6 内部 Flash 编程（F4 的 FLASH 控制器）
 *
 * 与 F1 的关键差别（这是"换目标芯片"最容易踩的地方）：
 *   - 扇区不是等大的页：16/16/16/16/64/128/128/128 KB，
 *     擦除必须整扇区，分区地址必须落在扇区边界上；
 *   - FLASH_CR.LOCK 在 bit31（F1 在 bit7）；
 *   - 并行度 PSIZE 必须与被写数据宽度匹配，3.3V 下用 x32（写 32 位字）；
 *   - 编程是"写地址即触发"，且同一个字在两次擦除之间只能写一次。
 */
#ifndef STM32F4_FLASH_H
#define STM32F4_FLASH_H

#include <stdint.h>

/* 寄存器 */
#define F4_FLASH_ACR      0x40023C00u
#define F4_FLASH_KEYR     0x40023C04u
#define F4_FLASH_OPTKEYR  0x40023C08u
#define F4_FLASH_SR       0x40023C0Cu
#define F4_FLASH_CR       0x40023C10u
#define F4_FLASH_OPTCR    0x40023C14u

#define F4_FLASH_KEY1     0x45670123u
#define F4_FLASH_KEY2     0xCDEF89ABu

/* SR 位（RM0090 Rev22 §3.9.6，F40x/F41x）
 *   bit0 EOP / bit1 OPERR / bit4 WRPERR / bit5 PGAERR / bit6 PGPERR
 *   bit7 PGSERR / bit8 RDERR(仅 F42x/43x，F405 上是保留位) / bit16 BSY
 * 全部是 rc_w1（写 1 清零）。 */
#define F4_SR_EOP      (1u << 0)
#define F4_SR_OPERR    (1u << 1)
#define F4_SR_WRPERR   (1u << 4)
#define F4_SR_PGAERR   (1u << 5)
#define F4_SR_PGPERR   (1u << 6)
#define F4_SR_PGSERR   (1u << 7)
#define F4_SR_RDERR    (1u << 8)
#define F4_SR_BSY      (1u << 16)
#define F4_SR_ERRMASK  (F4_SR_OPERR | F4_SR_WRPERR | F4_SR_PGAERR | \
                        F4_SR_PGPERR | F4_SR_PGSERR | F4_SR_RDERR)

/* CR 位（RM0090 Rev22 §3.9.7 "Flash control register (FLASH_CR)"，F40x/F41x 单 bank）
 *   bit0 PG / bit1 SER / bit2 MER / bits6:3 SNB[3:0] / bits9:8 PSIZE
 *   bit16 STRT / bit24 EOPIE / bit25 ERRIE / bit31 LOCK
 * ⚠ STRT 是 bit16（不是 bit15）——写错的话擦除根本不会启动，
 *   而 BSY 一直是 0，固件会"以为擦好了"，直到编程时撞上 PGAERR 才报错。 */
#define F4_CR_PG       (1u << 0)
#define F4_CR_SER      (1u << 1)
#define F4_CR_MER      (1u << 2)
#define F4_CR_SNB_POS  3u
#define F4_CR_SNB_MASK (0xFu << F4_CR_SNB_POS)
#define F4_CR_PSIZE_POS 8u
#define F4_CR_PSIZE_X32 (2u << F4_CR_PSIZE_POS)
#define F4_CR_STRT     (1u << 16)
#define F4_CR_LOCK     (1u << 31)

/* 回读校验的块读粒度（字）。16 字 = 64 字节栈上缓冲。 */
#define F4_VERIFY_CHUNK_WORDS 16u

int F4_Unlock(void);
int F4_Lock(void);
int F4_ReadStatus(uint32_t *sr);
int F4_ClearFlags(void);
int F4_IsLocked(uint8_t *locked);

uint32_t F4_SectorBase(uint32_t sector);
uint32_t F4_SectorSize(uint32_t sector);
int      F4_SectorOf(uint32_t addr, uint32_t *sector);

/** 擦除单个扇区（烧录状态机一次只擦一个，保持主循环响应性）。 */
int F4_EraseSector(uint32_t sector);

/** 擦除 [addr, addr+len) 覆盖到的所有扇区（整扇区擦除）。 */
int F4_EraseRange(uint32_t addr, uint32_t length);

/* ---- 编程会话：把 PSIZE/PG 只设一次，之后按块写 ----
 * 状态机分块烧写时用它：F4_ProgramBegin() → 多次 F4_ProgramWords() → F4_ProgramEnd()。
 * 每写一个字都要等 BSY 清零（F4 要求），但错误标志只在出错时清，
 * 这比"每字清一次标志"省 2 个 SWD 事务/字。 */
int F4_ProgramBegin(void);
int F4_ProgramWords(uint32_t addr, const uint8_t *data, uint32_t length);
int F4_ProgramEnd(void);

/** 编程一段镜像（内部自动处理 PSIZE/PG，尾部不足 4 字节按 0xFF 补齐）。 */
int F4_ProgramImage(uint32_t addr, const uint8_t *data, uint32_t length);

/** 回读一段并逐字节比对。 */
int F4_VerifyImage(uint32_t addr, const uint8_t *data, uint32_t length);

/** 读一段到缓冲区（供查版本号/UID 之类使用）。 */
int F4_ReadImage(uint32_t addr, uint8_t *buf, uint32_t length);

#endif /* STM32F4_FLASH_H */
