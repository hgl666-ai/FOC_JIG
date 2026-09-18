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
uint32_t GD25Q_ReadID(void);                                /* 24bit JEDEC ID, GD25Q128 = 0xC84018 */
void    GD25Q_Unprotect(void);                              /* 清 BP/TB/SEC/CMP 块保护 */
uint8_t GD25Q_EraseSector(uint32_t sector_addr);            /* 4KB 扇区擦除 */
uint8_t GD25Q_WritePage(const uint8_t *buf, uint32_t addr, uint16_t len); /* 跨页安全 */
void    GD25Q_ReadBuffer(uint8_t *buf, uint32_t addr, uint16_t len);
uint8_t GD25Q_ReadSR1(void);

#endif /* __GD25Q128_H */
