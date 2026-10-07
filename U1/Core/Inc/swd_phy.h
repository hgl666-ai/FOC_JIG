/*
 * SWD 物理层（位与包）
 *
 * 职责：把"一个 SWD 事务"翻译成 GPIO 上的电平序列。
 *   - 只管线路时序，不管寄存器含义（那是 swd_dp.c 的事）。
 *   - 仿真构建 (SWD_SIM_BUILD) 与目标构建共用同一份调用逻辑，
 *     只有最底层的 5 个原语不同（仿真走 HAL 替身以便拦截，目标直写 BSRR/CRL）。
 *
 * 线级约定（ADIv5 IHI0031）：
 *   - 主机在 SWCLK 下降沿驱动 SWDIO，目标在上升沿采样；读方向相反。
 *   - 请求/应答/数据一律 **LSB first**。
 *   - 请求头 8 位：Start(1) APnDP RnW A2 A3 Parity Stop(0) Park(1)。
 *   - ACK 3 位：OK=0b001 WAIT=0b010 FAULT=0b100（LSB first 上线路）。
 */
#ifndef SWD_PHY_H
#define SWD_PHY_H

#include <stdint.h>

/* 统一返回码：0 = 成功，负 = 失败 */
typedef enum {
    SWD_OK            = 0,
    SWD_ERR_NO_ACK    = -1,  /* ACK 无效应答：目标未上电/线断/未接 */
    SWD_ERR_WAIT      = -2,  /* 目标回 WAIT（重试用尽） */
    SWD_ERR_FAULT     = -3,  /* 目标回 FAULT（重试用尽） */
    SWD_ERR_PARITY    = -4,  /* 读回数据的奇偶校验错 */
    SWD_ERR_PARAM     = -5,  /* 参数非法 */
    SWD_ERR_TIMEOUT   = -6,  /* 轮询超时 */
    SWD_ERR_FLASH     = -7,  /* 目标 Flash 控制器报错 */
    SWD_ERR_NOT_HALTED= -8,  /* 内核未能停止 */
    SWD_ERR_VERIFY    = -9   /* 回读校验不一致 */
} swd_status_t;

/* ACK 编码（LSB first 解出的值） */
#define SWD_ACK_OK      0x01u
#define SWD_ACK_WAIT    0x02u
#define SWD_ACK_FAULT   0x04u

/* ---- 初始化 ---- */
void SWD_PhyInit(void);
void SWD_PhyNrst(uint8_t assert_low);   /* 1 = 拉低复位（assert） */

/* ---- 位级 ---- */
void    SWD_WriteBit(uint8_t b);
uint8_t SWD_ReadBit(void);
void    SWD_WriteBits(uint32_t v, uint8_t n);   /* LSB first */
uint32_t SWD_ReadBits(uint8_t n);               /* LSB first */
void    SWD_Turnaround(void);                   /* 释放总线一个周期 */
void    SWD_IdleClock(void);                    /* 主机驱动空闲电平一个周期 */
void    SWD_LineReset(void);                    /* ≥50 周期 SWDIO 高 */

/* ---- 包级 ---- */
uint8_t SWD_ParityOf(uint32_t v);               /* 偶校验 */

/**
 * 执行一次 SWD 事务。
 * @param apndp 0=DP 1=AP
 * @param rnw   0=写 1=读
 * @param addr  4 位地址的第 [3:2] 位（即 0x00/0x04/0x08/0x0C）
 * @param data  写入值 / 读出值
 * @return swd_status_t
 */
int SWD_Transfer(uint8_t apndp, uint8_t rnw, uint8_t addr, uint32_t *data);

/* 已产生的 SWCLK 周期数（上板可用来估算烧录时间） */
uint32_t SWD_PhyClockCount(void);

/* 最近一次读到的 3 位 ACK 原始值（诊断"没应答"到底是线断了还是被拉死） */
uint8_t SWD_PhyLastAck(void);

#endif /* SWD_PHY_H */
