/*
 * AHB-AP（MEM-AP）内存访问
 *
 * 关键点：**posted read（延迟读）**。
 *   写 TAR 只是"设定地址"，AP 读 DRW 返回的是 **上一次** 的结果，
 *   本次结果要通过再读一次 DRW 或读 DP 的 RDBUFF 才能拿到。
 *   第一次访问如果直接用 DRW 的返回值，拿到的就是脏数据——
 *   这是 SWD 主机最常见的 bug，本模块的单次读序列显式处理了它：
 *       write TAR  →  read DRW(丢弃，发起传输)  →  read RDBUFF(本次结果)
 */
#ifndef SWD_MEMAP_H
#define SWD_MEMAP_H

#include <stdint.h>

/* CSW 取值（ADIv5 IHI0031H §C2.2 表 C2-1 + §B2.3.1 字段定义）：
 *   HPROT[31:24]=0x23（data/privileged/non-cacheable/non-bufferable）
 *   MSTRDBG=bit29、RESERVED=bit24、DBGSTAT=bit6、Size[2:0]=0b010（32 位字）
 *   AddrInc[5:4] = 0b01（Increment single）
 *
 * ⚠ AddrInc 的语义最容易记反（我第一版就记反了，见 docs/02 §7.19）：
 *     0b00 = **自增关闭**：TAR 不变，连续 DRW 访问都打同一个地址
 *     0b01 = **每次 DRW 访问后按访问宽度自增**  ← 块读写要的是这个
 *     0b10 = packed
 *   所以单次访问与块访问**用同一个值** 0x23000052：
 *   单次访问每次都重写 TAR，自增与否无所谓；块访问必须自增。
 *   佐证：DAPLink 的 CSW_VALUE 就是 0x23000052（含 CSW_SADDRINC），
 *   见 INFO/SWD烧录参考的开源项目/DAPLink-main/source/daplink/interface/swd_host.c:39 */
#define AHBAP_CSW_WORD       0x23000052u

/* 只用于负向测试：把 AddrInc 改成"自增关闭"，验证模型/目标行为差异 */
#define AHBAP_CSW_WORD_NOINC 0x23000042u

/** 初始化 MEM-AP（选 AP0 + bank0，写 CSW）。 */
int SWD_MemInit(void);

/** 单次 32 位读（含 posted read 处理）。 */
int SWD_MemRead32(uint32_t addr, uint32_t *value);

/** 单次 32 位写。 */
int SWD_MemWrite32(uint32_t addr, uint32_t value);

/** 连续读 n 个字（AddrInc=increment-single，流水线 posted read）。 */
int SWD_MemReadBlock(uint32_t addr, uint32_t *buf, uint32_t n);

/** 连续写 n 个字（AddrInc=increment-single）。 */
int SWD_MemWriteBlock(uint32_t addr, const uint32_t *buf, uint32_t n);

/**
  * @brief  用指定的 CSW 做一次单字读（供宿主测试验证 CSW 字段语义）
  * @note   只给测试用；正常路径请用 SWD_MemRead32。
  */
int SWD_MemRead32WithCsw(uint32_t addr, uint32_t csw, uint32_t *value);

#endif /* SWD_MEMAP_H */
