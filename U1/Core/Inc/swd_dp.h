/*
 * SW-DP 层 —— 调试端口（ADIv5 DP）
 *
 * 职责：
 *   - 连接：线复位 → JTAG→SWD 切换序列(0xE79E) → 线复位 → 读 IDCODE
 *   - 上电：CTRL/STAT 的 CDBGPWRUPREQ / CSYSPWRUPREQ 并确认 ACK
 *   - 选择 AP 与 bank（SELECT）
 *   - 带重试的 DP / AP 寄存器读写（WAIT 重试、FAULT 清 sticky 后重试）
 */
#ifndef SWD_DP_H
#define SWD_DP_H

#include <stdint.h>

/* DP 寄存器地址（只需 A[3:2]，故取值 0x00/0x04/0x08/0x0C） */
#define SWD_DP_ABORT     0x00u   /* 写：清 sticky（ADIv5 §B4.2.1） */
#define SWD_DP_IDCODE    0x00u   /* 读：IDCODE */
#define SWD_DP_CTRLSTAT  0x04u
#define SWD_DP_SELECT    0x08u
#define SWD_DP_RDBUFF    0x0Cu

/* AP 寄存器地址 */
#define SWD_AP_CSW       0x00u
#define SWD_AP_TAR       0x04u
#define SWD_AP_DRW       0x0Cu

/* ABORT 寄存器位（ADIv5 §B4.2.1）
 *   bit1 STKCMPCLR / bit2 STKERRCLR / bit3 WDERRCLR / bit4 ORUNERRCLR
 * SW-DP 上清 sticky **必须走 ABORT**：CTRL/STAT 的 sticky 位在 SW-DP 上是 RO/WI，
 * 而且写 CTRL/STAT 会顺带把 CDBGPWRUPREQ 清零（第一版这么做是错的，见 docs/02 §7.19）。 */
#define SWD_ABORT_STKCMPCLR  (1u << 1)
#define SWD_ABORT_STKERRCLR  (1u << 2)
#define SWD_ABORT_WDERRCLR   (1u << 3)
#define SWD_ABORT_ORUNERRCLR (1u << 4)
#define SWD_ABORT_CLEAR_ALL  (SWD_ABORT_STKCMPCLR | SWD_ABORT_STKERRCLR | \
                              SWD_ABORT_WDERRCLR  | SWD_ABORT_ORUNERRCLR)

/* CTRL/STAT 位（ADIv5 §B4.2.2：bit0 ORUNDETECT / bit1 STICKYORUN /
 * bit28 CDBGPWRUPREQ / bit29 CDBGPWRUPACK / bit30 CSYSPWRUPREQ / bit31 CSYSPWRUPACK） */
#define SWD_CTRLSTAT_STICKYORUN   (1u << 1)
#define SWD_CTRLSTAT_CDBGPWRUPREQ (1u << 28)
#define SWD_CTRLSTAT_CDBGPWRUPACK (1u << 29)
#define SWD_CTRLSTAT_CSYSPWRUPREQ (1u << 30)
#define SWD_CTRLSTAT_CSYSPWRUPACK (1u << 31)

/* STM32F4 (Cortex-M4) 的 SW-DP IDCODE */
#define SWD_IDCODE_STM32F4  0x2BA01477u

/** 连接目标并读回 IDCODE。失败返回负错误码。 */
int SWD_Connect(uint32_t *idcode);

/** 只做一次"线路是否活着"的判断（不发复位时序）。 */
int SWD_IsAlive(void);

/** 请求调试/系统电源上电并等待 ACK。 */
int SWD_PowerUp(void);

/** 清 sticky 错误标志。 */
int SWD_ClearSticky(void);

/** 选择 AP 与 bank。 */
int SWD_SelectAP(uint8_t apsel, uint8_t bank);

int SWD_DPRead(uint8_t addr, uint32_t *value);
int SWD_DPWrite(uint8_t addr, uint32_t value);
int SWD_APRead(uint8_t addr, uint32_t *value);
int SWD_APWrite(uint8_t addr, uint32_t value);

/** 最近一次错误的裸 ACK（用于诊断 0x0112 的失败原因）。 */
uint8_t SWD_LastAck(void);

#endif /* SWD_DP_H */
