/*
 * FOC 治具 U1 —— SWD 烧录引擎：引脚与时序配置
 *
 * 本文件是 SWD 部分**唯一**需要按硬件改动的文件。
 *
 * ⚠⚠ 引脚是**占位值**，等真实板子做好后由项目负责人给出实际引脚再填。
 *    当前占位依据（现有原理图上"已经引出来"的脚里挑的，见 docs/09）：
 *      U1 空闲且已引出的脚只有 PB10/PB11（USART3 → 2 脚排针）
 *      和 PA9/PA10（USART1 → 4 脚排针），PA13/PA14 是 U1 自己的 SWD，不能占用。
 *    选 PB10+PB11 的理由：同属 GPIOB，方向切换与位操作都在 CRH，代码最省事，
 *    而且这两脚原本就是准备接被烧板的。
 *
 * 本设计**只用两根线**：SWCLK + SWDIO（+ 共地）。
 *   - 不做 connect-under-reset（没有 NRST）：
 *     只要被烧板里没把 PA13/PA14 当普通 IO 用、也没进 STOP/Standby，
 *     直接连接就能通；连不上时固件明确报错而不是死等。
 *   - 烧完靠 AIRCR 的 SYSRESETREQ 复位运行，同样不需要 NRST。
 */
#ifndef SWD_CONFIG_H
#define SWD_CONFIG_H

#include <stdint.h>

#if defined(SWD_SIM_BUILD)
#include "sim_hal.h"
#else
#include "main.h"
#endif

/* ------------------------------------------------------------------ */
/* 1. 引脚（占位，待真实硬件确认）                                     */
/* ------------------------------------------------------------------ */
/* 2026-xx 调整：原来占 PB10/PB11，但 PB10/PB11 是 U1 的 USART3（要留给
 * "U1<->U2 链路"），所以 SWD 让位到两片都用不到的空闲脚：
 *   SWCLK = PB1、SWDIO = PB3（都在 GPIOB 的 CRL 区，方向切换只写 CRL）
 * PB3 是 JTDO，需要 SWJ_NOJTAG —— 本工程 stm32f1xx_hal_msp.c 已经是 NOJTAG。
 * 真实板子排好脚之后只需改下面这几行 + 把 SWD_PINS_CONFIRMED 置 1。 */
#define SWD_CLK_PORT        GPIOB
#define SWD_CLK_PIN         GPIO_PIN_1
#define SWD_CLK_PINNUM      1u

#define SWD_DIO_PORT        GPIOB
#define SWD_DIO_PIN         GPIO_PIN_3
#define SWD_DIO_PINNUM      3u

/* NRST：本方案不用。保留宏是为了将来若接上可以做 connect-under-reset。 */
#define SWD_NRST_PORT       GPIOC
#define SWD_NRST_PIN        GPIO_PIN_13
#define SWD_NRST_PINNUM     13u

/* 1 = 引脚已按真实硬件填好并接线完成；0 = 占位值（上板前必须确认） */
#ifndef SWD_PINS_CONFIRMED
#define SWD_PINS_CONFIRMED  0
#endif

/* 1 = 使用 NRST 做 connect-under-reset（本方案为 0） */
#ifndef SWD_USE_NRST
#define SWD_USE_NRST        0
#endif

/* ------------------------------------------------------------------ */
/* 2. 时钟速率                                                         */
/* ------------------------------------------------------------------ */
/* 每个半周期插入的 NOP 循环次数（0 = 全速，仅仿真用）。
 * F103 @72MHz：8 大约 1MHz SWCLK，4 大约 2MHz，0 大约 3~4MHz。
 * 上板先用 8（慢而稳），确认能连上再往下调。 */
#ifndef SWD_DELAY_LOOPS
#define SWD_DELAY_LOOPS     8u
#endif

/* ------------------------------------------------------------------ */
/* 3. 线级约定（已按 ADIv5 IHI0031H §B4.1.1 与 DAPLink swd_host.c 逐条核对） */
/* ------------------------------------------------------------------ */
/* 依据 ADIv5：主机在 SWCLK 下降沿驱动 SWDIO，目标在上升沿采样。
 *   §B4.1.1：请求头→应答之间必须有 turnaround（方向改变）；
 *            **写**：应答→WDATA 之间要 turnaround；WDATA 之后主机继续驱动，
 *                  **没有额外 turnaround**；
 *            **读**：应答→RDATA 之间**没有** turnaround（都是目标驱动）；
 *                  RDATA 之后要 turnaround，然后主机接管。
 *   §B4.1.1：数据阶段之后主机要么立刻开始下一笔，要么用空闲周期继续驱动；
 *            如果要把时钟停掉，必须先打满 **≥8 个空闲周期**（我们从不停时钟，只补 1 拍）。
 *   空闲周期里 SWDIO 由主机驱动：DAPLink 在空闲周期里驱动低电平，之后拉高。 */
#define SWD_TA_AFTER_HEADER     1u   /* 请求头 → ACK：主机→目标，必须 1 */
#define SWD_TA_AFTER_ACK_WRITE  1u   /* ACK → 写数据：目标→主机，必须 1 */
#define SWD_TA_BEFORE_READ_DATA 0u   /* ACK → 读数据：同为目标驱动，必须 0 */
#define SWD_TA_TAIL             1u   /* 事务末 turnaround（读必须；写只在非 OK 时补） */
#define SWD_TAIL_IDLE_CYCLES    1u   /* 末 turnaround 之后的空闲周期数 */
#define SWD_IDLE_LEVEL          0u   /* 空闲周期 SWDIO 电平（DAPLink 即如此） */

/* ------------------------------------------------------------------ */
/* 4. 超时与重试                                                       */
/* ------------------------------------------------------------------ */
#define SWD_RETRY_WAIT          100u   /* WAIT 应答最大重试次数（对齐 DAPLink 的 MAX_SWD_RETRY） */
#define SWD_RETRY_FAULT         2u     /* FAULT 应答最大重试次数 */
#define SWD_CONNECT_RETRY       3u     /* 连接（读 IDCODE）重试次数 */
#define SWD_POLL_MAX            500000u/* 轮询上限，防死循环 */
#define F4_FLASH_TIMEOUT_MS     5000u  /* 擦/写等待 BSY 清零的超时 */
#define F4_BUSY_POLLS           1u     /* 仿真里 BSY 需要几次轮询才落（仅仿真用） */

/* ------------------------------------------------------------------ */
/* 5. STM32F405RGT6 目标常量                                           */
/* ------------------------------------------------------------------ */
#define F4_FLASH_BASE           0x08000000u
#define F4_FLASH_SIZE           (1024u * 1024u)
#define F4_SECTOR_COUNT         8u     /* 16,16,16,16,64,128,128,128 KB */

#endif /* SWD_CONFIG_H */
