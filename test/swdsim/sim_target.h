/*
 * 仿真目标模型（被烧板 STM32F405RGT6）
 *
 * 工作方式：**线级仿真**。
 *   swd_phy.c 通过 HAL 替身写 GPIO，sim_hal.c 把这些写操作变成 SWCLK 边沿 /
 *   SWDIO 事件送进来；本文件在 SWCLK 上升沿采样 SWDIO、在下降沿驱动 SWDIO，
 *   并按 ADIv5 约定解码请求头、ACK、数据。
 *   因此被测的是"固件真正打出来的波形"，而不是把协议 API 替换成桩函数。
 *
 * 模型包含：
 *   - SW-DP：IDCODE / CTRL-STAT（含 sticky、上电 ACK）/ SELECT / RDBUFF
 *   - AHB-AP：CSW / TAR / DRW，含 **posted read（延迟读）** 与 auto-increment
 *   - STM32F405 存储与调试寄存器：
 *       Flash 1MB（16/16/16/16/64/128/128/128 KB 扇区，擦/写/错误位语义）
 *       FLASH_KEYR/SR/CR/OPTCR
 *       DHCSR/DCRSR/DCRDR/DEMCR/AIRCR
 *       DBGMCU_IDCODE(0x413)/CR/APB1_FZ
 *
 * 明确不建模（留给上板）：真实时钟速率、电气特性、目标上电时序、
 * RDP 读保护、SWCLK 抖动。见 README。
 */
#ifndef SIM_TARGET_H
#define SIM_TARGET_H

#include <stdint.h>
#include "sim_hal.h"

/* ---- 控制 ---- */
void sim_target_reset(void);
void sim_target_attach(uint8_t on);        /* 0 = 目标不在（线悬空，用于测连接失败） */
void sim_target_set_wait(uint32_t n);      /* 接下来 n 次 AP/DP 事务回 WAIT */
void sim_target_set_fault(uint32_t n);     /* 接下来 n 次 AP/DP 事务回 FAULT */
void sim_target_set_read_ta(uint8_t n);    /* 读数据前的 turnaround 周期数，需与主机一致 */
void sim_target_set_stuck_busy(uint8_t on);/* BSY 永不落，用于测超时 */
void sim_target_set_idcode(uint32_t id);

/* ---- 观察 ---- */
uint8_t *sim_target_flash(void);
uint32_t sim_target_flash_size(void);
void     sim_target_flash_corrupt(uint32_t addr, uint8_t value);

uint32_t sim_target_clocks(void);
uint32_t sim_target_transactions(void);
uint32_t sim_target_contention(void);      /* 主机与目标同时驱动 SWDIO 的次数（必须 0） */
uint32_t sim_target_parity_errors(void);
uint32_t sim_target_wait_sent(void);
uint32_t sim_target_fault_sent(void);
uint32_t sim_target_ta_skipped(void);      /* 数据相位里"无人驱动"的周期数 */
uint32_t sim_target_locked_reject(void);   /* 未解锁就写 Flash 的次数 */
uint32_t sim_target_bad_psize(void);
uint32_t sim_target_unmapped(void);
uint32_t sim_target_switch_seen(void);
uint32_t sim_target_line_resets(void);
uint32_t sim_target_erases(void);
uint32_t sim_target_programs(void);
uint32_t sim_target_rdbuff_reads(void);
uint32_t sim_target_flash_err(void);
uint32_t sim_target_pc(void);
uint32_t sim_target_sp(void);
uint8_t  sim_target_halted(void);
uint32_t sim_target_iwdg_frozen(void);
uint32_t sim_target_nrst_asserts(void);

/* ---- GPIO 事件入口（由 sim_hal.c 调用） ---- */
void    sim_gpio_write(GPIO_TypeDef *port, uint16_t pin, uint8_t level);
void    sim_gpio_dir(GPIO_TypeDef *port, uint16_t pin, uint8_t output);
uint8_t sim_gpio_read(GPIO_TypeDef *port, uint16_t pin);

#endif /* SIM_TARGET_H */
