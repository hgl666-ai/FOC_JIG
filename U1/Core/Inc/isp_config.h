/**
  ******************************************************************************
  * @file    isp_config.h
  * @brief   ISP 烧录链路配置 — 唯一需要按实际硬件修改的地方
  *
  * @warning ⚠ 以下引脚/器件参数【尚未经硬件确认】，当前为占位值。
  *          确认后只改本文件即可，其余代码无需改动。
  *          相关待确认项见 docs/00_需求与硬件分析.md §5 Q2/Q3。
  ******************************************************************************
  */
#ifndef __ISP_CONFIG_H
#define __ISP_CONFIG_H

#include "main.h"

/*==============================================================================
 * 1. ISP 串口选择
 *    治具板 U1 有两个串口:
 *      USART1 = PA9/PA10  → 板上 4P 排针「串口1」(与 SWD 同座)
 *      USART3 = PB10/PB11 → 板上 2P 排针
 *    默认用 USART3 作为与被烧 FOC 板的通信口。
 *============================================================================*/
#define ISP_UART_INSTANCE       USART3
extern UART_HandleTypeDef       huart3;      /* usart.c 中定义 */
/* @note 用户已确认：先按此占位，后续可能更改。
 *       测径治具用的是 USART2（PC 走 USB CDC），USART3 是本板上与之对应的角色。 */

/*==============================================================================
 * 2. 被烧板 BOOT0 / RESET 控制引脚
 *
 * 【做法参考测径治具】测径治具是用 U1 的两个普通推挽 GPIO 直接驱动被烧板的
 * BOOT0 / RESET，进 bootloader 时序为:
 *       BOOT0 = H  →  RESET 拉低 10ms  →  RESET 拉高  →  等 50ms  →  发 0x7F
 * 本工程沿用同一做法与同一时序（见 isp_programmer.c 的 ISP_ST_CONNECT）。
 *
 * @note 测径治具用的是 U1 的 **PA8(BOOT0) / PB15(RESET)**；
 *       但本板这两个脚已被占用（PA8 = TIM1_CH1，PB15 = SPI2_MOSI → 外部 Flash U10），
 *       因此改用 U1 上同样空闲的两个脚作为占位:
 *           BOOT0 → PB1  (U1 pin19, 原理图上空闲)
 *           RESET → PC13 (U1 pin2,  原理图上空闲)
 *       用户已确认：先按此占位，后续接线定了再改（只改这两组宏）。
 *
 * @note 选脚注意（给后续改动的人）：
 *       ① **必须避开已被占用的脚**：PA8 = TIM1_CH1；PB12~PB15 = SPI2 接外部 Flash U10；
 *          PA9/PA10 = 调试口 USART1；PB10/PB11 = ISP 口 USART3；PA11/PA12 = USB。
 *       ② **PC13/PC14/PC15 属备份域且驱动能力弱（约 3mA）**。用作 RESET 输出没问题
 *          （被烧板 NRST 通常 10k 上拉 → 仅 0.33mA），但不建议再挂别的负载。
 *       ③ 若改用到其它端口，`ISP_GpioInit()` 已统一打开 GPIOA/B/C 的时钟，
 *          无需额外处理；改用其它端口（如 GPIOD）时需要在那里补一行时钟使能。
 *============================================================================*/
#define ISP_BOOT0_PORT          GPIOB
#define ISP_BOOT0_PIN           GPIO_PIN_1
#define ISP_RESET_PORT          GPIOC
#define ISP_RESET_PIN           GPIO_PIN_13

/**
  * @brief 是否已确认 BOOT0/RESET 的物理接线
  *        0 = 未确认 → ISP_Connect 不做复位时序, 直接发 0x7F 握手
  *            (适用于人工先把被烧板置于 bootloader 的场合)
  *        1 = 已确认 → 按 AN3155 时序: BOOT0=H → RESET 拉低 10ms → 拉高 → 等 50ms
  * @note 用 #ifndef 包住, 便于宿主测试用 -D 强制打开来验证引脚时序波形。
  */
#ifndef ISP_CTRL_PINS_CONFIRMED
#define ISP_CTRL_PINS_CONFIRMED 0
#endif

/*==============================================================================
 * 3. 被烧 MCU 写内存对齐
 *----------------------------------------------------------------------------
 * AN3155 对 Write Memory 的长度/地址有对齐要求，且不同器件族不一样:
 *   STM32F1 系列   : 4 字节对齐
 *   STM32G0/G4/L4 等: 8 字节对齐 (测径治具实测)
 *
 * 被烧板是"到时候外接"的，型号未定，所以这里**不猜**：
 * 统一按 8 字节补齐 —— 8 是 4 的倍数，因此同时满足两族的严格要求，
 * 代价只是每段镜像尾块多写最多 7 个 0xFF（写在镜像之外，无害）。
 * 地址侧本来就是 256 字节块边界，天然 8 字节对齐。
 *============================================================================*/
#define ISP_WRITE_ALIGN         8u

/* 器件族仅作记录/留档；对齐已统一为 8，无需按族切换。
 * 若将来要针对某族优化尾块长度，可改用 ISP_WRITE_ALIGN=4。 */
#define ISP_TARGET_F1           1
#define ISP_TARGET_G0           2

#ifndef ISP_TARGET_FAMILY
#define ISP_TARGET_FAMILY       ISP_TARGET_F1
#endif

/*==============================================================================
 * 4. 0x000D「查询固件版本号（被烧板中）」的被烧板应用层命令
 *    @warning 用户已明确：**本项先不管，以先把烧录流程跑通为先**。
 *             因此当前保持关闭，未配置时 0x000D 按协议返回 0x0C(失败) + 版本号 00 FF FF。
 *             拿到被烧板运行态固件的版本查询协议后，在
 *             isp_programmer.c 的 ISP_QueryTargetVersion() 里按 TODO 填即可。
 *============================================================================*/
#define FOC_APP_VER_QUERY_ENABLED   0

/*==============================================================================
 * 4.5 烧录后回读校验 (AN3155 Read Memory 0x11)
 *
 * 0 = 关闭 (默认)。烧录成功的判据仅来自写命令的 ACK。
 * 1 = 每写完一段镜像后, 逐 256B 回读目标 Flash 并与 SPI Flash 中的源镜像比对;
 *     不一致则报 ERR_PROG。
 *
 * ⚠ 为什么默认关闭:
 *   回读时序在不同 bootloader 版本上略有差异 (长度字节 N-1 后是否跟 (N-1)^0xFF
 *   校验字节、数据后是否补尾随 ACK)。若格式与实际器件不符, 回读会失败,
 *   于是**一次成功的烧录也会被判为 ERR_PROG**, 反而误导产线。
 *   因此 bring-up 阶段先保持关闭, 确认回读可用后再打开。
 *============================================================================*/
#ifndef ISP_VERIFY_ENABLED
#define ISP_VERIFY_ENABLED      0
#endif

/*==============================================================================
 * 5. 时序参数 (AN3155)
 *============================================================================*/
#define ISP_TIMEOUT_SYNC_MS     200u    /* 0x7F 同步等待 ACK               */
#define ISP_TIMEOUT_ACK_MS      500u    /* 普通命令 ACK                    */
#define ISP_TIMEOUT_ERASE_MS    15000u  /* 全片擦除 ACK (可能 >2s)          */
#define ISP_TIMEOUT_WRITE_MS    1000u   /* 页写入 ACK                      */
#define ISP_TIMEOUT_READ_MS     1000u   /* 回读数据超时                     */
#define ISP_RESET_LOW_MS        10u     /* RESET 拉低时间                   */
#define ISP_BOOT_SETTLE_MS      50u     /* RESET 释放后等 bootloader 就绪   */

#endif /* __ISP_CONFIG_H */
