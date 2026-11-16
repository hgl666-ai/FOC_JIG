/**
  ******************************************************************************
  * @file    UART_FLASH.h
  * @brief   M1 治具 —— 通过 UART5 走 DUT 系统 bootloader 烧录（AN3155）
  *
  * 接线（与 SWD 方案共用 NRST/BOOT0，但 SWD 位翻转脚改为 UART5 外设脚）：
  *   治具 UART5_TX(PC12) → DUT USART1_RX(PA10)
  *   治具 UART5_RX(PD2)  ← DUT USART1_TX(PA9)
  *   治具 PC6(NRST)      → DUT NRST（开漏输出，低有效）
  *   治具 PC7(BOOT0)     → DUT BOOT0（高 = 系统存储器 bootloader）
  *
  * ⚠ DUT 的 BOOT1(PB2) 必须为低。F105 只有 BOOT0=1、BOOT1=0 才进系统
  *   bootloader；多数目标板已经把 BOOT1 下拉，若没有，需要额外接一根线。
  *
  * 与 SWD_FLASH.h 的关系：
  *   - 本文件是独立实现，不引用 SWD_FLASH.c 的任何符号；
  *   - FLASH_MODE_UART5=1 时 SWD_FLASH.c 整体不编译，data_process.c 只调本文件；
  *   - FLASH_MODE_UART5=0 时本文件整体不编译，回到原有 SWD 方案。
  ******************************************************************************
  */
#ifndef __UART_FLASH_H
#define __UART_FLASH_H

#include <stdint.h>

#include "flash_mode.h"

#if FLASH_MODE_UART5

#include "main.h"       /* GPIOA/GPIOC/GPIOD、HAL 类型 */
#include "flash_map.h"  /* DUT_* 目标分区、APP_STATUS_* */

/*==============================================================================
 * 一、UART5 参数
 *----------------------------------------------------------------------------
 * STM32F1 系统 bootloader 的 USART1 自动波特率检测建议不要超过 115200。
 * 若现场线很短、目标 bootloader 实测支持更高，可自行改这个宏试：
 *   115200 → 约 8 秒 / 70KB（含擦除与 ACK 开销）
 *   230400 → 约 5 秒（能否识别取决于 bootloader 版本，需实测）
 *
 * ⚠ 关键：AN2606 Table 48 明确 STM32F105/107 系统 bootloader 的 USART1 是
 *   **8 数据位 + 偶校验 + 1 停止位（8E1）**，不是 8N1。UART_FLASH.c 用
 *   UART5_SetConfig(UARTF_BAUD, UART_PARITY_EVEN)，不要改成 NONE。
 *   HAL 里必须配成 UART_WORDLENGTH_9B + UART_PARITY_EVEN 才是 8 数据位+校验；
 *   UART_WORDLENGTH_8B + EVEN 实际只有 7 数据位。
 *   0x0099 透传跟 DUT App 通信时才用 8N1 256000。
 *============================================================================*/
#ifndef UARTF_BAUD
#define UARTF_BAUD              115200u
#endif

/* 0x0099 透传用的波特率：必须跟 DUT 用户程序一致。
 * 当前 DUT shredder 的 USART1 是 256000（shredder/Core/Src/usart.c），8N1。 */
#ifndef UARTF_TEST_BAUD
#define UARTF_TEST_BAUD         256000u
#endif

/* 单包写入字节数：AN3155 规定 1~256，且必须是 4 的倍数（F1 内部按字编程）。
 * 256 是协议上限，也是吞吐最高的选择。 */
#ifndef UARTF_CHUNK_BYTES
#define UARTF_CHUNK_BYTES       256u
#endif

/* AN3155 单包上限 256；且必须 4 的倍数，否则下一包地址会不对齐。
 * 这样写错时直接编译报错，而不是上板才失败。 */
#if (UARTF_CHUNK_BYTES == 0u) || (UARTF_CHUNK_BYTES > 256u) || \
    ((UARTF_CHUNK_BYTES % 4u) != 0u)
#error "UARTF_CHUNK_BYTES must be a multiple of 4 in range 4..256"
#endif

/* 1 = 写完后逐个 256B 块回读比对（最稳，但 @115200 会多花约 6 秒）
 * 0 = 只做首字（Boot/App 的栈顶）交叉检查（当前默认，速度优先） */
#ifndef UARTF_VERIFY_ENABLE
#define UARTF_VERIFY_ENABLE     0
#endif

/*==============================================================================
 * 二、目标板复位/启动控制（与 SWD 方案相同的 PC6/PC7）
 *============================================================================*/
#define UF_DUT_NRST_PORT        GPIOC
#define UF_DUT_NRST_PIN         GPIO_PIN_6
#define UF_DUT_BOOT_PORT        GPIOC
#define UF_DUT_BOOT_PIN         GPIO_PIN_7

/* NRST 有 3.3V 上拉、低电平复位；开漏只负责拉低，释放靠板上拉。 */
#ifndef UF_DUT_NRST_ACTIVE_LOW
#define UF_DUT_NRST_ACTIVE_LOW  1
#endif

/* BOOT0 高电平进系统 bootloader，正常运行保持低。 */
#ifndef UF_DUT_BOOT_ACTIVE_HIGH
#define UF_DUT_BOOT_ACTIVE_HIGH 1
#endif

#ifndef UF_DUT_NRST_MODE
#define UF_DUT_NRST_MODE        GPIO_MODE_OUTPUT_OD
#endif

/* 复位/启动时序（ms） */
#ifndef UF_RESET_HOLD_MS
#define UF_RESET_HOLD_MS        20u     /* NRST 低电平脉宽 */
#endif
#ifndef UF_BOOT_SETUP_MS
#define UF_BOOT_SETUP_MS        100u    /* BOOT0 拉高后、复位前的建立时间 */
#endif
#ifndef UF_APP_RUN_MS
#define UF_APP_RUN_MS           50u     /* 烧录前先让用户程序跑一下的时间 */
#endif
#ifndef UF_BOOT_READY_MS
#define UF_BOOT_READY_MS        50u     /* 复位释放后等 bootloader 启动 */
#endif

/*==============================================================================
 * 三、状态机状态（与 SWD 的 swdp_state_t 数值对齐，0x0014 查询兼容）
 *============================================================================*/
typedef enum {
    UFP_ST_IDLE = 0,
    UFP_ST_ENTER,        /* 1 = 进 bootloader + 0x7F 同步 + 读 ID/版本/命令表 */
    UFP_ST_PREPARE,      /* 2 = 校验两份镜像头、算擦除页范围（擦除之前！）   */
    UFP_ST_ERASE,        /* 3 = 整片擦除 / 页擦除                          */
    UFP_ST_PROGRAM,      /* 4 = 256B/包写入                                */
    UFP_ST_VERIFY,       /* 5 = 回读比对（UARTF_VERIFY_ENABLE=1 时才走）    */
    UFP_ST_GO,           /* 6 = BOOT0 低 + 复位运行新程序                  */
    UFP_ST_OK,           /* 7 = 成功                                       */
    UFP_ST_FAIL          /* 8 = 失败                                       */
} uartf_state_t;

/* 原始错误码（0x0012 迟到的 ACK 第 2 字节 / 日志用） */
typedef enum {
    UF_OK           = 0,
    UF_ERR_TIMEOUT  = 1,   /* 等 ACK 超时 / 读数据超时           */
    UF_ERR_NACK     = 2,   /* 目标回 0x1F                        */
    UF_ERR_PROTO    = 3,   /* 收到非法字节 / 应答长度异常        */
    UF_ERR_FLASH    = 4,   /* 治具 SPI Flash 读镜像失败          */
    UF_ERR_PARAM    = 5,   /* 参数非法（长度/地址）              */
    UF_ERR_VERIFY   = 6,   /* 回读不一致                         */
    UF_ERR_UART     = 7,   /* UART5 发送失败                     */
    UF_ERR_READ     = 8    /* 回读命令失败                       */
} uartf_err_t;

/* 0x0013 自检结果（不擦不写，只同步 + 读 ID/版本/栈顶） */
typedef struct {
    uint8_t  status;       /* APP_STATUS_OK / APP_STATUS_ERR_PROG */
    uint8_t  uart_err;     /* 失败时的 uartf_err_t               */
    uint8_t  rdp;          /* 1 = 目标开了读保护                 */
    uint8_t  bl_version;   /* bootloader 版本                    */
    uint16_t pid;          /* Get ID 返回的 Product ID           */
    uint32_t boot_sp;      /* 0x08000000 处的字（栈顶）           */
    uint32_t app_sp;       /* DUT_APP_ADDR 处的字（栈顶）         */
} uartf_probe_t;

/*==============================================================================
 * 四、对外接口（与 SWD_FLASH.h 的 SWDP_* 一一对应，便于 data_process 切换）
 *============================================================================*/
/** 初始化 PC6/PC7 和 UART5 波特率（幂等） */
void     UARTF_DutCtrlInit(void);

/** 启动烧录：BOOT0 低+复位跑 App → BOOT0 高+复位进 bootloader */
void     UARTF_Start(void);

/** 让 DUT 运行用户程序：BOOT0 低 + NRST 复位。
 *  0x0099 UART5 通路自检前调用，确保 DUT 不在 bootloader 里。 */
void     UARTF_RunApp(void);

/** 主循环步进：一次完成一个阶段/一包 256B */
void     UARTF_Step(void);

/** @retval 1 = 仍在烧录中；0 = 已结束（成功或失败） */
uint8_t  UARTF_Busy(void);

/** @retval 结束时返回 APP_STATUS_OK 或具体错误码 */
uint8_t  UARTF_Result(void);

/** 当前状态机状态（uartf_state_t） */
uint8_t  UARTF_State(void);

/** 本次要处理的总字节数（Boot + App） */
uint32_t UARTF_TotalBytes(void);

/** 当前镜像已处理到哪个偏移 */
uint32_t UARTF_LastOffset(void);

/** 原始错误码（uartf_err_t） */
uint8_t  UARTF_LastError(void);

/** 失败时停在哪个状态（uartf_state_t） */
uint8_t  UARTF_FailState(void);

/** 0x0013 自检：进 bootloader 同步并读 ID/版本/两个栈顶，然后复位运行 App。
 *  失败也会把 DUT 退出 bootloader，不会把目标留在下载模式。 */
uint8_t  UARTF_Probe(uartf_probe_t *out);

#endif /* FLASH_MODE_UART5 */

#endif /* __UART_FLASH_H */
