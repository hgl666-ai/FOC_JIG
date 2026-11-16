/**
  ******************************************************************************
  * @file    SWD_FLASH.h
  * @brief   M1 治具 —— 目标板 SWD 烧录（SWDIO/SWCLK + NRST/BOOT0）
  *
  * 全部实现都在 SWD_FLASH.c（单文件，按层用分隔线分开，改哪层看哪段）：
  *   PHY(位/包) → DP(调试端口) → MEM-AP(内存访问) → Cortex-M(停核/看门狗/复位)
  *   → F105 Flash(页擦/半字编程) → 非阻塞烧录状态机
  *
  * 与"FOC 治具 U1 已成功版本"的关系：协议层逐行相同，只替换了 F4 专属部分
  *   （扇区表→2KB 页、32 位字→16 位半字、FLASH 寄存器位、DBGMCU 寄存器）。
  *
  * 换板 / 换目标只改四处：
  *   1) 本文件"一、主机侧引脚与速度"
  *   2) 本文件"一之二、目标板复位/启动控制"（NRST=PC6 / BOOT0=PC7）
  *   3) 本文件"四、F105 目标常量"
  *   4) flash_map.h 里的 DUT_BOOT_ADDR / DUT_APP_ADDR / DUT_*_SIZE
  *
 * ⚠ DUT SWD 引脚由 STLINK_USE_SWDIO_SWCLK 选择：
 *   0 = PD2(SWCLK)/PC12(SWDIO)：PA13/PA14 保留 ST-Link 调试功能
 *       （第一次 SWD 命令时写 SWJ_CFG=010，恢复/保持 SW-DP）；
 *   1 = PA13(SWCLK)/PA14(SWDIO)：写 SWJ_CFG=100 关掉主机自己的 SW-DP，
 *       这两脚改作 DUT SWD。此模式下主机不能再被 ST-Link 调试。
  ******************************************************************************
  */
#ifndef __SWD_FLASH_H
#define __SWD_FLASH_H

#include <stdint.h>

#include "main.h"       /* GPIOA / HAL 类型 / __HAL_AFIO_* */
#include "flash_map.h"  /* DUT_* 目标分区、APP_STATUS_* */

/*==============================================================================
 * 一、主机侧引脚（换板只改这 6 行）
 *----------------------------------------------------------------------------
 * STLINK_USE_SWDIO_SWCLK:
 *   1 = DUT SWD 使用 PA13(SWCLK)/PA14(SWDIO)（原 ST-Link 调试脚），
 *       此时必须关掉主机自己的 SW-DP；
 *   0 = DUT SWD 使用 PD2(SWCLK)/PC12(SWDIO)，PA13/PA14 保留 ST-Link 功能。
 *============================================================================*/
#ifndef STLINK_USE_SWDIO_SWCLK
#define STLINK_USE_SWDIO_SWCLK    0
#endif
#if STLINK_USE_SWDIO_SWCLK
#define SWD_CLK_PORT        GPIOA
#define SWD_CLK_PIN         GPIO_PIN_13
#define SWD_CLK_PINNUM      13u

#define SWD_DIO_PORT        GPIOA
#define SWD_DIO_PIN         GPIO_PIN_14
#define SWD_DIO_PINNUM      14u
#else
#define SWD_CLK_PORT        GPIOD
#define SWD_CLK_PIN         GPIO_PIN_2
#define SWD_CLK_PINNUM      2u

#define SWD_DIO_PORT        GPIOC
#define SWD_DIO_PIN         GPIO_PIN_12
#define SWD_DIO_PINNUM      12u
#endif

/* 主机自己的 SW-DP（PA13/PA14）是否关掉：默认跟随 STLINK_USE_SWDIO_SWCLK。
 *   1 = DUT SWD 占用了 PA13/PA14，必须关掉主机 SW-DP；
 *   0 = DUT SWD 在 PD2/PC12，PA13/PA14 保持 ST-Link 调试功能（不写 SWJ_CFG）。 */
#ifndef SWD_FREE_DEBUG_PINS
#define SWD_FREE_DEBUG_PINS  STLINK_USE_SWDIO_SWCLK
#endif

/*==============================================================================
 * 一之二、目标板复位/启动控制（NRST = PC6，BOOT0 = PC7）
 *----------------------------------------------------------------------------
 * 烧录流程：
 *   ① 烧录前：BOOT0 低 + NRST 20ms 复位 → 让用户程序先跑一下；
 *   ② 进下载模式：BOOT0 高保持 100ms → NRST 20ms 复位 → 系统 bootloader；
 *   ③ SWD 连接 → 擦除 → 编程 → 回读校验；
 *   ④ 烧录完成后：BOOT0 低 + NRST 20ms 复位 → 运行新程序。
 *
 * ⚠ 若治具板上这两根线经过反相三极管/光耦，只改下面的极性宏和驱动方式宏，
 *   不要改流程代码。
 *============================================================================*/
#define DUT_NRST_PORT          GPIOC
#define DUT_NRST_PIN           GPIO_PIN_6
#define DUT_BOOT_PORT          GPIOC
#define DUT_BOOT_PIN           GPIO_PIN_7

/* NRST 极性：目标板 NRST 有 3.3V 上拉，**低电平复位**（直连，无反相）。
 *   PC6 高 → NRST 高（释放）
 *   PC6 低 → NRST 低（复位） */
#ifndef DUT_NRST_ACTIVE_LOW
#define DUT_NRST_ACTIVE_LOW    1
#endif

/* 1 = BOOT0 高电平进系统 bootloader（正常烧录/运行保持低） */
#ifndef DUT_BOOT_ACTIVE_HIGH
#define DUT_BOOT_ACTIVE_HIGH   1
#endif

/* NRST 驱动方式：开漏（目标板已有 3.3V 上拉，释放靠上拉回高，避免和板上
 * 复位电路/复位芯片对顶）。若治具侧没有上拉且目标板上拉也被去掉，改成
 * GPIO_MODE_OUTPUT_PP。 */
#ifndef DUT_NRST_MODE
#define DUT_NRST_MODE          GPIO_MODE_OUTPUT_OD
#endif

/* 1 = 烧录流程使用 NRST 复位保持 + 释放；0 = 回到旧行为（不复位控制） */
#ifndef SWD_USE_DUT_NRST
#define SWD_USE_DUT_NRST       1
#endif

/* NRST 低电平脉宽（ms）。STM32F1 只需要几十 µs，20ms 已非常保守；
 * 原来的 500ms 纯属浪费（每次复位多等半秒）。 */
#ifndef SWD_RESET_HOLD_MS
#define SWD_RESET_HOLD_MS      20u
#endif

/* BOOT0 拉高后、NRST 复位前的建立时间（ms）。BOOT 脚在复位后几个 SYSCLK
 * 就采样，100ms 远超需求，只是求稳。 */
#ifndef SWD_BOOT_SETUP_MS
#define SWD_BOOT_SETUP_MS      100u
#endif

/* 烧录前"BOOT 低 + NRST 复位"后，让用户程序跑多久（ms）再切下载模式。 */
#ifndef SWD_APP_RUN_MS
#define SWD_APP_RUN_MS         50u
#endif

/*==============================================================================
 * 二、SWCLK 速度
 *----------------------------------------------------------------------------
 * 每个半周期插入的空循环次数（0 = 最快，数值越大越慢）。
 *
 * 标定记录（FOC 治具 U1，同为 F103@72MHz，约 20cm 飞线）：
 *   延时 128 ≈ 62kHz；64 ≈ 125kHz；32 ≈ 250kHz；12 ≈ 500kHz（原默认）
 * 本板线长未知，先沿用 12；上板后用 0x0013 SWD_PROBE 现场扫档，
 * 参数 DATA=[延时高字节, 延时低字节]，不影响烧录流程。
 *============================================================================*/
#ifndef SWD_DELAY_LOOPS
#define SWD_DELAY_LOOPS     4u
#endif
/*
SWD 自检: A5 5B 00 13 00 01 00 00 8A B4
SWD 自检 + 延时 8：A5 5B 00 13 00 01 00 02 00 08 A1 56
SWD 自检 + 延时 6：A5 5B 00 13 00 01 00 02 00 06 65 D7
SWD 自检 + 延时 4：A5 5B 00 13 00 01 00 02 00 04 A4 56
SWD 自检 + 延时 64：A5 5B 00 13 00 01 00 02 00 40 97 56
*/

/*==============================================================================
 * 三、超时与重试（照 DAPLink / DAPProg 的取值）
 *============================================================================*/
#define SWD_RETRY_WAIT          100u     /* WAIT 应答最大重试次数            */
#define SWD_RETRY_FAULT         2u       /* FAULT 应答最大重试（先清 sticky）*/
#define SWD_RETRY_NOACK         8u       /* 畸形 ACK 最大重试（飞线噪声自愈）*/
/* 链路失步/回读不符后的自动重来次数。总烧录次数 = 1 + SWD_RECOVERY_MAX：
 *   0 = 只烧一次，失败立即退出（当前设置）；
 *   1 = 失败后重试一次（总共 2 次），以此类推。 */
#define SWD_RECOVERY_MAX        0u
#define SWD_CONNECT_RETRY       3u       /* 连接（线复位+IDCODE）重试次数    */
#define SWD_POLL_MAX            500000u  /* BSY 轮询上限，防死循环           */
#define F1_FLASH_TIMEOUT_MS     5000u    /* 擦/写等待 BSY 清零的超时         */

/*==============================================================================
 * 四、F105 目标常量（STM32F105RBT6）
 *============================================================================*/
#define F1_FLASH_BASE           0x08000000u
#define F1_FLASH_SIZE           (128u * 1024u)
#define F1_PAGE_SIZE            0x800u          /* 2KB（HAL：STM32F105xC → 0x800）*/
#define F1_PAGE_COUNT           (F1_FLASH_SIZE / F1_PAGE_SIZE)   /* 64 页 */

/* 目标 DEV_ID（DBGMCU_IDCODE[11:0]）：F105/F107 连接型 = 0x418 */
#define F1_DEV_ID              0x418u
/* Cortex-M3 的 SW-DP IDCODE（仅用于日志核对，不硬性拒绝） */
#define SWD_IDCODE_CM3         0x1BA01477u

/*==============================================================================
 * 五、返回码 / 线级约定（内部使用，公开只为让上层看得懂错误号）
 *============================================================================*/
typedef enum {
    SWD_OK            = 0,
    SWD_ERR_NO_ACK    = -1,  /* 无有效 ACK：目标没上电 / 线断 / 脚被占 */
    SWD_ERR_WAIT      = -2,  /* 目标回 WAIT（重试用尽）                */
    SWD_ERR_FAULT     = -3,  /* 目标回 FAULT（重试用尽）               */
    SWD_ERR_PARITY    = -4,  /* 读回数据奇偶校验错                     */
    SWD_ERR_PARAM     = -5,  /* 参数非法                               */
    SWD_ERR_TIMEOUT   = -6,  /* 轮询超时                               */
    SWD_ERR_FLASH     = -7,  /* 目标 Flash 控制器报错                  */
    SWD_ERR_NOT_HALTED= -8,  /* 内核未能停止                           */
    SWD_ERR_VERIFY    = -9   /* 回读校验不一致                         */
} swd_status_t;

/*==============================================================================
 * 六、对外接口
 *============================================================================*/
/* 烧录状态（0x0014 状态查询 / 日志用） */
typedef enum {
    SWDP_ST_IDLE = 0,
    SWDP_ST_CONNECT,      /* 线复位 + JTAG→SWD + IDCODE + 上电 + 选 AP */
    SWDP_ST_HALT,         /* 停核 + 冻结看门狗 + 认型号 + 校验两份镜像头 */
    SWDP_ST_ERASE,        /* 按页擦除（一步一页）                       */
    SWDP_ST_PROGRAM,      /* 按块写（一步 128B）                        */
    SWDP_ST_VERIFY,       /* 按块回读校验                               */
    SWDP_ST_GO,           /* SYSRESETREQ 复位运行                       */
    SWDP_ST_OK,
    SWDP_ST_FAIL
} swdp_state_t;

/* 0x0013 SWD_PROBE 自检结果（只读，不擦不写） */
typedef struct {
    uint8_t  status;      /* APP_STATUS_*                                  */
    uint8_t  swd_err;     /* 失败时的 swd_status_t 取负                    */
    uint8_t  rdprt;       /* 1 = 目标开着读保护（擦写必失败）              */
    uint16_t delay;       /* 当前半周期延时循环数                          */
    uint16_t dev_id;      /* DBGMCU_IDCODE[11:0]，F105 = 0x418             */
    uint16_t flash_kb;    /* 0x1FFFF7E0 读到的 Flash 容量（KB），应为 128  */
    uint32_t idcode;      /* SW-DP IDCODE，Cortex-M3 = 0x1BA01477          */
    uint32_t boot_sp;     /* 0x08000000 处的字（Boot 栈顶，空片=0xFFFFFFFF）*/
    uint32_t app_sp;      /* DUT_APP_ADDR 处的字（App 栈顶）               */
    uint8_t  mem_err;     /* 第一个失败的内存/AP 访问错误码（0 = 全部成功）：
                           *   1 = 无 ACK、2 = WAIT、3 = FAULT、4 = 奇偶错… */
    uint32_t ap_idr;      /* AHB-AP 的 IDR：F105/Cortex-M3 为 0x14770011，
                           * F4/Cortex-M4 为 0x24770011（AP 版本位不同）      */
    uint32_t dp_stat;     /* DP CTRL/STAT 原值（看上电位与 sticky 位）      */
    /* 逐项错误码：每读一项失败后都会做一次线复位重同步，
     * 所以这些码互相独立，能看出"哪一项失败"而不是"第一个失败后全崩"。 */
    uint8_t  err_dev;     /* 读 DBGMCU_IDCODE (0xE0042000) 的错误码        */
    uint8_t  err_kb;      /* 读 Flash 容量   (0x1FFFF7E0) 的错误码        */
    uint8_t  err_obr;     /* 读 FLASH_OBR    (0x4002201C) 的错误码        */
    uint8_t  err_boot;    /* 读 Boot 基址    (DUT_BOOT_ADDR) 的错误码     */
    uint8_t  err_app;     /* 读 App 基址     (DUT_APP_ADDR) 的错误码      */
    uint8_t  err_idr;     /* 读 AP IDR       (AP 0xFC) 的错误码           */
    uint8_t  err_dpstat;  /* 读 DP CTRL/STAT (0x04) 的错误码              */
    uint8_t  ram_ok;      /* RAM 写读自检：1 = 写进去又读回来一致         */
    uint8_t  err_ram;     /* RAM 写读自检的错误码（0 = 写和读都成功）     */
} swd_probe_t;

/** 初始化目标板复位/启动控制脚（PC6=NRST, PC7=BOOT0），上电后调用一次即可。
 *  只配置 GPIO，不碰 SWD 主机引脚，也不会关闭主机自己的 SW-DP。 */
void    SWDP_DutCtrlInit(void);

/** 初始化 SWD 引脚 + 复位内部状态（会按需关闭主机 SW-DP，见文件头） */
void    SWDP_Init(void);

/** 启动烧录流程（前置检查由调用方做：两份镜像都存在且不超分区） */
void    SWDP_Start(void);

/** 主循环步进：一次只推进一步（一次握手 / 擦一页 / 写 128B / 校验一块 / 复位） */
void    SWDP_Step(void);

/** @retval 1 = 仍在烧录中；0 = 已结束（成功或失败） */
uint8_t SWDP_Busy(void);

/** @retval 结束时返回 APP_STATUS_OK 或具体错误码 */
uint8_t SWDP_Result(void);

/** 当前状态机状态（swdp_state_t） */
uint8_t SWDP_State(void);

/** 本次烧录要处理的总字节数（Boot + App） */
uint32_t SWDP_TotalBytes(void);

/* ---- 诊断 ---- */
uint32_t SWDP_LastIdcode(void);
uint16_t SWDP_LastDevId(void);
uint32_t SWDP_LastOffset(void);      /* 当前镜像已处理到哪个偏移 */
uint8_t  SWDP_LastSwdError(void);    /* 原始的 swd_status_t 取负  */
uint8_t  SWDP_FailState(void);       /* 失败时停在哪个状态（swdp_state_t） */

/* ---- 0x0013 自检：不擦不写，只证明"线通 + 认得目标" ---- */
uint8_t SWDP_Probe(swd_probe_t *out);

/* ---- 速度调节（运行期，0x0013 的 DATA 用） ---- */
void     SWDP_SetDelay(uint16_t loops);
uint16_t SWDP_GetDelay(void);

#endif /* __SWD_FLASH_H */
