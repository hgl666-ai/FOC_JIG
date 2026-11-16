/**
  ******************************************************************************
  * @file    flash_map.h
  * @brief   M1 治具 — PC 协议命令码 / 状态码 / 板载 SPI Flash 分区 / 目标板分区
  *
  * 移植自 FOC_JIG 工程（U1/Core/Inc/flash_map.h）的"固件上传"部分。
  * 与 FOC_JIG 保持逐字节相同的线协议：
  *   A5 5B | CMD(2) | SEQ(2) | LEN(2) | DATA | CRC16(2)   （字段均大端）
  *
  * ⚠ 换板适配只看"三、板载 SPI Flash 分区"和"五、目标板分区"：
  *   改 FW_SLOT_SIZE / 槽基址 / DUT_* 即可，上层上传与烧录状态机都不用动。
  ******************************************************************************
  */
#ifndef __FLASH_MAP_H
#define __FLASH_MAP_H

#include <stdint.h>

/*==============================================================================
 * 一、协议命令码 (CMD 为 2 字节, 大端)
 *============================================================================*/
#define MGR_CMD_FW_START        0x0009u   /* 固件 起始帧 (PC→治具, LEN=9)  */
#define MGR_CMD_FW_DATA         0x000Au   /* 固件 数据帧 (PC→治具, LEN=L)  */
#define MGR_CMD_FW_END          0x000Bu   /* 固件 结束帧 (PC→治具, LEN=0)  */
#define MGR_CMD_QUERY_FLASH_VER 0x000Cu   /* 查询治具 Flash 里的固件版本 (LEN=0) */
#define MGR_CMD_QUERY_DUT_VER   0x000Du   /* 查询 DUT 固件版本：UART5 发 M115，应答 状态(1)+版本(3) */
#define MGR_CMD_SWD_PROGRAM     0x0112u   /* SWD 烧录：把两个槽写到目标板 (LEN=0) */
#define MGR_CMD_SWD_PROBE       0x0013u   /* SWD 自检：只读，LEN=0 或 2(延时高,低) */
#define MGR_CMD_SWD_STATUS      0x0014u   /* 查询烧录进度 (LEN=0)               */
#define MGR_CMD_VOLT_TEST       0x0015u   /* 3 路电压检测 (LEN=0)               */
#define MGR_CMD_ADC_TEST        0x0016u   /* 3 路 ADC 检测 (LEN=0)              */
#define MGR_CMD_IOIN_TEST       0x0017u   /* 8 路 IO 输入检测 (LEN=0)           */
#define MGR_CMD_UART5_TEST      0x0099u   /* UART5 透传：CDC↔DUT USART1，直到治具复位 (LEN=0) */
#define MGR_CMD_UART5_COPY      0x0098u   /* 默认模式 UART5→USART1 原样复制开关：
                                           * LEN=0 翻转；LEN>=1 按 DATA[0] 设置 (0=关,非0=开) */

#define SWD_PROGRAM_ACK_3BYTES  0

/*
SWD 烧录: A5 5B 01 12 00 10 00 00 9E D8
SWD 自检: A5 5B 00 13 00 01 00 00 8A B4
UART5 通路自检: A5 5B 00 99 00 01 00 00 55 2D
0x0098 翻转：A5 5B 00 98 00 01 00 00 95 10
0x0098 打开：A5 5B 00 98 00 01 00 01 01 9C 54
0x0098 关闭：A5 5B 00 98 00 01 00 01 00 5C 95
*/

/*==============================================================================
 * 二、通用状态码 (应答 DATA 第 0 字节)
 *============================================================================*/
#define APP_STATUS_OK               0x00u   /* 成功                          */
#define APP_STATUS_BUSY             0x01u   /* 系统忙 (非 IDLE 收到命令)      */
#define APP_STATUS_ERR_SIZE         0x02u   /* 文件大小 / 参数异常            */
#define APP_STATUS_ERR_FLASH        0x03u   /* SPI Flash 读写失败            */
#define APP_STATUS_ERR_PROG         0x04u   /* 目标板 SWD 烧录失败           */
#define APP_STATUS_ERR_NO_FW        0x06u   /* 固件不存在 (大小为0或越界)     */
#define APP_STATUS_ERR_INCOMPLETE   0x07u   /* 固件不完整 (完整标志缺失)      */
#define APP_STATUS_ERR_COUNT        0x0Au   /* 上传字节数与声明不符          */
#define APP_STATUS_ERR_PARAM        0x0Bu   /* 参数错误                      */
#define APP_STATUS_ERR_TIMEOUT      0x0Cu   /* 上传超时 (10s 无新数据)        */
#define APP_STATUS_ERR_OVERFLOW     0x0Du   /* 上传溢出 (超出声明大小)        */
#define APP_STATUS_ERR_NIY          0xFFu   /* 命令未实现                    */

/*==============================================================================
 * 三、治具板载 SPI Flash (GD25Q128, 16MB) 双槽布局
 *----------------------------------------------------------------------------
 * Slot0 = Bootloader，Slot1 = App —— 一次上电会话里连续上传两次，
 * 每收到一条 FW_END 就切槽（顺序与 FOC_JIG 完全一致，PC 端不用改）。
 *
 * 槽大小按"目标分区上限"取：Boot 40KB + App 88KB，所以每槽给 128KB。
 * （FOC_JIG 是 64KB/槽，这里必须放大 —— 88KB 的 App 塞不进 64KB 槽。）
 *
 * 槽头部 8 字节：
 *   [0..3] 文件大小 u32 小端
 *   [4..6] 版本号 主.次.修订
 *   [7]    完整标志 0xA5
 *============================================================================*/
#define FW_SLOT_SIZE            (128u * 1024u)    /* 每槽 128KB (> App 88KB)    */
#define FW_SLOT0_BASE           (0x00000000u)     /* Slot0 = Bootloader         */
#define FW_SLOT1_BASE           (0x00020000u)     /* Slot1 = App                */
#define FW_HEAD_SIZE            (8u)              /* 槽头部字节数                */
#define FW_MAX_IMAGE_SIZE       (FW_SLOT_SIZE - FW_HEAD_SIZE)

#define FW_HEAD_OFF_SIZE        (0u)              /* [0..3] 文件大小 u32 小端   */
#define FW_HEAD_OFF_VER         (4u)              /* [4..6] 版本号 主.次.修订   */
#define FW_HEAD_OFF_FLAG        (7u)              /* [7]    完整标志            */

#define FW_FLAG_COMPLETE        (0xA5u)           /* 完整                       */
#define FW_FLAG_ERASED          (0xFFu)           /* 擦除态 (=不完整)           */

/*==============================================================================
 * 四、缓冲与时序
 *============================================================================*/
#define APP_PAGE_SIZE           (256u)            /* SPI Flash 页大小            */
#define FRAME_MAX_DATA          (256u)            /* 单帧 DATA 上限 (协议 1~256) */
#define APP_FRAME_BUF_SIZE      (8u + FRAME_MAX_DATA + 2u)
#define APP_UPLOAD_TIMEOUT_MS   (10000u)          /* 上传 10s 无新数据超时        */

/*==============================================================================
 * 五、目标板 STM32F105RBT6 分区（SWD 烧录用）
 *----------------------------------------------------------------------------
 * 目标 128KB Flash = 64 个 2KB 页（HAL：STM32F105xC → FLASH_PAGE_SIZE=0x800）。
 * 分区由目标工程的链接脚本给定，必须落在 2KB 页边界上：
 *
 *   Boot : 0x08000000 + 0xA000 (40KB)  → 页 0..19
 *   App  : 0x0800A000 + 0x16000 (88KB) → 页 20..63（正好到 Flash 末尾）
 *
 * 两个值都要满足：能被 2KB 整除（已满足），且首尾相接、不越 128KB。
 * 改分区只改这三个宏；SWD 自检（0x0013）会读 DUT_APP_ADDR 处的栈顶字，
 * 上板可直接核对 App 基址假设对不对。
 *============================================================================*/
#define DUT_BOOT_ADDR           (0x08000000u)
#define DUT_BOOT_SIZE           (0xA000u)         /* 40KB = 20 页              */
#define DUT_APP_ADDR            (0x0800A000u)     /* App 起点（2KB 页对齐）     */
#define DUT_APP_SIZE            (0x16000u)        /* 88KB = 44 页              */
#define DUT_FLASH_SIZE          (128u * 1024u)    /* F105RB 片内 Flash 容量     */
#define DUT_PAGE_SIZE           (0x800u)          /* F105 页大小 2KB            */

/*==============================================================================
 * 六、编译期不变量自检 (负数组长度技巧, ARMCC5 / gcc 均支持)
 *============================================================================*/
#define M1_STATIC_ASSERT(cond, name) typedef char m1_static_assert_##name[(cond) ? 1 : -1]

M1_STATIC_ASSERT(APP_PAGE_SIZE == 256u,                page_size_must_be_256);
M1_STATIC_ASSERT(FW_HEAD_SIZE == 8u,                   head_must_be_8_bytes);
M1_STATIC_ASSERT((FW_SLOT_SIZE % 4096u) == 0u,         slot_must_be_sector_aligned);
M1_STATIC_ASSERT((FW_SLOT0_BASE % 4096u) == 0u,        slot0_must_be_sector_aligned);
M1_STATIC_ASSERT((FW_SLOT1_BASE % 4096u) == 0u,        slot1_must_be_sector_aligned);
M1_STATIC_ASSERT((FW_SLOT0_BASE + FW_SLOT_SIZE) <= FW_SLOT1_BASE, slots_must_not_overlap);
M1_STATIC_ASSERT((FW_SLOT1_BASE + FW_SLOT_SIZE) <= (16u * 1024u * 1024u), slots_must_fit_chip);
M1_STATIC_ASSERT(FW_MAX_IMAGE_SIZE > 0u,               max_image_size_positive);
M1_STATIC_ASSERT(FRAME_MAX_DATA <= APP_PAGE_SIZE,      one_frame_must_fit_one_page);
M1_STATIC_ASSERT(FRAME_MAX_DATA <= 256u,               frame_data_must_fit_u16_protocol);

/* 槽容量必须装得下目标分区（这条能挡住"App 变大后忘了放大槽"） */
M1_STATIC_ASSERT(FW_MAX_IMAGE_SIZE >= DUT_BOOT_SIZE,   slot_must_hold_boot_image);
M1_STATIC_ASSERT(FW_MAX_IMAGE_SIZE >= DUT_APP_SIZE,    slot_must_hold_app_image);

/* 目标分区：连续、页对齐、正好到 Flash 末尾 */
M1_STATIC_ASSERT(DUT_BOOT_ADDR == 0x08000000u,         dut_boot_must_start_at_flash_base);
M1_STATIC_ASSERT((DUT_BOOT_ADDR + DUT_BOOT_SIZE) == DUT_APP_ADDR, dut_partitions_must_be_contiguous);
M1_STATIC_ASSERT((DUT_BOOT_SIZE % DUT_PAGE_SIZE) == 0u, dut_boot_must_be_page_aligned);
M1_STATIC_ASSERT((DUT_APP_ADDR % DUT_PAGE_SIZE) == 0u,  dut_app_must_be_page_aligned);
M1_STATIC_ASSERT((DUT_APP_SIZE % DUT_PAGE_SIZE) == 0u,  dut_app_must_be_page_aligned_too);
M1_STATIC_ASSERT((DUT_APP_ADDR + DUT_APP_SIZE) == (DUT_BOOT_ADDR + DUT_FLASH_SIZE),
                                                        dut_app_must_end_at_flash_end);

#endif /* __FLASH_MAP_H */
