/**
  ******************************************************************************
  * @file    flash_map.h
  * @brief   FOC 烧录治具 — 命令码 / 状态码 / Flash 分区 / 目标板地址 定义
  * @note    协议来源: INFO/M1 测试治具PC端通讯协议.xlsx
  ******************************************************************************
  */
#ifndef __FLASH_MAP_H
#define __FLASH_MAP_H

#include <stdint.h>

/*==============================================================================
 * 一、协议命令码 (CMD 为 2 字节, 大端)
 *============================================================================*/
#define MGR_CMD_FW_START        0x0009u   /* 固件 起始帧       (PC→治具, LEN=9)  */
#define MGR_CMD_FW_DATA         0x000Au   /* 固件 数据帧       (PC→治具, LEN=L)  */
#define MGR_CMD_FW_END          0x000Bu   /* 固件 结束帧       (PC→治具, LEN=0)  */
#define MGR_CMD_QUERY_FLASH_VER 0x000Cu   /* 查询固件版本(Flash中) (LEN=0)       */
#define MGR_CMD_QUERY_FOC_VER   0x000Du   /* 查询固件版本(FOC板中) (LEN=0)       */
/* [修正] 原写成 0x0070 —— 那是把 xlsx 单元格里的十进制 `112` 误读成十六进制。
 *       正确值是 0x0112（与测径治具 PROGRAM 命令一致）。*/
#define MGR_CMD_PROGRAM         0x0112u   /* 烧录Flash固件到FOC板  (LEN=0)       */

/*==============================================================================
 * 二、通用状态码
 *============================================================================*/
#define APP_STATUS_OK               0x00u   /* 成功                          */
#define APP_STATUS_BUSY             0x01u   /* 系统忙 (非 IDLE 收到命令)      */
#define APP_STATUS_ERR_SIZE         0x02u   /* 文件大小异常                  */
#define APP_STATUS_ERR_FLASH        0x03u   /* SPI Flash 读写失败            */
#define APP_STATUS_ERR_PROG         0x04u   /* FOC 板 ISP 烧录失败           */
#define APP_STATUS_ERR_TEST         0x05u   /* 治具测试失败                  */
#define APP_STATUS_ERR_NO_FW        0x06u   /* 固件不存在 (大小为0或越界)     */
#define APP_STATUS_ERR_INCOMPLETE   0x07u   /* 固件不完整 (完整标志缺失)      */
#define APP_STATUS_ERR_CRC          0x08u   /* 帧 CRC 错误 (仅诊断, 不回 ACK) */
#define APP_STATUS_ERR_SEQ          0x09u   /* 帧序号非递增 (仅警告, 不拒绝)  */
#define APP_STATUS_ERR_COUNT        0x0Au   /* 上传字节数与声明不符          */
#define APP_STATUS_ERR_PARAM        0x0Bu   /* 参数错误                      */
#define APP_STATUS_ERR_TIMEOUT      0x0Cu   /* 上传超时 (10s 无新数据)        */
#define APP_STATUS_ERR_OVERFLOW     0x0Du   /* 上传溢出 (超出声明大小)        */
#define APP_STATUS_ERR_NIY          0xFFu   /* 命令未实现                    */

/*==============================================================================
 * 三、治具板载 SPI Flash (GD25Q128, 16MB) 双槽布局
 *----------------------------------------------------------------------------*/
#define FW_SLOT_SIZE            (64u * 1024u)     /* 每槽 64KB                  */
#define FW_SLOT0_BASE           (0x00000000u)     /* Slot0 = Bootloader         */
#define FW_SLOT1_BASE           (0x00010000u)     /* Slot1 = App                */
#define FW_HEAD_SIZE            (8u)              /* 槽头部字节数                */
#define FW_MAX_IMAGE_SIZE       (FW_SLOT_SIZE - FW_HEAD_SIZE)

/* 注: 参考项目 diameter_JIG 的最终代码用的是 Slot0=0x10000 / Slot1=0x20000
 * (早期误判 0x00000 掉电丢失后上移)。GD25Q128 全片擦除态无坏块, 0x00000 可用,
 * 此处按协议文档与用户确认值取 0x00000 / 0x10000。改这里即可整体迁移。      */

/* 槽头部 8 字节布局 (从槽基址开始) */
#define FW_HEAD_OFF_SIZE        (0u)              /* [0..3] 文件大小 u32 小端   */
#define FW_HEAD_OFF_VER         (4u)              /* [4..6] 版本号 主.次.修订   */
#define FW_HEAD_OFF_FLAG        (7u)              /* [7]    完整标志            */

#define FW_FLAG_COMPLETE        (0xA5u)           /* 完整                       */
#define FW_FLAG_ERASED          (0xFFu)           /* 擦除态 (=不完整)           */

/*==============================================================================
 * 四、被烧 FOC 板目标 MCU 的绝对地址 (STM32F103C8, 64KB Flash)
 *============================================================================*/
#define DUT_BOOT_ADDR           (0x08000000u)     /* Bootloader 起始            */
#define DUT_BOOT_SIZE           (0x5000u)         /* Bootloader 分配区 20KB      */
#define DUT_APP_ADDR            (0x08005000u)     /* App 起始 (固定, 不按 bin 推算) */
#define DUT_APP_SIZE            (0xA000u)         /* App 分配区 40KB            */
#define DUT_GO_ADDR             (0x08000000u)     /* 烧完跳转地址                */

/*==============================================================================
 * 五、缓冲与时序
 *============================================================================*/
#define APP_PAGE_SIZE           (256u)            /* SPI Flash 页大小            */
#define FRAME_MAX_DATA          (256u)            /* 单帧 DATA 上限 (协议 1~256) */
#define APP_FRAME_BUF_SIZE      (8u + FRAME_MAX_DATA + 2u)
#define APP_UPLOAD_TIMEOUT_MS   (10000u)          /* 上传 10s 无新数据超时        */
#define APP_TX_BUSY_TIMEOUT_MS  (50u)             /* USB CDC 发送忙等待上限       */

/*==============================================================================
 * 六、编译期不变量自检
 *----------------------------------------------------------------------------
 * 用 C89/C99 通用的"负数组长度"技巧 (ARMCC 5 与 gcc 均支持)。
 * 目的: 以后有人改上面的宏时, 一旦破坏布局约束就在编译期报错,
 *       而不是等到运行期出现"擦错扇区/写越界/帧缓冲溢出"。
 *============================================================================*/
#define FOC_STATIC_ASSERT(cond, name) typedef char foc_static_assert_##name[(cond) ? 1 : -1]

FOC_STATIC_ASSERT(APP_PAGE_SIZE == 256u,                       page_size_must_be_256);
FOC_STATIC_ASSERT(FW_HEAD_SIZE == 8u,                          head_must_be_8_bytes);
FOC_STATIC_ASSERT((FW_SLOT_SIZE % 4096u) == 0u,                slot_must_be_sector_aligned);
FOC_STATIC_ASSERT((FW_SLOT0_BASE % 4096u) == 0u,               slot0_must_be_sector_aligned);
FOC_STATIC_ASSERT((FW_SLOT1_BASE % 4096u) == 0u,               slot1_must_be_sector_aligned);
FOC_STATIC_ASSERT((FW_SLOT0_BASE + FW_SLOT_SIZE) <= FW_SLOT1_BASE, slots_must_not_overlap);
FOC_STATIC_ASSERT(FW_MAX_IMAGE_SIZE > 0u,                      max_image_size_positive);
FOC_STATIC_ASSERT(APP_FRAME_BUF_SIZE >= (8u + FRAME_MAX_DATA + 2u), frame_buf_must_fit_frame);
FOC_STATIC_ASSERT(FRAME_MAX_DATA <= APP_PAGE_SIZE,             one_frame_must_fit_one_page);
FOC_STATIC_ASSERT((FRAME_MAX_DATA % 4u) == 0u,                 frame_len_must_be_word_multiple);
FOC_STATIC_ASSERT(DUT_BOOT_SIZE > 0u,                          dut_boot_size_positive);
FOC_STATIC_ASSERT(DUT_APP_SIZE > 0u,                           dut_app_size_positive);
FOC_STATIC_ASSERT((DUT_BOOT_ADDR + DUT_BOOT_SIZE) == DUT_APP_ADDR, dut_partitions_must_be_contiguous);
FOC_STATIC_ASSERT((DUT_BOOT_SIZE % 4u) == 0u,                  dut_boot_size_word_aligned);
FOC_STATIC_ASSERT((DUT_APP_ADDR % 4u) == 0u,                   dut_app_addr_word_aligned);
FOC_STATIC_ASSERT(DUT_GO_ADDR == DUT_BOOT_ADDR,                go_addr_must_be_boot_base);

#endif /* __FLASH_MAP_H */
