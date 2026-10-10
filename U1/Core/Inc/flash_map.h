/**
  ******************************************************************************
  * @file    flash_map.h
  * @brief   FOC 烧录治具 — 命令码 / 状态码 / Flash 分区 / 目标板地址 定义
  * @note    
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
#define MGR_CMD_MEASURE         0x000Eu   /* 测量 ADC:   请求 1B mode, 应答 77B (meas_pack.h) */
#define MGR_CMD_PULSE           0x000Fu   /* 脉冲捕获:   请求 1B mode, 应答 91B (pulse_pack.h) */
#define MGR_CMD_PROGRAM         0x0112u   /* 烧录Flash固件到FOC板  (LEN=0)       */

/* 0x000E / 0x000F 请求里共用的 mode 字节（两条命令语义一致，各自独立） */
#define MGR_MODE_READ           0x00u   /* 只读最近一批结果, 不触发测量(调试用) */
#define MGR_MODE_TEST           0x01u
/* 兼容旧写法（0x000E 早期版本用的名字） */
#define MGR_MEAS_MODE_READ      MGR_MODE_READ
#define MGR_MEAS_MODE_TEST      MGR_MODE_TEST   /* 触发一次测量: 跑完判定才回 ACK       */

/*==============================================================================
 * 二、通用状态码
 *============================================================================*/
#define APP_STATUS_OK               0x00u   /* 成功                          */
#define APP_STATUS_BUSY             0x01u   /* 系统忙 (非 IDLE 收到命令)      */
#define APP_STATUS_ERR_SIZE         0x02u   /* 文件大小异常                  */
#define APP_STATUS_ERR_FLASH        0x03u   /* SPI Flash 读写失败            */
#define APP_STATUS_ERR_PROG         0x04u   /* FOC 板烧录失败 (SWD)           */
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
 * 四、被烧 FOC 板目标 MCU 的绝对地址 (STM32F405RGT6, 1MB Flash)
 *----------------------------------------------------------------------------
 * ⚠ 分区地址是**占位值**，等 FOC 工程的链接脚本确定后替换。
 *
 * 与旧 F1 方案最大的区别：F405 的扇区是
 *     16K,16K,16K,16K,64K,128K,128K,128K KB（扇区 0..7）
 * **分区边界必须落在扇区边界上**，否则擦一个分区会连带擦掉另一个
 * （F1/F0 是 1KB 页，怎么切都行；F4 不行）。
 *============================================================================*/
#define DUT_BOOT_ADDR           (0x08000000u)     /* 扇区 0 起 */
#define DUT_BOOT_SIZE           (0x8000u)         /* 32KB = 扇区 0+1 (16K+16K) */
#define DUT_APP_ADDR            (0x08008000u)     /* 扇区 2 起 */
#define DUT_APP_SIZE            (0xF8000u)        /* 992KB = 扇区 2..7 到片尾 (0x08100000) */
#define DUT_GO_ADDR             (0x08000000u)     /* 烧完复位运行地址 */

/* 被烧板运行态固件版本号在 App 分区中的偏移（0x000D 用）——
 * ⚠ 占位值，由 FOC 工程决定；SWD 可以直接读回来，不需要被烧板配合。 */
#define DUT_VER_OFFSET          (0x20u)

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

/* F405 扇区边界约束（改分区地址时最容易踩的就是这条）：
 *   扇区边界 = 0x00000,0x04000,0x08000,0x0C000,0x10000,0x20000,0x40000,0x60000
 *   这里把当前占位方案钉死：Boot = 扇区 0+1，App = 扇区 2 起到片尾。 */
FOC_STATIC_ASSERT(DUT_BOOT_ADDR == 0x08000000u,                dut_boot_must_start_at_sector0);
FOC_STATIC_ASSERT(DUT_BOOT_SIZE == 0x8000u,                    dut_boot_must_be_sector0_and_1);
FOC_STATIC_ASSERT(DUT_APP_ADDR == 0x08008000u,                 dut_app_must_start_at_sector2);
FOC_STATIC_ASSERT((DUT_APP_ADDR + DUT_APP_SIZE) == 0x08100000u, dut_app_must_end_at_flash_end);
FOC_STATIC_ASSERT((DUT_VER_OFFSET % 4u) == 0u,                 dut_ver_offset_word_aligned);

#endif /* __FLASH_MAP_H */
