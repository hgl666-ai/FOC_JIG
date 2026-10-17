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
/*── DUT 链路（被烧 FOC 板）: 协议见 docs/15, 命令空间 0x0200~0x02FF ──*/
#define MGR_CMD_DUT_RAW         0x0011u   /* DUT 透传: 请求 cmd(2)+data, 应答 status(1)+data */
                                          /* 调试/上板用: PC 可以直接发任意 DUT 命令并看原样应答 */
/*── ⑤ 编码器脉冲发生（治具自己发，不依赖 DUT）──*/
#define MGR_CMD_ENC_GEN         0x0012u   /* 请求 12B = cnt_a/b/z + freq_a/b/z（各 2B 大端）,
                                           * 或 LEN=0 用默认参数（A=1000@5kHz/B=2000@10kHz/Z=1）
                                           * 应答 9B = 状态(1)+完成掩码(1)+超时位(1)+三路翻转数(2×3) */
#define MGR_ENC_GEN_REQ_LEN     12u       /* 请求长度（LEN=0 也算合法，表示用默认） */
#define MGR_ENC_GEN_RESP_LEN    9u        /* 应答长度 */
#define MGR_CMD_ENC_PEER        0x0013u   /* ⑤ 回环：向 U2 要编码器线边沿计数
                                           * 请求 1B = 动作（0 清零 / 1 读取），转成内部链路
                                           *       的 ENC_XCHK(0x0109)；
                                           * 应答 1B jig 状态 + U2 的原始应答（清零 1B / 读取 15B） */
#define MGR_ENC_PEER_TIMEOUT_MS 50u       /* 等 U2 应答的上限（115200 下往返 < 2ms） */
/*── ⑥ 相电流检测：一条命令跑完整流程（治具侧闭环，需要 DUT 实现 CURRENT/GET_ADC）──*/
#define MGR_CMD_CUR_TEST        0x0014u   /* 请求 8B 或 0B（用默认）：
                                           *   hi_mask(1) + lo_mask(1) + duty_permille(2)
                                           *   + duration_ms(2) + ch_mask(1) + rsvd(1)
                                           * 应答 22B 报告 = status(1)+dut_st(1)+ch_count(1)+rsvd(1)
                                           *   + 每路 9B × 2：flags(1)+jig_idle(2)+jig_on(2)
                                           *                    +dut_idle(2)+dut_on(2) */
#define MGR_CUR_TEST_REQ_LEN    8u
#define MGR_CUR_TEST_RESP_LEN   22u       /* 4 + 9×2 */
#define MGR_CUR_TIMEOUT_MS      100u      /* 每个 DUT 往返的上限 */
#define MGR_CUR_SETTLE_MS       20u       /* 通流之后再等一会儿（电流稳定 + DUT 快照更新） */
/*── ② 电压检测：治具 9 路 vs 被测板自己报的 ──*/
#define MGR_CMD_VOL_TEST        0x0015u   /* 请求 LEN=0（不用参数）
                                           * 流程：问 DUT 的 INFO（拿能力位图）→ 按位图问 GET_ADC
                                           *       → 与治具 9 路逐通道判定 → 报告
                                           * 应答 71B = 头 8B + 每路 7B × 9
                                           *   头: status(1)+dut_st(1)+dut_ver(2)+adc_mask(2)+sum(2)
                                           *   每路: flags(1)+jig_mv(2)+dut_mv(2)+err_mv(2, 有符号) */
#define MGR_VOL_TEST_RESP_LEN   71u       /* 8 + 7×9 */
#define MGR_VOL_CH_ITEM_LEN     7u
/*── ④ 三相输出判据：对**已有的**测量/脉冲结果做判定，不需要新的采集流程 ──
 * 用法（PC 侧）：先 0x0011 透传 DUT_PWM 让被测板输出已知 PWM → 0x000F mode=1 开窗采集
 *              → 再发本命令判定。治具不需要自己串流程，PC 按顺序发即可。 */
#define MGR_CMD_PHASE_JUDGE     0x0016u   /* 请求 7B = mode(1) + duty_a/b/c(各 2B, ‰ 大端)
                                           *   mode: 0 = 静态电平（不查脉冲）, 1 = PWM（查脉冲）
                                           *   duty: 0xFFFF = 该相高阻 → 不判
                                           * 应答 34B = 状态(1)+汇总(2)+相数(1) + 每相 10B
                                           *   每相: flags(2) + expect_mv(2) + err_mv(2, 有符号)
                                           *         + tol_mv(2) + duty_pm(2) */
#define MGR_PHASE_JUDGE_REQ_LEN  7u
#define MGR_PHASE_JUDGE_RESP_LEN 34u      /* 4 + 10×3 */
#define MGR_PHASE_ITEM_LEN       10u
#define MGR_PHASE_MODE_STATIC    0u
#define MGR_PHASE_MODE_PWM       1u
#define MGR_PHASE_DUTY_HIZ       0xFFFFu  /* 请求里 duty 填这个 = 该相不判 */

/*── ⑤ 的判定层：三方对齐（治具意图 / U2 实测 / DUT 上报）──
 * 请求 38B 一次把"三个数 × A/B/Z"全带上（判据层是纯函数，不缓存任何状态，
 * 上位机手里本来就有这三组数：0x0012 的翻转数、0x0013 的 U2 计数、0x0011 透传的 DUT 计数）。 */
#define MGR_CMD_ENC_JUDGE       0x0018u   /* 越界守卫: 见 APP_EncJudgeCmd 的 len 校验
                                           * 应答 29B = 状态(1)+汇总(2)+路数(1)+U2丢沿(2)
                                           *            +DUT方向(1)+保留(1) + 3 路 × 7B */
#define MGR_ENC_JUDGE_REQ_LEN     38u     /* exp_a/b/z(6) + tol(2) + u2_a/b/z(12) + u2_lost(2)
                                           *   + dut_a/b/z(12) + dut_dir(1) + 可用标志(1) + rsvd(2) */
#define MGR_ENC_JUDGE_RESP_LEN    29u     /* 8 + 3×7 */
#define MGR_ENC_JUDGE_ITEM_LEN    7u      /* flags(1)+exp(2)+u2(2)+dut(2)，两端饱和到 u16 */
#define MGR_ENC_JUDGE_AV_U2       0x01u   /* 请求里的"可用标志"：有 U2 数据 */
#define MGR_ENC_JUDGE_AV_DUT      0x02u   /* 请求里的"可用标志"：有 DUT 数据 */

/*── 治具自检 / 在线状态（上位机"设备连接"页用）──
 * 只读、无副作用、不依赖 JIG_DUT_LINK，任何时候都能问。 */
#define MGR_CMD_JIG_STATUS      0x0017u   /* 请求 LEN=0；应答 32B，见 APP_JigStatusCmd */
#define MGR_JIG_STATUS_RESP_LEN   32u
#define MGR_JIG_ST_U2_ONLINE      0x01u   /* flags bit0：U2 在线 */
#define MGR_JIG_ST_U2_VER_SAME    0x02u   /* flags bit1：两片链路代码版本一致 */
#define MGR_JIG_ST_LINK_BUSY      0x04u   /* flags bit2：内部链路正忙 */
#define MGR_JIG_ST_DUT_LINK       0x08u   /* flags bit3：本固件编进了 DUT 链路（JIG_DUT_LINK=1） */

/*── 一键全流程：① 进测试模式 → ② 电压 → ③④ 脉冲+相电压 → ⑤ 编码器 → ⑥ 电流 → STOP ──
 * 中途每一步的应答**照常回给 PC**（上位机可以选择收下或忽略），
 * 最后再回一条本命令的汇总。详见 docs/17。 */
#define MGR_CMD_RUN_ALL         0x0010u   /* 请求 LEN=0；应答 29B */
#define MGR_RUN_ALL_RESP_LEN      29u     /* 4 + 12×2 + 1 */
#define MGR_RUN_ALL_STEP_COUNT    12u
#define MGR_RUN_ALL_STEP_LEN      2u      /* 每步：状态(1) + 结果码/摘要(1) */
#define MGR_RUN_ST_IDLE           0x00u   /* 这一步没跑（被跳过） */
#define MGR_RUN_ST_OK             0x01u
#define MGR_RUN_ST_FAIL           0x02u
#define MGR_RUN_ST_SKIP           0x03u   /* 数据不全/依赖缺失，未判 */
#define MGR_RUN_ST_TIMEOUT        0x04u

/*── 单项"一条命令跑完"：把"进测试模式 → 激励 → 采集 → 判定 → 关输出"打包 ──
 * 目的（用户要求）：③ 原来要 3 条、④ 要 4 条、⑤ 要 6 条，现在各一条。
 * 应答直接复用对应单项报告的线格式，上位机不用写两套解析：
 *   0x0019(③) 应答 = 0x000F 的 91B    （U1 45B + U2 45B）
 *   0x001A(④) 应答 = 0x0016 的 34B    （相电压判定）
 *   0x001B(⑤) 应答 = 0x0018 的 29B    （三方对齐判定）
 * 流程中途**不再把子命令的应答发给 PC**（只回最后那一条），所以真的"一条命令一条应答"。
 * 参数 LEN=0 全部用默认值；也可以带参数覆盖（见各命令说明）。 */
#define MGR_CMD_PULSE_TEST      0x0019u   /* ③ 一条命令 */
#define MGR_CMD_PHASE_TEST      0x001Au   /* ④ 一条命令 */
#define MGR_CMD_ENC_TEST        0x001Bu   /* ⑤ 一条命令 */
#define MGR_PULSE_TEST_RESP_LEN   91u     /* = PULSE_PACK_RESP_LEN */
#define MGR_PHASE_TEST_RESP_LEN   34u     /* = MGR_PHASE_JUDGE_RESP_LEN */
#define MGR_ENC_TEST_RESP_LEN     29u     /* = MGR_ENC_JUDGE_RESP_LEN */
#define MGR_FLOW_STEP_TIMEOUT_MS  4000u   /* 流程里每一步的超时上限 */
#define MGR_FLOW_WDT_MS           4000u   /* 给 DUT 的失联看门狗（比单步超时略长） */

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
