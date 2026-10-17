/**
  ******************************************************************************
  * @file    dut_proto.h
  * @brief   **DUT（被烧 FOC 板）通讯协议**的定长线格式编解码
  *
  * 规范：`docs/15_DUT通讯协议规格.md`（这份头文件是规范的代码侧镜像，
  *       两者不一致时**以规范为准**，并同步改这里）。
  *
  * 设计约束（与 `meas_pack.h` 同一套做法）：
  *   - **不搞 TLV**：一条命令一套固定布局，逐字节大端读写，不用 C struct 直接 memcpy；
  *   - 本模块**只有 U1 用**（U2 不跟被测板打交道）→ **不做两片副本**，不参与 fc /b 比对；
  *   - **纯函数**：不碰 HAL、不碰全局状态 → 可以在宿主机上直接测（`test/dut_proto_test.c`）；
  *   - 只负责 DATA 段的编解码；**帧头/SEQ/LEN/CRC 是 `protocol_frame.c` 的事**。
  *
  * 单帧 DATA 上限 64B 来自治具的链路层（`LINK_MAX_PAYLOAD`，见 `uart_link.h`），
  * 本文件里的长度常量都受这条约束（`DUT_GETADC_MAX_CH` 就是按它算出来的）。
  ******************************************************************************
  */
#ifndef __DUT_PROTO_H
#define __DUT_PROTO_H

#include <stdint.h>

/* ------------------------------------------------------------------ */
/* 1. 命令码（0x0200 ~ 0x02FF）                                        */
/* ------------------------------------------------------------------ */
#define DUT_CMD_PING        0x0200u   /* 回环自检 / 保活，应答 = 原样回显 */
#define DUT_CMD_INFO        0x0201u   /* 版本 + 能力位图                  */
#define DUT_CMD_TEST_MODE   0x0202u   /* 进入/退出测试模式（带看门狗）     */
#define DUT_CMD_STOP        0x0203u   /* 三相输出全关（安全命令）          */
#define DUT_CMD_PWM         0x0204u   /* 三相 PWM（③④）                   */
#define DUT_CMD_LEVEL       0x0205u   /* 三相静态电平（④）                */
#define DUT_CMD_CURRENT     0x0206u   /* 相电流激励（⑥）                  */
#define DUT_CMD_GET_ADC     0x0207u   /* 读 DUT 自己采到的模拟量（②⑥）    */
#define DUT_CMD_GET_ENC     0x0208u   /* 读 DUT 自己数到的编码器计数（⑤）  */

#define DUT_CMD_LO          0x0200u
#define DUT_CMD_HI          0x02FFu

/* 本协议版本（写进 INFO 应答，让治具判断对面按哪版规格实现） */
#define DUT_CODE_VER        0x0101u

/* ------------------------------------------------------------------ */
/* 2. 长度（改这些常量 = 改协议，必须同步 docs/15 与上位机）            */
/* ------------------------------------------------------------------ */
#define DUT_INFO_RESP_LEN       16u   /* 状态(1)+dut_ver(2)+fw(3)+caps(2)
                                         +pwm_min(2)+pwm_max(2)+adc_mask(2)+rsvd(2) */
#define DUT_TESTMODE_REQ_LEN     3u   /* mode(1) + wdt_ms(2)                */
#define DUT_PWM_REQ_LEN         10u   /* freq(2)+dutyA(2)+dutyB(2)+dutyC(2)+count(2) */
#define DUT_LEVEL_REQ_LEN        2u   /* level(1) + rsvd(1)                 */
#define DUT_CURRENT_REQ_LEN      8u   /* mode(1)+hi(1)+lo(1)+duty(2)+dur(2)+rsvd(1) */
#define DUT_GETADC_REQ_LEN       2u   /* ch_mask(2, 大端) —— 通道有 0..8 共 9 路，
                                         1 字节只能表达 0..7，会把第 8 路漏掉 */
#define DUT_GETADC_ITEM_LEN      5u   /* ch(1) + raw(2) + mv(2)             */
#define DUT_GETENC_RESP_LEN     14u   /* 状态(1)+A(4)+B(4)+Z(4)+dir(1)       */
#define DUT_STATUS_LEN           1u   /* 只回状态码的应答                    */

/* 单帧 DATA 上限（= 治具链路层 LINK_MAX_PAYLOAD） */
#define DUT_LINK_MAX_PAYLOAD    64u
/* GET_ADC 一次最多能问几路：1(状态) + n*5 <= 64 → n <= 12 */
#define DUT_GETADC_MAX_CH       12u

/* 编译期自检（ARMCC / GCC 都能过；不成立直接编译失败） */
typedef char dut_proto_assert_pwm[(DUT_PWM_REQ_LEN == 10u) ? 1 : -1];
typedef char dut_proto_assert_cur[(DUT_CURRENT_REQ_LEN == 8u) ? 1 : -1];
typedef char dut_proto_assert_info[(DUT_INFO_RESP_LEN == 16u) ? 1 : -1];
typedef char dut_proto_assert_enc[(DUT_GETENC_RESP_LEN == 14u) ? 1 : -1];
typedef char dut_proto_assert_adc_fit[
    ((1u + DUT_GETADC_MAX_CH * DUT_GETADC_ITEM_LEN) <= DUT_LINK_MAX_PAYLOAD) ? 1 : -1];
typedef char dut_proto_assert_info_fit[(DUT_INFO_RESP_LEN <= DUT_LINK_MAX_PAYLOAD) ? 1 : -1];

/* ------------------------------------------------------------------ */
/* 3. 状态码（**与另外两条链路共用同一张表**，不要另发明取值）          */
/* ------------------------------------------------------------------ */
#define DUT_ST_OK           0x00u
#define DUT_ST_BUSY         0x01u
#define DUT_ST_ERR_PARAM    0x0Bu
#define DUT_ST_ERR_TIMEOUT  0x0Cu
#define DUT_ST_NIY          0xFFu

/* ------------------------------------------------------------------ */
/* 4. 能力位图（bit i = 支持命令 0x0200+i）                            */
/* ------------------------------------------------------------------ */
#define DUT_CAP_PING        (1u << 0)
#define DUT_CAP_INFO        (1u << 1)
#define DUT_CAP_TEST_MODE   (1u << 2)
#define DUT_CAP_STOP        (1u << 3)
#define DUT_CAP_PWM         (1u << 4)
#define DUT_CAP_LEVEL       (1u << 5)
#define DUT_CAP_CURRENT     (1u << 6)
#define DUT_CAP_GET_ADC     (1u << 7)
#define DUT_CAP_GET_ENC     (1u << 8)
/* 必做项：链路与安全相关，任何实现都不能省 */
#define DUT_CAP_MANDATORY   (DUT_CAP_PING | DUT_CAP_INFO | DUT_CAP_TEST_MODE | DUT_CAP_STOP)
#define DUT_CAP_ALL         0x01FFu

/* ------------------------------------------------------------------ */
/* 5. 枚举 / 参数范围                                                  */
/* ------------------------------------------------------------------ */
/* TEST_MODE 的 mode */
#define DUT_TESTMODE_EXIT    0u
#define DUT_TESTMODE_ENTER   1u
/* CURRENT 的 mode */
#define DUT_CUR_MODE_STATIC  0u      /* 静态通流（⑥ 推荐） */
#define DUT_CUR_MODE_PWM     1u      /* 低频 PWM 通流       */
/* LEVEL 每相 2bit 编码 */
#define DUT_LVL_LOW          0u
#define DUT_LVL_HIGH         1u
#define DUT_LVL_HIZ          2u      /* 高阻/关断（默认安全态） */
/* 看门狗范围（ms）—— 不允许关掉 */
#define DUT_WDT_MIN_MS       200u
#define DUT_WDT_MAX_MS       10000u
/* 占空比用千分比 */
#define DUT_DUTY_MAX         1000u
/* 相掩码：bit0=A bit1=B bit2=C */
#define DUT_PHASE_A          0x01u
#define DUT_PHASE_B          0x02u
#define DUT_PHASE_C          0x04u
#define DUT_PHASE_MASK_ALL   0x07u

/* ------------------------------------------------------------------ */
/* 6. ADC 通道编号 —— **故意与治具 9 路信号序号完全一致**               */
/*    （治具侧定义见 docs/07 §2.1；这样 PC 拿到两边数据可逐通道直比）    */
/* ------------------------------------------------------------------ */
#define DUT_ADC_24V          0u
#define DUT_ADC_10V          1u
#define DUT_ADC_5V           2u
#define DUT_ADC_33V          3u
#define DUT_ADC_PH_A         4u
#define DUT_ADC_PH_B         5u
#define DUT_ADC_PH_C         6u
#define DUT_ADC_CUR1         7u
#define DUT_ADC_CUR2         8u
#define DUT_ADC_COUNT        9u
#define DUT_ADC_MASK_ALL     0x01FFu   /* bit0..8 */

/* ------------------------------------------------------------------ */
/* 7. 结构体（内存表示，与线格式无关）                                  */
/* ------------------------------------------------------------------ */
typedef struct {
    uint8_t  status;
    uint16_t dut_ver;        /* 协议版本，应为 DUT_CODE_VER */
    uint8_t  fw_ver[3];      /* [主][次][修订] */
    uint16_t caps;
    uint16_t pwm_min_hz;
    uint16_t pwm_max_hz;
    uint16_t adc_mask;       /* bit i = 支持通道 i */
} dut_info_t;

typedef struct {
    uint16_t freq_hz;
    uint16_t duty_a;         /* 千分比 0..1000 */
    uint16_t duty_b;
    uint16_t duty_c;
    uint16_t count;          /* 0 = 连续输出到 STOP / 看门狗 */
} dut_pwm_t;

typedef struct {
    uint8_t  mode;           /* DUT_CUR_MODE_* */
    uint8_t  hi_mask;        /* 上桥相掩码，只允许 1 位 */
    uint8_t  lo_mask;        /* 下桥相掩码，必须与 hi 不同相 */
    uint16_t duty_permille;
    uint16_t duration_ms;    /* 0 = 保持到 STOP / 看门狗 */
} dut_current_t;

typedef struct {
    uint8_t  ch;
    uint16_t raw;
    uint16_t mv;             /* **信号侧** mV（折回分压之前），见 docs/15 §7.8 */
} dut_adc_item_t;

typedef struct {
    uint8_t        status;   /* 应答里的状态码 */
    uint16_t       req_mask; /* 请求的通道掩码（bit0..8 对应 9 路） */
    uint8_t        n;        /* 实际收到的条数 */
    dut_adc_item_t item[DUT_GETADC_MAX_CH];
} dut_adc_t;

typedef struct {
    uint8_t  status;
    uint32_t cnt_a;          /* A 相累计边沿数 */
    uint32_t cnt_b;
    uint32_t cnt_z;
    int8_t   dir;            /* +1 / -1 / 0 */
} dut_enc_t;

/* ------------------------------------------------------------------ */
/* 8. 请求打包（返回写入字节数；参数或容量不对返回 0）                   */
/* ------------------------------------------------------------------ */
/* 注意：INFO / STOP / GET_ENC 三条命令的**请求 DATA 为空**（LEN=0），
 * 不需要打包函数，直接按 (in=NULL, inlen=0) 发即可。                    */
uint16_t DutPack_Ping(uint8_t *out, uint16_t max, const uint8_t *payload, uint16_t len);
uint16_t DutPack_TestMode(uint8_t *out, uint16_t max, uint8_t mode, uint16_t wdt_ms);
uint16_t DutPack_Pwm(uint8_t *out, uint16_t max, const dut_pwm_t *p);
uint16_t DutPack_Level(uint8_t *out, uint16_t max, uint8_t level_bits);
uint16_t DutPack_Current(uint8_t *out, uint16_t max, const dut_current_t *c);
uint16_t DutPack_GetAdc(uint8_t *out, uint16_t max, uint16_t ch_mask);

/* ------------------------------------------------------------------ */
/* 9. 应答解包（返回 1 = 成功, 0 = 长度/参数不对）                       */
/* ------------------------------------------------------------------ */
/** 解"只回状态码"的应答（TEST_MODE/STOP/PWM/LEVEL/CURRENT 都是这种） */
uint8_t DutUnpack_Status(const uint8_t *in, uint16_t len, uint8_t *status);
/** 解 INFO 应答 */
uint8_t DutUnpack_Info(const uint8_t *in, uint16_t len, dut_info_t *out);
/** 解 GET_ADC 应答；want_mask 传请求时用的掩码，会核对"要的通道是不是都来齐了" */
uint8_t DutUnpack_Adc(const uint8_t *in, uint16_t len, uint16_t want_mask, dut_adc_t *out);
/** 解 GET_ENC 应答 */
uint8_t DutUnpack_Enc(const uint8_t *in, uint16_t len, dut_enc_t *out);

/* ------------------------------------------------------------------ */
/* 10. 参数校验（**纯逻辑**，治具侧先用它"预检"再发，DUT 侧用同一套判据） */
/* ------------------------------------------------------------------ */
/**
  * @brief  PWM 参数是否可受理
  * @param  fmin/fmax 对端 INFO 里声明的频率范围（传 0,0 表示不检查范围）
  * @retval 1 = 合法, 0 = 非法（应回 DUT_ST_ERR_PARAM）
  */
uint8_t DutCheck_Pwm(const dut_pwm_t *p, uint16_t fmin, uint16_t fmax);

/**
  * @brief  相电流激励参数是否可受理
  * @note   三条硬规则：hi/lo 各只能 1 位；两者**不能同相**（否则同桥臂直通）；
  *         掩码只能出现在 A/B/C 三位内。
  * @retval 1 = 合法, 0 = 非法
  */
uint8_t DutCheck_Current(const dut_current_t *c);

/**
  * @brief  TEST_MODE 参数是否可受理（看门狗必须落在 [200,10000] ms）
  */
uint8_t DutCheck_TestMode(uint8_t mode, uint16_t wdt_ms);

/**
  * @brief  LEVEL 位图是否合法（保留位必须为 0，且每相取值不能是 11）
  */
uint8_t DutCheck_Level(uint8_t level_bits);

/* ------------------------------------------------------------------ */
/* 11. 小工具                                                          */
/* ------------------------------------------------------------------ */
/** 把三相电平编码成 LEVEL 请求里的 1 字节位图（入参取 DUT_LVL_*） */
uint8_t DutLevel_Pack(uint8_t a, uint8_t b, uint8_t c);
/** 从位图里取出某一相（phase 取 DUT_PHASE_A/B/C）的 2bit 电平值 */
uint8_t DutLevel_Get(uint8_t bits, uint8_t phase);
/** caps 里是否声明支持某条命令 */
uint8_t DutCaps_Has(uint16_t caps, uint16_t cmd);
/** 通道掩码里置 1 的位数（GET_ADC 用它算应答长度） */
uint8_t DutMask_Count(uint16_t mask);

#endif /* __DUT_PROTO_H */
