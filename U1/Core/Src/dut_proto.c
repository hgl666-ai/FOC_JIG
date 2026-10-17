/**
  ******************************************************************************
  * @file    dut_proto.c
  * @brief   DUT（被烧 FOC 板）通讯协议的定长线格式编解码
  *
  * 规范：`docs/15_DUT通讯协议规格.md`
  * 纯函数，不碰 HAL / 全局状态 → 宿主单测见 `test/dut_proto_test.c`。
  *
  * 全部逐字节大端读写，**不用 C struct 直接 memcpy**：
  * 结构体有填充和对齐差异，链路上必须一个字节一个字节写死。
  ******************************************************************************
  */
#include "dut_proto.h"

/* ------------------------------------------------------------------ */
/* 大端读写                                                            */
/* ------------------------------------------------------------------ */
static void put_u16(uint8_t *p, uint16_t v)
{
    p[0] = (uint8_t)(v >> 8);
    p[1] = (uint8_t)(v & 0xFFu);
}

static uint16_t get_u16(const uint8_t *p)
{
    return (uint16_t)(((uint16_t)p[0] << 8) | (uint16_t)p[1]);
}

static uint32_t get_u32(const uint8_t *p)
{
    return ((uint32_t)p[0] << 24) | ((uint32_t)p[1] << 16) |
           ((uint32_t)p[2] << 8)  | (uint32_t)p[3];
}

/* ------------------------------------------------------------------ */
/* 请求打包                                                            */
/* ------------------------------------------------------------------ */
uint16_t DutPack_Ping(uint8_t *out, uint16_t max, const uint8_t *payload, uint16_t len)
{
    uint16_t i;

    if (len > DUT_LINK_MAX_PAYLOAD || max < len) {
        return 0u;
    }
    if (len > 0u && (out == 0 || payload == 0)) {
        return 0u;
    }
    for (i = 0u; i < len; i++) {
        out[i] = payload[i];
    }
    return len;
}

uint16_t DutPack_TestMode(uint8_t *out, uint16_t max, uint8_t mode, uint16_t wdt_ms)
{
    if (out == 0 || max < DUT_TESTMODE_REQ_LEN) {
        return 0u;
    }
    out[0] = mode;
    put_u16(&out[1], wdt_ms);
    return DUT_TESTMODE_REQ_LEN;
}

uint16_t DutPack_Pwm(uint8_t *out, uint16_t max, const dut_pwm_t *p)
{
    if (out == 0 || p == 0 || max < DUT_PWM_REQ_LEN) {
        return 0u;
    }
    put_u16(&out[0], p->freq_hz);
    put_u16(&out[2], p->duty_a);
    put_u16(&out[4], p->duty_b);
    put_u16(&out[6], p->duty_c);
    put_u16(&out[8], p->count);
    return DUT_PWM_REQ_LEN;
}

uint16_t DutPack_Level(uint8_t *out, uint16_t max, uint8_t level_bits)
{
    if (out == 0 || max < DUT_LEVEL_REQ_LEN) {
        return 0u;
    }
    out[0] = level_bits;
    out[1] = 0u;                       /* rsvd 必须为 0 */
    return DUT_LEVEL_REQ_LEN;
}

uint16_t DutPack_Current(uint8_t *out, uint16_t max, const dut_current_t *c)
{
    if (out == 0 || c == 0 || max < DUT_CURRENT_REQ_LEN) {
        return 0u;
    }
    out[0] = c->mode;
    out[1] = c->hi_mask;
    out[2] = c->lo_mask;
    put_u16(&out[3], c->duty_permille);
    put_u16(&out[5], c->duration_ms);
    out[7] = 0u;                       /* rsvd 必须为 0 */
    return DUT_CURRENT_REQ_LEN;
}

uint16_t DutPack_GetAdc(uint8_t *out, uint16_t max, uint16_t ch_mask)
{
    if (out == 0 || max < DUT_GETADC_REQ_LEN) {
        return 0u;
    }
    put_u16(&out[0], ch_mask);
    return DUT_GETADC_REQ_LEN;
}

/* ------------------------------------------------------------------ */
/* 应答解包                                                            */
/* ------------------------------------------------------------------ */
uint8_t DutUnpack_Status(const uint8_t *in, uint16_t len, uint8_t *status)
{
    if (in == 0 || status == 0 || len < DUT_STATUS_LEN) {
        return 0u;
    }
    *status = in[0];
    return 1u;
}

uint8_t DutUnpack_Info(const uint8_t *in, uint16_t len, dut_info_t *out)
{
    if (in == 0 || out == 0 || len < DUT_INFO_RESP_LEN) {
        return 0u;
    }
    out->status     = in[0];
    out->dut_ver    = get_u16(&in[1]);
    out->fw_ver[0]  = in[3];
    out->fw_ver[1]  = in[4];
    out->fw_ver[2]  = in[5];
    out->caps       = get_u16(&in[6]);
    out->pwm_min_hz = get_u16(&in[8]);
    out->pwm_max_hz = get_u16(&in[10]);
    out->adc_mask   = get_u16(&in[12]);
    /* in[14..15] = rsvd，忽略 */
    return 1u;
}

uint8_t DutUnpack_Adc(const uint8_t *in, uint16_t len, uint16_t want_mask, dut_adc_t *out)
{
    uint16_t off;
    uint8_t  n, i;
    uint16_t got;

    if (in == 0 || out == 0 || len < DUT_STATUS_LEN) {
        return 0u;
    }
    out->status   = in[0];
    out->req_mask = want_mask;
    out->n        = 0u;

    /* 长度必须是 1 + k*5 */
    if (((len - DUT_STATUS_LEN) % DUT_GETADC_ITEM_LEN) != 0u) {
        return 0u;
    }
    n = (uint8_t)((len - DUT_STATUS_LEN) / DUT_GETADC_ITEM_LEN);
    if (n > DUT_GETADC_MAX_CH) {
        return 0u;
    }

    off = DUT_STATUS_LEN;
    for (i = 0u; i < n; i++) {
        out->item[i].ch  = in[off + 0u];
        out->item[i].raw = get_u16(&in[off + 1u]);
        out->item[i].mv  = get_u16(&in[off + 3u]);
        off = (uint16_t)(off + DUT_GETADC_ITEM_LEN);
    }
    out->n = n;

    /* 要的通道必须来齐，否则算失败（防止对端少给一路而调用方照旧拿下标当通道号用） */
    got = 0u;
    for (i = 0u; i < n; i++) {
        if (out->item[i].ch < 16u) {
            got = (uint16_t)(got | (uint16_t)(1u << out->item[i].ch));
        }
    }
    if ((uint16_t)(got & want_mask) != want_mask) {
        return 0u;
    }
    return 1u;
}

uint8_t DutUnpack_Enc(const uint8_t *in, uint16_t len, dut_enc_t *out)
{
    if (in == 0 || out == 0 || len < DUT_GETENC_RESP_LEN) {
        return 0u;
    }
    out->status = in[0];
    out->cnt_a  = get_u32(&in[1]);
    out->cnt_b  = get_u32(&in[5]);
    out->cnt_z  = get_u32(&in[9]);
    out->dir    = (int8_t)in[13];
    return 1u;
}

/* ------------------------------------------------------------------ */
/* 参数校验                                                            */
/* ------------------------------------------------------------------ */
uint8_t DutCheck_TestMode(uint8_t mode, uint16_t wdt_ms)
{
    if (mode != DUT_TESTMODE_EXIT && mode != DUT_TESTMODE_ENTER) {
        return 0u;
    }
    /* 看门狗不允许被关掉：退出模式时才允许不带看门狗 */
    if (mode == DUT_TESTMODE_ENTER) {
        if (wdt_ms < DUT_WDT_MIN_MS || wdt_ms > DUT_WDT_MAX_MS) {
            return 0u;
        }
    }
    return 1u;
}

uint8_t DutCheck_Pwm(const dut_pwm_t *p, uint16_t fmin, uint16_t fmax)
{
    if (p == 0) {
        return 0u;
    }
    if (p->freq_hz == 0u) {
        return 0u;
    }
    if (fmin != 0u || fmax != 0u) {
        if (p->freq_hz < fmin || (fmax != 0u && p->freq_hz > fmax)) {
            return 0u;
        }
    }
    if (p->duty_a > DUT_DUTY_MAX || p->duty_b > DUT_DUTY_MAX || p->duty_c > DUT_DUTY_MAX) {
        return 0u;
    }
    /* count 允许为 0（= 连续输出直到 STOP / 看门狗），不校验上界 */
    return 1u;
}

uint8_t DutCheck_Current(const dut_current_t *c)
{
    uint8_t hi, lo;

    if (c == 0) {
        return 0u;
    }
    if (c->mode != DUT_CUR_MODE_STATIC && c->mode != DUT_CUR_MODE_PWM) {
        return 0u;
    }
    hi = (uint8_t)(c->hi_mask & DUT_PHASE_MASK_ALL);
    lo = (uint8_t)(c->lo_mask & DUT_PHASE_MASK_ALL);

    /* 掩码不能出现 A/B/C 以外的位 */
    if (hi != c->hi_mask || lo != c->lo_mask) {
        return 0u;
    }
    /* 上下桥各只能选一相，否则就是多相并联/直通 */
    if (DutMask_Count(hi) != 1u || DutMask_Count(lo) != 1u) {
        return 0u;
    }
    /* 不能同相 —— 那就是同一个桥臂上下管直通，母线短路 */
    if (hi == lo) {
        return 0u;
    }
    if (c->duty_permille > DUT_DUTY_MAX) {
        return 0u;
    }
    /* duration_ms 允许为 0（= 保持到 STOP / 看门狗） */
    return 1u;
}

uint8_t DutCheck_Level(uint8_t level_bits)
{
    uint8_t i, v;

    /* 保留位 bit7..6 必须为 0 */
    if ((level_bits & 0xC0u) != 0u) {
        return 0u;
    }
    /* 每相取值不能是 11（保留） */
    for (i = 0u; i < 3u; i++) {
        v = (uint8_t)((level_bits >> (i * 2u)) & 0x03u);
        if (v == 3u) {
            return 0u;
        }
    }
    return 1u;
}

/* ------------------------------------------------------------------ */
/* 小工具                                                              */
/* ------------------------------------------------------------------ */
uint8_t DutLevel_Pack(uint8_t a, uint8_t b, uint8_t c)
{
    if (a > DUT_LVL_HIZ || b > DUT_LVL_HIZ || c > DUT_LVL_HIZ) {
        return 0xFFu;                  /* 非法，调用方应回 ERR_PARAM */
    }
    return (uint8_t)((a & 0x03u) | ((b & 0x03u) << 2) | ((c & 0x03u) << 4));
}

uint8_t DutLevel_Get(uint8_t bits, uint8_t phase)
{
    uint8_t sh;

    if (phase == DUT_PHASE_A) {
        sh = 0u;
    } else if (phase == DUT_PHASE_B) {
        sh = 2u;
    } else if (phase == DUT_PHASE_C) {
        sh = 4u;
    } else {
        return 0xFFu;
    }
    return (uint8_t)((bits >> sh) & 0x03u);
}

uint8_t DutCaps_Has(uint16_t caps, uint16_t cmd)
{
    uint16_t off;

    if (cmd < DUT_CMD_LO || cmd > DUT_CMD_HI) {
        return 0u;
    }
    off = (uint16_t)(cmd - DUT_CMD_LO);
    /* caps 只有 16 位 → 只能声明 0x0200..0x020F。0x0210 以上一律算"不支持"。
     * 这里必须挡住，否则 1u << 255 是未定义行为（宿主单测抓出来的）。 */
    if (off >= 16u) {
        return 0u;
    }
    return (uint8_t)(((caps & (uint16_t)(1u << off)) != 0u) ? 1u : 0u);
}

uint8_t DutMask_Count(uint16_t mask)
{
    uint8_t n = 0u;

    while (mask != 0u) {
        n = (uint8_t)(n + (uint8_t)(mask & 0x0001u));
        mask = (uint16_t)(mask >> 1);
    }
    return n;
}
