/**
  ******************************************************************************
  * @file    meas_pack.c
  * @brief   测量数据定长线格式的编解码实现（布局说明见 meas_pack.h）
  *
  * 线格式逐字节定义（大端）—— 快照 55B：
  *   [0]      flags      MEAS_SNAP_F_*
  *   [1]      ch_count   = 9
  *   [2..3]   round      采集回合号 u16
  *   [4..5]   dt_ms      距上一回合 ms u16
  *   [6..7]   vdda_mv    本回合使用的 VDDA u16
  *   [8..9]   cfg_crc    配置表指纹 u16
  *   [10 + 5*i ..] 第 i 路（i = 信号序号 0..8）：
  *            raw u16 | mv u16 | flags u8
  *
  * 交叉校验块 21B：
  *   [0]      cc_flags   MEAS_CC_F_*
  *   [1..2]   mask       超差掩码 u16（bit0..8）
  *   [3 + 2*i ..] 第 i 路对侧 mv u16
  ******************************************************************************
  */
#include "meas_pack.h"

/* 布局常量与内存里那份"通道数"必须一致；不一致就是改布局忘了改宏 */
typedef char meas_pack_assert_snap_len[
    ((10u + (5u * (uint16_t)MEAS_CH_COUNT)) == MEAS_PACK_SNAP_LEN) ? 1 : -1];
typedef char meas_pack_assert_cc_len[
    ((3u + (2u * (uint16_t)MEAS_CH_COUNT)) == MEAS_PACK_CC_LEN) ? 1 : -1];
typedef char meas_pack_assert_resp_len[
    ((1u + MEAS_PACK_SNAP_LEN + MEAS_PACK_CC_LEN) == MEAS_PACK_RESP_LEN) ? 1 : -1];

/* 快照 55B 必须装得进链路单帧（LINK_MAX_PAYLOAD 见 uart_link.h） */
typedef char meas_pack_assert_fits_link[
    (MEAS_PACK_SNAP_LEN <= MEAS_PACK_LINK_MAX_PAYLOAD) ? 1 : -1];

/* 77B 应答必须装得进 PC 侧单帧 DATA（FRAME_MAX_DATA = 256） */
typedef char meas_pack_assert_fits_frame[
    (MEAS_PACK_RESP_LEN <= MEAS_PACK_FRAME_MAX_DATA) ? 1 : -1];

/* ------------------------------------------------------------------ */
static void put_u16(uint8_t *p, uint16_t v)
{
    p[0] = (uint8_t)(v >> 8);
    p[1] = (uint8_t)(v);
}

static uint16_t get_u16(const uint8_t *p)
{
    return (uint16_t)(((uint16_t)p[0] << 8) | p[1]);
}

/* ------------------------------------------------------------------ */
uint16_t MeasPack_Snapshot(uint8_t *out, uint16_t max, const meas_snapshot_t *s)
{
    uint8_t i;

    if ((out == 0) || (s == 0) || (max < MEAS_PACK_SNAP_LEN)) {
        return 0u;
    }
    out[0] = s->flags;
    out[1] = s->ch_count;
    put_u16(&out[2], s->round);
    put_u16(&out[4], s->dt_ms);
    put_u16(&out[6], s->vdda_mv);
    put_u16(&out[8], s->cfg_crc);

    for (i = 0u; i < (uint8_t)MEAS_CH_COUNT; i++) {
        uint8_t *p = &out[10u + (5u * (uint16_t)i)];
        put_u16(&p[0], s->ch[i].raw);
        put_u16(&p[2], s->ch[i].mv);
        p[4] = s->ch[i].flags;
    }
    return MEAS_PACK_SNAP_LEN;
}

uint8_t MeasPack_UnpackSnapshot(const uint8_t *in, uint16_t len, meas_snapshot_t *s)
{
    uint8_t i;

    if ((in == 0) || (s == 0) || (len < MEAS_PACK_SNAP_LEN)) {
        return 0u;
    }
    s->flags    = in[0];
    s->ch_count = in[1];
    s->round    = get_u16(&in[2]);
    s->dt_ms    = get_u16(&in[4]);
    s->vdda_mv  = get_u16(&in[6]);
    s->cfg_crc  = get_u16(&in[8]);

    for (i = 0u; i < (uint8_t)MEAS_CH_COUNT; i++) {
        const uint8_t *p = &in[10u + (5u * (uint16_t)i)];
        s->ch[i].raw   = get_u16(&p[0]);
        s->ch[i].mv    = get_u16(&p[2]);
        s->ch[i].flags = p[4];
    }
    return 1u;
}

uint16_t MeasPack_CcBlock(uint8_t *out, uint16_t max,
                          uint8_t cc_flags, uint16_t mask,
                          const meas_snapshot_t *peer)
{
    uint8_t i;

    if ((out == 0) || (max < MEAS_PACK_CC_LEN)) {
        return 0u;
    }
    out[0] = cc_flags;
    put_u16(&out[1], mask);
    for (i = 0u; i < (uint8_t)MEAS_CH_COUNT; i++) {
        uint16_t mv = (peer != 0) ? peer->ch[i].mv : 0u;
        put_u16(&out[3u + (2u * (uint16_t)i)], mv);
    }
    return MEAS_PACK_CC_LEN;
}

uint8_t MeasPack_UnpackCcBlock(const uint8_t *in, uint16_t len,
                               uint8_t *cc_flags, uint16_t *mask,
                               meas_snapshot_t *peer)
{
    uint8_t i;

    if ((in == 0) || (len < MEAS_PACK_CC_LEN)) {
        return 0u;
    }
    if (cc_flags != 0) {
        *cc_flags = in[0];
    }
    if (mask != 0) {
        *mask = get_u16(&in[1]);
    }
    for (i = 0u; i < (uint8_t)MEAS_CH_COUNT; i++) {
        uint16_t mv = get_u16(&in[3u + (2u * (uint16_t)i)]);
        if (peer != 0) {
            peer->ch[i].mv = mv;
        }
    }
    return 1u;
}
