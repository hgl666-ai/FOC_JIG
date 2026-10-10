/**
  ******************************************************************************
  * @file    pulse_pack.c
  * @brief   PWM 时序结果定长线格式的实现（布局说明见 pulse_pack.h）
  ******************************************************************************
  */
#include "pulse_pack.h"

/* 布局与常量必须自洽：改布局忘改宏会直接编译失败 */
typedef char pulse_pack_assert_len[
    ((6u + (13u * (uint16_t)PULSE_PH_COUNT)) == PULSE_PACK_LEN) ? 1 : -1];
typedef char pulse_pack_assert_fits_link[
    (PULSE_PACK_LEN <= PULSE_PACK_LINK_MAX_PAYLOAD) ? 1 : -1];
typedef char pulse_pack_assert_fits_frame[
    (PULSE_PACK_RESP_LEN <= PULSE_PACK_FRAME_MAX_DATA) ? 1 : -1];

/* ------------------------------------------------------------------ */
static void put_u16(uint8_t *p, uint16_t v)
{
    p[0] = (uint8_t)(v >> 8);
    p[1] = (uint8_t)(v);
}

static void put_u32(uint8_t *p, uint32_t v)
{
    p[0] = (uint8_t)(v >> 24);
    p[1] = (uint8_t)(v >> 16);
    p[2] = (uint8_t)(v >> 8);
    p[3] = (uint8_t)(v);
}

static uint16_t get_u16(const uint8_t *p)
{
    return (uint16_t)(((uint16_t)p[0] << 8) | p[1]);
}

static uint32_t get_u32(const uint8_t *p)
{
    return ((uint32_t)p[0] << 24) | ((uint32_t)p[1] << 16) |
           ((uint32_t)p[2] << 8)  | (uint32_t)p[3];
}

/* ------------------------------------------------------------------ */
uint16_t PulsePack_Pack(uint8_t *out, uint16_t max, const pulse_snapshot_t *s)
{
    uint8_t i;

    if ((out == 0) || (max < PULSE_PACK_LEN)) {
        return 0u;
    }
    out[0] = 1u;                                            /* ver */
    out[1] = (s != 0) ? s->flags : PULSE_SNAP_F_NOSIGNAL;
    put_u16(&out[2], (s != 0) ? s->window_ms : 0u);
    out[4] = (uint8_t)PULSE_PH_COUNT;
    out[5] = 0u;                                            /* rsvd */

    for (i = 0u; i < (uint8_t)PULSE_PH_COUNT; i++) {
        uint8_t *p = &out[6u + (13u * (uint16_t)i)];

        put_u32(&p[0], (s != 0) ? s->ph[i].period_ns : 0u);
        put_u32(&p[4], (s != 0) ? s->ph[i].high_ns   : 0u);
        put_u16(&p[8], (s != 0) ? s->ph[i].duty_permille : 0u);
        put_u16(&p[10], (s != 0) ? s->ph[i].edges : 0u);
        p[12] = (s != 0) ? s->ph[i].flags : PULSE_F_NOSIGNAL;
    }
    return PULSE_PACK_LEN;
}

uint8_t PulsePack_Unpack(const uint8_t *in, uint16_t len, pulse_snapshot_t *s)
{
    uint8_t i;

    if ((in == 0) || (s == 0) || (len < PULSE_PACK_LEN)) {
        return 0u;
    }
    s->flags       = in[1];
    s->window_ms   = get_u16(&in[2]);
    s->phase_count = (uint8_t)PULSE_PH_COUNT;

    for (i = 0u; i < (uint8_t)PULSE_PH_COUNT; i++) {
        const uint8_t *p = &in[6u + (13u * (uint16_t)i)];

        s->ph[i].period_ns     = get_u32(&p[0]);
        s->ph[i].high_ns       = get_u32(&p[4]);
        s->ph[i].duty_permille = get_u16(&p[8]);
        s->ph[i].edges         = get_u16(&p[10]);
        s->ph[i].flags         = p[12];
    }
    return 1u;
}

/* ------------------------------------------------------------------ */
uint8_t PulsePack_Compare(const pulse_snapshot_t *out_side,
                          const pulse_snapshot_t *in_side,
                          uint8_t *diff)
{
    uint8_t i;
    uint8_t ndiff = 0u;

    if (diff != 0) {
        for (i = 0u; i < (uint8_t)PULSE_PH_COUNT; i++) {
            diff[i] = 0u;
        }
    }
    if ((out_side == 0) || (in_side == 0)) {
        return 0u;
    }

    for (i = 0u; i < (uint8_t)PULSE_PH_COUNT; i++) {
        const pulse_result_t *a = &out_side->ph[i];    /* 输出端 */
        const pulse_result_t *b = &in_side->ph[i];     /* 输入端 */
        uint8_t bad = 0u;
        uint8_t a_ok = ((a->flags & PULSE_F_VALID) != 0u) ? 1u : 0u;
        uint8_t b_ok = ((b->flags & PULSE_F_VALID) != 0u) ? 1u : 0u;

        if (a_ok != b_ok) {
            bad = 1u;                                  /* 一端有一端没有 = 通路问题 */
        } else if ((a_ok != 0u) && (b_ok != 0u)) {
            /* 两端都有信号 → 比周期（频率必须一致，这是强判据） */
            uint32_t pa = a->period_ns;
            uint32_t pb = b->period_ns;
            uint32_t d  = (pa > pb) ? (pa - pb) : (pb - pa);
            uint64_t lim = ((uint64_t)pb * (uint64_t)PULSE_PEER_TOL_PPM) / 1000000u;

            if ((pb == 0u) || ((uint64_t)d > lim)) {
                bad = 1u;
            }
        }
        if (bad != 0u) {
            ndiff++;
            if (diff != 0) {
                diff[i] = 1u;
            }
        }
    }
    return ndiff;
}
