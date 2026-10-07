/**
  ******************************************************************************
  * @file    uart_link.c
  * @brief   治具内部链路 (U1 <-> U2) 实现 — 逐字节收帧 + 请求/应答
  *
  * 上下文约定 (与 uart_port.c 一致)：
  *   中断：只把字节塞进 uart_port 的接收环。
  *   主循环：Link_Poll / Link_Request 搬数据、组帧、解帧、回帧。
  *   没有任何临界区——link_t 只被主循环触碰。
  ******************************************************************************
  */
#include "uart_link.h"
#include "protocol_frame.h"
#include <string.h>

/* 一次 Link_Poll 最多从接收环搬多少批 (每批 32B)。
 * 必须封顶：如果对端不停发，ISR 会一直填，无上限的 while 会卡死在主循环里。 */
#define LINK_DRAIN_BATCHES  16u

static void link_deliver(link_t *l, uint16_t cmd, uint16_t seq,
                         const uint8_t *data, uint16_t len);

static void link_resync(link_t *l)
{
    l->rx_cnt  = 0u;
    l->rx_need = 0u;
}

/*------------------------------------------------------------------------------
 * 逐字节收帧：A5 5B | CMD(2) | SEQ(2) | LEN(2) | DATA | CRC16(2)
 *----------------------------------------------------------------------------*/
static void link_feed_byte(link_t *l, uint8_t b)
{
    /* 1) 找帧头 */
    if (l->rx_cnt == 0u) {
        if (b == FRAME_HDR0) {
            l->rx_raw[0] = b;
            l->rx_cnt = 1u;
        } else {
            l->st.rx_resync++;
        }
        return;
    }
    if (l->rx_cnt == 1u) {
        if (b == FRAME_HDR1) {
            l->rx_raw[1] = b;
            l->rx_cnt = 2u;
        } else if (b == FRAME_HDR0) {
            l->st.rx_resync++;          /* 还是 0xA5: 保持在"已收 1 字节" */
        } else {
            l->st.rx_resync++;
            link_resync(l);
        }
        return;
    }

    /* 2) 固定头 6 字节 (CMD/SEQ/LEN) */
    if (l->rx_cnt < 8u) {
        l->rx_raw[l->rx_cnt] = b;
        l->rx_cnt++;
        if (l->rx_cnt == 8u) {
            uint16_t dlen = (uint16_t)(((uint16_t)l->rx_raw[6] << 8) | l->rx_raw[7]);
            if (dlen > LINK_MAX_PAYLOAD) {
                /* 长度非法：这一帧已经不可信，直接丢同步。剩下的数据里如果还有
                 * 完整帧，重新找 A5 5B 就能恢复 (只损失帧内偶发的 0xA5)。 */
                l->st.rx_len_err++;
                link_resync(l);
                return;
            }
            l->rx_need = (uint16_t)(8u + dlen + 2u);
        }
        return;
    }

    /* 3) DATA + CRC */
    l->rx_raw[l->rx_cnt] = b;
    l->rx_cnt++;
    if (l->rx_cnt >= l->rx_need) {
        uint16_t cmd = 0u, seq = 0u, dlen = 0u;

        if (Frame_Parse(l->rx_raw, l->rx_cnt, &cmd, &seq, l->rx_data, &dlen)) {
            link_deliver(l, cmd, seq, l->rx_data, dlen);
        } else {
            l->st.rx_crc_err++;
        }
        link_resync(l);
    }
}

/*------------------------------------------------------------------------------
 * 一帧收全之后的处理
 *----------------------------------------------------------------------------*/
static void link_deliver(link_t *l, uint16_t cmd, uint16_t seq,
                         const uint8_t *data, uint16_t len)
{
    l->st.rx_frames++;

    /* ── 主机：只认"当前在等的那一发"的应答 ── */
    if (l->is_master != 0u) {
        if ((l->waiting != 0u) && (seq == l->wait_seq)) {
            l->resp_cmd  = cmd;
            l->resp_len  = len;
            if ((len > 0u) && (data != 0)) {
                memcpy(l->resp, data, len);
            }
            l->resp_ready = 1u;
        } else {
            l->st.rx_stale++;           /* 陈旧/无关帧：丢弃，不动当前请求 */
        }
        return;
    }

    /* ── 从机：交应用层，拿到应答就回帧 (SEQ 原样回显，与 PC 侧 ACK 规则一致) ── */
    if (l->slave_fn == 0) {
        l->st.slave_drop++;
        return;
    }

    {
        uint16_t outlen = 0u;
        uint8_t  ok     = l->slave_fn(cmd, data, len, l->resp, &outlen);

        if ((ok == 0u) || (outlen > LINK_MAX_PAYLOAD)) {
            l->st.slave_drop++;
            return;
        }
        {
            uint16_t total = Frame_Build(l->tx_raw, cmd, seq, l->resp, outlen);
            if (UartPort_Write(l->port, l->tx_raw, total) == total) {
                l->st.slave_reply++;
            } else {
                l->st.tx_nospace++;     /* 发送队列满: 应答丢了，主机那边会超时 */
            }
        }
    }
}

/* 把接收环里的数据搬空并逐字节喂进状态机 (有上限，见 LINK_DRAIN_BATCHES) */
static void link_drain_rx(link_t *l)
{
    uint8_t  tmp[32];
    uint16_t n;
    uint8_t  round;

    for (round = 0u; round < LINK_DRAIN_BATCHES; round++) {
        n = UartPort_Read(l->port, tmp, (uint16_t)sizeof(tmp));
        if (n == 0u) {
            return;
        }
        for (uint16_t i = 0u; i < n; i++) {
            link_feed_byte(l, tmp[i]);
        }
    }
}

/*==============================================================================
 * API
 *============================================================================*/
void Link_Init(link_t *l, uart_id_t port, uint8_t is_master)
{
    if (l == 0) {
        return;
    }
    memset(l, 0, sizeof(*l));
    l->port      = port;
    l->is_master = (is_master != 0u) ? 1u : 0u;
    l->req_seq   = 0u;                  /* 第一发从 1 开始 */
    l->inited    = 1u;
}

void Link_SetSlaveHandler(link_t *l, link_slave_fn_t fn)
{
    if (l != 0) {
        l->slave_fn = fn;
    }
}

void Link_Poll(link_t *l)
{
    if ((l == 0) || (l->inited == 0u)) {
        return;
    }
    link_drain_rx(l);
}

uint8_t Link_RequestStart(link_t *l, uint16_t cmd,
                          const uint8_t *in, uint16_t inlen)
{
    uint16_t total;

    if ((l == 0) || (l->inited == 0u) || (l->is_master == 0u)) {
        return 0u;
    }
    if (inlen > LINK_MAX_PAYLOAD) {
        return 0u;
    }
    if (l->waiting != 0u) {             /* 上一次还没回来: 直接拒绝，不排队 */
        l->st.tx_busy++;
        return 0u;
    }

    l->req_seq++;
    if (l->req_seq == 0u) {
        l->req_seq = 1u;                /* 0 保留给"治具主动推送"事件帧 */
    }
    l->wait_seq   = l->req_seq;
    l->resp_ready = 0u;
    l->resp_len   = 0u;

    /* 请求帧借用 tx_raw：主机不会走"从机应答"分支，两边不会同时用 */
    total = Frame_Build(l->tx_raw, cmd, l->wait_seq, in, inlen);
    if (UartPort_Write(l->port, l->tx_raw, total) != total) {
        l->st.tx_nospace++;
        return 0u;
    }
    l->st.tx_frames++;
    l->req_t0  = HAL_GetTick();
    l->waiting = 1u;
    return 1u;
}

uint8_t Link_RequestPoll(link_t *l, uint32_t timeout_ms,
                         uint8_t *out, uint16_t *outlen)
{
    if (outlen != 0) {
        *outlen = 0u;
    }
    if ((l == 0) || (l->waiting == 0u)) {
        return 2u;                      /* 没有进行中的请求 */
    }

    Link_Poll(l);

    if (l->resp_ready != 0u) {
        l->waiting    = 0u;
        l->resp_ready = 0u;
        /* out 的容量约定为 >= LINK_MAX_PAYLOAD (见头文件), 所以直接整段拷 ——
         * 不做"按 *outlen 截断"是因为那个语义要求调用方先把长度当容量填进去,
         * 忘了填就会静默拷 0 字节(踩过一次)。 */
        if (out != 0) {
            memcpy(out, l->resp, l->resp_len);
        }
        if (outlen != 0) {
            *outlen = l->resp_len;
        }
        return 1u;
    }

    /* timeout_ms == 0 时这里必然成立 → "只轮询一遍"的语义, 不需要特判 */
    if ((uint32_t)(HAL_GetTick() - l->req_t0) >= timeout_ms) {
        l->waiting = 0u;
        l->st.tx_timeout++;
        return 2u;
    }
    return 0u;
}

uint8_t Link_Request(link_t *l, uint16_t cmd,
                     const uint8_t *in, uint16_t inlen,
                     uint8_t *out, uint16_t *outlen, uint32_t timeout_ms)
{
    uint8_t st;

    if (outlen != 0) {
        *outlen = 0u;
    }
    if (Link_RequestStart(l, cmd, in, inlen) == 0u) {
        return 0u;
    }
    for (;;) {
        st = Link_RequestPoll(l, timeout_ms, out, outlen);
        if (st == 1u) {
            return 1u;
        }
        if (st != 0u) {                 /* 2 = 超时 */
            return 0u;
        }
    }
}

uint8_t Link_Busy(const link_t *l)
{
    return ((l != 0) && (l->waiting != 0u)) ? 1u : 0u;
}

void Link_Reset(link_t *l)
{
    if (l == 0) {
        return;
    }
    link_resync(l);
    l->waiting    = 0u;
    l->resp_ready = 0u;
    l->resp_len   = 0u;
}

const link_stats_t *Link_Stats(const link_t *l)
{
    return (l != 0) ? &l->st : 0;
}

/*==============================================================================
 * LINK_CMD_STATUS 编解码
 *============================================================================*/
static void put_u32_be(uint8_t *p, uint32_t v)
{
    p[0] = (uint8_t)(v >> 24);
    p[1] = (uint8_t)(v >> 16);
    p[2] = (uint8_t)(v >> 8);
    p[3] = (uint8_t)(v);
}

static uint32_t get_u32_be(const uint8_t *p)
{
    return ((uint32_t)p[0] << 24) | ((uint32_t)p[1] << 16)
         | ((uint32_t)p[2] << 8)  | (uint32_t)p[3];
}

uint16_t Link_PackStatus(uint8_t *out, uint16_t max,
                         const uint8_t ver[2], uint16_t alive,
                         const link_stats_t *ls, const uart_stats_t *us)
{
    if ((out == 0) || (max < LINK_STATUS_LEN)) {
        return 0u;
    }
    out[0] = (ver != 0) ? ver[0] : 0u;
    out[1] = (ver != 0) ? ver[1] : 0u;
    out[2] = (uint8_t)(LINK_CODE_VER >> 8);       /* 链路代码版本 */
    out[3] = (uint8_t)(LINK_CODE_VER);
    out[4] = (uint8_t)(alive >> 8);
    out[5] = (uint8_t)(alive);
    put_u32_be(&out[6],  (ls != 0) ? ls->rx_frames  : 0u);
    put_u32_be(&out[10], (ls != 0) ? ls->rx_crc_err : 0u);
    put_u32_be(&out[14], (us != 0) ? us->rx_bytes   : 0u);
    put_u32_be(&out[18], (us != 0) ? us->rx_drop    : 0u);
    return LINK_STATUS_LEN;
}

uint8_t Link_UnpackStatus(const uint8_t *in, uint16_t len, link_status_t *out)
{
    if ((in == 0) || (out == 0) || (len < LINK_STATUS_LEN)) {
        return 0u;
    }
    out->ver_major  = in[0];
    out->ver_minor  = in[1];
    out->code_ver = (uint16_t)(((uint16_t)in[2] << 8) | in[3]);
    out->alive      = (uint16_t)(((uint16_t)in[4] << 8) | in[5]);
    out->rx_frames  = get_u32_be(&in[6]);
    out->rx_crc_err = get_u32_be(&in[10]);
    out->rx_bytes   = get_u32_be(&in[14]);
    out->rx_drop    = get_u32_be(&in[18]);
    return 1u;
}

uint8_t Link_CheckVersion(const link_status_t *st)
{
    if (st == 0) {
        return 0u;
    }
    return (st->code_ver == LINK_CODE_VER) ? 1u : 0u;
}

uint16_t Link_LastRespCmd(const link_t *l)
{
    return (l != 0) ? l->resp_cmd : 0u;
}
