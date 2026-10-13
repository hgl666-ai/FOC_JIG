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
    l->last_tick = HAL_GetTick();       /* 半截帧判定用 (docs/13 §3.5) */

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
 * 从机：把 l->resp 里的 outlen 字节组帧发出去，并记入幂等缓存
 * （EVENT_GET / ARM / RESET 这些"链路层自己处理"的应答也走这里）
 *----------------------------------------------------------------------------*/
static void link_slave_send(link_t *l, uint16_t cmd, uint16_t seq, uint16_t outlen)
{
    uint16_t total = Frame_Build(l->tx_raw, cmd, seq, l->resp, outlen);

    if (UartPort_Write(l->port, l->tx_raw, total) != total) {
        l->st.tx_nospace++;             /* 发送队列满: 应答丢了, 主机会重传 */
        return;
    }
    l->st.slave_reply++;
    l->idem_seq   = seq;                /* 应答已发出 → 缓存下来供重传复用 */
    l->idem_cmd   = cmd;
    l->idem_len   = outlen;
    if (outlen > 0u) {
        memcpy(l->idem, l->resp, outlen);
    }
    l->idem_valid = 1u;
}

/*------------------------------------------------------------------------------
 * 一帧收全之后的处理
 *----------------------------------------------------------------------------*/
static void link_deliver(link_t *l, uint16_t cmd, uint16_t seq,
                         const uint8_t *data, uint16_t len)
{
    l->st.rx_frames++;

    /* ── 主机：U2 主动事件帧 (N=0) —— 特殊分支, 不影响正在等待的请求 ──
     * 帧序号 0 与"应答"天然区分得开, 所以即使 U1 正在等应答也不会误配。 */
    if ((l->is_master != 0u) && (cmd == LINK_CMD_EVENT) && (seq == 0u)) {
        if (len >= LINK_EVENT_LEN) {
            l->rx_ev.code  = data[0];
            l->rx_ev.sev   = data[1];
            l->rx_ev.arg0  = (uint16_t)(((uint16_t)data[2] << 8) | data[3]);
            l->rx_ev.arg1  = (uint16_t)(((uint16_t)data[4] << 8) | data[5]);
            l->rx_ev.count = (uint16_t)(((uint16_t)data[6] << 8) | data[7]);
            l->rx_ev_ready = 1u;
            l->st.rx_event++;
        } else {
            l->st.rx_len_err++;
        }
        return;
    }

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

    /* ── 从机：重复请求（主机重传）→ 重发上次应答，不重新执行 (§3.4) ──
     * 这一条是 ARM 能安全重传的前提：重传不能把采集窗口重开一次。 */
    if ((l->idem_valid != 0u) && (seq == l->idem_seq) && (cmd == l->idem_cmd)) {
        uint16_t total = Frame_Build(l->tx_raw, cmd, seq, l->idem, l->idem_len);
        if (UartPort_Write(l->port, l->tx_raw, total) == total) {
            l->st.slave_reply++;
            l->st.slave_dup++;
        } else {
            l->st.tx_nospace++;
        }
        return;
    }
    l->idem_valid = 0u;             /* 新请求：先让缓存失效，避免留下半截缓存 */

    /* ── 从机：EVENT_GET 由链路层自己处理（事件表在这儿, 不麻烦应用层） ──
     * 取最早一条待决事件, 并把它的位清掉；参数与累计次数保留。 */
    if (cmd == LINK_CMD_EVENT_GET) {
        uint8_t i;
        uint8_t code = 0u;

        for (i = 0u; i < (uint8_t)LINK_EV_SLOTS; i++) {
            if ((l->ev_pending & (uint16_t)(1u << i)) != 0u) {
                code = (uint8_t)(i + 1u);
                break;
            }
        }
        l->resp[0] = LINK_ST_OK;
        memset(&l->resp[1], 0, (size_t)(LINK_EVENT_GET_LEN - 1u));
        if (code != 0u) {
            const link_event_t *e = &l->ev[code - 1u];

            l->resp[1] = e->code;
            l->resp[2] = e->sev;
            l->resp[3] = (uint8_t)(e->arg0 >> 8);
            l->resp[4] = (uint8_t)(e->arg0);
            l->resp[5] = (uint8_t)(e->arg1 >> 8);
            l->resp[6] = (uint8_t)(e->arg1);
            l->resp[7] = (uint8_t)(e->count >> 8);
            l->resp[8] = (uint8_t)(e->count);
            l->ev_pending &= (uint16_t)~(uint16_t)(1u << (code - 1u));   /* 取即清 */
        }
        link_slave_send(l, cmd, seq, LINK_EVENT_GET_LEN);
        return;
    }

    /* ── 从机：ARM —— 校验 + 幂等 + 状态机（都在链路层, 应用层只管开窗） ── */
    if (cmd == LINK_CMD_ARM) {
        uint16_t session = 0u, window = 0u;
        uint8_t  flags   = 0u;
        uint8_t  st      = LINK_ST_OK;

        if (len >= LINK_ARM_REQ_LEN) {
            session = (uint16_t)(((uint16_t)data[0] << 8) | data[1]);
            window  = (uint16_t)(((uint16_t)data[2] << 8) | data[3]);
            flags   = data[4];
        }
        if ((len < LINK_ARM_REQ_LEN) || (session == 0u) ||
            (window < LINK_ARM_WIN_MIN) || (window > LINK_ARM_WIN_MAX) ||
            (flags != 0u)) {                        /* flags 全 0 才合法(硬触发本版不支持) */
            st = LINK_ST_ERR_PARAM;
        } else if ((session == l->arm_session) && (l->arm_state != (uint8_t)LINK_ARM_IDLE)) {
            st = LINK_ST_OK;                        /* 同一 session: 不重开窗(幂等) */
        } else if (l->arm_state == (uint8_t)LINK_ARM_ARMED) {
            st = LINK_ST_BUSY;                      /* 当前窗口还在跑 */
        } else {
            l->arm_session   = session;
            l->arm_window_ms = window;
            l->arm_state     = (uint8_t)LINK_ARM_ARMED;
            l->arm_pending   = 1u;                  /* 通知应用层开窗 */
        }
        l->resp[0] = st;
        l->resp[1] = (uint8_t)(session >> 8);
        l->resp[2] = (uint8_t)(session);
        link_slave_send(l, cmd, seq, LINK_ARM_RESP_LEN);
        return;
    }

    /* ── 从机：LINK_RESET —— 先把 OK 发回去, 再清会话 ──
     * 清的是"会话态"（收发环 / 半截帧 / 等待 / ARM / 幂等缓存）；
     * **事件表与统计不清** —— 它们是故障证据 (docs/13 §3.6)。 */
    if (cmd == LINK_CMD_RESET) {
        l->resp[0] = LINK_ST_OK;
        link_slave_send(l, cmd, seq, 1u);
        UartPort_FlushRx(l->port);
        Link_Reset(l);
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
        link_slave_send(l, cmd, seq, outlen);
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
    /* 半截帧：收了几个字节就没下文了 → 丢掉，别让它污染下一帧 (docs/13 §3.5)。
     * 对端发到一半掉线时，没有这条规则的话那截字节会一直挂到下一帧 CRC 失败。 */
    if ((l->rx_cnt > 0u) &&
        ((uint32_t)(HAL_GetTick() - l->last_tick) > LINK_HALF_FRAME_MS)) {
        link_resync(l);
        l->st.rx_half++;
    }
    link_drain_rx(l);
}

uint8_t Link_RequestStart(link_t *l, uint16_t cmd,
                          const uint8_t *in, uint16_t inlen)
{
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

    /* 请求帧存进 req_raw：既要发出去，也要留一份供重传 (§3.4) */
    l->req_len = Frame_Build(l->req_raw, cmd, l->wait_seq, in, inlen);
    if (UartPort_Write(l->port, l->req_raw, l->req_len) != l->req_len) {
        l->st.tx_nospace++;
        return 0u;
    }
    l->st.tx_frames++;
    l->retried = 0u;
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

    /* timeout_ms == 0 时这里必然成立 → "只轮询一遍"的语义, 不需要特判。
     * 注意：这种"只看一眼"的调用**不重传** —— 调用方明确表示不等，
     * 重传等于替他继续等，语义就乱了（这条有专门的回归断言盯着）。 */
    if ((uint32_t)(HAL_GetTick() - l->req_t0) >= timeout_ms) {
        if ((timeout_ms > 0u) && (l->retried < LINK_RETRY_MAX)) {  /* 先重传（同序号, §3.4） */
            l->retried++;
            l->st.tx_retry++;
            if (UartPort_Write(l->port, l->req_raw, l->req_len) == l->req_len) {
                l->req_t0 = HAL_GetTick();
                return 0u;                      /* 继续等, 还没算超时 */
            }
            l->st.tx_nospace++;
        }
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
    l->retried    = 0u;
    l->idem_valid = 0u;         /* 会话复位 → 幂等缓存也失效 */
    l->arm_pending = 0u;        /* 未开出的 ARM 通知作废 */
    l->arm_state   = (uint8_t)LINK_ARM_IDLE;
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
                         const link_stats_t *ls, const uart_stats_t *us,
                         uint16_t caps, uint16_t ev_pending, uint16_t ev_overflow)
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
    out[22] = (uint8_t)(caps >> 8);               /* 能力位图 (§7.2) */
    out[23] = (uint8_t)(caps);
    out[24] = (uint8_t)(ev_pending >> 8);         /* 待决事件位图 */
    out[25] = (uint8_t)(ev_pending);
    out[26] = (uint8_t)(ev_overflow >> 8);        /* 丢弃的事件条数 */
    out[27] = (uint8_t)(ev_overflow);
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
    out->caps        = (uint16_t)(((uint16_t)in[22] << 8) | in[23]);
    out->ev_pending  = (uint16_t)(((uint16_t)in[24] << 8) | in[25]);
    out->ev_overflow = (uint16_t)(((uint16_t)in[26] << 8) | in[27]);
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

/*==============================================================================
 * 事件 (docs/13 §5.7)
 *============================================================================*/
void Link_SlaveEvent(link_t *l, uint8_t code, uint8_t sev, uint16_t arg0, uint16_t arg1)
{
    uint8_t       ev_raw[LINK_EVENT_LEN];
    link_event_t *e;
    uint16_t      total;

    if ((l == 0) || (l->inited == 0u) || (l->is_master != 0u)) {
        return;                                 /* 事件是"从机报给主机"的单向通道 */
    }
    if ((code < 1u) || (code > (uint8_t)LINK_EV_SLOTS)) {
        l->ev_overflow++;                       /* 认不出的码: 只计数, 不进表 */
        return;
    }

    e = &l->ev[code - 1u];
    e->code  = code;
    e->sev   = sev;
    e->arg0  = arg0;
    e->arg1  = arg1;
    e->count = (uint16_t)((e->count < 0xFFFFu) ? (e->count + 1u) : 0xFFFFu);
    l->ev_pending |= (uint16_t)(1u << (code - 1u));

    /* 快路径: 立刻主动发一帧 EVENT(N=0), 不需要应答。
     * 发不出去也不丢事件 —— 位图还置着, 主机每 2.5s 的 STATUS 会看到并补拉。 */
    ev_raw[0] = code;
    ev_raw[1] = sev;
    ev_raw[2] = (uint8_t)(arg0 >> 8);
    ev_raw[3] = (uint8_t)(arg0);
    ev_raw[4] = (uint8_t)(arg1 >> 8);
    ev_raw[5] = (uint8_t)(arg1);
    ev_raw[6] = (uint8_t)(e->count >> 8);
    ev_raw[7] = (uint8_t)(e->count);

    total = Frame_Build(l->tx_raw, LINK_CMD_EVENT, 0u, ev_raw, LINK_EVENT_LEN);
    if (UartPort_Write(l->port, l->tx_raw, total) != total) {
        l->st.tx_nospace++;
    }
}

uint16_t Link_SlaveEventPending(const link_t *l)
{
    return (l != 0) ? l->ev_pending : 0u;
}

uint16_t Link_SlaveEventOverflow(const link_t *l)
{
    return (l != 0) ? l->ev_overflow : 0u;
}

uint8_t Link_TakeEvent(link_t *l, link_event_t *out)
{
    if ((l == 0) || (l->rx_ev_ready == 0u)) {
        return 0u;
    }
    if (out != 0) {
        *out = l->rx_ev;
    }
    l->rx_ev_ready = 0u;
    return 1u;
}

/*==============================================================================
 * ARM 同步触发 (docs/13 §5.5)
 *============================================================================*/
uint16_t Link_ArmPackReq(uint8_t *out, uint16_t max, uint16_t session,
                         uint16_t window_ms, uint8_t flags)
{
    if ((out == 0) || (max < LINK_ARM_REQ_LEN)) {
        return 0u;
    }
    out[0] = (uint8_t)(session >> 8);
    out[1] = (uint8_t)(session);
    out[2] = (uint8_t)(window_ms >> 8);
    out[3] = (uint8_t)(window_ms);
    out[4] = flags;
    return LINK_ARM_REQ_LEN;
}

uint8_t Link_ArmCheckResp(const uint8_t *resp, uint16_t len, uint16_t session)
{
    if ((resp == 0) || (len < LINK_ARM_RESP_LEN) || (resp[0] != LINK_ST_OK)) {
        return 0u;
    }
    return (((uint16_t)(((uint16_t)resp[1] << 8) | resp[2]) == session) ? 1u : 0u);
}

uint8_t Link_SlaveArmTake(link_t *l, uint16_t *window_ms)
{
    if ((l == 0) || (l->arm_pending == 0u)) {
        return 0u;
    }
    l->arm_pending = 0u;
    if (window_ms != 0) {
        *window_ms = l->arm_window_ms;
    }
    return 1u;
}

void Link_SlaveArmDone(link_t *l)
{
    if ((l != 0) && (l->arm_state == (uint8_t)LINK_ARM_ARMED)) {
        l->arm_state = (uint8_t)LINK_ARM_READY;     /* 结果就绪, 等 U1 来取 */
    }
}

uint8_t Link_SlaveArmState(const link_t *l)
{
    return (l != 0) ? l->arm_state : (uint8_t)LINK_ARM_IDLE;
}

/*==============================================================================
 * 主机侧链路复位 (docs/13 §3.6)：断链恢复时由 U1 在空闲状态下调用
 *============================================================================*/
void Link_MasterReset(link_t *l)
{
    if ((l == 0) || (l->is_master == 0u)) {
        return;
    }
    UartPort_FlushRx(l->port);
    Link_Reset(l);
}
