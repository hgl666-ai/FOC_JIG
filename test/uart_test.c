/*
 * UART 四路驱动 + U1<->U2 内部链路 —— 宿主侧(gcc)测试
 *
 * 编译对象是固件真源码: U1/Core/Src/uart_port.c, U1/Core/Src/uart_link.c,
 * U1/Core/Src/protocol_frame.c + 本文件里的"假 HAL"。
 *
 * 假 HAL 干了三件事(其余都交给真实固件代码):
 *   1) 两个 UART 句柄 (USART1=侧0, USART3=侧1), 模拟 HAL 的 IT 收发:
 *      收: 每次 1 字节, 收完回调 → 驱动重挂 (与真实 HAL 的 Receive_IT 语义一致)
 *      发: 记下缓冲区, 逐字节往外"推", 推完回调 TxCplt → 驱动领下一段
 *   2) 一根**交叉线**: 侧0 的 TX 接侧1 的 RX, 反之亦然 —— 和真实的 H3/H4 跳线一样
 *   3) 可选的"对侧主循环": 字节到达时调一次对侧 Link_Poll, 模拟 U2 也在跑自己的
 *      主循环并及时应答
 *
 * 覆盖点(每条都对应一个真实会踩的坑):
 *   1) 环形缓冲: 逐字节收发 / 回卷 / available
 *   2) 发送链式续传: 一次写 300B(>256 环形) 必须不丢不乱序
 *   3) 接收环满: 丢新字节 + rx_drop 计数(而不是丢最旧, 也不静默)
 *   4) 发送队列满: 发送卡住时写超量 → tx_drop 计数 + 卡住解除后自动继续
 *   5) ORE 溢出后必须重新挂接收(否则这一路永久哑掉且无任何报错)
 *   6) 链路 PING 往返 / 应答数据搬运
 *   7) 链路超时: 对侧不在 → 超时返回 + tx_timeout 计数 + 状态复位(下次还能用)
 *   8) 链路 CRC 错恢复 / 前置垃圾重同步 / 超长 LEN 拒绝
 *   9) 链路忙保护(上一发没回就再发) 与 timeout=0 的非阻塞轮询语义
 *  10) STATUS 数据区打包/解包往返(两片共用的同一份编解码)
 */
#include <stdio.h>
#include <string.h>
#include <stdint.h>

#include "uart_port.h"
#include "uart_link.h"
#include "protocol_frame.h"
#include "meas_pack.h"
#include "pulse_pack.h"      /* 55B 快照线格式（0x000E / MEAS_XCHK 共用） */

static int g_checks = 0;
static int g_fails  = 0;

#define CHECK(cond, msg)                                                     \
    do {                                                                     \
        g_checks++;                                                          \
        if (!(cond)) { printf("  FAIL: %s  (line %d)\n", msg, __LINE__); g_fails++; } \
    } while (0)

#define SECTION(t) printf("\n[%s]\n", t)

/*==============================================================================
 * 假 HAL
 *============================================================================*/
#define SIM_SIDES   2
#define SIM_SIDE_A  0       /* U1 的 USART1 = 端口0 */
#define SIM_SIDE_B  1       /* U1 的 USART3 = 端口1 (内部链路) */

/* 与 U1 固件同名的两个句柄: 本测试直接编译固件的 U1/Core/Src/uart_cfg.c,
 * 让"回调 → 端口号"的分发逻辑也走真代码, 而不是在测试里再抄一份。 */
UART_HandleTypeDef huart1;
UART_HandleTypeDef huart3;

typedef struct {
    UART_HandleTypeDef *hu;
    uint8_t            *rxp;        /* 当前接收落点 (0 = 没挂接收) */
    uint16_t            rx_size;
    const uint8_t      *txp;        /* 当前待发缓冲 */
    uint16_t            tx_size;
    uint16_t            tx_pos;
    uint8_t             tx_active;
    uint8_t             hold_tx;    /* 1 = 发送卡住: 不推进、不回调 (模拟对端不收) */
    uint32_t            tx_calls;
    uint32_t            rx_arms;
    uint32_t            tx_dropped; /* 因为没挂接收而丢掉的字节 */
} sim_side_t;

static sim_side_t s_side[SIM_SIDES];
static uint32_t   g_tick;
static uint8_t    s_wire_on = 1;
static void     (*s_peer_poll)(void);

/* 真实固件用 HAL_GetTick 做超时; 宿主里每次读都推进 1ms, 保证超时循环一定结束 */
uint32_t HAL_GetTick(void) { return g_tick++; }

static sim_side_t *sim_find(UART_HandleTypeDef *huart)
{
    for (int i = 0; i < SIM_SIDES; i++) {
        if (s_side[i].hu == huart) return &s_side[i];
    }
    return 0;
}

static sim_side_t *sim_peer(sim_side_t *s)
{
    return (s == &s_side[SIM_SIDE_A]) ? &s_side[SIM_SIDE_B] : &s_side[SIM_SIDE_A];
}

static void sim_tx_done(sim_side_t *s)
{
    /* 幂等: 推线是嵌套的(发送完成的回调里会立刻发起下一段, 再次进推线),
     * 真实 HAL 每次 Transmit_IT 只回调一次 TxCplt, 这里也必须只回调一次,
     * 否则驱动会把 tx_tail 多推一格, 后续数据整体错位。 */
    if (s->tx_active == 0u) {
        return;
    }
    s->tx_active = 0;
    HAL_UART_TxCpltCallback(s->hu);         /* → UartPort_OnTxComplete → 领下一段 */
}

static void sim_deliver(sim_side_t *dst, uint8_t b)
{
    if (dst->rxp == 0) {                    /* 对侧没挂接收: 这字节真的丢了 */
        dst->tx_dropped++;
        return;
    }
    *dst->rxp = b;
    dst->rxp  = 0;                          /* 落点已消费, 等回调里重挂 */
    HAL_UART_RxCpltCallback(dst->hu);       /* → UartPort_OnRxComplete → 重挂接收 */
    g_tick++;                               /* 115200 下约 87us/字节, 用 tick 近似 */
    if (s_peer_poll != 0) s_peer_poll();    /* 对侧 MCU 也在跑自己的主循环 */
}

static uint8_t sim_pump_side(sim_side_t *s)
{
    if ((s->tx_active == 0u) || (s->hold_tx != 0u)) {
        return 0u;                          /* 卡住的发送不参与推线 */
    }
    if (s->tx_pos < s->tx_size) {
        uint8_t b = s->txp[s->tx_pos];
        s->tx_pos++;
        if (s_wire_on != 0u) {
            sim_deliver(sim_peer(s), b);
        }
        if (s->tx_pos >= s->tx_size) {
            sim_tx_done(s);
        }
        return s->tx_active;
    }
    sim_tx_done(s);
    return 0u;
}

/* 把"线"推到静止 (发送卡住的侧跳过)。真实硬件里这段时间由移位寄存器花掉 */
void sim_pump(void)
{
    for (int guard = 0; guard < 100000; guard++) {
        uint8_t busy = 0u;
        for (int i = 0; i < SIM_SIDES; i++) {
            busy |= sim_pump_side(&s_side[i]);
        }
        if (busy == 0u) return;
    }
    printf("  !! sim_pump guard 触发(疑似死循环)\n");
    g_fails++;
}

/* ---- HAL 替身 ---- */
HAL_StatusTypeDef HAL_UART_Receive_IT(UART_HandleTypeDef *huart, uint8_t *pData, uint16_t Size)
{
    sim_side_t *s = sim_find(huart);
    if (s == 0) return HAL_ERROR;
    if (s->rxp != 0) return HAL_BUSY;       /* 真实 HAL: 正在收时返回 BUSY, 不覆盖 */
    s->rxp = pData;
    s->rx_size = Size;
    s->rx_arms++;
    return HAL_OK;
}

HAL_StatusTypeDef HAL_UART_Transmit_IT(UART_HandleTypeDef *huart, uint8_t *pData, uint16_t Size)
{
    sim_side_t *s = sim_find(huart);
    if (s == 0) return HAL_ERROR;
    if (s->tx_active != 0u) return HAL_BUSY;
    s->txp = pData;
    s->tx_size = Size;
    s->tx_pos = 0u;
    s->tx_active = 1u;
    s->tx_calls++;
    if (Size == 0u) { sim_tx_done(s); return HAL_OK; }
    sim_pump();                             /* 测试里的"线"是零延迟的 */
    return HAL_OK;
}

/* 直接往某侧的接收落点灌一个字节 (模拟"线上来了数据", 不走发送侧) */
static void sim_inject(uint8_t side, uint8_t b)
{
    sim_side_t *dst = &s_side[side];
    if (dst->rxp == 0) { dst->tx_dropped++; return; }
    *dst->rxp = b;
    dst->rxp = 0;
    HAL_UART_RxCpltCallback(dst->hu);
    g_tick++;
}

static void sim_inject_buf(uint8_t side, const uint8_t *d, uint16_t n)
{
    for (uint16_t i = 0; i < n; i++) sim_inject(side, d[i]);
}

/*==============================================================================
 * 环境搭建 / 复位
 *============================================================================*/
static void sim_setup(void)
{
    memset(s_side, 0, sizeof(s_side));
    huart1.Instance = USART1;
    huart3.Instance = USART3;
    s_side[SIM_SIDE_A].hu = &huart1;
    s_side[SIM_SIDE_B].hu = &huart3;
    g_tick = 0u;
    s_wire_on = 1u;
    s_peer_poll = 0;
}

static void ports_start(void)
{
    /* 端口号就是"两个独立实例"的意思: 侧A 用端口0, 侧B 用端口1 */
    UartPort_InitPort(UART_PORT_0, &huart1);
    UartPort_InitPort(UART_PORT_1, &huart3);
    UartPort_Start(UART_PORT_0);
    UartPort_Start(UART_PORT_1);
}

/*==============================================================================
 * 1) 环形缓冲与逐字节收发
 *============================================================================*/
static void test_ring_basic(void)
{
    uint8_t  rx[64];
    uint8_t  i;
    uint8_t  seq[16];

    SECTION("1) 环形缓冲 / 逐字节收发");
    sim_setup();
    ports_start();

    for (i = 0; i < 16u; i++) seq[i] = (uint8_t)(0x40u + i);

    CHECK(UartPort_Available(UART_PORT_0) == 0u, "空环 available=0");
    CHECK(UartPort_Write(UART_PORT_0, seq, 16u) == 16u, "写入 16B 全部入队");
    sim_pump();
    CHECK(UartPort_Available(UART_PORT_1) == 16u, "对侧收到 16B");
    CHECK(UartPort_Read(UART_PORT_1, rx, sizeof(rx)) == 16u, "读出 16B");
    CHECK(memcmp(rx, seq, 16u) == 0, "内容与顺序一致");

    /* 回卷: 反复写满再读空, 让 head/tail 都绕过 0 */
    for (i = 0; i < 40u; i++) {
        uint8_t b = (uint8_t)(0xA0u + i);
        CHECK(UartPort_Write(UART_PORT_0, &b, 1u) == 1u, "单字节入队");
        sim_pump();
        CHECK(UartPort_Read(UART_PORT_1, rx, 1u) == 1u, "单字节读出");
        CHECK(rx[0] == b, "回卷后内容仍正确");
    }

    CHECK(UartPort_Stats(UART_PORT_1)->rx_bytes == 56u, "rx_bytes 计数 = 16+40");
    CHECK(UartPort_Stats(UART_PORT_1)->rx_drop == 0u, "无丢字节");
    CHECK(s_side[SIM_SIDE_B].tx_dropped == 0u, "线上无丢字节");
}

/*==============================================================================
 * 2) 发送链式续传 (写入量 > 发送环)
 *============================================================================*/
static void test_tx_chain(void)
{
    uint8_t buf[300];
    uint8_t rx[300];
    uint16_t got = 0u, n, sent = 0u;

    SECTION("2) 发送链式续传 (300B > 256B 环)");
    sim_setup();
    ports_start();

    for (uint16_t i = 0; i < sizeof(buf); i++) buf[i] = (uint8_t)(i * 7u + 1u);

    /* 一次 Write 只能填满环形缓冲(255B), 剩下的要等发送侧排空后再补写 ——
     * 这就是驱动规定的语义: 返回值 = 实际入队字节数, 调用方负责补写。
     * 同时必须一边写一边把对侧收到的取走: 对侧的接收环也只有 255B,
     * 不取就会丢字节。这个循环里的 Read 就代表"对侧主循环在取数据"。 */
    while (sent < 300u) {
        n = UartPort_Write(UART_PORT_1, buf + sent, (uint16_t)(300u - sent));
        CHECK(n > 0u, "每次 Write 至少能进一段");
        sent = (uint16_t)(sent + n);
        sim_pump();                             /* 让发送侧排空 */
        while ((n = UartPort_Read(UART_PORT_0, rx + got,
                                  (uint16_t)(sizeof(rx) - got))) > 0u) {
            got = (uint16_t)(got + n);
            sim_pump();
        }
    }
    CHECK(sent == 300u, "300B 全部入队");
    CHECK(s_side[SIM_SIDE_B].tx_calls >= 2u, "分多段发送 (链式续传)");

    sim_pump();
    while ((n = UartPort_Read(UART_PORT_0, rx + got, (uint16_t)(sizeof(rx) - got))) > 0u) {
        got = (uint16_t)(got + n);
        sim_pump();
    }
    CHECK(got == 300u, "对侧收满 300B");
    CHECK(memcmp(buf, rx, 300u) == 0, "300B 内容与顺序一致");
    CHECK(UartPort_Stats(UART_PORT_0)->rx_drop == 0u, "无丢字节");
    CHECK(UartPort_Stats(UART_PORT_1)->rx_err == 0u, "无错误计数");
}

/*==============================================================================
 * 3) 接收环满: 丢新字节 + 计数
 *============================================================================*/
static void test_rx_overflow(void)
{
    uint8_t b = 0x5Au;
    uint32_t drop;

    SECTION("3) 接收环满 → 丢新字节并计数");
    sim_setup();
    ports_start();

    /* 没人读, 灌 300B: 环容量 255 (留一格判满), 其余必须被计数丢掉 */
    for (int i = 0; i < 300; i++) sim_inject(SIM_SIDE_A, b);

    CHECK(UartPort_Available(UART_PORT_0) == 255u, "环满时 available 封顶在 255");
    drop = UartPort_Stats(UART_PORT_0)->rx_drop;
    CHECK(drop == 45u, "rx_drop = 300-255");
    CHECK(UartPort_Stats(UART_PORT_0)->rx_bytes == 300u, "rx_bytes 仍统计全部到达字节");

    /* 读空后必须恢复正常(丢的是新字节, 不是把缓冲搞坏) */
    for (int i = 0; i < 255; i++) {
        uint8_t out;
        CHECK(UartPort_Read(UART_PORT_0, &out, 1u) == 1u, "读出残留");
        CHECK(out == b, "残留内容正确");
    }
    CHECK(UartPort_Available(UART_PORT_0) == 0u, "读空");
    sim_inject(SIM_SIDE_A, 0x11u);
    CHECK(UartPort_Available(UART_PORT_0) == 1u, "溢出后仍能继续收");
}

/*==============================================================================
 * 4) 发送队列满: 计数 + 卡住解除后继续
 *============================================================================*/
static void test_tx_overflow(void)
{
    uint8_t buf[300];
    uint8_t rx[300];
    uint16_t got = 0u, n;

    SECTION("4) 发送队列满 → 计数, 解除后继续发");
    sim_setup();
    ports_start();

    memset(buf, 0x33, sizeof(buf));
    s_side[SIM_SIDE_B].hold_tx = 1u;            /* 对侧 USART 卡住 */
    CHECK(UartPort_Write(UART_PORT_1, buf, 300u) == 255u, "只有 255B 进得了队列");
    CHECK(UartPort_Stats(UART_PORT_1)->tx_drop == 45u, "tx_drop = 300-255");
    sim_pump();
    CHECK(s_side[SIM_SIDE_A].rx_arms > 0u, "接收一直挂着");

    s_side[SIM_SIDE_B].hold_tx = 0u;            /* 卡住解除 */
    sim_pump();
    while ((n = UartPort_Read(UART_PORT_0, rx + got, (uint16_t)(sizeof(rx) - got))) > 0u) {
        got = (uint16_t)(got + n);
        sim_pump();
    }
    CHECK(got == 255u, "卡住解除后把队列里的 255B 发完");
}

/*==============================================================================
 * 5) ORE (溢出错误) 之后必须重挂接收
 *============================================================================*/
static void test_error_rearm(void)
{
    uint32_t arms_before;

    SECTION("5) ORE 错误后重新挂接收 (否则这一路永久哑掉)");
    sim_setup();
    ports_start();

    /* 模拟 F1 HAL 的 ORE 分支: UART_EndRxTransfer 之后 RxState=READY,
     * 收发中断被关掉, 也就是"没挂接收"的状态 */
    s_side[SIM_SIDE_A].rxp = 0;
    arms_before = s_side[SIM_SIDE_A].rx_arms;

    UartPort_OnError(UART_PORT_0);              /* HAL_UART_ErrorCallback 里会调它 */

    CHECK(s_side[SIM_SIDE_A].rxp != 0, "OnError 之后接收被重新挂上");
    CHECK(s_side[SIM_SIDE_A].rx_arms == arms_before + 1u, "Receive_IT 被再次调用");
    CHECK(UartPort_Stats(UART_PORT_0)->rx_err == 1u, "rx_err 计数 +1");

    /* 重挂之后必须真的能收 */
    sim_inject(SIM_SIDE_A, 0x77u);
    {
        uint8_t out = 0u;
        CHECK(UartPort_Read(UART_PORT_0, &out, 1u) == 1u, "重挂后收得到数据");
        CHECK(out == 0x77u, "内容正确");
    }
}

/*==============================================================================
 * 链路: 从机角色 (模拟 U2)
 *============================================================================*/
static link_t s_master;
static link_t s_slave;
static meas_snapshot_t s_slave_snap;   /* 从机(模拟 U2)手里的快照, 由用例填充 */

/* 与 U2 固件一致的从机处理函数 (PING 回显 / STATUS 打包 / XCHK 换快照 / 其余 NIY) */
static uint8_t slave_handler(uint16_t cmd, const uint8_t *in, uint16_t inlen,
                             uint8_t *out, uint16_t *outlen)
{
    switch (cmd) {
    case LINK_CMD_PING:
        if (inlen > LINK_MAX_PAYLOAD) return 0u;
        for (uint16_t i = 0u; i < inlen; i++) out[i] = in[i];
        *outlen = inlen;
        return 1u;
    case LINK_CMD_MEAS_XCHK: {
        /* U2 的角色: 收到 U1 的 55B 快照, 回自己的 55B 快照(判定归 U1) */
        if (inlen < MEAS_PACK_SNAP_LEN) {
            out[0]  = LINK_ST_NIY;
            *outlen = 1u;
            return 1u;
        }
        *outlen = MeasPack_Snapshot(out, LINK_MAX_PAYLOAD, &s_slave_snap);
        return (*outlen != 0u) ? 1u : 0u;
    }
    case LINK_CMD_PULSE_XCHK: {
        /* U2 的角色: 收到 U1 的 45B(输出端), 回自己的 45B(输入端) */
        pulse_snapshot_t ps;
        uint8_t i;
        if (inlen < PULSE_PACK_LEN) {
            out[0]  = LINK_ST_NIY;
            *outlen = 1u;
            return 1u;
        }
        memset(&ps, 0, sizeof(ps));
        ps.phase_count = (uint8_t)PULSE_PH_COUNT;
        ps.flags       = PULSE_SNAP_F_VALID;
        ps.window_ms   = 200u;
        for (i = 0u; i < (uint8_t)PULSE_PH_COUNT; i++) {
            ps.ph[i].period_ns     = 50000u;
            ps.ph[i].high_ns       = 25000u;
            ps.ph[i].duty_permille = 500u;
            ps.ph[i].edges         = 4000u;
            ps.ph[i].flags         = PULSE_F_VALID;
        }
        *outlen = PulsePack_Pack(out, LINK_MAX_PAYLOAD, &ps);
        return (*outlen != 0u) ? 1u : 0u;
    }
    case LINK_CMD_STATUS: {
        static const uint8_t ver[2] = { 0x00u, 0x01u };
        *outlen = Link_PackStatus(out, LINK_MAX_PAYLOAD, ver, 7u,
                                  Link_Stats(&s_slave),
                                  UartPort_Stats(UART_PORT_1),
                                  LINK_CAP_ALL, 0u, 0u);
        return (*outlen != 0u) ? 1u : 0u;
    }
    default:
        out[0]  = LINK_ST_NIY;
        *outlen = 1u;
        return 1u;
    }
}

static void slave_poll_fn(void)
{
    /* 真实硬件里对侧是另一个 MCU, 它的主循环**不可能**重入 —— 但在这个测试里,
     * "对侧回帧"又会触发一次推线, 于是又回到这里。所以用一个 guard 把重入挡掉,
     * 语义上等于"对侧把这一批处理完再来下一批"。
     * (顺带记录一条设计约束: Link_Poll 本身不可重入, 只能在主循环上下文调用。) */
    static uint8_t in_poll = 0u;

    if (in_poll != 0u) return;
    in_poll = 1u;
    Link_Poll(&s_slave);
    in_poll = 0u;
}

static void link_setup(uint8_t with_peer)
{
    sim_setup();
    ports_start();
    Link_Init(&s_master, UART_PORT_0, 1u);
    Link_Init(&s_slave,  UART_PORT_1, 0u);
    if (with_peer != 0u) {
        Link_SetSlaveHandler(&s_slave, slave_handler);
        s_peer_poll = slave_poll_fn;            /* 对侧主循环(模拟 U2) */
    }
}

/*==============================================================================
 * 6) 链路 PING 往返 + 应答数据
 *============================================================================*/
static void test_link_ping(void)
{
    uint8_t  tx[LINK_MAX_PAYLOAD];
    uint8_t  rx[LINK_MAX_PAYLOAD];
    uint16_t rxlen = 0u;
    uint8_t  i;

    SECTION("6) 链路 PING 往返 / 应答数据搬运");
    link_setup(1u);

    for (i = 0; i < 32u; i++) tx[i] = (uint8_t)(0xF0u + i);

    CHECK(Link_Request(&s_master, LINK_CMD_PING, tx, 32u, rx, &rxlen, 50u) == 1u,
          "32B PING 收到应答");
    CHECK(rxlen == 32u, "应答长度 = 请求长度");
    CHECK(memcmp(tx, rx, 32u) == 0, "回显内容一致");
    CHECK(Link_Busy(&s_master) == 0u, "请求结束后不再忙");
    CHECK(Link_Stats(&s_master)->tx_frames == 1u, "tx_frames = 1");
    CHECK(Link_Stats(&s_master)->tx_timeout == 0u, "无超时");
    CHECK(Link_Stats(&s_master)->rx_crc_err == 0u, "无 CRC 错");
    CHECK(Link_Stats(&s_slave)->slave_reply == 1u, "从机应答 1 次");

    /* 0 长度 PING: 固件里 U1 的保活探测就是这种 */
    rxlen = 123u;
    CHECK(Link_Request(&s_master, LINK_CMD_PING, 0, 0u, 0, &rxlen, 50u) == 1u,
          "0 长度 PING 也能确认收到应答");
    CHECK(rxlen == 0u, "0 长度应答 outlen=0");

    /* 连打 5 发: SEQ 递增, 每发都必须能对上新应答而不是旧缓存 */
    for (i = 0; i < 5u; i++) {
        uint8_t one = i;
        uint8_t back = 0xFFu;
        rxlen = 0u;
        CHECK(Link_Request(&s_master, LINK_CMD_PING, &one, 1u, &back, &rxlen, 50u) == 1u,
              "连续 PING 有应答");
        CHECK(back == one && rxlen == 1u, "拿到的不是上一发的旧应答");
    }
    CHECK(Link_Stats(&s_master)->rx_stale == 0u, "没有陈旧帧");
}

/*==============================================================================
 * 7) 链路超时: 对侧不在
 *============================================================================*/
static void test_link_timeout(void)
{
    uint16_t rxlen = 99u;

    SECTION("7) 链路超时 (对侧不在) + 状态可恢复");
    link_setup(0u);                             /* 不注册从机、不跑对侧主循环 */

    CHECK(Link_Request(&s_master, LINK_CMD_PING, 0, 0u, 0, &rxlen, 20u) == 0u,
          "对侧不在 → 返回失败");
    CHECK(Link_Stats(&s_master)->tx_timeout == 1u, "tx_timeout 计数 +1");
    CHECK(Link_Busy(&s_master) == 0u, "超时后必须复位忙标志(否则永久卡死)");
    CHECK(rxlen == 0u, "失败时 outlen 清 0");

    /* 超时之后把对侧接上: 必须还能正常通 */
    Link_SetSlaveHandler(&s_slave, slave_handler);
    s_peer_poll = slave_poll_fn;
    CHECK(Link_Request(&s_master, LINK_CMD_PING, 0, 0u, 0, 0, 50u) == 1u,
          "对侧恢复后仍能通信");
    CHECK(Link_Stats(&s_master)->tx_timeout == 1u, "超时计数不再增长");
}

/*==============================================================================
 * 12) 交叉校验快照交换 (MEAS_XCHK: 请求 55B / 应答 55B, 链路单帧上限 64B)
 *     s_slave_snap 在"链路: 从机角色"一节声明（从机处理函数要用）
 *============================================================================*/
static void test_meas_xchk(void)
{
    uint8_t  req[MEAS_PACK_SNAP_LEN];
    uint8_t  resp[LINK_MAX_PAYLOAD];
    uint16_t rxlen = 0u;
    meas_snapshot_t mine, peer;
    uint8_t  i;

    SECTION("12) 交叉校验快照交换 (55B 负载)");
    link_setup(1u);

    /* 造两份快照：主片标称值，从片在 5V 那一路偏 400mV（应被判定为超差） */
    memset(&mine, 0, sizeof(mine));
    mine.ch_count = (uint8_t)MEAS_CH_COUNT;
    mine.flags    = MEAS_SNAP_F_VALID | MEAS_SNAP_F_ALL_OK;
    for (i = 0u; i < (uint8_t)MEAS_CH_COUNT; i++) {
        mine.ch[i].mv  = (uint16_t)(1000u + (uint16_t)i * 100u);
        mine.ch[i].raw = (uint16_t)(200u + i);
    }
    s_slave_snap = mine;
    s_slave_snap.ch[MEAS_CH_V5].mv = (uint16_t)(mine.ch[MEAS_CH_V5].mv + 400u);
    s_slave_snap.round = 7u;

    CHECK(MeasPack_Snapshot(req, sizeof(req), &mine) == MEAS_PACK_SNAP_LEN,
          "主片 55B 快照打包成功");
    CHECK(MEAS_PACK_SNAP_LEN <= LINK_MAX_PAYLOAD, "55B 装得进链路单帧(64B)");

    CHECK(Link_Request(&s_master, LINK_CMD_MEAS_XCHK, req, MEAS_PACK_SNAP_LEN,
                       resp, &rxlen, 50u) == 1u, "MEAS_XCHK 有应答");
    CHECK(rxlen == MEAS_PACK_SNAP_LEN, "应答也是 55B");

    memset(&peer, 0, sizeof(peer));
    CHECK(MeasPack_UnpackSnapshot(resp, rxlen, &peer) == 1u, "对侧快照可解析");
    CHECK(peer.round == 7u, "对侧的回合号传过来了");
    {
        int same = 1;
        for (i = 0u; i < (uint8_t)MEAS_CH_COUNT; i++) {
            if (peer.ch[i].mv != s_slave_snap.ch[i].mv) same = 0;
        }
        CHECK(same == 1, "9 路 mv 全部逐字节一致");
    }
    CHECK(peer.ch[MEAS_CH_V5].mv != mine.ch[MEAS_CH_V5].mv, "两片在 5V 那一路确实不同");

    /* 请求太短 → 从机明确回 NIY（1 字节），而不是静默超时 */
    rxlen = 0u;
    CHECK(Link_Request(&s_master, LINK_CMD_MEAS_XCHK, req, MEAS_PACK_SNAP_LEN - 1u,
                       resp, &rxlen, 50u) == 1u, "太短的请求也有应答");
    CHECK(rxlen == 1u && resp[0] == LINK_ST_NIY, "太短的请求 → NIY");

    /* 连续 3 发 55B 往返：确认长负载不会打乱 SEQ 配对 */
    for (i = 0u; i < 3u; i++) {
        rxlen = 0u;
        CHECK(Link_Request(&s_master, LINK_CMD_MEAS_XCHK, req, MEAS_PACK_SNAP_LEN,
                           resp, &rxlen, 50u) == 1u, "连续 55B 交换仍有应答");
        CHECK(rxlen == MEAS_PACK_SNAP_LEN, "长度稳定");
    }
    CHECK(Link_Stats(&s_master)->rx_stale == 0u, "没有陈旧帧(SEQ 配对正确)");
    CHECK(Link_Stats(&s_master)->rx_crc_err == 0u, "长负载没有 CRC 错");
    CHECK(Link_Stats(&s_master)->tx_nospace == 0u, "55B 请求没有把发送队列撑爆");
}


/*==============================================================================
 * 13) 脉冲结果交换 (PULSE_XCHK: 请求 45B / 应答 45B)
 *============================================================================*/
static void test_pulse_xchk(void)
{
    uint8_t  req[PULSE_PACK_LEN];
    uint8_t  resp[LINK_MAX_PAYLOAD];
    uint16_t rxlen = 0u;
    pulse_snapshot_t mine, peer;
    uint8_t  i;

    SECTION("13) 脉冲结果交换 (45B)");
    link_setup(1u);

    memset(&mine, 0, sizeof(mine));
    mine.phase_count = (uint8_t)PULSE_PH_COUNT;
    mine.flags       = PULSE_SNAP_F_VALID;
    mine.window_ms   = 200u;
    for (i = 0u; i < (uint8_t)PULSE_PH_COUNT; i++) {
        mine.ph[i].period_ns     = 50000u;      /* 20kHz */
        mine.ph[i].high_ns       = 25000u;      /* 50% */
        mine.ph[i].duty_permille = 500u;
        mine.ph[i].edges         = 4000u;
        mine.ph[i].flags         = PULSE_F_VALID;
    }

    CHECK(PulsePack_Pack(req, (uint16_t)sizeof(req), &mine) == PULSE_PACK_LEN,
          "45B 脉冲结果打包成功");
    CHECK(PULSE_PACK_LEN <= LINK_MAX_PAYLOAD, "45B 装得进链路单帧(64B)");

    CHECK(Link_Request(&s_master, LINK_CMD_PULSE_XCHK, req, PULSE_PACK_LEN,
                       resp, &rxlen, 50u) == 1u, "PULSE_XCHK 有应答");
    CHECK(rxlen == PULSE_PACK_LEN, "应答也是 45B");

    memset(&peer, 0, sizeof(peer));
    CHECK(PulsePack_Unpack(resp, rxlen, &peer) == 1u, "对侧脉冲结果可解析");
    CHECK(peer.window_ms == 200u, "窗口长度传过来了");
    CHECK(peer.ph[PULSE_PH_A].period_ns == 50000u, "对侧周期传对了");
    CHECK(peer.ph[PULSE_PH_C].duty_permille == 500u, "对侧占空比传对了");
    CHECK(PulsePack_Compare(&mine, &peer, 0) == 0u, "两端一致 → 0 个差异");

    /* 请求太短 → NIY（1 字节），而不是静默超时 */
    rxlen = 0u;
    CHECK(Link_Request(&s_master, LINK_CMD_PULSE_XCHK, req, PULSE_PACK_LEN - 1u,
                       resp, &rxlen, 50u) == 1u, "太短的请求也有应答");
    CHECK(rxlen == 1u && resp[0] == LINK_ST_NIY, "太短的请求 → NIY");

    /* 连打 3 发 45B：确认长负载不打乱 SEQ 配对 */
    for (i = 0u; i < 3u; i++) {
        rxlen = 0u;
        CHECK(Link_Request(&s_master, LINK_CMD_PULSE_XCHK, req, PULSE_PACK_LEN,
                           resp, &rxlen, 50u) == 1u, "连续 45B 交换仍有应答");
        CHECK(rxlen == PULSE_PACK_LEN, "长度稳定");
    }
    CHECK(Link_Stats(&s_master)->rx_stale == 0u, "没有陈旧帧");
    CHECK(Link_Stats(&s_master)->rx_crc_err == 0u, "长负载无 CRC 错");
}
/*==============================================================================
 * 8) 链路抗干扰: CRC 错 / 垃圾前缀 / 超长 LEN
 *============================================================================*/
static void test_link_garbage(void)
{
    uint8_t  raw[LINK_RAW_MAX];
    uint16_t total;
    uint8_t  payload[4] = { 1u, 2u, 3u, 4u };

    SECTION("8) 链路抗干扰 (CRC 错 / 垃圾 / 超长 LEN) + 恢复");
    link_setup(0u);

    /* 8.1 CRC 错的帧: 只计 CRC 错, 不得当成有效帧 */
    total = Frame_Build(raw, LINK_CMD_STATUS, 9u, payload, 4u);
    raw[total - 1u] ^= 0xFFu;                   /* 弄坏 CRC */
    sim_inject_buf(SIM_SIDE_A, raw, total);
    Link_Poll(&s_master);
    CHECK(Link_Stats(&s_master)->rx_crc_err == 1u, "CRC 错被计数");
    CHECK(Link_Stats(&s_master)->rx_frames == 0u, "CRC 错的帧不算合法帧");
    CHECK(Link_Stats(&s_master)->rx_stale == 0u, "CRC 错的帧不污染应答状态");

    /* 8.2 前置垃圾 + 一个合法帧: 必须能重新同步并解出来 */
    {
        uint8_t junk[5] = { 0x00u, 0xA5u, 0x11u, 0x22u, 0x5Bu };
        sim_inject_buf(SIM_SIDE_A, junk, 5u);
        total = Frame_Build(raw, LINK_CMD_PING, 10u, payload, 4u);
        sim_inject_buf(SIM_SIDE_A, raw, total);
        Link_Poll(&s_master);
        CHECK(Link_Stats(&s_master)->rx_resync > 0u, "垃圾被计入重同步计数");
        CHECK(Link_Stats(&s_master)->rx_stale == 1u, "合法但 SEQ 不匹配 → 计陈旧帧");
        CHECK(Link_Stats(&s_master)->rx_crc_err == 1u, "没有引入新的 CRC 错");
    }

    /* 8.3 超长 LEN: 直接判非法, 且之后的合法帧不受影响 */
    {
        uint8_t bad[8] = { FRAME_HDR0, FRAME_HDR1, 0x01u, 0x00u,
                           0x00u, 0x01u, 0x01u, 0x00u };   /* LEN = 0x0100 = 256 */
        sim_inject_buf(SIM_SIDE_A, bad, 8u);
        Link_Poll(&s_master);
        CHECK(Link_Stats(&s_master)->rx_len_err == 1u, "超长 LEN 被拒绝并计数");

        total = Frame_Build(raw, LINK_CMD_PING, 11u, payload, 4u);
        sim_inject_buf(SIM_SIDE_A, raw, total);
        Link_Poll(&s_master);
        CHECK(Link_Stats(&s_master)->rx_stale == 2u, "之后的合法帧仍能解出");
    }

    /* 8.4 干扰之后功能仍正常 */
    Link_SetSlaveHandler(&s_slave, slave_handler);
    s_peer_poll = slave_poll_fn;
    {
        uint8_t back = 0u;
        uint16_t rxlen = 0u;
        uint8_t  one = 0x5Au;
        CHECK(Link_Request(&s_master, LINK_CMD_PING, &one, 1u, &back, &rxlen, 50u) == 1u,
              "干扰后链路仍可用");
        CHECK(rxlen == 1u && back == 0x5Au, "回显正确");
    }
}

/*==============================================================================
 * 9) 忙保护 / 非阻塞语义
 *============================================================================*/
static void test_link_busy_and_nonblock(void)
{
    uint8_t  rx[4];
    uint16_t rxlen = 0u;

    SECTION("9) 忙保护 / timeout=0 的非阻塞语义");
    link_setup(0u);                             /* 没有对侧: 请求会一直挂着 */

    CHECK(Link_RequestStart(&s_master, LINK_CMD_PING, 0, 0u) == 1u, "第一发发出");
    CHECK(Link_Busy(&s_master) == 1u, "进入等待应答状态");
    CHECK(Link_RequestStart(&s_master, LINK_CMD_PING, 0, 0u) == 0u, "第二发被拒绝");
    CHECK(Link_Stats(&s_master)->tx_busy == 1u, "tx_busy 计数 +1");

    /* timeout=0: 只轮询一遍就返回 2 (主循环里用 Start/Poll 两段) */
    CHECK(Link_RequestPoll(&s_master, 0u, rx, &rxlen) == 2u, "timeout=0 → 立刻判失败");
    CHECK(Link_Busy(&s_master) == 0u, "失败后复位忙标志");
    CHECK(Link_Stats(&s_master)->tx_timeout == 1u, "计入超时");

    /* Poll 在没有进行中请求时返回 2, 不越界写 */
    rxlen = 88u;
    CHECK(Link_RequestPoll(&s_master, 100u, rx, &rxlen) == 2u, "空闲 Poll → 2");
    CHECK(rxlen == 0u, "空闲 Poll 不改 outlen");

    /* 从机侧调 Start 必须被拒 (角色不对) */
    CHECK(Link_RequestStart(&s_slave, LINK_CMD_PING, 0, 0u) == 0u, "从机不能发请求");
    /* 长度越界 */
    {
        uint8_t big[LINK_MAX_PAYLOAD + 1u];
        memset(big, 0, sizeof(big));
        CHECK(Link_RequestStart(&s_master, LINK_CMD_PING, big, sizeof(big)) == 0u,
              "超长请求被拒");
    }
}

/*==============================================================================
 * 10) STATUS 打包/解包 (两片共用同一份编解码)
 *============================================================================*/
static void test_link_status(void)
{
    uint8_t        buf[LINK_STATUS_LEN];
    link_status_t  st;
    link_stats_t   ls;
    uart_stats_t   us;
    static const uint8_t ver[2] = { 0x12u, 0x34u };

    SECTION("10) STATUS 数据区打包/解包");
    memset(&ls, 0, sizeof(ls));
    memset(&us, 0, sizeof(us));
    ls.rx_frames  = 0x01020304u;
    ls.rx_crc_err = 5u;
    us.rx_bytes   = 0x0A0B0C0Du;
    us.rx_drop    = 9u;

    CHECK(LINK_STATUS_LEN == 28u, "STATUS 应答固定 28B (v1.3 起)");
    CHECK(Link_PackStatus(buf, sizeof(buf), ver, 0x5566u, &ls, &us,
                          LINK_CAP_ALL, 0x0003u, 0x0007u) == LINK_STATUS_LEN,
          "打包返回 LINK_STATUS_LEN 字节");
    CHECK(buf[0] == 0x12u && buf[1] == 0x34u, "版本字节");
    CHECK(buf[2] == (uint8_t)(LINK_CODE_VER >> 8) && buf[3] == (uint8_t)LINK_CODE_VER,
          "链路代码版本大端排在 [2..3]");
    CHECK(buf[4] == 0x55u && buf[5] == 0x66u, "alive 大端排在 [4..5]");
    CHECK(buf[6] == 0x01u && buf[7] == 0x02u && buf[8] == 0x03u && buf[9] == 0x04u,
          "rx_frames 大端 u32 排在 [6..9]");
    CHECK(buf[14] == 0x0Au && buf[17] == 0x0Du, "rx_bytes 排在 [14..17]");
    CHECK(buf[18] == 0u && buf[21] == 9u, "rx_drop 排在 [18..21]");
    CHECK(buf[22] == 0x00u && buf[23] == 0xFFu, "caps 大端排在 [22..23]");
    CHECK(buf[24] == 0x00u && buf[25] == 0x03u, "ev_pending 大端排在 [24..25]");
    CHECK(buf[26] == 0x00u && buf[27] == 0x07u, "ev_overflow 大端排在 [26..27]");

    memset(&st, 0, sizeof(st));
    CHECK(Link_UnpackStatus(buf, LINK_STATUS_LEN, &st) == 1u, "解包成功");
    CHECK(st.ver_major == 0x12u && st.ver_minor == 0x34u, "版本一致");
    CHECK(st.code_ver == LINK_CODE_VER, "链路代码版本一致");
    CHECK(st.alive == 0x5566u, "alive 一致");
    CHECK(st.rx_frames == ls.rx_frames, "rx_frames 一致");
    CHECK(st.rx_crc_err == ls.rx_crc_err, "rx_crc_err 一致");
    CHECK(st.rx_bytes == us.rx_bytes, "rx_bytes 一致");
    CHECK(st.rx_drop == us.rx_drop, "rx_drop 一致");
    CHECK(Link_CheckVersion(&st) == 1u, "同版本 → 一致性检查通过");
    CHECK(st.caps == LINK_CAP_ALL, "caps 解出一致");
    CHECK(st.ev_pending == 0x0003u && st.ev_overflow == 0x0007u, "事件字段解出一致");

    /* 长度不足必须拒绝, 不许读越界 */
    CHECK(Link_UnpackStatus(buf, LINK_STATUS_LEN - 1u, &st) == 0u, "长度不足被拒");
    CHECK(Link_PackStatus(buf, 4u, ver, 1u, &ls, &us, 0u, 0u, 0u) == 0u, "缓冲不足被拒");
    /* 空指针统计必须当成 0 而不是崩 */
    CHECK(Link_PackStatus(buf, sizeof(buf), 0, 0u, 0, 0, 0u, 0u, 0u) == LINK_STATUS_LEN,
          "空指针安全");
}

/*==============================================================================
 * 10b) 两片 链路代码版本一致性 ("只重烧了一片"必须看得见)
 *============================================================================*/
static void test_code_ver(void)
{
    uint8_t       buf[LINK_STATUS_LEN];
    link_status_t st;
    static const uint8_t ver[2] = { 0x00u, 0x01u };

    SECTION("10b) 链路代码版本一致性检查");
    memset(&st, 0, sizeof(st));
    CHECK(Link_PackStatus(buf, sizeof(buf), ver, 1u, 0, 0, 0u, 0u, 0u) == LINK_STATUS_LEN, "打包成功");
    CHECK(Link_UnpackStatus(buf, sizeof(buf), &st) == 1u, "解包成功");
    CHECK(Link_CheckVersion(&st) == 1u, "两片版本相同 → 一致");

    st.code_ver = (uint16_t)(LINK_CODE_VER ^ 0x00A5u);   /* 假装对端是旧版本 */
    CHECK(Link_CheckVersion(&st) == 0u, "版本不同 → 检出不一致");
    CHECK(Link_CheckVersion(0) == 0u, "空指针安全 (按不一致处理)");
}

/*==============================================================================
 * 11) 端到端: 走链路的 STATUS (U1 拿 U2 的健康度)
 *============================================================================*/
static void test_link_status_e2e(void)
{
    uint8_t  rx[LINK_STATUS_LEN];
    uint16_t rxlen = 0u;
    link_status_t st;

    SECTION("11) 端到端 STATUS (U1 读 U2 健康度)");
    link_setup(1u);

    /* 先打一发 PING 让从机有统计 */
    CHECK(Link_Request(&s_master, LINK_CMD_PING, 0, 0u, 0, 0, 50u) == 1u, "先 PING 一发");

    memset(&st, 0, sizeof(st));
    CHECK(Link_Request(&s_master, LINK_CMD_STATUS, 0, 0u, rx, &rxlen, 50u) == 1u,
          "STATUS 有应答");
    CHECK(rxlen == LINK_STATUS_LEN, "应答长度 = LINK_STATUS_LEN");
    CHECK(Link_UnpackStatus(rx, rxlen, &st) == 1u, "解包成功");
    CHECK(st.ver_minor == 0x01u, "U2 版本 0.1");
    CHECK(st.code_ver == LINK_CODE_VER, "两片 链路代码版本一致");
    CHECK(Link_CheckVersion(&st) == 1u, "一致性检查通过");
    CHECK(st.alive == 7u, "存活计数来自处理函数");
    CHECK(st.rx_frames >= 2u, "从机已收 >=2 帧");
    CHECK(st.rx_bytes >= 20u, "从机端口已收 >=20B");
    CHECK(Link_LastRespCmd(&s_master) == LINK_CMD_STATUS, "记录到的应答命令是 STATUS");

    /* 未实现的命令要明确回 NIY, 不能静默超时 */
    {
        uint8_t niy = 0u;
        rxlen = 0u;
        CHECK(Link_Request(&s_master, LINK_CMD_MEAS_GET, 0, 0u, &niy, &rxlen, 50u) == 1u,
              "未实现命令也必须有应答");
        CHECK(rxlen == 1u && niy == LINK_ST_NIY, "应答内容是 NIY");
    }
}

/*==============================================================================
 * 14) 半截帧超时 (docs/13 §3.5)
 *============================================================================*/
static void test_link_half_frame(void)
{
    static const uint8_t junk[5] = { 0xA5u, 0x5Bu, 0x01u, 0x00u, 0x00u };

    SECTION("14) 半截帧超时");
    link_setup(1u);

    sim_inject_buf(SIM_SIDE_B, junk, 5u);      /* 只发到"半个帧头"就断了 */
    Link_Poll(&s_slave);
    CHECK(Link_Stats(&s_slave)->rx_half == 0u, "刚到的字节不算半截帧");

    g_tick += (LINK_HALF_FRAME_MS + 2u);       /* 时间跳过阈值 */
    Link_Poll(&s_slave);
    CHECK(Link_Stats(&s_slave)->rx_half == 1u, "超过 5ms 未续 → 判半截帧并丢弃");

    CHECK(Link_Request(&s_master, LINK_CMD_PING, 0, 0u, 0, 0, 50u) == 1u,
          "半截帧之后链路仍能正常收发");
}

/*==============================================================================
 * 15) 请求重传 + 从机幂等缓存 (docs/13 §3.4)
 *============================================================================*/
static void test_link_retry_dup(void)
{
    uint8_t  one = 0x5Au;
    uint8_t  rx[LINK_MAX_PAYLOAD];
    uint16_t rxlen = 0u;

    SECTION("15) 请求重传 + 从机幂等");
    link_setup(1u);
    s_peer_poll = 0;                        /* 对侧主循环先"罢工" */

    CHECK(Link_RequestStart(&s_master, LINK_CMD_PING, &one, 1u) == 1u, "第一发发出");
    g_tick += 60u;                          /* 越过 50ms 超时点 */
    CHECK(Link_RequestPoll(&s_master, 50u, 0, 0) == 0u, "超时点到了 → 自动重传, 仍在等");
    CHECK(Link_Stats(&s_master)->tx_retry == 1u, "tx_retry = 1");
    CHECK(Link_Stats(&s_master)->tx_timeout == 0u, "重传成功前不计超时");

    s_peer_poll = slave_poll_fn;            /* 对侧恢复 */
    Link_Poll(&s_slave);                    /* 对侧处理积压的两帧(同序号) */
    CHECK(Link_Stats(&s_slave)->slave_reply == 2u, "两帧都回了");
    CHECK(Link_Stats(&s_slave)->slave_dup == 1u, "第二帧识别为重复请求(未重复执行)");

    CHECK(Link_RequestPoll(&s_master, 50u, rx, &rxlen) == 1u, "重传后拿到应答");
    CHECK(rxlen == 1u && rx[0] == one, "回显内容正确");
    CHECK(Link_Stats(&s_master)->tx_timeout == 0u, "重传成功 → 不计超时");
    CHECK(Link_Busy(&s_master) == 0u, "请求已收尾");

    /* 两次都不成(重传也用尽)才算超时失败 */
    link_setup(0u);
    CHECK(Link_Request(&s_master, LINK_CMD_PING, 0, 0u, 0, 0, 20u) == 0u, "对侧不在 → 失败");
    CHECK(Link_Stats(&s_master)->tx_retry == 1u, "用尽 1 次重传");
    CHECK(Link_Stats(&s_master)->tx_timeout == 1u, "最终计 1 次超时");
}

/*==============================================================================
 * 16) 事件上报: 快路径 EVENT(N=0) + 慢路径 EVENT_GET (docs/13 §5.7/§5.8)
 *============================================================================*/
static void test_link_event(void)
{
    uint8_t      resp[LINK_MAX_PAYLOAD];
    uint16_t     rxlen = 0u;
    uint16_t     dup_seq;
    link_event_t ev;

    SECTION("16) 事件上报 (EVENT / EVENT_GET)");
    link_setup(1u);

    /* ── 快路径: 从机登记事件 → 立刻主动发 EVENT(N=0) ── */
    Link_SlaveEvent(&s_slave, LINK_EV_ADC_SAT, LINK_SEV_WARN, 5u, 4095u);
    Link_Poll(&s_master);
    CHECK(Link_Stats(&s_master)->rx_event == 1u, "主机收到 1 帧主动事件");
    CHECK(Link_Stats(&s_master)->rx_stale == 0u, "事件帧不会被当成陈旧帧");
    CHECK(Link_TakeEvent(&s_master, &ev) == 1u, "取出事件");
    CHECK(ev.code == LINK_EV_ADC_SAT && ev.sev == LINK_SEV_WARN, "事件码/严重度正确");
    CHECK(ev.arg0 == 5u && ev.arg1 == 4095u, "参数正确");
    CHECK(ev.count == 1u, "累计次数 = 1");
    CHECK(Link_TakeEvent(&s_master, &ev) == 0u, "取过就没了");

    /* 同一事件再发生 → 计数累加、位图置位、再发一帧 */
    Link_SlaveEvent(&s_slave, LINK_EV_ADC_SAT, LINK_SEV_WARN, 5u, 4095u);
    Link_Poll(&s_master);
    CHECK(Link_Stats(&s_master)->rx_event == 2u, "又收到一帧 EVENT");
    CHECK(Link_SlaveEventPending(&s_slave) == (uint16_t)(1u << (LINK_EV_ADC_SAT - 1u)),
          "从机待决位图置位");

    /* ── 慢路径: STATUS 看到位图 → EVENT_GET 取一条并清位 ── */
    CHECK(Link_Request(&s_master, LINK_CMD_EVENT_GET, 0, 0u, resp, &rxlen, 50u) == 1u,
          "EVENT_GET 有应答");
    CHECK(rxlen == LINK_EVENT_GET_LEN, "应答固定 9B");
    CHECK(resp[0] == LINK_ST_OK, "状态 OK");
    CHECK(resp[1] == LINK_EV_ADC_SAT && resp[2] == LINK_SEV_WARN, "取到事件码/严重度");
    CHECK(resp[8] == 2u, "累计次数 = 2");
    CHECK(Link_SlaveEventPending(&s_slave) == 0u, "取即清 → 位图清零");

    CHECK(Link_Request(&s_master, LINK_CMD_EVENT_GET, 0, 0u, resp, &rxlen, 50u) == 1u,
          "再取一次也有应答");
    CHECK(resp[1] == 0u, "没有待决事件 → ev_code = 0");

    /* ── 幂等: 重放"同一序号同一命令", 必须回缓存而不是再取一条 ── */
    Link_SlaveEvent(&s_slave, LINK_EV_EDGE_LOST, LINK_SEV_INFO, 3u, 0u);
    CHECK(Link_Request(&s_master, LINK_CMD_EVENT_GET, 0, 0u, resp, &rxlen, 50u) == 1u,
          "取走 EDGE_LOST");
    CHECK(resp[1] == LINK_EV_EDGE_LOST, "取到 EDGE_LOST");
    CHECK(Link_SlaveEventPending(&s_slave) == 0u, "位图已清空（现在没有待决事件了）");

    dup_seq = s_master.wait_seq;                 /* 刚才那一发用的序号 */
    {
        uint8_t  raw[LINK_RAW_MAX];
        uint16_t n = Frame_Build(raw, LINK_CMD_EVENT_GET, dup_seq, 0, 0);
        CHECK(UartPort_Write(UART_PORT_0, raw, n) == n, "手工重放同一请求(同序号同命令)");
    }
    Link_Poll(&s_slave);
    CHECK(Link_Stats(&s_slave)->slave_dup == 1u, "被识别为重复请求");
    {
        uint8_t  rxb[LINK_RAW_MAX];
        uint8_t  dat[LINK_MAX_PAYLOAD];
        uint16_t c2 = 0u, s2 = 0u, d2 = 0u;
        uint16_t got = UartPort_Read(UART_PORT_0, rxb, (uint16_t)sizeof(rxb));

        CHECK(got > 0u, "拿到应答字节");
        CHECK(Frame_Parse(rxb, got, &c2, &s2, dat, &d2) == 1u, "应答帧可解析");
        CHECK(c2 == LINK_CMD_EVENT_GET && s2 == dup_seq, "命令码与序号回显");
        CHECK(d2 == LINK_EVENT_GET_LEN && dat[1] == LINK_EV_EDGE_LOST,
              "回的是缓存内容(EDGE_LOST) 而不是空事件 → 幂等成立");
    }

    /* 事件码越界 → 只计数, 不进入事件表 */
    Link_SlaveEvent(&s_slave, 0x42u, LINK_SEV_INFO, 0u, 0u);
    CHECK(Link_SlaveEventOverflow(&s_slave) == 1u, "认不出的码计入 ev_overflow");
    CHECK(Link_SlaveEventPending(&s_slave) == 0u, "越界码不进待决位图");
}

/*==============================================================================
 * 17) ARM 同步触发: 两端在同一个时间窗里采 (docs/13 §5.5)
 *============================================================================*/
static void test_link_arm(void)
{
    uint8_t  req[LINK_ARM_REQ_LEN];
    uint8_t  resp[LINK_MAX_PAYLOAD];
    uint16_t rxlen = 0u;
    uint16_t win = 0u;

    SECTION("17) ARM 同步触发");
    link_setup(1u);

    CHECK(Link_ArmPackReq(req, sizeof(req), 7u, 200u, 0u) == LINK_ARM_REQ_LEN,
          "请求打包固定 5B");
    CHECK(Link_Request(&s_master, LINK_CMD_ARM, req, LINK_ARM_REQ_LEN, resp, &rxlen, 50u) == 1u,
          "ARM 有应答");
    CHECK(rxlen == LINK_ARM_RESP_LEN, "应答固定 3B");
    CHECK(Link_ArmCheckResp(resp, rxlen, 7u) == 1u, "状态 OK 且 session 回显一致");
    CHECK(Link_ArmCheckResp(resp, rxlen, 8u) == 0u, "session 对不上 → 不算成功");
    CHECK(resp[1] == 0u && resp[2] == 7u, "session 大端回显");

    /* 从机侧: 应用层应当被通知一次, 并且只通知一次 */
    CHECK(Link_SlaveArmState(&s_slave) == LINK_ARM_ARMED, "状态 = ARMED");
    CHECK(Link_SlaveArmTake(&s_slave, &win) == 1u, "应用层取到本次 ARM");
    CHECK(win == 200u, "窗口长度传过来了");
    CHECK(Link_SlaveArmTake(&s_slave, &win) == 0u, "同一次 ARM 不会再通知一次");

    /* 窗口还在跑, 来个**新 session** → BUSY (不打断当前窗口, 也不排队) */
    CHECK(Link_ArmPackReq(req, sizeof(req), 8u, 200u, 0u) == LINK_ARM_REQ_LEN, "换 session");
    CHECK(Link_Request(&s_master, LINK_CMD_ARM, req, LINK_ARM_REQ_LEN, resp, &rxlen, 50u) == 1u,
          "有应答");
    CHECK(resp[0] == LINK_ST_BUSY, "窗口进行中 → BUSY");
    CHECK(Link_SlaveArmTake(&s_slave, &win) == 0u, "BUSY 时不会开新窗");

    /* 窗口结束 → READY；此时新 session 可以覆盖 */
    Link_SlaveArmDone(&s_slave);
    CHECK(Link_SlaveArmState(&s_slave) == LINK_ARM_READY, "状态 = READY");
    CHECK(Link_Request(&s_master, LINK_CMD_ARM, req, LINK_ARM_REQ_LEN, resp, &rxlen, 50u) == 1u,
          "READY 时接受新 ARM");
    CHECK(resp[0] == LINK_ST_OK, "新 ARM 被接受");
    CHECK(Link_SlaveArmTake(&s_slave, &win) == 1u && win == 200u, "又通知一次");

    /* 幂等: 同一 session 再来(换了个序号, 模拟主机主动重发) → 回 OK 但不重开窗 */
    CHECK(Link_ArmPackReq(req, sizeof(req), 8u, 500u, 0u) == LINK_ARM_REQ_LEN, "同 session 改窗口");
    CHECK(Link_Request(&s_master, LINK_CMD_ARM, req, LINK_ARM_REQ_LEN, resp, &rxlen, 50u) == 1u,
          "有应答");
    CHECK(resp[0] == LINK_ST_OK, "同 session → 回 OK");
    CHECK(Link_SlaveArmTake(&s_slave, &win) == 0u, "同 session 不重开窗(不该再通知)");

    /* 参数校验: 一律 ERR_PARAM, 且不改变状态 */
    CHECK(Link_ArmPackReq(req, sizeof(req), 0u, 200u, 0u) == LINK_ARM_REQ_LEN, "session = 0");
    CHECK(Link_Request(&s_master, LINK_CMD_ARM, req, LINK_ARM_REQ_LEN, resp, &rxlen, 50u) == 1u,
          "有应答");
    CHECK(resp[0] == LINK_ST_ERR_PARAM, "session=0 → ERR_PARAM");
    CHECK(Link_ArmPackReq(req, sizeof(req), 9u, 0u, 0u) == LINK_ARM_REQ_LEN, "window = 0");
    CHECK(Link_Request(&s_master, LINK_CMD_ARM, req, LINK_ARM_REQ_LEN, resp, &rxlen, 50u) == 1u,
          "有应答");
    CHECK(resp[0] == LINK_ST_ERR_PARAM, "window=0 → ERR_PARAM");
    CHECK(Link_ArmPackReq(req, sizeof(req), 9u, (uint16_t)(LINK_ARM_WIN_MAX + 1u), 0u) == LINK_ARM_REQ_LEN,
          "window 越界");
    CHECK(Link_Request(&s_master, LINK_CMD_ARM, req, LINK_ARM_REQ_LEN, resp, &rxlen, 50u) == 1u,
          "有应答");
    CHECK(resp[0] == LINK_ST_ERR_PARAM, "window 越界 → ERR_PARAM");
    CHECK(Link_ArmPackReq(req, sizeof(req), 9u, 200u, LINK_ARM_F_HWSYNC) == LINK_ARM_REQ_LEN,
          "flags 硬触发");
    CHECK(Link_Request(&s_master, LINK_CMD_ARM, req, LINK_ARM_REQ_LEN, resp, &rxlen, 50u) == 1u,
          "有应答");
    CHECK(resp[0] == LINK_ST_ERR_PARAM, "本版不支持硬触发 → ERR_PARAM");

    /* 长度不对也要回 ERR_PARAM, 而不是 NIY */
    CHECK(Link_Request(&s_master, LINK_CMD_ARM, req, (uint16_t)(LINK_ARM_REQ_LEN - 1u),
                       resp, &rxlen, 50u) == 1u, "短请求也有应答");
    CHECK(resp[0] == LINK_ST_ERR_PARAM, "长度不足 → ERR_PARAM");
}

/*==============================================================================
 * 18) LINK_RESET: 清会话（收发环 / 半截帧 / ARM / 幂等），事件与统计要留
 *============================================================================*/
static void test_link_reset(void)
{
    uint8_t  resp[LINK_MAX_PAYLOAD];
    uint16_t rxlen = 0u;

    SECTION("18) LINK_RESET 链路复位");
    link_setup(1u);

    /* 先制造中间态: ARM 已开窗 */
    {
        uint8_t req[LINK_ARM_REQ_LEN];
        CHECK(Link_ArmPackReq(req, sizeof(req), 3u, 200u, 0u) == LINK_ARM_REQ_LEN, "打包 ARM");
        CHECK(Link_Request(&s_master, LINK_CMD_ARM, req, LINK_ARM_REQ_LEN, resp, &rxlen, 50u) == 1u,
              "先 ARM 一发");
        CHECK(Link_SlaveArmState(&s_slave) == LINK_ARM_ARMED, "ARMED");
    }
    Link_SlaveEvent(&s_slave, LINK_EV_ADC_SAT, LINK_SEV_WARN, 1u, 4095u);
    CHECK(Link_SlaveEventPending(&s_slave) != 0u, "有一条待决事件");

    /* 从机接收环里再堆点垃圾（不含帧头），复位时应当被一起清掉 */
    {
        uint8_t junk[16];
        memset(junk, 0x11, sizeof(junk));
        sim_inject_buf(SIM_SIDE_B, junk, (uint16_t)sizeof(junk));
        CHECK(UartPort_Available(UART_PORT_1) == (uint16_t)sizeof(junk), "垃圾先堆在环里");
    }

    CHECK(Link_Request(&s_master, LINK_CMD_RESET, 0, 0u, resp, &rxlen, 50u) == 1u, "RESET 有应答");
    CHECK(rxlen == 1u && resp[0] == LINK_ST_OK, "RESET → OK");
    CHECK(Link_SlaveArmState(&s_slave) == LINK_ARM_IDLE, "从机 ARM 状态被复位");
    CHECK(UartPort_Available(UART_PORT_1) == 0u, "从机接收环被清空");
    CHECK(Link_SlaveEventPending(&s_slave) != 0u, "事件表不清（故障证据要留）");

    /* 主机侧：复位自己的会话 */
    Link_MasterReset(&s_master);
    CHECK(Link_Busy(&s_master) == 0u, "主机不再忙");
    CHECK(Link_Request(&s_master, LINK_CMD_PING, 0, 0u, 0, 0, 50u) == 1u, "复位后链路仍可用");
    CHECK(Link_SlaveEventPending(&s_slave) != 0u, "事件仍在（没被复位动作吃掉）");
}

/*============================================================================*/
int main(void)
{
    printf("========================================\n");
    printf(" UART port + U1<->U2 link host test\n");
    printf("========================================\n");

    test_ring_basic();
    test_tx_chain();
    test_rx_overflow();
    test_tx_overflow();
    test_error_rearm();
    test_link_ping();
    test_link_timeout();
    test_link_garbage();
    test_link_busy_and_nonblock();
    test_link_status();
    test_code_ver();
    test_link_status_e2e();
    test_meas_xchk();
    test_pulse_xchk();
    test_link_half_frame();
    test_link_retry_dup();
    test_link_event();
    test_link_arm();
    test_link_reset();

    printf("\n========================================\n");
    printf(" checks: %d, failed: %d\n", g_checks, g_fails);
    printf(" %s\n", (g_fails == 0) ? "ALL PASS" : "FAILURES PRESENT");
    printf("========================================\n");
    return (g_fails == 0) ? 0 : 1;
}
