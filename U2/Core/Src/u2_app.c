/**
  ******************************************************************************
  * @file    u2_app.c
  * @brief   U2 应用层: 链路从机 (PING / STATUS / MEAS_XCHK / PULSE_XCHK)
  *
  * 设计上刻意很薄:
  *   帧层/收发在各自的 uart_link.c + uart_port.c, 采集/换算在 measure.c,
  *   打包在 meas_pack.c / pulse_pack.c（都是与 U1 逐字节一致的副本）。
  *   本文件只负责"收到命令做什么"——U2 就三件事: 报状态、交测量快照、
  *   交脉冲结果。**不做任何判定**（判定归 U1，避免两套判据跑偏）。
  *
  * 脉冲捕获在本片是**连续滚动**的：一个 200ms 窗口接一个窗口地跑
  * （U2 没有别的活，10% 左右的 CPU 完全吃得下），这样 U1 一问就有
  * 不超过 200ms 的"新鲜"结果，不需要事先打招呼。
  ******************************************************************************
  */
#include "u2_app.h"
#include "uart_cfg.h"
#include "uart_link.h"
#include "measure.h"
#include "meas_pack.h"
#include "pulse_cap.h"
#include "pulse_pack.h"
#include "pulse_cfg.h"

static link_t   s_u1_link;          /* 与 U1 的链路 (本片是从机) */
static uint16_t s_alive;            /* STATUS 里的存活计数: 证明"这一发是真的新应答" */
static meas_snapshot_t s_snap;      /* 交给 U1 的快照（55B 线格式由 meas_pack 编码） */

/*------------------------------------------------------------------------------
 * 链路命令处理 (主循环上下文, 由 Link_Poll → link_deliver 调用)
 *----------------------------------------------------------------------------*/
static uint8_t U2_LinkHandler(uint16_t cmd, const uint8_t *in, uint16_t inlen,
                              uint8_t *out, uint16_t *outlen)
{
    switch (cmd) {

    /*── 回环: 原样回显。U1 用它测往返用时/丢包, 也用来验证接线是否交叉正确 ──*/
    case LINK_CMD_PING: {
        uint16_t i;
        if (inlen > LINK_MAX_PAYLOAD) return 0u;
        for (i = 0u; i < inlen; i++) {
            out[i] = in[i];
        }
        *outlen = inlen;
        return 1u;
    }

    /*── 本片状态: 版本 / 存活计数 / 链路与端口的健康度 ──*/
    case LINK_CMD_STATUS: {
        static const uint8_t ver[2] = { LINK_U2_VER_MAJOR, LINK_U2_VER_MINOR };
        s_alive++;
        *outlen = Link_PackStatus(out, LINK_MAX_PAYLOAD, ver, s_alive,
                                  Link_Stats(&s_u1_link),
                                  UartPort_Stats(UART_CFG_PORT_LINK));
        return (*outlen != 0u) ? 1u : 0u;
    }

    /*── 交叉校验: 请求里是 U1 的 55B 快照, 应答是本片的 55B 快照 ──
     * 一条命令一次往返完成"交换快照"; **比对由 U1 做**（判定方只有一个），
     * 本片不需要容差表、也不需要知道自己那一发数据"合格不合格"。 */
    case LINK_CMD_MEAS_XCHK: {
        if ((in == 0) || (inlen < MEAS_PACK_SNAP_LEN)) {
            out[0]  = LINK_ST_NIY;          /* 请求太短: 不是我们认识的格式 */
            *outlen = 1u;
            return 1u;
        }
        MEAS_Snapshot(&s_snap);                              /* 取本片最新数据 */
        *outlen = MeasPack_Snapshot(out, LINK_MAX_PAYLOAD, &s_snap);
        return (*outlen != 0u) ? 1u : 0u;
    }

    /*── 脉冲结果交换: 请求里是 U1 的 45B(输出端), 应答是本片的 45B(输入端) ──*/
    case LINK_CMD_PULSE_XCHK: {
        pulse_snapshot_t ps;
        if ((in == 0) || (inlen < PULSE_PACK_LEN)) {
            out[0]  = LINK_ST_NIY;
            *outlen = 1u;
            return 1u;
        }
        PULSE_Snapshot(&ps);                       /* 本片最近一个窗口的结果 */
        *outlen = PulsePack_Pack(out, LINK_MAX_PAYLOAD, &ps);
        return (*outlen != 0u) ? 1u : 0u;
    }

    /*── 其它测量命令(0x0102 预留): 明确回 NIY ──
     * 回 NIY(而不是不应答)是有意的: U1 才能区分"U2 不在/线没接"和
     * "U2 在, 但这条命令还没实现", 否则只能看到一个超时。 */
    default:
        out[0]  = LINK_ST_NIY;
        *outlen = 1u;
        return 1u;
    }
}

void U2App_Init(void)
{
    s_alive = 0u;
    Link_Init(&s_u1_link, UART_CFG_PORT_LINK, 0u /* 从机 */);
    Link_SetSlaveHandler(&s_u1_link, U2_LinkHandler);

    /* 脉冲捕获：配好并启动第一个窗口（之后一个接一个地滚动） */
    PULSE_Init();
    PULSE_CfgInit();
    PULSE_CfgStart(PULSE_WINDOW_MS);
}

void U2App_Run(void)
{
    Link_Poll(&s_u1_link);

    /* 滚动窗口：窗口到时结算，然后立刻开下一个 —— 这样 U1 一问就有新鲜结果 */
    if (PULSE_CfgPoll() != 0u) {
        PULSE_CfgStart(PULSE_WINDOW_MS);
    }
}
