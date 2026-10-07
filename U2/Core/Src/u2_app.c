/**
  ******************************************************************************
  * @file    u2_app.c
  * @brief   U2 应用层: 链路从机 (PING / STATUS)
  *
  * 设计上刻意很薄:
  *   帧层/收发在 各自的 uart_link.c + uart_port.c (两片各一份副本),
  *   本文件只负责"收到命令做什么"。③ 的测量功能加进来时, 也只在这里加 case。
  ******************************************************************************
  */
#include "u2_app.h"
#include "uart_cfg.h"
#include "uart_link.h"

static link_t   s_u1_link;          /* 与 U1 的链路 (本片是从机) */
static uint16_t s_alive;            /* STATUS 里的存活计数: 证明"这一发是真的新应答" */

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

    /*── ③ 测量相关命令: 现在明确回 NIY ──
     * 回 NIY(而不是不应答)是有意的: U1 才能区分"U2 不在/线没接"和
     * "U2 在, 但这条命令还没实现", 否则只能看到一个超时。 */
    case LINK_CMD_MEAS_GET:
    case LINK_CMD_MEAS_XCHK:
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
}

void U2App_Run(void)
{
    Link_Poll(&s_u1_link);
}
