/**
  ******************************************************************************
  * @file    app.c
  * @brief   治具主控 U1 应用层 — 命令分发 / ACK / 主循环
  *
  * 实现协议 (INFO/M1 测试治具PC端通讯协议.xlsx) 全部 8 条命令:
 *   0x0009 固件起始帧          → fw_upload.c
 *   0x000A 固件数据帧          → fw_upload.c
 *   0x000B 固件结束帧          → fw_upload.c
 *   0x000C 查询固件版本(Flash) → 本文件
 *   0x000D 查询固件版本(FOC板) → swd_programmer.c (SWD 直接读目标 Flash)
 *   0x000E 测量 ADC (1B mode)  → measure.c / meas_pack.c
 *   0x000F 脉冲捕获 (1B mode)  → pulse_cfg.c / pulse_cap.c / pulse_pack.c
 *   0x0112 烧录Flash固件到FOC板 → swd_programmer.c (SWD 两线, 不用 NRST)
  *   0x0011 DUT 透传 (追加)     → dut_proto.c + 本文件 (需要 JIG_DUT_LINK=1)
  * 其余命令统一回 NIY(0xFF)。
  ******************************************************************************
  */
#include "app.h"
#include "app_frame.h"
#include "jig_as.h"
#include "protocol_frame.h"
#include "flash_map.h"
#include "fw_upload.h"
#include "gd25q128.h"
#include "swd_programmer.h"
#include "bsp_uart.h"
#include "uart_cfg.h"
#include "uart_link.h"
#include "measure.h"
#include "meas_pack.h"
#include "pulse_cap.h"
#include "pulse_pack.h"
#include "pulse_cfg.h"
#include "dut_proto.h"
#include "cur_chk.h"
#include "vol_chk.h"
#include "phase_chk.h"
#include "enc_plan.h"
#include "enc_chk.h"
#include "sn_report.h"      /* 0x0024 板子 SN 主动上报（烧录成功后） */
#include "swd_selftest.h"   /* 0x001E SWD 烧录自检（B1：不依赖 SPI Flash） */
#include "swd_memap.h"      /* SWD_MemReadBlock：读被测板 96-bit UID */
#include "swd_phy.h"        /* SWD_OK 状态枚举 / SWD_PhyInit / SWD_PhySetDelay */
#include "swd_config.h"     /* SWD_DELAY_LOOPS：0x0028 恢复默认速度用 */
#include "swd_dp.h"         /* SWD_Connect / SWD_PowerUp（0x001E mode=7/8 用） */
#include "enc_gen.h"
#include "jig_mode.h"
#include "pc_link.h"        /* PC 通讯口: USB CDC 或 USART1（jig_mode.h §2.1） */
#include <string.h>

/* ASCII 控制台的内部函数（PC 串口助手手敲 AS:xxx）：定义在文件末尾紧挨 APP_PcInput，
 * 但 APP_Run 里要先调用 → 必须在这里前置声明。
 * ⚠ 别把这块挪进下面那个"台架专用"#if —— 那个块还要求 PC 口在串口，
 *   产品构建（PC 走 USB）里会被整块裁掉，声明跟着消失（2026-11-09 踩到）。 */
#if (JIG_DUT_LINK != 0)
static void    APP_FlowTestTask(void);
static uint8_t APP_AsciiAccept(uint8_t b);
static void    APP_AsciiDispatch(void);
static void    APP_AsciiFlush(void);
static void    APP_AsciiPoll(void);
#endif

/* 台架版的"自报身份"直接复用 PC 链路的发送环（uart_tx_kick 里有临界区保护），
 * 绝不在这里调阻塞的 HAL_UART_Transmit —— 那会抢走 RX 重挂要用的 HAL Lock，
 * 把正在收的字节永久丢掉（2026-11-05 实测踩到）。*/
#if (JIG_DUT_LINK != 0) && (JIG_PC_LINK == JIG_PC_LINK_UART)
#define APP_BENCH_ALIVE   1
#include "usart.h"          /* extern huart1：心跳要读 huart1.ErrorCode */
#else
#define APP_BENCH_ALIVE   0
#endif

/*==============================================================================
 * 主机测试桩（只在本机单元测试里编）
 *----------------------------------------------------------------------------
 * `0x001E` mode=6 会调 USB_EvtIrqCount/StrOr，真实固件里由 stm32f1xx_it.c 提供
 * （非 USB 构建给空实现）。但主机测试**不编译** stm32f1xx_it.c，所以要在这里补桩，
 * 否则 app_test 链接失败。
 *============================================================================*/
#if defined(JIG_HOST_TEST)
uint32_t USB_EvtIrqCount(void) { return 0u; }
uint32_t USB_EvtIstrOr(void)   { return 0u; }
#endif

/*==============================================================================
 * 应答长度的编译期自检（2026-11-09 精简布局时加）
 *----------------------------------------------------------------------------
 * 改布局最容易出的事故：**常量改了、打包代码没改**（或反过来）。
 * 那种错不会编译失败，只会让上位机把数字念歪 —— 所以在这里把它们绑死。
 * 放在 app.c 而不是 flash_map.h：这里能同时看到 MEAS_CH_COUNT / ENC_CHK_PH_COUNT /
 * CUR_CHK_CH_COUNT 三个计数宏（它们分别来自 measure.h / enc_chk.h / cur_chk.h）。
 *============================================================================*/
FOC_STATIC_ASSERT(MGR_VOL_TEST_RESP_LEN == (2u + (MEAS_CH_COUNT * MGR_VOL_CH_ITEM_LEN)),
                  resp_len_must_match_layout_vol);
FOC_STATIC_ASSERT(MGR_PHASE_JUDGE_RESP_LEN == (1u + (3u * MGR_PHASE_ITEM_LEN)),
                  resp_len_must_match_layout_phase);
FOC_STATIC_ASSERT(MGR_ENC_JUDGE_RESP_LEN == (3u + (ENC_CHK_PH_COUNT * MGR_ENC_JUDGE_ITEM_LEN)),
                  resp_len_must_match_layout_enc);
FOC_STATIC_ASSERT(MGR_CUR_TEST_RESP_LEN == (2u + (CUR_CHK_CH_COUNT * MGR_CUR_ITEM_LEN)),
                  resp_len_must_match_layout_cur);

/*==============================================================================
 * 全局状态
 *============================================================================*/
APP_State_t app_state = APP_STATE_INIT;
uint16_t    pc_seq    = 0u;

/* SWD 位翻转速度的用户设定（0 = 用编译默认 SWD_DELAY_LOOPS）。
 * 掉电不保留：避免忘了改回来，导致以后一直用慢速。 */
static uint16_t s_swd_delay_ovr;

static uint16_t prog_seq = 0u;      /* 0x0112 受理时的帧序号, 烧完 ACK 回显用 */
static uint16_t s_sst_seq = 0u;     /* 0x001E SWD 自检受理时的帧序号 */
static uint8_t  tx_buf[APP_FRAME_BUF_SIZE];

/*===== 到 DUT（被烧 FOC 板）的链路 — USART1，协议见 docs/15 =====
 * 只在 JIG_DUT_LINK=1 时存在；USART1 与调试打印二选一（jig_mode.h §3.4 有 #error 互锁）。
 * U1 是主机、DUT 是从机。本文件目前只实现「透传」(0x0011)；
 * ③④⑤⑥ 的编排后续加在同一套状态机上。 */
#if (JIG_DUT_LINK != 0)
#define APP_DUT_TIMEOUT_MS  200u    /* 等 DUT 应答的上限（透传是 bring-up 用，给宽些） */
#define APP_DUT_ACK_MAX     (1u + LINK_MAX_PAYLOAD)   /* ACK = jig 状态(1) + DUT 的 DATA 段 */

static link_t   s_dut_link;
static uint16_t s_dut_seq  = 0u;    /* 受理 0x0011 时的帧序号, 回 ACK 时回显（同 0x0112 的做法）*/
static uint8_t  s_dut_resp[LINK_MAX_PAYLOAD];   /* DUT 应答缓冲，放 .bss 不上栈 */
static uint8_t  s_dut_ack[APP_DUT_ACK_MAX];

/* 最近一次**应答**的命令号与状态字节（主动事件 seq=0 不记）。
 * 只给 0x0010 一键流程采集"每一步的结果"用，不改变任何既有行为。
 * 放在 DUT 链路块里：非 DUT 构建没有 0x0010，这两个变量就没人读（ARMCC 会报"set but never used"）。 */
static uint8_t  s_last_ack_st  = 0xFFu;

/*===== 单项"一条命令跑完"的流程状态（0x0019 ③ / 0x001A ④ / 0x001B ⑤ / 0x0014 ⑥）=====
 * 骨架统一是：进测试模式 → 激励/采集/判定 → 关输出 → 回一条报告。
 * 流程进行中**不把子命令的应答发给 PC**（见 APP_SendFrame），所以"一条命令 = 一条应答"。 */
#define APP_FLOW_REP_MAX     96u        /* 最大的一份子报告 = 0x000F 的 91B */
#define APP_FLOW_KIND_CUR    0u
#define APP_FLOW_KIND_PULSE  1u
#define APP_FLOW_KIND_PHASE  2u
#define APP_FLOW_KIND_ENC    3u

static uint8_t  s_flow_active;
static uint8_t  s_flow_wait;
static uint8_t  s_flow_kind;
static uint8_t  s_flow_step;
static uint8_t  s_flow_steps;
static uint8_t  s_flow_rep_step;
static uint16_t s_flow_seq;
static uint16_t s_flow_cmd;
static uint32_t s_flow_t0;
static uint8_t  s_flow_res;
static uint8_t  s_flow_cap[APP_FLOW_REP_MAX];   /* 最近一条子应答（逐步刷新） */
static uint16_t s_flow_cap_len;
static uint8_t  s_flow_rep[APP_FLOW_REP_MAX];   /* 最终要回的那份报告 */
static uint16_t s_flow_rep_len;
static uint8_t  s_flow_buf[2u + 16u];           /* 组 DUT 命令用 */

static void APP_CurStartMachine(void);
static void APP_FlowStart(uint8_t kind, uint16_t cmd);
#endif

/*===== ⑤ 编码器脉冲发生（0x0012） =====
 * 纯治具侧：U1 用 TIM4 在 CN2 的 A/B/Z 上发定量脉冲串，U2（将来）用 TIM4 采集做回环。
 * 这条命令单独留着，是为了**不接 DUT、不做编排就能上板验证脉冲发生**。 */
static uint16_t   s_enc_seq = 0u;       /* 受理时的帧序号，发完回 ACK 用 */
static enc_plan_t s_enc_plan;

/* ⑤ 回环：把 0x0013 转成内部链路的 ENC_XCHK 发给 U2 */
static uint16_t s_encp_seq = 0u;
static uint8_t  s_encp_act = 0u;        /* 要发的动作（0 清零 / 1 读取） */
static uint8_t  s_encp_started = 0u;    /* 请求是否已经发出去了 */
static uint32_t s_encp_t0 = 0u;         /* 等待链路空出来的起点 */
static uint8_t  s_encp_resp[LINK_MAX_PAYLOAD];
static uint8_t  s_encp_ack[1u + LINK_MAX_PAYLOAD];

/*===== 到 U2 的内部链路 (USART3, 见 docs/10_UART四路驱动设计.md) =====
 * ⚠ 台架形态（jig_mode.h §3.4.1，JIG_U2_LINK=0）下 U2 不在、USART3 归被测板，
 *   这一整段**不编** —— 否则每 500ms 一次的保活探测会把 U2 的帧灌到被测板的
 *   串口线上（被测板收到垃圾帧，治具自己的应答却看起来一切正常）。
 *
 *   下面那组 APP_U2xxx 包装在两种形态下**签名完全一样**：没有 U2 时它就是
 *   "发不出去 / 等不到 / 不忙"，调用方原有的失败分支（NO_PEER、超时、BUSY）
 *   自然生效，不必在十几处测量代码里塞 #if。 */
#if (JIG_U2_LINK != 0)
#define APP_U2_PROBE_PERIOD_MS  500u    /* 空闲时每 500ms 探一次 U2 */
#define APP_U2_PROBE_TIMEOUT_MS 50u     /* 50ms 没回算丢 (115200 下往返 < 2ms) */
#define APP_U2_STATUS_EVERY     5u      /* 每 5 次探测里用一次 STATUS (≈2.5s) */

static link_t        s_u2_link;
static uint32_t      s_probe_tick;
static uint8_t       s_probe_cnt;
static uint8_t       s_probe_is_status;     /* 当前在等的是 STATUS 的应答 */
static uint16_t      s_probe_ok;
static uint16_t      s_probe_fail;
static uint8_t       s_u2_alive;
static uint8_t       s_u2_ver_ok;           /* 最近一次 STATUS: 链路代码版本一致 */
static uint8_t       s_u2_ver_checked;      /* 是否收到过 STATUS */
static link_status_t s_u2_status;
static uint8_t       s_link_rx[LINK_MAX_PAYLOAD];

static uint8_t APP_U2Send(uint16_t cmd, const uint8_t *d, uint16_t n)
{
    return Link_RequestStart(&s_u2_link, cmd, d, n);
}
static uint8_t APP_U2Poll(uint32_t tmo, uint8_t *buf, uint16_t *len)
{
    return Link_RequestPoll(&s_u2_link, tmo, buf, len);
}
static void APP_U2Flush(void)
{
    (void)Link_RequestPoll(&s_u2_link, 0u, 0, 0);   /* 只巡一遍，收掉陈旧请求 */
}
static uint8_t APP_U2Busy(void)
{
    return Link_Busy(&s_u2_link);
}
static const link_stats_t *APP_U2Stats(void)
{
    return Link_Stats(&s_u2_link);
}
#else
/* 台架形态：U2 不在。返回值语义与真链路对齐，让上层分支原样复用 */
static uint8_t APP_U2Send(uint16_t cmd, const uint8_t *d, uint16_t n)
{
    (void)cmd; (void)d; (void)n;
    return 0u;                                       /* 发不出去 */
}
static uint8_t APP_U2Poll(uint32_t tmo, uint8_t *buf, uint16_t *len)
{
    (void)tmo; (void)buf;
    if (len != 0) { *len = 0u; }
    return 2u;                                       /* 2 = 超时 */
}
static void APP_U2Flush(void) { }
static uint8_t APP_U2Busy(void) { return 0u; }
static const link_stats_t *APP_U2Stats(void) { return 0; }

/* 这个变量在"没有 U2"时也要存在：APP_U2CapsHas 会读它（恒为 0 → 一律不支持）*/
static link_status_t s_u2_status;
#endif

/*===== 0x000E 测量 (见 docs/07 §数据包与测试流程) =====*/
#define APP_MEAS_PEER_TIMEOUT_MS  60u   /* 等 U2 快照的上限 (115200 下 55B 往返 < 15ms) */

/* 测试推进阶段 */
typedef enum {
    APP_MEAS_ST_IDLE = 0,
    APP_MEAS_ST_WAIT_ARM_ACK,   /* 已发 ARM(本片窗口同时已开), 等对侧确认同窗 */
    APP_MEAS_ST_WAIT_STABLE,    /* 等 9 路连续稳定 (MEAS_STABLE_N 次) */
    APP_MEAS_ST_WAIT_PULSE,     /* 等脉冲捕获窗口结束 */
    APP_MEAS_ST_WAIT_PEER,      /* 已发测量快照交换请求, 等 U2 的快照 */
    APP_MEAS_ST_WAIT_PULSE_PEER /* 已发脉冲结果交换请求, 等 U2 的结果 */
} APP_MeasStage_t;

/* ARM 同步触发 (docs/13 §5.5): 窗口长度按测试类型给 —— 电压只需要几十毫秒稳定,
 * 脉冲要用整个捕获窗口。 */
#define APP_ARM_WIN_MEAS_MS       20u
#define APP_ARM_WIN_PULSE_MS      PULSE_WINDOW_MS
#define APP_MEAS_ARM_TIMEOUT_MS   50u   /* ARM 是"收到就回"的快命令 */

/* 断链恢复 (docs/13 §3.6) */
#define APP_LINK_FAIL_LIMIT       3u      /* 连续超时到这个数就复位链路 */
#define APP_LINK_RESET_TIMEOUT_MS 50u
#define APP_LINK_RESET_BACKOFF_MS 5000u   /* 复位也失败 → 退避 5s 再试, 别刷屏 */
#define APP_EV_GET_TIMEOUT_MS     50u     /* EVENT_GET 补拉的等待上限 */

static APP_MeasStage_t s_meas_stage;
static uint16_t        s_meas_seq;                       /* 受理时的帧序号 */
static uint16_t        s_test_cmd;                       /* 正在服务的命令: 0x000E / 0x000F */
static meas_snapshot_t s_snap;                           /* 本片快照 */
static meas_snapshot_t s_peer;                           /* U2 快照 */
static uint8_t         s_meas_req[MEAS_PACK_SNAP_LEN];   /* 发给 U2 的 55B 请求 */
static uint8_t         s_peer_raw[LINK_MAX_PAYLOAD];     /* U2 应答原始负载 (>=55B) */
/* 0x000E / 0x000F 共用的应答缓冲，按两者较大的开 */
#define APP_RESP_BUF_MAX  ((MEAS_PACK_RESP_LEN > PULSE_PACK_RESP_LEN) \
                           ? MEAS_PACK_RESP_LEN : PULSE_PACK_RESP_LEN)
static uint8_t         s_meas_resp[APP_RESP_BUF_MAX];
static pulse_snapshot_t s_pulse;                         /* 本片(输出端)脉冲结果 */
static pulse_snapshot_t s_pulse_peer;                    /* U2(输入端)脉冲结果 */
static uint8_t          s_pulse_peer_ok;                 /* 是否拿到 U2 的脉冲结果 */
static uint8_t          s_pulse_diff[PULSE_PH_COUNT];    /* 两端差异标记 */
static uint8_t          s_last_cc_flags;                 /* 最近一次交叉校验标志 */
static uint16_t        s_last_cc_mask;                   /* 最近一次超差掩码 */
static uint8_t         s_last_cc_peer;                   /* 最近一次是否拿到 U2 数据 */

/*===== ARM 同步触发 / 断链恢复 / 事件 (docs/13) =====*/
static uint16_t s_arm_session;          /* 每次测量 +1, 防重传把窗口重开 */
static uint8_t  s_arm_tried;            /* 本次测量是否发过 ARM */
static uint8_t  s_arm_ok;               /* 对侧是否确认"同一时间窗" */
/* 下面四个只在"有 U2"时才有人用（断链恢复是 U2 链路的机制），
 * 台架形态（JIG_U2_LINK=0）没有 U2 → 挪进守卫块，免得 ARMCC 报"声明未引用"。 */
#if (JIG_U2_LINK != 0)
static uint8_t  s_link_fail;            /* 连续请求超时次数 */
static uint32_t s_reset_t0;             /* 上次链路复位的时刻 */
static uint32_t s_reset_backoff_ms;     /* 复位失败后的退避窗口 */
static uint8_t  s_ev_fetch;             /* 有一次 EVENT_GET 补拉在进行 */
#endif

/*------------------------------------------------------------------------------
 * 几个小工具 (都在主循环上下文)
 *----------------------------------------------------------------------------*/
/* 对端是否声明支持某条能力 (docs/13 §7.2); 没收到过 STATUS 前一律按"不支持" */
static uint8_t APP_U2CapsHas(uint16_t bit)
{
    return ((s_u2_status.caps & bit) != 0u) ? 1u : 0u;
}

/* 本片开始采集: 脉冲开捕获窗口; 电压只是清稳定计数重新累计 */
static void APP_MeasStartLocal(uint16_t win)
{
    if (s_test_cmd == MGR_CMD_PULSE) {
        PULSE_CfgStart(win);
        s_meas_stage = APP_MEAS_ST_WAIT_PULSE;
    } else {
        MEAS_TestBegin();
        s_meas_stage = APP_MEAS_ST_WAIT_STABLE;
    }
}

/* U2 报上来的事件: 先打调试口; 将来要上报 PC 就在这里加 */
#if (JIG_U2_LINK != 0)
static void APP_LogU2Event(uint8_t code, uint8_t sev, uint16_t arg0)
{
#if (MEAS_PRINT_ENABLE != 0)
    BSP_DBG_Send("  [U2-EV] code=");
    BSP_DBG_SendHexByte(code);
    BSP_DBG_Send(" sev=");
    BSP_DBG_SendHexByte(sev);
    BSP_DBG_Send(" arg=");
    BSP_DBG_SendHexByte((uint8_t)(arg0 >> 8));
    BSP_DBG_SendHexByte((uint8_t)(arg0));
    BSP_DBG_Send("\r\n");
#else
    (void)code;
    (void)sev;
    (void)arg0;
#endif
}
#endif /* JIG_U2_LINK */

/* mode=1 的统一入口: **先 ARM 让两端同窗**, 再开本片窗口 (docs/13 §6.2) */
static void APP_MeasStartWithArm(uint16_t cmd, uint16_t win)
{
    uint8_t req[LINK_ARM_REQ_LEN];

    s_test_cmd = cmd;
    s_meas_seq = pc_seq;
    s_arm_tried = 0u;
    s_arm_ok    = 0u;
    s_arm_session++;
    if (s_arm_session == 0u) {
        s_arm_session = 1u;
    }

    /* 对端不支持 ARM(老固件) / 链路正忙 → 退回"各自最近一次"的老流程 */
    if ((APP_U2CapsHas(LINK_CAP_ARM) == 0u) ||
        (Link_ArmPackReq(req, (uint16_t)sizeof(req), s_arm_session, win, 0u) != LINK_ARM_REQ_LEN) ||
        (APP_U2Send(LINK_CMD_ARM, req, LINK_ARM_REQ_LEN) == 0u)) {
        APP_MeasStartLocal(win);
        app_state = APP_STATE_TESTING;
        return;
    }

    APP_MeasStartLocal(win);            /* 发完 ARM 立刻开本片窗口: 两端错位 < 2ms */
    s_arm_tried  = 1u;
    s_meas_stage = APP_MEAS_ST_WAIT_ARM_ACK;
    app_state    = APP_STATE_TESTING;
}

/*==============================================================================
 * PC 帧"半截帧"超时复位
 *----------------------------------------------------------------------------
 * 为什么需要（2026-11-05 实测踩到）：
 *   线上丢了几个字节 → 解析器把 LEN 读成一个大数（实测 0x00A5 = 165）
 *   → 它安静地等 165+10 个永远不会来的字节 → **以后每一帧都被吃掉**。
 *   现象是"PC 明明发出去了、板子也确实收到了字节，但永远没有应答"，
 *   而且看不出任何错误。这里用"攒的字节数长时间不涨"识别并重新对齐。
 *
 * 100 ms 的依据：单帧最长 256B，115200 下 22 ms 发完；留 4 倍余量。
 *============================================================================*/
#define APP_FRAME_STALL_MS   100u

static uint16_t s_stall_pos;
static uint32_t s_stall_tick;
static uint16_t s_stall_cnt;                /* 半截帧复位次数（诊断用）*/

static void APP_FrameStallCheck(void)
{
    uint16_t pos = APP_Frame_Buffered();
    uint32_t now = HAL_GetTick();

    if (pos == 0u) {                        /* 空闲：没有半截帧 */
        s_stall_pos  = 0u;
        s_stall_tick = now;
        return;
    }
    if (pos != s_stall_pos) {               /* 还在往里长 → 正常收帧中 */
        s_stall_pos  = pos;
        s_stall_tick = now;
        return;
    }
    if ((uint32_t)(now - s_stall_tick) > APP_FRAME_STALL_MS) {
        APP_Frame_Init();                   /* 半截帧超时：丢掉，从 A5 5B 重新对齐 */
        s_stall_pos  = 0u;
        s_stall_tick = now;
        if (s_stall_cnt < 0xFFFFu) {
            s_stall_cnt++;                  /* 心跳里报出来：>0 说明确实丢过字节 */
        }
    }
}

/*==============================================================================
 * 台架版"自报身份"（JIG_DUT_LINK=1 且 PC 口在 USART1）
 *----------------------------------------------------------------------------
 * 上电先打一行横幅；之后**只要还没收到过任何一帧合法 PC 帧**，每 3 秒打一行心跳，
 * 带"PC 口一共收到多少字节 / 解析成功几帧"。收到第一帧后永久闭嘴。
 *
 * 为什么必须有它（2026-11-05 加的）：这版没有调试打印（USART1 归 PC 了），
 * 板子不回话时**完全分不清**下面三种情况，而这三种的修法完全不同：
 *   ① 应用根本没在跑（BOOT0 还高、烧录没进去、启动就卡死）
 *   ② 应用在跑，但 PC 发的字节没进 MCU（接错口 / 没交叉 / 没共地）
 *   ③ 字节进来了，但没被解析成帧（固件 bug）
 * 心跳里的 rx_bytes 正好把 ② 和 ③ 分开：
 *   PC 发了命令而 **rx_bytes 不涨** → 字节没到 MCU（②，查线）
 *   **rx_bytes 涨、frames 还是 0**    → 到了但没解析成帧（③，是固件）
 * 三种之外还有一种"连心跳都收不到" = ①，或 USART1 的 TX 根本没通。
 *============================================================================*/
#if (APP_BENCH_ALIVE != 0)
static uint32_t s_tx_drop;              /* 因发送环忙被丢掉的帧数（诊断用）*/
#endif

#if (APP_BENCH_ALIVE != 0)

static uint32_t s_pc_in_bytes;          /* PC 口一共收到多少字节（含 ASCII 命令）*/
static uint32_t s_bench_frames;         /* 解析成功的 PC 帧数 */
static uint32_t s_bench_tick;           /* 上次打心跳的时刻（0 = 横幅还没发）*/
static uint32_t s_bench_rb;             /* 上次看到的 rx_bytes（看它有没有在动）*/
static uint32_t s_bench_quiet;          /* 链路最后一次"有事"的时刻 */
static uint32_t s_loop_max_us;          /* 主循环单圈最大耗时（µs，DWT 量的）*/

static uint16_t bench_u32(char *dst, uint32_t v)
{
    char    tmp[10];
    uint8_t n = 0u;
    uint8_t i;

    if (v == 0u) {
        dst[0] = '0';
        return 1u;
    }
    while ((v != 0u) && (n < (uint8_t)sizeof(tmp))) {
        tmp[n] = (char)('0' + (char)(v % 10u));
        n++;
        v /= 10u;
    }
    for (i = 0u; i < n; i++) {
        dst[i] = tmp[(uint8_t)(n - 1u - i)];
    }
    return (uint16_t)n;
}

static void APP_BenchSay(void)
{
    static const char s_boot[] =
        "\r\n[JIG] U1_DUT alive  PC=USART1(PA9/PA10)  DUT=USART3(PB10/PB11)\r\n";
    static const char s_wait[] = "[JIG] t=";
    static const char s_rb[]   = "s  rx_bytes=";
    static const char s_fr[]   = "  frames=";
    static const char s_sq[]   = "  seq=";
    static const char s_st[]   = "  stalls=";
    static const char s_bf[]   = "  buf=";
    static const char s_td[]   = "  txdrop=";
    static const char s_re[]   = "  rearm=";
    static const char s_er[]   = "  rxerr=";
    static const char s_he[]   = "  herr=0x";
    static const char s_lm[]   = "  loopmax=";
    char     buf[128];
    uint16_t n = 0u;
    uint8_t  i;
    uint32_t now = HAL_GetTick();
    const uart_stats_t *ps = UartPort_Stats(UART_CFG_PORT_PC);
    uint32_t rb = (ps != 0) ? ps->rx_bytes : 0u;

    /* ⚠ **只在链路彻底安静 3 秒时说话**（2026-11-05 再修一次）。
     *   踩过的坑：心跳和应答都往同一个发送环里塞。虽然 APP_WaitTxDone() 会等，
     *   但"诊断文本"和"协议帧"抢同一个环始终是隐患 —— 实测表现就是
     *   "第一次 status 超时、重试那次才通"。现在只要链路上还有任何动静
     *   （rx 计数变了 / 解析器里有半截帧 / 有帧待处理 / 发送环非空），
     *   心跳一律闭嘴并把静默起点往后推。 */
    if (rb != s_bench_rb) {
        s_bench_rb    = rb;
        s_bench_quiet = now;
    }
    if ((APP_Frame_Buffered() != 0u) || (APP_Frame_Pending() != 0u) ||
        (PcLink_TxIdle() == 0u)) {
        s_bench_quiet = now;                     /* 链路上还有事，别插嘴 */
        return;
    }
    if ((uint32_t)(now - s_bench_quiet) < 3000u) {
        return;
    }
    if (s_bench_tick == 0u) {
        /* 第一拍：横幅，只发一次。
         * ⚠ **绝不能用阻塞的 HAL_UART_Transmit**：它和"RX 重挂"共用 HAL 的
         *   `huart->Lock`，阻塞期间（一行文字约 3ms）到达的字节会让 ISR 里的
         *   `HAL_UART_Receive_IT` 直接返回 HAL_BUSY → 那几字节**永久丢掉**。
         *   实测（2026-11-05）：PC 发 10 字节的 0x0017，板子只收到前 7 字节，
         *   帧永远凑不齐（心跳读数是 rx_bytes=7 frames=0）。
         *   正确做法：走和普通应答同一个发送环（uart_tx_kick 里那对
         *   `__disable_irq/__enable_irq` 保证了 HAL 调用与 ISR 不会互相插队）。 */
        (void)UartPort_Write(UART_CFG_PORT_PC, (const uint8_t *)s_boot,
                             (uint16_t)(sizeof(s_boot) - 1u));
        s_bench_tick = (now == 0u) ? 1u : now;
        return;
    }
    if ((uint32_t)(now - s_bench_tick) < 3000u) {
        return;
    }
    /* 有人开始跟它说话了（发过帧命令、或串口助手里敲过 AS: 命令）→ **永久闭嘴**。
     * 心跳的用处只有"上电后证明它还活着"；一旦进入交互，它就是在刷屏、
     * 还会和应答抢同一个发送环（2026-11-10 用户要求关掉）。
     * 横幅不受影响：它在上面那个 `s_bench_tick == 0` 分支里，开机只打一次。*/
    if (s_pc_in_bytes != 0u) {
        return;
    }
    s_bench_tick = now;

    for (i = 0u; i < (uint8_t)(sizeof(s_wait) - 1u); i++) { buf[n++] = s_wait[i]; }
    n = (uint16_t)(n + bench_u32(&buf[n], now / 1000u));
    for (i = 0u; i < (uint8_t)(sizeof(s_rb) - 1u); i++) { buf[n++] = s_rb[i]; }
    n = (uint16_t)(n + bench_u32(&buf[n], (ps != 0) ? ps->rx_bytes : 0u));
    for (i = 0u; i < (uint8_t)(sizeof(s_fr) - 1u); i++) { buf[n++] = s_fr[i]; }
    n = (uint16_t)(n + bench_u32(&buf[n], s_bench_frames));
    for (i = 0u; i < (uint8_t)(sizeof(s_sq) - 1u); i++) { buf[n++] = s_sq[i]; }
    n = (uint16_t)(n + bench_u32(&buf[n], (uint32_t)pc_seq));
    for (i = 0u; i < (uint8_t)(sizeof(s_st) - 1u); i++) { buf[n++] = s_st[i]; }
    n = (uint16_t)(n + bench_u32(&buf[n], (uint32_t)s_stall_cnt));
    for (i = 0u; i < (uint8_t)(sizeof(s_bf) - 1u); i++) { buf[n++] = s_bf[i]; }
    n = (uint16_t)(n + bench_u32(&buf[n], (uint32_t)APP_Frame_Buffered()));
    for (i = 0u; i < (uint8_t)(sizeof(s_td) - 1u); i++) { buf[n++] = s_td[i]; }
    n = (uint16_t)(n + bench_u32(&buf[n], s_tx_drop));
    for (i = 0u; i < (uint8_t)(sizeof(s_re) - 1u); i++) { buf[n++] = s_re[i]; }
    n = (uint16_t)(n + bench_u32(&buf[n], (ps != 0) ? ps->rx_rearm : 0u));
    for (i = 0u; i < (uint8_t)(sizeof(s_er) - 1u); i++) { buf[n++] = s_er[i]; }
    n = (uint16_t)(n + bench_u32(&buf[n], (ps != 0) ? ps->rx_err : 0u));
    for (i = 0u; i < (uint8_t)(sizeof(s_he) - 1u); i++) { buf[n++] = s_he[i]; }
    {
        static const char hex[] = "0123456789ABCDEF";
        extern volatile uint32_t g_uart1_err_or;
        uint32_t ec = g_uart1_err_or & 0xFFu;   /* 只增不清的错误码位图 */

        buf[n++] = hex[(ec >> 4) & 0x0Fu];
        buf[n++] = hex[ec & 0x0Fu];
    }
    for (i = 0u; i < (uint8_t)(sizeof(s_lm) - 1u); i++) { buf[n++] = s_lm[i]; }
    n = (uint16_t)(n + bench_u32(&buf[n], s_loop_max_us));
    buf[n++] = '\r';
    buf[n++] = '\n';
    (void)UartPort_Write(UART_CFG_PORT_PC, (const uint8_t *)buf, n);
}
#endif

/*==============================================================================
 * 帧发送
 *============================================================================*/
static void APP_WaitTxDone(void)
{
    uint32_t tick = HAL_GetTick();

    /* 等"上一帧已经送到线上"。USB 分支看 TxState，串口分支看发送队列是否排空，
     * 差异都收在 pc_link.c 里；这里的语义与原来完全相同：等不到就放弃本帧。 */
    while (PcLink_TxIdle() == 0u) {
        if ((HAL_GetTick() - tick) > APP_TX_BUSY_TIMEOUT_MS) break;
    }
}

static void APP_SendFrame(uint16_t cmd, uint16_t seq, const uint8_t *data, uint16_t len)
{
    uint16_t total;

    /* ⚠ 必须**先等上一次发完**，再往 tx_buf 里组帧：
     *    USB 分支里超过 64B 的帧（0x000E 的 77B / 0x000F 的 91B 应答）由 USB
     *    中断继续从 tx_buf 取后续包；上一帧还在飞就覆写 → 主机会收到混叠/截断
     *    的帧。等不到（链路卡住）就放弃这一帧：宁可让 PC 超时重试，也不发坏帧。 */
    APP_WaitTxDone();
    if (PcLink_TxIdle() == 0u) {
#if (APP_BENCH_ALIVE != 0)
        s_tx_drop++;            /* 心跳里报出来：>0 = 应答被丢弃过 */
#endif
        return;
    }

#if (JIG_DUT_LINK != 0)
    /* 记下这一条应答（供 0x0010 采集每步结果）；主动事件 seq=0 不算 */
    if (seq != 0u) {
        s_last_ack_st  = ((data != 0) && (len > 0u)) ? data[0] : 0xFFu;
    }
    /* 单项流程进行中：子命令的应答**不外发**，只留一份给最后那条报告 */
    if ((seq != 0u) && (s_flow_active != 0u)) {
        s_flow_res     = s_last_ack_st;
        s_flow_cap_len = 0u;
        if ((data != 0) && (len > 0u) && (len <= APP_FLOW_REP_MAX)) {
            memcpy(s_flow_cap, data, len);
            s_flow_cap_len = len;
        }
        return;
    }
#endif

    total = Frame_Build(tx_buf, cmd, seq, data, len);

    PcLink_Send(tx_buf, total);
    APP_WaitTxDone();
}

/* ACK: 帧序号回显请求序号 */
void APP_SendAck(uint16_t cmd, const uint8_t *data, uint16_t len)
{
    APP_SendFrame(cmd, pc_seq, data, len);
}

/* 治具主动推送: 帧序号恒为 0 */
void APP_SendEvent(uint16_t cmd, const uint8_t *data, uint16_t len)
{
    APP_SendFrame(cmd, 0u, data, len);
}

/*==============================================================================
 * 0x000E 测量
 *----------------------------------------------------------------------------
 * 请求 DATA = 1 字节 mode:
 *   0x00 只读：立刻把"最近一批"数据回给 PC（bring-up 调试时一直看电压用），
 *             不触发测试流程、不影响状态机。
 *   0x01 测试：非阻塞跑一次完整测试（等 9 路连续稳定 → 与 U2 交叉校验 →
 *             汇总判定），跑完才回 ACK —— 与 0x0112 的"终态回 ACK"规则一致。
 * 应答 DATA = 77 字节（布局见 meas_pack.h）:
 *   [0]      命令状态 (0x00 OK / 0x05 ERR_TEST / 0x0C ERR_TIMEOUT)
 *   [1..55]  本片快照 55B
 *   [56..76] 交叉校验块 21B (标志 + 超差掩码 + U2 的 9 路 mv)
 *============================================================================*/
static uint16_t APP_MeasBuildResp(uint8_t cmd_status, uint8_t cc_flags,
                                  uint16_t mask, uint8_t with_peer)
{
    uint16_t n = 0u;
    uint16_t left;

    s_meas_resp[n++] = cmd_status;

    left = (uint16_t)(MEAS_PACK_RESP_LEN - n);
    if (MeasPack_Snapshot(&s_meas_resp[n], left, &s_snap) != MEAS_PACK_SNAP_LEN) {
        return 0u;
    }
    n = (uint16_t)(n + MEAS_PACK_SNAP_LEN);

    left = (uint16_t)(MEAS_PACK_RESP_LEN - n);
    if (MeasPack_CcBlock(&s_meas_resp[n], left, cc_flags, mask,
                         (with_peer != 0u) ? &s_peer : 0) != MEAS_PACK_CC_LEN) {
        return 0u;
    }
    n = (uint16_t)(n + MEAS_PACK_CC_LEN);

    return (n == MEAS_PACK_RESP_LEN) ? n : 0u;
}

/*==============================================================================
 * 0x000F 脉冲捕获（与 0x000E 完全独立的一条命令）
 *----------------------------------------------------------------------------
 * 请求 DATA = 1 字节 mode（语义与 0x000E 一样）:
 *   0x00 只读：立刻回"最近一个窗口"的结果，不触发捕获。
 *   0x01 测试：开一个 200ms 捕获窗口 → 结算本片三相 → 与 U2 交换（PULSE_XCHK）
 *             → 汇总判定，跑完才回 ACK。
 * 应答 DATA = 91 字节:
 *   [0]      命令状态 (0x00 OK / 0x05 ERR_TEST / 0x0B ERR_PARAM)
 *   [1..45]  本片(三相输出端)脉冲结果 45B
 *   [46..90] 对侧 U2(三相驱动输入端)脉冲结果 45B（拿不到则按"无数据"打包）
 *============================================================================*/
/* 对侧( U2 )只回一个结论字节：0 = 没有对侧数据 · 1 = 一致 · 2 = 不一致 */
static uint8_t APP_PulsePeerByte(void)
{
    uint8_t i;

    if (s_pulse_peer_ok == 0u) {
        return 0u;
    }
    for (i = 0u; i < (uint8_t)PULSE_PH_COUNT; i++) {
        if (s_pulse_diff[i] != 0u) {
            return 2u;
        }
    }
    return 1u;
}

/* 0x000F / 0x0019 的应答：状态(1) + 对侧(1) + 三相×(周期ns 4 + 占空比 2 + 旗标 1) */
static uint16_t APP_PulseBuildResp(uint8_t cmd_status)
{
    uint16_t n = 0u;
    uint8_t  i;

    s_meas_resp[n++] = cmd_status;
    s_meas_resp[n++] = APP_PulsePeerByte();

    for (i = 0u; i < (uint8_t)PULSE_PH_COUNT; i++) {
        uint32_t per  = s_pulse.ph[i].period_ns;
        uint16_t duty = s_pulse.ph[i].duty_permille;

        s_meas_resp[n++] = (uint8_t)(per >> 24);
        s_meas_resp[n++] = (uint8_t)(per >> 16);
        s_meas_resp[n++] = (uint8_t)(per >> 8);
        s_meas_resp[n++] = (uint8_t)(per);
        s_meas_resp[n++] = (uint8_t)(duty >> 8);
        s_meas_resp[n++] = (uint8_t)(duty);
        s_meas_resp[n++] = (uint8_t)(s_pulse.ph[i].flags & 0xFFu);
    }
    return (n == (uint16_t)PULSE_PACK_RESP_LEN) ? n : 0u;
}

/* 0x000F 的判定（默认 PULSE_JUDGE_ENABLE=0 → 只记录不判） */
static uint8_t APP_PulseVerdict(void)
{
#if (PULSE_JUDGE_ENABLE != 0)
    {
        uint8_t pi;
        for (pi = 0u; pi < (uint8_t)PULSE_PH_COUNT; pi++) {
            if (s_pulse_diff[pi] != 0u) {
                return APP_STATUS_ERR_TEST;      /* 两端对不上 */
            }
            if ((s_pulse.ph[pi].flags & PULSE_F_VALID) == 0u) {
                return APP_STATUS_ERR_TEST;      /* 没测到波形 */
            }
        }
    }
#endif
    return APP_STATUS_OK;
}

/* 判定：先看本片 9 路，再看与 U2 的一致性 */
static uint8_t APP_MeasVerdict(uint8_t cc_flags, uint16_t mask)
{
    (void)cc_flags;      /* MEAS_CC_REQUIRED=0 时用不到; 形参保留以便开关切换 */

    if (MEAS_AllOk() == 0u) {
        return APP_STATUS_ERR_TEST;              /* 本片有通道不达标 */
    }
    if (mask != 0u) {
        return APP_STATUS_ERR_TEST;              /* 两片读数不一致 → 治具自身可疑 */
    }
#if (MEAS_CC_REQUIRED != 0)
    if ((cc_flags & MEAS_CC_F_NO_PEER) != 0u) {
        return APP_STATUS_ERR_TEST;              /* 产线模式: U2 不在就不放行 */
    }
#endif
    return APP_STATUS_OK;
}

/* 收尾：发唯一一次 ACK、清干净状态。
 * 同一个收尾函数服务两条命令：应答内容按 s_test_cmd 走不同的打包函数。 */
static void APP_MeasFinish(uint8_t cmd_status, uint8_t cc_flags, uint16_t mask,
                           uint8_t with_peer)
{
    uint16_t n;

    s_last_cc_flags = cc_flags;
    s_last_cc_mask  = mask;
    s_last_cc_peer  = with_peer;

    if (s_test_cmd == MGR_CMD_PULSE) {
        n = APP_PulseBuildResp(cmd_status);
    } else {
        n = APP_MeasBuildResp(cmd_status, cc_flags, mask, with_peer);
    }

    if (n == 0u) {                               /* 理论上不会发生: 编解码长度不符 */
        uint8_t e = APP_STATUS_ERR_PARAM;
        APP_SendFrame(s_test_cmd, s_meas_seq, &e, 1u);
    } else {
        APP_SendFrame(s_test_cmd, s_meas_seq, s_meas_resp, n);
    }

    s_meas_stage = APP_MEAS_ST_IDLE;
    app_state    = APP_STATE_IDLE;
    /* 测试期间 PC 若仍在发帧, 半截帧会留在解析器里 —— 和 0x0112 一样清掉 */
    APP_Frame_Init();
}

/* 每轮主循环推进一步 (非阻塞, 与 SWDP_Step 同一个套路) */
static void APP_MeasStep(void)
{
    uint8_t  st;
    uint16_t peer_len = 0u;

    switch (s_meas_stage) {

    /* ── 等对侧确认"同一时间窗"(ARM 应答)。本片窗口在这一刻已经在跑了 ── */
    case APP_MEAS_ST_WAIT_ARM_ACK: {
        uint8_t  resp[LINK_MAX_PAYLOAD];
        uint16_t rlen = 0u;

        st = APP_U2Poll(APP_MEAS_ARM_TIMEOUT_MS, resp, &rlen);
        if (st == 0u) {
            return;                              /* 还在等 ARM 应答 */
        }
        s_arm_ok = ((st == 1u) && (Link_ArmCheckResp(resp, rlen, s_arm_session) != 0u)) ? 1u : 0u;
        /* 注意: 即使 ARM 没成功也不打断本片的采集 —— 让本片结果照样有效,
         * 只是后面**不做交叉校验**(两端不同窗的数据比对没有意义)。 */
        s_meas_stage = (s_test_cmd == MGR_CMD_PULSE) ? APP_MEAS_ST_WAIT_PULSE
                                                     : APP_MEAS_ST_WAIT_STABLE;
        return;
    }

    case APP_MEAS_ST_WAIT_STABLE:
        st = MEAS_TestPoll();
        if (st == 0u) {
            return;                              /* 还在等数据稳定 */
        }
        if (st == 2u) {                          /* 超时: 数据一直不稳 */
            MEAS_Snapshot(&s_snap);
            APP_MeasFinish(APP_STATUS_ERR_TIMEOUT, 0u, 0u, 0u);
            return;
        }

        MEAS_Snapshot(&s_snap);
        if (MeasPack_Snapshot(s_meas_req, (uint16_t)sizeof(s_meas_req), &s_snap)
                != MEAS_PACK_SNAP_LEN) {
            APP_MeasFinish(APP_STATUS_ERR_PARAM, 0u, 0u, 0u);
            return;
        }
        /* ARM 试过但没对上窗 → 两端不是同一段时间, 交叉校验没意义 */
        if ((s_arm_tried != 0u) && (s_arm_ok == 0u)) {
            APP_MeasFinish(APP_MeasVerdict(MEAS_CC_F_NO_PEER, 0u), MEAS_CC_F_NO_PEER, 0u, 0u);
            return;
        }
        /* 把本片快照发给 U2, 换回它的快照 (一条链路命令完成交换+可比对) */
        if (APP_U2Send(LINK_CMD_MEAS_XCHK,
                       s_meas_req, MEAS_PACK_SNAP_LEN) == 0u) {
            APP_MeasFinish(APP_MeasVerdict(MEAS_CC_F_NO_PEER, 0u),
                           MEAS_CC_F_NO_PEER, 0u, 0u);
            return;
        }
        s_meas_stage = APP_MEAS_ST_WAIT_PEER;
        return;

    case APP_MEAS_ST_WAIT_PEER:
        st = APP_U2Poll(APP_MEAS_PEER_TIMEOUT_MS,
                        s_peer_raw, &peer_len);
        if (st == 0u) {
            return;                              /* 还在等 U2 */
        }
        if ((st == 2u) || (peer_len != MEAS_PACK_SNAP_LEN) ||
            (MeasPack_UnpackSnapshot(s_peer_raw, peer_len, &s_peer) == 0u)) {
            APP_MeasFinish(APP_MeasVerdict(MEAS_CC_F_NO_PEER, 0u),
                           MEAS_CC_F_NO_PEER, 0u, 0u);
            return;
        }
        {
            uint16_t mask = MEAS_Compare(&s_snap, &s_peer, 0);
            uint8_t  cc   = (uint8_t)(MEAS_CC_F_DONE |
                                      ((mask != 0u) ? MEAS_CC_F_MISMATCH : 0u));
            APP_MeasFinish(APP_MeasVerdict(cc, mask), cc, mask, 1u);
        }
        return;

    /* ── 0x000F 脉冲捕获的两段（与上面的 ADC 流程完全独立） ── */
    case APP_MEAS_ST_WAIT_PULSE:
        if (PULSE_CfgPoll() == 0u) {
            return;                              /* 200ms 窗口还没到 */
        }
        PULSE_Snapshot(&s_pulse);
        s_pulse_peer_ok = 0u;
        memset(s_pulse_diff, 0, sizeof(s_pulse_diff));

        /* 请求 = 本片(输出端) 45B，应答 = U2(输入端) 45B */
        {
            uint8_t preq[PULSE_PACK_LEN];
            if (PulsePack_Pack(preq, (uint16_t)sizeof(preq), &s_pulse) != PULSE_PACK_LEN) {
                APP_MeasFinish(APP_STATUS_ERR_PARAM, 0u, 0u, 0u);
                return;
            }
            /* ARM 试过但没对上窗 → 两端不是同一段时间, 不做交叉校验 */
            if ((s_arm_tried != 0u) && (s_arm_ok == 0u)) {
                APP_MeasFinish(APP_PulseVerdict(), 0u, 0u, 0u);
                return;
            }
            if (APP_U2Send(LINK_CMD_PULSE_XCHK,
                           preq, PULSE_PACK_LEN) == 0u) {
                APP_MeasFinish(APP_PulseVerdict(), 0u, 0u, 0u);
                return;
            }
        }
        s_meas_stage = APP_MEAS_ST_WAIT_PULSE_PEER;
        return;

    case APP_MEAS_ST_WAIT_PULSE_PEER:
        st = APP_U2Poll(APP_MEAS_PEER_TIMEOUT_MS,
                        s_peer_raw, &peer_len);
        if (st == 0u) {
            return;                              /* 还在等 U2 的脉冲结果 */
        }
        if ((st == 1u) && (peer_len == PULSE_PACK_LEN) &&
            (PulsePack_Unpack(s_peer_raw, peer_len, &s_pulse_peer) != 0u)) {
            s_pulse_peer_ok = 1u;
            (void)PulsePack_Compare(&s_pulse, &s_pulse_peer, s_pulse_diff);
        }
        APP_MeasFinish(APP_PulseVerdict(), 0u, 0u, 0u);
        return;

    default:
        s_meas_stage = APP_MEAS_ST_IDLE;
        app_state    = APP_STATE_IDLE;
        return;
    }
}

/* 0x000F 脉冲捕获命令入口 */
static void APP_PulseCmd(uint16_t cmd, const uint8_t *data, uint16_t len)
{
    uint8_t mode = (len >= 1u) ? data[0] : 0xFFu;

    if (mode == MGR_MODE_READ) {
        /* 只读：立刻回最近一个窗口的结果（不触发捕获，不占 200ms） */
        uint16_t rn = APP_PulseBuildResp(APP_STATUS_OK);

        if (rn == 0u) {
            uint8_t e = APP_STATUS_ERR_PARAM;
            APP_SendAck(cmd, &e, 1u);
            return;
        }
        APP_SendAck(cmd, s_meas_resp, rn);
        return;
    }

    if (mode == MGR_MODE_TEST) {
        APP_MeasStartWithArm(cmd, APP_ARM_WIN_PULSE_MS);
        return;                                  /* ACK 由 APP_MeasStep 发 */
    }

    {
        uint8_t e = APP_STATUS_ERR_PARAM;        /* mode 非法 */
        APP_SendAck(cmd, &e, 1u);
    }
}

/* 0x000E 命令入口 (在 APP_Dispatch 里调用) —— 只管 ADC 测量 */
static void APP_MeasCmd(uint16_t cmd, const uint8_t *data, uint16_t len)
{
    uint8_t mode = (len >= 1u) ? data[0] : 0xFFu;

    if (mode == MGR_MODE_READ) {
        /* 只读: 不碰测试流程, 立刻回"最近一批" + 最近一次交叉校验结果 */
        uint16_t rn;

        MEAS_Snapshot(&s_snap);
        rn = APP_MeasBuildResp(APP_STATUS_OK, s_last_cc_flags, s_last_cc_mask,
                               s_last_cc_peer);
        if (rn == 0u) {
            uint8_t e = APP_STATUS_ERR_PARAM;
            APP_SendAck(cmd, &e, 1u);
            return;
        }
        APP_SendAck(cmd, s_meas_resp, rn);       /* ⚠ 用实际长度, 不能写死 */
        return;
    }

    if (mode == MGR_MODE_TEST) {
        APP_MeasStartWithArm(cmd, APP_ARM_WIN_MEAS_MS);
        return;                                  /* ACK 由 APP_MeasStep 发 */
    }

    {
        uint8_t e = APP_STATUS_ERR_PARAM;        /* mode 非法 */
        APP_SendAck(cmd, &e, 1u);
    }
}

/*==============================================================================
 * SPI Flash 槽头部操作
 *============================================================================*/
/**
  * @brief 读取某槽头部
  * @param slot   0=Bootloader, 1=App
  * @param p_size [out, 可空] 文件大小
  * @param ver    [out, 可空] 3B 版本号
  * @param p_flag [out, 可空] 完整标志
  * @retval 1 = 该槽存在完整固件, 0 = 否则
  */
uint8_t Flash_ReadHead(uint8_t slot, uint32_t *p_size, uint8_t ver[3], uint8_t *p_flag)
{
    uint8_t  hdr[FW_HEAD_SIZE];
    uint32_t base = (slot == 0u) ? FW_SLOT0_BASE : FW_SLOT1_BASE;

    /* SPI 总线读失败 → 返回 2（与"空槽/不完整"区分开）。
     * 绝不能当空槽处理：那会把"Flash 坏了/没焊"伪装成"还没上传固件"，
     * 真实案例见 gd25q128.c 顶部：0x000C 卡 24 秒后回了一个误导性的 06。 */
    if (!GD25Q_ReadBuffer(hdr, base, FW_HEAD_SIZE)) {
        if (p_size) *p_size = 0u;
        if (p_flag) *p_flag = 0u;
        return 2u;
    }

    uint32_t size = (uint32_t)hdr[FW_HEAD_OFF_SIZE]
                  | ((uint32_t)hdr[FW_HEAD_OFF_SIZE + 1u] << 8)
                  | ((uint32_t)hdr[FW_HEAD_OFF_SIZE + 2u] << 16)
                  | ((uint32_t)hdr[FW_HEAD_OFF_SIZE + 3u] << 24);
    uint8_t  flag = hdr[FW_HEAD_OFF_FLAG];

    if (p_size) *p_size = size;
    if (ver)    { ver[0] = hdr[FW_HEAD_OFF_VER];
                  ver[1] = hdr[FW_HEAD_OFF_VER + 1u];
                  ver[2] = hdr[FW_HEAD_OFF_VER + 2u]; }
    if (p_flag) *p_flag = flag;

    return (size > 0u && size <= FW_MAX_IMAGE_SIZE && flag == FW_FLAG_COMPLETE) ? 1u : 0u;
}

void Flash_WriteCompleteFlag(uint8_t slot, uint8_t flag)
{
    uint32_t base = (slot == 0u) ? FW_SLOT0_BASE : FW_SLOT1_BASE;
    (void)GD25Q_WritePage(&flag, base + FW_HEAD_OFF_FLAG, 1u);
}

/*==============================================================================
 * 0x000C 查询固件版本号 (Flash 中)
 * 优先级: App 槽(Slot1) → Bootloader 槽(Slot0)
 *============================================================================*/
static uint8_t APP_QueryFlashVersion(uint8_t ver[3])
{
    uint8_t incomplete_seen = 0u;

    for (int8_t slot = 1; slot >= 0; slot--) {
        uint32_t size = 0u;
        uint8_t  flag = 0u;
        uint8_t  v[3] = {0};
        uint8_t  r;

        r = Flash_ReadHead((uint8_t)slot, &size, v, &flag);
        if (r == 2u) {
            /* SPI 总线读不到: 这才是真实故障, 必须报 ERR_FLASH 而不是"空槽" */
            return APP_STATUS_ERR_FLASH;
        }
        if (size == 0u || size > FW_MAX_IMAGE_SIZE) continue;    /* 空槽 */
        if (flag == FW_FLAG_COMPLETE) {
            ver[0] = v[0]; ver[1] = v[1]; ver[2] = v[2];
            return APP_STATUS_OK;
        }
        incomplete_seen = 1u;                                     /* 有固件但不完整 */
    }

    ver[0] = 0x00u; ver[1] = 0xFFu; ver[2] = 0xFFu;
    return incomplete_seen ? APP_STATUS_ERR_INCOMPLETE : APP_STATUS_ERR_NO_FW;
}

/*==============================================================================
 * 0x0112 前置检查: 两槽都必须是完整固件
 *============================================================================*/
static uint8_t APP_ProgramPreCheck(void)
{
    /* 各槽镜像对应的目标分区上限: 超出会写坏另一段, 必须在擦除被烧板之前就拒绝 */
    static const uint32_t max_size[2] = { DUT_BOOT_SIZE, DUT_APP_SIZE };
    uint8_t incomplete = 0u;

    for (uint8_t slot = 0u; slot < 2u; slot++) {
        uint32_t size = 0u;
        uint8_t  flag = 0u;
        uint8_t  r;

        r = Flash_ReadHead(slot, &size, 0, &flag);
        if (r == 2u) return APP_STATUS_ERR_FLASH;   /* SPI 总线读不到 */

        if (size == 0u || size > FW_MAX_IMAGE_SIZE) return APP_STATUS_ERR_NO_FW;
        if (size > max_size[slot])                return APP_STATUS_ERR_SIZE;
        if (flag != FW_FLAG_COMPLETE) incomplete = 1u;
    }

    return incomplete ? APP_STATUS_ERR_INCOMPLETE : APP_STATUS_OK;
}

/*==============================================================================
 * DUT 链路（被烧 FOC 板）— 0x0011 透传
 *============================================================================*/
#if (JIG_DUT_LINK != 0)
/*===== DUT 口原始字节嗅探（0x001F / 0x0020 用）=====
 * 为什么需要：DUT 链路层（Link_Poll）在空闲时会把口上的字节读走当帧解析，
 * **不是合法帧的直接丢掉**。可现在对面的固件还没有 0x02xx 协议 —— 它吐出来的
 * 可能是任意 ASCII（例如 AS: 命令的回报），必须先原样留下来给人看。
 * 所以空闲时改成"只嗅探、不解析"：字节进 s_dut_sniff，由 0x0020 取走。
 * （链路层只在"我们确实在等一帧应答"时才读口，那时 Link_Busy 为 1，走原路。） */
/* DUT 口原始字节日志（诊断用，0x0026 读）：只保留"上一条命令之后"收到的内容 */
static uint8_t  s_dut_log[MGR_DUT_LOG_SIZE];
static uint16_t s_dut_log_len;

static uint8_t  s_dut_sniff[MGR_DUT_SNIFF_SIZE];
static uint16_t s_dut_sniff_len;
static uint16_t s_dut_sniff_drop;

/* 0x001F 的收发窗口状态 */
static uint16_t s_dutsnd_seq;
static uint16_t s_dutsnd_sent;
static uint16_t s_dutsnd_tmo;
static uint32_t s_dutsnd_t0;

static void APP_DutSniff(void)
{
    uint8_t b;

    while (UartPort_Read(UART_CFG_PORT_DUT, &b, 1u) == 1u) {
        if (s_dut_log_len < (uint16_t)MGR_DUT_LOG_SIZE) {
            s_dut_log[s_dut_log_len] = b;        /* 原样留一份给 0x0026 */
            s_dut_log_len++;
        } else {
            /* 满了就左移一格（滚动缓冲）：永远保留"最近"的 96 字节。
             * ⚠ 不能靠"发命令时清空"来限制长度 —— 那样收尾命令会把现场擦掉
             *   （2026-11-12 实测：想看的步3 证据被 AS:S0 清没了）。*/
            memmove(&s_dut_log[0], &s_dut_log[1], (size_t)(MGR_DUT_LOG_SIZE - 1u));
            s_dut_log[MGR_DUT_LOG_SIZE - 1u] = b;
        }
        if (s_dut_sniff_len < (uint16_t)MGR_DUT_SNIFF_SIZE) {
            s_dut_sniff[s_dut_sniff_len] = b;
            s_dut_sniff_len++;
        } else if (s_dut_sniff_drop < 0xFFFFu) {
            s_dut_sniff_drop++;
        }
    }
}

/**
  * @brief  0x0011 DUT 透传：请求 DATA = dut_cmd(2) + dut_data(N)
  *
  * **这条命令是上板 bring-up 的关键**：PC 可以直接发任意一条 DUT 命令并看原样应答，
  * 不必等 ③④⑤⑥ 的编排做完就能把 DUT 链路单独调通（和现有 `raw` 一个思路）。
  *
  * 非阻塞：这里只发起，终态在 APP_DutTask 里回 ACK（沿用 0x0112 / 0x000E 的做法）。
  */
static void APP_DutCmd(uint16_t cmd, const uint8_t *data, uint16_t len)
{
    uint16_t dcmd;
    uint8_t  e;

    if (len < 2u) {                         /* 至少要 2 字节命令码 */
        e = APP_STATUS_ERR_PARAM;
        APP_SendAck(cmd, &e, 1u);
        return;
    }
    dcmd = (uint16_t)(((uint16_t)data[0] << 8) | (uint16_t)data[1]);

    /* 只放行 DUT 命令空间（0x0200~0x02FF）：
     * 防止 PC 误把别的码透传过去，让被测板收到一条它没预期的命令 */
    if (dcmd < DUT_CMD_LO || dcmd > DUT_CMD_HI) {
        e = APP_STATUS_ERR_PARAM;
        APP_SendAck(cmd, &e, 1u);
        return;
    }
    if (Link_RequestStart(&s_dut_link, dcmd, &data[2], (uint16_t)(len - 2u)) == 0u) {
        e = APP_STATUS_BUSY;                /* 上一发没收尾 / 发送队列满 */
        APP_SendAck(cmd, &e, 1u);
        return;
    }
    s_dut_seq = pc_seq;
    app_state = APP_STATE_DUT;              /* 终态由 APP_DutTask 回 ACK */
}

/**
  * @brief  DUT 链路：每个主循环推进一步
  * @note   空闲时只做 Link_Poll —— 把 DUT 上电/复位时吐出来的杂字节排掉，
  *         否则端口 0 的环形缓冲会满，后面真正的应答会被丢掉。
  */
static void APP_DutTask(void)
{
    uint16_t rn = 0u;
    uint8_t  st;

    if (app_state == APP_STATE_DUT_RAW) {
        return;                             /* 原始收发期间不许链路层碰这个口 */
    }
    if (app_state != APP_STATE_DUT) {
        if (Link_Busy(&s_dut_link) == 0u) {
            /* 空闲：**只嗅探、不当帧解析** —— 对面现在还没实现 0x02xx，
             * 它吐的字节（可能是 ASCII）要能被人看见，不能被链路层吃掉。 */
            APP_DutSniff();
        }
        return;
    }

    st = Link_RequestPoll(&s_dut_link, APP_DUT_TIMEOUT_MS, s_dut_resp, &rn);
    if (st == 1u) {
        /* ACK = jig 状态(1) + DUT 应答的 DATA 段
         * （DUT 自己的状态码就在它 DATA 的第 0 字节，PC 直接看得到） */
        s_dut_ack[0] = APP_STATUS_OK;
        if (rn > 0u) {
            memcpy(&s_dut_ack[1], s_dut_resp, rn);
        }
        app_state = APP_STATE_IDLE;
        APP_SendFrame(MGR_CMD_DUT_RAW, s_dut_seq, s_dut_ack, (uint16_t)(1u + rn));
    } else if (st == 2u) {
        uint8_t e = APP_STATUS_ERR_TIMEOUT;
        app_state = APP_STATE_IDLE;
        APP_SendFrame(MGR_CMD_DUT_RAW, s_dut_seq, &e, 1u);
    } else {
        /* 0 = 还在等 */
    }
}

/**
  * @brief  0x001F 给被测板发一段**原始字节**，并抓这段时间它回了什么
  * @param  data/len 请求 DATA = 等回应时长 ms(2, 大端, 0=默认 200) + 原始字节(N)
  * @note   走**裸串口**，不套 A5 5B 帧 —— 专门用来调对面还没实现 0x02xx 的固件
  *         （典型：它的 ASCII 命令 `AS:S800` 转、`AS:S0` 停）。
  *         应答 = 状态(1) + 实际发出字节数(2) + 收到字节数(2) + 收到的字节(≤64)。
  *         **"发出去了但对方一个字节都没回"也是 00** —— 那本身就是结论：
  *         线通了（能发），对面不回而已。
  *         非阻塞：发完等 tmo 毫秒，到点由 APP_DutRawTask 回 ACK。
  */
static void APP_DutSendCmd(uint16_t cmd, const uint8_t *data, uint16_t len)
{
    uint16_t tmo;
    uint16_t plen;
    uint16_t n;
    uint8_t  e;

    if ((data == 0) || (len <= (uint16_t)MGR_DUT_SEND_HDR_LEN)) {
        e = APP_STATUS_ERR_PARAM;           /* 至少要有 1 个字节要发 */
        APP_SendAck(cmd, &e, 1u);
        return;
    }
    plen = (uint16_t)(len - (uint16_t)MGR_DUT_SEND_HDR_LEN);
    if (plen > (uint16_t)MGR_DUT_SEND_MAX) {
        e = APP_STATUS_ERR_PARAM;
        APP_SendAck(cmd, &e, 1u);
        return;
    }
    tmo = (uint16_t)(((uint16_t)data[0] << 8) | data[1]);
    if (tmo == 0u) {
        tmo = MGR_DUT_SEND_DEF_TMO_MS;
    } else if (tmo > (uint16_t)MGR_DUT_SEND_MAX_TMO_MS) {
        tmo = MGR_DUT_SEND_MAX_TMO_MS;      /* 夹住上限，别被 PC 传的大数拖死 */
    }

    /* 先把旧字节清掉：否则回来的字节里混着上一次的，分不清是哪一次的回字 */
    s_dut_sniff_len  = 0u;
    s_dut_sniff_drop = 0u;
    APP_DutSniff();                         /* 读空硬件环里的陈字节 */

    n = UartPort_Write(UART_CFG_PORT_DUT, &data[MGR_DUT_SEND_HDR_LEN], plen);
    if (n != plen) {
        e = APP_STATUS_BUSY;                /* 发送环满 = 上一条还没发完 */
        APP_SendAck(cmd, &e, 1u);
        return;
    }
    s_dutsnd_seq  = pc_seq;
    s_dutsnd_sent = n;
    s_dutsnd_tmo  = tmo;
    s_dutsnd_t0   = HAL_GetTick();
    app_state     = APP_STATE_DUT_RAW;      /* 终态由 APP_DutRawTask 回 ACK */
}

/** 0x001F 的终态：等够 tmo 就把"发出多少 / 收到多少 / 收到了什么"回给 PC */
static void APP_DutRawTask(void)
{
    uint8_t  out[5u + MGR_DUT_SNIFF_SIZE];
    uint16_t rxlen;

    if (app_state != APP_STATE_DUT_RAW) {
        return;
    }
    APP_DutSniff();                         /* 每轮都收，别等 256B 环满才读 */
    if ((uint32_t)(HAL_GetTick() - s_dutsnd_t0) < (uint32_t)s_dutsnd_tmo) {
        return;                             /* 窗口还没到 */
    }

    rxlen = (s_dut_sniff_len > (uint16_t)MGR_DUT_SNIFF_SIZE)
            ? (uint16_t)MGR_DUT_SNIFF_SIZE : s_dut_sniff_len;
    out[0] = APP_STATUS_OK;
    out[1] = (uint8_t)(s_dutsnd_sent >> 8);
    out[2] = (uint8_t)s_dutsnd_sent;
    out[3] = (uint8_t)(rxlen >> 8);
    out[4] = (uint8_t)rxlen;
    if (rxlen > 0u) {
        memcpy(&out[5], s_dut_sniff, rxlen);
    }
    s_dut_sniff_len = 0u;                   /* 已经报给 PC 了，别重复报 */
    app_state = APP_STATE_IDLE;
    APP_SendFrame(MGR_CMD_DUT_SEND, s_dutsnd_seq, out, (uint16_t)(5u + rxlen));
}

/**
  * @brief  0x0020 只读：把 DUT 口攒下的原始字节取走（**读后清空**）
  * @note   不改变状态机、不发任何东西给对面。用途：对面主动上报、或者把
  *         0x001F 之后又吐出来的字节取回来。LEN 必须为 0。
  */
static void APP_DutRecvCmd(uint16_t cmd, const uint8_t *data, uint16_t len)
{
    uint8_t  out[3u + MGR_DUT_SNIFF_SIZE];
    uint16_t n;

    (void)data;
    if (len != 0u) {
        uint8_t e = APP_STATUS_ERR_PARAM;
        APP_SendAck(cmd, &e, 1u);
        return;
    }
    APP_DutSniff();                         /* 先把这个窗口里到的都收进来 */
    n = (s_dut_sniff_len > (uint16_t)MGR_DUT_SNIFF_SIZE)
        ? (uint16_t)MGR_DUT_SNIFF_SIZE : s_dut_sniff_len;
    out[0] = APP_STATUS_OK;
    out[1] = (uint8_t)(n >> 8);
    out[2] = (uint8_t)n;
    if (n > 0u) {
        memcpy(&out[3], s_dut_sniff, n);
    }
    s_dut_sniff_len = 0u;                   /* 读后清空 */
    APP_SendAck(cmd, out, (uint16_t)(3u + n));
}

/*==============================================================================
 * 0x0022 六步测试流程
 *----------------------------------------------------------------------------
 * 为什么要有它：产线上不想点八次，只想"烧完 → 按一下 → 看合格/不合格"。
 * 它把 FOC 侧的几条 AS 命令串成一条链，每步的结果都留在报告里，方便定位。
 *
 * 步骤与 detail 含义：
 *   步1 ADC    detail = 第一个不合格的**通道号**（0xFF = 六路都好）
 *   步2 存活   detail = 0（收到了 AS:V…）
 *   步3 自检   detail = 0=A0 1=A1 2=A2 3=A3（AS:CS 的原样返回码）
 *   步4 三相   detail = 位掩码 bit0/1/2 = A/B/C 有 1kHz/50% 波形
 *   步5 编码器 detail = 位掩码 bit0/1/2 = A/B/C 计数与治具发的一致
 *   步6 收尾   detail = 0
 *
 * ⚠ 安全：步3 一拿到 FOC 的回复就**立刻发 AS:0 停电机**，不管回的是哪个码 ——
 *   因为 A0 意味着"已经转起来了"。停完再等 50ms 让三相真正关断，才做后续采集。
 *============================================================================*/
#define FL6_S_ADC     0u
#define FL6_S_ALIVE   1u
#define FL6_S_SELF    2u
#define FL6_S_PWM     3u
#define FL6_S_ENC     4u
#define FL6_S_DONE    5u

#define FL6_SUB_SEND  0u
#define FL6_SUB_WAIT  1u
#define FL6_SUB_RUN   2u
#define FL6_SUB_SPIN  3u          /* 起转观察中（AS:S800 已发） */
#define FL6_SUB_HOLD  4u          /* 停车静置中（AS:S0 已发，等惯性） */

#define FL6_ALIVE_TRIES   15u      /* AS:Bb 轮询次数 */
#define FL6_ALIVE_GAP_MS  400u     /* 每次间隔：对面回一条要 1.1~1.8s，太短会互相踩 */
#define FL6_SELF_TMO_MS   12000u   /* AS:CS 等回字上限：对面校准中间会回 AS:A5，
                                    * 我们忽略它继续等 A0~A3，所以总时长要给够 */
#define FL6_STOP_MS       50u      /* 停电机后等三相关断 */
#define FL6_SPIN_MS       2000u    /* 自检通过后起转观察时间（AS:S800 转这么久） */
#define FL6_SETTLE_MS     1000u    /* 停车后静置：等惯性停住 + 反电动势衰减，
                                    * 否则步4 采相电压、步5 数编码器都会被污染 */
#define FL6_PWM_TMO_MS    3000u    /* AS:PW + 采三相的上限 */
#define FL6_PULSE_WIN_MS  200u     /* 脉冲采集窗口 */
#define FL6_ENC_TMO_MS    2000u    /* 发完脉冲等 FOC 报数的上限 */
#define FL6_ENC_PULSES    100u     /* A/B 各发 100 个 */
#define FL6_ENC_PULSES_Z  10u      /* Z 发 10 个（1 个的话 ±2% 没意义） */
#define FL6_ENC_FREQ_HZ   5000u
#define FL6_ENC_RUN_MS    2000u    /* 等脉冲发完的上限（真机上 100 个 @5kHz ≈ 20ms） */
#define FL6_ENC_ACK_MS    3000u    /* 等对面回 AS:MC0 的上限（实测约 1.5~2s） */
#define FL6_TOTAL_TMO_MS  30000u   /* 整个流程的总看门狗 —— 卡住也一定会收尾 */
/* 两条 AS 命令之间的最小间隔。
 * 依据（2026-11-12 实测）：手动连发间隔 246ms 正常，流程里 4ms 就丢命令 ——
 * 对面刚发完回复，RX 还没重新挂好，这时候推过去的命令会被丢掉。
 * 宿主测试里没有这个物理延迟，用 -DFL6_CMD_GAP_MS=0u 关掉，免得几十条时序断言跟着改。*/
#ifndef FL6_CMD_GAP_MS
#define FL6_CMD_GAP_MS    150u
#endif

static uint16_t s_fl6_seq;
static uint8_t  s_fl6_step;
static uint8_t  s_fl6_sub;
static uint8_t  s_fl6_code[MGR_FLOW_STEPS];
static uint16_t s_fl6_det[MGR_FLOW_STEPS];
static uint32_t s_fl6_t0;
static uint16_t s_fl6_tries;
static uint8_t  s_fl6_cs;                  /* AS:CS 的返回码 0..3，0xFF = 超时 */
static uint32_t s_fl6_tall;                /* 流程开始时刻（总看门狗用） */
static uint32_t s_fl6_next;                /* 最早允许发下一条命令的时刻 */
static uint8_t  s_fl6_forced;              /* 1 = 看门狗已经强制收尾过 */
static uint8_t  s_fl6_force;                /* 0=正常；2/3 = 强制走步5/步4（调试用） */
static uint16_t s_fl6_last_a;              /* FOC 上报的原始数字（0x0025 读） */
static uint16_t s_fl6_last_b;
static uint16_t s_fl6_last_z;
static uint8_t  s_fl6_last_ok;             /* 1 = 收到过并解析成功 */
static uint8_t  s_fl6_exp_a;
static uint8_t  s_fl6_exp_b;
static uint8_t  s_fl6_exp_z;

/** 往 DUT 口发一条 AS 命令（不套帧；先清掉上一次的回字，免得认错） */
static uint8_t FL6_SendEx(const char *cmd, uint8_t clear_bg)
{
    uint16_t n = 0u;
    uint8_t  ok;

    while (cmd[n] != '\0') {
        n++;
    }
    /* clear_bg=0 时**保留**已收到的字节：重试不能清，否则对面慢回（1.1~1.8s）
     * 的回复会被下一次重试抹掉，永远认不出来（2026-11-12 实测的竞态）。*/
    if (clear_bg != 0u) {
        s_dut_sniff_len  = 0u;
        s_dut_sniff_drop = 0u;
        APP_DutSniff();                    /* 读空硬件环里的陈字节 */
    }
    ok = (UartPort_Write(UART_CFG_PORT_DUT, (const uint8_t *)cmd, n) == n) ? 1u : 0u;
    s_fl6_t0  = HAL_GetTick();
    s_fl6_sub = FL6_SUB_WAIT;
    /* ⚠ 这里**不能**清 s_fl6_tries：存活分支靠它数"问了几次"，
     *   每次重发都清零的话永远到不了上限（2026-11-10 审查发现，会死循环）。
     *   计数只在进入某一步时清（见 FL6_Goto）。*/
    return ok;
}

/** 普通命令：发之前清空接收缓冲 */
static uint8_t FL6_Send(const char *cmd)
{
    return FL6_SendEx(cmd, 1u);
}

/** 记一步的结果 */
static void FL6_Set(uint8_t step, uint8_t code, uint16_t det)
{
    if (step < (uint8_t)MGR_FLOW_STEPS) {
        s_fl6_code[step] = code;
        s_fl6_det[step]  = det;
    }
}

/** 跳到某一步（重置子状态） */
static void FL6_Goto(uint8_t step)
{
    s_fl6_step  = step;
    s_fl6_sub   = FL6_SUB_SEND;
    s_fl6_t0    = HAL_GetTick();
    s_fl6_tries = 0u;                    /* 换一步就重新数 */
    /* 换步之后先等 150ms 再开口：对面可能还在发上一条的回复，
     * 这时候推命令过去会被它丢掉（2026-11-12 实测）。*/
    s_fl6_next  = HAL_GetTick() + (uint32_t)FL6_CMD_GAP_MS;
}

/** 到点了吗？各步的 SEND 分支先问一句，没到就下一轮再来 */
static uint8_t FL6_Ready(void)
{
    return ((int32_t)(HAL_GetTick() - s_fl6_next) >= 0) ? 1u : 0u;
}

/** 本步做完 → 决定下一步走哪（分支都在这里） */
static void FL6_Advance(void)
{
    switch (s_fl6_step) {
    case FL6_S_ADC:   FL6_Goto(FL6_S_ALIVE); break;
    case FL6_S_ALIVE:
        /* 存活失败就别再问自检了：板子都没起来，AS:CS 必然也超时（白等 5 秒）。
         * 直接收尾，报告里步3 留"跳过"，一眼能分清"没起来"和"自检没通过"。*/
        FL6_Goto((s_fl6_code[FL6_S_ALIVE] == 0x02u) ? FL6_S_DONE : FL6_S_SELF);
        break;
    case FL6_S_SELF:
        /* 强制分支（调试用）：让健康板也能走步4/步5。
         * 报告里步3 的 detail 会显示被强制的码，如实反映"这次是怎么走的"。*/
        if ((s_fl6_force == (uint8_t)MGR_FLOW_FORCE_A2) ||
            (s_fl6_force == (uint8_t)MGR_FLOW_FORCE_A3)) {
            s_fl6_cs = s_fl6_force;
            FL6_Set(FL6_S_SELF, 0x01u, s_fl6_force);
        }
        if (s_fl6_cs == 0x03u) {            /* A3 相位缺失 → 采三相看 MCU 有没有输出 */
            FL6_Goto(FL6_S_PWM);
        } else if (s_fl6_cs == 0x02u) {     /* A2 编码器校准失败 → 人工比对计数 */
            FL6_Goto(FL6_S_ENC);
        } else {
            FL6_Goto(FL6_S_DONE);           /* A0 全好 / A1 硬失败 / 超时 */
        }
        break;
    case FL6_S_PWM:
    case FL6_S_ENC:
    default:          FL6_Goto(FL6_S_DONE);  break;
    }
}

/** 步1：治具自己读 6 路 ADC 并判（不依赖 FOC，板子通不通都能读） */
static void FL6_Adc(void)
{
    static const uint32_t rail_mv[4] = { 24000u, 10000u, 5000u, 3300u };
    meas_snapshot_t snap;
    uint16_t bad = 0xFFu;
    uint8_t  i;

    MEAS_Snapshot(&snap);

    for (i = 0u; i < 4u; i++) {                          /* 4 路电源：±5%，下限 200mV */
        uint32_t ex  = rail_mv[i];
        uint32_t tol = (ex * 5u) / 100u;
        uint32_t mv  = snap.ch[i].mv;

        if (tol < 200u) {
            tol = 200u;
        }
        if (((snap.ch[i].flags & MEAS_CHF_OK) == 0u) || (mv + tol < ex) || (mv > ex + tol)) {
            bad = (uint16_t)i;
            break;
        }
    }
    /* 两路相电流**不参与判定**（2026-11-13 定）。
     * 原因：产线不测相电流（传感器不接/不要求），判它会让每块板都在步1 失败。
     * 它们仍在 MEAS 里被采样（0x000E 可单独读），只是不再影响合格判定。
     * 所以步1 的 detail 现在只会是 0~3（24V/10V/5V/3.3V），全好仍是 0x00FF。 */
    FL6_Set(FL6_S_ADC, (bad == 0xFFu) ? 0x01u : 0x02u, bad);
    FL6_Advance();
}

/** 六步流程主推进：每轮 APP_Run 调一次 */
static void APP_FlowTestTask(void)
{
    uint32_t now = HAL_GetTick();
    uint16_t n;
    const uint8_t *rx;

    if (app_state != APP_STATE_FLOW6) {
        return;
    }

    /* 总看门狗：不管卡在哪一步，到 30 秒强制收尾。
     * 为什么必须要有：步5 等脉冲那段原来没有超时，一旦 EncGen 卡住，
     * 流程永远停在 FLOW6 —— 之后 PC 的每条命令都被判"忙"，
     * 表现就是"FOC 收不到命令"（2026-11-11 用户实际遇到）。*/
    if ((s_fl6_forced == 0u) &&
        ((uint32_t)(now - s_fl6_tall) > (uint32_t)FL6_TOTAL_TMO_MS)) {
        s_fl6_forced = 1u;
        PULSE_CfgStop();
        EncGen_Stop();
        (void)FL6_Send(AS_CMD_STOP);
        if (s_fl6_code[s_fl6_step] == 0x00u) {
            FL6_Set(s_fl6_step, 0x02u, 0xFFFFu);   /* detail=FFFF = 看门狗强制结束 */
        }
        s_fl6_step = FL6_S_DONE;
        s_fl6_sub  = FL6_SUB_SEND;
        return;
    }

    switch (s_fl6_step) {
    /*──────── 步1 ADC ────────*/
    case FL6_S_ADC:
        FL6_Adc();
        break;

    /*──────── 步2 存活：轮询 AS:Bb ────────*/
    case FL6_S_ALIVE:
        if (s_fl6_sub == FL6_SUB_SEND) {
            if (FL6_Ready() == 0u) {
                break;                                    /* 还没到可发时刻 */
            }
            /* 只在第一次清缓冲；重试时累积着等（识别是"全文找前缀"，累积无副作用） */
            (void)FL6_SendEx(AS_CMD_VERSION, (s_fl6_tries == 0u) ? 1u : 0u);
            break;
        }
        rx = s_dut_sniff;
        n  = s_dut_sniff_len;
        if ((APP_AsHasPrefix(rx, n, AS_PREFIX_VERSION) != 0u) ||
            (APP_AsHasPrefix(rx, n, AS_PREFIX_VERSION2) != 0u)) {
            /* 实测回字是 `bb:0.3.0`；两种前缀都认，版本号只记录不判 */
            FL6_Set(FL6_S_ALIVE, 0x01u, 0u);
            FL6_Advance();
        } else if ((uint32_t)(now - s_fl6_t0) >= (uint32_t)FL6_ALIVE_GAP_MS) {
            s_fl6_tries++;
            if (s_fl6_tries >= (uint16_t)FL6_ALIVE_TRIES) {
                FL6_Set(FL6_S_ALIVE, 0x02u, s_fl6_tries);
                FL6_Advance();
            } else {
                s_fl6_sub = FL6_SUB_SEND;                /* 再问一次 */
            }
        }
        break;

    /*──────── 步3 自检 AS:CS → 立刻停 ────────*/
    case FL6_S_SELF:
        if (s_fl6_sub == FL6_SUB_SEND) {
            if (FL6_Ready() == 0u) {
                break;
            }
            (void)FL6_Send(AS_CMD_SELFTEST);
            break;
        }
        if (s_fl6_sub == FL6_SUB_WAIT) {
            uint8_t c = APP_AsFindCode(s_dut_sniff, s_dut_sniff_len);

            if (c != 0xFFu) {
                s_fl6_cs = c;
                FL6_Set(FL6_S_SELF, 0x01u, c);
                if (c == 0u) {
                    /* A0 = 自检通过 → **起转验证**。
                     * 实测 AS:CS 回 A0 并不会自己转，必须显式发 AS:S800。*/
                    (void)FL6_Send(AS_CMD_RUN);
                    s_fl6_sub = FL6_SUB_SPIN;            /* 覆盖 Send 设的 WAIT */
                    s_fl6_t0  = HAL_GetTick();
                } else {
                    (void)FL6_Send(AS_CMD_STOP);         /* A1/A2/A3：有故障，只停不加电 */
                    s_fl6_sub = FL6_SUB_RUN;
                }
            } else if ((uint32_t)(now - s_fl6_t0) >= (uint32_t)FL6_SELF_TMO_MS) {
                s_fl6_cs = 0xFFu;
                FL6_Set(FL6_S_SELF, 0x02u, 0xFFu);
                (void)FL6_Send(AS_CMD_STOP);
                s_fl6_sub = FL6_SUB_RUN;
            }
            break;
        }
        if (s_fl6_sub == FL6_SUB_SPIN) {         /* 起转观察中 */
            if ((uint32_t)(now - s_fl6_t0) >= (uint32_t)FL6_SPIN_MS) {
                (void)FL6_Send(AS_CMD_STOP);     /* 转满 → 停车 */
                s_fl6_sub = FL6_SUB_HOLD;        /* 覆盖 Send 设的 WAIT */
                s_fl6_t0  = HAL_GetTick();
            }
            break;
        }
        if (s_fl6_sub == FL6_SUB_HOLD) {         /* 停车静置：等惯性 + 反电动势 */
            if ((uint32_t)(now - s_fl6_t0) >= (uint32_t)FL6_SETTLE_MS) {
                FL6_Advance();
            }
            break;
        }
        /* SUB_RUN：等电机停稳（三相真正关断）再走下一步 */
        if ((uint32_t)(now - s_fl6_t0) >= (uint32_t)FL6_STOP_MS) {
            FL6_Advance();
        }
        break;

    /*──────── 步4 三相：AS:PW + 采 3 路 PWM ────────*/
    case FL6_S_PWM:
        if (s_fl6_sub == FL6_SUB_SEND) {
            if (FL6_Ready() == 0u) {
                break;
            }
            (void)FL6_Send(AS_CMD_PWM);
            /* ⚠ 必须走 cfg 层（PULSE_CfgStart）：它负责开 NVIC、清寄存器、清边沿环。
             *   只调算法层的 PULSE_Start 在真板子上等于"窗口开了但捕获中断没使能"
             *   → 永远采不到波形（2026-11-10 审查发现）。*/
            PULSE_CfgStart((uint16_t)FL6_PULSE_WIN_MS);
            s_fl6_sub = FL6_SUB_RUN;
            s_fl6_t0  = HAL_GetTick();
            break;
        }
        if (PULSE_CfgPoll() != 0u) {
            pulse_snapshot_t ps;
            uint16_t mask = 0u;
            uint8_t  i;

            PULSE_Snapshot(&ps);
            for (i = 0u; i < (uint8_t)PULSE_PH_COUNT; i++) {
                uint32_t per  = ps.ph[i].period_ns;
                uint16_t duty = ps.ph[i].duty_permille;

                /* 三相都该是 1kHz/50%：周期 0.5~2ms、占空比 450~550‰ 就算有波形 */
                if ((per >= 500000u) && (per <= 2000000u) &&
                    (duty >= 450u) && (duty <= 550u)) {
                    mask |= (uint16_t)(1u << i);
                }
            }
            (void)FL6_Send(AS_CMD_PWX);          /* 退出 PW（对面回 AS:P1，不等） */
            FL6_Set(FL6_S_PWM, (mask == 0x0007u) ? 0x01u : 0x02u, mask);
            FL6_Advance();
        } else if ((uint32_t)(now - s_fl6_t0) >= (uint32_t)FL6_PWM_TMO_MS) {
            PULSE_CfgStop();                     /* 万一捕获卡住，别让定时器一直开着 */
            (void)FL6_Send(AS_CMD_PWX);
            FL6_Set(FL6_S_PWM, 0x02u, 0x0000u);
            FL6_Advance();
        }
        break;

    /*──────── 步5 编码器：AS:MC + 发脉冲 + 收 AS:AaBbCc ────────*/
    case FL6_S_ENC:
        if (s_fl6_sub == FL6_SUB_SEND) {
            if (FL6_Ready() == 0u) {
                break;
            }
            (void)FL6_Send(AS_CMD_ENC);
            break;
        }
        if (s_fl6_sub == FL6_SUB_WAIT) {
            /* ⚠ 这里改成"发完 AS:MC 等 20ms 就打"，**不再等 AS:MC0**。
             *   依据（2026-11-12 实测）：MC0 要 1.5~2 秒才回，而 FOC 的计数窗口
             *   看来在 `AS:MC` 之后就开始了（MC0 一到，`AS:D` 紧跟其后就上报）。
             *   等 MC0 再打，脉冲正好落在窗口外 → 数到 0。
             *   20ms 是给对面配 EXTI 的余量，不能省。*/
            if ((uint32_t)(now - s_fl6_t0) < 20u) {
                break;                                   /* 给对面几毫秒启动 */
            }
            {
                enc_plan_t p;

                p.cnt_a = (uint16_t)FL6_ENC_PULSES;
                p.cnt_b = (uint16_t)FL6_ENC_PULSES;
                p.cnt_z = (uint16_t)FL6_ENC_PULSES_Z;
                p.freq_a_hz = (uint16_t)FL6_ENC_FREQ_HZ;
                p.freq_b_hz = (uint16_t)FL6_ENC_FREQ_HZ;
                p.freq_z_hz = (uint16_t)FL6_ENC_FREQ_HZ;

                s_dut_sniff_len = 0u;                /* 清掉，别把 OK 当计数 */
                if (EncGen_Start(&p) == 0u) {
                    FL6_Set(FL6_S_ENC, 0x02u, 0xFFFEu);   /* 参数被拒（内部错误） */
                    FL6_Advance();
                } else {
                    s_fl6_exp_a = (uint8_t)FL6_ENC_PULSES;
                    s_fl6_exp_b = (uint8_t)FL6_ENC_PULSES;
                    s_fl6_exp_z = (uint8_t)FL6_ENC_PULSES_Z;
                    s_fl6_sub   = FL6_SUB_RUN;
                    s_fl6_t0    = HAL_GetTick();   /* 给"等脉冲发完"这一段计时 */
                }
            }
            break;
        }
        if (s_fl6_sub == FL6_SUB_RUN) {              /* 等脉冲发完 */
            EncGen_Poll();
            if ((EncGen_Busy() != 0u) &&
                ((uint32_t)(now - s_fl6_t0) >= (uint32_t)FL6_ENC_RUN_MS)) {
                /* 脉冲发生器卡住：别把整个流程拖死 */
                EncGen_Stop();
                (void)FL6_Send(AS_CMD_STOP);
                FL6_Set(FL6_S_ENC, 0x02u, 0xFFFDu);   /* FFFD = 脉冲发不完 */
                FL6_Advance();
                break;
            }
            if (EncGen_Busy() == 0u) {
                /* 脉冲发完 → 发 AS:ME 让对面结束计数并上报（它会回 AS:D,A=..,B=..,Z=..） */
                (void)FL6_Send(AS_CMD_ME);
                s_fl6_sub = 0x03u;                   /* 覆盖 FL6_Send 设的 WAIT */
                s_fl6_t0  = HAL_GetTick();
            }
            break;
        }
        {
            uint16_t a, b, c;

            if (APP_AsParseEncD(s_dut_sniff, s_dut_sniff_len, &a, &b, &c) != 0u) {
                uint16_t mask = 0u;

                s_fl6_last_a  = a;                   /* 留一份原始数字给 0x0025 */
                s_fl6_last_b  = b;
                s_fl6_last_z  = c;
                s_fl6_last_ok = 1u;

                if ((a + 2u >= (uint16_t)s_fl6_exp_a) && (a <= (uint16_t)s_fl6_exp_a + 2u)) {
                    mask |= 0x0001u;
                }
                if ((b + 2u >= (uint16_t)s_fl6_exp_b) && (b <= (uint16_t)s_fl6_exp_b + 2u)) {
                    mask |= 0x0002u;
                }
                if ((c + 1u >= (uint16_t)s_fl6_exp_z) && (c <= (uint16_t)s_fl6_exp_z + 1u)) {
                    mask |= 0x0004u;
                }
                if ((c == 0u) && (mask == 0x0003u)) {
                    mask |= 0x0004u;                 /* 没有 Z 的编码器：报 C0 也算对 */
                }
                (void)FL6_Send(AS_CMD_STOP);
                FL6_Set(FL6_S_ENC, (mask == 0x0007u) ? 0x01u : 0x02u, mask);
                FL6_Advance();
            } else if ((uint32_t)(now - s_fl6_t0) >= (uint32_t)FL6_ENC_TMO_MS) {
                (void)FL6_Send(AS_CMD_STOP);
                /* 0xFFF0 = **压根没收到 AS:D**（和"收到了但数不对"区分开，
                 * 后者 detail 是比对掩码 0~7）。2026-11-12 实测发现两者都是 0，
                 * 无法判断该查接线还是查协议。*/
                FL6_Set(FL6_S_ENC, 0x02u, 0xFFF0u);
                FL6_Advance();
            }
        }
        break;

    /*──────── 步6 收尾：再发一次 AS:0，然后回报告 ────────*/
    case FL6_S_DONE:
    default:
        if (s_fl6_sub == FL6_SUB_SEND) {
            if (FL6_Ready() == 0u) {
                break;
            }
            (void)FL6_Send(AS_CMD_STOP);
            FL6_Set(FL6_S_DONE, 0x01u, 0u);
            break;
        }
        {
            uint8_t  out[MGR_FLOW_TEST_RESP_LEN];
            uint8_t  any_bad = 0u;
            uint8_t  first   = 0u;
            uint8_t  i;

            for (i = 0u; i < (uint8_t)MGR_FLOW_STEPS; i++) {
                if ((s_fl6_code[i] == 0x02u) && (first == 0u)) {
                    first = (uint8_t)(i + 1u);
                }
            }
            /* A1（ADC 校准失败）时步3 是"通过"但整体要判失败，单独补一条 */
            if ((s_fl6_cs == 0x01u) && (first == 0u)) {
                first = (uint8_t)(FL6_S_SELF + 1u);
            }
            any_bad = (first != 0u) ? 1u : 0u;

            out[0] = (any_bad != 0u) ? APP_STATUS_ERR_TEST : APP_STATUS_OK;
            out[1] = (uint8_t)(s_fl6_step + 1u);
            out[2] = first;
            for (i = 0u; i < (uint8_t)MGR_FLOW_STEPS; i++) {
                uint8_t *q = &out[3u + ((uint16_t)i * MGR_FLOW_ITEM_LEN)];

                q[0] = s_fl6_code[i];
                q[1] = (uint8_t)(s_fl6_det[i] >> 8);
                q[2] = (uint8_t)(s_fl6_det[i] & 0xFFu);
            }
            app_state = APP_STATE_IDLE;
            APP_SendFrame(MGR_CMD_FLOW_TEST, s_fl6_seq, out, MGR_FLOW_TEST_RESP_LEN);
        }
        break;
    }
}

/** 0x0022 受理：清结果、进流程 */
static void APP_FlowTestCmd(uint16_t cmd, const uint8_t *data, uint16_t len)
{
    uint8_t i;

    if (len > 1u) {                          /* 只接受 LEN=0（正常）或 1（强制分支）*/
        uint8_t e = APP_STATUS_ERR_PARAM;

        APP_SendAck(cmd, &e, 1u);
        return;
    }
    s_fl6_force = (uint8_t)MGR_FLOW_FORCE_NONE;
    if (len == 1u) {
        if ((data[0] != (uint8_t)MGR_FLOW_FORCE_NONE) &&
            (data[0] != (uint8_t)MGR_FLOW_FORCE_A2) &&
            (data[0] != (uint8_t)MGR_FLOW_FORCE_A3)) {
            uint8_t e = APP_STATUS_ERR_PARAM;

            APP_SendAck(cmd, &e, 1u);
            return;
        }
        s_fl6_force = data[0];
    }
    for (i = 0u; i < (uint8_t)MGR_FLOW_STEPS; i++) {
        s_fl6_code[i] = 0u;                  /* 00 = 跳过（没走这条分支） */
        s_fl6_det[i]  = 0u;
    }
    s_fl6_seq    = pc_seq;
    s_fl6_cs     = 0xFFu;
    s_fl6_tall    = HAL_GetTick();
    s_fl6_forced  = 0u;
    s_fl6_last_ok = 0u;
    s_fl6_last_a  = 0u;
    s_fl6_last_b  = 0u;
    s_fl6_last_z  = 0u;
    FL6_Goto(FL6_S_ADC);
    app_state = APP_STATE_FLOW6;
}
#endif /* JIG_DUT_LINK */

/*==============================================================================
 * ⑤ 编码器脉冲发生（0x0012）
 *============================================================================*/
static uint16_t enc_be16(const uint8_t *p)
{
    return (uint16_t)(((uint16_t)p[0] << 8) | (uint16_t)p[1]);
}

/**
  * @brief  0x0012 发一串编码器脉冲
  * @param  data/len 请求 DATA：LEN=0 → 用默认参数；LEN=12 → cnt_a/b/z + freq_a/b/z（大端）
  * @note   非阻塞：这里只发起，发完（约 200ms）由 APP_EncGenTask 回 ACK。
  *         ACK 里带上**实际翻转次数**，上板时一眼就能看出"到底发出去没有"。
  */
static void APP_EncGenCmd(uint16_t cmd, const uint8_t *data, uint16_t len)
{
    enc_plan_t p;
    uint8_t    e;

    if (len == 0u) {
        EncPlan_Default(&p);
    } else if (len == MGR_ENC_GEN_REQ_LEN) {
        p.cnt_a     = enc_be16(&data[0]);
        p.cnt_b     = enc_be16(&data[2]);
        p.cnt_z     = enc_be16(&data[4]);
        p.freq_a_hz = enc_be16(&data[6]);
        p.freq_b_hz = enc_be16(&data[8]);
        p.freq_z_hz = enc_be16(&data[10]);
    } else {
        e = APP_STATUS_ERR_PARAM;
        APP_SendAck(cmd, &e, 1u);
        return;
    }

    /* **命令层自己先校验一遍**（不把校验全甩给发生层）：
     *   - 符合"参数校验先于动作"的既有规矩，非法参数绝不会走到寄存器那一层；
     *   - 让参数错误路径在宿主单测里可测（发生层的寄存器代码宿主机编译不了）；
     *   - EncGen_Start 里还会再校验一次，属于纵深防御，不是重复。 */
    if (EncPlan_Check(&p) == 0u) {
        e = APP_STATUS_ERR_PARAM;
        APP_SendAck(cmd, &e, 1u);
        return;
    }
    if (EncGen_Start(&p) == 0u) {
        /* 走到这里只可能是"上一次还没收尾"——非 IDLE 时进不到 dispatch，
         * 所以正常情况下到不了；留着是为了不漏掉任何一条失败路径。 */
        e = APP_STATUS_BUSY;
        APP_SendAck(cmd, &e, 1u);
        return;
    }
    s_enc_plan = p;
    s_enc_seq  = pc_seq;
    app_state  = APP_STATE_ENC;             /* 终态由 APP_EncGenTask 回 ACK */
}

/**
  * @brief  编码器脉冲串：每个主循环推进一步，发完回 ACK
  * @note   ACK 9B = 状态(1) + 完成掩码(1) + 超时强停(1) + A/B/Z 实际翻转次数(2×3, 大端)
  */
static void APP_EncGenTask(void)
{
    uint8_t out[MGR_ENC_GEN_RESP_LEN];

    if (app_state != APP_STATE_ENC) {
        return;
    }
    EncGen_Poll();
    if (EncGen_Busy() != 0u) {
        return;                             /* 还在发 */
    }

    out[0] = APP_STATUS_OK;
    out[1] = EncGen_DoneMask();
    out[2] = EncGen_Aborted();
    {
        uint16_t f;
        f = EncGen_Flips(0u); out[3] = (uint8_t)(f >> 8); out[4] = (uint8_t)f;
        f = EncGen_Flips(1u); out[5] = (uint8_t)(f >> 8); out[6] = (uint8_t)f;
        f = EncGen_Flips(2u); out[7] = (uint8_t)(f >> 8); out[8] = (uint8_t)f;
    }
    app_state = APP_STATE_IDLE;
    APP_SendFrame(MGR_CMD_ENC_GEN, s_enc_seq, out, MGR_ENC_GEN_RESP_LEN);
}

/**
  * @brief  0x0013 向 U2 要编码器线的边沿计数（⑤ 的回环）
  * @param  data/len 1 字节动作：0 = 清零（发脉冲之前）, 1 = 读取（发完之后）
  * @note   转成内部链路的 ENC_XCHK(0x0109)。非阻塞，终态由 APP_EncPeerTask 回 ACK。
  *         **清零放在发脉冲之前、读取放在之后**，这样 U2 数到的就正好是这一串。
  */
static void APP_EncPeerCmd(uint16_t cmd, const uint8_t *data, uint16_t len)
{
    uint8_t e;

    if ((len != 1u) || (data == 0) ||
        ((data[0] != LINK_ENC_ACT_CLEAR) && (data[0] != LINK_ENC_ACT_READ))) {
        e = APP_STATUS_ERR_PARAM;
        APP_SendAck(cmd, &e, 1u);
        return;
    }
    /* ⚠ 台架形态（JIG_U2_LINK=0）没有 U2，这里**当场回超时**：
     *   不这么写的话会走下面"等链路空出来"的分支，白等 MGR_ENC_PEER_TIMEOUT_MS
     *   才报错 —— ④ 的流程里这一步要跑两次，白白多花一倍时间。 */
#if (JIG_U2_LINK == 0)
    e = APP_STATUS_ERR_TIMEOUT;
    APP_SendAck(cmd, &e, 1u);
#else
    /* ⚠ **不在这里直接发**：内部链路上随时可能有一发保活探测（PING/STATUS）在飞，
     * 那时 RequestStart 会返回 0。早先的写法就是直接回 BUSY —— 结果是
     * "PC 发命令的时机碰巧撞上保活探测就失败"，这不是调用方的错。
     * 改成先把请求**挂起来**，由 APP_EncPeerTask 等到链路空出来再发（有上限）。 */
    /* ⚠ 先把内部链路上可能残留的**保活探测**收掉，再挂我们的请求。
     *   踩过的坑：`APP_LinkTask()` 只在 `app_state == IDLE` 时推进，
     *   而我们马上要进入 ENC_PEER（非 IDLE）→ 那一发在飞的探测再也没人轮询，
     *   `waiting` 永远是 1 → `Link_RequestStart` 永远返回 0 → 命令永远等不到应答、
     *   状态机卡在 ENC_PEER，最后只能超时。现象是"命令看起来不工作"，
     *   其实只是链路被一发探测占住了。
     *   `timeout_ms = 0` 的语义正是"只巡一遍"，用它把陈旧请求收尾最合适。 */
    APP_U2Flush();

    s_encp_act     = data[0];
    s_encp_seq     = pc_seq;
    s_encp_started = 0u;
    s_encp_t0      = HAL_GetTick();
    app_state      = APP_STATE_ENC_PEER;
#endif
}

static void APP_EncPeerTask(void)
{
    uint16_t rn = 0u;
    uint8_t  st;

    if (app_state != APP_STATE_ENC_PEER) {
        return;
    }

    /* 第一阶段：等链路空出来，把 ENC_XCHK 发出去 */
    if (s_encp_started == 0u) {
        if (APP_U2Send(LINK_CMD_ENC_XCHK, &s_encp_act, 1u) != 0u) {
            s_encp_started = 1u;
        } else if ((uint32_t)(HAL_GetTick() - s_encp_t0) > MGR_ENC_PEER_TIMEOUT_MS) {
            uint8_t e = APP_STATUS_BUSY;        /* 链路一直占着（对端可能卡住） */
            app_state = APP_STATE_IDLE;
            APP_SendFrame(MGR_CMD_ENC_PEER, s_encp_seq, &e, 1u);
        }
        return;
    }

    /* 第二阶段：等 U2 的应答 */
    st = APP_U2Poll(MGR_ENC_PEER_TIMEOUT_MS, s_encp_resp, &rn);
    if (st == 1u) {
        s_encp_ack[0] = APP_STATUS_OK;
        if (rn > 0u) {
            memcpy(&s_encp_ack[1], s_encp_resp, rn);
        }
        app_state = APP_STATE_IDLE;
        APP_SendFrame(MGR_CMD_ENC_PEER, s_encp_seq, s_encp_ack, (uint16_t)(1u + rn));
    } else if (st == 2u) {
        uint8_t e = APP_STATUS_ERR_TIMEOUT;     /* U2 不在 / 不支持 ENC_XCHK */
        app_state = APP_STATE_IDLE;
        APP_SendFrame(MGR_CMD_ENC_PEER, s_encp_seq, &e, 1u);
    } else {
        /* 0 = 还在等 */
    }
}

/*===== ⑥ 相电流检测：0x0014 完整流程（需要 JIG_DUT_LINK=1）=====
 * 顺序：读基线（本片 + DUT）→ 让 DUT 通流 → 等 → 再读（本片 + DUT）→ 判定 → 出报告。
 * 全程非阻塞：每个 DUT 往返都用 RequestStart/RequestPoll，一次大约 duration_ms + 50ms。 */
#if (JIG_DUT_LINK != 0)
#define APP_CUR_S_JIG_IDLE   0u   /* 读本片基线 */
#define APP_CUR_S_DUT_IDLE   1u   /* 向 DUT 要基线 */
#define APP_CUR_S_DUT_CUR    2u   /* 让 DUT 通流 */
#define APP_CUR_S_WAIT       3u   /* 等电流稳定 */
#define APP_CUR_S_JIG_ON     4u   /* 读本片"通流中" */
#define APP_CUR_S_DUT_ON     5u   /* 向 DUT 要"通流中" */
#define APP_CUR_S_DONE       6u   /* 判定 + 出报告 */

static uint16_t      s_cur_seq;
static uint8_t       s_cur_stage;
static uint8_t       s_cur_started;     /* 当前阶段的 DUT 请求已发出 */
static uint32_t      s_cur_t0;          /* 阶段计时（超时兜底） */
static uint32_t      s_cur_wait_t0;     /* WAIT 阶段的起点 */
static uint16_t      s_cur_dur_ms;
static uint8_t       s_cur_mode;        /* DUT_CUR_MODE_STATIC / _PWM（0x0014 可选第 9 字节）*/
static uint8_t       s_cur_amask[DUT_GETADC_REQ_LEN];   /* 向 DUT 要哪两路（大端 u16，bit7/bit8）*/
static uint8_t       s_cur_req[MGR_CUR_TEST_REQ_LEN];
static uint8_t       s_cur_cur[DUT_CURRENT_REQ_LEN];    /* DUT_CURRENT 的请求（单独一个缓冲，
                                                         * 免得和 s_cur_req 混用）*/
static uint8_t       s_cur_resp[LINK_MAX_PAYLOAD];
static uint8_t       s_cur_dut_st;      /* DUT_CURRENT 的状态码（报告里带回去） */
static cur_chk_in_t  s_cur_in[CUR_CHK_CH_COUNT];
static cur_chk_out_t s_cur_out[CUR_CHK_CH_COUNT];

/* 从解好的 DUT_GET_ADC 应答里取某一路电流的 mV；返回 1 = 拿到了 */
static uint8_t APP_CurTakeDut(const dut_adc_t *a, uint8_t which, uint16_t *mv)
{
    uint8_t want_ch;
    uint8_t i;

    if ((a == 0) || (mv == 0)) {
        return 0u;
    }
    want_ch = (which == CUR_CHK_CH1) ? (uint8_t)DUT_ADC_CUR1 : (uint8_t)DUT_ADC_CUR2;
    for (i = 0u; i < a->n; i++) {
        if (a->item[i].ch == want_ch) {
            *mv = a->item[i].mv;
            return 1u;
        }
    }
    return 0u;                              /* DUT 没报这一路 */
}

/* 本片快照 → 两路电流的 mV；返回哪几路可用（bit0 = 电流1） */
static uint8_t APP_CurTakeJig(uint16_t *mv1, uint16_t *mv2)
{
    meas_snapshot_t snap;
    uint16_t a = 0u, b = 0u;
    uint8_t  ok = 0u;

    MEAS_Snapshot(&snap);
    if (CurChk_TakeJig(&snap, CUR_CHK_CH1, &a) != 0u) { ok |= 0x01u; }
    if (CurChk_TakeJig(&snap, CUR_CHK_CH2, &b) != 0u) { ok |= 0x02u; }
    if (mv1 != 0) { *mv1 = a; }
    if (mv2 != 0) { *mv2 = b; }
    return ok;
}

static void APP_CurTestCmd(uint16_t cmd, const uint8_t *data, uint16_t len)
{
    uint8_t e;

    if (len == 0u) {
        /* 默认：上桥 A、下桥 B、静态通流、1000‰、200ms、两路电流都读 */
        s_cur_req[0] = DUT_PHASE_A;
        s_cur_req[1] = DUT_PHASE_B;
        s_cur_req[2] = 0x03u; s_cur_req[3] = 0xE8u;      /* 1000‰ */
        s_cur_req[4] = 0x00u; s_cur_req[5] = 0xC8u;      /* 200 ms */
        s_cur_req[6] = 0x03u;                            /* 两路 */
        s_cur_req[7] = 0x00u;
        s_cur_mode   = (uint8_t)DUT_CUR_MODE_STATIC;
    } else if ((len == MGR_CUR_TEST_REQ_LEN) ||
               (len == (uint16_t)(MGR_CUR_TEST_REQ_LEN + 1u))) {
        /* ⚠ 第 9 字节（可省略）= DUT 的通流模式：
         *   0 = 静态全通（DUT 按 1000‰ 处理，**duty 这个字段会被忽略**）
         *   1 = 低频 PWM（duty 才生效）
         * 为什么加它：静态全通是给"夹具提供电流回路"的产线工况用的；台架上被测板
         * 接着**三相电机**时，静态全通用 24V 直接压在两个绕组上 = 几十安培，
         * 会烧功率管或电机。要安全地试，必须用 mode=1 + 小占空比。
         * LEN=8 的老帧仍然按 mode=0 解释（向后兼容）。 */
        memcpy(s_cur_req, data, MGR_CUR_TEST_REQ_LEN);
        s_cur_mode = (len == (uint16_t)(MGR_CUR_TEST_REQ_LEN + 1u))
                     ? data[MGR_CUR_TEST_REQ_LEN]
                     : (uint8_t)DUT_CUR_MODE_STATIC;
    } else {
        e = APP_STATUS_ERR_PARAM;
        APP_SendAck(cmd, &e, 1u);
        return;
    }

    /* 参数预检（命令层自己先过一遍，不把校验全甩给 DUT） */
    s_cur_dur_ms = (uint16_t)(((uint16_t)s_cur_req[4] << 8) | s_cur_req[5]);
    {
        dut_current_t c;
        c.mode          = s_cur_mode;
        c.hi_mask       = s_cur_req[0];
        c.lo_mask       = s_cur_req[1];
        c.duty_permille = (uint16_t)(((uint16_t)s_cur_req[2] << 8) | s_cur_req[3]);
        c.duration_ms   = s_cur_dur_ms;

        if ((DutCheck_Current(&c) == 0u) ||
            (s_cur_mode > (uint8_t)DUT_CUR_MODE_PWM) ||
            ((s_cur_req[6] & ~(uint8_t)0x03u) != 0u) ||
            ((s_cur_req[6] & 0x03u) == 0u) ||
            (s_cur_req[7] != 0u)) {
            e = APP_STATUS_ERR_PARAM;
            APP_SendAck(cmd, &e, 1u);
            return;
        }
    }
    /* DUT 的 ADC 通道掩码：电流检测1 → bit7，电流检测2 → bit8（**2 字节大端**，
     * 不是 1 字节 —— 通道号到 8，1 字节表达不了，这条在 docs/15 §7.8 专门写过）。 */
    {
        uint16_t am = 0u;
        if ((s_cur_req[6] & 0x01u) != 0u) { am |= (uint16_t)(1u << DUT_ADC_CUR1); }
        if ((s_cur_req[6] & 0x02u) != 0u) { am |= (uint16_t)(1u << DUT_ADC_CUR2); }
        s_cur_amask[0] = (uint8_t)(am >> 8);
        s_cur_amask[1] = (uint8_t)(am & 0xFFu);
    }

    /* 清内部链路可能残留的保活探测（同 0x0013 的理由） */
    (void)Link_RequestPoll(&s_dut_link, 0u, 0, 0);

    /* 参数校验都过了 → 起"一条命令跑完"的流程：
     * 先进测试模式（被测板在正常模式下收到 CURRENT 会回 BUSY），跑完再关输出。 */
    APP_FlowStart(APP_FLOW_KIND_CUR, cmd);
}

/* 真正启动 ⑥ 的测量状态机（由流程在第 2 步调用；参数已经校验并写进 s_cur_req） */
static void APP_CurStartMachine(void)
{
    (void)Link_RequestPoll(&s_dut_link, 0u, 0, 0);

    memset(s_cur_in, 0, sizeof(s_cur_in));
    memset(s_cur_out, 0, sizeof(s_cur_out));
    s_cur_seq     = pc_seq;
    s_cur_stage   = APP_CUR_S_JIG_IDLE;
    s_cur_started = 0u;
    s_cur_t0      = HAL_GetTick();
    s_cur_dut_st  = APP_STATUS_ERR_TIMEOUT;
    app_state     = APP_STATE_CUR;
}

static void APP_CurTestAbort(uint8_t e)
{
    app_state = APP_STATE_IDLE;
    APP_SendFrame(MGR_CMD_CUR_TEST, s_cur_seq, &e, 1u);
}

static void APP_CurTestTask(void)
{
    if (app_state != APP_STATE_CUR) {
        return;
    }

    /* 兜底：整个流程别超过 5 秒 */
    if ((uint32_t)(HAL_GetTick() - s_cur_t0) > 5000u) {
        APP_CurTestAbort(APP_STATUS_ERR_TIMEOUT);
        return;
    }

    switch (s_cur_stage) {

    case APP_CUR_S_JIG_IDLE: {
        uint16_t a = 0u, b = 0u;
        uint8_t  ok = APP_CurTakeJig(&a, &b);
        s_cur_in[CUR_CHK_CH1].jig_idle_mv = a;
        s_cur_in[CUR_CHK_CH1].jig_ok      = (uint8_t)((ok & 0x01u) != 0u);
        s_cur_in[CUR_CHK_CH2].jig_idle_mv = b;
        s_cur_in[CUR_CHK_CH2].jig_ok      = (uint8_t)((ok & 0x02u) != 0u);
        s_cur_stage   = APP_CUR_S_DUT_IDLE;
        s_cur_started = 0u;
        break;
    }

    case APP_CUR_S_DUT_IDLE:
    case APP_CUR_S_DUT_ON: {
        uint16_t rn = 0u;
        uint8_t  st;
        uint8_t  final = (uint8_t)(s_cur_stage == APP_CUR_S_DUT_ON);

        if (s_cur_started == 0u) {
            if (Link_RequestStart(&s_dut_link, DUT_CMD_GET_ADC,
                                  s_cur_amask, DUT_GETADC_REQ_LEN) != 0u) {
                s_cur_started = 1u;
                s_cur_t0      = HAL_GetTick();
            } else if ((uint32_t)(HAL_GetTick() - s_cur_t0) > MGR_CUR_TIMEOUT_MS) {
                APP_CurTestAbort(APP_STATUS_BUSY);
                return;
            }
            return;
        }
        st = Link_RequestPoll(&s_dut_link, MGR_CUR_TIMEOUT_MS, s_cur_resp, &rn);
        if (st == 0u) {
            return;                             /* 还在等 */
        }
        s_cur_started = 0u;
        if (st == 2u) {
            APP_CurTestAbort(APP_STATUS_ERR_TIMEOUT);
            return;
        }
        {
            dut_adc_t a;
            uint16_t  mv = 0u;
            uint8_t   i;

            if (DutUnpack_Adc(s_cur_resp, rn, 0u, &a) == 0u) {
                APP_CurTestAbort(APP_STATUS_ERR_PARAM);   /* 应答格式不对 */
                return;
            }
            for (i = 0u; i < (uint8_t)CUR_CHK_CH_COUNT; i++) {
                if (APP_CurTakeDut(&a, i, &mv) != 0u) {
                    if (final != 0u) { s_cur_in[i].dut_on_mv   = mv; }
                    else             { s_cur_in[i].dut_idle_mv = mv; }
                    s_cur_in[i].dut_ok = 1u;
                } else {
                    s_cur_in[i].dut_ok = 0u;      /* DUT 没报这一路 → 不判 */
                }
            }
        }
        s_cur_stage = (uint8_t)(final ? APP_CUR_S_DONE : APP_CUR_S_DUT_CUR);
        s_cur_started = 0u;
        break;
    }

    case APP_CUR_S_DUT_CUR: {
        uint16_t rn = 0u;
        uint8_t  st;

        if (s_cur_started == 0u) {
            /* ⚠ **发出去的必须是 DutPack_Current 打好的 s_cur_cur**。
             *   老代码在这里发的是 s_cur_req（PC 的 8 字节请求布局：hi/lo/duty/dur/ch/rsvd），
             *   而 DUT 那 8 字节是 mode/hi/lo/duty/dur/rsvd —— 两个布局的第 0 字节不同。
             *   结果 DUT 收到 mode=hi_mask(0x01)、lo_mask=duty 高字节，一律回 0B ERR_PARAM，
             *   现象是"⑥ 永远 dut_st=0B、两路电流全 0"，而且看帧长是对的、只错内容。 */
            {
                dut_current_t c;
                c.mode          = s_cur_mode;
                c.hi_mask       = s_cur_req[0];
                c.lo_mask       = s_cur_req[1];
                c.duty_permille = (uint16_t)(((uint16_t)s_cur_req[2] << 8) | s_cur_req[3]);
                c.duration_ms   = s_cur_dur_ms;
                if (DutPack_Current(s_cur_cur, sizeof(s_cur_cur), &c) == 0u) {
                    APP_CurTestAbort(APP_STATUS_ERR_PARAM);
                    return;
                }
            }
            if (Link_RequestStart(&s_dut_link, DUT_CMD_CURRENT,
                                  s_cur_cur, DUT_CURRENT_REQ_LEN) != 0u) {
                s_cur_started = 1u;
                s_cur_t0      = HAL_GetTick();
            } else if ((uint32_t)(HAL_GetTick() - s_cur_t0) > MGR_CUR_TIMEOUT_MS) {
                APP_CurTestAbort(APP_STATUS_BUSY);
                return;
            }
            return;
        }
        st = Link_RequestPoll(&s_dut_link, MGR_CUR_TIMEOUT_MS, s_cur_resp, &rn);
        if (st == 0u) {
            return;
        }
        s_cur_started = 0u;
        if (st == 2u) {
            APP_CurTestAbort(APP_STATUS_ERR_TIMEOUT);
            return;
        }
        /* DUT_CURRENT 是"收到即应答"，状态码带回去给 PC 看 */
        (void)DutUnpack_Status(s_cur_resp, rn, &s_cur_dut_st);
        s_cur_wait_t0 = HAL_GetTick();
        s_cur_stage   = APP_CUR_S_WAIT;
        break;
    }

    case APP_CUR_S_WAIT:
        /* 等通流时间本身 + 一点稳定余量（电流稳定 + DUT 自己的快照更新） */
        if ((uint32_t)(HAL_GetTick() - s_cur_wait_t0) <
            ((uint32_t)s_cur_dur_ms + MGR_CUR_SETTLE_MS)) {
            return;
        }
        s_cur_stage = APP_CUR_S_JIG_ON;
        break;

    case APP_CUR_S_JIG_ON: {
        uint16_t a = 0u, b = 0u;
        uint8_t  ok = APP_CurTakeJig(&a, &b);
        s_cur_in[CUR_CHK_CH1].jig_on_mv = a;
        s_cur_in[CUR_CHK_CH1].jig_ok    = (uint8_t)((ok & 0x01u) != 0u);
        s_cur_in[CUR_CHK_CH2].jig_on_mv = b;
        s_cur_in[CUR_CHK_CH2].jig_ok    = (uint8_t)((ok & 0x02u) != 0u);
        s_cur_stage = APP_CUR_S_DUT_ON;
        break;
    }

    case APP_CUR_S_DONE: {
        uint8_t out[MGR_CUR_TEST_RESP_LEN];
        uint8_t i;
        uint16_t any_bad = 0u;

        for (i = 0u; i < (uint8_t)CUR_CHK_CH_COUNT; i++) {
            uint16_t f = CurChk_Judge(&s_cur_in[i], &s_cur_out[i]);
            if ((f != 0u) &&
                ((f & CUR_CHK_F_NODATA) == 0u) &&
                (f != CUR_CHK_F_OK)) {
                any_bad = 1u;
            }
        }

        /* 应答：状态(1) + 被测板状态(1) + 两路×5（治具变化量、被测板变化量、旗标）
         * 不再发：路数/预留、通流前后的绝对值 —— 判的就是"变化量"，
         *         而同一路的绝对值在 ① 电压检测里已经报过（ch7/ch8）。*/
        out[0] = any_bad ? APP_STATUS_ERR_TEST : APP_STATUS_OK;
        out[1] = s_cur_dut_st;
        for (i = 0u; i < (uint8_t)CUR_CHK_CH_COUNT; i++) {
            uint8_t *p = &out[2u + (i * MGR_CUR_ITEM_LEN)];   /* 每路 5B */
            int16_t  dj = (int16_t)((int32_t)s_cur_in[i].jig_on_mv -
                                    (int32_t)s_cur_in[i].jig_idle_mv);
            int16_t  dd = (int16_t)((int32_t)s_cur_in[i].dut_on_mv -
                                    (int32_t)s_cur_in[i].dut_idle_mv);

            p[0] = (uint8_t)((uint16_t)dj >> 8);  p[1] = (uint8_t)((uint16_t)dj);
            p[2] = (uint8_t)((uint16_t)dd >> 8);  p[3] = (uint8_t)((uint16_t)dd);
            p[4] = (uint8_t)(s_cur_out[i].flags & 0xFFu);
        }
        app_state = APP_STATE_IDLE;
        APP_SendFrame(MGR_CMD_CUR_TEST, s_cur_seq, out, MGR_CUR_TEST_RESP_LEN);
        break;
    }

    default:
        APP_CurTestAbort(APP_STATUS_ERR_TEST);
        break;
    }
}
#endif /* JIG_DUT_LINK */

/*==============================================================================
 * ② 电压检测：0x0015 —— 治具 9 路 vs 被测板自己报的
 *============================================================================*/
#if (JIG_DUT_LINK != 0)
#define APP_VOL_S_INFO   0u   /* 问 DUT 的 INFO（拿能力位图 + 版本） */
#define APP_VOL_S_ADC    1u   /* 按位图问 GET_ADC */
#define APP_VOL_S_DONE   2u   /* 9 路判定 + 出报告 */

static uint16_t      s_vol_seq;
static uint8_t       s_vol_stage;
static uint8_t       s_vol_started;
static uint32_t      s_vol_t0;
static uint16_t      s_vol_adc_mask;
static uint8_t       s_vol_amask[DUT_GETADC_REQ_LEN];
static uint8_t       s_vol_resp[LINK_MAX_PAYLOAD];
static uint8_t       s_vol_info_st;
static dut_adc_t     s_vol_dut;
static vol_chk_out_t s_vol_out[MEAS_CH_COUNT];

static void APP_VolTestCmd(uint16_t cmd, const uint8_t *data, uint16_t len)
{
    uint8_t e;

    (void)data;                             /* 这条命令不需要参数 */
    if (len != 0u) {                        /* 这条命令不需要参数 */
        e = APP_STATUS_ERR_PARAM;
        APP_SendAck(cmd, &e, 1u);
        return;
    }
    (void)Link_RequestPoll(&s_dut_link, 0u, 0, 0);   /* 清残留的保活探测 */
    memset(&s_vol_dut, 0, sizeof(s_vol_dut));
    memset(s_vol_out, 0, sizeof(s_vol_out));
    s_vol_seq     = pc_seq;
    s_vol_stage   = APP_VOL_S_INFO;
    s_vol_started = 0u;
    s_vol_t0      = HAL_GetTick();
    s_vol_adc_mask = 0u;
    s_vol_info_st = APP_STATUS_ERR_TIMEOUT;
    app_state     = APP_STATE_VOL;
}

static void APP_VolTestAbort(uint8_t e)
{
    app_state = APP_STATE_IDLE;
    APP_SendFrame(MGR_CMD_VOL_TEST, s_vol_seq, &e, 1u);
}

static void APP_VolTestTask(void)
{
    uint16_t rn = 0u;
    uint8_t  st;

    if (app_state != APP_STATE_VOL) {
        return;
    }
    if ((uint32_t)(HAL_GetTick() - s_vol_t0) > 3000u) {
        APP_VolTestAbort(APP_STATUS_ERR_TIMEOUT);
        return;
    }

    /* 两个阶段共用同一套"发起 → 等 → 处理"骨架 */
    if (s_vol_started == 0u) {
        uint16_t dc = (s_vol_stage == APP_VOL_S_INFO) ? DUT_CMD_INFO : DUT_CMD_GET_ADC;
        const uint8_t *in = (s_vol_stage == APP_VOL_S_INFO) ? 0 : s_vol_amask;
        uint16_t inlen    = (s_vol_stage == APP_VOL_S_INFO) ? 0u : DUT_GETADC_REQ_LEN;

        if (Link_RequestStart(&s_dut_link, dc, in, inlen) != 0u) {
            s_vol_started = 1u;
            s_vol_t0      = HAL_GetTick();
        } else if ((uint32_t)(HAL_GetTick() - s_vol_t0) > MGR_CUR_TIMEOUT_MS) {
            APP_VolTestAbort(APP_STATUS_BUSY);
        }
        return;
    }

    st = Link_RequestPoll(&s_dut_link, MGR_CUR_TIMEOUT_MS, s_vol_resp, &rn);
    if (st == 0u) {
        return;
    }
    s_vol_started = 0u;
    if (st == 2u) {
        APP_VolTestAbort(APP_STATUS_ERR_TIMEOUT);
        return;
    }

    if (s_vol_stage == APP_VOL_S_INFO) {
        dut_info_t fi;
        uint16_t   am = 0u;
        uint8_t    i;

        if (DutUnpack_Info(s_vol_resp, rn, &fi) == 0u) {
            APP_VolTestAbort(APP_STATUS_ERR_PARAM);
            return;
        }
        s_vol_info_st  = fi.status;
        s_vol_adc_mask = fi.adc_mask;
        for (i = 0u; i < (uint8_t)MEAS_CH_COUNT; i++) {
            if ((fi.adc_mask & (uint16_t)(1u << i)) != 0u) {
                am |= (uint16_t)(1u << i);
            }
        }
        s_vol_amask[0] = (uint8_t)(am >> 8);
        s_vol_amask[1] = (uint8_t)(am & 0xFFu);
        s_vol_stage = APP_VOL_S_ADC;
    } else if (s_vol_stage == APP_VOL_S_ADC) {
        /* want_mask 传 0：**不要求"要的通道必须来齐"** —— 少给的那几路
         * 由 VolChk_All 判成 NODATA（不判），而不是整条命令失败。
         * 这样"被测板只支持一部分通道"也能用。 */
        if (DutUnpack_Adc(s_vol_resp, rn, 0u, &s_vol_dut) == 0u) {
            APP_VolTestAbort(APP_STATUS_ERR_PARAM);
            return;
        }
        s_vol_stage = APP_VOL_S_DONE;
    } else {
        meas_snapshot_t snap;
        uint8_t         out[MGR_VOL_TEST_RESP_LEN];
        uint16_t        sum;
        uint8_t         i;

        MEAS_Snapshot(&snap);
        sum = VolChk_All(&snap, &s_vol_dut, s_vol_adc_mask, s_vol_out);

        /* 应答：状态(1) + 被测板状态(1) + 九路×5（治具mV、被测板mV、旗标）
         * 不再发：版本号/通道声明（INFO 里有）、差值（两数相减就有）、汇总字（看旗标就有）*/
        out[0] = ((sum & VOL_CHK_SUM_ANY_BAD) != 0u) ? APP_STATUS_ERR_TEST : APP_STATUS_OK;
        out[1] = s_vol_info_st;
        for (i = 0u; i < (uint8_t)MEAS_CH_COUNT; i++) {
            uint8_t *p = &out[2u + ((uint16_t)i * MGR_VOL_CH_ITEM_LEN)];
            uint16_t jm = 0u, dm = 0u;
            (void)VolChk_TakeJig(&snap, i, &jm);
            (void)VolChk_TakeDut(&s_vol_dut, i, &dm);
            p[0] = (uint8_t)(jm >> 8);  p[1] = (uint8_t)jm;
            p[2] = (uint8_t)(dm >> 8);  p[3] = (uint8_t)dm;
            p[4] = (uint8_t)(s_vol_out[i].flags & 0xFFu);
        }
        app_state = APP_STATE_IDLE;
        APP_SendFrame(MGR_CMD_VOL_TEST, s_vol_seq, out, MGR_VOL_TEST_RESP_LEN);
    }
}
#endif /* JIG_DUT_LINK */

/*==============================================================================
 * ④ 三相输出判据：0x0016 —— 对**已有的**测量/脉冲结果做判定
 *----------------------------------------------------------------------------
 * 这条命令**没有状态机**：它就是把"上一个测量快照（相电压 + 母线）"
 * 和"上一个捕获窗口的脉冲结果（占空比）"喂给 `phase_chk` 判一遍。
 * 采集本身由已有命令负责（PC 按顺序发）：
 *   ① 0x0011 透传 DUT_PWM    → 让被测板输出已知 PWM（由被测板固件实现）
 *   ② 0x000F mode=1          → 开窗采集（含 ARM 两端同窗）
 *   ③ 0x0016 mode=1 duty=... → 判定（本命令）
 * 这样治具不用再养一套编排状态机，而 ④ 的判据（V ≈ D × 母线）就有了出口。
 *============================================================================*/
static void APP_PhaseJudgeCmd(uint16_t cmd, const uint8_t *data, uint16_t len)
{
    uint8_t         out[MGR_PHASE_JUDGE_RESP_LEN];
    phase_chk_in_t  ins[3];
    phase_chk_out_t outs[3];
    meas_snapshot_t snap;
    pulse_snapshot_t ps;
    uint8_t         mode;
    uint16_t        duty[3];
    uint16_t        sum;
    uint8_t         i;

    if ((len != MGR_PHASE_JUDGE_REQ_LEN) || (data == 0)) {
        uint8_t e = APP_STATUS_ERR_PARAM;
        APP_SendAck(cmd, &e, 1u);
        return;
    }
    mode = data[0];
    if ((mode != MGR_PHASE_MODE_STATIC) && (mode != MGR_PHASE_MODE_PWM)) {
        uint8_t e = APP_STATUS_ERR_PARAM;
        APP_SendAck(cmd, &e, 1u);
        return;
    }
    for (i = 0u; i < 3u; i++) {
        duty[i] = (uint16_t)(((uint16_t)data[1u + (i * 2u)] << 8) | data[2u + (i * 2u)]);
    }

    MEAS_Snapshot(&snap);
    PULSE_Snapshot(&ps);

    for (i = 0u; i < 3u; i++) {
        uint16_t ep = (duty[i] == MGR_PHASE_DUTY_HIZ) ? PHASE_CHK_LVL_HIZ : duty[i];
        /* 高阻那一路不查脉冲（节点浮空，查了必误报）；PWM 模式才查 */
        uint8_t  epwm = (uint8_t)((mode == MGR_PHASE_MODE_PWM) &&
                                  (ep != PHASE_CHK_LVL_HIZ));
        PhaseChk_Make(&snap, &ps, i, ep, epwm, &ins[i]);
    }
    sum = PhaseChk_All(ins, outs);

    /* 应答：状态(1) + 三相×5（实测mV、期望mV、旗标）
     * 不再发：汇总字/相数（看旗标就有）、容差（由母线电压算得出）、
     *         占空比（PC 刚发过）、差值（实测 − 期望就有）*/
    out[0] = ((sum & PHASE_CHK_SUM_ANY_BAD) != 0u) ? APP_STATUS_ERR_TEST : APP_STATUS_OK;
    for (i = 0u; i < 3u; i++) {
        uint8_t *p = &out[1u + ((uint16_t)i * MGR_PHASE_ITEM_LEN)];
        uint32_t meas = ins[i].vph_mv;
        uint16_t mv   = (meas > 0xFFFFu) ? 0xFFFFu : (uint16_t)meas;
        uint16_t ex   = (outs[i].expect_mv > 0xFFFFu) ? 0xFFFFu : (uint16_t)outs[i].expect_mv;

        p[0] = (uint8_t)(mv >> 8);  p[1] = (uint8_t)mv;
        p[2] = (uint8_t)(ex >> 8);  p[3] = (uint8_t)ex;
        p[4] = (uint8_t)(outs[i].flags & 0xFFu);
    }
    APP_SendAck(cmd, out, MGR_PHASE_JUDGE_RESP_LEN);
}

/*==============================================================================
 * 0x0017 治具自检 / 在线状态
 *----------------------------------------------------------------------------
 * 上位机"设备连接"页用：一次问清"治具在不在、U2 在不在、链路有没有错"。
 * 纯读、无副作用、不依赖 JIG_DUT_LINK，任何时候都能问（非 IDLE 也能问，
 * 因为它不改状态机 —— 这正是它比"发一条业务命令试探"好的地方）。
 *============================================================================*/
static void APP_JigStatusCmd(uint16_t cmd, const uint8_t *data, uint16_t len)
{
    uint8_t        out[MGR_JIG_STATUS_RESP_LEN];
    uint8_t        flags = 0u;
    uint16_t       ok = 0u, fail = 0u;
    link_status_t  u2st;
    const link_stats_t *ls;
    const uart_stats_t *us;

    (void)data;
    if (len != 0u) {
        uint8_t e = APP_STATUS_ERR_PARAM;
        APP_SendAck(cmd, &e, 1u);
        return;
    }
    memset(out, 0, sizeof(out));

    out[1] = (uint8_t)app_state;

    if (APP_U2Alive() != 0u)         flags |= MGR_JIG_ST_U2_ONLINE;
    if (APP_U2VerOk() != 0u)         flags |= MGR_JIG_ST_U2_VER_SAME;
    if (APP_U2Busy() != 0u)         flags |= MGR_JIG_ST_LINK_BUSY;
#if (JIG_DUT_LINK != 0)
    flags |= MGR_JIG_ST_DUT_LINK;
#endif
    out[2] = flags;

    memset(&u2st, 0, sizeof(u2st));
    (void)APP_U2Status(&u2st);                 /* 没收到过 STATUS 时保持全 0 */
    out[3]  = (uint8_t)(u2st.code_ver >> 8);
    out[4]  = (uint8_t)(u2st.code_ver);
    out[5]  = (uint8_t)(u2st.caps >> 8);
    out[6]  = (uint8_t)(u2st.caps);
    out[7]  = (uint8_t)(u2st.alive >> 8);
    out[8]  = (uint8_t)(u2st.alive);

    APP_U2ProbeStats(&ok, &fail);
    out[9]  = (uint8_t)(ok >> 8);    out[10] = (uint8_t)(ok);
    out[11] = (uint8_t)(fail >> 8);  out[12] = (uint8_t)(fail);

    ls = APP_U2Stats();
    if (ls != 0) {
        out[13] = (uint8_t)((uint32_t)ls->rx_frames >> 24);
        out[14] = (uint8_t)((uint32_t)ls->rx_frames >> 16);
        out[15] = (uint8_t)((uint32_t)ls->rx_frames >> 8);
        out[16] = (uint8_t)(ls->rx_frames);
        out[17] = (uint8_t)((uint32_t)ls->rx_crc_err >> 24);
        out[18] = (uint8_t)((uint32_t)ls->rx_crc_err >> 16);
        out[19] = (uint8_t)((uint32_t)ls->rx_crc_err >> 8);
        out[20] = (uint8_t)(ls->rx_crc_err);
        out[21] = (uint8_t)((uint16_t)ls->tx_timeout >> 8);
        out[22] = (uint8_t)(ls->tx_timeout);
        out[23] = (uint8_t)((uint16_t)ls->tx_busy >> 8);
        out[24] = (uint8_t)(ls->tx_busy);
    }
    us = UartPort_Stats(UART_CFG_PORT_LINK);
    if (us != 0) {
        out[25] = (uint8_t)((uint16_t)us->rx_drop >> 8);
        out[26] = (uint8_t)(us->rx_drop);
        out[27] = (uint8_t)((uint16_t)us->tx_err >> 8);
        out[28] = (uint8_t)(us->tx_err);
        out[29] = (uint8_t)((uint16_t)u2st.rx_drop >> 8);
        out[30] = (uint8_t)(u2st.rx_drop);
    } else {
        out[29] = (uint8_t)((uint16_t)u2st.rx_drop >> 8);
        out[30] = (uint8_t)(u2st.rx_drop);
    }
    out[0]  = APP_STATUS_OK;
    out[31] = 0u;                              /* 保留 */
    APP_SendAck(cmd, out, MGR_JIG_STATUS_RESP_LEN);
}

/*==============================================================================
 * 0x0018 ⑤ 编码器判定：三方对齐（治具意图 / U2 实测 / DUT 上报）
 *----------------------------------------------------------------------------
 * 判据层是纯函数（enc_chk.c，可宿主穷举），这里只负责拆包/打包。
 * 上位机手里本来就有这三组数：0x0012 应答里的翻转数、0x0013 读到的 U2 计数、
 * 0x0011 透传 0x0208 读到的 DUT 计数 —— 所以本命令**不缓存任何状态**。
 *============================================================================*/
static void APP_EncJudgeCmd(uint16_t cmd, const uint8_t *data, uint16_t len)
{
    enc_chk_in_t  in[ENC_CHK_PH_COUNT];
    enc_chk_out_t outs[ENC_CHK_PH_COUNT];
    uint8_t  out[MGR_ENC_JUDGE_RESP_LEN];
    uint16_t pulses[ENC_CHK_PH_COUNT];
    uint32_t tol;
    uint16_t u2_lost;
    int8_t   dut_dir;
    uint8_t  av;
    uint8_t  i;
    uint16_t sum;

    if ((len != MGR_ENC_JUDGE_REQ_LEN) || (data == 0)) {
        uint8_t e = APP_STATUS_ERR_PARAM;
        APP_SendAck(cmd, &e, 1u);
        return;
    }
    av = data[35];
    if ((av & (uint8_t)~(MGR_ENC_JUDGE_AV_U2 | MGR_ENC_JUDGE_AV_DUT)) != 0u) {
        uint8_t e = APP_STATUS_ERR_PARAM;
        APP_SendAck(cmd, &e, 1u);
        return;
    }
    if ((data[36] != 0u) || (data[37] != 0u)) {          /* 保留位必须为 0 */
        uint8_t e = APP_STATUS_ERR_PARAM;
        APP_SendAck(cmd, &e, 1u);
        return;
    }

    tol = (uint32_t)(((uint16_t)data[6] << 8) | data[7]);
    u2_lost = (uint16_t)(((uint16_t)data[20] << 8) | data[21]);
    dut_dir = (int8_t)data[34];

    memset(in, 0, sizeof(in));
    for (i = 0u; i < (uint8_t)ENC_CHK_PH_COUNT; i++) {
        pulses[i] = (uint16_t)(((uint16_t)data[i * 2u] << 8) | data[(i * 2u) + 1u]);
        in[i].exp_edges = EncChk_EdgesFromPulses(pulses[i]);
        in[i].u2_edges  = ((uint32_t)data[8u + (i * 4u)] << 24) |
                          ((uint32_t)data[9u + (i * 4u)] << 16) |
                          ((uint32_t)data[10u + (i * 4u)] << 8) |
                          (uint32_t)data[11u + (i * 4u)];
        in[i].dut_edges = ((uint32_t)data[22u + (i * 4u)] << 24) |
                          ((uint32_t)data[23u + (i * 4u)] << 16) |
                          ((uint32_t)data[24u + (i * 4u)] << 8) |
                          (uint32_t)data[25u + (i * 4u)];
        in[i].has_u2  = ((av & MGR_ENC_JUDGE_AV_U2)  != 0u) ? 1u : 0u;
        in[i].has_dut = ((av & MGR_ENC_JUDGE_AV_DUT) != 0u) ? 1u : 0u;
    }

    memset(outs, 0, sizeof(outs));
    sum = EncChk_All(in, tol, u2_lost, dut_dir, outs);

    /* 应答：状态(1) + 方向(1) + 对侧结论(1) + 三相×3（被测板边沿数、旗标）
     * 不再发：汇总字/相数/预留、U2 丢沿、每相的期望边沿（治具默认脉冲串固定：
     *         A=1000 脉冲→2000 边沿、B=2000→4000、Z=1→2）、U2 每相的数（只留结论）*/
    out[0] = ((sum & ENC_CHK_SUM_ANY_BAD) != 0u) ? APP_STATUS_ERR_TEST : APP_STATUS_OK;
    out[1] = (uint8_t)dut_dir;
    {
        uint8_t peer = 0u;
        if ((av & MGR_ENC_JUDGE_AV_U2) != 0u) {
            peer = 1u;                              /* 有对侧数据，先按"一致" */
            for (i = 0u; i < (uint8_t)ENC_CHK_PH_COUNT; i++) {
                if ((outs[i].flags & ENC_CHK_F_U2_DIFF) != 0u) {
                    peer = 2u;                      /* 有一相不一致 */
                    break;
                }
            }
        }
        out[2] = peer;
    }
    for (i = 0u; i < (uint8_t)ENC_CHK_PH_COUNT; i++) {
        uint8_t *p = &out[3u + ((uint16_t)i * MGR_ENC_JUDGE_ITEM_LEN)];
        uint16_t dt = (outs[i].dut_edges > 0xFFFFu) ? 0xFFFFu : (uint16_t)outs[i].dut_edges;

        p[0] = (uint8_t)(dt >> 8);
        p[1] = (uint8_t)(dt);
        p[2] = (uint8_t)(outs[i].flags & 0xFFu);
    }
    (void)u2_lost;                                  /* 留作诊断，不上报（见 enc_chk.h）*/
    APP_SendAck(cmd, out, MGR_ENC_JUDGE_RESP_LEN);
}

/*==============================================================================
 * 0x0010 一键全流程（① 进测试模式 → ② → ③④ → ⑤ → ⑥ → STOP）
 *----------------------------------------------------------------------------
 * 设计取舍（重要）：
 *   1) **不重写子流程**：直接复用已经测过的 0x0015 / 0x000F / 0x0016 / 0x0012 /
 *      0x0013 / 0x0014 和 DUT 透传。子流程照原样跑、照原样把自己的应答发给 PC，
 *      本命令只负责"按顺序点火 + 采集每步结果 + 最后汇总"。
 *      → 好处：子流程的行为和单独调用时**完全一致**，不会出现"两套实现两种结果"。
 *      → 代价：PC 在收到 0x0010 的汇总前，会先收到各子命令的应答（同 SEQ）；
 *              上位机可以收下显示、也可以忽略（docs/17 已写明）。
 *   2) 每步都有独立超时；某一步超时/失败**不中断整体**（继续跑完，汇总里标出来），
 *      这样一次压合就能拿到全貌。只有 PC 发 0x000E/0x000F 那类长命令同理。
 *   3) ⑤ 的判定在**本文件内**用 enc_chk 做完（步骤 8 的 U2 数 + 步骤 9 的 DUT 数
 *      正好都在 s_encp_ack / s_dut_ack 里），所以一键跑完就自带 ⑤ 的结论。
 *============================================================================*/
#if (JIG_DUT_LINK != 0)

#define APP_RUN_STEP_TIMEOUT_MS  4000u    /* 单步上限（0x000F 的 200ms 窗口等都在里面） */
#define APP_RUN_WDT_MS           4000u    /* DUT 失联看门狗：比单步超时略长，避免自己把自己关断 */

#define APP_RUN_STEP_TESTMODE   0u
#define APP_RUN_STEP_VOL        1u
#define APP_RUN_STEP_PWM        2u
#define APP_RUN_STEP_PULSE      3u
#define APP_RUN_STEP_PHASE      4u
#define APP_RUN_STEP_TESTMODE2  5u        /* 再进一次：让 DUT 把编码器计数清零 */
#define APP_RUN_STEP_ENC_CLR    6u
#define APP_RUN_STEP_ENC_GEN    7u
#define APP_RUN_STEP_ENC_READ   8u
#define APP_RUN_STEP_ENC_DUT    9u        /* 读 DUT 的计数（顺便在这里判 ⑤） */
#define APP_RUN_STEP_CUR        10u
#define APP_RUN_STEP_STOP       11u

static uint8_t  s_run_active;                                  /* 1 = 编排中 */
static uint8_t  s_run_waiting;                                 /* 1 = 上一步已点火、等它收尾 */
static uint8_t  s_run_step;
static uint16_t s_run_seq;
static uint32_t s_run_t0;
static uint8_t  s_run_res[MGR_RUN_ALL_STEP_COUNT][2];          /* [步骤][状态, 结果码] */
static uint8_t  s_run_dut[2u + 16u];                           /* 组 DUT 命令用 */

/* 组一条 DUT 命令并交给透传通道（失败会立刻回 BUSY/0B，由 s_last_ack_* 采集） */
static void APP_RunDut(uint16_t dcmd, const uint8_t *d, uint8_t n)
{
    s_run_dut[0] = (uint8_t)(dcmd >> 8);
    s_run_dut[1] = (uint8_t)(dcmd & 0xFFu);
    if ((d != 0) && (n != 0u)) {
        memcpy(&s_run_dut[2], d, n);
    }
    APP_DutCmd(MGR_CMD_DUT_RAW, s_run_dut, (uint16_t)(2u + n));
}

/* ⑤ 的三方比对（用步骤 8/9 的应答 + 默认脉冲参数） */
static uint16_t APP_RunAllEncJudge(void)
{
    enc_chk_in_t in[ENC_CHK_PH_COUNT];
    enc_plan_t   plan;
    uint8_t      i;
    uint8_t      u2_ok;
    uint8_t      dut_ok;
    uint16_t     lost;

    EncPlan_Default(&plan);

    /* 步骤 8：s_encp_ack[0]=治具状态, [1]=U2 状态, [2..13]=a/b/z(u32), [14..15]=lost */
    u2_ok  = ((s_encp_ack[0] == APP_STATUS_OK) && (s_encp_ack[1] == APP_STATUS_OK)) ? 1u : 0u;
    /* 步骤 9：s_dut_ack[0]=治具状态, [1]=DUT 状态, [2..13]=a/b/z(u32), [14]=dir */
    dut_ok = ((s_dut_ack[0] == APP_STATUS_OK) && (s_dut_ack[1] == APP_STATUS_OK)) ? 1u : 0u;

    memset(in, 0, sizeof(in));
    for (i = 0u; i < (uint8_t)ENC_CHK_PH_COUNT; i++) {
        uint16_t pulses = (i == (uint8_t)ENC_CHK_PH_A) ? plan.cnt_a
                        : (i == (uint8_t)ENC_CHK_PH_B) ? plan.cnt_b : plan.cnt_z;
        uint8_t  o = (uint8_t)(2u + (i * 4u));

        in[i].exp_edges = EncChk_EdgesFromPulses(pulses);
        in[i].has_u2    = u2_ok;
        in[i].has_dut   = dut_ok;
        in[i].u2_edges  = ((uint32_t)s_encp_ack[o] << 24) | ((uint32_t)s_encp_ack[o + 1u] << 16) |
                          ((uint32_t)s_encp_ack[o + 2u] << 8) | (uint32_t)s_encp_ack[o + 3u];
        in[i].dut_edges = ((uint32_t)s_dut_ack[o] << 24) | ((uint32_t)s_dut_ack[o + 1u] << 16) |
                          ((uint32_t)s_dut_ack[o + 2u] << 8) | (uint32_t)s_dut_ack[o + 3u];
    }
    lost = (uint16_t)(((uint16_t)s_encp_ack[14] << 8) | s_encp_ack[15]);
    return EncChk_All(in, ENC_CHK_TOL_EDGES, lost, (int8_t)s_dut_ack[14], 0);
}

static void APP_RunAllFinish(void)
{
    uint8_t out[MGR_RUN_ALL_RESP_LEN];
    uint8_t i;
    uint8_t any_bad = 0u;
    uint8_t failed  = 0u;

    for (i = 0u; i < (uint8_t)MGR_RUN_ALL_STEP_COUNT; i++) {
        if ((s_run_res[i][0] == MGR_RUN_ST_FAIL) ||
            (s_run_res[i][0] == MGR_RUN_ST_TIMEOUT)) {
            if (any_bad == 0u) {
                failed = (uint8_t)(i + 1u);          /* 1 基，方便对着 docs 看 */
            }
            any_bad = 1u;
        }
    }
    memset(out, 0, sizeof(out));
    out[0] = (any_bad != 0u) ? APP_STATUS_ERR_TEST : APP_STATUS_OK;
    out[1] = (uint8_t)MGR_RUN_ALL_STEP_COUNT;
    out[2] = (s_run_step > (uint8_t)MGR_RUN_ALL_STEP_COUNT)
             ? (uint8_t)MGR_RUN_ALL_STEP_COUNT : s_run_step;
    out[3] = failed;
    for (i = 0u; i < (uint8_t)MGR_RUN_ALL_STEP_COUNT; i++) {
        out[4u + ((uint16_t)i * MGR_RUN_ALL_STEP_LEN)]     = s_run_res[i][0];
        out[5u + ((uint16_t)i * MGR_RUN_ALL_STEP_LEN)]     = s_run_res[i][1];
    }
    s_run_active = 0u;
    s_run_waiting = 0u;
    app_state   = APP_STATE_IDLE;
    APP_SendFrame(MGR_CMD_RUN_ALL, s_run_seq, out, MGR_RUN_ALL_RESP_LEN);
}

/* 启动第 n 步；返回 1 = 异步（要等它回 IDLE），0 = 同步已出结果 */
static uint8_t APP_RunStartStep(uint8_t n)
{
    uint8_t d[12];

    switch (n) {
    case APP_RUN_STEP_TESTMODE:
    case APP_RUN_STEP_TESTMODE2:
        d[0] = 1u;                                   /* mode = 进入 */
        d[1] = (uint8_t)(APP_RUN_WDT_MS >> 8);
        d[2] = (uint8_t)(APP_RUN_WDT_MS);
        APP_RunDut(DUT_CMD_TEST_MODE, d, 3u);
        return 1u;

    case APP_RUN_STEP_VOL:
        APP_VolTestCmd(MGR_CMD_VOL_TEST, 0, 0u);
        return 1u;

    case APP_RUN_STEP_PWM:
        /* 1 kHz、三相 50%、300 个 → burst 300 ms（≥ 治具 200 ms 窗口 + 余量） */
        d[0] = 0x03u; d[1] = 0xE8u;
        d[2] = 0x01u; d[3] = 0xF4u;
        d[4] = 0x01u; d[5] = 0xF4u;
        d[6] = 0x01u; d[7] = 0xF4u;
        d[8] = 0x01u; d[9] = 0x2Cu;
        APP_RunDut(DUT_CMD_PWM, d, 10u);
        return 1u;

    case APP_RUN_STEP_PULSE:
        d[0] = MGR_MODE_TEST;
        APP_PulseCmd(MGR_CMD_PULSE, d, 1u);
        return 1u;

    case APP_RUN_STEP_PHASE: {
        uint8_t p[MGR_PHASE_JUDGE_REQ_LEN];
        p[0] = MGR_PHASE_MODE_PWM;
        p[1] = 0x01u; p[2] = 0xF4u;
        p[3] = 0x01u; p[4] = 0xF4u;
        p[5] = 0x01u; p[6] = 0xF4u;
        APP_PhaseJudgeCmd(MGR_CMD_PHASE_JUDGE, p, MGR_PHASE_JUDGE_REQ_LEN);
        return 0u;                                   /* 同步出结果 */
    }

    case APP_RUN_STEP_ENC_CLR:
        d[0] = LINK_ENC_ACT_CLEAR;
        APP_EncPeerCmd(MGR_CMD_ENC_PEER, d, 1u);
        return 1u;

    case APP_RUN_STEP_ENC_GEN:
        APP_EncGenCmd(MGR_CMD_ENC_GEN, 0, 0u);        /* LEN=0 → 默认参数 */
        return 1u;

    case APP_RUN_STEP_ENC_READ:
        d[0] = LINK_ENC_ACT_READ;
        APP_EncPeerCmd(MGR_CMD_ENC_PEER, d, 1u);
        return 1u;

    case APP_RUN_STEP_ENC_DUT:
        APP_RunDut(DUT_CMD_GET_ENC, 0, 0u);
        return 1u;

    case APP_RUN_STEP_CUR:
        /* ⚠ 这里必须调**内层**的 APP_CurStartMachine，不能调 0x0014 的命令入口：
         *   0x0014 现在自己会起一条独立流程（进测试模式+关输出），
         *   从 RUN_ALL 里调它会把控制权抢走（两套流程抢同一个状态机）。 */
        APP_CurStartMachine();
        return 1u;

    case APP_RUN_STEP_STOP:
        APP_RunDut(DUT_CMD_STOP, 0, 0u);
        return 1u;

    default:
        return 0u;
    }
}

/* 记录某一步的结果；第 9 步额外做 ⑤ 的三方比对 */
static void APP_RunStepDone(uint8_t step)
{
    uint8_t st = (s_last_ack_st == APP_STATUS_OK) ? MGR_RUN_ST_OK : MGR_RUN_ST_FAIL;

    s_run_res[step][0] = st;
    s_run_res[step][1] = s_last_ack_st;

    if (step == (uint8_t)APP_RUN_STEP_ENC_DUT) {
        uint16_t sum = APP_RunAllEncJudge();

        s_run_res[step][0] = ((sum & ENC_CHK_SUM_ANY_BAD) != 0u) ? MGR_RUN_ST_FAIL
                            : ((sum & ENC_CHK_SUM_ANY_NODATA) != 0u) ? MGR_RUN_ST_SKIP
                            : MGR_RUN_ST_OK;
        s_run_res[step][1] = (uint8_t)(sum & 0xFFu);
    }
}

static void APP_RunAllTask(void)
{
    if (s_run_active == 0u) {
        return;
    }

    /* 上一轮点过火的那一步：等它回到 IDLE（或超时） */
    if (s_run_waiting != 0u) {
        if ((app_state != APP_STATE_IDLE) && (app_state != APP_STATE_RUN)) {
            if ((uint32_t)(HAL_GetTick() - s_run_t0) > APP_RUN_STEP_TIMEOUT_MS) {
                s_run_res[s_run_step][0] = MGR_RUN_ST_TIMEOUT;
                s_run_res[s_run_step][1] = (uint8_t)(s_run_step + 1u);
                s_run_step++;
                s_run_waiting = 0u;
            } else {
                return;                              /* 还在跑 */
            }
        } else {
            APP_RunStepDone(s_run_step);
            s_run_step++;
            s_run_waiting = 0u;
        }
    }

    while (s_run_step < (uint8_t)MGR_RUN_ALL_STEP_COUNT) {
        app_state = APP_STATE_RUN;
        s_run_t0  = HAL_GetTick();
        if (APP_RunStartStep(s_run_step) == 0u) {
            APP_RunStepDone(s_run_step);             /* 同步步骤 */
            s_run_step++;
            continue;
        }
        if (app_state == APP_STATE_RUN) {
            /* 子步骤同步就拒绝了（BUSY / 参数错），不会再有终态 */
            s_run_res[s_run_step][0] = MGR_RUN_ST_FAIL;
            s_run_res[s_run_step][1] = s_last_ack_st;
            s_run_step++;
            continue;
        }
        s_run_waiting = 1u;                          /* 异步：下一轮再看 */
        return;
    }
    APP_RunAllFinish();
}

static void APP_RunAllCmd(uint16_t cmd, const uint8_t *data, uint16_t len)
{
    (void)data;
    if (len != 0u) {
        uint8_t e = APP_STATUS_ERR_PARAM;
        APP_SendAck(cmd, &e, 1u);
        return;
    }
    memset(s_run_res, 0, sizeof(s_run_res));         /* 全 IDLE */
    s_run_active  = 1u;
    s_run_waiting = 0u;
    s_run_step    = 0u;
    s_run_seq     = pc_seq;
    s_run_t0      = HAL_GetTick();
    /* 清掉内部链路上可能残留的保活探测（同 0x0013 的理由） */
    APP_U2Flush();
    app_state = APP_STATE_RUN;
}

#endif /* JIG_DUT_LINK */

/*==============================================================================
 * 单项"一条命令跑完"：0x0019 ③ / 0x001A ④ / 0x001B ⑤（⑥ 的 0x0014 也走这套）
 *----------------------------------------------------------------------------
 * 骨架统一（每个 kind 只是"中间那几步"不同）：
 *     第 1 步  进测试模式           中间若干步  激励/采集/判定        最后一步  关输出
 * 中间步骤**全部复用已有实现**（0x000F / 0x0016 / 0x0012 / 0x0013 / 0x0018 / ⑥的内层状态机），
 * 一行测量逻辑都不重写。
 * 流程进行中不把子命令的应答发给 PC（APP_SendFrame 里拦掉），
 * 只把"那一份报告"原样回给你 → **一条命令 = 一条应答**，上位机不用认谁是谁。
 *============================================================================*/
#if (JIG_DUT_LINK != 0)

static uint8_t APP_FlowSteps(uint8_t kind)
{
    switch (kind) {
    case APP_FLOW_KIND_CUR:   return 3u;   /* TM → ⑥内层 → STOP */
    case APP_FLOW_KIND_PULSE: return 4u;   /* TM → PWM → 采集 → STOP */
    case APP_FLOW_KIND_PHASE: return 5u;   /* TM → PWM → 采集 → 判定 → STOP */
    case APP_FLOW_KIND_ENC:   return 7u;   /* TM → 清零 → 发 → 读U2 → 读DUT → 判定 → STOP */
    default:                  return 0u;
    }
}

static uint8_t APP_FlowRepStep(uint8_t kind)
{
    switch (kind) {
    case APP_FLOW_KIND_CUR:   return 1u;
    case APP_FLOW_KIND_PULSE: return 2u;
    case APP_FLOW_KIND_PHASE: return 3u;
    case APP_FLOW_KIND_ENC:   return 5u;
    default:                  return 0u;
    }
}

static void APP_FlowDut(uint16_t dcmd, const uint8_t *d, uint8_t n)
{
    s_flow_buf[0] = (uint8_t)(dcmd >> 8);
    s_flow_buf[1] = (uint8_t)(dcmd & 0xFFu);
    if ((d != 0) && (n != 0u)) {
        memcpy(&s_flow_buf[2], d, n);
    }
    APP_DutCmd(MGR_CMD_DUT_RAW, s_flow_buf, (uint16_t)(2u + n));
}

/* ⑤ 第 5 步：把 U2 与 DUT 的数拼成 0x0018 的 38B 请求（判据还是那一个函数） */
static void APP_FlowEncJudgeReq(uint8_t *r)
{
    enc_plan_t plan;
    uint8_t i;

    memset(r, 0, MGR_ENC_JUDGE_REQ_LEN);
    EncPlan_Default(&plan);
    r[0] = (uint8_t)(plan.cnt_a >> 8);  r[1] = (uint8_t)plan.cnt_a;
    r[2] = (uint8_t)(plan.cnt_b >> 8);  r[3] = (uint8_t)plan.cnt_b;
    r[4] = (uint8_t)(plan.cnt_z >> 8);  r[5] = (uint8_t)plan.cnt_z;
    for (i = 0u; i < 3u; i++) {
        uint8_t o = (uint8_t)(2u + (i * 4u));

        r[8u + (i * 4u)]  = s_encp_ack[o];
        r[9u + (i * 4u)]  = s_encp_ack[o + 1u];
        r[10u + (i * 4u)] = s_encp_ack[o + 2u];
        r[11u + (i * 4u)] = s_encp_ack[o + 3u];
        r[22u + (i * 4u)] = s_dut_ack[o];
        r[23u + (i * 4u)] = s_dut_ack[o + 1u];
        r[24u + (i * 4u)] = s_dut_ack[o + 2u];
        r[25u + (i * 4u)] = s_dut_ack[o + 3u];
    }
    r[20] = s_encp_ack[14];             /* u2_lost 高 */
    r[21] = s_encp_ack[15];             /* u2_lost 低 */
    r[34] = s_dut_ack[14];              /* dut_dir */
    r[35] = (uint8_t)(MGR_ENC_JUDGE_AV_U2 | MGR_ENC_JUDGE_AV_DUT);
}

/* 启动第 step 步；返回 1 = 异步（要等它回 IDLE），0 = 同步已出结果 */
static uint8_t APP_FlowStep(uint8_t step)
{
    uint8_t d[12];

    if (step == 0u) {                                   /* 所有流程的第 1 步 */
        d[0] = 1u;
        d[1] = (uint8_t)(MGR_FLOW_WDT_MS >> 8);
        d[2] = (uint8_t)(MGR_FLOW_WDT_MS);
        APP_FlowDut(DUT_CMD_TEST_MODE, d, 3u);
        return 1u;
    }
    if (step == (uint8_t)(s_flow_steps - 1u)) {         /* 所有流程的最后一步 */
        APP_FlowDut(DUT_CMD_STOP, 0, 0u);
        return 1u;
    }

    switch (s_flow_kind) {
    case APP_FLOW_KIND_CUR:
        APP_CurStartMachine();
        return 1u;

    case APP_FLOW_KIND_PULSE:
    case APP_FLOW_KIND_PHASE:
        if (step == 1u) {                               /* 让 DUT 输出已知 PWM */
            d[0] = 0x03u; d[1] = 0xE8u;                 /* 1 kHz */
            d[2] = 0x01u; d[3] = 0xF4u;                 /* A 50% */
            d[4] = 0x01u; d[5] = 0xF4u;                 /* B 50% */
            d[6] = 0x01u; d[7] = 0xF4u;                 /* C 50% */
            d[8] = 0x01u; d[9] = 0x2Cu;                 /* 300 个 → 300 ms */
            APP_FlowDut(DUT_CMD_PWM, d, 10u);
            return 1u;
        }
        if (step == 2u) {                               /* 采集 */
            d[0] = MGR_MODE_TEST;
            APP_PulseCmd(MGR_CMD_PULSE, d, 1u);
            return 1u;
        }
        if ((s_flow_kind == APP_FLOW_KIND_PHASE) && (step == 3u)) {
            uint8_t p[MGR_PHASE_JUDGE_REQ_LEN];

            p[0] = MGR_PHASE_MODE_PWM;
            p[1] = 0x01u; p[2] = 0xF4u;
            p[3] = 0x01u; p[4] = 0xF4u;
            p[5] = 0x01u; p[6] = 0xF4u;
            APP_PhaseJudgeCmd(MGR_CMD_PHASE_JUDGE, p, MGR_PHASE_JUDGE_REQ_LEN);
            return 0u;                                  /* 同步 */
        }
        break;

    case APP_FLOW_KIND_ENC:
        if (step == 1u) {
            d[0] = LINK_ENC_ACT_CLEAR;
            APP_EncPeerCmd(MGR_CMD_ENC_PEER, d, 1u);
            return 1u;
        }
        if (step == 2u) {
            APP_EncGenCmd(MGR_CMD_ENC_GEN, 0, 0u);      /* LEN=0 → 默认脉冲串 */
            return 1u;
        }
        if (step == 3u) {
            d[0] = LINK_ENC_ACT_READ;
            APP_EncPeerCmd(MGR_CMD_ENC_PEER, d, 1u);
            return 1u;
        }
        if (step == 4u) {
            APP_FlowDut(DUT_CMD_GET_ENC, 0, 0u);
            return 1u;
        }
        if (step == 5u) {
            uint8_t r[MGR_ENC_JUDGE_REQ_LEN];

            APP_FlowEncJudgeReq(r);
            APP_EncJudgeCmd(MGR_CMD_ENC_JUDGE, r, MGR_ENC_JUDGE_REQ_LEN);
            return 0u;                                  /* 同步 */
        }
        break;

    default:
        break;
    }
    return 0u;
}

/* 一步收尾：如果这一步正是"要回给 PC 的那份报告"，留一份 */
static void APP_FlowStepDone(void)
{
    if ((s_flow_step == s_flow_rep_step) && (s_flow_cap_len > 0u)) {
        memcpy(s_flow_rep, s_flow_cap, s_flow_cap_len);
        s_flow_rep_len = s_flow_cap_len;
    }
}

static void APP_FlowFinish(void)
{
    uint8_t st = s_flow_res;

    s_flow_active = 0u;
    s_flow_wait   = 0u;
    app_state     = APP_STATE_IDLE;
    if (s_flow_rep_len == 0u) {
        /* 一个可用报告都没拿到（比如头一步就失败）：回 1 字节状态 */
        uint8_t e = (st != APP_STATUS_OK) ? st : APP_STATUS_ERR_TIMEOUT;

        APP_SendFrame(s_flow_cmd, s_flow_seq, &e, 1u);
        return;
    }
    /* 报告原样回给 PC（里面的状态字节就是判定结果：00 合格 / 05 不合格） */
    APP_SendFrame(s_flow_cmd, s_flow_seq, s_flow_rep, s_flow_rep_len);
}

static void APP_FlowTask(void)
{
    if (s_flow_active == 0u) {
        return;
    }

    if (s_flow_wait != 0u) {
        if ((app_state != APP_STATE_IDLE) && (app_state != APP_STATE_RUN)) {
            if ((uint32_t)(HAL_GetTick() - s_flow_t0) > MGR_FLOW_STEP_TIMEOUT_MS) {
                s_flow_res  = APP_STATUS_ERR_TIMEOUT;
                s_flow_wait = 0u;
                s_flow_step++;
            } else {
                return;                                  /* 还在跑 */
            }
        } else {
            APP_FlowStepDone();
            s_flow_wait = 0u;
            s_flow_step++;
        }
    }

    while (s_flow_step < s_flow_steps) {
        uint8_t async;

        app_state = APP_STATE_RUN;
        s_flow_t0 = HAL_GetTick();
        async     = APP_FlowStep(s_flow_step);
        if (async == 0u) {
            APP_FlowStepDone();                          /* 同步步骤 */
            s_flow_step++;
            continue;
        }
        if (app_state == APP_STATE_RUN) {
            s_flow_res = s_last_ack_st;                  /* 子步骤同步就失败了 */
            s_flow_step++;
            continue;
        }
        s_flow_wait = 1u;
        return;
    }
    APP_FlowFinish();
}

static void APP_FlowStart(uint8_t kind, uint16_t cmd)
{
    s_flow_kind     = kind;
    s_flow_steps    = APP_FlowSteps(kind);
    s_flow_rep_step = APP_FlowRepStep(kind);
    s_flow_cmd      = cmd;
    s_flow_seq      = pc_seq;
    s_flow_step     = 0u;
    s_flow_wait     = 0u;
    s_flow_res      = APP_STATUS_ERR_TIMEOUT;
    s_flow_rep_len  = 0u;
    s_flow_cap_len  = 0u;
    s_flow_t0       = HAL_GetTick();
    s_flow_active   = 1u;
    app_state       = APP_STATE_RUN;
}

/* 三条"一条命令"的入口（都不收参数，全部用默认值） */
static void APP_PulseTestCmd(uint16_t cmd, const uint8_t *data, uint16_t len)
{
    (void)data;
    if (len != 0u) {
        uint8_t e = APP_STATUS_ERR_PARAM;
        APP_SendAck(cmd, &e, 1u);
        return;
    }
    APP_FlowStart(APP_FLOW_KIND_PULSE, cmd);
}

static void APP_PhaseTestCmd(uint16_t cmd, const uint8_t *data, uint16_t len)
{
    (void)data;
    if (len != 0u) {
        uint8_t e = APP_STATUS_ERR_PARAM;
        APP_SendAck(cmd, &e, 1u);
        return;
    }
    APP_FlowStart(APP_FLOW_KIND_PHASE, cmd);
}

static void APP_EncTestCmd(uint16_t cmd, const uint8_t *data, uint16_t len)
{
    (void)data;
    if (len != 0u) {
        uint8_t e = APP_STATUS_ERR_PARAM;
        APP_SendAck(cmd, &e, 1u);
        return;
    }
    APP_FlowStart(APP_FLOW_KIND_ENC, cmd);
}

#endif /* JIG_DUT_LINK */

/*==============================================================================
 * 板子 SN 主动上报（0x0024）
 *----------------------------------------------------------------------------
 * 烧录成功后读被测板 96-bit UID（RM0090 Unique device ID，F405 在 0x1FFF7A10），
 * 组 16B（算法版本 + UID + XOR + 预留）后以**自发帧**（SEQ=0）推给 PC。
 * 三条约定：读 UID 失败静默跳过 / 同板不重复报 / 绝不影响烧录结果。
 *============================================================================*/
static void APP_SnReport(void)
{
    uint32_t w[SN_UID_WORDS];
    uint8_t  uid[SN_UID_LEN];
    uint8_t  buf[MGR_BOARD_SN_RESP_LEN];
    uint8_t  i;

    /* 读 3 个 32 位字（地址/长度都在 sn_report.h 的占位宏里，D9 定了只改那儿） */
    if (SWD_MemReadBlock(SN_UID_BASE, w, (uint32_t)SN_UID_WORDS) == 0) {
        return;                                  /* 读不到 → 静默跳过 */
    }
    for (i = 0u; i < (uint8_t)SN_UID_WORDS; i++) {
        uid[(i * 4u) + 0u] = (uint8_t)(w[i] >> 24);
        uid[(i * 4u) + 1u] = (uint8_t)(w[i] >> 16);
        uid[(i * 4u) + 2u] = (uint8_t)(w[i] >> 8);
        uid[(i * 4u) + 3u] = (uint8_t)(w[i]);
    }
    if (SnReport_IsNewBoard(uid) == 0u) {
        return;                                  /* 同一块板重复烧录 → 不重复上报 */
    }
    if (SnReport_Build(uid, buf) == 0u) {
        return;
    }
    SnReport_MarkReported(uid);
    APP_SendEvent(MGR_CMD_BOARD_SN, buf, MGR_BOARD_SN_RESP_LEN);
}

/*==============================================================================
 * 命令分发
 *============================================================================*/
static void APP_Dispatch(uint16_t cmd, const uint8_t *data, uint16_t len)
{
    /* 上传三段式命令由 fw_upload 独占处理 (内部自带 BUSY/参数校验) */
    if (FwUpload_Handle(cmd, data, len)) return;

    /* 0x0017 是**只读自检**：任何状态都要能问。
     * （上位机最需要它的时刻，往往正是治具忙着跑别的命令的时候。） */
    if (cmd == MGR_CMD_JIG_STATUS) {
        APP_JigStatusCmd(cmd, data, len);
        return;
    }

    /* 非 IDLE 状态下, 其余命令一律回 BUSY (不变量 I3) */
    if (app_state != APP_STATE_IDLE) {
        uint8_t e = APP_STATUS_BUSY;
        APP_SendAck(cmd, &e, 1);
        return;
    }

    switch (cmd) {

    /*── 0x001C 诊断: SPI Flash 连通性（排"读不到 Flash"用，不参与正常流程）──*/
    case MGR_CMD_FLASH_DIAG: {
        uint8_t d[MGR_FLASH_DIAG_RESP_LEN];
        (void)GD25Q_Diag(d);
        APP_SendAck(cmd, d, (uint16_t)MGR_FLASH_DIAG_RESP_LEN);
        break;
    }

    /*── 0x001E SWD 烧录自检（B1：不依赖 SPI Flash，单独验证 SWD 这一段）──
     * 擦一个 128KB 扇区约 1~1.5 秒，所以和 0x0112 一样"跑完才回 ACK"，
     * 终态在 APP_Run 里发。自检写的是分区外的暂存扇区，不碰 Boot/App。 */
    case MGR_CMD_SWD_SELFTEST: {
        uint8_t busy;

        /* DATA[0] = 1 → "SWD 引脚活动测试"（约 7 秒：2 秒方波 + 5 秒双线低，供万用表量）
         * DATA[0] = 2 → "SWD 速度扫描"（一次试 8 档位翻转速度）
         * DATA[0] = 3 → "SWD 波形捕捉"（抓 ACK 窗口 32 拍电平，替代示波器）
         * DATA[0] = 4 → "连续连接尝试"（5 秒内不停重连，期间按被测板复位 → 验证复位下连接）
         * DATA[0] = 5 → "DP/AP 探针"（打印 CTRL/STAT、AP IDR、DP SELECT）
         * DATA[0] = 6 → "USB 事件计数"
         * DATA[0] = 7 → "设置 SWD 位翻转延时"（DATA[1..2] = 延时值，0=最快）
         * DATA[0] = 8 → "读目标任意地址"（DATA[1..4] = 地址，返回 4 字节值）
         * 缺省/0     → 完整 SWD 烧录自检 */
        if ((len >= 1u) && (data != 0) &&
            (data[0] >= 1u) && (data[0] <= 8u)) {
            uint8_t pr[MGR_SWD_SELFTEST_RESP_LEN];

            if (data[0] == 7u) {
                /* 运行期设置位翻转延时（每半位插入的空循环数，0=最快）。
                 * 用途：一次烧录就能试多档速度，不必反复重烧治具。
                 * 注意：设置在 RAM 里，治具重启会恢复成 swd_config.h 的默认值。 */
                extern void     SWD_PhySetDelay(uint16_t loops);
                extern uint16_t SWD_PhyGetDelay(void);
                uint16_t d = (uint16_t)SWD_PhyGetDelay();
                uint8_t  k;

                if (len >= 3u) {
                    d = (uint16_t)(((uint16_t)data[1] << 8) | data[2]);
                    SWD_PhySetDelay(d);
                    SWD_PhyInit();          /* 换速度后重配引脚（幂等） */
                }
                for (k = 0u; k < (uint8_t)MGR_SWD_SELFTEST_RESP_LEN; k++) {
                    pr[k] = 0u;
                }
                pr[0] = APP_STATUS_OK;
                pr[1] = (uint8_t)(d >> 8);
                pr[2] = (uint8_t)d;
                pr[3] = 0xB6u;
                APP_SendAck(cmd, pr, (uint16_t)MGR_SWD_SELFTEST_RESP_LEN);
                break;
            }

            if (data[0] == 8u) {
                /* 读目标任意 32 位地址：连接→上电→MEM-AP→SWD_MemRead32。
                 * 典型用法：读 0x08010000（App 的 SP）应与 app*.bin 头 4 字节一致。 */
                uint32_t addr = 0u;
                uint32_t val  = 0u;
                uint8_t  st   = APP_STATUS_ERR_PARAM;
                uint8_t  k;

                for (k = 0u; k < (uint8_t)MGR_SWD_SELFTEST_RESP_LEN; k++) {
                    pr[k] = 0u;
                }
                if (len >= 5u) {
                    uint32_t id = 0u;

                    addr = ((uint32_t)data[1] << 24) | ((uint32_t)data[2] << 16) |
                           ((uint32_t)data[3] << 8)  |  (uint32_t)data[4];
                    if (SWD_Connect(&id) != SWD_OK)        { st = APP_STATUS_ERR_TIMEOUT; }
                    else if (SWD_PowerUp() != SWD_OK)      { st = APP_STATUS_ERR_TIMEOUT; }
                    else if (SWD_MemInit() != SWD_OK)      { st = APP_STATUS_ERR_TIMEOUT; }
                    else if (SWD_MemRead32(addr, &val) != SWD_OK) { st = APP_STATUS_ERR_TIMEOUT; }
                    else                                   { st = APP_STATUS_OK; }
                }
                pr[0] = st;
                pr[1] = (uint8_t)(val >> 24);
                pr[2] = (uint8_t)(val >> 16);
                pr[3] = (uint8_t)(val >> 8);
                pr[4] = (uint8_t)val;
                pr[5] = 0xB6u;
                APP_SendAck(cmd, pr, (uint16_t)MGR_SWD_SELFTEST_RESP_LEN);
                break;
            }

            if (data[0] == 6u) {
                /* USB 事件计数：由 stm32f1xx_it.c 的 USB 中断累计（非 USB 构建返回 0） */
                extern uint32_t USB_EvtIrqCount(void);
                extern uint32_t USB_EvtIstrOr(void);
                uint32_t irq  = USB_EvtIrqCount();
                uint32_t istr = USB_EvtIstrOr();
                uint8_t  k;

                for (k = 0u; k < (uint8_t)MGR_SWD_SELFTEST_RESP_LEN; k++) {
                    pr[k] = 0u;
                }
                pr[0] = APP_STATUS_OK;
                pr[1] = (uint8_t)(irq >> 24);
                pr[2] = (uint8_t)(irq >> 16);
                pr[3] = (uint8_t)(irq >> 8);
                pr[4] = (uint8_t)irq;
                pr[5] = (uint8_t)(istr >> 24);
                pr[6] = (uint8_t)(istr >> 16);
                pr[7] = (uint8_t)(istr >> 8);
                pr[8] = (uint8_t)istr;
                pr[9] = 0xB6u;          /* 本版标记 */
                APP_SendAck(cmd, pr, (uint16_t)MGR_SWD_SELFTEST_RESP_LEN);
                break;
            }

            if (data[0] == 1u) {
                SST_PinActivityTest(pr);
            } else if (data[0] == 2u) {
                SST_SpeedProbe(pr);
            } else if (data[0] == 3u) {
                SST_WaveCapture(pr);
            } else if (data[0] == 4u) {
                SST_ConnectBurst(pr);
            } else {
                SST_ApWriteCapture(pr);
            }
            APP_SendAck(cmd, pr, (uint16_t)MGR_SWD_SELFTEST_RESP_LEN);
            break;
        }

        if (SST_Busy() != 0u) {
            busy = APP_STATUS_BUSY;
            APP_SendAck(cmd, &busy, 1);
            break;
        }
        s_sst_seq = pc_seq;
        SST_Start();
        break;
    }

    /*── 0x000C 查询固件版本号 (Flash 中) ──*/
    case MGR_CMD_QUERY_FLASH_VER: {
        /* 版本字节先填占位 0.255.255：任何提前返回（含 SPI 坏 → ERR_FLASH）
         * 都不会把栈上的垃圾当版本号发出去。 */
        uint8_t resp[4] = { 0u, 0x00u, 0xFFu, 0xFFu };
        resp[0] = APP_QueryFlashVersion(&resp[1]);
        APP_SendAck(cmd, resp, 4);
        break;
    }

    /*── 0x000D 查询固件版本号 (FOC 板中) ──*/
    case MGR_CMD_QUERY_FOC_VER: {
        uint8_t resp[4];
        resp[0] = SWDP_QueryTargetVersion(&resp[1]);
        APP_SendAck(cmd, resp, 4);
        break;
    }

    /*── 0x0112 烧录 Flash 固件到 FOC 板 ──
     * ACK 在烧录完成后才返回 (协议要求), 因此这里只启动状态机,
     * 终态由 APP_Run 负责回 ACK。 */
    case MGR_CMD_PROGRAM: {
        uint8_t pre = APP_ProgramPreCheck();
        if (pre != APP_STATUS_OK) {
            APP_SendAck(cmd, &pre, 1);
            break;
        }
        prog_seq  = pc_seq;
        /* 每次开烧前重新套用速度设定：`0x001E` 的速度扫描结束会把延时
         * 恢复成编译默认（swd_selftest.c:676），不重新套用就等于把用户的设定抹了。*/
        SWD_PhySetDelay((s_swd_delay_ovr != 0u) ? s_swd_delay_ovr
                                                : (uint16_t)SWD_DELAY_LOOPS);
        app_state = APP_STATE_PROGRAMMING;
        SWDP_Start();
        break;
    }

    /*── 0x000E 测量 ADC (mode=0 只读 / mode=1 测试) ──*/
    case MGR_CMD_MEASURE:
        APP_MeasCmd(cmd, data, len);
        break;

    /*── 0x000F 脉冲捕获 (mode=0 只读 / mode=1 测试) ──*/
    case MGR_CMD_PULSE:
        APP_PulseCmd(cmd, data, len);
        break;

    /*── 0x0011 DUT 透传（需要 JIG_DUT_LINK=1）──*/
#if (JIG_DUT_LINK != 0)
    case MGR_CMD_DUT_RAW:
        APP_DutCmd(cmd, data, len);
        break;

    /*── 0x0028 设置/查询 SWD 速度（试档位：设一档、烧一次、看结果）──*/
    case MGR_CMD_SWD_SPEED: {
        if (len == 0u) {                       /* 查询 */
            uint8_t  out[MGR_SWD_SPEED_RESP_LEN];
            uint16_t eff = SWD_PhyGetDelay();

            out[0] = (uint8_t)(s_swd_delay_ovr >> 8);
            out[1] = (uint8_t)(s_swd_delay_ovr);
            out[2] = (uint8_t)(eff >> 8);
            out[3] = (uint8_t)(eff);
            APP_SendAck(cmd, out, MGR_SWD_SPEED_RESP_LEN);
            break;
        }
        if (len != 1u) {
            uint8_t e = APP_STATUS_ERR_PARAM;

            APP_SendAck(cmd, &e, 1u);
            break;
        }
        {
            uint8_t st = APP_STATUS_OK;

            s_swd_delay_ovr = (uint16_t)data[0];       /* 0 = 恢复默认 */
            SWD_PhySetDelay((s_swd_delay_ovr != 0u) ? s_swd_delay_ovr
                                                    : (uint16_t)SWD_DELAY_LOOPS);
            APP_SendAck(cmd, &st, 1u);
        }
        break;
    }

    /*── 0x0027 双槽完整状态（上传之后确认两个槽都齐了）──*/
    case MGR_CMD_FLASH_SLOTS: {
        uint8_t out[MGR_FLASH_SLOTS_RESP_LEN];
        uint8_t k;

        for (k = 0u; k < 2u; k++) {
            uint32_t sz   = 0u;
            uint8_t  flag = 0u;
            uint8_t  v[3] = { 0u, 0u, 0u };
            uint8_t  st;
            uint8_t  r = Flash_ReadHead(k, &sz, v, &flag);

            if (r == 2u) {
                st = (uint8_t)FW_SLOT_ST_READFAIL;      /* SPI 读不到：硬件故障 */
            } else if ((sz == 0u) || (sz > FW_MAX_IMAGE_SIZE)) {
                st = (uint8_t)FW_SLOT_ST_EMPTY;         /* 没上传过 */
            } else if (flag == FW_FLAG_COMPLETE) {
                st = (uint8_t)FW_SLOT_ST_COMPLETE;      /* 完整标志已落盘 */
            } else {
                st = (uint8_t)FW_SLOT_ST_INCOMPLETE;    /* 传了但没收到结束帧 */
            }

            out[(k * 8u) + 0u] = st;
            out[(k * 8u) + 1u] = (uint8_t)(sz >> 24);
            out[(k * 8u) + 2u] = (uint8_t)(sz >> 16);
            out[(k * 8u) + 3u] = (uint8_t)(sz >> 8);
            out[(k * 8u) + 4u] = (uint8_t)(sz);
            out[(k * 8u) + 5u] = v[0];
            out[(k * 8u) + 6u] = v[1];
            out[(k * 8u) + 7u] = v[2];
        }
        APP_SendAck(cmd, out, MGR_FLASH_SLOTS_RESP_LEN);
        break;
    }

    /*── 0x0026 dump DUT 口最近收到的原始字节（诊断：流程期间对面回了什么）──*/
    case MGR_CMD_DUT_LOG: {
        uint8_t out[1u + MGR_DUT_LOG_SIZE];
        uint16_t n2 = (s_dut_log_len > (uint16_t)MGR_DUT_LOG_SIZE)
                      ? (uint16_t)MGR_DUT_LOG_SIZE : s_dut_log_len;

        out[0] = (uint8_t)n2;
        if (n2 > 0u) {
            memcpy(&out[1], s_dut_log, n2);
        }
        APP_SendAck(cmd, out, (uint16_t)(1u + n2));
        break;
    }

    /*── 0x0025 上一次流程里 FOC 上报的编码器原始数字（诊断）──*/
    case MGR_CMD_ENC_LAST: {
        uint8_t out[MGR_ENC_LAST_RESP_LEN];

        out[0] = s_fl6_last_ok;
        out[1] = (uint8_t)(s_fl6_last_a >> 8); out[2] = (uint8_t)s_fl6_last_a;
        out[3] = (uint8_t)(s_fl6_last_b >> 8); out[4] = (uint8_t)s_fl6_last_b;
        out[5] = (uint8_t)(s_fl6_last_z >> 8); out[6] = (uint8_t)s_fl6_last_z;
        APP_SendAck(cmd, out, MGR_ENC_LAST_RESP_LEN);
        break;
    }

    /*── 0x0023 编码器发生器的寄存器快照（诊断：0x0012 发不出脉冲时用）──*/
    case MGR_CMD_ENC_DBG: {
        uint16_t dbg[6];
        uint8_t  out[MGR_ENC_DBG_RESP_LEN];
        uint8_t  k;

        EncGen_DbgSnapshot(dbg);
        for (k = 0u; k < 6u; k++) {
            out[k * 2u]      = (uint8_t)(dbg[k] >> 8);
            out[(k * 2u) + 1u] = (uint8_t)(dbg[k] & 0xFFu);
        }
        APP_SendAck(cmd, out, MGR_ENC_DBG_RESP_LEN);
        break;
    }

    /*── 0x0022 六步测试流程（烧完之后一条命令跑完）──*/
    case MGR_CMD_FLOW_TEST:
        APP_FlowTestCmd(cmd, data, len);
        break;

    /*── 0x001F 给被测板发**原始字节**（对面还没实现 0x02xx 时先验串口通不通）──*/
    case MGR_CMD_DUT_SEND:
        APP_DutSendCmd(cmd, data, len);
        break;

    /*── 0x0020 取走 DUT 口攒下的原始字节（只读，读后清空）──*/
    case MGR_CMD_DUT_RECV:
        APP_DutRecvCmd(cmd, data, len);
        break;

    /*── 0x0014 ⑥ 相电流检测：一条命令跑完整流程 ──*/
    case MGR_CMD_CUR_TEST:
        APP_CurTestCmd(cmd, data, len);
        break;

    /*── 0x0015 ② 电压检测：9 路 vs 被测板自己报的 ──*/
    case MGR_CMD_VOL_TEST:
        APP_VolTestCmd(cmd, data, len);
        break;

    /*── 0x0010 一键全流程（进测试模式 → ②③④⑤⑥ → STOP）──*/
    case MGR_CMD_RUN_ALL:
        APP_RunAllCmd(cmd, data, len);
        break;

    /*── 0x0019 ③ 一条命令（TM → PWM → 采集 → STOP）──*/
    case MGR_CMD_PULSE_TEST:
        APP_PulseTestCmd(cmd, data, len);
        break;

    /*── 0x001A ④ 一条命令（TM → PWM → 采集 → 判定 → STOP）──*/
    case MGR_CMD_PHASE_TEST:
        APP_PhaseTestCmd(cmd, data, len);
        break;

    /*── 0x001B ⑤ 一条命令（TM → 清零 → 发 → 读U2 → 读DUT → 判定 → STOP）──*/
    case MGR_CMD_ENC_TEST:
        APP_EncTestCmd(cmd, data, len);
        break;
#endif

    /*── 0x0018 ⑤ 编码器三方对齐判定（纯计算，不依赖 DUT 链路）──*/
    case MGR_CMD_ENC_JUDGE:
        APP_EncJudgeCmd(cmd, data, len);
        break;

    /*── 0x0012 编码器脉冲发生（⑤，纯治具侧）──*/
    case MGR_CMD_ENC_GEN:
        APP_EncGenCmd(cmd, data, len);
        break;

    /*── 0x0013 ⑤ 回环：向 U2 要编码器计数 ──*/
    case MGR_CMD_ENC_PEER:
        APP_EncPeerCmd(cmd, data, len);
        break;

    /*── 0x0016 ④ 三相输出判据（对已有结果判定，无状态机）──*/
    case MGR_CMD_PHASE_JUDGE:
        APP_PhaseJudgeCmd(cmd, data, len);
        break;

    /*── 未实现命令 ──*/
    default: {
        uint8_t e = APP_STATUS_ERR_NIY;
        APP_SendAck(cmd, &e, 1);
        break;
    }
    }
}

/*==============================================================================
 * 帧到达处理 (主循环上下文)
 *============================================================================*/
static void APP_HandlePendingFrame(void)
{
    uint16_t flen = 0u;
    uint16_t cmd = 0u, seq = 0u, dlen = 0u;
    /* 帧数据缓冲放 .bss 而非栈: 256B 栈数组会显著抬高主循环栈深,
     * 叠加 USB CDC 中断嵌套后余量不足 (参考项目也踩过栈上大数组的坑)。 */
    static uint8_t data[FRAME_MAX_DATA];
    uint8_t *raw = APP_Frame_Consume(&flen);

    if (!Frame_Parse(raw, flen, &cmd, &seq, data, &dlen)) {
        /* CRC 错误: 协议规定仅诊断, 不回 ACK */
        return;
    }

    pc_seq = seq;
#if (APP_BENCH_ALIVE != 0)
    s_bench_frames++;               /* 收到了第一帧 → 心跳永久闭嘴 */
#endif
    APP_Dispatch(cmd, data, dlen);
}

/*==============================================================================
 * 到 U2 的内部链路
 *----------------------------------------------------------------------------
 * 作用:
 *   1) 持续证明"两片之间的串口是通的"——每 500ms 探一次并检查应答;
 *   2) 每 5 次探测插一次 STATUS: 拿到 U2 版本/统计, 并核对两片的 链路代码版本
 *      （两片各有一份副本, 只重烧一片会导致字段对不上 → 这里看得见）;
 *   3) 给 ③ 的测量交叉校验留通道 (U1 测一遍、U2 测一遍, 结果必须一致)。
 *
 * 为什么用 Start/Poll 两段而不是 Link_Request:
 *   主循环里不能有 >50ms 的阻塞——USB 侧(协议帧)和 SWD 烧录都靠这个循环推进。
 *   Start 只负责把请求塞进发送队列, Poll 每轮推进一次, 全程零阻塞。
 *============================================================================*/
#if (JIG_U2_LINK != 0)
/* 连续超时到阈值 → 复位链路 (docs/13 §3.6)。
 * 只在空闲状态下被调用；复位也失败就退避 5s，避免一直刷屏重试。 */
static void APP_LinkRecover(void)
{
    uint8_t  resp[4];
    uint16_t rlen = 0u;
    uint32_t now;
    uint8_t  ok;

    if (s_link_fail < APP_LINK_FAIL_LIMIT) {
        return;
    }
    now = HAL_GetTick();
    if ((uint32_t)(now - s_reset_t0) < s_reset_backoff_ms) {
        return;                                     /* 退避中 */
    }

    (void)Link_Request(&s_u2_link, LINK_CMD_RESET, 0, 0u, resp, &rlen, APP_LINK_RESET_TIMEOUT_MS);
    ok = ((rlen == 1u) && (resp[0] == LINK_ST_OK)) ? 1u : 0u;
    Link_MasterReset(&s_u2_link);                   /* 无论成败都清本地会话 */

    s_link_fail        = 0u;
    s_reset_t0         = now;
    s_reset_backoff_ms = (ok != 0u) ? 0u : APP_LINK_RESET_BACKOFF_MS;
}

static void APP_LinkTask(void)
{
    uint8_t  st;
    uint16_t rxlen = 0u;

    /* 烧录期间完全不碰链路: 这时 USB 侧必须及时响应, 而且 U2 的数据也没人用 */
    if (app_state != APP_STATE_IDLE) {
        return;
    }

    /* 事件补拉（慢路径, docs/13 §6.4）: STATUS 说"有位待决"就取一条回来 */
    if (s_ev_fetch != 0u) {
        st = Link_RequestPoll(&s_u2_link, APP_EV_GET_TIMEOUT_MS, s_link_rx, &rxlen);
        if (st == 0u) {
            return;
        }
        s_ev_fetch = 0u;
        if ((st == 1u) && (rxlen >= LINK_EVENT_GET_LEN) && (s_link_rx[1] != 0u)) {
            APP_LogU2Event(s_link_rx[1], s_link_rx[2],
                           (uint16_t)(((uint16_t)s_link_rx[3] << 8) | s_link_rx[4]));
        }
        return;
    }

    if (Link_Busy(&s_u2_link) != 0u) {
        st = Link_RequestPoll(&s_u2_link, APP_U2_PROBE_TIMEOUT_MS, s_link_rx, &rxlen);
        if (st == 1u) {
            s_probe_ok++;
            s_u2_alive  = 1u;
            s_link_fail = 0u;                       /* 通了 → 连续失败清零 */
            /* 只有在"发的是 STATUS 且回来的也是 STATUS"时才解析:
             * 0 长度的 PING 应答没有数据区, 拿去解包只会解出误判。 */
            if ((s_probe_is_status != 0u) &&
                (Link_LastRespCmd(&s_u2_link) == LINK_CMD_STATUS)) {
                link_status_t tmp;
                if (Link_UnpackStatus(s_link_rx, rxlen, &tmp) != 0u) {
                    s_u2_status      = tmp;
                    s_u2_ver_ok      = Link_CheckVersion(&tmp);
                    s_u2_ver_checked = 1u;
                    /* 对侧还有待决事件 → 安排一次 EVENT_GET 补拉 */
                    if ((tmp.ev_pending != 0u) &&
                        (Link_RequestStart(&s_u2_link, LINK_CMD_EVENT_GET, 0, 0u) != 0u)) {
                        s_ev_fetch = 1u;
                    }
                }
            }
        } else if (st == 2u) {
            s_probe_fail++;
            s_u2_alive = 0u;
            if (s_link_fail < 0xFFu) {
                s_link_fail++;
            }
            APP_LinkRecover();                      /* 连续超时 → 链路复位 */
        }
        return;
    }

    Link_Poll(&s_u2_link);              /* 空闲时也收: U2 主动上报/U2 侧的应答 */

    /* 快路径: U2 主动发来的事件帧, 主循环里慢慢取出来 */
    {
        link_event_t ev;
        while (Link_TakeEvent(&s_u2_link, &ev) != 0u) {
            APP_LogU2Event(ev.code, ev.sev, ev.arg0);
        }
    }

    if ((uint32_t)(HAL_GetTick() - s_probe_tick) >= APP_U2_PROBE_PERIOD_MS) {
        s_probe_tick = HAL_GetTick();
        s_probe_cnt++;
        s_probe_is_status = ((s_probe_cnt % APP_U2_STATUS_EVERY) == 0u) ? 1u : 0u;
        /* PING 不带数据: 只要收到应答帧(哪怕 0 长度)就算这一路通的 */
        (void)Link_RequestStart(&s_u2_link,
                                (s_probe_is_status != 0u) ? LINK_CMD_STATUS : LINK_CMD_PING,
                                0, 0u);
    }
}

uint8_t APP_U2Alive(void)
{
    return s_u2_alive;
}

void APP_U2ProbeStats(uint16_t *ok, uint16_t *fail)
{
    if (ok   != 0) *ok   = s_probe_ok;
    if (fail != 0) *fail = s_probe_fail;
}

uint8_t APP_U2VerOk(void)
{
    /* 还没收到过 STATUS 时不算"不一致": 上电头 2.5s 不该报错 */
    if (s_u2_ver_checked == 0u) {
        return 1u;
    }
    return s_u2_ver_ok;
}

uint8_t APP_U2Status(link_status_t *out)
{
    if ((out == 0) || (s_u2_ver_checked == 0u)) {
        return 0u;
    }
    *out = s_u2_status;
    return 1u;
}

#else  /* JIG_U2_LINK == 0：台架形态，U2 不在（USART3 归被测板） */

static void APP_LinkTask(void)
{
    /* 没有 U2 就没有保活探测、没有事件补拉、没有链路恢复。
     * 这里必须是**空实现**而不是"发不出去就算了"：只要往 s_u2_link 发过一次，
     * 那一发就会落到 USART3 上 —— 而 USART3 现在是被测板的线。 */
}

uint8_t APP_U2Alive(void)                 { return 0u; }
void    APP_U2ProbeStats(uint16_t *ok, uint16_t *fail)
{
    if (ok   != 0) *ok   = 0u;
    if (fail != 0) *fail = 0u;
}
uint8_t APP_U2VerOk(void)                 { return 0u; }   /* 没有 U2 = 谈不上版本一致 */
uint8_t APP_U2Status(link_status_t *out)  { (void)out; return 0u; }

#endif /* JIG_U2_LINK */

/*==============================================================================
 * 对外 API
 *============================================================================*/
void APP_Init(void)
{
    app_state = APP_STATE_INIT;
    pc_seq    = 0u;
    prog_seq  = 0u;

    APP_Frame_Init();
    FwUpload_Init();

    SWDP_Init();

    /* 串口: 先绑端口开接收, 再建链路对象。反过来会漏掉早到的请求 */
    UartCfg_Init();
#if (JIG_U2_LINK != 0)
    Link_Init(&s_u2_link, UART_CFG_PORT_LINK, 1u /* 主机 */);
#endif
#if (JIG_DUT_LINK != 0)
    /* DUT 链路：同一个链路层。出厂形态绑端口 0（USART1），
     * 台架形态绑端口 1（USART3，U2 让位）—— 由 uart_cfg.h 的 UART_CFG_PORT_DUT 决定。 */
    Link_Init(&s_dut_link, UART_CFG_PORT_DUT, 1u /* 主机 */);
    s_dut_seq = 0u;
#endif
#if (JIG_U2_LINK != 0)
    s_probe_tick      = HAL_GetTick();
    s_probe_cnt       = 0u;
    s_probe_is_status = 0u;
    s_probe_ok        = 0u;
    s_probe_fail      = 0u;
    s_u2_alive        = 0u;
    s_u2_ver_ok       = 0u;
    s_u2_ver_checked  = 0u;
#endif
    memset(&s_u2_status, 0, sizeof(s_u2_status));

    /* 测量/测试状态 */
    s_meas_stage    = APP_MEAS_ST_IDLE;
    s_meas_seq      = 0u;
    s_last_cc_flags = 0u;
    s_last_cc_mask  = 0u;
    s_last_cc_peer  = 0u;
    memset(&s_snap, 0, sizeof(s_snap));
    memset(&s_peer, 0, sizeof(s_peer));
    s_pulse_peer_ok = 0u;
    memset(&s_pulse, 0, sizeof(s_pulse));
    memset(&s_pulse_peer, 0, sizeof(s_pulse_peer));
    memset(s_pulse_diff, 0, sizeof(s_pulse_diff));

    /* 脉冲捕获：配好定时器（此时不采），捕获窗口由 0x000E mode=1 打开 */
    PULSE_Init();
    PULSE_CfgInit();

    /* ⑤ 编码器脉冲发生：配好并把 CN2 的三根线收回输入态（此时不发） */
    EncGen_Init();
    s_enc_seq = 0u;
    memset(&s_enc_plan, 0, sizeof(s_enc_plan));

#if (APP_BENCH_ALIVE != 0)
    /* 开 DWT 周期计数器：量"主循环单圈最大耗时"。
     * 115200 下一个字节约 87µs；主循环只要卡超过这个数，RX 就会被顶出 ORE。*/
    CoreDebug->DEMCR |= CoreDebug_DEMCR_TRCENA_Msk;
    DWT->CYCCNT = 0u;
    DWT->CTRL  |= DWT_CTRL_CYCCNTENA_Msk;
    s_loop_max_us = 0u;
#endif

    app_state = APP_STATE_IDLE;
}

void APP_Run(void)
{
#if (APP_BENCH_ALIVE != 0)
    uint32_t app_run_t0 = DWT->CYCCNT;      /* 72MHz：72 个周期 = 1µs */
#endif

    /* ── 0a) USB 关键寄存器打印到调试口（PA9），每 2 秒一行（2026-11-03 加）──
     * 为什么从这里打：USB 不枚举时**没有 COM 口**，命令发不进来；
     * 而 USB 版本来就把调试打印开在 PA9 上，所以把寄存器从那里吐出来最直接。
     * 格式：[USB] irq istr cntr btable daddr ep0r  （各 4 字节 hex）
     * 判读：
     *   daddr 的 bit7(EF)=0        → USB 功能没使能，SIE 不响应任何包
     *   ep0r 的 STAT_RX != 11( VALID ) → EP0 没准备好接 SETUP，必然全 NAK
     *   cntr 的 bit15(CTRM)=0      → 传输完成中断被关掉
     *   istr 出现 CTR(0x8000)      → 至少完成过一次传输 */
#if (JIG_PC_LINK == JIG_PC_LINK_USB) && (BSP_DBG_ENABLE != 0)
    {
        extern uint32_t USB_EvtIrqCount(void);
        extern uint32_t USB_EvtIstrOr(void);
        static uint32_t s_usb_dbg_tick = 0u;
        uint32_t now = HAL_GetTick();

        if ((now - s_usb_dbg_tick) >= 2000u) {
            uint32_t w[6];
            uint8_t  k;

            s_usb_dbg_tick = now;
            w[0] = USB_EvtIrqCount();
            w[1] = USB_EvtIstrOr();
            w[2] = (uint32_t)USB->CNTR;
            w[3] = (uint32_t)USB->BTABLE;
            w[4] = (uint32_t)USB->DADDR;
            w[5] = (uint32_t)USB->EP0R;

            BSP_DBG_Send("[USB] ");
            for (k = 0u; k < 6u; k++) {
                BSP_DBG_SendHexByte((uint8_t)(w[k] >> 24));
                BSP_DBG_SendHexByte((uint8_t)(w[k] >> 16));
                BSP_DBG_SendHexByte((uint8_t)(w[k] >> 8));
                BSP_DBG_SendHexByte((uint8_t)w[k]);
                BSP_DBG_Send(" ");
            }
            BSP_DBG_Send("\r\n");
        }
    }
#endif

    /* 0) PC 口收到的字节搬进帧解析器（USB 分支是空操作，见 pc_link.c） */
    PcLink_Poll();

#if (APP_BENCH_ALIVE != 0)
    /* 0b) 台架版自报身份：连上之前每 3 秒一行心跳（连上后自动闭嘴） */
    APP_BenchSay();
#endif

    /* 1) 把接收环形缓冲里的字节喂给解析器, 处理收到的完整帧 */
    APP_Frame_Poll();
    APP_FrameStallCheck();          /* 半截帧超时 → 丢掉重新对齐（见函数注释）*/
#if (JIG_DUT_LINK != 0)
    APP_AsciiPoll();                /* PC 串口助手的命令行（无结束符超时 / 等回字窗口）*/
#endif
    if (APP_Frame_Pending()) {
        APP_HandlePendingFrame();
    }

    /* 2) 上传超时检查 (10s 无新数据) */
    FwUpload_TimeoutCheck();

    /* 3) SWD 烧录推进 + 终态回 ACK (不变量 I4: 只回一次) */
    if (app_state == APP_STATE_PROGRAMMING) {
        SWDP_Step();
        if (!SWDP_Busy()) {
            uint8_t st = SWDP_Result();
            app_state = APP_STATE_IDLE;
            APP_SendFrame(MGR_CMD_PROGRAM, prog_seq, &st, 1u);
            /* 烧录成功 → 读被测板 96-bit UID，主动上报 SN（0x0024）。
             * 放在 ACK **之后**发：先让 PC 拿到烧录结果，SN 帧随后到。
             * 读不到就静默跳过，绝不影响烧录结果、也不推脏数据。 */
            if (st == APP_STATUS_OK) {
                APP_SnReport();
            }
            /* SWDP_Step 内单个 128KB 扇区擦除要 ~1.3s, 期间 PC 若仍在发帧,
             * 512B 接收环形缓冲可能被填满而丢弃字节, 留下半截帧。
             * 协议规定 PC 必须等 0x0112 的 ACK 才继续, 因此这里直接清空
             * 接收侧, 保证下一条命令从干净的解析状态开始 (见并发分析: 序号 10)。*/
            APP_Frame_Init();
        }
    }

    /* 3b) 0x001E SWD 烧录自检推进 + 终态回 ACK（不依赖 SPI Flash，只走 SWD） */
    if (SST_Busy() != 0u) {
        SST_Step();
        if (SST_Busy() == 0u) {
            uint8_t r[MGR_SWD_SELFTEST_RESP_LEN];

            (void)SST_Result(r);
            APP_SendFrame(MGR_CMD_SWD_SELFTEST, s_sst_seq, r,
                          (uint16_t)MGR_SWD_SELFTEST_RESP_LEN);
        }
    }

    /* 4) 到 U2 的内部链路 (非阻塞: 每轮最多推进一步) */
    APP_LinkTask();
#if (JIG_DUT_LINK != 0)
    /* 4b) 到 DUT（被烧 FOC 板）的链路：同样每轮推进一步 */
    APP_DutTask();

    /* 4c) 0x001F 的收发窗口：收字节、到点回 ACK */
    APP_DutRawTask();

    /* 4d) 0x0022 六步测试流程 */
    APP_FlowTestTask();

    /* 7b) ⑥ 相电流检测流程推进 + 终态回 ACK（只回一次） */
    APP_CurTestTask();

    /* 7c) ② 电压检测流程推进 + 终态回 ACK */
    APP_VolTestTask();

    /* 7d) 0x0010 一键全流程编排（每轮推进一步） */
    APP_RunAllTask();

    /* 7e) 0x0019/0x001A/0x001B/0x0014 单项"一条命令跑完"的流程推进 */
    APP_FlowTask();
#endif


    /* 5) 0x000E 测试推进 + 终态回 ACK (不变量 I4: 只回一次) */
    if (app_state == APP_STATE_TESTING) {
        APP_MeasStep();
    }

    /* 6) 0x0012 编码器脉冲串推进 + 终态回 ACK (同样只回一次) */
    APP_EncGenTask();

    /* 7) 0x0013 向 U2 要编码器计数 + 终态回 ACK */
    APP_EncPeerTask();

#if (APP_BENCH_ALIVE != 0)
    {
        uint32_t dt = DWT->CYCCNT - app_run_t0;
        uint32_t us = dt / 72u;

        if (us > s_loop_max_us) {
            s_loop_max_us = us;             /* 记最大值，心跳里报 */
        }
    }
#endif
}

/* USB CDC 收到一批字节 (中断上下文) — 只喂状态机, 不做重活 */
/*==============================================================================
 * PC 串口助手"手敲命令行"（ASCII 控制台）
 *----------------------------------------------------------------------------
 * 场景：FOC 固件还没实现 0x02xx 帧协议，但调试时要能**在串口助手里直接敲**
 *       `AS:Bb` / `AS:CS` / `AS:0` 看它回什么。
 * 规则（docs/24 §1）：
 *      · 字节流以 A5 5B 开头          → 走原来的帧协议（20 条命令照旧）
 *      · 全是可打印 ASCII(0x20~0x7E)  → 当一行命令：原样转给 DUT 口，
 *                                        把 DUT 的回字**原样**吐回 PC（无帧）
 *      · 其它                          → 丢弃（和以前一样，等 A5 5B）
 * 为什么 0xA5 不会和 ASCII 混：0xA5 = 165 > 0x7E，本身就不是可打印字符。
 * 只在 IDLE 时受理：否则正在跑的命令/流程也在用 DUT 口，会互相抢。
 *============================================================================*/
#if (JIG_DUT_LINK != 0)
static uint8_t  s_ascii_line[AS_LINE_MAX];
static uint16_t s_ascii_len;
static uint32_t s_ascii_tlast;          /* 最后一个字节的时刻（无结束符时靠它） */
static uint32_t s_ascii_t0;             /* 命令发出去的时刻 */
static uint16_t s_ascii_rxlen;          /* 上一次看到的回字长度（判断"还在长"） */
static uint32_t s_ascii_rxt;            /* 回字最后变长的时刻 */
static uint32_t s_ascii_cnt;            /* 成功转发的命令行数（调试用） */
/* 1 = 之后"治具空闲时"把 DUT 口收到的字节实时转给 PC（透明桥）。
 * 为什么需要：`AS:MC` 的结束上报 `AS:D,...` 是 2 秒后才主动发的，
 * 控制台 300ms 的窗口早关了 —— 不桥接的话串口助手里永远看不到。 */
static uint8_t  s_ascii_bridge;
/* 上一个 PC 字节是不是帧头首字节 A5 —— 见 APP_AsciiAccept 里的注释：
 * 帧头第二字节 0x5B 就是 '['，只看"缓冲是否为 0"会把它当命令行吃掉。 */
static uint8_t  s_pc_after_a5;

/** 收一个字节；返回 1 = 这个字节被控制台吃了，不要再喂给帧解析 */
static uint8_t APP_AsciiAccept(uint8_t b)
{
    if (s_ascii_len > 0u) {                             /* 正在收一行 */
        if ((b == (uint8_t)'\r') || (b == (uint8_t)'\n')) {
            APP_AsciiDispatch();
            return 1u;
        }
        if ((b >= 0x20u) && (b <= 0x7Eu)) {
            if (s_ascii_len < (uint16_t)AS_LINE_MAX) {
                s_ascii_line[s_ascii_len] = b;
                s_ascii_len++;
            } else {
                s_ascii_len = 0u;                       /* 超长：整行作废 */
            }
            s_ascii_tlast = HAL_GetTick();
            return 1u;
        }
        s_ascii_len = 0u;                               /* 混进非可打印字节：这行作废 */
        return 0u;                                      /* 交回帧解析（可能是 A5） */
    }

    /* 干净起点才决定"这是不是一行命令"，三个条件缺一不可：
     *   · Buffered()==0  解析器里没攒着半截帧
     *   · RingCount()==0 环形缓冲里也没有刚收到、还没解析的字节
     *     （只看 Buffered 会把正在到达的帧当成命令行 —— 帧里的 CRC 和版本字符串
     *      是可见 ASCII，会被吃掉，2026-11-09 踩到）
     *   · 上一个字节不是 A5：帧头首字节之后的 0x5B 就是 '['，也是可打印字符 */
    if ((APP_Frame_Buffered() == 0u) && (APP_Frame_RingCount() == 0u) &&
        (s_pc_after_a5 == 0u)) {
        if ((b == (uint8_t)'\r') || (b == (uint8_t)'\n')) {
            return 1u;                                  /* 空行：吃掉，别污染帧解析 */
        }
        if ((b >= 0x20u) && (b <= 0x7Eu)) {
            s_ascii_line[0] = b;
            s_ascii_len     = 1u;
            s_ascii_tlast   = HAL_GetTick();
            return 1u;
        }
    }
    return 0u;
}

/** 把攒够的一行发出去，并进入"等回字"窗口 */
static void APP_AsciiDispatch(void)
{
    uint8_t  buf[AS_LINE_MAX + 2u];
    uint16_t n = s_ascii_len;
    uint16_t blen;

    s_ascii_len = 0u;                                   /* 先交出缓冲，避免重入 */
    if (n == 0u) {
        return;
    }
    if (app_state != APP_STATE_IDLE) {                   /* 正忙：别抢 DUT 口 */
        PcLink_Send((const uint8_t *)AS_MSG_BUSY, (uint16_t)(sizeof(AS_MSG_BUSY) - 1u));
        return;
    }
    memcpy(buf, s_ascii_line, n);
    blen = n;
#if (JIG_AS_APPEND_CRLF != 0)
    buf[blen] = (uint8_t)'\r'; blen++;
    buf[blen] = (uint8_t)'\n'; blen++;
#endif

    /* 先把 DUT 口攒下的陈字节读空，否则回字里混着上一次的，看不出是哪一次的回 */
    s_dut_sniff_len  = 0u;
    s_dut_sniff_drop = 0u;
    APP_DutSniff();

    if (UartPort_Write(UART_CFG_PORT_DUT, buf, blen) != blen) {
        PcLink_Send((const uint8_t *)AS_MSG_BUSY, (uint16_t)(sizeof(AS_MSG_BUSY) - 1u));
        return;
    }
    s_ascii_cnt++;
    s_ascii_bridge = 1u;                   /* 从此以后异步上报也转给 PC */
    s_ascii_t0    = HAL_GetTick();
    s_ascii_rxlen = 0u;
    s_ascii_rxt   = s_ascii_t0;
    app_state     = APP_STATE_ASCII;
}

/** 窗口结束：把回字原样吐回 PC（没有回字就给一句提示，别让人干等） */
static void APP_AsciiFlush(void)
{
    if (s_dut_sniff_len > 0u) {
        PcLink_Send(s_dut_sniff, s_dut_sniff_len);
    } else {
        PcLink_Send((const uint8_t *)AS_MSG_NO_REPLY,
                    (uint16_t)(sizeof(AS_MSG_NO_REPLY) - 1u));
    }
    s_dut_sniff_len = 0u;
    s_ascii_rxlen   = 0u;
    app_state       = APP_STATE_IDLE;
}

/** 主循环里推进一步：没结束符的行超时发出；等回字的窗口到点收尾 */
static void APP_AsciiPoll(void)
{
    uint32_t now = HAL_GetTick();

    if (s_ascii_len > 0u) {
        if ((uint32_t)(now - s_ascii_tlast) >= (uint32_t)AS_LINE_IDLE_MS) {
            APP_AsciiDispatch();                         /* 没按回车的命令也能发出去 */
        }
        return;
    }
    if (app_state != APP_STATE_ASCII) {
        /* 控制台收尾之后，DUT 口上"迟到"的字节也要实时转给 PC。
         * 典型就是 `AS:MC`/`AS:MQ` 的 `AS:D,...`：它在 MC0 之后约 2 秒才来，
         * 那时控制台窗口早关了（2026-11-11 实测：直连能看到、经治具看不到）。*/
        if ((app_state == APP_STATE_IDLE) && (s_ascii_bridge != 0u)) {
            APP_DutSniff();
            if (s_dut_sniff_len > 0u) {
                PcLink_Send(s_dut_sniff, s_dut_sniff_len);
                s_dut_sniff_len = 0u;
            }
        }
        return;
    }
    APP_DutSniff();                                      /* 每轮都收，别等环满 */
    if (s_dut_sniff_len != s_ascii_rxlen) {
        s_ascii_rxlen = s_dut_sniff_len;
        s_ascii_rxt   = now;
    }
    if (((s_ascii_rxlen > 0u) && ((uint32_t)(now - s_ascii_rxt) >= (uint32_t)AS_REPLY_GAP_MS)) ||
        ((uint32_t)(now - s_ascii_t0) >= (uint32_t)AS_REPLY_MS)) {
        APP_AsciiFlush();
    }
}
#endif /* JIG_DUT_LINK */

/*==============================================================================
 * FOC 回字的文本解析（纯函数）
 *----------------------------------------------------------------------------
 * 为什么不写进状态机里：这三个函数是"认不认得出"的问题，和"什么时候该问"无关。
 * 拆成纯函数就能拿表驱动单测直接砸几十种畸形输入（T29），不用搭时序。
 * 容错规则见 docs/24 §7 与文件头。
 *============================================================================*/
#if (JIG_DUT_LINK != 0)
/** 转小写：对 'A'~'Z' 有效；数字、':' 等本来就是小写形式，|0x20 不影响 */
static uint8_t AS_Lower(uint8_t c)
{
    return (uint8_t)(c | 0x20u);
}

/** 读一组十进制数字；要求 1~5 位且 ≤ 65535（超了整条不认，别悄悄截断） */
static uint8_t AS_Digits(const uint8_t *p, uint16_t avail, uint16_t *val, uint16_t *used)
{
    uint32_t v = 0u;
    uint16_t i = 0u;

    while ((i < avail) && (i < 5u) && (p[i] >= (uint8_t)'0') && (p[i] <= (uint8_t)'9')) {
        v = (v * 10u) + (uint32_t)(p[i] - (uint8_t)'0');
        i++;
    }
    if (i == 0u) {
        return 0u;                                       /* 一位数字都没有 */
    }
    if ((i < avail) && (p[i] >= (uint8_t)'0') && (p[i] <= (uint8_t)'9')) {
        return 0u;                                       /* 超过 5 位：溢出，整条不认 */
    }
    if (v > 65535u) {
        return 0u;
    }
    *val  = (uint16_t)v;
    *used = i;
    return 1u;
}

uint8_t APP_AsParseEnc(const uint8_t *buf, uint16_t len,
                       uint16_t *out_a, uint16_t *out_b, uint16_t *out_c)
{
    uint16_t i;

    if ((buf == 0) || (len < 8u)) {                       /* 最短 "AS:A1B1C1" = 8 */
        return 0u;
    }
    for (i = 0u; (uint16_t)(i + 4u) < len; i++) {
        uint16_t p, a, b, c, used;

        if ((AS_Lower(buf[i])        != (uint8_t)'a') ||
            (AS_Lower(buf[i + 1u])   != (uint8_t)'s') ||
            (buf[i + 2u]             != (uint8_t)':') ||
            (AS_Lower(buf[i + 3u])   != (uint8_t)'a')) {
            continue;
        }
        p = (uint16_t)(i + 4u);
        if (AS_Digits(&buf[p], (uint16_t)(len - p), &a, &used) == 0u) {
            continue;                                     /* 可能是 AS:A0 这种单组码 */
        }
        p = (uint16_t)(p + used);
        if ((p >= len) || (AS_Lower(buf[p]) != (uint8_t)'b')) {
            continue;
        }
        p++;
        if (AS_Digits(&buf[p], (uint16_t)(len - p), &b, &used) == 0u) {
            continue;
        }
        p = (uint16_t)(p + used);
        if ((p >= len) || (AS_Lower(buf[p]) != (uint8_t)'c')) {
            continue;
        }
        p++;
        if (AS_Digits(&buf[p], (uint16_t)(len - p), &c, &used) == 0u) {
            continue;
        }
        if (out_a != 0) { *out_a = a; }
        if (out_b != 0) { *out_b = b; }
        if (out_c != 0) { *out_c = c; }
        return 1u;
    }
    return 0u;
}

/* 解析 "AS:D,A=25,B=83,Z=0"：先要有 AS:D 前缀，再找三个键值对（顺序无所谓） */
uint8_t APP_AsParseEncD(const uint8_t *buf, uint16_t len,
                        uint16_t *out_a, uint16_t *out_b, uint16_t *out_z)
{
    uint16_t i;
    uint16_t n = 0u;
    uint16_t val[3] = { 0u, 0u, 0u };
    uint8_t  got[3] = { 0u, 0u, 0u };

    if ((buf == 0) || (len < 8u)) {
        return 0u;
    }
    /* 必须有 AS:D 前缀（否则可能是 AS:A0 那类状态码） */
    if (APP_AsHasPrefix(buf, len, AS_PREFIX_ENCDATA) == 0u) {
        return 0u;
    }
    for (i = 0u; (uint16_t)(i + 2u) < len; i++) {   /* 要访问 buf[i+2]，只需 i+2 < len */
        uint8_t  key = AS_Lower(buf[i]);
        uint8_t  k;
        uint16_t v, used;

        if ((key != (uint8_t)'a') && (key != (uint8_t)'b') && (key != (uint8_t)'z')) {
            continue;
        }
        if ((buf[i + 1u] != (uint8_t)'=') || (buf[i + 2u] < (uint8_t)'0') ||
            (buf[i + 2u] > (uint8_t)'9')) {
            continue;                        /* 不是 key=数字 的形式 */
        }
        if (AS_Digits(&buf[i + 2u], (uint16_t)(len - i - 2u), &v, &used) == 0u) {
            continue;
        }
        k = (uint8_t)((key == (uint8_t)'a') ? 0u : ((key == (uint8_t)'b') ? 1u : 2u));
        val[k] = v;
        got[k] = 1u;
        n++;
        i = (uint16_t)(i + 2u + used - 1u);
    }
    if ((got[0] == 0u) || (got[1] == 0u) || (got[2] == 0u)) {
        return 0u;
    }
    if (out_a != 0) { *out_a = val[0]; }
    if (out_b != 0) { *out_b = val[1]; }
    if (out_z != 0) { *out_z = val[2]; }
    return 1u;
}

uint8_t APP_AsHasPrefix(const uint8_t *buf, uint16_t len, const char *prefix)
{
    uint16_t n = 0u;
    uint16_t i;

    while (prefix[n] != '\0') {
        n++;
    }
    if ((buf == 0) || (n == 0u) || (len < n)) {
        return 0u;
    }
    for (i = 0u; (uint16_t)(i + n) <= len; i++) {
        uint16_t k;
        uint8_t  hit = 1u;

        for (k = 0u; k < n; k++) {
            if (AS_Lower(buf[i + k]) != AS_Lower((uint8_t)prefix[k])) {
                hit = 0u;
                break;
            }
        }
        if (hit != 0u) {
            return 1u;
        }
    }
    return 0u;
}

uint8_t APP_AsFindCode(const uint8_t *buf, uint16_t len)
{
    uint16_t i;

    if ((buf == 0) || (len < 5u)) {
        return 0xFFu;
    }
    for (i = 0u; (uint16_t)(i + 5u) <= len; i++) {
        uint8_t d;

        if ((AS_Lower(buf[i])      != (uint8_t)'a') ||
            (AS_Lower(buf[i + 1u]) != (uint8_t)'s') ||
            (buf[i + 2u]           != (uint8_t)':') ||
            (AS_Lower(buf[i + 3u]) != (uint8_t)'a')) {
            continue;
        }
        d = buf[i + 4u];
        if ((d < (uint8_t)'0') || (d > (uint8_t)'3')) {
            continue;
        }
        /* 状态码后面必须是结束符 —— 否则 AS:A0B0C0 会被认成"自检通过" */
        if ((uint16_t)(i + 6u) <= len) {
            uint8_t t = buf[i + 5u];

            if ((t != (uint8_t)'\r') && (t != (uint8_t)'\n') &&
                (t != (uint8_t)' ')  && (t != 0u)) {
                continue;
            }
        }
        return (uint8_t)(d - (uint8_t)'0');
    }
    return 0xFFu;
}
#endif /* JIG_DUT_LINK */

void APP_PcInput(uint8_t b)
{
#if (APP_BENCH_ALIVE != 0)
    s_pc_in_bytes++;                         /* 只要 PC 说过话，心跳就闭嘴 */
#endif
#if (JIG_DUT_LINK != 0)
    uint8_t after_a5 = s_pc_after_a5;

    s_pc_after_a5 = (b == 0xA5u) ? 1u : 0u;   /* 给下一个字节用 */

    /* 紧跟帧头 A5 的那个字节永远交给帧解析：它就是 0x5B='['，是可打印字符。
     * 少了这一条，所有 A5 5B 帧都会被控制台吃成命令行（2026-11-09 改后审查发现）。*/
    if (after_a5 != 0u) {
        APP_Frame_Feed(b);
        return;
    }
    if (APP_AsciiAccept(b) == 0u) {          /* 不是命令行 → 交给帧解析 */
        APP_Frame_Feed(b);
    }
#else
    APP_Frame_Feed(b);
#endif
}

void APP_USB_Receive(uint8_t *data, uint32_t len)
{
    for (uint32_t i = 0; i < len; i++) {
        APP_PcInput(data[i]);                            /* 统一入口：ASCII 优先判 */
    }
}
