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
#include "enc_gen.h"
#include "jig_mode.h"
#include "usbd_cdc_if.h"
#include "usb_device.h"
#include <string.h>

/*==============================================================================
 * 全局状态
 *============================================================================*/
APP_State_t app_state = APP_STATE_INIT;
uint16_t    pc_seq    = 0u;

static uint16_t prog_seq = 0u;      /* 0x0112 受理时的帧序号, 烧完 ACK 回显用 */
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

/*===== 到 U2 的内部链路 (USART3, 见 docs/10_UART四路驱动设计.md) =====*/
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
static uint8_t         s_meas_resp[PULSE_PACK_RESP_LEN]; /* 0x000E(77B) / 0x000F(91B) 共用应答缓冲(取较大者) */
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
static uint8_t  s_link_fail;            /* 连续请求超时次数 */
static uint32_t s_reset_t0;             /* 上次链路复位的时刻 */
static uint32_t s_reset_backoff_ms;     /* 复位失败后的退避窗口 */
static uint8_t  s_ev_fetch;             /* 有一次 EVENT_GET 补拉在进行 */

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
        (Link_RequestStart(&s_u2_link, LINK_CMD_ARM, req, LINK_ARM_REQ_LEN) == 0u)) {
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
 * 帧发送
 *============================================================================*/
static void APP_WaitTxDone(void)
{
    USBD_CDC_HandleTypeDef *hcdc = (USBD_CDC_HandleTypeDef *)hUsbDeviceFS.pClassData;
    uint32_t tick = HAL_GetTick();

    if (hcdc == 0) return;
    while (hcdc->TxState != 0u) {
        if ((HAL_GetTick() - tick) > APP_TX_BUSY_TIMEOUT_MS) break;
    }
}

static void APP_SendFrame(uint16_t cmd, uint16_t seq, const uint8_t *data, uint16_t len)
{
    uint16_t total;
    uint32_t tick;
    USBD_CDC_HandleTypeDef *hcdc;

    /* ⚠ 必须**先等上一次发完**，再往 tx_buf 里组帧：
     *    超过 64B 的帧（0x000E 的 77B / 0x000F 的 91B 应答）由 USB 中断继续从
     *    tx_buf 取后续包；上一帧还在飞就覆写 → 主机会收到混叠/截断的帧。
     *    等不到（USB 卡住）就放弃这一帧：宁可让 PC 超时重试，也不发坏帧。 */
    APP_WaitTxDone();
    hcdc = (USBD_CDC_HandleTypeDef *)hUsbDeviceFS.pClassData;
    if ((hcdc != 0) && (hcdc->TxState != 0u)) {
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
    tick  = HAL_GetTick();

    while (CDC_Transmit_FS(tx_buf, total) == USBD_BUSY) {
        if ((HAL_GetTick() - tick) > APP_TX_BUSY_TIMEOUT_MS) return;
    }
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
static uint16_t APP_PulseBuildResp(uint8_t cmd_status)
{
    uint16_t n = 0u;
    uint16_t left;

    s_meas_resp[n++] = cmd_status;

    left = (uint16_t)(PULSE_PACK_RESP_LEN - n);
    if (PulsePack_Pack(&s_meas_resp[n], left, &s_pulse) != PULSE_PACK_LEN) {
        return 0u;
    }
    n = (uint16_t)(n + PULSE_PACK_LEN);

    /* 对侧结果：拿不到就传 0（按"无数据"打包），配合 per-phase flags 区分 */
    left = (uint16_t)(PULSE_PACK_RESP_LEN - n);
    if (PulsePack_Pack(&s_meas_resp[n], left,
                       (s_pulse_peer_ok != 0u) ? &s_pulse_peer : 0) != PULSE_PACK_LEN) {
        return 0u;
    }
    n = (uint16_t)(n + PULSE_PACK_LEN);

    return (n == PULSE_PACK_RESP_LEN) ? n : 0u;
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

        st = Link_RequestPoll(&s_u2_link, APP_MEAS_ARM_TIMEOUT_MS, resp, &rlen);
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
        if (Link_RequestStart(&s_u2_link, LINK_CMD_MEAS_XCHK,
                              s_meas_req, MEAS_PACK_SNAP_LEN) == 0u) {
            APP_MeasFinish(APP_MeasVerdict(MEAS_CC_F_NO_PEER, 0u),
                           MEAS_CC_F_NO_PEER, 0u, 0u);
            return;
        }
        s_meas_stage = APP_MEAS_ST_WAIT_PEER;
        return;

    case APP_MEAS_ST_WAIT_PEER:
        st = Link_RequestPoll(&s_u2_link, APP_MEAS_PEER_TIMEOUT_MS,
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
            if (Link_RequestStart(&s_u2_link, LINK_CMD_PULSE_XCHK,
                                  preq, PULSE_PACK_LEN) == 0u) {
                APP_MeasFinish(APP_PulseVerdict(), 0u, 0u, 0u);
                return;
            }
        }
        s_meas_stage = APP_MEAS_ST_WAIT_PULSE_PEER;
        return;

    case APP_MEAS_ST_WAIT_PULSE_PEER:
        st = Link_RequestPoll(&s_u2_link, APP_MEAS_PEER_TIMEOUT_MS,
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

    GD25Q_ReadBuffer(hdr, base, FW_HEAD_SIZE);

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

        (void)Flash_ReadHead((uint8_t)slot, &size, v, &flag);

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

        (void)Flash_ReadHead(slot, &size, 0, &flag);

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

    if (app_state != APP_STATE_DUT) {
        if (Link_Busy(&s_dut_link) == 0u) {
            Link_Poll(&s_dut_link);
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
    (void)Link_RequestPoll(&s_u2_link, 0u, 0, 0);

    s_encp_act     = data[0];
    s_encp_seq     = pc_seq;
    s_encp_started = 0u;
    s_encp_t0      = HAL_GetTick();
    app_state      = APP_STATE_ENC_PEER;
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
        if (Link_RequestStart(&s_u2_link, LINK_CMD_ENC_XCHK, &s_encp_act, 1u) != 0u) {
            s_encp_started = 1u;
        } else if ((uint32_t)(HAL_GetTick() - s_encp_t0) > MGR_ENC_PEER_TIMEOUT_MS) {
            uint8_t e = APP_STATUS_BUSY;        /* 链路一直占着（对端可能卡住） */
            app_state = APP_STATE_IDLE;
            APP_SendFrame(MGR_CMD_ENC_PEER, s_encp_seq, &e, 1u);
        }
        return;
    }

    /* 第二阶段：等 U2 的应答 */
    st = Link_RequestPoll(&s_u2_link, MGR_ENC_PEER_TIMEOUT_MS, s_encp_resp, &rn);
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
        /* 默认：上桥 A、下桥 B、100%、200ms、两路电流都读 */
        s_cur_req[0] = DUT_PHASE_A;
        s_cur_req[1] = DUT_PHASE_B;
        s_cur_req[2] = 0x03u; s_cur_req[3] = 0xE8u;      /* 1000‰ */
        s_cur_req[4] = 0x00u; s_cur_req[5] = 0xC8u;      /* 200 ms */
        s_cur_req[6] = 0x03u;                            /* 两路 */
        s_cur_req[7] = 0x00u;
    } else if (len == MGR_CUR_TEST_REQ_LEN) {
        memcpy(s_cur_req, data, MGR_CUR_TEST_REQ_LEN);
    } else {
        e = APP_STATUS_ERR_PARAM;
        APP_SendAck(cmd, &e, 1u);
        return;
    }

    /* 参数预检（命令层自己先过一遍，不把校验全甩给 DUT） */
    s_cur_dur_ms = (uint16_t)(((uint16_t)s_cur_req[4] << 8) | s_cur_req[5]);
    {
        dut_current_t c;
        c.mode          = DUT_CUR_MODE_STATIC;
        c.hi_mask       = s_cur_req[0];
        c.lo_mask       = s_cur_req[1];
        c.duty_permille = (uint16_t)(((uint16_t)s_cur_req[2] << 8) | s_cur_req[3]);
        c.duration_ms   = s_cur_dur_ms;

        if ((DutCheck_Current(&c) == 0u) ||
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
            /* 请求 DATA 与 0x0206 的布局一致（去掉 rsvd 之后 7B… 见 DutPack_Current） */
            {
                dut_current_t c;
                c.mode          = DUT_CUR_MODE_STATIC;
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
                                  s_cur_req, DUT_CURRENT_REQ_LEN) != 0u) {
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

        out[0] = any_bad ? APP_STATUS_ERR_TEST : APP_STATUS_OK;
        out[1] = s_cur_dut_st;
        out[2] = (uint8_t)CUR_CHK_CH_COUNT;
        out[3] = 0u;
        for (i = 0u; i < (uint8_t)CUR_CHK_CH_COUNT; i++) {
            uint8_t *p = &out[4u + (i * 9u)];     /* 每路 9B */
            p[0] = (uint8_t)(s_cur_out[i].flags & 0xFFu);
            p[1] = (uint8_t)(s_cur_in[i].jig_idle_mv >> 8);
            p[2] = (uint8_t)(s_cur_in[i].jig_idle_mv);
            p[3] = (uint8_t)(s_cur_in[i].jig_on_mv >> 8);
            p[4] = (uint8_t)(s_cur_in[i].jig_on_mv);
            p[5] = (uint8_t)(s_cur_in[i].dut_idle_mv >> 8);
            p[6] = (uint8_t)(s_cur_in[i].dut_idle_mv);
            p[7] = (uint8_t)(s_cur_in[i].dut_on_mv >> 8);
            p[8] = (uint8_t)(s_cur_in[i].dut_on_mv);
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
static uint16_t      s_vol_ver;
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
    s_vol_ver     = 0u;
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
        s_vol_ver      = fi.dut_ver;
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

        out[0] = ((sum & VOL_CHK_SUM_ANY_BAD) != 0u) ? APP_STATUS_ERR_TEST : APP_STATUS_OK;
        out[1] = s_vol_info_st;
        out[2] = (uint8_t)(s_vol_ver >> 8);
        out[3] = (uint8_t)(s_vol_ver & 0xFFu);
        out[4] = (uint8_t)(s_vol_adc_mask >> 8);
        out[5] = (uint8_t)(s_vol_adc_mask & 0xFFu);
        out[6] = (uint8_t)(sum >> 8);
        out[7] = (uint8_t)(sum & 0xFFu);
        for (i = 0u; i < (uint8_t)MEAS_CH_COUNT; i++) {
            uint8_t *p = &out[8u + ((uint16_t)i * MGR_VOL_CH_ITEM_LEN)];
            uint16_t jm = 0u, dm = 0u;
            (void)VolChk_TakeJig(&snap, i, &jm);
            (void)VolChk_TakeDut(&s_vol_dut, i, &dm);
            p[0] = (uint8_t)(s_vol_out[i].flags & 0xFFu);
            p[1] = (uint8_t)(jm >> 8);  p[2] = (uint8_t)jm;
            p[3] = (uint8_t)(dm >> 8);  p[4] = (uint8_t)dm;
            p[5] = (uint8_t)((uint16_t)(s_vol_out[i].err_mv >> 8) & 0xFFu);
            p[6] = (uint8_t)((uint16_t)s_vol_out[i].err_mv & 0xFFu);
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

    out[0] = ((sum & PHASE_CHK_SUM_ANY_BAD) != 0u) ? APP_STATUS_ERR_TEST : APP_STATUS_OK;
    out[1] = (uint8_t)(sum >> 8);
    out[2] = (uint8_t)(sum & 0xFFu);
    out[3] = 3u;
    for (i = 0u; i < 3u; i++) {
        uint8_t *p = &out[4u + ((uint16_t)i * MGR_PHASE_ITEM_LEN)];
        uint16_t ex = (outs[i].expect_mv > 0xFFFFu) ? 0xFFFFu : (uint16_t)outs[i].expect_mv;
        uint16_t er = (uint16_t)(int16_t)((outs[i].err_mv > 32767) ? 32767
                                         : ((outs[i].err_mv < -32768) ? -32768
                                            : outs[i].err_mv));
        uint16_t to = (outs[i].tol_mv > 0xFFFFu) ? 0xFFFFu : (uint16_t)outs[i].tol_mv;

        p[0] = (uint8_t)(outs[i].flags >> 8);
        p[1] = (uint8_t)(outs[i].flags & 0xFFu);
        p[2] = (uint8_t)(ex >> 8);    p[3] = (uint8_t)ex;
        p[4] = (uint8_t)(er >> 8);    p[5] = (uint8_t)er;
        p[6] = (uint8_t)(to >> 8);    p[7] = (uint8_t)to;
        p[8] = (uint8_t)(ins[i].duty_pm >> 8);
        p[9] = (uint8_t)(ins[i].duty_pm & 0xFFu);
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
    if (Link_Busy(&s_u2_link) != 0u) flags |= MGR_JIG_ST_LINK_BUSY;
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

    ls = Link_Stats(&s_u2_link);
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

    out[0] = ((sum & ENC_CHK_SUM_ANY_BAD) != 0u) ? APP_STATUS_ERR_TEST : APP_STATUS_OK;
    out[1] = (uint8_t)(sum >> 8);
    out[2] = (uint8_t)(sum & 0xFFu);
    out[3] = (uint8_t)ENC_CHK_PH_COUNT;
    out[4] = (uint8_t)(u2_lost >> 8);
    out[5] = (uint8_t)(u2_lost);
    out[6] = (uint8_t)dut_dir;
    out[7] = 0u;
    for (i = 0u; i < (uint8_t)ENC_CHK_PH_COUNT; i++) {
        uint8_t *p = &out[8u + ((uint16_t)i * MGR_ENC_JUDGE_ITEM_LEN)];
        uint16_t u2 = (outs[i].u2_edges  > 0xFFFFu) ? 0xFFFFu : (uint16_t)outs[i].u2_edges;
        uint16_t dt = (outs[i].dut_edges > 0xFFFFu) ? 0xFFFFu : (uint16_t)outs[i].dut_edges;

        /* flags 只有 1 字节（ENC_CHK_F_* 都在低 4 位，和 cur_chk/vol_chk 一致）；
         * ⚠ 这一格写成 2 字节会把后面三相全部挤歪、最后一相还要越界 1 字节。 */
        p[0] = (uint8_t)(outs[i].flags & 0xFFu);
        p[1] = (uint8_t)(outs[i].exp_edges >> 8);  p[2] = (uint8_t)outs[i].exp_edges;
        p[3] = (uint8_t)(u2 >> 8);                 p[4] = (uint8_t)u2;
        p[5] = (uint8_t)(dt >> 8);                 p[6] = (uint8_t)dt;
    }
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
    (void)Link_RequestPoll(&s_u2_link, 0u, 0, 0);
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

    /*── 0x000C 查询固件版本号 (Flash 中) ──*/
    case MGR_CMD_QUERY_FLASH_VER: {
        uint8_t resp[4];
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
    Link_Init(&s_u2_link, UART_CFG_PORT_LINK, 1u /* 主机 */);
#if (JIG_DUT_LINK != 0)
    /* DUT 链路：同一个链路层，绑到端口 0（USART1），U1 当主机 */
    Link_Init(&s_dut_link, UART_CFG_PORT_DUT, 1u /* 主机 */);
    s_dut_seq = 0u;
#endif
    s_probe_tick      = HAL_GetTick();
    s_probe_cnt       = 0u;
    s_probe_is_status = 0u;
    s_probe_ok        = 0u;
    s_probe_fail      = 0u;
    s_u2_alive        = 0u;
    s_u2_ver_ok       = 0u;
    s_u2_ver_checked  = 0u;
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

    app_state = APP_STATE_IDLE;
}

void APP_Run(void)
{
    /* 1) 把接收环形缓冲里的字节喂给解析器, 处理收到的完整帧 */
    APP_Frame_Poll();
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
            /* SWDP_Step 内单个 128KB 扇区擦除要 ~1.3s, 期间 PC 若仍在发帧,
             * 512B 接收环形缓冲可能被填满而丢弃字节, 留下半截帧。
             * 协议规定 PC 必须等 0x0112 的 ACK 才继续, 因此这里直接清空
             * 接收侧, 保证下一条命令从干净的解析状态开始 (见并发分析: 序号 10)。*/
            APP_Frame_Init();
        }
    }

    /* 4) 到 U2 的内部链路 (非阻塞: 每轮最多推进一步) */
    APP_LinkTask();

#if (JIG_DUT_LINK != 0)
    /* 4b) 到 DUT（被烧 FOC 板）的链路：同样每轮推进一步 */
    APP_DutTask();

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
}

/* USB CDC 收到一批字节 (中断上下文) — 只喂状态机, 不做重活 */
void APP_USB_Receive(uint8_t *data, uint32_t len)
{
    for (uint32_t i = 0; i < len; i++) {
        APP_Frame_Feed(data[i]);
    }
}
