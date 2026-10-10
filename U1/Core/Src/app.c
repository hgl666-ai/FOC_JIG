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
    APP_MEAS_ST_WAIT_STABLE,    /* 等 9 路连续稳定 (MEAS_STABLE_N 次) */
    APP_MEAS_ST_WAIT_PULSE,     /* 等脉冲捕获窗口结束 */
    APP_MEAS_ST_WAIT_PEER,      /* 已发测量快照交换请求, 等 U2 的快照 */
    APP_MEAS_ST_WAIT_PULSE_PEER /* 已发脉冲结果交换请求, 等 U2 的结果 */
} APP_MeasStage_t;

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
        s_test_cmd   = cmd;
        s_meas_seq   = pc_seq;
        PULSE_CfgStart(PULSE_WINDOW_MS);
        s_meas_stage = APP_MEAS_ST_WAIT_PULSE;
        app_state    = APP_STATE_TESTING;
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
        MEAS_TestBegin();
        s_test_cmd   = cmd;
        s_meas_seq   = pc_seq;
        s_meas_stage = APP_MEAS_ST_WAIT_STABLE;
        app_state    = APP_STATE_TESTING;
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
 * 命令分发
 *============================================================================*/
static void APP_Dispatch(uint16_t cmd, const uint8_t *data, uint16_t len)
{
    /* 上传三段式命令由 fw_upload 独占处理 (内部自带 BUSY/参数校验) */
    if (FwUpload_Handle(cmd, data, len)) return;

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
static void APP_LinkTask(void)
{
    uint8_t  st;
    uint16_t rxlen = 0u;

    /* 烧录期间完全不碰链路: 这时 USB 侧必须及时响应, 而且 U2 的数据也没人用 */
    if (app_state != APP_STATE_IDLE) {
        return;
    }

    if (Link_Busy(&s_u2_link) != 0u) {
        st = Link_RequestPoll(&s_u2_link, APP_U2_PROBE_TIMEOUT_MS, s_link_rx, &rxlen);
        if (st == 1u) {
            s_probe_ok++;
            s_u2_alive = 1u;
            /* 只有在"发的是 STATUS 且回来的也是 STATUS"时才解析:
             * 0 长度的 PING 应答没有数据区, 拿去解包只会解出误判。 */
            if ((s_probe_is_status != 0u) &&
                (Link_LastRespCmd(&s_u2_link) == LINK_CMD_STATUS)) {
                link_status_t tmp;
                if (Link_UnpackStatus(s_link_rx, rxlen, &tmp) != 0u) {
                    s_u2_status      = tmp;
                    s_u2_ver_ok      = Link_CheckVersion(&tmp);
                    s_u2_ver_checked = 1u;
                }
            }
        } else if (st == 2u) {
            s_probe_fail++;
            s_u2_alive = 0u;
        }
        return;
    }

    Link_Poll(&s_u2_link);              /* 空闲时也收: U2 主动上报/U2 侧的应答 */

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

    /* 5) 0x000E 测试推进 + 终态回 ACK (不变量 I4: 只回一次) */
    if (app_state == APP_STATE_TESTING) {
        APP_MeasStep();
    }
}

/* USB CDC 收到一批字节 (中断上下文) — 只喂状态机, 不做重活 */
void APP_USB_Receive(uint8_t *data, uint32_t len)
{
    for (uint32_t i = 0; i < len; i++) {
        APP_Frame_Feed(data[i]);
    }
}
