/*
 * PC ↔ 治具 U1 全链路宿主仿真测试
 *
 * 直接把固件的 app.c / fw_upload.c / app_frame.c / protocol_frame.c /
 * gd25q128.c / swd_programmer.c 等真源码编译进来, 宿主侧提供:
 *   - GD25Q128 SPI Flash 芯片仿真 (状态寄存器/WEL/扇区擦除/跨页编程)
 *   - 被烧板仿真: STM32F405 Flash + 按 ADIv5 解码的 SWD 线级目标模型 (test/swdsim/)
 *   - USB CDC 收发替身 (捕获 U1 发出的帧 / 注入 PC 发来的帧)
 * 从而在无硬件条件下验证整条链路:
 *   PC 帧 → app 分发 → 上传状态机 → SPI Flash 驱动 → SWD → 被烧板 Flash
 */
#include <stdio.h>
#include <string.h>
#include <stdint.h>

#include "app.h"
#include "jig_as.h"
#include "enc_plan.h"      /* ⑤ 编码器脉冲发生：替身里要用 enc_plan_t / ENC_PLAN_TIM_CLK_HZ */
#include "app_frame.h"
#include "protocol_frame.h"
#include "flash_map.h"
#include "fw_upload.h"
#include "gd25q128.h"
#include "swd_programmer.h"
#include "swd_config.h"
#include "swd_phy.h"
#include "sim_target.h"
#include "bsp_uart.h"
#include "measure.h"
#include "meas_pack.h"
#include "pulse_cap.h"
#include "pulse_pack.h"
#include "pulse_cap.h"
#include "pulse_pack.h"
#include "usbd_cdc_if.h"
#include "usb_device.h"
#include "uart_link.h"      /* 链路层：T16 的"假 U2"要用 */
#include "uart_cfg.h"       /* 端口号定义 */
#include "phase_chk.h"      /* ④ 三相输出判据（T22 在两种构建里都跑 → 必须在保护外）*/
#include "enc_chk.h"        /* ⑤ 三方对齐判据（T24 在两种构建里都跑 → 必须在保护外）*/

/*==============================================================================
 * 虚拟时间
 *============================================================================*/
static uint32_t g_tick = 0;
uint32_t HAL_GetTick(void) { return g_tick++; }
void     HAL_Delay(uint32_t ms) { g_tick += ms; }
static void host_advance(uint32_t ms) { g_tick += ms; }

/*==============================================================================
 * GD25Q128 仿真 (16MB)
 *============================================================================*/
static uint8_t g_spi[16u * 1024u * 1024u];

static struct {
    int      cs;              /* 1 = CS# 拉低 (片选有效) */
    uint8_t  cmd;
    uint8_t  phase;           /* 0=命令 1=地址 2=读输出 3=写SR 4=页编程 9=忽略 */
    uint32_t addr;
    uint8_t  addr_cnt;
    uint8_t  sr1;             /* bit0=WIP */
    uint8_t  sr2;
    uint8_t  wel;
    uint8_t  is_read, is_prog, is_erase;
    uint8_t  win[256];
    uint16_t win_idx;
    uint32_t page_start;
    uint8_t  out[4];
    uint8_t  out_len, out_pos;
    uint8_t  force_wrong_id;   /* 1 = RDID 返回全 0 (模拟 Flash 不在线/型号不对) */
    /* 观测点 */
    uint32_t rdsr_count, wren_count, pp_count, se_count;
    uint32_t wel_violations;
} fl;

static void flash_cs_low(void)
{
    fl.cs = 1; fl.phase = 0; fl.addr_cnt = 0; fl.out_len = 0; fl.out_pos = 0;
    fl.is_read = fl.is_prog = fl.is_erase = 0; fl.win_idx = 0;
}

static void flash_cs_high(void)
{
    if (fl.is_prog && fl.win_idx > 0) {
        if (!fl.wel) fl.wel_violations++;
        else {
            memcpy(&g_spi[fl.page_start], fl.win, fl.win_idx);
            fl.pp_count++;
        }
        fl.wel = 0;
    }
    fl.cs = 0;
    fl.is_prog = 0;
}

static uint8_t flash_xfer(uint8_t tx)
{
    uint8_t rx = 0xFFu;

    if (!fl.cs) return 0xFFu;

    switch (fl.phase) {

    case 0:                                     /* 命令 */
        fl.cmd = tx;
        switch (tx) {
        case 0x06: fl.wel = 1; fl.wren_count++; break;              /* WREN */
        case 0x05: fl.out[0] = fl.sr1; fl.out_len = 1; fl.out_pos = 0;
                   fl.phase = 2; fl.rdsr_count++; break;            /* RDSR1 */
        case 0x9F: fl.out[0] = fl.force_wrong_id ? 0x00u : 0xC8u;
                   fl.out[1] = fl.force_wrong_id ? 0x00u : 0x40u;
                   fl.out[2] = fl.force_wrong_id ? 0x00u : 0x18u;
                   fl.out_len = 3; fl.out_pos = 0; fl.phase = 2; break; /* RDID */
        case 0x01: fl.phase = 3; fl.cmd = 0x01; break;              /* WRSR1 */
        case 0x31: fl.phase = 3; fl.cmd = 0x31; break;              /* WRSR2 */
        case 0x03: fl.addr = 0; fl.addr_cnt = 0; fl.is_read = 1; fl.phase = 1; break; /* READ */
        case 0x02: fl.addr = 0; fl.addr_cnt = 0; fl.is_prog = 1; fl.win_idx = 0; fl.phase = 1; break;
        case 0x20: fl.addr = 0; fl.addr_cnt = 0; fl.is_erase = 1; fl.phase = 1; break;
        default:   fl.phase = 9; break;
        }
        break;

    case 1:                                     /* 3 字节地址 */
        fl.addr = (fl.addr << 8) | (uint32_t)tx;
        if (++fl.addr_cnt == 3) {
            if (fl.is_erase) {
                if (!fl.wel) fl.wel_violations++;
                else { memset(&g_spi[fl.addr & ~0xFFFu], 0xFF, 4096u); fl.se_count++; }
                fl.wel = 0;
                fl.phase = 9;
            } else if (fl.is_prog) {
                fl.page_start = fl.addr;
                fl.phase = 4;
            } else {
                fl.phase = 2;
            }
        }
        break;

    case 2:                                     /* 读输出 */
        if (fl.is_read) {
            rx = g_spi[fl.addr++];
        } else {
            rx = (fl.out_pos < fl.out_len) ? fl.out[fl.out_pos++] : 0xFFu;
        }
        break;

    case 3:                                     /* 写状态寄存器 */
        if (fl.cmd == 0x01) fl.sr1 = tx; else fl.sr2 = tx;
        fl.sr1 &= (uint8_t)~0x01u;              /* 仿真: WIP 立即完成 */
        fl.phase = 9;
        break;

    case 4:                                     /* 页编程数据 */
        fl.win[fl.win_idx++] = tx;
        break;

    default:
        break;
    }
    return rx;
}

/* ---- HAL 替身 ---- */
SPI_HandleTypeDef hspi2;                    /* gd25q128.c 里 &hspi2 需要实体 */

/* 串口替身：app.c 会初始化 U1 的两路串口并建立到 U2 的内部链路，所以要有
 * 两个句柄 (uart_cfg.c 里 extern 的就是它们) 和 HAL_UART_*_IT。
 *
 * 默认行为（g_u2_on == 0）：发送永不完成 → 链路保活 PING 必然超时，等于"U2 不在"。
 * T14/T15 依赖这个场景（对侧缺席的路径）。
 *
 * 打开 g_u2_on 之后：两路串口被当作"交叉的两根线"（U1 的端口1 ↔ 假 U2 的端口0），
 * 由 test/app_test.c 里的 U2Fake_* 扮演 U2 —— T16 用它覆盖 **ARM 主路径**。 */
UART_HandleTypeDef huart1;
UART_HandleTypeDef huart3;

static uint8_t  g_u2_on;                /* 1 = 链路上有一个会应答的"假 U2" */
static uint8_t *s_rxp1;                 /* huart1 的接收落点 (假 U2 那一侧) */
static uint8_t *s_rxp3;                 /* huart3 的接收落点 (U1 链路那一侧) */

HAL_StatusTypeDef HAL_UART_Receive_IT(UART_HandleTypeDef *huart, uint8_t *pData, uint16_t Size)
{
    (void)Size;
    if (huart == &huart1) {
        s_rxp1 = pData;
    } else if (huart == &huart3) {
        s_rxp3 = pData;
    }
    return HAL_OK;
}

/* 往某一侧的接收落点塞一个字节，并走真 HAL 的回调 → UartPort_OnRxComplete */
static void u2_deliver(uint8_t **slot, UART_HandleTypeDef *hu, uint8_t b)
{
    if (*slot == 0) {
        return;                         /* 对侧没挂接收: 这字节真的丢了 */
    }
    **slot = b;
    *slot  = 0;                         /* 落点已消费, 等回调里重挂 */
    HAL_UART_RxCpltCallback(hu);
    g_tick++;
}

#if (JIG_DUT_LINK != 0)
/* DUT 模式（docs/15）：字节级假 DUT。实现放在本文件后面。 */
static void DutFake_Feed(const uint8_t *p, uint16_t n);
#endif

HAL_StatusTypeDef HAL_UART_Transmit_IT(UART_HandleTypeDef *huart, uint8_t *pData, uint16_t Size)
{
#if (JIG_DUT_LINK != 0)
    /* DUT 模式：huart1 = U1 的 DUT 链路（端口0，见 jig_mode.h §3.4）。
     * U1 发出去的每个字节都喂给字节级假 DUT；假 DUT 的回帧立刻塞回 port0 的
     * 接收落点 → 等于"同一根线的两头"。huart3（内部链路）没人接，等于 U2 不在。 */
    if (huart == &huart1) {
        DutFake_Feed(pData, Size);
        HAL_UART_TxCpltCallback(huart);         /* 发完这一段 → 驱动推进 tx_tail */
        return HAL_OK;
    }
    (void)pData; (void)Size;
    return HAL_OK;
#else
    if (g_u2_on != 0u) {
        uint16_t i;
        /* 交叉接线: U1(huart3) 发的 → 进假 U2 那一侧; 假 U2(huart1) 发的 → 进 U1 */
        if (huart == &huart3) {
            for (i = 0u; i < Size; i++) {
                u2_deliver(&s_rxp1, &huart1, pData[i]);
            }
        } else {
            for (i = 0u; i < Size; i++) {
                u2_deliver(&s_rxp3, &huart3, pData[i]);
            }
        }
        HAL_UART_TxCpltCallback(huart);         /* 发完这一段 → 驱动推进 tx_tail */
        return HAL_OK;
    }
    (void)huart; (void)pData; (void)Size;
    return HAL_OK;              /* 故意不回调 TxCplt：模拟"线没接/对端不在" */
#endif
}

/*==============================================================================
 * 假 U2（T16 用）：真的跑一份 uart_link 从机 + 采集模块，只把"板级"那层换成替身
 *============================================================================*/
static link_t   s_u2;
#if (JIG_DUT_LINK == 0)
static uint8_t  s_u2_inited;        /* 只有 U2Fake_Init（T16 专有）用 */
static uint16_t s_u2_alive;         /* 只有 U2Fake_Handler（T16 专有）用 */
/* 假 U2 的编码器边沿计数（T19 用）。放在这个保护里是对的：
 * U2Fake_Handler 的 ENC_XCHK 分支只在非 DUT 构建里编译，T19 也是。 */
static uint32_t s_u2enc_a, s_u2enc_b, s_u2enc_z;
static uint16_t s_u2enc_lost;
static uint16_t s_u2enc_clears;
#endif
static uint8_t  s_u2_arm_active;
static uint16_t s_u2_win;
static uint32_t s_u2_arm_t0;

#if (JIG_DUT_LINK == 0)
static uint8_t U2Fake_Handler(uint16_t cmd, const uint8_t *in, uint16_t inlen,
                              uint8_t *out, uint16_t *outlen)
{
    switch (cmd) {
    case LINK_CMD_PING: {
        uint16_t i;
        if (inlen > LINK_MAX_PAYLOAD) {
            return 0u;
        }
        for (i = 0u; i < inlen; i++) {
            out[i] = in[i];
        }
        *outlen = inlen;
        return 1u;
    }
    case LINK_CMD_STATUS: {
        static const uint8_t ver[2] = { LINK_U2_VER_MAJOR, LINK_U2_VER_MINOR };
        s_u2_alive++;
        *outlen = Link_PackStatus(out, LINK_MAX_PAYLOAD, ver, s_u2_alive,
                                  Link_Stats(&s_u2), UartPort_Stats(UART_PORT_0),
                                  LINK_CAP_ALL,
                                  Link_SlaveEventPending(&s_u2),
                                  Link_SlaveEventOverflow(&s_u2));
        return (*outlen != 0u) ? 1u : 0u;
    }
    case LINK_CMD_MEAS_XCHK: {
        meas_snapshot_t ms;
        if ((in == 0) || (inlen < MEAS_PACK_SNAP_LEN)) {
            out[0] = LINK_ST_NIY; *outlen = 1u; return 1u;
        }
        MEAS_Snapshot(&ms);
        *outlen = MeasPack_Snapshot(out, LINK_MAX_PAYLOAD, &ms);
        return (*outlen != 0u) ? 1u : 0u;
    }
    case LINK_CMD_PULSE_XCHK: {
        pulse_snapshot_t ps;
        if ((in == 0) || (inlen < PULSE_PACK_LEN)) {
            out[0] = LINK_ST_NIY; *outlen = 1u; return 1u;
        }
        PULSE_Snapshot(&ps);
        /* 故意报成"另一个频率"(40kHz)，这样能证明对侧那段数据**真的传过来了**，
         * 而不是把本片的 20kHz 又抄了一遍。 */
        {
            uint8_t i;
            for (i = 0u; i < (uint8_t)PULSE_PH_COUNT; i++) {
                ps.ph[i].period_ns     = 25000u;
                ps.ph[i].high_ns       = 12500u;
                ps.ph[i].duty_permille = 500u;
                ps.ph[i].flags         = PULSE_F_VALID;
            }
        }
        *outlen = PulsePack_Pack(out, LINK_MAX_PAYLOAD, &ps);
        return (*outlen != 0u) ? 1u : 0u;
    }
    /*── ⑤ 编码器回环采集: 独立计数器，和滚动窗口并行（U2 侧真实现的替身）──*/
    case LINK_CMD_ENC_XCHK: {
        uint8_t act = 0u;
        if (Link_EncUnpackReq(in, inlen, &act) == 0u) {
            out[0] = LINK_ST_ERR_PARAM;
            *outlen = 1u;
            return 1u;
        }
        if (act == LINK_ENC_ACT_CLEAR) {
            s_u2enc_a = s_u2enc_b = s_u2enc_z = 0u;
            s_u2enc_clears++;
            out[0] = LINK_ST_OK;
            *outlen = 1u;
            return 1u;
        }
        {
            link_enc_cnt_t c;
            c.a = s_u2enc_a; c.b = s_u2enc_b; c.z = s_u2enc_z; c.lost = s_u2enc_lost;
            *outlen = Link_EncPackResp(out, LINK_MAX_PAYLOAD, LINK_ST_OK, &c);
        }
        return (*outlen != 0u) ? 1u : 0u;
    }
    default:
        out[0] = LINK_ST_NIY; *outlen = 1u; return 1u;
    }
}

/* 假 U2 使用"U1 的端口0"当自己的链路端口（真实板子上那是 DUT 串口排针） */
static void U2Fake_Init(void)
{
    if (s_u2_inited == 0u) {
        Link_Init(&s_u2, UART_PORT_0, 0u /* 从机 */);
        Link_SetSlaveHandler(&s_u2, U2Fake_Handler);
        s_u2_inited = 1u;
    }
    g_u2_on = 1u;
}
#endif

/* 每轮都推一下假 U2：收帧 → 应答；被 ARM 之后模拟窗口跑完 */
static void U2Fake_Poll(void)
{
    if (g_u2_on == 0u) {
        return;
    }
    Link_Poll(&s_u2);

    if (Link_SlaveArmTake(&s_u2, &s_u2_win) != 0u) {
        s_u2_arm_active = 1u;
        s_u2_arm_t0     = HAL_GetTick();
    }
    if ((s_u2_arm_active != 0u) &&
        ((uint32_t)(HAL_GetTick() - s_u2_arm_t0) >= (uint32_t)s_u2_win)) {
        s_u2_arm_active = 0u;
        Link_SlaveArmDone(&s_u2);
    }
}

/* U1 是否已经从 STATUS 拿到了对端能力（T16 用） */
#if (JIG_DUT_LINK == 0)
static int u2fake_caps_has_arm(void)
{
    link_status_t st;
    if (APP_U2Status(&st) == 0u) {
        return 0;
    }
    return ((st.caps & LINK_CAP_ARM) != 0u) ? 1 : 0;
}
#endif

#if (JIG_DUT_LINK != 0)
/*==============================================================================
 * 字节级假 DUT（只在 JIG_DUT_LINK=1 的构建里有）
 *----------------------------------------------------------------------------
 * U1 的 DUT 链路是主机；线那头是被测板。测试里只有一个 huart 句柄，
 * 所以这头不做成 link_t（那需要第二个端口），而是**字节级**的：
 *   收 U1 发出去的帧 → 按 docs/15 回一帧 → 塞回 port0 的接收落点。
 * 关键断言点是"帧真的到了那头、回帧真的回到了 U1"，而不是"代码看起来对"。
 *============================================================================*/
#include "dut_proto.h"
#include "cur_chk.h"        /* ⑥ 相电流判据（T20 用；app.c 在 JIG_DUT_LINK 下引用它）*/
#include "vol_chk.h"        /* ② 电压检测判据（T21 用）*/

static uint8_t  s_dutrx[192];
static uint16_t s_dutrx_len;
static uint16_t s_dut_seen;                     /* 收到过几帧合法请求 */
static uint16_t s_dut_last_cmd;
static uint16_t s_dut_last_seq;
static uint16_t s_dut_last_len;
static uint8_t  s_dut_last_data[LINK_MAX_PAYLOAD];
static uint8_t  s_dut_bad_crc;                  /* 收到过几次 CRC 错的帧 */
/* 0x001F 用：把 U1 从 DUT 口发出去的**所有**字节原样存一份（不要求是帧）。
 * 帧解析那套会把非帧字节当垃圾丢掉，而 0x001F 恰恰发的就是非帧的 ASCII。 */
static uint8_t  s_dut_raw[64];
static uint16_t s_dut_raw_len;
static uint8_t  s_dut_quiet;                    /* 1 = 收到了但故意不回（测超时） */
static uint8_t  s_dutfake_cur_on;               /* 1 = 已经"通上流"了（⑥ 用） */
static uint16_t s_dutfake_cur_seen;             /* 收到过几次 DUT_CURRENT */
static uint16_t s_dutfake_cur_len;              /* 最近一次 DUT_CURRENT 的 DATA 长度 */
static uint8_t  s_dutfake_cur_mode;
static uint8_t  s_dutfake_cur_hi;
static uint8_t  s_dutfake_cur_lo;
static uint16_t s_dutfake_cur_duty;
static uint16_t s_dutfake_cur_dur;
static uint8_t  s_dutfake_cur_rsvd;

/* 按 docs/15 §12 的示例应答生成回帧，并塞回 U1 */
static void DutFake_Reply(uint16_t cmd, uint16_t seq)
{
    uint8_t  out[96];
    uint16_t olen;
    uint16_t i;

    if (s_dut_quiet != 0u) {
        return;                                 /* 故意不回：U1 应走超时分支 */
    }

    switch (cmd) {
    case DUT_CMD_PING:
        /* 回环：原样回显请求 DATA */
        olen = Frame_Build(out, cmd, seq, s_dut_last_data, s_dut_last_len);
        break;
    case DUT_CMD_INFO: {
        static const uint8_t info[16] = {
            0x00, 0x01, 0x01, 0x01, 0x00, 0x00, 0x03, 0xFF,
            0x00, 0x64, 0x4E, 0x20, 0x00, 0x03, 0x00, 0x00 };
        olen = Frame_Build(out, cmd, seq, info, 16u);
        break;
    }
    case DUT_CMD_GET_ADC: {
        /* 按**请求的掩码**逐路应答（真 DUT 也该这样）。
         * 电流两路（ch7/ch8）会随"有没有通流"变化 —— 模拟一个**中点偏置**的放大器：
         * 零电流输出约 1650mV，通流后升到约 2000mV。 */
        uint8_t  dbuf[1u + (5u * DUT_GETADC_MAX_CH)];
        uint16_t mask = 0u;
        uint8_t  i;
        uint8_t  n = 0u;

        if (s_dut_last_len >= DUT_GETADC_REQ_LEN) {
            mask = (uint16_t)(((uint16_t)s_dut_last_data[0] << 8) | s_dut_last_data[1]);
        }
        dbuf[0] = DUT_ST_OK;
        for (i = 0u; i < 16u; i++) {
            uint16_t raw = 0u;
            uint16_t mv  = 0u;
            if ((mask & (uint16_t)(1u << i)) == 0u) {
                continue;
            }
            if (i == 0u)      { raw = 0x081Au; mv = 300u; }
            else if (i == 1u) { raw = 0x0830u; mv = 312u; }
            else if (i == 7u) { mv = (s_dutfake_cur_on != 0u) ? 2000u : 1650u;
                                raw = (uint16_t)((mv * 4095u) / 3300u); }
            else if (i == 8u) { mv = (s_dutfake_cur_on != 0u) ? 2005u : 1655u;
                                raw = (uint16_t)((mv * 4095u) / 3300u); }
            dbuf[1u + (n * 5u) + 0u] = (uint8_t)i;
            dbuf[1u + (n * 5u) + 1u] = (uint8_t)(raw >> 8);
            dbuf[1u + (n * 5u) + 2u] = (uint8_t)raw;
            dbuf[1u + (n * 5u) + 3u] = (uint8_t)(mv >> 8);
            dbuf[1u + (n * 5u) + 4u] = (uint8_t)mv;
            n++;
        }
        /* ⚠ 这里产出的是 **DATA 段**，还要套一层帧（原来那句是 Frame_Build，
         *   改写时漏了套帧 → 对端收到一堆裸字节，表现成"应答长度不对"。 */
        olen = Frame_Build(out, cmd, seq, dbuf, (uint16_t)(1u + ((uint16_t)n * 5u)));
        break;
    }
    case DUT_CMD_CURRENT: {
        static const uint8_t ok = DUT_ST_OK;
        /* 把收到的 8 字节按 **DUT 的布局** 拆出来存好，由测试主体断言
         * （CHECK 宏在本函数之后才定义，这里不能直接用）。
         * 为什么要存：治具曾经把 PC 的 8 字节请求（hi/lo/duty/dur/ch/rsvd）
         * 原样发给 DUT（DUT 侧是 mode/hi/lo/duty/dur/rsvd）—— 长度一样、内容全错，
         * 真 DUT 回 0B；而假 DUT 一律回 OK，于是这个 bug 在回归里完全是隐形的。 */
        s_dutfake_cur_len  = s_dut_last_len;
        s_dutfake_cur_mode = (s_dut_last_len >= 1u) ? s_dut_last_data[0] : 0xFFu;
        s_dutfake_cur_hi   = (s_dut_last_len >= 2u) ? s_dut_last_data[1] : 0xFFu;
        s_dutfake_cur_lo   = (s_dut_last_len >= 3u) ? s_dut_last_data[2] : 0xFFu;
        s_dutfake_cur_duty = (s_dut_last_len >= 5u)
                             ? (uint16_t)(((uint16_t)s_dut_last_data[3] << 8) | s_dut_last_data[4])
                             : 0xFFFFu;
        s_dutfake_cur_dur  = (s_dut_last_len >= 7u)
                             ? (uint16_t)(((uint16_t)s_dut_last_data[5] << 8) | s_dut_last_data[6])
                             : 0xFFFFu;
        s_dutfake_cur_rsvd = (s_dut_last_len >= 8u) ? s_dut_last_data[7] : 0xFFu;
        s_dutfake_cur_on = 1u;
        s_dutfake_cur_seen++;
        olen = Frame_Build(out, cmd, seq, &ok, 1u);
        break;
    }
    default: {
        static const uint8_t ok = DUT_ST_OK;
        olen = Frame_Build(out, cmd, seq, &ok, 1u);
        break;
    }
    }

    for (i = 0u; i < olen; i++) {
        u2_deliver(&s_rxp1, &huart1, out[i]);   /* 同一根线的另一头 */
    }
}

/* U1 往 DUT 链路写出去的字节，逐个喂进来 */
static void DutFake_Feed(const uint8_t *p, uint16_t n)
{
    uint16_t i;

    for (i = 0u; i < n; i++) {
        uint16_t need;

        if (s_dutrx_len >= (uint16_t)sizeof(s_dutrx)) {
            s_dutrx_len = 0u;                   /* 溢出：丢掉重来 */
        }
        if (s_dut_raw_len < (uint16_t)sizeof(s_dut_raw)) {
            s_dut_raw[s_dut_raw_len++] = p[i];
        }
        s_dutrx[s_dutrx_len++] = p[i];

        /* 错位重同步：不是帧头就丢掉最前面那个字节（和真实现一个思路） */
        while ((s_dutrx_len >= 2u) &&
               (s_dutrx[0] != FRAME_HDR0 || s_dutrx[1] != FRAME_HDR1)) {
            memmove(&s_dutrx[0], &s_dutrx[1], (size_t)(s_dutrx_len - 1u));
            s_dutrx_len--;
        }
        if (s_dutrx_len < FRAME_OVERHEAD) {
            continue;
        }
        need = (uint16_t)(FRAME_OVERHEAD +
                          (uint16_t)(((uint16_t)s_dutrx[6] << 8) | s_dutrx[7]));
        if (s_dutrx_len < need) {
            continue;
        }
        {
            uint16_t cmd = 0u, seq = 0u, dlen = 0u;
            if (Frame_Parse(s_dutrx, need, &cmd, &seq, s_dut_last_data, &dlen) != 0u) {
                s_dut_last_cmd = cmd;
                s_dut_last_seq = seq;
                s_dut_last_len = dlen;
                s_dut_seen++;
                DutFake_Reply(cmd, seq);
            } else {
                s_dut_bad_crc++;
            }
        }
        s_dutrx_len = 0u;
    }
}
#endif /* JIG_DUT_LINK */

/* GPIO 替身一肩挑两件事：
 *   1) 治具自己的 SPI Flash 片选 (PB12)
 *   2) SWD 烧录引脚 → 送进线级目标模型 sim_target，由它按 ADIv5 解码波形 */
void HAL_GPIO_Init(GPIO_TypeDef *port, GPIO_InitTypeDef *init)
{
    sim_gpio_dir(port, (uint16_t)init->Pin,
                 (init->Mode == GPIO_MODE_INPUT) ? 0u : 1u);
}

void HAL_GPIO_WritePin(GPIO_TypeDef *port, uint16_t pin, GPIO_PinState state)
{
    if (port == GPIOB && pin == GPIO_PIN_12) {          /* 治具 Flash 片选 */
        if (state == GPIO_PIN_RESET) flash_cs_low();
        else                         flash_cs_high();
    }
    sim_gpio_write(port, pin, (state == GPIO_PIN_SET) ? 1u : 0u);
}

GPIO_PinState HAL_GPIO_ReadPin(GPIO_TypeDef *port, uint16_t pin)
{
    return sim_gpio_read(port, pin) ? GPIO_PIN_SET : GPIO_PIN_RESET;
}

HAL_StatusTypeDef HAL_SPI_TransmitReceive(SPI_HandleTypeDef *hspi,
                                          uint8_t *pTxData, uint8_t *pRxData,
                                          uint16_t Size, uint32_t Timeout)
{
    (void)hspi; (void)Timeout;
    for (uint16_t i = 0; i < Size; i++) pRxData[i] = flash_xfer(pTxData[i]);
    return HAL_OK;
}

void BSP_DBG_Send(const char *s) { (void)s; }
void BSP_DBG_SendHexByte(uint8_t b) { (void)b; }

/*==============================================================================
 * USB CDC 替身
 *============================================================================*/
static USBD_CDC_HandleTypeDef g_cdc;
USBD_HandleTypeDef hUsbDeviceFS;

static uint8_t  tx_buf[APP_FRAME_BUF_SIZE];   /* 必须装得下整帧(0x000E 应答 87B) */
static uint8_t  s_ascii_in[64];               /* T28：模拟 PC 敲进来的字节 */
static uint16_t tx_len = 0;
static uint32_t tx_count = 0;
static uint32_t g_tx_oversize = 0;            /* 应答超过替身缓冲的次数(必须为 0) */

uint8_t CDC_Transmit_FS(uint8_t *Buf, uint16_t Len)
{
    if (Len <= sizeof(tx_buf)) {
        memcpy(tx_buf, Buf, Len);
        tx_len = Len;
        tx_count++;
    } else {
        /* 不能静默丢：否则"应答太长"会伪装成"根本没应答" */
        printf("  !! CDC 替身缓冲太小: %u > %u\n", (unsigned)Len, (unsigned)sizeof(tx_buf));
        g_tx_oversize++;
        tx_len = 0;
    }
    return USBD_OK;
}
void MX_USB_DEVICE_Init(void) { }

/*==============================================================================
 * 测试辅助
 *============================================================================*/
static int g_checks = 0, g_fails = 0;
#define CHECK(c, m) do { g_checks++; if (!(c)) { printf("  FAIL: %s (line %d)\n", m, __LINE__); g_fails++; } } while (0)

static void pump(int n) { for (int i = 0; i < n; i++) APP_Run(); }

/* 推进主循环直到 U1 主动发出指定命令的帧 (用于 0x0070 这种"烧完才回"的 ACK) */
static int wait_for_cmd(uint16_t cmd, uint16_t *seq_out, uint8_t *st_out)
{
    for (uint32_t i = 0; i < 200000u; i++) {
        tx_len = 0;
        APP_Run();
        if (tx_len > 0) {
            uint16_t c = 0, s = 0, d = 0;
            uint8_t  dd[FRAME_MAX_DATA];
            if (Frame_Parse(tx_buf, tx_len, &c, &s, dd, &d) && c == cmd) {
                if (seq_out) *seq_out = s;
                if (st_out)  *st_out  = d ? dd[0] : 0xFFu;
                return 1;
            }
        }
    }
    return 0;
}

/* 发一帧给 U1 并收集应答; 返回应答帧长度(0=无应答) */
static uint16_t pc_send(uint16_t cmd, uint16_t seq, const uint8_t *data, uint16_t len)
{
    uint8_t  raw[8 + FRAME_MAX_DATA + 2];
    uint16_t n = Frame_Build(raw, cmd, seq, data, len);
    tx_len = 0;
    APP_USB_Receive(raw, n);
    pump(8);
    return tx_len;
}

/* 断言应答内容 */
static void expect_ack(uint16_t cmd, uint16_t seq, const uint8_t *st, uint16_t stlen, const char *what)
{
    uint16_t cmd_o = 0, seq_o = 0, dlen = 0;
    uint8_t  d[FRAME_MAX_DATA];
    int ok = (tx_len > 0) && Frame_Parse(tx_buf, tx_len, &cmd_o, &seq_o, d, &dlen);
    CHECK(ok, what);
    if (!ok) { printf("      (%s 无有效应答)\n", what); return; }
    CHECK(cmd_o == cmd, what);
    CHECK(seq_o == seq, "SEQ 回显正确");
    CHECK(dlen == stlen, "ACK LEN 正确");
    if (dlen == stlen) CHECK(memcmp(d, st, stlen) == 0, "ACK 内容正确");
    if (dlen != stlen || memcmp(d, st, stlen) != 0) {
        printf("      得到:"); for (uint16_t i = 0; i < dlen; i++) printf(" %02X", d[i]); printf("\n");
        printf("      期望:"); for (uint16_t i = 0; i < stlen; i++) printf(" %02X", st[i]); printf("\n");
    }
}

/* 走一遍完整上传流程 */
static void upload(uint16_t *seq, const uint8_t *img, uint32_t size, uint8_t ver[3])
{
    uint8_t  d9[9];
    uint16_t frames = (uint16_t)((size + 255u) / 256u);

    d9[0] = (uint8_t)(frames >> 8); d9[1] = (uint8_t)(frames);
    d9[2] = (uint8_t)(size >> 24);  d9[3] = (uint8_t)(size >> 16);
    d9[4] = (uint8_t)(size >> 8);   d9[5] = (uint8_t)(size);
    d9[6] = ver[0]; d9[7] = ver[1]; d9[8] = ver[2];

    pc_send(MGR_CMD_FW_START, (*seq)++, d9, 9);

    for (uint16_t i = 0; i < frames; i++) {
        uint32_t off = (uint32_t)i * 256u;
        uint16_t n = (uint16_t)((size - off > 256u) ? 256u : (size - off));
        pc_send(MGR_CMD_FW_DATA, (*seq)++, &img[off], n);
    }
    pc_send(MGR_CMD_FW_END, (*seq)++, 0, 0);
}

/*==============================================================================
 * 用例
 *============================================================================*/
static uint8_t g_boot[101];
static uint8_t g_app[301];

static void init_env(void)
{
    memset(g_spi, 0xFF, sizeof(g_spi));
    memset(&fl, 0, sizeof(fl));
    fl.sr1 = 0x00;
    g_tick = 0;
    tx_len = 0; tx_count = 0;
    sim_target_reset();
    sim_target_set_read_ta((uint8_t)SWD_TA_BEFORE_READ_DATA);
    hUsbDeviceFS.pClassData = &g_cdc;
    g_cdc.TxState = 0;
    for (int i = 0; i < 101; i++) g_boot[i] = (uint8_t)(0x11 + i * 3);
    for (int i = 0; i < 301; i++) g_app[i]  = (uint8_t)(0xA0 + i * 7);

    MEAS_Init();    /* 固件里由 main.c 调；这里补上, 0x000E 才拿得到配置指纹 */
    MEAS_Init();    /* 固件里由 main.c 调；这里补上, 0x000E 才拿得到配置指纹 */
    APP_Init();     /* 内部会调 GD25Q 初始化等 */
    GD25Q_Init();
}

static void t1_flash_id(void)
{
    printf("[T1] SPII Flash 器件识别\n");
    init_env();
    CHECK(GD25Q_Test() == 1u, "GD25Q_Test 通过 (制造商 0xC8 + 容量 0x18)");
    printf("    JEDEC ID = 0x%06X (期望 0xC84018)\n", GD25Q_ReadID());
    CHECK(GD25Q_ReadID() == 0xC84018u, "JEDEC ID = 0xC84018");
}

static void t2_empty_and_unknown(void)
{
    printf("[T2] 空 Flash 查询 / 未知命令 / CRC 错误 / SEQ 回显\n");
    init_env();

    { uint8_t e[4] = {0x06, 0x00, 0xFF, 0xFF};
      pc_send(MGR_CMD_QUERY_FLASH_VER, 0x0101, 0, 0);
      expect_ack(MGR_CMD_QUERY_FLASH_VER, 0x0101, e, 4, "空 Flash → ERR_NO_FW"); }

    { uint8_t e[4] = {0x06, 0x00, 0xFF, 0xFF};
      pc_send(MGR_CMD_QUERY_FOC_VER, 0x0202, 0, 0);
      expect_ack(MGR_CMD_QUERY_FOC_VER, 0x0202, e, 4, "0x000D 被烧板 App 区为空 → 0x06(无固件) + 00FFFF"); }

    { uint8_t e[1] = {0xFF};
      pc_send(0x1234, 0x0303, 0, 0);
      expect_ack(0x1234, 0x0303, e, 1, "未知命令 → NIY 0xFF"); }

    /* CRC 被破坏 → 按协议不回 ACK */
    { uint8_t raw[16]; uint16_t n = Frame_Build(raw, MGR_CMD_QUERY_FLASH_VER, 0x0404, 0, 0);
      raw[n - 1] ^= 0xFF; tx_len = 0; APP_USB_Receive(raw, n); pump(8);
      CHECK(tx_len == 0, "CRC 错误 → 不回 ACK"); }

    /* 非 IDLE 语义: 空闲态发 DATA/END 应为 BUSY */
    { uint8_t e[1] = {0x01};
      pc_send(MGR_CMD_FW_DATA, 0x0505, (const uint8_t *)"AB", 2);
      expect_ack(MGR_CMD_FW_DATA, 0x0505, e, 1, "空闲态 0x000A → BUSY"); }
    { uint8_t e[1] = {0x01};
      pc_send(MGR_CMD_FW_END, 0x0606, 0, 0);
      expect_ack(MGR_CMD_FW_END, 0x0606, e, 1, "空闲态 0x000B → BUSY"); }
}

static void t3_upload_and_slot_switch(void)
{
    printf("[T3] 上传 → 查询 → 双槽切换\n");
    init_env();
    uint16_t seq = 0x1000;
    uint8_t  ver1[3] = {1, 2, 3};
    uint8_t  ver2[3] = {4, 5, 6};

    upload(&seq, g_boot, sizeof(g_boot), ver1);
    { uint8_t e[4] = {0x00, 1, 2, 3};
      pc_send(MGR_CMD_QUERY_FLASH_VER, seq++, 0, 0);
      expect_ack(MGR_CMD_QUERY_FLASH_VER, seq - 1, e, 4, "第一次上传后查询到 v1.2.3"); }

    /* SPI Flash Slot0 内容逐字节核对 */
    { uint8_t hdr[FW_HEAD_SIZE];
      GD25Q_ReadBuffer(hdr, FW_SLOT0_BASE, FW_HEAD_SIZE);
      uint32_t sz = (uint32_t)hdr[0] | ((uint32_t)hdr[1] << 8) | ((uint32_t)hdr[2] << 16) | ((uint32_t)hdr[3] << 24);
      CHECK(sz == sizeof(g_boot), "Slot0 头部 size 正确");
      CHECK(hdr[4] == 1 && hdr[5] == 2 && hdr[6] == 3, "Slot0 头部版本正确");
      CHECK(hdr[7] == FW_FLAG_COMPLETE, "Slot0 完整标志 = 0xA5");
      static uint8_t rd[128];
      GD25Q_ReadBuffer(rd, FW_SLOT0_BASE + FW_HEAD_SIZE, (uint16_t)sizeof(g_boot));
      CHECK(memcmp(rd, g_boot, sizeof(g_boot)) == 0, "Slot0 镜像逐字节一致"); }

    /* 第二次上传自动切到 Slot1 */
    upload(&seq, g_app, sizeof(g_app), ver2);
    { uint8_t e[4] = {0x00, 4, 5, 6};
      pc_send(MGR_CMD_QUERY_FLASH_VER, seq++, 0, 0);
      expect_ack(MGR_CMD_QUERY_FLASH_VER, seq - 1, e, 4, "第二次上传后查询到 v4.5.6 (App 槽优先)"); }

    { static uint8_t rd[320];
      GD25Q_ReadBuffer(rd, FW_SLOT1_BASE + FW_HEAD_SIZE, (uint16_t)sizeof(g_app));
      CHECK(memcmp(rd, g_app, sizeof(g_app)) == 0, "Slot1 镜像逐字节一致"); }

    printf("    扇区擦除=%u 页编程=%u WREN=%u RDSR=%u WEL违规=%u\n",
           fl.se_count, fl.pp_count, fl.wren_count, fl.rdsr_count, fl.wel_violations);
    CHECK(fl.wel_violations == 0, "无 WEL 违规 (每次擦写前都先 WREN)");
}

static void t4_start_param_errors(void)
{
    printf("[T4] 0x0009 参数异常\n");
    init_env();
    uint16_t seq = 0x2000;
    uint8_t  e2[1] = {0x02};

    { uint8_t d[8] = {0, 2, 0, 0, 1, 0, 1, 2};      /* LEN=8 < 9 */
      pc_send(MGR_CMD_FW_START, seq++, d, 8);
      expect_ack(MGR_CMD_FW_START, seq - 1, e2, 1, "LEN<9 → ERR_SIZE"); }

    { uint8_t d[9] = {0, 2, 0, 0, 0, 0, 1, 2, 3};   /* size=0 */
      pc_send(MGR_CMD_FW_START, seq++, d, 9);
      expect_ack(MGR_CMD_FW_START, seq - 1, e2, 1, "size=0 → ERR_SIZE"); }

    { uint8_t d[9] = {0, 0, 0, 0, 0, 10, 1, 2, 3};  /* frames=0 */
      pc_send(MGR_CMD_FW_START, seq++, d, 9);
      expect_ack(MGR_CMD_FW_START, seq - 1, e2, 1, "frames=0 → ERR_SIZE"); }

    { uint8_t d[9] = {0, 2, 0, 1, 0, 0, 1, 2, 3};   /* size=65536 > 限 */
      pc_send(MGR_CMD_FW_START, seq++, d, 9);
      expect_ack(MGR_CMD_FW_START, seq - 1, e2, 1, "size 越界 → ERR_SIZE"); }
}

static void t5_overflow_and_latch(void)
{
    printf("[T5] 上传溢出 → 错误锁存\n");
    init_env();
    uint16_t seq = 0x3000;
    uint8_t  ver[3] = {1, 0, 0};
    uint8_t  d9[9] = {0, 1, 0, 0, 0, 100, 1, 0, 0};   /* 声报 100 字节 */
    uint8_t  ok[1] = {0x00}, ov[1] = {0x0D};

    pc_send(MGR_CMD_FW_START, seq++, d9, 9);
    expect_ack(MGR_CMD_FW_START, seq - 1, ok, 1, "START OK");

    { uint8_t big[200]; memset(big, 0x5A, sizeof(big));
      pc_send(MGR_CMD_FW_DATA, seq++, big, 200);       /* 超出发报 100 */
      expect_ack(MGR_CMD_FW_DATA, seq - 1, ov, 1, "溢出 → ERR_OVERFLOW 0x0D"); }

    { uint8_t small[10]; memset(small, 0x11, sizeof(small));
      pc_send(MGR_CMD_FW_DATA, seq++, small, 10);      /* 锁存后仍回 0x0D, 不写 Flash */
      expect_ack(MGR_CMD_FW_DATA, seq - 1, ov, 1, "锁存后仍回 0x0D"); }

    pc_send(MGR_CMD_FW_END, seq++, 0, 0);
    expect_ack(MGR_CMD_FW_END, seq - 1, ov, 1, "END 回锁存错误码");

    { uint8_t e[4] = {0x07, 0x00, 0xFF, 0xFF};
      pc_send(MGR_CMD_QUERY_FLASH_VER, seq++, 0, 0);
      expect_ack(MGR_CMD_QUERY_FLASH_VER, seq - 1, e, 4, "未写完整标志 → ERR_INCOMPLETE"); }
    (void)ver;
}

static void t6_count_mismatch(void)
{
    printf("[T6] 字节数与声明不符 → ERR_COUNT 且不写完整标志\n");
    init_env();
    uint16_t seq = 0x4000;
    uint8_t  d9[9] = {0, 1, 0, 0, 0, 200, 2, 0, 0};   /* 声明 200 字节 / 1 帧 */
    uint8_t  ok[1] = {0x00}, ec[1] = {0x0A};

    pc_send(MGR_CMD_FW_START, seq++, d9, 9);
    expect_ack(MGR_CMD_FW_START, seq - 1, ok, 1, "START OK");

    { uint8_t d[100]; memset(d, 0x33, sizeof(d));
      pc_send(MGR_CMD_FW_DATA, seq++, d, 100);        /* 只发 100, 声明 200 */
      expect_ack(MGR_CMD_FW_DATA, seq - 1, ok, 1, "DATA OK"); }

    pc_send(MGR_CMD_FW_END, seq++, 0, 0);
    expect_ack(MGR_CMD_FW_END, seq - 1, ec, 1, "END → ERR_COUNT 0x0A");

    { uint8_t e[4] = {0x07, 0x00, 0xFF, 0xFF};        /* 有内容但无完整标志 */
      pc_send(MGR_CMD_QUERY_FLASH_VER, seq++, 0, 0);
      expect_ack(MGR_CMD_QUERY_FLASH_VER, seq - 1, e, 4, "完整标志未写 → ERR_INCOMPLETE"); }
}

static void t7_upload_timeout(void)
{
    printf("[T7] 上传 10s 无新数据 → ERR_TIMEOUT 并回 IDLE\n");
    init_env();
    uint16_t seq = 0x5000;
    uint8_t  d9[9] = {0, 1, 0, 0, 1, 0, 1, 0, 0};     /* 声明 256 字节 1 帧 */
    uint8_t  ok[1] = {0x00};

    pc_send(MGR_CMD_FW_START, seq++, d9, 9);
    expect_ack(MGR_CMD_FW_START, seq - 1, ok, 1, "START OK");

    /* 不发数据, 直接推进 20 秒 */
    tx_len = 0;
    host_advance(20000u);
    pump(4);
    { uint8_t e[1] = {0x0C};
      expect_ack(MGR_CMD_FW_DATA, seq - 1, e, 1, "超时 → ACK ERR_TIMEOUT 0x0C"); }

    /* 已回 IDLE: 再发 DATA 应为 BUSY */
    { uint8_t e[1] = {0x01};
      pc_send(MGR_CMD_FW_DATA, seq++, (const uint8_t *)"X", 1);
      expect_ack(MGR_CMD_FW_DATA, seq - 1, e, 1, "超时后回到 IDLE (DATA → BUSY)"); }
}

static void t8_program_precheck(void)
{
    printf("[T8] 0x0070 前置检查\n");
    init_env();
    uint16_t seq = 0x6000;
    uint8_t  ok[1] = {0x00};
    uint8_t  ver[3] = {1, 0, 0};

    { uint8_t e[1] = {0x06};
      pc_send(MGR_CMD_PROGRAM, seq++, 0, 0);
      expect_ack(MGR_CMD_PROGRAM, seq - 1, e, 1, "两槽皆空 → ERR_NO_FW"); }

    upload(&seq, g_boot, sizeof(g_boot), ver);        /* 只填 Slot0 */
    { uint8_t e[1] = {0x06};
      pc_send(MGR_CMD_PROGRAM, seq++, 0, 0);
      expect_ack(MGR_CMD_PROGRAM, seq - 1, e, 1, "仅 Slot0 有固件 → ERR_NO_FW"); }

    upload(&seq, g_app, sizeof(g_app), ver);          /* 填 Slot1 */
    (void)ok;
}

static void t9_full_program_chain(void)
{
    printf("[T9] 端到端: PC帧 → 上传 → SPI Flash → 0x0112 SWD → 被烧板 Flash\n");
    init_env();
    uint16_t seq = 0x7000;
    uint8_t  ver1[3] = {1, 2, 3};
    uint8_t  ver2[3] = {4, 5, 6};
    uint8_t *dut = sim_target_flash();          /* 线级模型里的 F405 Flash */
    uint32_t clk0 = SWD_PhyClockCount();

    /* 上传两段镜像 (自动 Slot0 → Slot1) */
    upload(&seq, g_boot, sizeof(g_boot), ver1);
    upload(&seq, g_app,  sizeof(g_app),  ver2);

    /* 触发烧录并一直推进到 ACK */
    {
        uint8_t  raw[8 + 2];
        uint16_t n = Frame_Build(raw, MGR_CMD_PROGRAM, seq, 0, 0);
        tx_len = 0;
        APP_USB_Receive(raw, n);
    }
    { uint16_t rseq = 0; uint8_t rst = 0xFF;
      CHECK(wait_for_cmd(MGR_CMD_PROGRAM, &rseq, &rst) == 1, "收到 0x0112 应答");
      CHECK(rseq == seq, "0x0112 ACK 的 SEQ = 受理时的序号 (锁存生效)");
      CHECK(rst == APP_STATUS_OK, "0x0112 ACK 状态 = OK"); }
    seq++;

    /* 被烧板 Flash 逐字节核对 */
    CHECK(memcmp(&dut[DUT_BOOT_ADDR - 0x08000000u], g_boot, sizeof(g_boot)) == 0,
          "被烧板 Bootloader 区 == 源镜像");
    CHECK(memcmp(&dut[DUT_APP_ADDR - 0x08000000u], g_app, sizeof(g_app)) == 0,
          "被烧板 App 区 == 源镜像");
    /* 线路层与协议层的不变量 */
    CHECK(sim_target_switch_seen() == 1u,   "JTAG→SWD 切换序列被目标识别");
    CHECK(sim_target_contention() == 0u,    "SWDIO 全程零总线冲突");
    CHECK(sim_target_parity_errors() == 0u, "线路层零奇偶校验错");
    /* 擦除次数按**分区几何**推导，不写死数字（改分区地址时这条要跟着走）：
     * Boot = 0x10000 (扇区 0..3) → 4 个 16K 扇区；App 镜像 g_app 只占 1 个 16K 扇区 → 共 5 个。 */
    CHECK(sim_target_erases() ==
              ((DUT_BOOT_SIZE / 0x4000u) + 1u),
          "按扇区擦除次数 = Boot 扇区数(64K/16K=4) + App 1 个 = 5");
    CHECK(sim_target_locked_reject() == 0u, "没有在未解锁状态下碰过目标 Flash");
    CHECK(sim_target_halted() == 0u,        "烧完目标在运行态 (SYSRESETREQ 后 S_HALT=0)");
    CHECK(SWDP_LastIdcode() == 0x2BA01477u, "SW-DP IDCODE = 0x2BA01477");
    CHECK(SWDP_LastDevId() == 0x0413u,      "目标 DEV_ID = 0x413 (F405/407)");

    printf("    IDCODE=0x%08lX DEV_ID=0x%03lX 擦扇区=%lu 编程字=%lu SWD 事务=%lu\n",
           (unsigned long)SWDP_LastIdcode(), (unsigned long)SWDP_LastDevId(),
           (unsigned long)sim_target_erases(), (unsigned long)sim_target_programs(),
           (unsigned long)sim_target_transactions());
    /* 实测时钟周期 → 上板节拍推算（SWCLK 频率由 SWD_DELAY_LOOPS 决定） */
    {
        uint32_t cycles = SWD_PhyClockCount() - clk0;
        uint32_t bytes  = (uint32_t)(sizeof(g_boot) + sizeof(g_app));
        uint32_t per_kb = (uint32_t)(((unsigned long long)cycles * 1024ull) / bytes);
        printf("    烧 %u B 用了 %lu 个 SWCLK 周期 (%lu 周期/KB)\n",
               bytes, (unsigned long)cycles, (unsigned long)per_kb);
        printf("    推算 300KB: %.1f s @1MHz   %.1f s @2MHz   %.1f s @4MHz\n",
               (double)per_kb * 300.0 / 1.0e6, (double)per_kb * 300.0 / 2.0e6,
               (double)per_kb * 300.0 / 4.0e6);
        printf("    (只算 SWD 链路时间; 真实擦除还要等 Flash: 128KB 扇区约 1.3s/个)\n");
    }
}

static void t10_busy_and_target_absent(void)
{
    printf("[T10] 烧录中 BUSY / 目标不在时干净失败\n");
    uint16_t seq = 0x8000;
    uint8_t  ver[3] = {7, 7, 7};
    uint8_t *dut = sim_target_flash();

    /* ── 10a: 烧录途中插入命令必须回 BUSY ── */
    init_env();
    upload(&seq, g_boot, sizeof(g_boot), ver);
    upload(&seq, g_app,  sizeof(g_app),  ver);
    {
        uint8_t  raw[8 + 2];
        uint16_t n = Frame_Build(raw, MGR_CMD_PROGRAM, seq, 0, 0);
        tx_len = 0;
        APP_USB_Receive(raw, n);
    }
    uint16_t prog_seq = seq;
    seq++;

    APP_Run();
    APP_Run();                        /* 已进入 PROGRAMMING, 但远未烧完 */

    {
        uint8_t  raw[16];
        uint16_t n = Frame_Build(raw, MGR_CMD_QUERY_FLASH_VER, seq++, 0, 0);
        uint8_t  e[1] = {APP_STATUS_BUSY};
        tx_len = 0;
        APP_USB_Receive(raw, n);
        APP_Run();
        expect_ack(MGR_CMD_QUERY_FLASH_VER, seq - 1, e, 1, "烧录中查版本 → BUSY");
    }

    { uint16_t rseq = 0; uint8_t rst = 0xFF;
      CHECK(wait_for_cmd(MGR_CMD_PROGRAM, &rseq, &rst) == 1, "收到 0x0112 应答");
      CHECK(rseq == prog_seq, "0x0112 ACK SEQ = 受理序号");
      CHECK(rst == APP_STATUS_OK, "0x0112 → OK"); }
    CHECK(memcmp(&dut[DUT_BOOT_ADDR - 0x08000000u], g_boot, sizeof(g_boot)) == 0, "Boot 区一致");
    CHECK(memcmp(&dut[DUT_APP_ADDR - 0x08000000u], g_app, sizeof(g_app)) == 0, "App 区一致");

    /* ── 10b: 目标不在（线悬空）→ 必须干净失败, 一个字节都不写 ── */
    init_env();
    sim_target_attach(0u);
    upload(&seq, g_boot, sizeof(g_boot), ver);
    upload(&seq, g_app,  sizeof(g_app),  ver);
    {
        uint8_t  raw[8 + 2];
        uint16_t n = Frame_Build(raw, MGR_CMD_PROGRAM, seq, 0, 0);
        tx_len = 0;
        APP_USB_Receive(raw, n);
    }
    { uint16_t rseq = 0; uint8_t rst = 0xFF;
      CHECK(wait_for_cmd(MGR_CMD_PROGRAM, &rseq, &rst) == 1, "目标不在时仍然回 ACK (不死等)");
      CHECK(rst == APP_STATUS_ERR_PROG, "目标不在 → ERR_PROG");
      CHECK(SWDP_LastSwdError() != 0u, "记录了底层 SWD 错误码供诊断"); }
    CHECK(sim_target_programs() == 0u, "目标不在时一个字节都没写进目标 Flash");
    CHECK(sim_target_erases() == 0u,   "目标不在时没有擦除任何扇区");
}

static void t13_flash_offline_detected(void)
{
    printf("[T13] SPI Flash 不在线/型号不对 → 第一条命令就以 ERR_FLASH 暴露\n");
    init_env();
    fl.force_wrong_id = 1u;          /* RDID 返回全 0 */
    CHECK(GD25Q_Test() == 0u, "GD25Q_Test 判定失败");

    uint16_t seq = 0x9000;
    uint8_t  d9[9] = {0, 1, 0, 0, 1, 0, 1, 0, 0};
    uint8_t  e3[1] = {0x03};

    pc_send(MGR_CMD_FW_START, seq++, d9, 9);
    expect_ack(MGR_CMD_FW_START, seq - 1, e3, 1, "0x0009 → ERR_FLASH 0x03");

    CHECK(fl.se_count == 0u && fl.pp_count == 0u, "未做任何擦除/写入");
    CHECK(fl.wel_violations == 0u, "无 WEL 违规");
}

/*==============================================================================
 * T14: 0x000E 测量 (mode=0 只读 / mode=1 触发测试)
 *----------------------------------------------------------------------------
 * 本测试环境没有 U2（HAL_UART_Transmit_IT 永不完成），所以走的是
 * "对侧无应答"路径：应答里应带 MEAS_CC_F_NO_PEER、对侧脉冲段为"无数据"，
 * 但结论仍是 OK（MEAS_CC_REQUIRED=0 / PULSE_JUDGE_ENABLE=0，bring-up 期
 * 不因为 U2 缺席或没信号就判失败）。
 *============================================================================*/
static void pump_meas(int n)
{
    for (int i = 0; i < n; i++) {
        U2Fake_Poll();        /* 有假 U2 时推它一把；没开就是空转 */
        MEAS_Poll();          /* 固件里 main.c 就是这么调的 */
        APP_Run();
    }
}

/* ── 脉冲采集层的宿主替身 ──
 * pulse_cfg.c 是板级的（直接读写 TIM 寄存器），宿主上没法编译，所以这里替身：
 * 算法层 pulse_cap.c 是真的，只有"寄存器那一层"换成"窗口一开始就灌一段
 * 20kHz/50% 的假 PWM 边沿"——等于把一个理想信号发生器接在三相上。 */
static uint8_t g_pulse_edges_on = 1u;      /* 用例可以关掉，模拟"没信号" */

/* 假波形的频率/占空比：默认 20kHz/50%（= 原来写死的行为，老用例不受影响）。
 * 第 4 步（AS:PW）要验的是 1kHz/50%，所以做成可设的。 */
static uint32_t g_pulse_freq_hz = 20000u;
static uint16_t g_pulse_duty_pm = 500u;
/* bit0/1/2 = A/B/C 要不要灌边沿。默认全灌；关掉某一位就等于"那一相没波形"，
 * 用来验"单相缺失"（真机上对应某一相驱动级坏掉）。*/
static uint8_t  g_pulse_phase_mask = 0x07u;

static void pulse_fake_set(uint32_t freq_hz, uint16_t duty_pm)
{
    if ((freq_hz < 10u) || (freq_hz > 60000u)) {
        return;                            /* 超出固件量程：不改 */
    }
    if (duty_pm == 0u)   { duty_pm = 500u; }
    if (duty_pm > 1000u) { duty_pm = 1000u; }
    g_pulse_freq_hz = freq_hz;
    g_pulse_duty_pm = duty_pm;
}

static void pulse_fake_phase_mask(uint8_t mask)
{
    g_pulse_phase_mask = (uint8_t)(mask & 0x07u);
}

static void pulse_fake_reset(void)
{
    g_pulse_freq_hz = 20000u;
    g_pulse_duty_pm = 500u;
    g_pulse_phase_mask = 0x07u;
}

/* ── ⑤ 编码器脉冲发生层的宿主替身 ──
 * enc_gen.c 直接写 TIM4 寄存器与 GPIO，宿主机上编译不了，所以这里替身。
 * 替身要能验证的是 **app 层那一段**：请求解析、参数校验、非阻塞状态机、ACK 布局。
 * 所以它把"规划"原样记下来（供断言），并在一次 Poll 之后按"全部发完"收尾。 */
static uint8_t  g_enc_busy;
static uint8_t  g_enc_done;
static uint8_t  g_enc_abort;
static uint16_t g_enc_need[3];
static uint16_t g_enc_flip[3];
static uint16_t g_enc_rec_cnt[3];       /* 记下 Start 收到的规划，供断言 */
static uint16_t g_enc_rec_freq[3];
static uint8_t  g_enc_start_fail;       /* 1 = 强制 Start 失败（测失败路径） */
static uint32_t g_enc_start_calls;

void EncGen_Init(void)
{
    g_enc_busy = 0u; g_enc_done = 0u; g_enc_abort = 0u;
    g_enc_need[0] = g_enc_need[1] = g_enc_need[2] = 0u;
    g_enc_flip[0] = g_enc_flip[1] = g_enc_flip[2] = 0u;
    g_enc_rec_cnt[0] = g_enc_rec_cnt[1] = g_enc_rec_cnt[2] = 0u;
    g_enc_rec_freq[0] = g_enc_rec_freq[1] = g_enc_rec_freq[2] = 0u;
    g_enc_start_fail = 0u;
    g_enc_start_calls = 0u;
}

uint8_t EncGen_Start(const enc_plan_t *p)
{
    g_enc_start_calls++;
    if ((p == 0) || (g_enc_start_fail != 0u) || (g_enc_busy != 0u)) {
        return 0u;
    }
    g_enc_rec_cnt[0] = p->cnt_a;  g_enc_rec_freq[0] = p->freq_a_hz;
    g_enc_rec_cnt[1] = p->cnt_b;  g_enc_rec_freq[1] = p->freq_b_hz;
    g_enc_rec_cnt[2] = p->cnt_z;  g_enc_rec_freq[2] = p->freq_z_hz;
    g_enc_need[0] = (uint16_t)((uint32_t)p->cnt_a * 2u);
    g_enc_need[1] = (uint16_t)((uint32_t)p->cnt_b * 2u);
    g_enc_need[2] = (uint16_t)((uint32_t)p->cnt_z * 2u);
    g_enc_done = 0u;
    g_enc_abort = 0u;
    g_enc_busy = 1u;
    return 1u;
}

void EncGen_Poll(void)
{
    if (g_enc_busy == 0u) {
        return;
    }
    /* 替身：一次 Poll 就当"全部发完" */
    g_enc_flip[0] = g_enc_need[0];
    g_enc_flip[1] = g_enc_need[1];
    g_enc_flip[2] = g_enc_need[2];
    g_enc_done = 0u;
    if (g_enc_need[0] > 0u) { g_enc_done |= 0x01u; }
    if (g_enc_need[1] > 0u) { g_enc_done |= 0x02u; }
    if (g_enc_need[2] > 0u) { g_enc_done |= 0x04u; }
    g_enc_busy = 0u;
}

void     EncGen_Stop(void)      { g_enc_busy = 0u; }
uint8_t  EncGen_Busy(void)      { return g_enc_busy; }
uint8_t  EncGen_DoneMask(void)  { return g_enc_done; }
uint8_t  EncGen_Aborted(void)   { return g_enc_abort; }
uint16_t EncGen_Flips(uint8_t ph) { return (ph < 3u) ? g_enc_flip[ph] : 0u; }
uint16_t EncGen_LastPsc(void)   { return 0u; }

/* 寄存器快照（真板上读 TIM4；宿主没有硬件，返回 0） */
void EncGen_DbgSnapshot(uint16_t out[6])
{
    int i;

    if (out == 0) {
        return;
    }
    for (i = 0; i < 6; i++) {
        out[i] = 0u;
    }
}
uint32_t EncGen_LastClkHz(void) { return ENC_PLAN_TIM_CLK_HZ; }

void PULSE_CfgInit(void) { }

void PULSE_CfgStart(uint16_t window_ms)
{
    uint32_t k;
    uint32_t t;
    uint32_t per;
    uint32_t high;
    uint32_t win;

    PULSE_Start(window_ms);
    if (g_pulse_edges_on == 0u) {
        return;                            /* 没信号：一个边沿都不灌 */
    }
    /* 捕获计时器 72 tick = 1us。按配置的频率/占空比灌满整个窗口：
     *   默认 20kHz/50% → 周期 3600 tick、高电平 1800 tick（与原来写死的完全一致）；
     *   设成 1kHz/50%  → 周期 72000 tick、高电平 36000 tick。 */
    per = 72000000u / g_pulse_freq_hz;
    if (per < 4u) {
        per = 4u;
    }
    high = (per * (uint32_t)g_pulse_duty_pm) / 1000u;
    if (high == 0u)      { high = 1u; }
    if (high >= per)     { high = per - 1u; }
    win = (uint32_t)window_ms * 72000u;

    t = 0u;
    for (k = 0u; (t < win) && (k < 60000u); k++) {
        if ((g_pulse_phase_mask & 0x01u) != 0u) {
            PULSE_OnEdge(PULSE_PH_A, t);
            PULSE_OnEdge(PULSE_PH_A, t + high);
        }
        if ((g_pulse_phase_mask & 0x02u) != 0u) {
            PULSE_OnEdge(PULSE_PH_B, t);
            PULSE_OnEdge(PULSE_PH_B, t + high);
        }
        if ((g_pulse_phase_mask & 0x04u) != 0u) {
            PULSE_OnEdge(PULSE_PH_C, t);
            PULSE_OnEdge(PULSE_PH_C, t + high);
        }
        t += per;
    }
}

void     PULSE_CfgStop(void) { PULSE_Stop(); }
uint8_t  PULSE_CfgPoll(void) { return PULSE_Poll(); }
uint32_t PULSE_CfgLostEdges(void) { return 0u; }

/* 等 0x000E 的应答：必须同时喂 MEAS_Poll（固件 main.c 的循环顺序就是
 * MEAS_Poll() → APP_Run()），只用 APP_Run 的话稳定计数永远不涨 → 必然超时。 */
static int wait_meas_ack(uint16_t expect_cmd, uint16_t *seq_out, uint8_t *st_out)
{
    for (uint32_t i = 0; i < 200000u; i++) {
        tx_len = 0;
        U2Fake_Poll();        /* 有假 U2 时推它一把；没开就是空转 */
        MEAS_Poll();
        APP_Run();
        if (tx_len > 0) {
            uint16_t c = 0, s = 0, d = 0;
            uint8_t  dd[FRAME_MAX_DATA];
            if (Frame_Parse(tx_buf, tx_len, &c, &s, dd, &d) && c == expect_cmd) {
                if (seq_out) *seq_out = s;
                if (st_out)  *st_out  = d ? dd[0] : 0xFFu;
                return 1;
            }
        }
    }
    return 0;
}

/* 解析 0x000E 的 77B 应答（状态 + 快照 + 交叉校验块） */
static int parse_meas_resp(uint8_t *status, meas_snapshot_t *snap,
                           uint8_t *cc_flags, uint16_t *mask, meas_snapshot_t *peer)
{
    uint16_t cmd_o = 0, seq_o = 0, dlen = 0;
    uint8_t  d[FRAME_MAX_DATA];

    if ((tx_len == 0u) || (!Frame_Parse(tx_buf, tx_len, &cmd_o, &seq_o, d, &dlen))) {
        return 0;
    }
    if (cmd_o != MGR_CMD_MEASURE || dlen != MEAS_PACK_RESP_LEN) {
        return 0;
    }
    if (status) *status = d[0];
    if (snap)   (void)MeasPack_UnpackSnapshot(&d[1], MEAS_PACK_SNAP_LEN, snap);
    if (cc_flags || mask || peer) {
        (void)MeasPack_UnpackCcBlock(&d[1 + MEAS_PACK_SNAP_LEN], MEAS_PACK_CC_LEN,
                                     cc_flags, mask, peer);
    }
    return 1;
}

/* 解析 0x000F 的**精简**应答（23B）：
 *   状态(1) + 对侧结论(1) + 三相×(周期ns 4 + 占空比‰ 2 + 旗标 1)
 * 对侧结论：0=没有对侧数据 1=一致 2=不一致（原来这里是 45B 的一整块数，现在只留结论）。
 * 高电平/边沿数已经不发了（都能由周期与占空比算出来，或"周期非 0"本身就说明有信号）。 */
static int parse_pulse_resp(uint8_t *status, pulse_snapshot_t *mine, uint8_t *peer)
{
    uint16_t cmd_o = 0, seq_o = 0, dlen = 0;
    uint8_t  d[FRAME_MAX_DATA];
    uint8_t  i;

    if ((tx_len == 0u) || (!Frame_Parse(tx_buf, tx_len, &cmd_o, &seq_o, d, &dlen))) {
        return 0;
    }
    if (cmd_o != MGR_CMD_PULSE || dlen != PULSE_PACK_RESP_LEN) {
        return 0;
    }
    if (status) *status = d[0];
    if (peer) *peer = d[1];
    if (mine != 0) {
        memset(mine, 0, sizeof(*mine));
        mine->phase_count = (uint8_t)PULSE_PH_COUNT;
        for (i = 0u; i < (uint8_t)PULSE_PH_COUNT; i++) {
            const uint8_t *p = &d[2u + ((uint16_t)i * PULSE_RESP_ITEM_LEN)];

            mine->ph[i].period_ns = ((uint32_t)p[0] << 24) | ((uint32_t)p[1] << 16) |
                                    ((uint32_t)p[2] << 8)  |  (uint32_t)p[3];
            mine->ph[i].duty_permille = (uint16_t)(((uint16_t)p[4] << 8) | p[5]);
            mine->ph[i].flags = p[6];
        }
    }
    return 1;
}

static void t14_measure_cmd(void)
{
    uint16_t seq = 0xA000;
    uint8_t  mode;
    uint8_t  status = 0xFF;
    meas_snapshot_t snap, peer;
    uint8_t  cc = 0u;
    uint16_t mask = 0xFFFFu;

    printf("[T14] 0x000E 测量: mode=0 只读 / mode=1 测试 / 非法 mode\n");
    init_env();

    /* ── 14a: 非法 mode → ERR_PARAM ── */
    mode = 0x02u;
    pc_send(MGR_CMD_MEASURE, seq++, &mode, 1);
    {
        uint8_t e[1] = {APP_STATUS_ERR_PARAM};
        expect_ack(MGR_CMD_MEASURE, seq - 1, e, 1, "mode=0x02 → ERR_PARAM 0x0B");
    }

    /* ── 14b: 请求没有 DATA (LEN=0) → 也按参数错误处理 ── */
    pc_send(MGR_CMD_MEASURE, seq++, 0, 0);
    {
        uint8_t e[1] = {APP_STATUS_ERR_PARAM};
        expect_ack(MGR_CMD_MEASURE, seq - 1, e, 1, "LEN=0 → ERR_PARAM");
    }

    /* ── 14c: mode=0 只读: 立刻回 77B, 不进入测试流程 ── */
    mode = MGR_MEAS_MODE_READ;
    pump_meas((int)MEAS_STABLE_N + 2);          /* 先让数据稳定 */
    pc_send(MGR_CMD_MEASURE, seq++, &mode, 1);
    CHECK(parse_meas_resp(&status, &snap, &cc, &mask, &peer) == 1,
          "mode=0 → 77B 应答可解析");
    CHECK(status == APP_STATUS_OK, "mode=0 命令状态 OK");
    CHECK(snap.ch_count == (uint8_t)MEAS_CH_COUNT, "快照 ch_count = 9");
    CHECK((snap.flags & MEAS_SNAP_F_VALID) != 0u, "快照标记有效");
    CHECK(snap.cfg_crc == MEAS_GetCfgCrc(), "快照带上了配置指纹");
    CHECK((cc & MEAS_CC_F_DONE) == 0u, "只读模式不该声称做过交叉校验");
    CHECK(app_state == APP_STATE_IDLE, "只读模式不改变状态机");

    /* ── 14d: mode=1 触发测试: 跑完才回 ACK, U2 不在 → NO_PEER 但不判失败 ── */
    mode = MGR_MEAS_MODE_TEST;
    {
        uint8_t  raw[8 + 1];
        uint16_t n = Frame_Build(raw, MGR_CMD_MEASURE, seq++, &mode, 1);
        uint16_t rseq = 0;
        uint8_t  rst = 0xFF;

        tx_len = 0;
        APP_USB_Receive(raw, n);
        CHECK(wait_meas_ack(MGR_CMD_MEASURE, &rseq, &rst) == 1,
              "mode=1 → 测试跑完后给出应答");
        CHECK(rseq == (uint16_t)(seq - 1), "应答 SEQ = 受理序号");
        CHECK(app_state == APP_STATE_IDLE, "测试结束后回到 IDLE");
    }
    CHECK(parse_meas_resp(&status, &snap, &cc, &mask, &peer) == 1, "测试应答可解析");
    CHECK(status == APP_STATUS_OK, "9 路全量程内 + U2 缺席(不强制) → OK");
    CHECK((snap.flags & MEAS_SNAP_F_ALL_OK) != 0u, "快照标记 9 路全部合格");
    CHECK((cc & MEAS_CC_F_NO_PEER) != 0u, "标志里说明 U2 没有应答");
    CHECK((cc & MEAS_CC_F_DONE) == 0u, "U2 不在时不算做过交叉校验");
    CHECK(mask == 0u, "没有对侧数据时掩码为 0");

    /* ── 14d-2: 0x000E 只管 ADC，应答里**不应该**再带脉冲段 ── */
    CHECK(PULSE_PACK_RESP_LEN != MEAS_PACK_RESP_LEN, "两条命令的应答长度确实不同");
    CHECK(MEAS_PACK_RESP_LEN == 77u, "0x000E 应答固定 77B(状态+快照+校验块)");

    /* ── 14e: 测试期间其它命令回 BUSY（不变量 I3） ── */
    {
        uint8_t  raw[8 + 1];
        uint16_t n = Frame_Build(raw, MGR_CMD_MEASURE, seq++, &mode, 1);
        tx_len = 0;
        APP_USB_Receive(raw, n);
        APP_Run();                            /* 进入 TESTING */
        CHECK(app_state == APP_STATE_TESTING, "mode=1 之后进入 TESTING 状态");

        {
            uint8_t  raw2[16];
            uint16_t n2 = Frame_Build(raw2, MGR_CMD_QUERY_FLASH_VER, seq++, 0, 0);
            uint8_t  e[1] = {APP_STATUS_BUSY};
            tx_len = 0;
            APP_USB_Receive(raw2, n2);
            APP_Run();
            expect_ack(MGR_CMD_QUERY_FLASH_VER, seq - 1, e, 1, "测试中查版本 → BUSY");
        }
        /* 让这次测试自己走完, 免得影响后续用例 */
        { uint16_t rseq = 0; uint8_t rst = 0xFF;
          CHECK(wait_meas_ack(MGR_CMD_MEASURE, &rseq, &rst) == 1, "测试仍能正常收尾"); }
        CHECK(app_state == APP_STATE_IDLE, "收尾后回到 IDLE, 后续命令不再 BUSY");
        pc_send(MGR_CMD_QUERY_FLASH_VER, seq++, 0, 0);
        CHECK(tx_len > 0u, "测试结束后命令恢复正常处理");
    }

    /* ── 14f: 手动把某一路做成"贴边" → 快照里应带 sat 标记 ── */
    mode = MGR_MODE_READ;
    MEAS_TestInject(MEAS_CH_V5, 0u);          /* 码值 0 = 开路/短路可疑 */
    pump_meas(2);
    pc_send(MGR_CMD_MEASURE, seq++, &mode, 1);
    CHECK(parse_meas_resp(&status, &snap, 0, 0, 0) == 1, "贴边用例应答可解析");
    CHECK((snap.ch[MEAS_CH_V5].flags & MEAS_CHF_SAT) != 0u, "码值贴边 → sat 标志置位");
    CHECK(snap.ch[MEAS_CH_V5].raw == 0u, "快照里的 raw 码值同步为 0");
}

/*==============================================================================
 * T15: 0x000F 脉冲捕获（与 0x000E 完全独立的一条命令）
 *----------------------------------------------------------------------------
 * 宿主替身 PULSE_CfgStart() 会在窗口开始时灌一段 20kHz/50% 的假 PWM，
 * 相当于接了个理想信号发生器；U2 缺席 → 对侧那段按"无数据"打包。
 *============================================================================*/
static void t15_pulse_cmd(void)
{
    uint16_t seq = 0xB000;
    uint8_t  mode;
    uint8_t  status = 0xFF;
    pulse_snapshot_t pm;
    uint8_t          peer_b = 0xFFu;
    int      i;

    printf("[T15] 0x000F 脉冲捕获: mode=0 只读 / mode=1 测试 / 非法 mode\n");
    init_env();

    /* ── 15a: 非法 mode → ERR_PARAM ── */
    mode = 0x07u;
    pc_send(MGR_CMD_PULSE, seq++, &mode, 1);
    {
        uint8_t e[1] = {APP_STATUS_ERR_PARAM};
        expect_ack(MGR_CMD_PULSE, seq - 1, e, 1, "mode=0x07 → ERR_PARAM 0x0B");
    }

    /* ── 15b: LEN=0 → 也按参数错误 ── */
    pc_send(MGR_CMD_PULSE, seq++, 0, 0);
    {
        uint8_t e[1] = {APP_STATUS_ERR_PARAM};
        expect_ack(MGR_CMD_PULSE, seq - 1, e, 1, "LEN=0 → ERR_PARAM");
    }

    /* ── 15c: mode=1 触发一次捕获（200ms 窗口，跑完才回 ACK） ── */
    mode = MGR_MODE_TEST;
    {
        uint8_t  raw[8 + 1];
        uint16_t n = Frame_Build(raw, MGR_CMD_PULSE, seq++, &mode, 1);
        uint16_t rseq = 0;
        uint8_t  rst = 0xFF;

        tx_len = 0;
        APP_USB_Receive(raw, n);
        APP_Run();                            /* 解析这一帧 → 进入 TESTING */
        CHECK(app_state == APP_STATE_TESTING, "mode=1 之后进入 TESTING 状态");
        CHECK(wait_meas_ack(MGR_CMD_PULSE, &rseq, &rst) == 1, "捕获窗口跑完后给出应答");
        CHECK(rseq == (uint16_t)(seq - 1), "应答 SEQ = 受理序号");
        CHECK(app_state == APP_STATE_IDLE, "结束后回到 IDLE");
    }
    memset(&pm, 0, sizeof(pm));
    CHECK(parse_pulse_resp(&status, &pm, &peer_b) == 1, "0x000F 应答(23B)可解析");
    CHECK(PULSE_PACK_RESP_LEN == 23u, "应答长度 = 状态1 + 对侧1 + 三相×7");
    CHECK(status == APP_STATUS_OK, "默认不判脉冲 → 结论 OK");
    for (i = 0; i < (int)PULSE_PH_COUNT; i++) {
        char nm[64];
        sprintf(nm, "第 %d 相: 20kHz → 周期 50us", i);
        CHECK(pm.ph[i].period_ns == 50000u, nm);
        CHECK(pm.ph[i].duty_permille == 500u, "占空比 50.0%");
        CHECK((pm.ph[i].flags & PULSE_F_VALID) != 0u, "该相标记有效");
    }
    CHECK(peer_b == 0u, "U2 缺席 → 对侧结论 = 0（没有对侧数据）");

    /* ── 15d: mode=0 只读：立刻回上一批结果，不重新采 ── */
    mode = MGR_MODE_READ;
    {
        uint32_t t0 = g_tick;
        pc_send(MGR_CMD_PULSE, seq++, &mode, 1);
        CHECK(parse_pulse_resp(&status, &pm, &peer_b) == 1, "只读模式应答可解析");
        CHECK(pm.ph[PULSE_PH_A].period_ns == 50000u, "读到的是上一批结果");
        CHECK((g_tick - t0) < 50u, "只读模式没有等 200ms 窗口");
    }

    /* ── 15e: 没有信号时（替身不灌边沿）→ NOSIGNAL，而不是垃圾数据 ── */
    g_pulse_edges_on = 0u;
    mode = MGR_MODE_TEST;
    {
        uint8_t  raw[8 + 1];
        uint16_t n = Frame_Build(raw, MGR_CMD_PULSE, seq++, &mode, 1);
        uint16_t rseq = 0;
        uint8_t  rst = 0xFF;

        tx_len = 0;
        APP_USB_Receive(raw, n);
        APP_Run();
        CHECK(wait_meas_ack(MGR_CMD_PULSE, &rseq, &rst) == 1, "没信号时也能正常收尾");
    }
    memset(&pm, 0, sizeof(pm));
    CHECK(parse_pulse_resp(&status, &pm, 0) == 1, "无信号应答可解析");
    CHECK((pm.ph[PULSE_PH_B].flags & PULSE_F_NOSIGNAL) != 0u, "该相标记没信号");
    CHECK(pm.ph[PULSE_PH_B].period_ns == 0u, "没信号时周期为 0（不硬算）");
    g_pulse_edges_on = 1u;

    /* ── 15f: 0x000E 与 0x000F 互不干扰（各自独立读） ── */
    {
        uint8_t m2 = MGR_MODE_READ;
        pc_send(MGR_CMD_MEASURE, seq++, &m2, 1);
        CHECK(parse_meas_resp(&status, 0, 0, 0, 0) == 1, "0x000E 仍然可用");
        pc_send(MGR_CMD_PULSE, seq++, &m2, 1);
        CHECK(parse_pulse_resp(&status, 0, 0) == 1, "0x000F 也仍然可用");
    }
}

/*==============================================================================
 * T16: ARM 同步触发主路径（链路上真的有一个 U2 时走的那条）
 *----------------------------------------------------------------------------
 * T14/T15 覆盖的是"U2 缺席"的降级路径；这里覆盖生产时真正走的那条：
 *   U1 收到 0x000F mode=1 → 发 ARM 让两端同窗 → 各自采 → PULSE_XCHK 取回对侧结果。
 *============================================================================*/
/* T16 只在"端口0 = 假 U2"的构建里跑；DUT 模式端口0 给 DUT 链路用（jig_mode.h §3.4） */
#if (JIG_DUT_LINK == 0)
static void t16_arm_sync_path(void)
{
    uint16_t seq = 0xC000;
    uint8_t  mode = MGR_MODE_TEST;
    uint8_t  status = 0xFF;
    pulse_snapshot_t pm;
    uint8_t          peer_b = 0xFFu;
    uint8_t  i;

    printf("[T16] ARM 同步触发主路径（有 U2）\n");
    init_env();
    U2Fake_Init();

    /* 1) 先跑保活：U1 必须从 STATUS 里拿到对端能力位图（含 ARM）才会用 ARM。
     *    PING 每 500ms 一发、每 5 发插一次 STATUS，所以这里等到拿到为止（有上限）。 */
    {
        int tries;
        for (tries = 0; (tries < 40) && (u2fake_caps_has_arm() == 0); tries++) {
            host_advance(600u);
            pump_meas(200);
        }
    }
    CHECK(APP_U2Alive() != 0u, "链路保活通了");
    CHECK(APP_U2VerOk() != 0u, "两片链路代码版本一致");
    CHECK(u2fake_caps_has_arm() != 0, "对端能力位图里有 ARM");

    /* 2) 0x000F mode=1：先 ARM(两端同窗)，再取对侧结果 */
    {
        uint8_t  raw[8 + 1];
        uint16_t n = Frame_Build(raw, MGR_CMD_PULSE, seq++, &mode, 1);
        uint16_t rseq = 0;
        uint8_t  rst = 0xFF;

        tx_len = 0;
        APP_USB_Receive(raw, n);
        CHECK(wait_meas_ack(MGR_CMD_PULSE, &rseq, &rst) == 1, "跑完给出应答");
        CHECK(rst == APP_STATUS_OK, "默认不判脉冲 → 结论 OK");
        CHECK(rseq == (uint16_t)(seq - 1), "应答 SEQ = 受理序号");
        CHECK(app_state == APP_STATE_IDLE, "结束后回到 IDLE");
    }

    memset(&pm, 0, sizeof(pm));
    CHECK(parse_pulse_resp(&status, &pm, &peer_b) == 1, "23B 应答可解析");
    CHECK(status == APP_STATUS_OK, "命令状态 OK");
    for (i = 0u; i < (uint8_t)PULSE_PH_COUNT; i++) {
        CHECK(pm.ph[i].period_ns == 50000u, "本片: 20kHz → 50us 周期");
    }
    /* 对侧不再发整块数，只发结论：假 U2 报 40kHz，和本片 20kHz 不一致 → 2 */
    CHECK(peer_b == 2u, "对侧结论 = 2（U2 报 40kHz，与本片 20kHz 不一致）");

    /* 3) 关掉假 U2 再跑一次：应当退回"对侧缺席"(NO_PEER)，证明两条路都在 */
    g_u2_on = 0u;
    {
        uint8_t  raw[8 + 1];
        uint16_t n = Frame_Build(raw, MGR_CMD_PULSE, seq++, &mode, 1);
        uint16_t rseq = 0;
        uint8_t  rst = 0xFF;

        tx_len = 0;
        APP_USB_Receive(raw, n);
        CHECK(wait_meas_ack(MGR_CMD_PULSE, &rseq, &rst) == 1, "U2 又不在时仍能收尾");
    }
    CHECK(parse_pulse_resp(&status, &pm, &peer_b) == 1, "应答仍可解析");
    CHECK(peer_b == 0u, "对侧缺席时结论 = 0（没有对侧数据）");
}

#endif /* JIG_DUT_LINK == 0 —— T16 结束 */

#if (JIG_DUT_LINK == 0)
/*==============================================================================
 * T19: 0x0013 ⑤ 回环 —— 向 U2 要编码器线的边沿计数
 *----------------------------------------------------------------------------
 * ⑤ 的治具侧闭环是两步：`0x0012` 发脉冲 + `0x0013` 问 U2 数到几个。
 * 这里验的是第二步：参数校验、透传语义（清零/读取）、应答布局、
 * U2 不在时的超时、以及超时之后状态机不卡死。
 * **需要假 U2**，所以只在"端口0 = 假 U2"的构建里跑（同 T16）。
 *============================================================================*/
/* 给 T19 用的发送辅助：**必须同时推假 U2**。
 * `pc_send()` 只推 APP_Run()，而假 U2 是靠 `U2Fake_Poll()` 驱动的
 * （T16 用的是 pump_meas，那里推了，所以没暴露这个问题）。
 * DUT 链路的 T17 不需要这个——它的假 DUT 是在 UART 替身里同步应答的。 */
static uint16_t enc_peer_send(uint16_t cmd, uint16_t seq, const uint8_t *data, uint16_t len)
{
    uint8_t  raw[8 + FRAME_MAX_DATA + 2];
    uint16_t n = Frame_Build(raw, cmd, seq, data, len);

    tx_len = 0;
    APP_USB_Receive(raw, n);
    for (int i = 0; i < 60; i++) {
        U2Fake_Poll();
        APP_Run();
    }
    return tx_len;
}

static void t19_enc_peer(void)
{
    uint8_t  req[1];
    uint8_t  d[FRAME_MAX_DATA];
    uint16_t n, ac, as_, dlen;
    uint16_t seq = 0x0500u;

    printf("\n-- T19 0x0013 编码器回环（向 U2 要计数）--\n");
    /* ⚠ 宿主替身的局限：`HAL_UART_Transmit_IT` 在 `g_u2_on == 0` 时**故意不回调 TxCplt**
     *（用来模拟"线没接/对端不在"），而驱动的 `tx_busy` 只能靠 TxCplt 清 ——
     * 于是一旦在"对端不在"的状态下发过一次，那个端口就被永久钉住了。
     * 这是替身的问题，不是固件行为（真 HAL 总会给回调或报错）。
     * 本用例明确要"线是通的"，所以先把两个端口整体复位一次，拿一个干净起点。 */
    UartCfg_Init();
    U2Fake_Init();
    s_u2enc_a = s_u2enc_b = s_u2enc_z = 0u;
    s_u2enc_lost = 0u;
    s_u2enc_clears = 0u;

    /* 1) 参数非法 → 0B，且不该往 U2 发 */
    n = enc_peer_send(MGR_CMD_ENC_PEER, seq++, NULL, 0u);
    CHECK(Frame_Parse(tx_buf, n, &ac, &as_, d, &dlen) != 0u && dlen == 1u && d[0] == 0x0Bu,
          "LEN=0 → 0B ERR_PARAM");
    req[0] = 9u;
    n = enc_peer_send(MGR_CMD_ENC_PEER, seq++, req, 1u);
    CHECK(Frame_Parse(tx_buf, n, &ac, &as_, d, &dlen) != 0u && dlen == 1u && d[0] == 0x0Bu,
          "动作 9（非法）→ 0B ERR_PARAM");
    CHECK(s_u2enc_clears == 0u, "参数错时没有触发 U2 侧的任何动作");

    /* 2) 清零：应答 = jig 状态(1) + U2 的状态(1) */
    s_u2enc_a = 123u; s_u2enc_b = 456u;
    req[0] = LINK_ENC_ACT_CLEAR;
    n = enc_peer_send(MGR_CMD_ENC_PEER, seq++, req, 1u);
    CHECK(Frame_Parse(tx_buf, n, &ac, &as_, d, &dlen) != 0u && dlen == 2u,
          "清零应答 = 1+1 字节");
    CHECK(ac == MGR_CMD_ENC_PEER && as_ == (uint16_t)(seq - 1u), "命令与序号回显正确");
    CHECK(d[0] == 0x00u && d[1] == LINK_ST_OK, "jig 状态 00 + U2 状态 00");
    CHECK(s_u2enc_clears == 1u, "U2 那边确实收到并执行了清零");
    CHECK(s_u2enc_a == 0u && s_u2enc_b == 0u && s_u2enc_z == 0u, "U2 的计数被清零");

    /* 3) 读取：应答 = jig 状态(1) + U2 的 15B */
    s_u2enc_a = 2000u; s_u2enc_b = 4000u; s_u2enc_z = 2u;
    req[0] = LINK_ENC_ACT_READ;
    n = enc_peer_send(MGR_CMD_ENC_PEER, seq++, req, 1u);
    CHECK(Frame_Parse(tx_buf, n, &ac, &as_, d, &dlen) != 0u
          && dlen == (uint16_t)(1u + LINK_ENC_RESP_LEN), "读取应答 = 1+15 字节");
    CHECK(d[0] == 0x00u, "jig 状态 00");
    {
        link_enc_cnt_t c; uint8_t st = 0xFFu;
        CHECK(Link_EncUnpackResp(&d[1], (uint16_t)(dlen - 1u), &st, &c) == 1u,
              "用 Link_EncUnpackResp 能解出来（PC 侧可直接用这套编解码）");
        CHECK(st == LINK_ST_OK, "U2 状态 00");
        CHECK(c.a == 2000u && c.b == 4000u && c.z == 2u, "A/B/Z 计数原样带回");
        CHECK(EncPlan_Edges(1000u) == c.a,
              "A：发 1000 个脉冲 = 2000 个边沿（和 enc_plan 的换算对得上）");
        CHECK(EncPlan_Edges(2000u) == c.b, "B：发 2000 个脉冲 = 4000 个边沿");
        CHECK(EncPlan_Edges(1u) == c.z, "Z：发 1 个脉冲 = 2 个边沿");
    }

    /* 4) U2 不在 → 0C 超时（不是静默成功） */
    g_u2_on = 0u;
    req[0] = LINK_ENC_ACT_READ;
    {
        uint8_t  raw[8 + 1];
        uint16_t rn = Frame_Build(raw, MGR_CMD_ENC_PEER, seq++, req, 1u);
        tx_len = 0;
        APP_USB_Receive(raw, rn);
        for (int i = 0; (i < 250) && (tx_len == 0); i++) { host_advance(5u); APP_Run(); }
    }
    CHECK(tx_len > 0u && Frame_Parse(tx_buf, tx_len, &ac, &as_, d, &dlen) != 0u
          && dlen == 1u && d[0] == 0x0Cu, "U2 不在 → 0C ERR_TIMEOUT");

    /* 5) 超时之后状态机必须回到 IDLE。
     * 上面那一步把"对端不在"的状态留在了替身的发送侧（同 T19 开头的说明），
     * 所以这里同样先复位端口，再发下一条。 */
    UartCfg_Init();
    U2Fake_Init();
    s_u2enc_a = 7u;
    req[0] = LINK_ENC_ACT_READ;
    n = enc_peer_send(MGR_CMD_ENC_PEER, seq++, req, 1u);
    CHECK(Frame_Parse(tx_buf, n, &ac, &as_, d, &dlen) != 0u
          && dlen == (uint16_t)(1u + LINK_ENC_RESP_LEN),
          "超时后还能继续用（没卡在 APP_STATE_ENC_PEER）");
    CHECK(app_state == APP_STATE_IDLE, "状态机确实回到了 IDLE");

    g_u2_on = 0u;
}
#endif /* JIG_DUT_LINK == 0 —— T19 */

/*==============================================================================
 * T18: 0x0012 编码器脉冲发生（⑤，纯治具侧）
 *----------------------------------------------------------------------------
 * 这条命令的价值是**不接 DUT、不做编排就能上板验证脉冲发生**，
 * 所以这里的判据是：请求解析对不对、参数校验挡不挡得住、ACK 布局对不对、
 * 状态机能不能收尾（不卡在 APP_STATE_ENC）。
 * "真发出去多少个脉冲"由 enc_gen.c 负责，宿主用替身；上板用示波器/DUT 计数验证。
 *============================================================================*/
static void t18_enc_gen(void)
{
    uint8_t  req[8 + MGR_ENC_GEN_REQ_LEN];
    uint8_t  d[FRAME_MAX_DATA];
    uint16_t n, ac, as_, dlen;
    uint16_t seq = 0x0400u;

    printf("\n-- T18 0x0012 编码器脉冲发生 --\n");
    EncGen_Init();

    /* 1) LEN=0 → 用默认参数（A=1000@5kHz / B=2000@10kHz / Z=1@5kHz） */
    n = pc_send(MGR_CMD_ENC_GEN, seq++, NULL, 0u);
    CHECK(n > 0u && Frame_Parse(tx_buf, n, &ac, &as_, d, &dlen) != 0u, "空请求有可解析的应答");
    CHECK(ac == MGR_CMD_ENC_GEN && as_ == (uint16_t)(seq - 1u), "命令与序号回显正确");
    CHECK(dlen == MGR_ENC_GEN_RESP_LEN, "应答 9 字节");
    CHECK(d[0] == 0x00u, "状态 00 OK");
    CHECK(g_enc_start_calls == 1u, "EncGen_Start 被调用了一次");
    CHECK(g_enc_rec_cnt[0] == 1000u && g_enc_rec_cnt[1] == 2000u && g_enc_rec_cnt[2] == 1u,
          "默认个数 A=1000/B=2000/Z=1 传到了发生层");
    CHECK(g_enc_rec_freq[0] == 5000u && g_enc_rec_freq[1] == 10000u && g_enc_rec_freq[2] == 5000u,
          "默认频率 A=5k/B=10k/Z=5k 传到了发生层");
    CHECK(d[1] == 0x07u, "完成掩码 = 三路都发完");
    CHECK(d[2] == 0x00u, "没有超时强停");
    CHECK(d[3] == 0x07u && d[4] == 0xD0u, "A 实际翻转 2000 次（0x07D0）");
    CHECK(d[5] == 0x0Fu && d[6] == 0xA0u, "B 实际翻转 4000 次（0x0FA0）");
    CHECK(d[7] == 0x00u && d[8] == 0x02u, "Z 实际翻转 2 次");

    /* 2) 显式参数：逐字节大端解析 */
    {
        uint16_t vals[6] = { 100u, 200u, 1u, 2000u, 4000u, 1000u };
        int i;
        for (i = 0; i < 6; i++) {
            req[i * 2]     = (uint8_t)(vals[i] >> 8);
            req[i * 2 + 1] = (uint8_t)(vals[i] & 0xFFu);
        }
        n = pc_send(MGR_CMD_ENC_GEN, seq++, req, MGR_ENC_GEN_REQ_LEN);
        CHECK(n > 0u && Frame_Parse(tx_buf, n, &ac, &as_, d, &dlen) != 0u && d[0] == 0x00u,
              "显式参数被受理");
        CHECK(g_enc_rec_cnt[0] == 100u && g_enc_rec_cnt[1] == 200u && g_enc_rec_cnt[2] == 1u,
              "个数解析正确（大端）");
        CHECK(g_enc_rec_freq[0] == 2000u && g_enc_rec_freq[1] == 4000u && g_enc_rec_freq[2] == 1000u,
              "频率解析正确（大端）");
        CHECK(d[3] == 0x00u && d[4] == 0xC8u, "A 翻转 200 次（0x00C8）");
    }

    /* 3) 长度不对 → 0B，且**不能**往发生层发 */
    {
        uint32_t before = g_enc_start_calls;
        n = pc_send(MGR_CMD_ENC_GEN, seq++, req, 5u);
        CHECK(Frame_Parse(tx_buf, n, &ac, &as_, d, &dlen) != 0u && dlen == 1u && d[0] == 0x0Bu,
              "长度 5 → 0B ERR_PARAM");
        CHECK(g_enc_start_calls == before, "参数错时没有调 EncGen_Start");
    }

    /* 4) 频率超范围 → 发生层拒绝 → 0B */
    {
        uint16_t bad[6] = { 100u, 100u, 1u, 50u, 4000u, 1000u };   /* freq_a = 50Hz 低于下限 */
        int i;
        for (i = 0; i < 6; i++) {
            req[i * 2]     = (uint8_t)(bad[i] >> 8);
            req[i * 2 + 1] = (uint8_t)(bad[i] & 0xFFu);
        }
        n = pc_send(MGR_CMD_ENC_GEN, seq++, req, MGR_ENC_GEN_REQ_LEN);
        CHECK(Frame_Parse(tx_buf, n, &ac, &as_, d, &dlen) != 0u && dlen == 1u && d[0] == 0x0Bu,
              "频率 50Hz（低于下限）→ 发生层拒收 → 0B");
    }

    /* 5) 发生层启动失败 → BUSY（不是静默成功） */
    {
        g_enc_start_fail = 1u;
        n = pc_send(MGR_CMD_ENC_GEN, seq++, NULL, 0u);
        CHECK(Frame_Parse(tx_buf, n, &ac, &as_, d, &dlen) != 0u && dlen == 1u && d[0] == 0x01u,
              "EncGen_Start 失败 → 01 BUSY");
        g_enc_start_fail = 0u;
    }

    /* 6) 发完之后状态机必须回到 IDLE（不卡在 APP_STATE_ENC） */
    n = pc_send(MGR_CMD_ENC_GEN, seq++, NULL, 0u);
    CHECK(Frame_Parse(tx_buf, n, &ac, &as_, d, &dlen) != 0u && d[0] == 0x00u,
          "下一次还能正常发（状态机回到 IDLE）");
    CHECK(as_ == (uint16_t)(seq - 1u), "序号仍然回显正确");

    /* 7) 发完之后别的命令要能正常跑（不是把状态留着） */
    {
        uint8_t mode = MGR_MODE_READ;
        uint8_t raw2[8 + 1];
        uint16_t n2 = Frame_Build(raw2, MGR_CMD_MEASURE, seq++, &mode, 1u);
        tx_len = 0;
        APP_USB_Receive(raw2, n2);
        CHECK(wait_meas_ack(MGR_CMD_MEASURE, 0, 0) == 1, "0x0012 之后 0x000E 仍然可用");
    }
}

#if (JIG_DUT_LINK != 0)
/*==============================================================================
 * T17: 0x0011 DUT 透传（docs/15）
 *----------------------------------------------------------------------------
 * 这条命令是上板 bring-up 的关键：DUT 链路一旦通了，③④⑤⑥ 才有基础。
 * 所以这里的判据是"**帧真的到了那头、回帧真的回到了 U1、PC 侧能直接解析**"，
 * 而不是"代码看起来对"。用字节级假 DUT（见前面 DutFake_*）跑真链路。
 *============================================================================*/
static void t17_dut_passthrough(void)
{
    uint8_t  req[8 + LINK_MAX_PAYLOAD];
    uint16_t seq = 0x0200u;
    uint16_t n, ac, as_, adlen;
    uint8_t  ad[FRAME_MAX_DATA];

    printf("\n-- T17 0x0011 DUT 透传 --\n");
    s_dut_seen = 0u;
    s_dut_bad_crc = 0u;
    s_dut_quiet = 0u;

    /* 1) 请求太短（连 2 字节命令码都没有）→ 0B，且**一个字节都不能往 DUT 发** */
    n = pc_send(MGR_CMD_DUT_RAW, seq++, NULL, 0u);
    CHECK(n > 0u && Frame_Parse(tx_buf, n, &ac, &as_, ad, &adlen) != 0u, "空请求有可解析的应答");
    CHECK(ac == MGR_CMD_DUT_RAW && as_ == (uint16_t)(seq - 1u), "应答回显命令与序号");
    CHECK(adlen == 1u && ad[0] == 0x0Bu, "空请求 → 0B ERR_PARAM");
    CHECK(s_dut_seen == 0u, "参数错时一个字节都没往 DUT 发");

    /* 2) 命令码不在 DUT 空间 → 0B，同样不往 DUT 发（防止误透传） */
    req[0] = 0x01; req[1] = 0x12;                 /* 0x0112 是治具自己的命令码 */
    n = pc_send(MGR_CMD_DUT_RAW, seq++, req, 2u);
    CHECK(Frame_Parse(tx_buf, n, &ac, &as_, ad, &adlen) != 0u && adlen == 1u && ad[0] == 0x0Bu,
          "非 DUT 命令空间 → 0B ERR_PARAM");
    CHECK(s_dut_seen == 0u, "越界命令没有被透传出去");

    /* 3) PING 透传：DUT 原样回显，PC 收到 jig 状态 + DUT 的 DATA */
    req[0] = 0x02; req[1] = 0x00;                 /* DUT_PING */
    req[2] = 0xDE; req[3] = 0xAD; req[4] = 0xBE; req[5] = 0xEF;
    n = pc_send(MGR_CMD_DUT_RAW, seq++, req, 6u);
    CHECK(n > 0u && Frame_Parse(tx_buf, n, &ac, &as_, ad, &adlen) != 0u, "PING 透传有应答");
    CHECK(as_ == (uint16_t)(seq - 1u), "PING 透传应答回显序号");
    CHECK(s_dut_seen == 1u, "DUT 那头确实收到了 1 帧");
    CHECK(s_dut_last_cmd == DUT_CMD_PING && s_dut_last_len == 4u,
          "DUT 收到的是 PING + 4 字节 DATA");
    CHECK(s_dut_last_data[0] == 0xDE && s_dut_last_data[3] == 0xEF, "DATA 原样到达 DUT");
    CHECK(adlen == 1u + 4u, "应答 = jig 状态(1) + DUT 的 4B 回显");
    CHECK(ad[0] == 0x00u, "jig 状态 = 00 OK");
    CHECK(ad[1] == 0xDE && ad[4] == 0xEF, "回显原样带回 PC");

    /* 4) INFO 透传：16B 应答原样带回，并能被 dut_proto 直接解 */
    req[0] = 0x02; req[1] = 0x01;
    n = pc_send(MGR_CMD_DUT_RAW, seq++, req, 2u);
    CHECK(Frame_Parse(tx_buf, n, &ac, &as_, ad, &adlen) != 0u && adlen == 1u + 16u,
          "INFO 应答 16B 原样带回（没被链路层截断）");
    {
        dut_info_t fi;
        CHECK(DutUnpack_Info(&ad[1], (uint16_t)(adlen - 1u), &fi) != 0u
              && fi.dut_ver == DUT_CODE_VER, "DutUnpack_Info 能解透传回来的应答");
        CHECK(DutCaps_Has(fi.caps, DUT_CMD_GET_ADC) != 0u, "caps 里的 GET_ADC 能解出来");
    }

    /* 5) GET_ADC 透传：2 字节掩码 + 11 字节应答 */
    req[0] = 0x02; req[1] = 0x07; req[2] = 0x00; req[3] = 0x03;
    n = pc_send(MGR_CMD_DUT_RAW, seq++, req, 4u);
    CHECK(s_dut_last_cmd == DUT_CMD_GET_ADC && s_dut_last_len == 2u, "DUT 收到 2B 掩码");
    CHECK(s_dut_last_data[0] == 0x00 && s_dut_last_data[1] == 0x03, "掩码值原样到达 DUT");
    CHECK(Frame_Parse(tx_buf, n, &ac, &as_, ad, &adlen) != 0u && adlen == 1u + 11u,
          "GET_ADC 应答 11B 带回");
    {
        dut_adc_t fa;
        CHECK(DutUnpack_Adc(&ad[1], (uint16_t)(adlen - 1u), 0x0003u, &fa) != 0u,
              "DutUnpack_Adc 能解透传回来的应答");
        CHECK(fa.item[0].raw == 0x081Au && fa.item[1].mv == 312u, "两路值都对");
    }

    /* 6) DUT 不回 → 0C ERR_TIMEOUT（不是静默丢弃），且状态机必须回到 IDLE */
    s_dut_quiet = 1u;
    req[0] = 0x02; req[1] = 0x03;                 /* DUT_STOP，故意不应答 */
    {
        uint8_t  raw[8 + 2];
        uint16_t rn = Frame_Build(raw, MGR_CMD_DUT_RAW, seq++, req, 2u);
        tx_len = 0;
        APP_USB_Receive(raw, rn);
        /* 超时 200ms + 一次重传 → 虚拟时间推到 1s 足够 */
        for (int i = 0; (i < 250) && (tx_len == 0); i++) {
            host_advance(5u);
            APP_Run();
        }
    }
    CHECK(tx_len > 0u && Frame_Parse(tx_buf, tx_len, &ac, &as_, ad, &adlen) != 0u,
          "DUT 不回时有应答（不是静默丢弃）");
    CHECK(adlen == 1u && ad[0] == 0x0Cu, "DUT 不回 → 0C ERR_TIMEOUT");

    /* 7) 超时之后必须还能继续用（没卡在 APP_STATE_DUT） */
    s_dut_quiet = 0u;
    req[0] = 0x02; req[1] = 0x03;
    n = pc_send(MGR_CMD_DUT_RAW, seq++, req, 2u);
    CHECK(Frame_Parse(tx_buf, n, &ac, &as_, ad, &adlen) != 0u && ad[0] == 0x00u,
          "超时之后还能继续透传（没卡在 APP_STATE_DUT）");
    CHECK(as_ == (uint16_t)(seq - 1u), "超时恢复后序号回显仍然正确");

    /* 8) 透传期间其它命令必须回 BUSY（不变量 I3 仍然成立） */
    {
        uint8_t  raw[8];
        uint8_t  dmode = 0u;
        uint16_t rn = Frame_Build(raw, MGR_CMD_DUT_RAW, seq++, req, 2u);
        uint16_t qn;
        uint8_t  qraw[8 + 1];

        /* 让 DUT 暂时不回，把 U1 卡在 APP_STATE_DUT */
        s_dut_quiet = 1u;
        tx_len = 0;
        APP_USB_Receive(raw, rn);
        APP_Run();                              /* 受理，进入 APP_STATE_DUT */

        qn = Frame_Build(qraw, MGR_CMD_MEASURE, 0x0300u, &dmode, 1u);
        tx_len = 0;
        APP_USB_Receive(qraw, qn);
        APP_Run();
        CHECK(tx_len > 0u && Frame_Parse(tx_buf, tx_len, &ac, &as_, ad, &adlen) != 0u
              && adlen == 1u && ad[0] == 0x01u,
              "透传进行中收到别的命令 → 01 BUSY");

        /* 收尾：等它超时回 IDLE，别把状态留给下一个用例 */
        tx_len = 0;
        for (int i = 0; (i < 250) && (tx_len == 0); i++) {
            host_advance(5u);
            APP_Run();
        }
        s_dut_quiet = 0u;
        CHECK(tx_len > 0u, "收尾：透传超时后状态回到 IDLE");
    }
}

/*==============================================================================
 * T20: 0x0014 ⑥ 相电流检测 —— 一条命令跑完整流程
 *----------------------------------------------------------------------------
 * 用假 DUT 跑真链路：假 DUT 在收到 `DUT_CURRENT` 之后会把电流两路的读数
 * 从 1650mV 改成 2000mV（模拟中点偏置的放大器），于是"通流前/通流中"能分辨。
 *
 * 这一层验的是**编排**：读基线 → 通流 → 等 → 再读 → 判定 → 22B 报告；
 * 以及参数非法 / DUT 缺席 / 状态机收尾。
 * **判定本身的数值正确性**由 `test/cur_chk_test.c`（43 项纯函数用例）负责 ——
 * 宿主环境里治具自己的 ADC 没有收敛的真实数据，这一层不去编造它。
 *============================================================================*/
static uint16_t cur_send(uint16_t seq, const uint8_t *data, uint16_t len)
{
    /* 8 头 + 最大 9 字节 DATA（8 = 老布局，9 = 多一个 mode）+ 2 CRC */
    uint8_t  raw[8 + MGR_CUR_TEST_REQ_LEN + 1 + 2];
    uint16_t n = Frame_Build(raw, MGR_CMD_CUR_TEST, seq, data, len);

    tx_len = 0;
    APP_USB_Receive(raw, n);
    /* 流程里有 duration_ms(200) + settle(20) 的等待，所以要推虚拟时钟 */
    for (int i = 0; (i < 1000) && (tx_len == 0); i++) {
        host_advance(1u);
        APP_Run();
    }
    return tx_len;
}

static void t20_cur_test(void)
{
    uint8_t  d[FRAME_MAX_DATA];
    uint16_t n, ac, as_, dlen;
    uint16_t seq = 0x0600u;

    printf("\n-- T20 0x0014 相电流检测（完整流程）--\n");
    s_dutfake_cur_on   = 0u;
    s_dutfake_cur_seen = 0u;
    s_dut_quiet        = 0u;

    /* 1) 参数非法（上下桥同相 = 同桥臂直通）→ 0B，且不该往 DUT 发 */
    {
        uint8_t bad[MGR_CUR_TEST_REQ_LEN] = { DUT_PHASE_A, DUT_PHASE_A, 0x03, 0xE8,
                                              0x00, 0xC8, 0x03, 0x00 };
        n = cur_send(seq++, bad, MGR_CUR_TEST_REQ_LEN);
        CHECK(Frame_Parse(tx_buf, n, &ac, &as_, d, &dlen) != 0u && dlen == 1u && d[0] == 0x0Bu,
              "上下桥同相（母线短路）→ 0B ERR_PARAM");
        CHECK(s_dutfake_cur_seen == 0u, "参数错时没有往 DUT 发通流命令");
    }
    /* 长度不对 → 0B */
    {
        uint8_t one = DUT_PHASE_A;
        n = cur_send(seq++, &one, 1u);
        CHECK(Frame_Parse(tx_buf, n, &ac, &as_, d, &dlen) != 0u && dlen == 1u && d[0] == 0x0Bu,
              "长度 1 → 0B ERR_PARAM");
    }

    /* 2) 完整流程（LEN=0 → 默认：上桥 A、下桥 B、100%、200ms、两路） */
    n = cur_send(seq++, NULL, 0u);
    CHECK(n > 0u && Frame_Parse(tx_buf, n, &ac, &as_, d, &dlen) != 0u, "有可解析的应答");
    CHECK(ac == MGR_CMD_CUR_TEST && as_ == (uint16_t)(seq - 1u), "命令与序号回显正确");
    CHECK(dlen == MGR_CUR_TEST_RESP_LEN, "报告 12 字节");
    CHECK(d[0] == APP_STATUS_OK, "命令状态 OK");
    CHECK(d[1] == DUT_ST_OK, "DUT_CURRENT 的状态带回来了");
    CHECK(s_dutfake_cur_seen == 1u, "DUT 确实被要求通流了一次");

    /* 3) 关键顺序：DUT 的基线必须是**通流前**读的、on 必须是**通流后**读的。
     *    新布局每路 5B，从 out[2] 开始 → ch1 在 [2..6]、ch2 在 [7..11]。
     *      ch1: jig_delta@2..3, dut_delta@4..5, flags@6
     *      ch2: jig_delta@7..8, dut_delta@9..10, flags@11
     *    变化量 = 通流中 − 通流前：假 DUT 是 1650 → 2000，所以 dut_delta 应为 +350。
     *    （治具侧在这个宿主环境里没有有效读数，所以 jig_delta 应该是 0。） */
    {
        int16_t dj1 = (int16_t)(((uint16_t)d[2] << 8) | d[3]);
        int16_t dd1 = (int16_t)(((uint16_t)d[4] << 8) | d[5]);
        int16_t dd2 = (int16_t)(((uint16_t)d[9] << 8) | d[10]);

        CHECK(dd1 == 350, "电流1 的 DUT 变化量 = 2000−1650 = +350mV（顺序对）");
        CHECK(dd2 == 350, "电流2 的 DUT 变化量 = 2005−1655 = +350mV");
        CHECK(dj1 == 0, "治具侧没有有效读数 → 变化量 0");
        CHECK(dd1 > 0, "变化量为正：通流中确实比通流前大");
    }

    /* 4) 治具侧没有有效读数 → 判 NODATA，**但不因此判失败** */
    CHECK((d[6] & CUR_CHK_F_NODATA) != 0u || (d[6] & CUR_CHK_F_OK) != 0u,
          "ch1 的旗标是 NODATA 或 OK（不会是别的）");
    CHECK(d[0] == APP_STATUS_OK, "NODATA 不算失败（状态码仍是 OK）");

    /* 5) DUT 缺席 → 0C 超时（不是静默成功），且状态机回 IDLE */
    s_dut_quiet = 1u;
    n = cur_send(seq++, NULL, 0u);
    CHECK(Frame_Parse(tx_buf, n, &ac, &as_, d, &dlen) != 0u && dlen == 1u && d[0] == 0x0Cu,
          "DUT 不回 → 0C ERR_TIMEOUT");
    s_dut_quiet = 0u;
    CHECK(app_state == APP_STATE_IDLE, "超时后状态机回到 IDLE");

    /* 6) 之后还能继续用 */
    n = cur_send(seq++, NULL, 0u);
    CHECK(Frame_Parse(tx_buf, n, &ac, &as_, d, &dlen) != 0u
          && dlen == MGR_CUR_TEST_RESP_LEN && d[0] == APP_STATUS_OK,
          "超时之后还能继续跑（没卡在 APP_STATE_CUR）");

    /* 7) 显式参数：上桥 B、下桥 C */
    {
        uint8_t req[MGR_CUR_TEST_REQ_LEN] = { DUT_PHASE_B, DUT_PHASE_C, 0x01, 0xF4,
                                              0x00, 0x64, 0x03, 0x00 };
        n = cur_send(seq++, req, MGR_CUR_TEST_REQ_LEN);
        CHECK(Frame_Parse(tx_buf, n, &ac, &as_, d, &dlen) != 0u && d[0] == APP_STATUS_OK,
              "上桥 B、下桥 C、50%、100ms → 正常跑完");
        CHECK(s_dutfake_cur_seen >= 2u, "又通了一次流");

        /* ⚠ 这一段是 0x0014 最容易错的地方：**发给 DUT 的 8 字节布局与 PC 的请求
         *   布局不同**（PC: hi/lo/duty/dur/ch/rsvd；DUT: mode/hi/lo/duty/dur/rsvd）。
         *   之前治具把 PC 的请求原样转发 → 长度对、内容全错 → 真 DUT 回 0B。
         *   假 DUT 不做校验的话这个 bug 在回归里看不出来，所以这里逐字段钉住。 */
        CHECK(s_dutfake_cur_len == DUT_CURRENT_REQ_LEN, "0x0206 的 DATA 正好 8 字节");
        CHECK(s_dutfake_cur_mode == (uint8_t)DUT_CUR_MODE_STATIC,
              "第 0 字节是 mode（LEN=8 时按静态通流 0）");
        CHECK(s_dutfake_cur_hi == DUT_PHASE_B, "第 1 字节 = 上桥相 B");
        CHECK(s_dutfake_cur_lo == DUT_PHASE_C, "第 2 字节 = 下桥相 C");
        CHECK(s_dutfake_cur_duty == 0x01F4u, "第 3..4 字节 = 占空比 500‰");
        CHECK(s_dutfake_cur_dur == 0x0064u, "第 5..6 字节 = 时长 100ms");
        CHECK(s_dutfake_cur_rsvd == 0u, "第 7 字节 rsvd = 0");
    }

    /* 7b) 可选第 9 字节 = 通流模式（1 = 低频 PWM）。
     *     台架上被测板接着三相电机时，静态全通（mode=0）等于 24V 直压两个绕组，
     *     所以必须能用 PWM 模式 + 小占空比去试 —— 这一条钉住"mode 真的传下去了"。 */
    {
        uint8_t req[MGR_CUR_TEST_REQ_LEN + 1u] = { DUT_PHASE_A, DUT_PHASE_B,
                                                   0x00, 0x14,     /* 20‰ = 2% */
                                                   0x00, 0x64,     /* 100ms   */
                                                   0x03, 0x00,     /* 两路都读 */
                                                   DUT_CUR_MODE_PWM };
        n = cur_send(seq++, req, (uint16_t)(MGR_CUR_TEST_REQ_LEN + 1u));
        CHECK(Frame_Parse(tx_buf, n, &ac, &as_, d, &dlen) != 0u && d[0] == APP_STATUS_OK,
              "LEN=9 带 mode=1 能被受理");
        CHECK(s_dutfake_cur_mode == (uint8_t)DUT_CUR_MODE_PWM,
              "DUT 收到的第 0 字节 = mode 1（不是静止的 0）");
        CHECK(s_dutfake_cur_duty == 0x0014u, "PWM 模式下的占空比 20‰ 原样传下去");
        CHECK(s_dutfake_cur_dur == 0x0064u, "时长 100ms 原样传下去");
    }

    /* 8) 跑完之后别的命令仍然可用 */
    {
        uint8_t mode = MGR_MODE_READ;
        uint8_t raw2[8 + 1];
        uint16_t n2 = Frame_Build(raw2, MGR_CMD_MEASURE, seq++, &mode, 1u);
        tx_len = 0;
        APP_USB_Receive(raw2, n2);
        CHECK(wait_meas_ack(MGR_CMD_MEASURE, 0, 0) == 1, "0x0014 之后 0x000E 仍然可用");
    }
}
/*==============================================================================
 * T21: 0x0015 ② 电压检测 —— 治具 9 路 vs 被测板自己报的
 *----------------------------------------------------------------------------
 * 假 DUT 的 INFO 报 adc_mask = 0x0003（只支持 ch0/ch1），GET_ADC 按掩码答。
 * 验的是**编排**：问 INFO 拿能力位图 → 按位图问 GET_ADC → 9 路判定 → 71B 报告。
 * 数值判定的正确性由 `test/vol_chk_test.c`（51 项纯函数用例）负责 ——
 * 宿主环境里治具自己的 ADC 没有收敛数据，这一层不编造它。
 *============================================================================*/
static uint16_t vol_send(uint16_t seq)
{
    uint8_t  raw[8 + 2];
    uint16_t n = Frame_Build(raw, MGR_CMD_VOL_TEST, seq, 0, 0u);

    tx_len = 0;
    APP_USB_Receive(raw, n);
    for (int i = 0; (i < 400) && (tx_len == 0); i++) {
        host_advance(1u);
        APP_Run();
    }
    return tx_len;
}

static void t21_vol_test(void)
{
    uint8_t  d[FRAME_MAX_DATA];
    uint16_t n, ac, as_, dlen;
    uint16_t seq = 0x0700u;

    printf("\n-- T21 0x0015 电压检测（9 路 vs DUT）--\n");
    s_dut_quiet = 0u;

    /* 1) 这条命令不需要参数 → 带了参数就是 0B */
    {
        uint8_t one = 0u;
        uint8_t raw[8 + 1];
        uint16_t n2 = Frame_Build(raw, MGR_CMD_VOL_TEST, seq++, &one, 1u);
        tx_len = 0;
        APP_USB_Receive(raw, n2);
        APP_Run();
        CHECK(tx_len > 0u && Frame_Parse(tx_buf, tx_len, &ac, &as_, d, &dlen) != 0u
              && dlen == 1u && d[0] == 0x0Bu, "带了参数 → 0B ERR_PARAM");
    }

    /* 2) 完整流程 */
    n = vol_send(seq++);
    CHECK(n > 0u && Frame_Parse(tx_buf, n, &ac, &as_, d, &dlen) != 0u, "有可解析的应答");
    CHECK(ac == MGR_CMD_VOL_TEST && as_ == (uint16_t)(seq - 1u), "命令与序号回显正确");
    CHECK(dlen == MGR_VOL_TEST_RESP_LEN, "报告 47 字节");
    CHECK(d[0] == APP_STATUS_OK, "命令状态 OK");

    /* 3) 头里只剩"被测板状态"——版本号/通道声明不再发（INFO 里问得到）*/
    CHECK(d[1] == DUT_ST_OK, "INFO 的状态带回来了");

    /* 4) 九路全是「不判」：ch0/ch1 有 DUT 数据但治具侧在这个宿主环境里没有有效读数，
     *    其余 7 路 DUT 没声明。**不判不算失败**，所以状态码仍是 OK。 */
    {
        uint8_t i, n_nodata = 0u, n_bad = 0u;
        for (i = 0u; i < (uint8_t)MEAS_CH_COUNT; i++) {
            uint8_t f = d[2u + ((uint16_t)i * MGR_VOL_CH_ITEM_LEN) + 4u];
            if ((f & VOL_CHK_F_NODATA) != 0u) n_nodata++;
            if ((f & VOL_CHK_F_DEVIATION) != 0u) n_bad++;
        }
        CHECK(n_nodata == (uint8_t)MEAS_CH_COUNT, "九路都是「不判」（数据缺）");
        CHECK(n_bad == 0u, "没有一路被判超差");
        CHECK(d[0] == APP_STATUS_OK, "「不判」不算失败（状态码仍是 OK）");
    }

    /* 5) 逐通道的下标位置对得上（ch4 = A 相，在 2+4×5 = 22） */
    {
        uint8_t *p4 = &d[2u + (4u * MGR_VOL_CH_ITEM_LEN)];
        CHECK((p4[4] & VOL_CHK_F_NODATA) != 0u, "ch4（A 相）是 NODATA —— 下标没串");
        CHECK(p4[0] == 0u && p4[1] == 0u && p4[2] == 0u && p4[3] == 0u,
              "没判的路 治具/被测板 mV 都是 0（没拿 0 当读数去比）");
    }

    /* 6) DUT 缺席 → 0C 超时，状态机回 IDLE */
    s_dut_quiet = 1u;
    n = vol_send(seq++);
    CHECK(Frame_Parse(tx_buf, n, &ac, &as_, d, &dlen) != 0u && dlen == 1u && d[0] == 0x0Cu,
          "DUT 不回 → 0C ERR_TIMEOUT");
    s_dut_quiet = 0u;
    CHECK(app_state == APP_STATE_IDLE, "超时后状态机回到 IDLE");

    /* 7) 之后还能继续用 */
    n = vol_send(seq++);
    CHECK(Frame_Parse(tx_buf, n, &ac, &as_, d, &dlen) != 0u
          && dlen == MGR_VOL_TEST_RESP_LEN, "超时之后还能继续跑（没卡在 APP_STATE_VOL）");

    /* 8) 跑完之后别的命令仍然可用 */
    {
        uint8_t mode = MGR_MODE_READ;
        uint8_t raw2[8 + 1];
        uint16_t n2 = Frame_Build(raw2, MGR_CMD_MEASURE, seq++, &mode, 1u);
        tx_len = 0;
        APP_USB_Receive(raw2, n2);
        CHECK(wait_meas_ack(MGR_CMD_MEASURE, 0, 0) == 1, "0x0015 之后 0x000E 仍然可用");
    }
}
#endif /* JIG_DUT_LINK */

/*==============================================================================
 * T22: 0x0016 ④ 三相输出判据 —— 对**已有的**测量/脉冲结果判定
 *----------------------------------------------------------------------------
 * 这条命令没有状态机：它把"上一个测量快照（相电压 + 母线）"和
 * "上一个捕获窗口的脉冲结果（占空比）"喂给 phase_chk 判一遍。
 * 所以 T22 的关键是**先造出结果**：脉冲结果用 T15 那套假波形（20kHz/50%），
 * 电压用 `MEAS_TestInject` 灌进去（宿主环境没有真 ADC）。
 *============================================================================*/
static uint8_t phj_send(uint16_t seq, uint8_t mode, uint16_t da, uint16_t db, uint16_t dc)
{
    uint8_t  req[MGR_PHASE_JUDGE_REQ_LEN];
    uint8_t  raw[8 + MGR_PHASE_JUDGE_REQ_LEN + 2];
    uint16_t n;

    req[0] = mode;
    req[1] = (uint8_t)(da >> 8); req[2] = (uint8_t)da;
    req[3] = (uint8_t)(db >> 8); req[4] = (uint8_t)db;
    req[5] = (uint8_t)(dc >> 8); req[6] = (uint8_t)dc;
    n = Frame_Build(raw, MGR_CMD_PHASE_JUDGE, seq, req, MGR_PHASE_JUDGE_REQ_LEN);
    tx_len = 0;
    APP_USB_Receive(raw, n);
    pump(8);
    return tx_len;
}

static void t22_phase_judge(void)
{
    uint8_t  d[FRAME_MAX_DATA];
    uint16_t n, ac, as_, dlen;
    uint16_t seq = 0x0800u;

    printf("\n-- T22 0x0016 三相输出判据 --\n");

    /* 1) 参数非法 */
    {
        uint8_t raw[8 + 2];
        uint16_t n2;

        n2 = Frame_Build(raw, MGR_CMD_PHASE_JUDGE, seq++, 0, 0u);
        tx_len = 0;
        APP_USB_Receive(raw, n2);
        pump(8);
        CHECK(tx_len > 0u && Frame_Parse(tx_buf, tx_len, &ac, &as_, d, &dlen) != 0u
              && dlen == 1u && d[0] == 0x0Bu, "LEN=0 → 0B ERR_PARAM");

        n2 = phj_send(seq++, 7u, 500u, 500u, 500u);      /* mode=7 非法 */
        CHECK(Frame_Parse(tx_buf, n2, &ac, &as_, d, &dlen) != 0u && dlen == 1u && d[0] == 0x0Bu,
              "mode=7（非法）→ 0B ERR_PARAM");
    }

    /* 2) 先造出结果：脉冲（假波形 20kHz/50%）+ 电压（注入）*/
    {
        uint8_t raw[8 + 1];
        uint8_t mode = MGR_MODE_TEST;
        uint16_t n2 = Frame_Build(raw, MGR_CMD_PULSE, seq++, &mode, 1u);

        tx_len = 0;
        APP_USB_Receive(raw, n2);
        CHECK(wait_meas_ack(MGR_CMD_PULSE, 0, 0) == 1, "先跑一次 0x000F mode=1 拿到脉冲结果");
    }
    /* 灌电压：母线 24V（码 3773）、三相各 12V（码 1861）→ 期望 500‰ × 24V = 12V */
    MEAS_TestInject((meas_id_t)MEAS_CH_V24,  3773u);
    MEAS_TestInject((meas_id_t)MEAS_CH_PH_A, 1861u);
    MEAS_TestInject((meas_id_t)MEAS_CH_PH_B, 1861u);
    MEAS_TestInject((meas_id_t)MEAS_CH_PH_C, 1861u);
    for (int i = 0; i < 600; i++) {
        MEAS_Poll();
    }

    /* 3) Arduino… 不，正常判定：PWM 模式、三相都 500‰ */
    n = phj_send(seq++, MGR_PHASE_MODE_PWM, 500u, 500u, 500u);
    CHECK(n > 0u && Frame_Parse(tx_buf, n, &ac, &as_, d, &dlen) != 0u, "有可解析的应答");
    CHECK(ac == MGR_CMD_PHASE_JUDGE && as_ == (uint16_t)(seq - 1u), "命令与序号回显正确");
    CHECK(dlen == MGR_PHASE_JUDGE_RESP_LEN, "报告 16 字节");
    CHECK(d[0] == APP_STATUS_OK || d[0] == APP_STATUS_ERR_TEST,
          "状态码只能是 OK 或 ERR_TEST");

    /* 强路径：注入的电压（母线 24V、三相各 12V）配上 500‰ 的占空比，
     * 期望值 = 500‰ × 24V = 12V，实测也是 12V → 三相都该判 OK。
     * 这条同时证明了 `MEAS_TestInject` 的收敛和 phase_chk 的整条数据通路。 */
    /* 新布局每相 5B：从 out[1] 开始 → A 在 [1..5]、B 在 [6..10]、C 在 [11..15]
     *   meas_mv(2) expect_mv(2) flags(1) */
    {
        uint16_t fa = d[5];
        uint16_t fb = d[10];
        uint16_t fc = d[15];
        CHECK((fa & PHASE_CHK_F_NODATA) == 0u,
              "A 相数据可用（注入收敛了，不是 NODATA）");
        CHECK((fa & PHASE_CHK_F_OK) != 0u, "A 相：12V 实测 vs 12V 期望 → OK");
        CHECK((fb & PHASE_CHK_F_OK) != 0u, "B 相 → OK");
        CHECK((fc & PHASE_CHK_F_OK) != 0u, "C 相 → OK");
        CHECK(d[0] == APP_STATUS_OK, "三相全合格 → 命令状态 OK");
    }
    /* 期望值确实按 500‰ × 母线折出来了（12V = 12000mV），实测值也应≈12V */
    {
        uint16_t ex = (uint16_t)((d[3] << 8) | d[4]);
        uint16_t mv = (uint16_t)((d[1] << 8) | d[2]);
        CHECK(ex == 12000u, "A 相的期望值 = 12000mV（500‰ × 24000mV）");
        /* 实测值由"注入的 ADC 码 + 分压换算"决定，不是精确等于期望值；
         * 该断言要钉的是**它在容差内**（判据用的就是这条），不是钉死一个数。
         * 实测 10851mV vs 期望 12000mV，差 1149 ≤ 容差 1195~1200 → 判 OK。 */
        CHECK((mv + 1200u >= ex) && (mv <= ex + 1200u),
              "A 相实测值落在容差内（实测与期望差 ≤ 1200mV）");
    }

    /* 4) 高阻：该相不判（**不算失败**）*/
    n = phj_send(seq++, MGR_PHASE_MODE_STATIC, MGR_PHASE_DUTY_HIZ, 0u, 1000u);
    CHECK(Frame_Parse(tx_buf, n, &ac, &as_, d, &dlen) != 0u
          && dlen == MGR_PHASE_JUDGE_RESP_LEN, "带高阻相的请求也能正常回");
    {
        uint16_t fa = d[5];         /* A 相 flags */
        uint16_t fc = d[15];        /* C 相 flags */
        CHECK((fa & PHASE_CHK_F_HIZ) != 0u, "A 相高阻 → 置 HIZ 旗标");
        CHECK((fa & PHASE_CHK_F_OK) != 0u, "高阻算「不判」，旗标里带 OK（不是失败）");
        CHECK((fc & PHASE_CHK_F_HIZ) == 0u, "C 相没填高阻 → 不置 HIZ");
    }

    /* 5) 静态模式不查脉冲：命令 1000‰ 但捕获到的是 50% 也不该报 DUTY_DIFF */
    n = phj_send(seq++, MGR_PHASE_MODE_STATIC, 1000u, 1000u, 1000u);
    CHECK(Frame_Parse(tx_buf, n, &ac, &as_, d, &dlen) != 0u, "静态模式能跑");
    {
        uint16_t fa = (uint16_t)((d[4] << 8) | d[5]);
        CHECK((fa & PHASE_CHK_F_DUTY_DIFF) == 0u,
              "静态电平模式**不查脉冲** → 不会误报占空比不符");
    }

    /* 6) 跑完之后别的命令仍然可用 */
    {
        uint8_t mode = MGR_MODE_READ;
        uint8_t raw2[8 + 1];
        uint16_t n2 = Frame_Build(raw2, MGR_CMD_MEASURE, seq++, &mode, 1u);
        tx_len = 0;
        APP_USB_Receive(raw2, n2);
        CHECK(wait_meas_ack(MGR_CMD_MEASURE, 0, 0) == 1, "0x0016 之后 0x000E 仍然可用");
    }
}

/*----------------------------------------------------------------------------
 * T23 0x0017 治具自检 / 在线状态
 *----------------------------------------------------------------------------*/
static void encj_set16(uint8_t *p, uint16_t v)
{
    p[0] = (uint8_t)(v >> 8);
    p[1] = (uint8_t)v;
}
static void encj_set32(uint8_t *p, uint32_t v)
{
    p[0] = (uint8_t)(v >> 24);
    p[1] = (uint8_t)(v >> 16);
    p[2] = (uint8_t)(v >> 8);
    p[3] = (uint8_t)v;
}

static void t23_jig_status(void)
{
    uint8_t  d[FRAME_MAX_DATA];
    uint16_t n, ac, as_, dlen;
    uint16_t seq = 0x0A00u;

    printf("\n-- T23 0x0017 治具自检 / 在线状态 --\n");

    /* 1) 参数非法：这条命令不吃参数，带 LEN 就是错 */
    n = pc_send(MGR_CMD_JIG_STATUS, seq++, 0, 0u);
    CHECK(Frame_Parse(tx_buf, n, &ac, &as_, d, &dlen) != 0u
          && dlen == MGR_JIG_STATUS_RESP_LEN, "LEN=0 → 正常回 32 字节");
    {
        uint8_t one = 0u;
        n = pc_send(MGR_CMD_JIG_STATUS, seq++, &one, 1u);
        CHECK(Frame_Parse(tx_buf, n, &ac, &as_, d, &dlen) != 0u
              && dlen == 1u && d[0] == 0x0Bu, "LEN=1 → 0B ERR_PARAM（不收参数）");
    }

    /* 2) 正常读：状态字段如实反映状态机 */
    n = pc_send(MGR_CMD_JIG_STATUS, seq++, 0, 0u);
    CHECK(Frame_Parse(tx_buf, n, &ac, &as_, d, &dlen) != 0u && dlen == MGR_JIG_STATUS_RESP_LEN,
          "应答 32 字节");
    if (dlen == MGR_JIG_STATUS_RESP_LEN) {
        CHECK(ac == MGR_CMD_JIG_STATUS, "命令号回显");
        CHECK(d[0] == APP_STATUS_OK, "状态码 OK");
        CHECK(d[31] == 0u, "保留字节为 0");
#if (JIG_DUT_LINK != 0)
        CHECK((d[2] & MGR_JIG_ST_DUT_LINK) != 0u, "DUT 构建：flags 报 DUT 链路已启用");
#else
        CHECK((d[2] & MGR_JIG_ST_DUT_LINK) == 0u, "非 DUT 构建：flags 不报 DUT 链路");
#endif
    }

    /* 3) **核心特性：非 IDLE 状态也能问**（普通命令这时只能拿到 BUSY）。
     *    直接把状态机伪造成"一键流程编排中"（这个状态没有自己的推进器会抢答，
     *    比构造一个真异步流程更确定）。 */
    {
        APP_State_t saved = app_state;
        uint8_t mode_read = MGR_MODE_READ;

        app_state = APP_STATE_RUN;
        n = pc_send(MGR_CMD_JIG_STATUS, seq++, 0, 0u);
        CHECK(Frame_Parse(tx_buf, n, &ac, &as_, d, &dlen) != 0u
              && dlen == MGR_JIG_STATUS_RESP_LEN && d[0] == APP_STATUS_OK,
              "非 IDLE 时仍然正常应答（不走 BUSY 闸门）");
        CHECK(dlen == MGR_JIG_STATUS_RESP_LEN && d[1] == (uint8_t)APP_STATE_RUN,
              "状态字段如实回报当前状态机");
        /* 链路忙不忙取决于前面测试留下的保活探测（本宿主里 g_u2_on=0 会把它卡住），
         * 所以这里不钉死 LINK_BUSY 的值，只钉"flags 里没有未定义的位"。 */
        CHECK(dlen == MGR_JIG_STATUS_RESP_LEN
              && ((d[2] & (uint8_t)~(MGR_JIG_ST_U2_ONLINE | MGR_JIG_ST_U2_VER_SAME |
                                     MGR_JIG_ST_LINK_BUSY | MGR_JIG_ST_DUT_LINK)) == 0u),
              "flags 只允许那 4 个已定义的位");

        /* 对照组：同一时刻普通命令必须回 BUSY */
        n = pc_send(MGR_CMD_MEASURE, seq++, &mode_read, 1u);
        CHECK(Frame_Parse(tx_buf, n, &ac, &as_, d, &dlen) != 0u
              && dlen == 1u && d[0] == 0x01u, "同一时刻 0x000E 回 BUSY（对照组）");

        app_state = saved;
    }
}

/*----------------------------------------------------------------------------
 * T24 0x0018 ⑤ 编码器三方对齐判定
 *----------------------------------------------------------------------------*/
static uint16_t encj_send(uint16_t seq,
                          uint16_t pa, uint16_t pb, uint16_t pz,
                          uint32_t ua, uint32_t ub, uint32_t uz, uint16_t lost,
                          uint32_t da, uint32_t db, uint32_t dz, int8_t dir,
                          uint8_t av)
{
    uint8_t d[MGR_ENC_JUDGE_REQ_LEN];

    memset(d, 0, sizeof(d));
    encj_set16(&d[0], pa);
    encj_set16(&d[2], pb);
    encj_set16(&d[4], pz);
    encj_set16(&d[6], 0u);              /* tol = 0 */
    encj_set32(&d[8], ua);
    encj_set32(&d[12], ub);
    encj_set32(&d[16], uz);
    encj_set16(&d[20], lost);
    encj_set32(&d[22], da);
    encj_set32(&d[26], db);
    encj_set32(&d[30], dz);
    d[34] = (uint8_t)dir;
    d[35] = av;
    d[36] = 0u;
    d[37] = 0u;
    return pc_send(MGR_CMD_ENC_JUDGE, seq, d, MGR_ENC_JUDGE_REQ_LEN);
}

static void t24_enc_judge(void)
{
    uint8_t  d[FRAME_MAX_DATA];
    uint16_t n, ac, as_, dlen;
    uint16_t seq = 0x0B00u;
    uint8_t  avb = (uint8_t)(MGR_ENC_JUDGE_AV_U2 | MGR_ENC_JUDGE_AV_DUT);

    printf("\n-- T24 0x0018 ⑤ 编码器三方对齐判定 --\n");

    /* 1) 参数检查 */
    n = pc_send(MGR_CMD_ENC_JUDGE, seq++, 0, 0u);
    CHECK(Frame_Parse(tx_buf, n, &ac, &as_, d, &dlen) != 0u && dlen == 1u && d[0] == 0x0Bu,
          "LEN=0 → 0B ERR_PARAM");
    {
        uint8_t bad[2] = { 0u, 0u };
        n = pc_send(MGR_CMD_ENC_JUDGE, seq++, bad, 2u);
        CHECK(Frame_Parse(tx_buf, n, &ac, &as_, d, &dlen) != 0u && dlen == 1u && d[0] == 0x0Bu,
              "LEN=2 → 0B ERR_PARAM（长度固定 38）");
    }
    {
        /* 可用标志里填了非法位（bit2） */
        uint8_t dd[MGR_ENC_JUDGE_REQ_LEN];
        memset(dd, 0, sizeof(dd));
        encj_set16(&dd[0], 1000u);
        dd[35] = 0x04u;
        n = pc_send(MGR_CMD_ENC_JUDGE, seq++, dd, MGR_ENC_JUDGE_REQ_LEN);
        CHECK(Frame_Parse(tx_buf, n, &ac, &as_, d, &dlen) != 0u && dlen == 1u && d[0] == 0x0Bu,
              "可用标志有非法位 → 0B ERR_PARAM");
    }
    {
        /* 保留字节非 0 */
        uint8_t dd[MGR_ENC_JUDGE_REQ_LEN];
        memset(dd, 0, sizeof(dd));
        dd[36] = 0x01u;
        n = pc_send(MGR_CMD_ENC_JUDGE, seq++, dd, MGR_ENC_JUDGE_REQ_LEN);
        CHECK(Frame_Parse(tx_buf, n, &ac, &as_, d, &dlen) != 0u && dlen == 1u && d[0] == 0x0Bu,
              "保留字节非 0 → 0B ERR_PARAM");
    }

    /* 2) 三边一致 → 合格。exp 由"脉冲数 ×2"算出来，这里顺带把它验了 */
    n = encj_send(seq++, 1000u, 2000u, 1u, 2000u, 4000u, 2u, 0u, 2000u, 4000u, 2u, 1, avb);
    CHECK(Frame_Parse(tx_buf, n, &ac, &as_, d, &dlen) != 0u && dlen == MGR_ENC_JUDGE_RESP_LEN,
          "应答 12 字节");
    CHECK(ac == MGR_CMD_ENC_JUDGE && as_ == (uint16_t)(seq - 1u), "命令号与序号回显");
    if (dlen == MGR_ENC_JUDGE_RESP_LEN) {
        CHECK(d[0] == APP_STATUS_OK, "三边一致 → 状态 OK");
        CHECK(d[2] == 1u, "对侧结论 = 1（有 U2 数据且一致）");
        CHECK(d[1] == 1u, "方向 = +1");
        /* 三相各 3B：边沿数(2) + 旗标(1) → A@3..5, B@6..8, Z@9..11 */
        CHECK(((uint16_t)((d[3] << 8) | d[4])) == 2000u,
              "A 相被测板数到 2000 个边沿");
        CHECK(d[5] == ENC_CHK_F_OK, "A 相 flags = OK");
        CHECK(d[8] == ENC_CHK_F_OK, "B 相 flags = OK");
        CHECK(d[11] == ENC_CHK_F_OK, "Z 相 flags = OK");
    }

    /* 3) DUT 那个数少了 → 不合格，且错误要落在 DUT 那一位上 */
    n = encj_send(seq++, 1000u, 2000u, 1u, 2000u, 4000u, 2u, 0u, 2000u, 2000u, 2u, 1, avb);
    CHECK(Frame_Parse(tx_buf, n, &ac, &as_, d, &dlen) != 0u && dlen == MGR_ENC_JUDGE_RESP_LEN,
          "DUT 不符也能正常应答");
    if (dlen == MGR_ENC_JUDGE_RESP_LEN) {
        CHECK(d[0] == APP_STATUS_ERR_TEST, "状态码 = 05（判定不合格，不是通讯错误）");
        CHECK((d[8] & ENC_CHK_F_DUT_DIFF) != 0u, "B 相：DUT_DIFF 置位");
        CHECK((d[8] & ENC_CHK_F_U2_DIFF) == 0u, "B 相：U2 对得上，不报 U2_DIFF");
        CHECK((d[5] & ENC_CHK_F_OK) != 0u, "A 相仍然 OK（错误只落在 B 相）");
        CHECK(d[2] == 1u, "对侧仍然一致（错在被测板）");
    }

    /* 4) 只有 U2 数据（DUT 还没实现）→ 不判，但**不能**判合格 */
    n = encj_send(seq++, 1000u, 2000u, 1u, 2000u, 4000u, 2u, 0u, 0u, 0u, 0u, 0,
                  MGR_ENC_JUDGE_AV_U2);
    CHECK(Frame_Parse(tx_buf, n, &ac, &as_, d, &dlen) != 0u && dlen == MGR_ENC_JUDGE_RESP_LEN,
          "只有 U2 数据也能回");
    if (dlen == MGR_ENC_JUDGE_RESP_LEN) {
        CHECK((d[5] & ENC_CHK_F_NODATA) != 0u, "A 相：缺数据 → NODATA 旗标");
        CHECK((d[5] & ENC_CHK_F_OK) == 0u, "缺数据时不给 OK 旗标");
        CHECK(d[0] == APP_STATUS_OK, "缺数据 → 状态仍 OK（不冤判）");
        CHECK(d[2] == 1u, "对侧有数据且一致（DUT 缺席不影响对侧结论）");
    }

    /* 5) U2 丢过边沿 → 警告位带上（不影响合格判定） */
    n = encj_send(seq++, 1000u, 2000u, 1u, 2000u, 4000u, 2u, 7u, 2000u, 4000u, 2u, 1, avb);
    CHECK(Frame_Parse(tx_buf, n, &ac, &as_, d, &dlen) != 0u && dlen == MGR_ENC_JUDGE_RESP_LEN,
          "带丢沿计数也能回");
    if (dlen == MGR_ENC_JUDGE_RESP_LEN) {
        /* 丢沿计数（U2 自报）**不再上报**：它是治具内部的自检细节，
         * 判定上本来就"只警告不判失败"，精简后省掉这 2 字节。 */
        CHECK(d[0] == APP_STATUS_OK, "只丢沿不判失败");
        CHECK((d[5] & ENC_CHK_F_OK) != 0u, "丢沿不影响合格结论");
    }
}

/*----------------------------------------------------------------------------
 * T25 0x0010 一键全流程（编排骨架：DUT 不在时也要跑完 12 步并给出报告）
 *----------------------------------------------------------------------------*/
#if (JIG_DUT_LINK != 0)
static void t25_run_all(void)
{
    uint8_t  d[FRAME_MAX_DATA];
    uint16_t n, ac, as_, dlen;
    uint16_t seq = 0x0C00u;
    int      found = 0;
    uint32_t i;

    printf("\n-- T25 0x0010 一键全流程（DUT 不在：验证编排不卡死 + 报告正确）--\n");

    g_u2_on = 0u;                       /* 假 U2 也关掉：整条链路都不在 */

    /* 1) 带参数 → 参数错（本命令目前不吃参数） */
    {
        uint8_t one = 1u;
        n = pc_send(MGR_CMD_RUN_ALL, seq++, &one, 1u);
        CHECK(Frame_Parse(tx_buf, n, &ac, &as_, d, &dlen) != 0u && dlen == 1u && d[0] == 0x0Bu,
              "LEN=1 → 0B ERR_PARAM");
    }

    /* 2) 启动：立即回 BUSY 之外的行为 —— 等最终的汇总帧 */
    tx_len = 0;
    {
        uint8_t raw[12];
        uint16_t rn = Frame_Build(raw, MGR_CMD_RUN_ALL, seq, 0, 0u);

        APP_USB_Receive(raw, rn);
    }
    /* 期间会先收到各子命令的应答，这里只等 0x0010 的汇总 */
    for (i = 0u; i < 400000u; i++) {
        uint16_t flen = tx_len;
        uint16_t c = 0, s = 0, dr = 0;
        uint8_t  dd[FRAME_MAX_DATA];

        if (flen > 0u && Frame_Parse(tx_buf, flen, &c, &s, dd, &dr)) {
            if (c == MGR_CMD_RUN_ALL) {
                memcpy(d, dd, dr);
                dlen = dr;
                ac   = c;
                as_  = s;
                found = 1;
                break;
            }
        }
        tx_len = 0;
        APP_Run();
    }
    CHECK(found == 1, "最终收到了 0x0010 的汇总帧（12 步都跑过了，没有卡死）");
    if (found != 0) {
        CHECK(as_ == seq, "汇总帧的序号 = 请求序号");
        CHECK(dlen == MGR_RUN_ALL_RESP_LEN, "汇总 29 字节");
        CHECK(d[1] == (uint8_t)MGR_RUN_ALL_STEP_COUNT, "steps_total = 12");
        CHECK(d[2] == (uint8_t)MGR_RUN_ALL_STEP_COUNT, "steps_done = 12（一步没落下）");
        /* DUT/U2 全不在：必然有步骤失败，状态码应为 05；failed 指向第一个失败步 */
        CHECK(d[0] == APP_STATUS_ERR_TEST, "链路全不在 → 状态 = 05（不是通讯错，是判定失败）");
        CHECK(d[3] >= 1u && d[3] <= (uint8_t)MGR_RUN_ALL_STEP_COUNT,
              "failed_step 落在 1..12 之间（1 基）");
        CHECK(d[28] == 0u, "末字节保留为 0");
        /* 每步的"状态"必须是已知取值 */
        {
            uint8_t k, bad = 0u;
            for (k = 0u; k < (uint8_t)MGR_RUN_ALL_STEP_COUNT; k++) {
                uint8_t st = d[4u + (k * 2u)];
                if ((st != MGR_RUN_ST_OK) && (st != MGR_RUN_ST_FAIL) &&
                    (st != MGR_RUN_ST_SKIP) && (st != MGR_RUN_ST_TIMEOUT) &&
                    (st != MGR_RUN_ST_IDLE)) {
                    bad = (uint8_t)(bad + 1u);
                }
            }
            CHECK(bad == 0u, "12 步的状态码都在合法取值里");
        }
    }

    /* 3) 跑完之后状态机回到 IDLE，其它命令照常可用 */
    {
        uint8_t mode_read = MGR_MODE_READ;

        n = pc_send(MGR_CMD_JIG_STATUS, seq++, 0, 0u);
        CHECK(Frame_Parse(tx_buf, n, &ac, &as_, d, &dlen) != 0u
              && dlen == MGR_JIG_STATUS_RESP_LEN && d[1] == (uint8_t)APP_STATE_IDLE,
              "一键流程结束后状态机回到 IDLE");
        n = pc_send(MGR_CMD_MEASURE, seq++, &mode_read, 1u);
        CHECK(Frame_Parse(tx_buf, n, &ac, &as_, d, &dlen) != 0u && dlen != 1u,
              "之后 0x000E 不再回 BUSY（状态机干净）");
    }
}
#endif /* JIG_DUT_LINK */

/*----------------------------------------------------------------------------
 * T26 单项"一条命令跑完"（0x0019 ③ / 0x001A ④ / 0x001B ⑤）
 *----------------------------------------------------------------------------*/
#if (JIG_DUT_LINK != 0)
static void t26_one_cmd_flows(void)
{
    static const uint16_t cmds[3] = { MGR_CMD_PULSE_TEST, MGR_CMD_PHASE_TEST,
                                      MGR_CMD_ENC_TEST };
    static const char *names[3] = { "0x0019 ③", "0x001A ④", "0x001B ⑤" };
    uint8_t  d[FRAME_MAX_DATA];
    uint16_t n, ac, as_, dlen;
    uint16_t seq = 0x0D00u;
    int      k;

    printf("\n-- T26 一条命令跑完（0x0019/0x001A/0x001B 打包验证）--\n");
    g_u2_on = 0u;                       /* 假 U2 关掉：让流程走"失败但走完"的路径 */

    /* 1) 这三条都不收参数 */
    for (k = 0; k < 3; k++) {
        uint8_t one = 1u;

        n = pc_send(cmds[k], seq++, &one, 1u);
        CHECK(Frame_Parse(tx_buf, n, &ac, &as_, d, &dlen) != 0u && dlen == 1u && d[0] == 0x0Bu,
              "带参数 → 0B ERR_PARAM（不收参数）");
    }

    /* 2) **核心：一条命令只能有一条应答**
     *    —— 子命令（测试模式/发PWM/采集/判定…）的应答必须被拦下来，
     *    否则上位机就得"认出哪条是哪条"，打包就白做了。 */
    for (k = 0; k < 3; k++) {
        uint8_t  raw[12];
        uint16_t rn = Frame_Build(raw, cmds[k], seq++, 0, 0u);
        uint32_t i;
        uint32_t frames = 0u;
        uint32_t mine   = 0u;

        tx_len = 0;
        APP_USB_Receive(raw, rn);
        for (i = 0u; i < 20000u; i++) {
            uint16_t c = 0, s = 0, dr = 0;
            uint8_t  dd[FRAME_MAX_DATA];

            if (tx_len > 0u) {
                frames++;
                if (Frame_Parse(tx_buf, tx_len, &c, &s, dd, &dr)) {
                    if (c == cmds[k]) {
                        mine++;
                    }
                }
                tx_len = 0;
            }
            APP_Run();
        }
        printf("      %s: 共 %lu 帧应答\n", names[k], (unsigned long)frames);
        CHECK(mine == 1u, "收到了这条命令自己的应答（1 帧）");
        CHECK(frames == 1u, "全程只有 1 条应答 —— 子命令的应答被拦住了");
    }

    /* 3) 流程结束后状态机必须回到 IDLE */
    {
        uint16_t s2 = seq++;
        uint8_t  mode_read = MGR_MODE_READ;

        n = pc_send(MGR_CMD_JIG_STATUS, s2, 0, 0u);
        CHECK(Frame_Parse(tx_buf, n, &ac, &as_, d, &dlen) != 0u
              && dlen == MGR_JIG_STATUS_RESP_LEN && d[1] == (uint8_t)APP_STATE_IDLE,
              "三条流程跑完后都回到 IDLE");
        (void)mode_read;
    }
}
#endif /* JIG_DUT_LINK */

/*==============================================================================
 * T27: 0x001F / 0x0020 —— 给 DUT 发**原始字节**（对面还没实现 0x02xx 时的过渡）
 *----------------------------------------------------------------------------
 * 现场用法：FOC 板现有的是 ASCII 命令 `AS:S800`（转）/`AS:S0`（停）。
 * 用这两条命令先证明"USART 这条线通了"，不必等对面实现整套帧协议。
 *============================================================================*/
#if (JIG_DUT_LINK != 0)
static uint16_t raw_send(uint16_t seq, const uint8_t *data, uint16_t len, int max_ms)
{
    uint8_t  raw[8 + 64 + 2];
    uint16_t n = Frame_Build(raw, MGR_CMD_DUT_SEND, seq, data, len);

    tx_len = 0;
    APP_USB_Receive(raw, n);
    /* 窗口是按毫秒等的，所以要推虚拟时钟；每次 1ms 足够细 */
    for (int i = 0; (i < max_ms) && (tx_len == 0); i++) {
        host_advance(1u);
        APP_Run();
    }
    return tx_len;
}

static void t27_dut_raw(void)
{
    static const uint8_t as_start[] = { 'A', 'S', ':', 'S', '8', '0', '0' };  /* AS:S800 */
    uint8_t  d[FRAME_MAX_DATA];
    uint16_t n, ac, as_, dlen, seq = 0x0900u;

    printf("\n-- T27 0x001F / 0x0020 DUT 原始字节收发 --\n");

    /* 1) 发 "AS:S800"（默认 200ms 窗口），对面不回字 */
    s_dut_raw_len = 0;
    {
        uint8_t req[2 + sizeof(as_start)] = { 0x00, 0xC8 };   /* 200 ms */
        memcpy(&req[2], as_start, sizeof(as_start));
        n = raw_send(seq++, req, (uint16_t)sizeof(req), 400);
    }
    CHECK(n > 0u && Frame_Parse(tx_buf, n, &ac, &as_, d, &dlen) != 0u,
          "0x001F 有应答");
    CHECK(ac == MGR_CMD_DUT_SEND && as_ == (uint16_t)(seq - 1u), "命令与序号回显");
    CHECK(dlen == 5u, "没回字时应答正好 5 字节（状态+发出数+收到数）");
    CHECK(d[0] == APP_STATUS_OK, "状态 00：发出去了就算 OK（对面不回不是错误）");
    CHECK((((uint16_t)d[1] << 8) | d[2]) == (uint16_t)sizeof(as_start),
          "实际发出字节数 = 7");
    CHECK((((uint16_t)d[3] << 8) | d[4]) == 0u, "收到字节数 = 0");
    CHECK(s_dut_raw_len == (uint16_t)sizeof(as_start) &&
          memcmp(s_dut_raw, as_start, sizeof(as_start)) == 0,
          "DUT 口收到的就是 AS:S800 这 7 个字节（原样）");

    /* 2) 对面回字：把 "OK\\r\\n" 塞进 DUT 口，再发一次 AS:S0 */
    s_dut_raw_len = 0;
    {
        static const uint8_t stop[] = { 'A', 'S', ':', 'S', '0' };
        uint8_t req[2 + sizeof(stop)] = { 0x00, 0x00 };       /* 0 = 默认 200ms */
        uint8_t reply[] = { 'O', 'K', 0x0D, 0x0A };
        uint16_t i;

        memcpy(&req[2], stop, sizeof(stop));
        /* 先让 U1 把请求发出去，再把"对端的回字"塞进接收落点 */
        {
            uint8_t  framebuf[8 + 16 + 2];
            uint16_t fn = Frame_Build(framebuf, MGR_CMD_DUT_SEND, seq,
                                      req, (uint16_t)sizeof(req));
            tx_len = 0;
            APP_USB_Receive(framebuf, fn);
            for (i = 0u; i < 4u; i++) {
                u2_deliver(&s_rxp1, &huart1, reply[i]);
            }
            for (int k = 0; (k < 400) && (tx_len == 0); k++) {
                host_advance(1u);
                APP_Run();
            }
        }
        seq++;
        n = tx_len;
        CHECK(n > 0u && Frame_Parse(tx_buf, n, &ac, &as_, d, &dlen) != 0u,
              "第二次 0x001F 有应答");
        CHECK(dlen == 9u, "收到 4 字节 → 应答 5+4 = 9 字节");
        CHECK((((uint16_t)d[3] << 8) | d[4]) == 4u, "收到字节数 = 4");
        CHECK(d[5] == 'O' && d[6] == 'K' && d[7] == 0x0D && d[8] == 0x0A,
              "回的字原样带回来（含 \\r\\n 这种非可打印字节）");
        CHECK(s_dut_raw_len == (uint16_t)sizeof(stop) &&
              memcmp(s_dut_raw, stop, sizeof(stop)) == 0,
              "DUT 口收到的是 AS:S0");
    }

    /* 3) 0x0020 只读：能把空闲期攒下的字节取走，且读后清空 */
    {
        uint8_t rq[8 + 2];
        uint16_t rn;
        uint8_t hello[] = { 'H', 'I' };

        u2_deliver(&s_rxp1, &huart1, hello[0]);
        u2_deliver(&s_rxp1, &huart1, hello[1]);
        pump(3);                                       /* 主循环把字节嗅探进去 */

        tx_len = 0;
        rn = Frame_Build(rq, MGR_CMD_DUT_RECV, seq++, 0, 0u);
        APP_USB_Receive(rq, rn);
        pump(30);
        CHECK(tx_len > 0u && Frame_Parse(tx_buf, tx_len, &ac, &as_, d, &dlen) != 0u,
              "0x0020 有应答");
        CHECK(dlen == 5u && d[0] == APP_STATUS_OK, "0x0020 状态 00，3+2 字节");
        CHECK((((uint16_t)d[1] << 8) | d[2]) == 2u, "0x0020 报收到 2 字节");
        CHECK(d[3] == 'H' && d[4] == 'I', "0x0020 内容是 HI");

        /* 读后清空：再来一次应该是 0 字节 */
        tx_len = 0;
        rn = Frame_Build(rq, MGR_CMD_DUT_RECV, seq++, 0, 0u);
        APP_USB_Receive(rq, rn);
        pump(30);
        CHECK(tx_len > 0u && Frame_Parse(tx_buf, tx_len, &ac, &as_, d, &dlen) != 0u
              && dlen == 3u && d[1] == 0u && d[2] == 0u,
              "0x0020 读后清空：第二次 0 字节");
    }

    /* 4) 参数校验 */
    {
        uint8_t req[2] = { 0x00, 0xC8 };              /* 只有头、没有要发的字节 */
        n = raw_send(seq++, req, 2u, 60);
        CHECK(n > 0u && Frame_Parse(tx_buf, n, &ac, &as_, d, &dlen) != 0u
              && dlen == 1u && d[0] == APP_STATUS_ERR_PARAM,
              "0x001F 没有要发的字节 → 0B");

        tx_len = 0;
        {
            uint8_t rq[8 + 1 + 2];
            uint16_t rn = Frame_Build(rq, MGR_CMD_DUT_RECV, seq++, (const uint8_t *)"x", 1u);
            APP_USB_Receive(rq, rn);
        }
        pump(30);
        CHECK(tx_len > 0u && Frame_Parse(tx_buf, tx_len, &ac, &as_, d, &dlen) != 0u
              && dlen == 1u && d[0] == APP_STATUS_ERR_PARAM,
              "0x0020 带参数 → 0B");
    }
}
#endif /* JIG_DUT_LINK */

/*==============================================================================
 * T28 PC 串口助手"手敲命令行"（ASCII 控制台）
 *----------------------------------------------------------------------------
 * 场景：FOC 固件还没实现 0x02xx，调试时要能直接在串口助手里打 `AS:Bb` 看回什么。
 * 规则：以 A5 5B 开头的走帧协议；可打印 ASCII 走命令行；其它丢弃。
 *============================================================================*/
#if (JIG_DUT_LINK != 0)
static void t28_ascii_console(void)
{
    uint16_t ac, as_, dlen, n;
    uint8_t  d[FRAME_MAX_DATA];
    uint16_t seq = 0x0B00u;

    printf("\n-- T28 PC 串口助手 ASCII 控制台 --\n");

    /* 1) 打 "AS:Bb" + 回车 → 原样转给 DUT；DUT 回版本 → PC 原样收到 */
    {
        static const uint8_t cmd[] = { 'A', 'S', ':', 'B', 'b', '\r' };
        static const uint8_t rep[] = { 'A', 'S', ':', 'V', '1', '.', '2', '.', '3', '\r', '\n' };
        uint16_t i;

        s_dut_raw_len = 0;
        tx_len = 0;
        memcpy(s_ascii_in, cmd, sizeof(cmd));
        APP_USB_Receive(s_ascii_in, (uint32_t)sizeof(cmd));
        CHECK(app_state == APP_STATE_ASCII, "打完回车就进入 ASCII 状态");
        CHECK(s_dut_raw_len == 5u && memcmp(s_dut_raw, "AS:Bb", 5u) == 0,
              "DUT 口收到的就是 AS:Bb（5 字节，回车不发出去）");

        for (i = 0u; i < (uint16_t)sizeof(rep); i++) {
            u2_deliver(&s_rxp1, &huart1, rep[i]);
        }
        host_advance(1u);  pump(1);          /* 收进嗅探缓冲 */
        host_advance(40u); pump(1);          /* 静默 40ms > 30ms → 收尾 */
        CHECK(app_state == APP_STATE_IDLE, "收完回 IDLE");
        CHECK(tx_len == (uint16_t)sizeof(rep) &&
              memcmp(tx_buf, rep, sizeof(rep)) == 0,
              "PC 原样收到 AS:V1.2.3\\r\\n（没有额外加帧）");
    }

    /* 2) 不打回车：静默 60ms 也要发出去 */
    {
        s_dut_raw_len = 0;
        tx_len = 0;
        memcpy(s_ascii_in, "AS:0", 4u);
        APP_USB_Receive(s_ascii_in, 4u);
        CHECK(s_dut_raw_len == 0u, "没有结束符时先不急着发");
        host_advance(60u); pump(1);
        CHECK(s_dut_raw_len == 4u && memcmp(s_dut_raw, "AS:0", 4u) == 0,
              "静默 60ms 后自动发出去（AS:0 停电机）");
        host_advance(2100u); pump(2);        /* 等窗口到点收尾（AS_REPLY_MS=2000） */
        CHECK(app_state == APP_STATE_IDLE, "没回字也会收尾回 IDLE");
        /* 提示语本身以 "\r\n" 开头（串口终端里换行更整齐），所以跳过前 2 字节比 */
        CHECK(tx_len >= 16u && memcmp(&tx_buf[2], "[JIG] no reply", 14u) == 0,
              "没回字时给 PC 一句 [JIG] no reply，别让人干等");
    }

    /* 3) 控制台之后，帧命令照旧能用（A5 5B 优先） */
    {
        tx_len = 0;
        n = Frame_Build((uint8_t *)s_ascii_in, MGR_CMD_JIG_STATUS, seq++, 0, 0u);
        APP_USB_Receive(s_ascii_in, n);
        pump(3);
        CHECK(tx_len > 0u && Frame_Parse(tx_buf, tx_len, &ac, &as_, d, &dlen) != 0u
              && ac == MGR_CMD_JIG_STATUS,
              "紧接着的 A5 5B 帧命令仍然正常（两条路共存）");
    }

    /* 4) 正在跑别的命令时打命令行 → busy，且不碰 DUT 口 */
    {
        static const uint8_t cmd[] = { 'A', 'S', ':', 'C', 'S', '\r' };
        uint8_t mode = MGR_MODE_TEST;
        uint16_t i;

        tx_len = 0;
        n = Frame_Build((uint8_t *)s_ascii_in, MGR_CMD_PULSE, seq++, &mode, 1u);
        APP_USB_Receive(s_ascii_in, n);      /* 开 200ms 采集窗口 → 状态非 IDLE */
        pump(1);
        CHECK(app_state != APP_STATE_IDLE, "采集窗口进行中（非 IDLE）");

        s_dut_raw_len = 0;
        tx_len = 0;
        memcpy(s_ascii_in, cmd, sizeof(cmd));
        APP_USB_Receive(s_ascii_in, (uint32_t)sizeof(cmd));
        CHECK(s_dut_raw_len == 0u, "忙的时候不往 DUT 口发东西");
        CHECK(tx_len >= 12u && memcmp(&tx_buf[2], "[JIG] busy", 10u) == 0,
              "忙的时候回一句 busy 给 PC");
        for (i = 0u; i < 300u; i++) { host_advance(1u); pump(1); }   /* 等采集窗口跑完 */
        host_advance(2100u); pump(2);                                /* 等 ASCII 窗口跑完 */
        CHECK(app_state == APP_STATE_IDLE, "窗口跑完回 IDLE");
    }

    /* 4c) 透明桥：命令行收尾之后，"迟到"的字节也要自动转给 PC。
     *     真实场景：AS:MC → AS:MC0（300ms 内到）；AS:D,... 约 2 秒后才主动发。*/
    {
        static const uint8_t mc[]  = { 'A', 'S', ':', 'M', 'C', '\r' };
        static const uint8_t mc0[] = { 'A', 'S', ':', 'M', 'C', '0' };
        static const uint8_t rep[] = { 'A', 'S', ':', 'D', ',', 'A', '=', '2', '5',
                                       ',', 'B', '=', '8', '3', ',', 'Z', '=', '0' };
        uint16_t i;

        s_dut_raw_len = 0;
        tx_len = 0;
        memcpy(s_ascii_in, mc, sizeof(mc));
        APP_USB_Receive(s_ascii_in, (uint32_t)sizeof(mc));
        CHECK(s_dut_raw_len == 5u && memcmp(s_dut_raw, "AS:MC", 5u) == 0,
              "透明桥：AS:MC 转发给 DUT");

        for (i = 0u; i < (uint16_t)sizeof(mc0); i++) {
            u2_deliver(&s_rxp1, &huart1, mc0[i]);
        }
        host_advance(1u);  pump(1);          /* 收进嗅探 */
        host_advance(40u); pump(1);          /* 静默 40ms → 控制台收尾 */
        CHECK(tx_len == (uint16_t)sizeof(mc0) && memcmp(tx_buf, mc0, sizeof(mc0)) == 0,
              "透明桥：PC 先收到 AS:MC0");
        CHECK(app_state == APP_STATE_IDLE, "透明桥：控制台已收尾");

        /* 现在模拟"2 秒后才来的上报" */
        tx_len = 0;
        for (i = 0u; i < (uint16_t)sizeof(rep); i++) {
            u2_deliver(&s_rxp1, &huart1, rep[i]);
        }
        pump(3);
        CHECK(tx_len == (uint16_t)sizeof(rep) && memcmp(tx_buf, rep, sizeof(rep)) == 0,
              "透明桥：迟到的 AS:D,... 自动转给 PC（不用再发别的命令）");
        CHECK(app_state == APP_STATE_IDLE, "透明桥：转发后仍是 IDLE");
    }

    /* 5) 空行 / 回车 不污染帧解析 */
    {
        tx_len = 0;
        memcpy(s_ascii_in, "\r\n", 2u);
        APP_USB_Receive(s_ascii_in, 2u);
        pump(2);
        CHECK(tx_len == 0u && app_state == APP_STATE_IDLE, "空行被吃掉，没有任何反应");
    }
}
#endif /* JIG_DUT_LINK */

/*==============================================================================
 * T29 AS 文本解析器（纯函数表驱动）
 *----------------------------------------------------------------------------
 * 这些函数只回答"认不认得出"，所以可以拿各种畸形输入直接砸，不需要搭时序。
 * 覆盖：正常 / 大小写 / 前后垃圾 / 缺组 / 单组状态码 / 全零 / 上限 / 溢出 / 前导 0
 *============================================================================*/
#if (JIG_DUT_LINK != 0)
static void t29_as_parse(void)
{
    uint16_t a, b, c;
    struct { const char *s; int ok; uint16_t a, b, c; } enc[] = {
        { "AS:A100B100C10\r\n",  1, 100, 100, 10 },
        { "as:a100b100c10",      1, 100, 100, 10 },   /* 大小写不敏感 */
        { "AS:A0B0C0\r\n",       1,   0,   0,  0 },   /* 全零也算合法（没 Z 时就这样） */
        { "log xxAS:A7B9C2yy",   1,   7,   9,  2 },   /* 前后有垃圾也能挑出来 */
        { "AS:A65535B65535C65535", 1, 65535, 65535, 65535 },
        { "AS:A00100B0100C010",  1, 100, 100, 10 },   /* 前导 0 容错（对面不该发） */
        { "AS:A0\r\n",           0,   0,   0,  0 },   /* 单组 = 自检状态码，不是上报 */
        { "AS:A100B100",         0,   0,   0,  0 },   /* 缺 C 组 */
        { "AS:A100B100C",        0,   0,   0,  0 },   /* C 组没数字 */
        { "AS:A65536B1C1",       0,   0,   0,  0 },   /* 超 65535 → 整条不认 */
        { "AS:A123456B1C1",      0,   0,   0,  0 },   /* 超 5 位 → 不认 */
        { "B100C10",             0,   0,   0,  0 },   /* 没头 */
    };
    int i;

    printf("\n-- T29 AS 文本解析器 --\n");
    for (i = 0; i < (int)(sizeof(enc) / sizeof(enc[0])); i++) {
        uint16_t len = (uint16_t)strlen(enc[i].s);
        uint8_t  got;
        char     msg[96];

        a = b = c = 0xDEADu;
        got = APP_AsParseEnc((const uint8_t *)enc[i].s, len, &a, &b, &c);
        sprintf(msg, "ParseEnc[%d] \"%s\" → %s", i, enc[i].s, enc[i].ok ? "认" : "不认");
        CHECK((got != 0u) == (enc[i].ok != 0), msg);
        if (enc[i].ok && got) {
            sprintf(msg, "ParseEnc[%d] 数值 = %u/%u/%u", i,
                    (unsigned)a, (unsigned)b, (unsigned)c);
            CHECK(a == enc[i].a && b == enc[i].b && c == enc[i].c, msg);
        }
    }

    /* AS:D,A=..,B=..,Z=.. —— FOC 实际的上报格式 */
    {
        struct { const char *s; int ok; uint16_t a, b, z; } dd[] = {
            { "AS:D,A=25,B=83,Z=0\r\n",  1,  25,  83,  0 },
            { "as:d,a=100,b=100,z=10",     1, 100, 100, 10 },
            { "AS:D,B=7,A=3,Z=9",          1,   3,   7,  9 },  /* 顺序无所谓 */
            { "AS:D,A=25,B=83",            0,   0,   0,  0 },  /* 缺 Z */
            { "AS:A0\r\n",               0,   0,   0,  0 },  /* 没 AS:D 前缀 */
            { "AS:D,A=99999,B=1,Z=1",      0,   0,   0,  0 },  /* 超 65535 */
        };
        int k;

        for (k = 0; k < (int)(sizeof(dd) / sizeof(dd[0])); k++) {
            uint16_t la = 0, lb = 0, lz = 0;
            uint8_t  g2 = APP_AsParseEncD((const uint8_t *)dd[k].s,
                                          (uint16_t)strlen(dd[k].s), &la, &lb, &lz);
            char     m2[96];

            sprintf(m2, "ParseEncD[%d] \"%s\" → %s", k, dd[k].s, dd[k].ok ? "认" : "不认");
            CHECK((g2 != 0u) == (dd[k].ok != 0), m2);
            if (dd[k].ok && g2) {
                sprintf(m2, "ParseEncD[%d] 数值 = %u/%u/%u", k,
                        (unsigned)la, (unsigned)lb, (unsigned)lz);
                CHECK(la == dd[k].a && lb == dd[k].b && lz == dd[k].z, m2);
            }
        }
    }

    /* 前缀匹配：版本号 */
    CHECK(APP_AsHasPrefix((const uint8_t *)"bb:0.3.0\r\n", 10u, AS_PREFIX_VERSION) == 1u,
          "实测回字 bb:0.3.0 → 版本前缀认得出");
    CHECK(APP_AsHasPrefix((const uint8_t *)"AS:V1.2.3\r\n", 11u, AS_PREFIX_VERSION2) == 1u,
          "兼容前缀 AS:V 也认");
    CHECK(APP_AsHasPrefix((const uint8_t *)"BB:0.3.0", 8u, AS_PREFIX_VERSION) == 1u,
          "大写 BB: 也认（大小写不敏感）");
    CHECK(APP_AsHasPrefix((const uint8_t *)"as:v1.2.3", 9u, AS_PREFIX_VERSION2) == 1u,
          "小写 as:v 也认（兼容前缀）");
    CHECK(APP_AsHasPrefix((const uint8_t *)"AS:A0", 5u, AS_PREFIX_VERSION) == 0u,
          "AS:A0 不是版本前缀");

    /* 状态码：A0~A3，且必须"完整"（后面是结束符） */
    CHECK(APP_AsFindCode((const uint8_t *)"AS:A0\r\n", 7u) == 0u, "A0 = 自检通过");
    CHECK(APP_AsFindCode((const uint8_t *)"as:a1", 5u) == 1u, "a1 = ADC 校准失败（小写）");
    CHECK(APP_AsFindCode((const uint8_t *)"AS:A2\n", 6u) == 2u, "A2 = 编码器校准失败");
    CHECK(APP_AsFindCode((const uint8_t *)"AS:A3", 5u) == 3u, "A3 = 相位缺失（串尾）");
    CHECK(APP_AsFindCode((const uint8_t *)"AS:A4\r\n", 7u) == 0xFFu, "A4 不是合法状态码");
    CHECK(APP_AsFindCode((const uint8_t *)"AS:A0B0C0\r\n", 11u) == 0xFFu,
          "AS:A0B0C0 是编码器上报，不能被认成状态码 A0");
    CHECK(APP_AsFindCode((const uint8_t *)"hello", 5u) == 0xFFu, "没有状态码时返回 FF");
}
#endif /* JIG_DUT_LINK */

/*==============================================================================
 * T30 0x0022 六步测试流程
 *----------------------------------------------------------------------------
 * 两条路径：
 *   SC1  FOC 回 AS:A0 —— 全好（ADC 没注入 → 步1 失败，其余照跑）
 *   SC2  FOC 回 AS:A2 —— 走编码器分支：发 AS:MC、治具打 100/100/10、解析 AS:AaBbCc
 * 报告 21B：[0]状态 [1]跑到第几步 [2]第一个失败步 + 6×(code, detail u16)
 *============================================================================*/
#if (JIG_DUT_LINK != 0)
static void as_inject(const char *txt)
{
    while (*txt != '\0') {
        u2_deliver(&s_rxp1, &huart1, (uint8_t)(*txt));
        txt++;
    }
}

static int as_sent(const char *cmd)
{
    uint16_t n = (uint16_t)strlen(cmd);
    uint16_t i;

    if ((cmd == 0) || (n == 0u) || (s_dut_raw_len < n)) {
        return 0;
    }
    for (i = 0u; (uint16_t)(i + n) <= s_dut_raw_len; i++) {
        if (memcmp(&s_dut_raw[i], cmd, n) == 0) {
            return 1;
        }
    }
    return 0;
}

static void t30_flow6(void)
{
    uint16_t n, ac, as_, dlen;
    uint16_t seq = 0x0E00u;
    uint8_t  d[FRAME_MAX_DATA];

    printf("\n-- T30 0x0022 六步测试流程 --\n");

    /* ═══ SC1：AS:A0（全好）═══ */
    {
        s_dut_raw_len = 0;
        tx_len = 0;
        n = Frame_Build((uint8_t *)s_ascii_in, MGR_CMD_FLOW_TEST, seq++, 0, 0u);
        APP_USB_Receive((uint8_t *)s_ascii_in, n);
        host_advance(200u); pump(2);
        CHECK(app_state == APP_STATE_FLOW6, "受理后进入 FLOW6");
        CHECK(as_sent("AS:Bb"), "步2 先问版本（AS:Bb）");

        as_inject("AS:V1.2.3\r\n");
        pump(2);
        CHECK(as_sent("AS:CS"), "收到版本后发 AS:CS 自检");

        as_inject("AS:A0\r\n");
        pump(2);
        CHECK(as_sent("AS:S800"), "A0 之后起转（AS:S800）");
        host_advance(2100u); pump(2);
        CHECK(as_sent("AS:S0"), "转满 2 秒后停车（AS:S0）");

        host_advance(1100u);
        pump(3);                                       /* 静置 1 秒 → DONE → 发报告 */
        CHECK(tx_len > 0u && Frame_Parse(tx_buf, tx_len, &ac, &as_, d, &dlen) != 0u,
              "流程跑完给出报告");
        CHECK(ac == MGR_CMD_FLOW_TEST && dlen == MGR_FLOW_TEST_RESP_LEN,
              "报告 21 字节");
        CHECK(d[1] == 6u, "跑到第 6 步");
        CHECK(d[3] == 0x02u, "步1(ADC)：宿主没注入电压 → 判失败");
        CHECK(d[6] == 0x01u, "步2(存活)：通过");
        CHECK(d[9] == 0x01u && d[10] == 0u && d[11] == 0u,
              "步3(自检)：通过，detail = A0");
        CHECK(d[12] == 0x00u, "步4(三相)：A0 时跳过（code 00）");
        CHECK(d[15] == 0x00u, "步5(编码器)：A0 时跳过（code 00）");
        CHECK(d[18] == 0x01u, "步6(收尾)：通过");
        CHECK(d[2] == 1u, "第一个失败步 = 1（ADC）");
        CHECK(d[0] == APP_STATUS_ERR_TEST, "有失败 → 状态 05");
        CHECK(app_state == APP_STATE_IDLE, "结束后回 IDLE");
    }

    /* ═══ SC2：AS:A2 → 编码器分支 ═══ */
    {
        s_dut_raw_len = 0;
        tx_len = 0;
        n = Frame_Build((uint8_t *)s_ascii_in, MGR_CMD_FLOW_TEST, seq++, 0, 0u);
        APP_USB_Receive((uint8_t *)s_ascii_in, n);
        host_advance(200u); pump(2);
        as_inject("AS:V1.2.3\r\n");
        pump(2);
        as_inject("AS:A2\r\n");
        pump(2);
        CHECK(as_sent("AS:S0"), "A2 也要先停电机");
        host_advance(60u);
        pump(3);
        CHECK(as_sent("AS:MC"), "A2 分支：发 AS:MC 进数编码器模式");

        host_advance(30u); as_inject("AS:MC0\r\n");
        pump(2);                                       /* 进模式 → 开始打脉冲 */
        CHECK(g_enc_rec_cnt[0] == 100u && g_enc_rec_cnt[1] == 100u &&
              g_enc_rec_cnt[2] == 10u,
              "治具打 A=100 B=100 Z=10");
        CHECK(g_enc_rec_freq[0] == 5000u, "脉冲频率 5kHz");

        CHECK(as_sent("AS:ME"), "脉冲发完发 AS:ME 让对面结束并上报");
        as_inject("AS:D,A=100,B=100,Z=10\r\n");
        pump(4);
        CHECK(tx_len > 0u && Frame_Parse(tx_buf, tx_len, &ac, &as_, d, &dlen) != 0u
              && dlen == MGR_FLOW_TEST_RESP_LEN, "编码器分支给出报告");
        CHECK(d[15] == 0x01u, "步5(编码器)：通过");
        CHECK(((uint16_t)((d[16] << 8) | d[17])) == 0x0007u, "三相对数 → 掩码 7");
        CHECK(d[12] == 0x00u, "步4(三相)：A2 时跳过");
        CHECK(app_state == APP_STATE_IDLE, "结束后回 IDLE");
    }

    /* ═══ SC3：AS:A2 + 没有 Z（报 C0）→ 仍算三相对 ═══ */
    {
        s_dut_raw_len = 0;
        tx_len = 0;
        n = Frame_Build((uint8_t *)s_ascii_in, MGR_CMD_FLOW_TEST, seq++, 0, 0u);
        APP_USB_Receive((uint8_t *)s_ascii_in, n);
        host_advance(200u); pump(2);
        as_inject("AS:V1.2.3\r\n");
        pump(2);
        as_inject("AS:A2\r\n");
        pump(2);
        host_advance(60u);
        pump(3);
        host_advance(30u); as_inject("AS:MC0\r\n");
        pump(2);
        pump(4);                                         /* 等脉冲发完 + 发出 AS:ME */
        as_inject("AS:D,A=100,B=100,Z=0\r\n");         /* 没有 Z 的编码器 */
        pump(4);
        CHECK(tx_len > 0u && Frame_Parse(tx_buf, tx_len, &ac, &as_, d, &dlen) != 0u
              && dlen == MGR_FLOW_TEST_RESP_LEN, "无 Z 的编码器也能给出报告");
        CHECK(d[15] == 0x01u, "步5(编码器)：C0 且 A/B 对 → 仍判通过");
        CHECK(((uint16_t)((d[16] << 8) | d[17])) == 0x0007u, "掩码仍是 7（Z 按不判处理）");
    }

    /* ═══ SC4：计数差太多 → 判失败 ═══ */
    {
        s_dut_raw_len = 0;
        tx_len = 0;
        n = Frame_Build((uint8_t *)s_ascii_in, MGR_CMD_FLOW_TEST, seq++, 0, 0u);
        APP_USB_Receive((uint8_t *)s_ascii_in, n);
        host_advance(200u); pump(2);
        as_inject("AS:V1.2.3\r\n");
        pump(2);
        as_inject("AS:A2\r\n");
        pump(2);
        host_advance(60u);
        pump(3);
        host_advance(30u); as_inject("AS:MC0\r\n");
        pump(2);
        pump(4);
        as_inject("AS:D,A=150,B=100,Z=10\r\n");        /* A 相多了 50 个 → 超 ±2% */
        pump(4);
        CHECK(tx_len > 0u && Frame_Parse(tx_buf, tx_len, &ac, &as_, d, &dlen) != 0u
              && dlen == MGR_FLOW_TEST_RESP_LEN, "计数不符也能给报告");
        CHECK(d[15] == 0x02u, "步5(编码器)：A 相数不对 → 判失败");
        CHECK(((uint16_t)((d[16] << 8) | d[17])) == 0x0006u, "掩码 = bit1|bit2（A 相没对上）");
    }
}
#endif /* JIG_DUT_LINK */

/*==============================================================================
 * T31 0x0022 步4：AS:PW 三相 1kHz/50%
 *----------------------------------------------------------------------------
 * 步4 的判据是"周期 0.5~2ms、占空比 450~550‰"。这个用例把替身设成 1kHz/50%，
 * 验证三分支：正常通过 / 频率不对 / 占空比不对。
 *============================================================================*/
#if (JIG_DUT_LINK != 0)
/* 跑到第 4 步并等到报告：返回 1 = 拿到 21B 报告 */
static int flow6_run_a3(uint8_t *d, uint16_t *dlen_out)
{
    uint16_t n, ac, as_, dlen;
    uint16_t seq = 0x0F00u;
    uint32_t i;

    s_dut_raw_len = 0;
    tx_len = 0;
    n = Frame_Build((uint8_t *)s_ascii_in, MGR_CMD_FLOW_TEST, seq, 0, 0u);
    APP_USB_Receive((uint8_t *)s_ascii_in, n);
    host_advance(200u); pump(2);
    as_inject("AS:V1.2.3\r\n");
    pump(2);
    as_inject("AS:A3\r\n");                        /* 相位缺失 → 走步4 */
    pump(2);
    host_advance(60u);
    pump(3);
    if (!as_sent("AS:PW")) {
        return 0;
    }
    for (i = 0u; i < 6000u; i++) {                 /* 等采集窗口跑完（200ms）*/
        if (tx_len > 0u) {
            break;
        }
        host_advance(1u);
        pump(1);
    }
    if (tx_len == 0u) {
        return 0;
    }
    if (Frame_Parse(tx_buf, tx_len, &ac, &as_, d, &dlen) == 0u) {
        return 0;
    }
    *dlen_out = dlen;
    return 1;
}

static void t31_flow6_pwm(void)
{
    uint8_t  d[FRAME_MAX_DATA];
    uint16_t dlen = 0;

    printf("\n-- T31 0x0022 步4：AS:PW 三相 1kHz/50%% --\n");

    /* ── 1) 1kHz / 50%：三路都该判"有波形" ── */
    pulse_fake_set(1000u, 500u);
    CHECK(flow6_run_a3(d, &dlen) != 0, "A3 分支能跑到第 4 步并给出报告");
    CHECK(dlen == MGR_FLOW_TEST_RESP_LEN, "报告 21 字节");
    CHECK(d[12] == 0x01u, "步4(三相)：1kHz/50% 三路都有 → 通过");
    CHECK(((uint16_t)((d[13] << 8) | d[14])) == 0x0007u, "掩码 7（A/B/C 都有波形）");
    CHECK(d[0] == APP_STATUS_ERR_TEST && d[2] == 1u,
          "步1(ADC) 没注入 → 整体 05、第一个失败步仍是 1");
    CHECK(app_state == APP_STATE_IDLE, "结束后回 IDLE");

    /* ── 2) 频率不对（20kHz）：判"没波形" ── */
    pulse_fake_set(20000u, 500u);
    CHECK(flow6_run_a3(d, &dlen) != 0, "频率不对时也能收尾出报告");
    CHECK(d[12] == 0x02u, "步4：20kHz 不在 0.5~2ms 窗口内 → 判失败");
    CHECK(((uint16_t)((d[13] << 8) | d[14])) == 0x0000u, "掩码 0（三路都不算有波形）");

    /* ── 3) 占空比不对（80%）── */
    pulse_fake_set(1000u, 800u);
    CHECK(flow6_run_a3(d, &dlen) != 0, "占空比不对时也能收尾出报告");
    CHECK(d[12] == 0x02u, "步4：80% 占空比超出 450~550‰ → 判失败");
    CHECK(((uint16_t)((d[13] << 8) | d[14])) == 0x0000u, "掩码 0");

    /* ── 4) 边界：470‰ 仍算合格（容差内）── */
    pulse_fake_set(1000u, 470u);
    CHECK(flow6_run_a3(d, &dlen) != 0, "边界占空比也能收尾");
    CHECK(d[12] == 0x01u, "步4：470‰ 在容差内 → 仍判通过");

    pulse_fake_reset();                            /* 恢复默认 20kHz，别影响别的用例 */
}
#endif /* JIG_DUT_LINK */

/*==============================================================================
 * T32~T37 0x0022 的其余路径（把"每一步"的通过与失败都覆盖到）
 *============================================================================*/
#if (JIG_DUT_LINK != 0)
/* 把 6 路 ADC 灌成"应该合格"的码值，并等 EMA 收敛。
 * 换算：raw = mv × 4095 × k_num / (3300 × k_den)（与 measure.c 的 k_cfg 一致） */
static void adc_inject_ok(void)
{
    MEAS_TestInject((meas_id_t)MEAS_CH_V24,  3773u);   /* 24V   (k=47/371) */
    MEAS_TestInject((meas_id_t)MEAS_CH_V10,  1572u);   /* 10V   (k=47/371) */
    MEAS_TestInject((meas_id_t)MEAS_CH_V5,   3723u);   /* 5V    (k=3/5)    */
    MEAS_TestInject((meas_id_t)MEAS_CH_V33,  3723u);   /* 3.3V  (k=10/11 -> 引脚 3000mV) */
    MEAS_TestInject((meas_id_t)MEAS_CH_CUR1, 2048u);   /* 1.65V (k=1)      */
    MEAS_TestInject((meas_id_t)MEAS_CH_CUR2, 2048u);
    for (int i = 0; i < 800; i++) {
        MEAS_Poll();
        g_tick++;
    }
    {
        meas_snapshot_t dbg;
        MEAS_Snapshot(&dbg);
        printf("     [ADC] ");
        for (int i = 0; i < 6; i++) {
            static const int ch[6] = { 0, 1, 2, 3, 7, 8 };
            printf("ch%d=%umV/0x%02X ", ch[i],
                   (unsigned)dbg.ch[ch[i]].mv, (unsigned)dbg.ch[ch[i]].flags);
        }
        printf("\n");
    }
}

/* 发一条 0x0022 并跑到"步1 完成" */
static void f6_start(uint16_t seq)
{
    uint16_t n;

    s_dut_raw_len = 0;
    tx_len = 0;
    n = Frame_Build((uint8_t *)s_ascii_in, MGR_CMD_FLOW_TEST, seq, 0, 0u);
    APP_USB_Receive((uint8_t *)s_ascii_in, n);
    host_advance(200u); pump(2);
}

/* 等报告（最多 6000 个 tick）；返回 1 = 拿到 */
static int f6_wait(uint8_t *d, uint16_t *dlen)
{
    uint16_t ac, as_;
    uint32_t i;

    for (i = 0u; i < 6000u; i++) {
        if (tx_len > 0u) {
            break;
        }
        host_advance(1u);
        pump(1);
    }
    if (tx_len == 0u) {
        return 0;
    }
    return (Frame_Parse(tx_buf, tx_len, &ac, &as_, d, dlen) != 0u) ? 1 : 0;
}

static void t32_flow6_all_green(void)
{
    uint8_t  d[FRAME_MAX_DATA];
    uint16_t dlen = 0;

    printf("\n-- T32 六步流程：全绿路径 --\n");
    adc_inject_ok();
    pulse_fake_set(1000u, 500u);          /* 备用：A0 不会走到步4 */

    f6_start(0x1100u);
    CHECK(as_sent("AS:Bb"), "步2 发出 AS:Bb");
    as_inject("AS:V1.2.3\r\n");
    pump(2);
    CHECK(as_sent("AS:CS"), "发出 AS:CS");
    as_inject("AS:A0\r\n");
    pump(2);
    CHECK(as_sent("AS:S800"), "A0 之后起转（AS:S800）");
    host_advance(2100u); pump(2);
    CHECK(as_sent("AS:S0"), "转满 2 秒后停车");
    host_advance(1100u);
    pump(3);

    CHECK(f6_wait(d, &dlen) != 0, "给出报告");
    CHECK(dlen == MGR_FLOW_TEST_RESP_LEN, "报告 21 字节");
    CHECK(d[0] == APP_STATUS_OK, "**六步全过 → 状态 00**");
    CHECK(d[1] == 6u && d[2] == 0u, "跑到第 6 步，没有失败步");
    CHECK(d[3] == 0x01u, "步1(ADC)：注入合格码值 → 通过");
    CHECK(((uint16_t)((d[4] << 8) | d[5])) == 0x00FFu, "步1 detail = FF（六路都好）");
    CHECK(d[6] == 0x01u, "步2(存活)：通过");
    CHECK(d[9] == 0x01u, "步3(自检)：通过");
    CHECK(d[12] == 0x00u && d[15] == 0x00u, "步4/步5：A0 时跳过");
    CHECK(d[18] == 0x01u, "步6(收尾)：通过");
    CHECK(app_state == APP_STATE_IDLE, "回 IDLE");

    pulse_fake_reset();
}

static void t33_flow6_alive_timeout(void)
{
    uint8_t  d[FRAME_MAX_DATA];
    uint16_t dlen = 0;
    int      i;

    printf("\n-- T33 六步流程：存活超时（板子没起来）--\n");
    f6_start(0x1110u);
    /* 15 次轮询、每次 200ms：一路推进但不注入任何回字 */
    for (i = 0; i < 60; i++) {
        if (app_state != APP_STATE_FLOW6) {
            break;
        }
        host_advance(220u);
        pump(2);
    }
    CHECK(f6_wait(d, &dlen) != 0, "超时后仍能收尾出报告");
    CHECK(d[6] == 0x02u, "步2(存活)：判失败");
    CHECK(((uint16_t)((d[7] << 8) | d[8])) == 15u, "步2 detail = 问了 15 次");
    CHECK(d[9] == 0x00u, "步3(自检)：**跳过**（板子没起来就不再问）");
    CHECK(d[2] == 2u, "第一个失败步 = 2");
    CHECK(d[0] == APP_STATUS_ERR_TEST, "整体 05");
}

static void t34_flow6_a1(void)
{
    uint8_t  d[FRAME_MAX_DATA];
    uint16_t dlen = 0;

    printf("\n-- T34 六步流程：AS:A1（ADC 校准失败）--\n");
    f6_start(0x1120u);
    as_inject("AS:V1.2.3\r\n");
    pump(2);
    as_inject("AS:A1\r\n");
    pump(2);
    CHECK(as_sent("AS:S0"), "A1 也要先停车");
    host_advance(60u);
    pump(3);
    CHECK(f6_wait(d, &dlen) != 0, "给出报告");
    CHECK(d[9] == 0x01u, "步3(自检)：命令本身成功（拿到回复）");
    CHECK(((uint16_t)((d[10] << 8) | d[11])) == 1u, "步3 detail = 1（A1）");
    CHECK(d[12] == 0x00u && d[15] == 0x00u, "步4/步5：A1 时都跳过");
    CHECK(d[2] == 3u, "第一个失败步 = 3（A1 是硬失败）");
    CHECK(d[0] == APP_STATUS_ERR_TEST, "整体 05");
}

static void t35_flow6_pwm_no_signal(void)
{
    uint8_t  d[FRAME_MAX_DATA];
    uint16_t dlen = 0;
    uint32_t i;

    printf("\n-- T35 六步流程：A3 但三相一个边沿都没有 --\n");
    g_pulse_edges_on = 0u;                /* 替身不灌任何边沿 */
    f6_start(0x1130u);
    as_inject("AS:V1.2.3\r\n");
    pump(2);
    as_inject("AS:A3\r\n");
    pump(2);
    host_advance(60u);
    pump(3);
    CHECK(as_sent("AS:PW"), "A3 分支发出 AS:PW");
    for (i = 0u; i < 6000u; i++) {
        if (tx_len > 0u) {
            break;
        }
        host_advance(1u);
        pump(1);
    }
    CHECK(f6_wait(d, &dlen) != 0, "给出报告");
    CHECK(d[12] == 0x02u, "步4(三相)：没有波形 → 失败");
    CHECK(((uint16_t)((d[13] << 8) | d[14])) == 0x0000u, "掩码 0");
    g_pulse_edges_on = 1u;
}

static void t36_flow6_enc_timeout(void)
{
    uint8_t  d[FRAME_MAX_DATA];
    uint16_t dlen = 0;
    uint32_t i;

    printf("\n-- T36 六步流程：A2 但 FOC 不报数 --\n");
    f6_start(0x1140u);
    as_inject("AS:V1.2.3\r\n");
    pump(2);
    as_inject("AS:A2\r\n");
    pump(2);
    host_advance(60u);
    pump(3);
    CHECK(as_sent("AS:MC"), "发出 AS:MC");
    host_advance(30u); as_inject("AS:MC0\r\n");
    pump(2);
    CHECK(g_enc_start_calls > 0u, "治具开始打脉冲");
    /* 之后什么都不注入，等 2 秒超时 */
    for (i = 0u; i < 4000u; i++) {
        if (tx_len > 0u) {
            break;
        }
        host_advance(1u);
        pump(1);
    }
    CHECK(f6_wait(d, &dlen) != 0, "超时后仍能收尾");
    CHECK(d[15] == 0x02u, "步5(编码器)：没收到报数 → 失败");
    CHECK(((uint16_t)((d[16] << 8) | d[17])) == 0xFFF0u,
          "detail = 0xFFF0（没收到 AS:D，与数不对区分开）");
}

static void t37_flow6_param_err(void)
{
    uint16_t n, ac, as_, dlen;
    uint8_t  d[FRAME_MAX_DATA];
    uint8_t  one = 0x01u;

    printf("\n-- T37 六步流程：参数校验 --\n");
    tx_len = 0;
    n = Frame_Build((uint8_t *)s_ascii_in, MGR_CMD_FLOW_TEST, 0x1150u, &one, 1u);
    APP_USB_Receive((uint8_t *)s_ascii_in, n);
    pump(3);
    CHECK(tx_len > 0u && Frame_Parse(tx_buf, tx_len, &ac, &as_, d, &dlen) != 0u
          && dlen == 1u && d[0] == APP_STATUS_ERR_PARAM,
          "LEN≠0 → 0B ERR_PARAM");
}

static void t32_t37_flow6_rest(void)
{
    t32_flow6_all_green();
    t33_flow6_alive_timeout();
    t34_flow6_a1();
    t35_flow6_pwm_no_signal();
    t36_flow6_enc_timeout();
    t37_flow6_param_err();
}
#endif /* JIG_DUT_LINK */

/*==============================================================================
 * T31b 0x0022 步4：单相缺失（掩码 3 / 5 / 6）
 *----------------------------------------------------------------------------
 * 真机场景：某一相驱动级坏了 → 那一相没有 PWM。
 * 治具必须说得出"是哪一相没有"，所以 detail 的掩码要逐位准。
 *============================================================================*/
#if (JIG_DUT_LINK != 0)
static void t31b_flow6_pwm_single_phase(void)
{
    uint8_t  d[FRAME_MAX_DATA];
    uint16_t dlen = 0;
    struct { uint8_t mask; uint16_t want_det; const char *why; } cases[] = {
        { 0x03u, 0x0003u, "C 相没波形（A/B 有）" },
        { 0x05u, 0x0005u, "B 相没波形（A/C 有）" },
        { 0x06u, 0x0006u, "A 相没波形（B/C 有）" },
        { 0x01u, 0x0001u, "只有 A 相有波形" },
        { 0x00u, 0x0000u, "三相都没波形" },
    };
    int i;

    printf("\n-- T31b 0x0022 步4：单相缺失 --\n");

    for (i = 0; i < (int)(sizeof(cases) / sizeof(cases[0])); i++) {
        char msg[96];

        pulse_fake_set(1000u, 500u);                 /* 频率/占空比正常，只是缺相 */
        pulse_fake_phase_mask(cases[i].mask);
        CHECK(flow6_run_a3(d, &dlen) != 0, "缺相时也能收尾出报告");
        sprintf(msg, "%s → 步4 判失败", cases[i].why);
        CHECK(d[12] == 0x02u, msg);
        sprintf(msg, "%s → 掩码 = 0x%04X", cases[i].why, (unsigned)cases[i].want_det);
        CHECK(((uint16_t)((d[13] << 8) | d[14])) == cases[i].want_det, msg);
    }

    pulse_fake_reset();                              /* 恢复：20kHz/50%/三相全灌 */
}
#endif /* JIG_DUT_LINK */

/*==============================================================================
 * T38 0x0022 强制分支（调试用）：健康板也能走步5
 *----------------------------------------------------------------------------
 * 请求 LEN=1 DATA=[02] → 不管 AS:CS 回什么，都按 A2 处理 → 一定走步5。
 * 报告里步3 的 detail 应显示被强制的码（02），如实反映"这次是怎么走的"。
 *============================================================================*/
#if (JIG_DUT_LINK != 0)
static void t38_flow6_force(void)
{
    uint16_t n, ac, as_, dlen;
    uint8_t  d[FRAME_MAX_DATA];
    uint8_t  force = MGR_FLOW_FORCE_A2;
    uint32_t i;

    printf("\n-- T38 0x0022 强制分支（强制走步5）--\n");

    adc_inject_ok();                       /* 让步1 通过，报告更干净 */
    s_dut_raw_len = 0;
    tx_len = 0;
    n = Frame_Build((uint8_t *)s_ascii_in, MGR_CMD_FLOW_TEST, 0x1180u, &force, 1u);
    APP_USB_Receive((uint8_t *)s_ascii_in, n);
    host_advance(200u); pump(2);

    as_inject("bb:0.3.0\r\n");             /* 步2：实测的版本回字格式 */
    pump(2);
    CHECK(as_sent("AS:CS"), "发出 AS:CS");
    as_inject("AS:A0\r\n");                /* 真实是 A0，但强制分支要按 A2 走 */
    pump(2);
    CHECK(as_sent("AS:S800"), "A0 后先起转（AS:S800）");
    host_advance(2100u); pump(2);          /* 起转观察 2 秒 */
    host_advance(1100u); pump(3);          /* 停车静置 1 秒 → 强制分支生效 */
    CHECK(as_sent("AS:MC"), "**强制后仍然走进步5**（发出 AS:MC）");

    host_advance(30u); as_inject("AS:MC0\r\n");               /* 对面进计数模式 */
    pump(2);
    CHECK(g_enc_start_calls > 0u, "收到 AS:MC0 之后才开始打脉冲");
    pump(2);                               /* 等脉冲发完 → 发 AS:ME */
    CHECK(as_sent("AS:ME"), "脉冲发完发 AS:ME");
    as_inject("AS:D,A=100,B=100,Z=10\r\n");
    pump(4);

    for (i = 0u; i < 4000u; i++) {
        if (tx_len > 0u) {
            break;
        }
        host_advance(1u);
        pump(1);
    }
    CHECK(tx_len > 0u && Frame_Parse(tx_buf, tx_len, &ac, &as_, d, &dlen) != 0u
          && dlen == MGR_FLOW_TEST_RESP_LEN, "强制分支给出报告");
    CHECK(d[9] == 0x01u, "步3(自检)：命令成功");
    CHECK(((uint16_t)((d[10] << 8) | d[11])) == 2u, "步3 detail = 2（被强制成 A2）");
    CHECK(d[15] == 0x01u, "步5(编码器)：通过");
    CHECK(((uint16_t)((d[16] << 8) | d[17])) == 0x0007u, "步5 掩码 = 7");
    CHECK(d[12] == 0x00u, "步4(三相)：跳过");
    CHECK(app_state == APP_STATE_IDLE, "结束后回 IDLE");

    /* 参数校验：非法强制码 → 0B */
    {
        uint8_t bad = 0x07u;

        tx_len = 0;
        n = Frame_Build((uint8_t *)s_ascii_in, MGR_CMD_FLOW_TEST, 0x1181u, &bad, 1u);
        APP_USB_Receive((uint8_t *)s_ascii_in, n);
        pump(3);
        CHECK(tx_len > 0u && Frame_Parse(tx_buf, tx_len, &ac, &as_, d, &dlen) != 0u
              && dlen == 1u && d[0] == APP_STATUS_ERR_PARAM,
              "非法强制码 → 0B ERR_PARAM");
    }
}
#endif /* JIG_DUT_LINK */

/*==============================================================================
 * T39 慢回也算活着：`AS:Bb` 的回复在几次重试之后才到，仍要认出来
 *----------------------------------------------------------------------------
 * 实测 FOC 回一条命令要 1.1~1.8 秒；步2 的重试间隔只有几百毫秒。
 * 原来每次重试都清空接收缓冲 → 慢回被抹掉 → 误判"板子没起来"。
 *============================================================================*/
#if (JIG_DUT_LINK != 0)
static void t39_flow6_slow_alive(void)
{
    uint8_t  d[FRAME_MAX_DATA];
    uint16_t dlen = 0;
    uint32_t i;

    printf("\n-- T39 慢回也算活着（AS:Bb 延迟回复）--\n");
    f6_start(0x1190u);

    /* 先让它重试几轮（不注入任何回字） */
    for (i = 0u; i < 4u; i++) {
        host_advance(450u);
        pump(2);
    }
    CHECK(app_state == APP_STATE_FLOW6, "还在步2 重试中");

    /* 现在才注入回复（"慢回"） */
    as_inject("bb:0.3.0\r\n");
    pump(2);
    CHECK(as_sent("AS:CS"), "慢到的版本回复也能认出来 → 继续走步3");

    /* 收尾：给个 A0，让它正常结束 */
    as_inject("AS:A0\r\n");
    pump(2);
    host_advance(60u);
    pump(3);
    CHECK(f6_wait(d, &dlen) != 0, "能给出报告");
    CHECK(d[6] == 0x01u, "步2(存活)：通过");
    CHECK(d[9] == 0x01u, "步3(自检)：通过");
}
#endif /* JIG_DUT_LINK */

int main(void)
{
    printf("=== FOC JIG U1 全链路宿主仿真 (PC帧→上传→SPI Flash→SWD→被烧板) ===\n\n");
    t1_flash_id();
    t2_empty_and_unknown();
    t3_upload_and_slot_switch();
    t4_start_param_errors();
    t5_overflow_and_latch();
    t6_count_mismatch();
    t7_upload_timeout();
    t8_program_precheck();
    t9_full_program_chain();
    t10_busy_and_target_absent();
    t13_flash_offline_detected();
    t14_measure_cmd();
    t15_pulse_cmd();
#if (JIG_DUT_LINK != 0)
    /* DUT 模式：端口0 被 DUT 链路占用（jig_mode.h §3.4），U2 相关的 T16 不跑 */
    t17_dut_passthrough();
    t20_cur_test();             /* ⑥ 相电流：需要 DUT 链路 + 假 DUT */
    t21_vol_test();             /* ② 电压检测：9 路 vs DUT */
#else
    t16_arm_sync_path();
#endif
    t18_enc_gen();              /* ⑤ 编码器脉冲发生：两种构建里都跑（不依赖端口0） */
    t22_phase_judge();          /* ④ 三相输出判据：两种构建里都跑（不依赖 DUT 链路） */
    t23_jig_status();           /* 0x0017 治具自检：两种构建里都跑 */
    t24_enc_judge();            /* 0x0018 ⑤ 三方对齐判定：两种构建里都跑 */
#if (JIG_DUT_LINK != 0)
    t25_run_all();              /* 0x0010 一键全流程：需要 DUT 链路 */
    t26_one_cmd_flows();        /* 0x0019/0x001A/0x001B 单项打包 */
#if (JIG_DUT_LINK != 0)
    t27_dut_raw();              /* 0x001F/0x0020 DUT 原始字节收发 */
#endif
#if (JIG_DUT_LINK != 0)
    t28_ascii_console();        /* PC 串口助手 ASCII 控制台 */
#endif
#if (JIG_DUT_LINK != 0)
    t29_as_parse();             /* AS 文本解析器（纯函数） */
#endif
#if (JIG_DUT_LINK != 0)
    t30_flow6();                /* 0x0022 六步流程 */
#endif
#if (JIG_DUT_LINK != 0)
    t31_flow6_pwm();            /* 0x0022 步4：AS:PW 1kHz/50% */
#endif
#if (JIG_DUT_LINK != 0)
    t31b_flow6_pwm_single_phase();   /* 0x0022 步4：单相缺失 */
#endif
#if (JIG_DUT_LINK != 0)
    t32_t37_flow6_rest();       /* 0x0022 其余路径：全绿/存活超时/A1/无信号/编码超时/参数错 */
#endif
#if (JIG_DUT_LINK != 0)
    t38_flow6_force();          /* 0x0022 强制分支 */
    t39_flow6_slow_alive();     /* 0x0022 慢回也算活着 */
#endif
#endif
#if (JIG_DUT_LINK == 0)
    t19_enc_peer();             /* ⑤ 回环：需要假 U2，只在非 DUT 构建里跑 */
#endif
    CHECK(g_tx_oversize == 0u, "没有任何应答被 CDC 替身缓冲截断");
    printf("\n=== 共 %d 项检查, 失败 %d 项 ===\n", g_checks, g_fails);
    return g_fails == 0 ? 0 : 1;
}
