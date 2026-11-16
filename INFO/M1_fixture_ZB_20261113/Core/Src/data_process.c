/**
  ******************************************************************************
  * @file    data_process.c
  * @brief   PC 通讯协议层 + 固件上传状态机（M1 治具）
  *
  * 本文件承担三件事（实现集中在 data_process.c，是为了遵守"不改 Keil 工程"
  * 的约束 —— 新增 .c 不会进编译，所以全部放在这个已在工程里的文件里）：
  *
  *   1) 帧层     A5 5B | CMD(2) | SEQ(2) | LEN(2) | DATA | CRC16(2)
  *               CRC-16/MODBUS，范围 = A5 .. DATA 末字节，字段均大端
  *   2) 分发     FwUpload_Handle() 优先；上传中其它命令回 BUSY
  *   3) 上传     0x0009 START → N×0x000A DATA → 0x000B END
  *               攒满 256B 写一页 SPI Flash，END 时写完整标志并回读校验
  *
  * 数据流：
  *   USB 中断  CDC_Receive_FS → CDC_Transport_RxPush()      （只入包队列）
  *   主循环    CDC_Transport_Poll() → CDC_FrameHandler()     （整包交给本文件）
  *             → pc_frame_feed() → pc_dispatch() → pc_send() → CDC_Transport_Send()
  *
  * 与 FOC_JIG 的行为差异只有一处：上传过程中的应答改走 cdc_transport 的发送队列
  * （immediate=true，等发完才返回），行为等价于原来的 CDC_Transmit_FS 忙等，
  * 但不会与 USB 中断抢占同一个 TxState。
  ******************************************************************************
  */
#include "data_process.h"

#include "usart_m.h"
#include "cdc_transport.h"
#include "log.h"

#include "flash_mode.h"
#include "flash_map.h"
#include "fw_upload.h"
#include "gd25q128.h"

#include "fx_adc.h"
#include "fx_input.h"

#if FLASH_MODE_UART5
#include "UART_FLASH.h"
#else
#include "SWD_FLASH.h"
#endif

#include <string.h>

/*------------------------------------------------------------------------------
 * 烧录方案统一入口：data_process.c 只调这一组别名，
 * 具体实现由 flash_mode.h 的 FLASH_MODE_UART5 选择，互不引用。
 *---------------------------------------------------------------------------*/
#if FLASH_MODE_UART5
#define DUT_FLASH_Init()          UARTF_DutCtrlInit()
#define DUT_FLASH_Start()         UARTF_Start()
#define DUT_FLASH_Step()          UARTF_Step()
#define DUT_FLASH_Busy()          UARTF_Busy()
#define DUT_FLASH_Result()        UARTF_Result()
#define DUT_FLASH_State()         UARTF_State()
#define DUT_FLASH_TotalBytes()    UARTF_TotalBytes()
#define DUT_FLASH_LastOffset()    UARTF_LastOffset()
#define DUT_FLASH_LastError()     UARTF_LastError()
#define DUT_FLASH_FailState()     UARTF_FailState()
#else
#define DUT_FLASH_Init()          SWDP_DutCtrlInit()
#define DUT_FLASH_Start()         SWDP_Start()
#define DUT_FLASH_Step()          SWDP_Step()
#define DUT_FLASH_Busy()          SWDP_Busy()
#define DUT_FLASH_Result()        SWDP_Result()
#define DUT_FLASH_State()         SWDP_State()
#define DUT_FLASH_TotalBytes()    SWDP_TotalBytes()
#define DUT_FLASH_LastOffset()    SWDP_LastOffset()
#define DUT_FLASH_LastError()     SWDP_LastSwdError()
#define DUT_FLASH_FailState()     SWDP_FailState()
#endif

/*==============================================================================
 * 一、帧层
 *============================================================================*/
#define PC_HDR0             (0xA5u)
#define PC_HDR1             (0x5Bu)

static uint16_t s_pc_seq = 0u;              /* 最近一帧的序号，ACK 原样回显 */

static uint16_t pc_crc16(const uint8_t *data, uint16_t len)
{
    uint16_t crc = 0xFFFFu;
    uint16_t i;
    uint8_t  j;

    for (i = 0u; i < len; i++) {
        crc ^= (uint16_t)data[i];
        for (j = 0u; j < 8u; j++) {
            if ((crc & 0x0001u) != 0u) {
                crc = (uint16_t)((crc >> 1) ^ 0xA001u);
            } else {
                crc = (uint16_t)(crc >> 1);
            }
        }
    }
    return crc;
}

static uint16_t pc_frame_build(uint8_t *buf, uint16_t cmd, uint16_t seq,
                               const uint8_t *data, uint16_t len)
{
    uint16_t total;
    uint16_t crc;

    buf[0] = PC_HDR0;
    buf[1] = PC_HDR1;
    buf[2] = (uint8_t)(cmd >> 8);
    buf[3] = (uint8_t)(cmd);
    buf[4] = (uint8_t)(seq >> 8);
    buf[5] = (uint8_t)(seq);
    buf[6] = (uint8_t)(len >> 8);
    buf[7] = (uint8_t)(len);
    if ((len > 0u) && (data != 0)) {
        memcpy(&buf[8], data, len);
    }

    total = (uint16_t)(8u + len);
    crc   = pc_crc16(buf, total);
    buf[total]      = (uint8_t)(crc >> 8);  /* CRC 高字节在前 */
    buf[total + 1u] = (uint8_t)(crc);

    return (uint16_t)(total + 2u);
}

/** 发一整帧应答（USB 链路：阻塞到发送完成，失败只记日志不重试）
 *  @param seq 应答帧里回显的序号：普通命令用 s_pc_seq，
 *             SWD 烧录的"迟到 ACK"用启动那次命令的序号。 */
static void pc_send_seq(uint16_t cmd, uint16_t seq, const uint8_t *data, uint16_t len)
{
    static uint8_t tx[APP_FRAME_BUF_SIZE];
    uint16_t       total;
    CDC_TxResult   rc;

    if (len > FRAME_MAX_DATA) {
        LOGE("pc tx: len %d too large", (int)len);
        return;
    }

    total = pc_frame_build(tx, cmd, seq, data, len);
    rc    = CDC_Transport_Send(tx, total, true);
    if (rc != CDC_TX_OK) {
        LOGW("pc tx failed: cmd=0x%x rc=%d", (int)cmd, (int)rc);
    }
}

static void pc_send(uint16_t cmd, const uint8_t *data, uint16_t len)
{
    pc_send_seq(cmd, s_pc_seq, data, len);
}

/*------------------------------------------------------------------------------
 * 接收字节流 → 完整帧（严格头校验 + 失步重同步，移植自 FOC_JIG/app_frame.c）
 *----------------------------------------------------------------------------*/
typedef enum {
    PC_ST_HDR0 = 0,
    PC_ST_HDR1,
    PC_ST_HEAD,
    PC_ST_BODY
} pc_frame_st_t;

static uint8_t       s_frame_buf[APP_FRAME_BUF_SIZE];
static uint16_t      s_frame_pos;
static uint16_t      s_frame_dlen;
static pc_frame_st_t s_frame_st;

static void pc_dispatch(uint16_t cmd, const uint8_t *data, uint16_t len);

/** 在当前已缓存字节里找下一个 A5 5B，把解析器重新对齐 */
static void pc_frame_resync(void)
{
    uint16_t i;

    for (i = 1u; (uint16_t)(i + 1u) < s_frame_pos; i++) {
        if ((s_frame_buf[i] == PC_HDR0) && (s_frame_buf[i + 1u] == PC_HDR1)) break;
    }

    if ((uint16_t)(i + 1u) < s_frame_pos) {
        uint16_t n = (uint16_t)(s_frame_pos - i);
        memmove(s_frame_buf, &s_frame_buf[i], n);
        s_frame_pos = n;                    /* >= 2, 头两字节已就绪 */
        s_frame_st  = PC_ST_HEAD;
    } else if ((s_frame_pos > 0u) && (s_frame_buf[s_frame_pos - 1u] == PC_HDR0)) {
        s_frame_buf[0] = PC_HDR0;           /* 末尾可能是新帧头的第一个字节 */
        s_frame_pos = 1u;
        s_frame_st  = PC_ST_HDR1;
    } else {
        s_frame_pos = 0u;
        s_frame_st  = PC_ST_HDR0;
    }
}

/** 完整帧到达：校验 CRC，通过后更新序号并分发；CRC 错按协议静默丢弃 */
static void pc_frame_complete(void)
{
    uint16_t calc;
    uint16_t recv;
    uint16_t cmd;
    uint16_t seq;

    calc = pc_crc16(s_frame_buf, (uint16_t)(8u + s_frame_dlen));
    recv = (uint16_t)(((uint16_t)s_frame_buf[8u + s_frame_dlen] << 8) |
                       (uint16_t)s_frame_buf[9u + s_frame_dlen]);
    if (calc != recv) {
        LOGW("pc rx: crc error, len=%d", (int)s_frame_dlen);
        return;
    }

    cmd = (uint16_t)(((uint16_t)s_frame_buf[2] << 8) | (uint16_t)s_frame_buf[3]);
    seq = (uint16_t)(((uint16_t)s_frame_buf[4] << 8) | (uint16_t)s_frame_buf[5]);

    s_pc_seq = seq;
    LOGI("pc rx: cmd=0x%x seq=%d len=%d", (int)cmd, (int)seq, (int)s_frame_dlen);
    pc_dispatch(cmd, &s_frame_buf[8], s_frame_dlen);
}

static void pc_frame_feed(uint8_t b)
{
    switch (s_frame_st) {

    case PC_ST_HDR0:
        if (b == PC_HDR0) {
            s_frame_buf[0] = b;
            s_frame_pos = 1u;
            s_frame_st  = PC_ST_HDR1;
        }
        break;

    case PC_ST_HDR1:
        if (b == PC_HDR1) {
            s_frame_buf[1] = b;
            s_frame_pos = 2u;
            s_frame_st  = PC_ST_HEAD;
        } else if (b == PC_HDR0) {
            s_frame_buf[0] = b;             /* 连续 A5: 停在 HDR1 等下一个 5B */
            s_frame_pos = 1u;
        } else {
            s_frame_pos = 0u;
            s_frame_st  = PC_ST_HDR0;
        }
        break;

    case PC_ST_HEAD:                        /* CMD(2) SEQ(2) LEN(2) */
        s_frame_buf[s_frame_pos++] = b;
        if (s_frame_pos == 8u) {
            s_frame_dlen = (uint16_t)(((uint16_t)s_frame_buf[6] << 8) |
                                       (uint16_t)s_frame_buf[7]);
            if (s_frame_dlen > FRAME_MAX_DATA) {
                pc_frame_resync();          /* LEN 非法 → 立即重新对齐 */
                break;
            }
            s_frame_st = PC_ST_BODY;
        }
        break;

    case PC_ST_BODY:                        /* DATA(L) + CRC(2) */
        s_frame_buf[s_frame_pos++] = b;
        if (s_frame_pos == (uint16_t)(8u + s_frame_dlen + 2u)) {
            pc_frame_complete();
            s_frame_pos = 0u;
            s_frame_st  = PC_ST_HDR0;
        }
        break;

    default:
        s_frame_pos = 0u;
        s_frame_st  = PC_ST_HDR0;
        break;
    }
}

/*==============================================================================
 * 二、固件上传状态机（0x0009 / 0x000A / 0x000B）
 *----------------------------------------------------------------------------
 * 边缘处理与状态码一一对应：
 *   BUSY        : 非 IDLE 收到 START，或上传中收到 DATA/END 以外的命令
 *   ERR_SIZE    : LEN<9 / size=0 / size 越界 / 帧数=0
 *   ERR_FLASH   : SPI Flash 自检、擦除、写入或回读失败 → 锁存并中止
 *   ERR_OVERFLOW: 累计字节数超过声明大小 → 锁存
 *   ERR_COUNT   : END 时实际字节数 != 声明字节数 → 不写完整标志
 *   ERR_TIMEOUT : 10s 无新数据 → 主动回一帧 ERR_TIMEOUT
 *============================================================================*/
typedef enum {
    APP_STATE_INIT = 0,
    APP_STATE_IDLE,
    APP_STATE_UPLOADING
} app_state_t;

static app_state_t s_app_state = APP_STATE_INIT;

static uint32_t s_up_addr;                  /* 当前写入地址 (base+8 起)   */
static uint32_t s_up_total;                 /* 声明的文件大小 (字节)      */
static uint32_t s_up_count;                 /* 已写入字节数               */
static uint32_t s_up_start_tick;            /* 本次上传起始时刻           */
static uint32_t s_up_last_tick;             /* 最近一次收到 DATA 的时刻   */
static uint8_t  s_up_version[3];
static uint16_t s_up_page_idx;
static uint8_t  s_up_slot;                  /* 0=Bootloader, 1=App        */
static uint8_t  s_up_error;                 /* 0=正常, 非0=错误码 (锁存)  */

static uint8_t  s_page_buf[APP_PAGE_SIZE];

static uint32_t fw_slot_base(uint8_t slot)
{
    return (slot == 0u) ? FW_SLOT0_BASE : FW_SLOT1_BASE;
}

/** 攒页写入：满 256B 才提交一次页编程（跨页由 GD25Q_WritePage 处理） */
static uint8_t upload_data(const uint8_t *data, uint16_t len)
{
    uint16_t i;

    if ((s_up_count + (uint32_t)len) > s_up_total) {
        return APP_STATUS_ERR_OVERFLOW;
    }

    for (i = 0u; i < len; i++) {
        s_page_buf[s_up_page_idx] = data[i];
        s_up_page_idx++;
        s_up_count++;
        if (s_up_page_idx >= APP_PAGE_SIZE) {
            if (!GD25Q_WritePage(s_page_buf, s_up_addr, APP_PAGE_SIZE)) {
                return APP_STATUS_ERR_FLASH;
            }
            s_up_addr += APP_PAGE_SIZE;
            s_up_page_idx = 0u;
        }
    }
    return APP_STATUS_OK;
}

/** 写完整标志（0xA5）。NOR 页编程允许 1→0，所以擦除态 0xFF 可直接改成 0xA5 */
static uint8_t flash_write_complete_flag(uint8_t slot)
{
    uint8_t flag = FW_FLAG_COMPLETE;

    return GD25Q_WritePage(&flag, fw_slot_base(slot) + FW_HEAD_OFF_FLAG, 1u);
}

void FwUpload_Init(void)
{
    s_app_state     = APP_STATE_IDLE;
    s_up_addr       = 0u;
    s_up_total      = 0u;
    s_up_count      = 0u;
    s_up_start_tick = 0u;
    s_up_last_tick  = 0u;
    s_up_page_idx   = 0u;
    s_up_slot       = 0u;
    s_up_error      = 0u;
    memset(s_up_version, 0, sizeof(s_up_version));
    memset(s_page_buf, 0, sizeof(s_page_buf));
}

uint8_t FwUpload_CurrentSlot(void)
{
    return s_up_slot;
}

uint8_t FwUpload_IsBusy(void)
{
    return (s_app_state == APP_STATE_UPLOADING) ? 1u : 0u;
}

uint8_t FwUpload_Handle(uint16_t cmd, const uint8_t *data, uint16_t len)
{
    switch (cmd) {

    /*────────────────── 起始帧 (LEN=9) ──────────────────
     * [0..1] u16 总帧数 (大端)
     * [2..5] u32 文件大小 (大端)
     * [6..8] 3B  版本号 [主].[次].[修订]
     */
    case MGR_CMD_FW_START: {
        uint16_t frames;
        uint32_t bytes;
        uint32_t base;
        uint32_t sec_start;
        uint32_t sec_count;
        uint32_t s;
        uint32_t t0;
        uint8_t  hdr[FW_HEAD_SIZE];
        uint8_t  e;

        if (s_app_state != APP_STATE_IDLE) {
            e = APP_STATUS_BUSY;
            pc_send(cmd, &e, 1u);
            break;
        }
        if (len < 9u) {
            e = APP_STATUS_ERR_SIZE;
            LOGE("fw start: bad len %d", (int)len);
            pc_send(cmd, &e, 1u);
            break;
        }

        frames = (uint16_t)(((uint16_t)data[0] << 8) | (uint16_t)data[1]);
        bytes  = ((uint32_t)data[2] << 24) | ((uint32_t)data[3] << 16)
               | ((uint32_t)data[4] << 8)  |  (uint32_t)data[5];

        if ((bytes == 0u) || (bytes > FW_MAX_IMAGE_SIZE) || (frames == 0u)) {
            e = APP_STATUS_ERR_SIZE;
            LOGE("fw start: bad size %d / frames %d", (int)bytes, (int)frames);
            pc_send(cmd, &e, 1u);
            break;
        }

        /* 第一条命令就暴露"Flash 不在线/型号不对"，而不是等到擦除失败 */
        if (!GD25Q_Test()) {
            e = APP_STATUS_ERR_FLASH;
            LOGE("fw start: flash not responding");
            pc_send(cmd, &e, 1u);
            break;
        }

        base                = fw_slot_base(s_up_slot);
        s_up_addr           = base + FW_HEAD_SIZE;
        s_up_total          = bytes;
        s_up_count          = 0u;
        s_up_version[0]     = data[6];
        s_up_version[1]     = data[7];
        s_up_version[2]     = data[8];
        s_up_page_idx       = 0u;
        s_up_error          = 0u;
        s_up_start_tick     = HAL_GetTick();
        s_up_last_tick      = s_up_start_tick;
        memset(s_page_buf, 0, sizeof(s_page_buf));

        LOGI("fw start: slot=%d size=%d frames=%d ver=%d.%d.%d",
             (int)s_up_slot, (int)bytes, (int)frames,
             (int)s_up_version[0], (int)s_up_version[1], (int)s_up_version[2]);

        /* 解除块保护: BP 位保护会导致擦除/写入静默失败 */
        GD25Q_Unprotect();

        /* 擦除覆盖 头部 + 数据 的扇区 */
        sec_start = base / 4096u;
        sec_count = (bytes + FW_HEAD_SIZE + 4095u) / 4096u;
        t0        = HAL_GetTick();
        for (s = sec_start; s < (sec_start + sec_count); s++) {
            if (!GD25Q_EraseSector(s * 4096u)) {
                e = APP_STATUS_ERR_FLASH;
                LOGE("fw start: erase sector %d failed", (int)s);
                pc_send(cmd, &e, 1u);
                return 1u;
            }
        }
        LOGI("fw start: erased %d sectors in %d ms",
             (int)sec_count, (int)(HAL_GetTick() - t0));

        /* 写头部 7B: [4B 大小 小端][3B 版本号]，完整标志留到 END 再写 */
        hdr[0] = (uint8_t)(bytes);
        hdr[1] = (uint8_t)(bytes >> 8);
        hdr[2] = (uint8_t)(bytes >> 16);
        hdr[3] = (uint8_t)(bytes >> 24);
        hdr[4] = s_up_version[0];
        hdr[5] = s_up_version[1];
        hdr[6] = s_up_version[2];
        if (!GD25Q_WritePage(hdr, base, FW_HEAD_SIZE - 1u)) {
            e = APP_STATUS_ERR_FLASH;
            LOGE("fw start: header write failed");
            pc_send(cmd, &e, 1u);
            break;
        }

        s_app_state = APP_STATE_UPLOADING;
        e = APP_STATUS_OK;
        pc_send(cmd, &e, 1u);
        break;
    }

    /*────────────────── 数据帧 ──────────────────*/
    case MGR_CMD_FW_DATA: {
        uint8_t resp;

        if (s_app_state != APP_STATE_UPLOADING) {
            resp = APP_STATUS_BUSY;
            pc_send(cmd, &resp, 1u);
            break;
        }
        if (s_up_error != 0u) {
            /* 错误已锁存: 拒绝继续写入, 只回原错误码 */
            resp = s_up_error;
            pc_send(cmd, &resp, 1u);
            break;
        }

        s_up_last_tick = HAL_GetTick();
        resp = upload_data(data, len);
        if (resp != APP_STATUS_OK) {
            s_up_error = resp;
            LOGE("fw data: write failed rc=%d at %d/%d",
                 (int)resp, (int)s_up_count, (int)s_up_total);
        } else {
            LOGD("fw data: %d/%d", (int)s_up_count, (int)s_up_total);
        }
        pc_send(cmd, &resp, 1u);
        break;
    }

    /*────────────────── 结束帧 ──────────────────
     * 收尾: 1) 刷残页  2) 比对字节数  3) 写完整标志  4) 切槽
     */
    case MGR_CMD_FW_END: {
        uint8_t  resp = APP_STATUS_OK;
        uint8_t  cur_slot;
        uint32_t base;
        uint32_t elapsed;

        if (s_app_state != APP_STATE_UPLOADING) {
            resp = APP_STATUS_BUSY;
            pc_send(cmd, &resp, 1u);
            break;
        }

        cur_slot = s_up_slot;
        base     = fw_slot_base(cur_slot);

        if (s_up_error != 0u) {
            resp = s_up_error;
        } else if (s_up_page_idx > 0u) {
            if (!GD25Q_WritePage(s_page_buf, s_up_addr, s_up_page_idx)) {
                resp = APP_STATUS_ERR_FLASH;
            }
        }

        /* 比对声明字节数 —— 只有前面都成功时才判定 */
        if ((resp == APP_STATUS_OK) && (s_up_count != s_up_total)) {
            resp = APP_STATUS_ERR_COUNT;
        }

        /* 完整标志只在字节数匹配时写入，并回读确认真的变成 0xA5 */
        if (resp == APP_STATUS_OK) {
            uint8_t hdr[FW_HEAD_SIZE];

            if (!flash_write_complete_flag(cur_slot) ||
                !GD25Q_ReadBuffer(hdr, base, FW_HEAD_SIZE) ||
                (hdr[FW_HEAD_OFF_FLAG] != FW_FLAG_COMPLETE)) {
                resp = APP_STATUS_ERR_FLASH;
            }
        }

        elapsed = HAL_GetTick() - s_up_start_tick;

        /* 切槽, 为第二次上传 (App) 做准备 */
        s_up_slot   = (uint8_t)((s_up_slot == 0u) ? 1u : 0u);
        s_app_state = APP_STATE_IDLE;

        pc_send(cmd, &resp, 1u);

        if (resp == APP_STATUS_OK) {
            LOGI("fw end: slot=%d bytes=%d time=%d ms, next slot=%d",
                 (int)cur_slot, (int)s_up_count, (int)elapsed, (int)s_up_slot);
        } else {
            LOGE("fw end: slot=%d failed rc=%d (bytes %d/%d)",
                 (int)cur_slot, (int)resp, (int)s_up_count, (int)s_up_total);
        }
        break;
    }

    default:
        return 0u;      /* 非上传命令, 交给上层继续分发 */
    }

    return 1u;          /* 本命令已被上传模块处理 */
}

void FwUpload_TimeoutCheck(void)
{
    uint8_t e;

    if (s_app_state != APP_STATE_UPLOADING) {
        return;
    }
    if ((uint32_t)(HAL_GetTick() - s_up_last_tick) <= APP_UPLOAD_TIMEOUT_MS) {
        return;
    }

    e = (s_up_error != 0u) ? s_up_error : APP_STATUS_ERR_TIMEOUT;
    if (s_up_error == 0u) {
        s_up_error = APP_STATUS_ERR_TIMEOUT;
    }
    s_app_state = APP_STATE_IDLE;

    LOGW("fw upload timeout: %d/%d bytes", (int)s_up_count, (int)s_up_total);

    /* 错误码经 ACK 回传（SEQ 沿用最后一帧序号） */
    pc_send(MGR_CMD_FW_DATA, &e, 1u);
}

/*==============================================================================
 * 三、SWD 烧录胶水（0x0012 烧录 / 0x0013 自检 / 0x0014 进度）
 *----------------------------------------------------------------------------
 * 实现在 SWD_FLASH.c；这里只负责三件事：
 *   1) 烧录前的前置检查（两份镜像都在、都不超目标分区）——擦除之前拒绝
 *   2) 主循环里推进一步，结束时用"原命令 + 原序号"回一帧结果
 *   3) 烧录期间挡住其它命令（见 pc_dispatch 顶部）
 *============================================================================*/
static uint8_t  s_swd_active;      /* 1 = 烧录状态机正在跑 */
static uint16_t s_swd_seq;         /* 启动烧录那条命令的序号 */

/** 与 FOC_JIG 的 APP_ProgramPreCheck 同义：擦目标之前先确认两个槽都能烧 */
static uint8_t pc_swd_precheck(void)
{
    static const uint32_t max_size[2] = { DUT_BOOT_SIZE, DUT_APP_SIZE };
    uint8_t incomplete = 0u;
    uint8_t slot;

    for (slot = 0u; slot < 2u; slot++) {
        uint8_t  hdr[FW_HEAD_SIZE];
        uint32_t size;
        uint8_t  flag;

        if (!GD25Q_ReadBuffer(hdr, fw_slot_base(slot), FW_HEAD_SIZE)) {
            return APP_STATUS_ERR_FLASH;         /* SPI 总线读不到 = 真故障 */
        }
        size = (uint32_t)hdr[FW_HEAD_OFF_SIZE]
             | ((uint32_t)hdr[FW_HEAD_OFF_SIZE + 1u] << 8)
             | ((uint32_t)hdr[FW_HEAD_OFF_SIZE + 2u] << 16)
             | ((uint32_t)hdr[FW_HEAD_OFF_SIZE + 3u] << 24);
        flag = hdr[FW_HEAD_OFF_FLAG];

        if (size == 0u || size > FW_MAX_IMAGE_SIZE) {
            return APP_STATUS_ERR_NO_FW;
        }
        if (size > max_size[slot]) {
            return APP_STATUS_ERR_SIZE;          /* 超出目标分区：拒绝擦目标 */
        }
        if (flag != FW_FLAG_COMPLETE) {
            incomplete = 1u;
        }
    }
    return (incomplete != 0u) ? APP_STATUS_ERR_INCOMPLETE : APP_STATUS_OK;
}

/** 0x0014 进度查询：状态(1) + 结果(1) + 已处理(4) + 总量(4)，字段大端 */
static void pc_swd_status(uint16_t cmd)
{
    uint8_t  out[10];
    uint32_t off = DUT_FLASH_LastOffset();
    uint32_t tot = DUT_FLASH_TotalBytes();

    out[0] = DUT_FLASH_State();
    out[1] = DUT_FLASH_Result();
    out[2] = (uint8_t)(off >> 24);
    out[3] = (uint8_t)(off >> 16);
    out[4] = (uint8_t)(off >> 8);
    out[5] = (uint8_t)(off);
    out[6] = (uint8_t)(tot >> 24);
    out[7] = (uint8_t)(tot >> 16);
    out[8] = (uint8_t)(tot >> 8);
    out[9] = (uint8_t)(tot);
    pc_send(cmd, out, 10u);
}

/*==============================================================================
 * 四、命令分发
 *============================================================================*/
/** 0x000C 查询治具 Flash 里的固件版本：优先 App 槽，其次 Bootloader 槽 */
static uint8_t pc_query_flash_ver(uint8_t ver[3])
{
    int8_t  slot;
    uint8_t incomplete = 0u;

    for (slot = 1; slot >= 0; slot--) {
        uint8_t  hdr[FW_HEAD_SIZE];
        uint32_t size;
        uint8_t  flag;

        if (!GD25Q_ReadBuffer(hdr, fw_slot_base((uint8_t)slot), FW_HEAD_SIZE)) {
            return APP_STATUS_ERR_FLASH;        /* 总线读不到 = 真故障 */
        }

        size = (uint32_t)hdr[FW_HEAD_OFF_SIZE]
             | ((uint32_t)hdr[FW_HEAD_OFF_SIZE + 1u] << 8)
             | ((uint32_t)hdr[FW_HEAD_OFF_SIZE + 2u] << 16)
             | ((uint32_t)hdr[FW_HEAD_OFF_SIZE + 3u] << 24);
        flag = hdr[FW_HEAD_OFF_FLAG];

        if ((size == 0u) || (size > FW_MAX_IMAGE_SIZE)) {
            continue;                            /* 空槽 */
        }
        if (flag == FW_FLAG_COMPLETE) {
            ver[0] = hdr[FW_HEAD_OFF_VER];
            ver[1] = hdr[FW_HEAD_OFF_VER + 1u];
            ver[2] = hdr[FW_HEAD_OFF_VER + 2u];
            return APP_STATUS_OK;
        }
        incomplete = 1u;                         /* 有固件但不完整 */
    }

    ver[0] = 0x00u;
    ver[1] = 0xFFu;
    ver[2] = 0xFFu;
    return (incomplete != 0u) ? APP_STATUS_ERR_INCOMPLETE : APP_STATUS_ERR_NO_FW;
}

/*==============================================================================
 * 0x000D 查询 DUT 固件版本（UART5 发 M115）
 *----------------------------------------------------------------------------
 * DUT 的版本字符串有两种来源：
 *   1) M115 应答：        "version: v1.3.9\n"（LOG 宏，无前缀）
 *   2) 复位时 main 打印： "(I) version: v1.3.9\n"（LOGI，带 "(I) " 前缀）
 * 解析器只找 "version:"，后面允许空格和可选的 'v'，取 X.Y.Z。
 *============================================================================*/
#if FLASH_MODE_UART5
typedef struct {
    uint8_t  state;      /* 0=搜索 "version:"，1=冒号后，2=读数字 */
    uint8_t  key_idx;
    uint8_t  fields;
    uint16_t cur;
    uint8_t  ver[3];     /* major, minor, rev */
} dut_ver_parser_t;

static const char s_dut_ver_key[] = "version:";

static void dut_ver_parser_reset(dut_ver_parser_t *p)
{
    memset(p, 0, sizeof(*p));
}

static void dut_ver_parser_store(dut_ver_parser_t *p)
{
    uint8_t v = (p->cur > 255u) ? 255u : (uint8_t)p->cur;

    if (p->fields < 3u) {
        p->ver[p->fields] = v;
    }
    p->fields++;
    p->cur = 0u;
}

/* @retval 1 = 已解析到版本号（缺的字段保持 0） */
static uint8_t dut_ver_parser_feed(dut_ver_parser_t *p, uint8_t b)
{
    if (p->state == 0u) {
        if (b == (uint8_t)s_dut_ver_key[p->key_idx]) {
            p->key_idx++;
            if (s_dut_ver_key[p->key_idx] == '\0') {
                p->state = 1u;
                p->key_idx = 0u;
            }
        } else {
            p->key_idx = (b == (uint8_t)s_dut_ver_key[0]) ? 1u : 0u;
        }
        return 0u;
    }

    if (p->state == 1u) {
        if ((b == ' ') || (b == '\t')) {
            return 0u;
        }
        if ((b == 'v') || (b == 'V')) {
            p->state = 2u;
            p->cur = 0u;
            return 0u;
        }
        if ((b >= '0') && (b <= '9')) {
            p->state = 2u;
            p->cur = (uint16_t)(b - '0');
            return 0u;
        }
        /* 冒号后不是版本号：回到搜索状态 */
        p->state = 0u;
        p->key_idx = (b == (uint8_t)s_dut_ver_key[0]) ? 1u : 0u;
        return 0u;
    }

    /* state 2：数字 */
    if ((b >= '0') && (b <= '9')) {
        p->cur = (uint16_t)(p->cur * 10u + (uint16_t)(b - '0'));
        if (p->cur > 999u) {
            p->cur = 999u;
        }
        return 0u;
    }
    if (b == '.') {
        dut_ver_parser_store(p);
        return (p->fields >= 3u) ? 1u : 0u;
    }

    /* 非数字/点：版本结束（换行、回车、空格等） */
    dut_ver_parser_store(p);
    return (p->fields > 0u) ? 1u : 0u;
}

/* 窗口结束时收尾最后一个数字（正常有换行时会提前返回） */
static uint8_t dut_ver_parser_finish(dut_ver_parser_t *p)
{
    if (p->state == 2u) {
        dut_ver_parser_store(p);
    }
    return (p->fields > 0u) ? 1u : 0u;
}

static void dut_ver_send_m115(void)
{
    static const uint8_t m115[] = { 'M', '1', '1', '5', '\n' };
    (void)UART5_Transmit(m115, (uint16_t)sizeof(m115), 100u);
}

/* 0x000D 查询 DUT 版本：
 *   1) 立刻发 M115，3s 窗口（M115 应答 / DUT 复位日志都接收）；
 *   2) 3s 没收到，再发一次 M115，50ms 窗口；
 *   3) 仍没有则版本 0.0.0（状态仍回 OK，方便 PC 区分“查了但没连上”）。 */
static uint8_t pc_query_dut_ver(uint8_t ver[3])
{
    dut_ver_parser_t p;
    uint32_t start;
    uint8_t b;
    uint8_t found = 0u;

    ver[0] = 0u;
    ver[1] = 0u;
    ver[2] = 0u;

    /* 确保 UART5 在默认 8N1 256000；配置已匹配时不会清环形缓冲 */
    UART5_SetConfig(UARTF_TEST_BAUD, UART_PARITY_NONE);
    UART5_SetMode(UART5_MODE_DEFAULT);

    dut_ver_parser_reset(&p);

    /* 立刻发第一条 M115 */
    dut_ver_send_m115();

    /* 3 秒窗口：M115 应答和 DUT 复位日志都收 */
    start = HAL_GetTick();
    while ((uint32_t)(HAL_GetTick() - start) < 3000u) {
        while (UART5_ReadByte(&b)) {
            if (dut_ver_parser_feed(&p, b) != 0u) {
                found = 1u;
                break;
            }
        }
        if (found != 0u) {
            break;
        }
    }
    if ((found == 0u) && (dut_ver_parser_finish(&p) != 0u)) {
        found = 1u;
    }

    /* 3 秒内没收到：再发一条，50ms 超时 */
    if (found == 0u) {
        dut_ver_parser_reset(&p);
        dut_ver_send_m115();

        start = HAL_GetTick();
        while ((uint32_t)(HAL_GetTick() - start) < 50u) {
            while (UART5_ReadByte(&b)) {
                if (dut_ver_parser_feed(&p, b) != 0u) {
                    found = 1u;
                    break;
                }
            }
            if (found != 0u) {
                break;
            }
        }
        if ((found == 0u) && (dut_ver_parser_finish(&p) != 0u)) {
            found = 1u;
        }
    }

    if (found != 0u) {
        ver[0] = p.ver[0];
        ver[1] = p.ver[1];
        ver[2] = p.ver[2];
        LOGI("dut version: %d.%d.%d", (int)ver[0], (int)ver[1], (int)ver[2]);
    } else {
        LOGW("dut version: no response, use 0.0.0");
    }

    return APP_STATUS_OK;
}
#endif /* FLASH_MODE_UART5 */

uint8_t g_u5_copy_to_log = 0u;   /* 0x0098 控制：默认模式 UART5→USART1 原样复制 */

/*==============================================================================
 * UART5 默认模式的数据分发
 *----------------------------------------------------------------------------
 * 烧录模式下 UART5 收到的字节由 UART_FLASH.c 的 bootloader 协议消费；
 * 默认模式（上电/烧录结束/0x0099 之后）由这里处理：
 *   - g_u5_copy_to_log != 0（0x0098）：原样复制一份到 USART1 TX；
 *   - s_u5_bridge_active != 0（0x0099）：原样送到 USB CDC；
 *   - 两个都不开时也要读空环形缓冲，避免 DUT App 数据把缓冲灌满。
 *============================================================================*/
#if FLASH_MODE_UART5
static uint8_t s_u5_bridge_active;

static void uart5_default_poll(void)
{
    /* USART1 的 DMA 发送是异步的，用双缓冲保证上一包还在发时不会覆盖。
     * 每次启动新 DMA 前，M_UART1_Transmit_AndWait() 会等上一包发完。 */
    static uint8_t copy_buf[2][64];
    static uint8_t copy_idx = 0u;
    static uint8_t cdc_buf[64];

    uint8_t *cb = copy_buf[copy_idx];
    uint8_t cn = 0u;
    uint8_t dn = 0u;
    uint8_t b;

    while (UART5_ReadByte(&b)) {
        if (g_u5_copy_to_log != 0u) {
            cb[cn++] = b;
            if (cn >= (uint8_t)sizeof(copy_buf[0])) {
                M_UART1_Transmit_AndWait(cb, cn);
                copy_idx ^= 1u;
                cb = copy_buf[copy_idx];
                cn = 0u;
            }
        }
        if (s_u5_bridge_active != 0u) {
            cdc_buf[dn++] = b;
            if (dn >= (uint8_t)sizeof(cdc_buf)) {
                (void)CDC_Transport_Send(cdc_buf, dn, true);
                dn = 0u;
            }
        }
    }

    if (cn > 0u) {
        M_UART1_Transmit_AndWait(cb, cn);
        copy_idx ^= 1u;
    }
    if (dn > 0u) {
        (void)CDC_Transport_Send(cdc_buf, dn, true);
    }
}

/* 进入透传：先切好默认模式/波特率，再复位 DUT，启动日志不会丢 */
static void uart5_bridge_begin(void)
{
    UART5_SetBaud(UARTF_TEST_BAUD);          /* 8N1 256000 */
    UART5_SetMode(UART5_MODE_DEFAULT);
    s_u5_bridge_active = 1u;
    UART5_FlushRx();
    UARTF_RunApp();

    LOGI("uart5 bridge: CDC <-> UART5 @%d, until fixture reset",
         (int)UARTF_TEST_BAUD);
}
#endif /* FLASH_MODE_UART5 */

static void pc_dispatch(uint16_t cmd, const uint8_t *data, uint16_t len)
{
    /* SWD 烧录进行中：先挡住所有命令 —— 必须放在 FwUpload_Handle 之前，
     * 否则上传命令会在烧录读 SPI 镜像的同时改写同一片 Flash。
     * 只放行进度查询，方便 PC 显示"烧到哪了"。 */
    if (s_swd_active) {
        if (cmd == MGR_CMD_SWD_STATUS) {
            pc_swd_status(cmd);
        } else {
            uint8_t e = APP_STATUS_BUSY;
            pc_send(cmd, &e, 1u);
        }
        return;
    }

    /* 上传三段式命令由 fw_upload 独占处理（内部自带 BUSY/参数校验） */
    if (FwUpload_Handle(cmd, data, len)) {
        return;
    }

    /* 非 IDLE 状态下，其余命令一律回 BUSY */
    if (FwUpload_IsBusy()) {
        uint8_t e = APP_STATUS_BUSY;
        pc_send(cmd, &e, 1u);
        return;
    }

    switch (cmd) {

    /*── 0x000C 查治具 Flash 里的固件版本: 应答 状态(1)+版本(3) ──*/
    case MGR_CMD_QUERY_FLASH_VER: {
        uint8_t out[4];
        out[0] = pc_query_flash_ver(&out[1]);
        pc_send(cmd, out, 4u);
        break;
    }

    /*── 0x000D 查 DUT 固件版本：UART5 发 M115，3s + 50ms 两次窗口。
     *   应答 状态(1)+版本(3)；连不上回 0.0.0。
     *   注意：本命令最长会阻塞约 3.05 秒，PC 端超时要大于这个值。 ──*/
    case MGR_CMD_QUERY_DUT_VER: {
        uint8_t out[4];
#if FLASH_MODE_UART5
        out[0] = pc_query_dut_ver(&out[1]);
#else
        out[0] = APP_STATUS_ERR_NIY;
        out[1] = 0u;
        out[2] = 0u;
        out[3] = 0u;
#endif
        pc_send(cmd, out, 4u);
        break;
    }

    /*── 0x0013 自检（只读，不擦不写）──
     * SWD 模式：线复位 + 读 DEV_ID/容量/OBR/两份栈顶/AP/DP，应答 35B。
     * UART 模式：BOOT0 高 + 复位进 bootloader，0x7F 同步后读
     *            bootloader 版本 / PID / BootSP / AppSP，然后复位运行 App。
     * 两种模式都保持 35B 应答长度，前 17B 布局一致：
     *   [0]状态 [1]连接错误 [2]RDP [3..4]延时(UART=0) [5..6]DEV_ID/PID
     *   [7..8]FlashKB [9..12]IDCODE(UART=0) [13..16]AppSP
     *   [17]首个失败码 … 其余字段 UART 模式填 0。 */
#if FLASH_MODE_UART5
    case MGR_CMD_SWD_PROBE: {
        uartf_probe_t p;
        uint8_t       out[35];

        memset(out, 0, sizeof(out));
        out[0]  = UARTF_Probe(&p);
        out[1]  = p.uart_err;
        out[2]  = p.rdp;
        out[5]  = (uint8_t)(p.pid >> 8);
        out[6]  = (uint8_t)(p.pid);
        out[7]  = 0u;
        out[8]  = 128u;                 /* STM32F105RB = 128KB */
        out[13] = (uint8_t)(p.app_sp >> 24);
        out[14] = (uint8_t)(p.app_sp >> 16);
        out[15] = (uint8_t)(p.app_sp >> 8);
        out[16] = (uint8_t)(p.app_sp);
        pc_send(cmd, out, 35u);
        break;
    }
#else
    case MGR_CMD_SWD_PROBE: {
        swd_probe_t p;
        uint8_t     out[35];

        if (len >= 2u) {
            SWDP_SetDelay((uint16_t)(((uint16_t)data[0] << 8) | (uint16_t)data[1]));
        }
        out[0]  = SWDP_Probe(&p);
        out[1]  = p.swd_err;
        out[2]  = p.rdprt;
        out[3]  = (uint8_t)(p.delay >> 8);
        out[4]  = (uint8_t)(p.delay);
        out[5]  = (uint8_t)(p.dev_id >> 8);
        out[6]  = (uint8_t)(p.dev_id);
        out[7]  = (uint8_t)(p.flash_kb >> 8);
        out[8]  = (uint8_t)(p.flash_kb);
        out[9]  = (uint8_t)(p.idcode >> 24);
        out[10] = (uint8_t)(p.idcode >> 16);
        out[11] = (uint8_t)(p.idcode >> 8);
        out[12] = (uint8_t)(p.idcode);
        out[13] = (uint8_t)(p.app_sp >> 24);
        out[14] = (uint8_t)(p.app_sp >> 16);
        out[15] = (uint8_t)(p.app_sp >> 8);
        out[16] = (uint8_t)(p.app_sp);
        out[17] = p.mem_err;
        out[18] = (uint8_t)(p.ap_idr >> 24);
        out[19] = (uint8_t)(p.ap_idr >> 16);
        out[20] = (uint8_t)(p.ap_idr >> 8);
        out[21] = (uint8_t)(p.ap_idr);
        out[22] = (uint8_t)(p.dp_stat >> 24);
        out[23] = (uint8_t)(p.dp_stat >> 16);
        out[24] = (uint8_t)(p.dp_stat >> 8);
        out[25] = (uint8_t)(p.dp_stat);
        out[26] = p.err_dev;
        out[27] = p.err_kb;
        out[28] = p.err_obr;
        out[29] = p.err_boot;
        out[30] = p.err_app;
        out[31] = p.err_idr;
        out[32] = p.err_dpstat;
        out[33] = p.err_ram;
        out[34] = p.ram_ok;
        pc_send(cmd, out, 35u);
        break;
    }
#endif

    /*── 0x0014 烧录进度：空闲时也可查，返回上一次的终态 ──*/
    case MGR_CMD_SWD_STATUS:
        pc_swd_status(cmd);
        break;

    /*── 0x0015 3 路电压检测：状态(1) + 3×[U1 mV(2) U2 mV(2) flags(1)] ──*/
    case MGR_CMD_VOLT_TEST: {
        fx_volt_result_t r[FX_VOLT_QTY];
        uint8_t out[16];
        uint8_t st;
        uint8_t i;

        st = fx_adc_voltage_test(r);
        out[0] = st;
        for (i = 0u; i < (uint8_t)FX_VOLT_QTY; i++) {
            uint8_t o = (uint8_t)(1u + i * 5u);

            out[o + 0u] = (uint8_t)(r[i].u1_mv >> 8);
            out[o + 1u] = (uint8_t)(r[i].u1_mv);
            out[o + 2u] = (uint8_t)(r[i].u2_mv >> 8);
            out[o + 3u] = (uint8_t)(r[i].u2_mv);
            out[o + 4u] = r[i].flags;
        }
        /* 真实值/假数据的日志在 fx_adc_voltage_test() 里输出，
         * 这里只负责把最终 ACK 数据打包。 */
        pc_send(cmd, out, 16u);
        break;
    }

    /*── 0x0016 3 路 ADC 检测：状态(1) + 3×[U1 mV(2) U2 mV(2) DUT mV(2)] ──*/
    case MGR_CMD_ADC_TEST: {
        fx_adc3_result_t r[FX_ADC3_QTY];
        uint8_t out[19];
        uint8_t st;
        uint8_t i;
        /* DUT 触点经 1k/3k 分压后才是治具节点电压。
         * ACK 里 DUT 字段保持原始读数，U1/U2 按 ×4/3 折算到 DUT 侧，
         * 这样 PC 端三个值可以直接比较；原始 U1/U2 只留在日志里。 */
        uint16_t u1_dut[FX_ADC3_QTY];
        uint16_t u2_dut[FX_ADC3_QTY];
        uint16_t exp_node[FX_ADC3_QTY];

        st = fx_adc_3ch_test(r);

        for (i = 0u; i < 2u; i++) {
            u1_dut[i] = (uint16_t)((uint32_t)r[i].u1_mv * 4u / 3u);
            u2_dut[i] = (uint16_t)((uint32_t)r[i].u2_mv * 4u / 3u);
            exp_node[i] = (uint16_t)((uint32_t)r[i].dut_mv * 3u / 4u);
        }
        u1_dut[FX_ADC3_MER]  = 0u;
        u2_dut[FX_ADC3_MER]  = 0u;
        exp_node[FX_ADC3_MER] = 0u;

        out[0] = st;
        for (i = 0u; i < (uint8_t)FX_ADC3_QTY; i++) {
            uint8_t o = (uint8_t)(1u + i * 6u);

            out[o + 0u] = (uint8_t)(u1_dut[i] >> 8);
            out[o + 1u] = (uint8_t)(u1_dut[i]);
            out[o + 2u] = (uint8_t)(u2_dut[i] >> 8);
            out[o + 3u] = (uint8_t)(u2_dut[i]);
            out[o + 4u] = (uint8_t)(r[i].dut_mv >> 8);
            out[o + 5u] = (uint8_t)(r[i].dut_mv);
        }

        /* 原始值日志：U1/U2 是治具节点电压，DUT 是 DUT 自己读到的电压 */
        LOGI("adc3: nht %d/%d/%d exp %d, ntc %d/%d/%d exp %d, "
             "mer mv %d/%d/%d lv %d/%d, st=%d",
             (int)r[0].u1_mv, (int)r[0].u2_mv, (int)r[0].dut_mv, (int)exp_node[0],
             (int)r[1].u1_mv, (int)r[1].u2_mv, (int)r[1].dut_mv, (int)exp_node[1],
             (int)r[2].u1_mv, (int)r[2].u2_mv, (int)r[2].dut_mv,
             (int)r[2].mer_u1_lv, (int)r[2].mer_u2_lv,
             (int)st);

        /* 转换后日志：和 ACK 内容一致，U1/U2 已折算到 DUT 侧，DUT 为原始值 */
        LOGI("adc3 conv: nht %d/%d/%d, ntc %d/%d/%d, "
             "mer mv %d/%d/%d lv %d/%d, st=%d",
             (int)u1_dut[0], (int)u2_dut[0], (int)r[0].dut_mv,
             (int)u1_dut[1], (int)u2_dut[1], (int)r[1].dut_mv,
             (int)r[2].u1_mv, (int)r[2].u2_mv, (int)r[2].dut_mv,
             (int)r[2].mer_u1_lv, (int)r[2].mer_u2_lv,
             (int)st);

        pc_send(cmd, out, 19u);
        break;
    }

    /*── 0x0017 IO 输入检测：状态(1) + 8×flags(1) ──*/
    case MGR_CMD_IOIN_TEST: {
        uint8_t flags[FX_IN_QTY];
        uint8_t out[9];

        out[0] = fx_input_test(flags);
        memcpy(&out[1], flags, FX_IN_QTY);
        LOGI("ioin: %d %d %d %d %d %d %d %d, st=%d",
             (int)flags[0], (int)flags[1], (int)flags[2], (int)flags[3],
             (int)flags[4], (int)flags[5], (int)flags[6], (int)flags[7],
             (int)out[0]);
        pc_send(cmd, out, 9u);
        break;
    }

    /*── 0x0012 烧录（SWD 或 UART5，由 FLASH_MODE_UART5 选择）：
     *   ACK 要等烧录完成才回，这里只启动状态机 ──*/
    case MGR_CMD_SWD_PROGRAM: {
        uint8_t pre = pc_swd_precheck();

        if (pre != APP_STATUS_OK) {
            uint8_t e = pre;
            pc_send(cmd, &e, 1u);
            break;
        }
        s_swd_seq    = s_pc_seq;
        s_swd_active = 1u;
        DUT_FLASH_Start();
        break;
    }

    /*── 0x0098 默认模式 UART5→USART1 原样复制开关 ──
     *  LEN=0：翻转；LEN>=1：DATA[0]=0 关，非 0 开。
     *  应答 [状态, 当前开关值]。*/
    case MGR_CMD_UART5_COPY: {
#if FLASH_MODE_UART5
        uint8_t out[2];

        if (len >= 1u) {
            g_u5_copy_to_log = (data[0] != 0u) ? 1u : 0u;
        } else {
            g_u5_copy_to_log = (g_u5_copy_to_log == 0u) ? 1u : 0u;
        }
        out[0] = APP_STATUS_OK;
        out[1] = g_u5_copy_to_log;
        pc_send(cmd, out, 2u);
        LOGI("uart5 copy to USART1: %s", (g_u5_copy_to_log != 0u) ? "on" : "off");
#else
        uint8_t e = APP_STATUS_ERR_NIY;
        pc_send(cmd, &e, 1u);
#endif
        break;
    }

    /*── 0x0099 UART5 透传：先回 OK，再切模式并复位 DUT ──*/
    case MGR_CMD_UART5_TEST: {
#if FLASH_MODE_UART5
        uint8_t e = APP_STATUS_OK;
        pc_send(cmd, &e, 1u);       /* 最后一帧带协议头的应答，之后就透传 */
        uart5_bridge_begin();
#else
        uint8_t e = APP_STATUS_ERR_NIY;   /* SWD 模式已释放 UART5，不能透传 */
        pc_send(cmd, &e, 1u);
#endif
        break;
    }

    default: {
        uint8_t e = APP_STATUS_ERR_NIY;
        pc_send(cmd, &e, 1u);
        break;
    }
    }
}

/*==============================================================================
 * 五、对外入口
 *============================================================================*/
void pc_app_init(void)
{
    FwUpload_Init();
    DUT_FLASH_Init();        /* PC6=NRST / PC7=BOOT0；UART 默认 8N1 256000 */

    g_u5_copy_to_log = 0u;   /* 0x0098 默认关闭 */

    s_pc_seq     = 0u;
    s_frame_pos  = 0u;
    s_frame_dlen = 0u;
    s_frame_st   = PC_ST_HDR0;

    s_swd_active = 0u;
    s_swd_seq    = 0u;
#if FLASH_MODE_UART5
    s_u5_bridge_active = 0u;
#endif

    LOGI("pc link ready: A5 5B/CRC16, fw slot=%d, dut app=%x",
         (int)FwUpload_CurrentSlot(), (int)DUT_APP_ADDR);
#if FLASH_MODE_UART5
    LOGI("flash mode: UART5 -> DUT USART1 bootloader, baud=%d chunk=%d",
         (int)UARTF_BAUD, (int)UARTF_CHUNK_BYTES);
    LOGI("uart5 default: 8N1 @%d; flashing: 8E1 @%d",
         (int)UARTF_TEST_BAUD, (int)UARTF_BAUD);
#else
    LOGI("flash mode: SWD bit-bang, delay=%d", (int)SWDP_GetDelay());
#endif
    LOGI("uart log: tx cache=256, rx echo off");
}

void pc_app_poll(void)
{
#if FLASH_MODE_UART5
    /* 只有默认模式才由这里消费 UART5 RX；烧录模式的字节归 bootloader 协议 */
    if (UART5_GetMode() == UART5_MODE_DEFAULT) {
        uart5_default_poll();    /* 0x0099→CDC，0x0098→USART1 */
    }
    if (s_u5_bridge_active) {
        return;                  /* 透传模式不再处理上传/烧录命令 */
    }
#endif

    FwUpload_TimeoutCheck();

    /* 烧录推进 + 终态回 ACK（不变量：只回一次）。
     * SWD：一步 10~40ms（擦一页 / 写 128B）。
     * UART5：一步可能是一整片擦除（约 100ms）或一包 256B（约 25ms）。
     * 期间 USB 中断仍在跑，但主循环的 CDC_Transport_Poll 会被推迟 ——
     * 协议上 PC 必须等 ACK，所以这段窗口里不会有多余下行数据。 */
    if (s_swd_active) {
        DUT_FLASH_Step();
        if (!DUT_FLASH_Busy()) {
            uint8_t st  = DUT_FLASH_Result();
            uint8_t res[3];

            s_swd_active = 0u;
            /* 应答 3B：状态 + 原始 SWD 错误码 + 失败时停在哪一步（1=连接 2=停核
             * 3=擦除 4=写 5=校验 6=复位 8=FAIL）。成功时后两字节为 0。 */
            res[0] = st;
            res[1] = DUT_FLASH_LastError();
            res[2] = DUT_FLASH_FailState();
            #if SWD_PROGRAM_ACK_3BYTES
            pc_send_seq(MGR_CMD_SWD_PROGRAM, s_swd_seq, res, 3u);
            #else
            pc_send_seq(MGR_CMD_SWD_PROGRAM, s_swd_seq, res, 1u);
            #endif

            /* 长烧录期间 PC 若仍在发帧，接收缓冲可能被填满而丢字节，
             * 留下半截帧。这里直接清空解析器，让下一条命令从干净状态开始。 */
            s_frame_pos  = 0u;
            s_frame_dlen = 0u;
            s_frame_st   = PC_ST_HDR0;
        }
    }
}

/**
  * @brief  USB CDC 整包/短包到达（主循环上下文，由 CDC_Transport_Poll 调起）
  * @note   cdc_transport 只负责把 OUT 包拼成"一次 USB 传输"，这里再按 A5 5B
  *         字节流解析 —— 这样 PC 把多帧写在一起、或一帧被拆成多次传输都能兼容。
  */
void CDC_FrameHandler(const uint8_t *data, uint16_t len)
{
    uint16_t i;

    if (data == 0) {
        return;
    }

#if FLASH_MODE_UART5
    for (i = 0u; i < len; i++) {
        if (s_u5_bridge_active) {
            /* 透传：把本包剩余字节原样发给 DUT。
             * 逐字节检查是为了 0x0099 和后续数据挤在同一个 USB 包里时，
             * 命令解析完之后的字节也能直接进透传。 */
            (void)UART5_Transmit(&data[i], (uint16_t)(len - i), 1000u);
            return;
        }
        pc_frame_feed(data[i]);
    }
#else
    for (i = 0u; i < len; i++) {
        pc_frame_feed(data[i]);
    }
#endif
}

/** USART1 只作为日志输出口（PC 命令走 USB CDC）。
 *  不再回显收到的字节：PA10 上的噪声/误码会被原样插进日志流，造成乱码。
 *  队列仍然每 1ms 清空一次，防止噪声把 RX 队列填满。 */
void uart_rx_handle(void)
{
    uint8_t *buf;
    uint8_t  len = uart_rx_queue_get_head(&gUart1RxQueue, &buf);

    (void)buf;
    if (len > 0) {
        uart_rx_queue_out(&gUart1RxQueue);
    }
}
