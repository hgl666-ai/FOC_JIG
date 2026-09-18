/**
  ******************************************************************************
  * @file    app.c
  * @brief   治具主控 U1 应用层 — 命令分发 / ACK / 主循环
  *
  * 实现协议 (INFO/M1 测试治具PC端通讯协议.xlsx) 全部 6 条命令:
  *   0x0009 固件起始帧          → fw_upload.c
  *   0x000A 固件数据帧          → fw_upload.c
  *   0x000B 固件结束帧          → fw_upload.c
  *   0x000C 查询固件版本(Flash) → 本文件
  *   0x000D 查询固件版本(FOC板) → isp_programmer.c
  *   0x0112 烧录Flash固件到FOC板 → isp_programmer.c
  * 其余命令统一回 NIY(0xFF)。
  ******************************************************************************
  */
#include "app.h"
#include "app_frame.h"
#include "protocol_frame.h"
#include "flash_map.h"
#include "fw_upload.h"
#include "gd25q128.h"
#include "isp_programmer.h"
#include "isp_config.h"
#include "bsp_uart.h"
#include "usbd_cdc_if.h"
#include "usb_device.h"

/*==============================================================================
 * 全局状态
 *============================================================================*/
APP_State_t app_state = APP_STATE_INIT;
uint16_t    pc_seq    = 0u;

static uint16_t prog_seq = 0u;      /* 0x0112 受理时的帧序号, 烧完 ACK 回显用 */
static uint8_t  tx_buf[APP_FRAME_BUF_SIZE];

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
    uint16_t total = Frame_Build(tx_buf, cmd, seq, data, len);
    uint32_t tick  = HAL_GetTick();

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
        resp[0] = ISP_QueryTargetVersion(&resp[1]);
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
        ISP_Start();
        break;
    }

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
 * 对外 API
 *============================================================================*/
void APP_Init(void)
{
    app_state = APP_STATE_INIT;
    pc_seq    = 0u;
    prog_seq  = 0u;

    APP_Frame_Init();
    FwUpload_Init();

    ISP_GpioInit();
    BSP_ISP_UART_Init();

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

    /* 3) ISP 烧录推进 + 终态回 ACK (不变量 I4: 只回一次) */
    if (app_state == APP_STATE_PROGRAMMING) {
        ISP_Step();
        if (!ISP_Busy()) {
            uint8_t st = ISP_Result();
            app_state = APP_STATE_IDLE;
            APP_SendFrame(MGR_CMD_PROGRAM, prog_seq, &st, 1u);
            /* ISP_Step 内可能阻塞十余秒 (等全片擦除 ACK), 期间 PC 若仍在发帧,
             * 512B 接收环形缓冲可能被填满而丢弃字节, 留下半截帧。
             * 协议规定 PC 必须等 0x0112 的 ACK 才继续, 因此这里直接清空
             * 接收侧, 保证下一条命令从干净的解析状态开始 (见并发分析: 序号 10)。*/
            APP_Frame_Init();
        }
    }
}

/* USB CDC 收到一批字节 (中断上下文) — 只喂状态机, 不做重活 */
void APP_USB_Receive(uint8_t *data, uint32_t len)
{
    for (uint32_t i = 0; i < len; i++) {
        APP_Frame_Feed(data[i]);
    }
}
